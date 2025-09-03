#!/usr/bin/env python3

import time
import functools
import argparse

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


from tbai_safe.systems import SimpleSingleIntegrator2D
from tbai_safe.cbf import ControlBarrierFunctionFactory, visualize_cbfs
from tbai_safe.symperf import jit_expr
from tbai_safe.mppi import (
  AcceleratedSafetyMPPI,
  get_cost_function_parameterized,
  MppiCbfCost,
  MppiCbfCostInputs,
  cost_fn,
)
from tbai_safe.mppi import set_default_dtype, set_default_backend
from tbai_safe.anim import save_animation


def main(show_animation=True):
  # Initial and final states
  x_initial = np.array([-2.0, -3.4])
  x_desired = np.array([0.0, 0.0])
  r = 3.04
  current_time = 0

  set_default_backend("cuda")
  set_default_dtype("float64")

  # Soft CBFs
  factory2 = ControlBarrierFunctionFactory()
  d = 1.5
  cbf10 = factory2.get_sphere().substitute(c_x=0, c_y=-d, r=1.3)
  cbf11 = factory2.get_sphere().substitute(c_x=-d, c_y=0, r=1.3)
  cbf12 = factory2.get_sphere().substitute(c_x=d, c_y=0, r=1.3)
  cbf13 = factory2.get_sphere().substitute(c_x=0, c_y=d, r=1.3)

  # Visualize CBFs
  fig, ax = plt.subplots()
  # add minor grid
  ax.minorticks_on()
  ax.grid(which="both", alpha=0.2)
  ax.set_aspect("equal")
  ax.set_xlim(-4, 4)
  ax.set_ylim(-4, 4)
  colors = ["blue", "red", "red", "red"]
  weights = [1.0, 5000.0, 5000.0, 5000.0]
  pcm = visualize_cbfs([cbf10, cbf11, cbf12, cbf13], ax, granularity=200, unsafe_colors=colors, alpha=0.9)

  # Visualize initial and desired states
  ax.plot(x_initial[0], x_initial[1], "rx", label="Initial state", markersize=10)
  (desired_plot,) = ax.plot(x_desired[0], x_desired[1], "bx", label="Desired state", markersize=10)
  ax.legend()

  # Initialize system
  system = SimpleSingleIntegrator2D().reset(x_initial, visualizer=(fig, ax), visualize_history=True)
  system.visualize()
  dt = 0.02

  # Get LQR controller
  Q, R = np.eye(2), np.eye(2)

  # Prepare stage cost
  x1, x2, u1, u2 = factory2.x, factory2.y, factory2.u1, factory2.u2
  import sympy as sp

  x_desireds, y_desireds = sp.symbols("x_desired, y_desired")
  lqr_stage_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desireds, y_desireds, 0.0, 0.0)
  lqr_stage_jit = jit_expr(lqr_stage_cost_expr)
  cbf10_jit = jit_expr(cbf10.get_expr(substitute=True))
  cbf11_jit = jit_expr(cbf11.get_expr(substitute=True))
  cbf12_jit = jit_expr(cbf12.get_expr(substitute=True))
  cbf13_jit = jit_expr(cbf13.get_expr(substitute=True))

  @functools.partial(cost_fn, locals=locals(), globals=globals())
  def stage_cost(x, y, u1, u2, weight1, weight2, weight3, weight4, alpha, x_desired, y_desired):
    cbf10_val = cbf10_jit(x=x, y=y)
    cbf11_val = cbf11_jit(x=x, y=y)
    cbf12_val = cbf12_jit(x=x, y=y)
    cbf13_val = cbf13_jit(x=x, y=y)
    cost = 0.0
    cost += (-weight1 * cbf10_val) if cbf10_val < 0 else 0
    cost += (-weight2 * cbf11_val) if cbf11_val < 0 else 0
    cost += (-weight3 * cbf12_val) if cbf12_val < 0 else 0
    cost += (-weight4 * cbf13_val) if cbf13_val < 0 else 0
    abs_cost = -cost if cost < 0 else cost
    if abs_cost == 0:
      abs_cost = 1.0
    abs_cost = max(abs_cost, 1.0)
    cost += (alpha * lqr_stage_jit(x=x, y=y, u1=0.0, u2=0.0, x_desired=x_desired, y_desired=y_desired)) / abs_cost
    return cost

  lqr_final_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desireds, y_desireds, 0.0, 0.0)
  lqr_final_jit = jit_expr(lqr_final_cost_expr)

  @functools.partial(cost_fn, locals=locals(), globals=globals())
  def terminal_cost(x, y, weight1, weight2, weight3, weight4, alpha, x_desired, y_desired):
    return lqr_final_jit(x=x, y=y, u1=0.0, u2=0.0, x_desired=x_desired, y_desired=y_desired)

  mppi_cost_fn = get_cost_function_parameterized(
    stage_cost,
    terminal_cost,
    scalar_args=["weight1", "weight2", "weight3", "weight4"],
    vector_args_ew=["alpha", "x_desired", "y_desired"],
    vector_args_vw=[],
  )

  cost = MppiCbfCost(mppi_cost_fn)

  print("Jitted!")

  mppi = AcceleratedSafetyMPPI(
    system=system,
    lqr_Q=np.eye(2),
    lqr_R=np.eye(2),
    dt=0.02,
    horizon=20,
    mc_rollouts=50000,
    lmbda=50.0,
    sigma=np.eye(2) * 4.0,
    transition_time=20,
    x_desired=x_desired,
    return_sampled_trajectories=False,
    return_optimal_trajectory=True,
  )

  sampled_trajectories = []
  for _ in range(1):
    sampled_trajectories.append(ax.plot([], [], "k-")[0])

  (optimal_trajectory_plot,) = ax.plot([], [], "k-", label="Optimal trajectory")
  flip = False

  def on_key_press(event):
    nonlocal flip, system, fig, ax
    if event.key == "c":
      flip = True
    if event.key == "x":
      system.reset(x_initial, visualizer=(fig, ax), visualize_history=True)
      mppi.reset_relaxation()

  fig.canvas.mpl_connect("key_press_event", on_key_press)

  @save_animation(fig, ax, filename="animation.gif", fps=20, repeat=True)
  def update(_):
    nonlocal current_time, desired_plot
    nonlocal weights, flip, pcm, colors

    # control = lqr_controller(system.state)
    if flip:
      mppi.reset_relaxation()
      c = colors.pop()
      colors.insert(0, c)
      w = weights.pop()
      weights.insert(0, w)

      pcm = visualize_cbfs([cbf10, cbf11, cbf12, cbf13], ax, granularity=200, unsafe_colors=colors, alpha=0.5, pcm=pcm)
      print("Flipped")
      flip = False

    t1 = time.time()
    control, _, optimal_trajectory, st = mppi.calc_control_input(
      system.state,
      [
        (
          cost,
          MppiCbfCostInputs(
            scalars=[*weights],
            vectors_ew=[
              mppi.relaxation_alphas,
              [x_desired[0] + r * np.sin(current_time) for _ in range(mppi.T)],
              [x_desired[1] + r * np.cos(current_time) for _ in range(mppi.T)],
            ],
            vectors_vw=[],
          ),
        ),
      ],
      x_desired=np.array([x_desired[0] + r * np.sin(current_time), x_desired[1] + r * np.cos(current_time)]),
    )
    desired_plot.set_data([x_desired[0] + r * np.sin(current_time)], [x_desired[1] + r * np.cos(current_time)])
    current_time += 0.05
    t2 = time.time()
    print(f"Time taken: {t2 - t1} seconds")
    system.step(control, dt=dt)
    system.visualize()
    time.sleep(0.05)

    if mppi.return_sampled_trajectories:
      for i, sampled_trajectory in enumerate(sampled_trajectories):
        sampled_trajectory.set_data(st[i][:, 0], st[i][:, 1])

    if mppi.return_optimal_trajectory:
      optimal_trajectory_plot.set_data(optimal_trajectory[:, 0], optimal_trajectory[:, 1])

  _ = FuncAnimation(fig, update, interval=33, frames=100)
  if show_animation:
    plt.show()
  else:
    plt.ioff()
    for frame in range(100):
      update(frame)
    plt.ion()


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--no_gui", action="store_true")
  args = parser.parse_args()
  main(show_animation=not args.no_gui)
