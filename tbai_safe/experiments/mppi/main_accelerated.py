#!/usr/bin/env python3

import time

import functools

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from tbai_safe.systems import SimpleSingleIntegrator2D
from tbai_safe.cbf import ControlBarrierFunctionFactory, visualize_cbfs
from tbai_safe.control import VanillaSafetyFilterNew
from tbai_safe.symperf import jit_expr
from tbai_safe.mppi import (
  AcceleratedSafetyMPPI,
  get_cost_function_parameterized,
  MppiCbfCost,
  MppiCbfCostInputs,
  cost_fn,
)
from tbai_safe.anim import save_animation


def main():
  # Initial and final states
  x_initial = np.array([-3.0, -3.4])
  x_desired = np.array([3.0, 3.0])

  # Strict CBFs
  factory1 = ControlBarrierFunctionFactory()
  cbf1 = factory1.get_rectangle(method="approximate").substitute(c_x=3, c_y=-0.5, w=2.0, h=2.0)
  cbf2 = factory1.get_rectangle(method="approximate").substitute(c_x=-3, c_y=-0.5, w=2.0, h=2.0)
  cbf3 = factory1.union([cbf1, cbf2], method="approximate").substitute(kappa=10)

  # Soft CBFs
  factory2 = ControlBarrierFunctionFactory()
  cbf4 = factory2.get_sphere().substitute(c_x=1, c_y=0, r=1.3)
  cbf5 = factory2.get_sphere().substitute(c_x=-1, c_y=0.2, r=1.3)

  # Visualize CBFs
  colors = ["red", "green", "blue"]
  fig, ax = plt.subplots()
  ax.set_aspect("equal")
  ax.set_xlim(-4, 4)
  ax.set_ylim(-4, 4)
  pcm = visualize_cbfs([cbf3, cbf4, cbf5], ax, granularity=200, unsafe_colors=colors, alpha=0.5)

  # Visualize initial and desired states
  ax.plot(x_initial[0], x_initial[1], "rx", label="Initial state", markersize=10)
  ax.plot(x_desired[0], x_desired[1], "bx", label="Desired state", markersize=10)
  ax.legend()

  # Initialize system
  system = SimpleSingleIntegrator2D().reset(x_initial, visualizer=(fig, ax), visualize_history=True)
  system.visualize()
  dt = 0.02

  # Get LQR controller
  Q, R = np.eye(2), np.eye(2)

  # Get safety filter
  safety_filter = VanillaSafetyFilterNew(system.get_A(), system.get_B(), cbf3, alpha=5.5)

  # Prepare stage cost
  x1, x2, u1, u2 = factory1.x, factory1.y, factory1.u1, factory1.u2
  lqr_stage_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desired[0], x_desired[1], 0.0, 0.0)
  lqr_stage_jit = jit_expr(lqr_stage_cost_expr)
  cbf4_jit = jit_expr(cbf4.get_expr(substitute=True))
  cbf5_jit = jit_expr(cbf5.get_expr(substitute=True))

  @functools.partial(cost_fn, locals=locals(), globals=globals())
  def stage_cost(x, y, u1, u2, weight1, weight2, alpha):
    cost = alpha * lqr_stage_jit(y=y, x=x, u1=u1, u2=u2)

    cbf4_val = cbf4_jit(x=x, y=y)
    cbf5_val = cbf5_jit(x=x, y=y)
    cost += (-weight1 * cbf4_val) if cbf4_val < 0 else 0
    cost += (-weight2 * cbf5_val) if cbf5_val < 0 else 0
    return cost

  lqr_final_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desired[0], x_desired[1], 0.0, 0.0)
  lqr_final_jit = jit_expr(lqr_final_cost_expr)

  @functools.partial(cost_fn, locals=locals(), globals=globals())
  def terminal_cost(x, y, weight1=1.0, weight2=1.0, alpha=1.0):
    return lqr_final_jit(x=x, y=y, u1=0.0, u2=0.0)

  mppi_cost_fn = get_cost_function_parameterized(
    stage_cost,
    terminal_cost,
    scalar_args=["weight1", "weight2"],
    vector_args_ew=["alpha"],
  )

  cost = MppiCbfCost(mppi_cost_fn)

  print("Jitted!")

  mppi = AcceleratedSafetyMPPI(
    system=system,
    lqr_Q=np.eye(2),
    lqr_R=np.eye(2),
    dt=0.02,
    horizon=20,
    mc_rollouts=1000,
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
  weight1 = 1.0
  weight2 = 5000.0
  flip = False

  # Enable grid
  ax.minorticks_on()
  ax.grid(which="both", alpha=0.2)

  def on_key_press(event):
    nonlocal flip, system, fig, ax
    if event.key == "c":
      flip = True
    if event.key == "x":
      system.reset(x_initial, visualizer=(fig, ax), visualize_history=True)
      mppi.reset_relaxation()

  fig.canvas.mpl_connect("key_press_event", on_key_press)

  current_time = 0.0

  @save_animation(fig, ax, filename="animation.gif", fps=20, repeat=True, include_all=True)
  def update(i):
    nonlocal weight1, weight2, flip, pcm, colors, current_time

    # control = lqr_controller(system.state)
    if flip:
      weight1, weight2 = weight2, weight1
      mppi.reset_relaxation()

      colors[1], colors[2] = colors[2], colors[1]

      pcm = visualize_cbfs([cbf3, cbf4, cbf5], ax, granularity=200, unsafe_colors=colors, alpha=0.5, pcm=pcm)
      print("Flipped")
      flip = False

    t1 = time.time()
    control, _, optimal_trajectory, st = mppi.calc_control_input(
      system.state,
      [
        (
          cost,
          MppiCbfCostInputs(scalars=[weight1, weight2], vectors_ew=[np.array(mppi.relaxation_alphas)], vectors_vw=[]),
        ),
      ],
    )
    control = safety_filter.solve(state=system.state, u_nominal=control)
    t2 = time.time()
    print(f"Time taken: {t2 - t1} seconds")
    system.step(control, dt=dt)
    system.visualize()

    # att title and set the time
    fig.suptitle(f"Time: {current_time:.2f} s")
    current_time += dt
    time.sleep(0.05)

    if mppi.return_sampled_trajectories:
      for i, sampled_trajectory in enumerate(sampled_trajectories):
        sampled_trajectory.set_data(st[i][:, 0], st[i][:, 1])

    if mppi.return_optimal_trajectory:
      optimal_trajectory_plot.set_data(optimal_trajectory[:, 0], optimal_trajectory[:, 1])

  _ = FuncAnimation(fig, update, interval=33, frames=100)
  plt.show()


if __name__ == "__main__":
  main()
