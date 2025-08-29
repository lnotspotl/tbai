#!/usr/bin/env python3

import time

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
  MppiCost,
  cost_fn,
)
from tbai_safe.anim import save_animation


def main():
  # Initial and final states
  x_initial = np.array([-2.0, -3.4])
  x_desired = np.array([3.0, 3.0])

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
  ax.plot(x_desired[0], x_desired[1], "bx", label="Desired state", markersize=10)
  ax.legend()

  # Initialize system
  system = SimpleSingleIntegrator2D().reset(x_initial, visualizer=(fig, ax), visualize_history=True)
  system.visualize()
  dt = 0.02

  # Get LQR controller
  Q, R = np.eye(2), np.eye(2)

  # Prepare stage cost
  x1, x2, u1, u2 = factory2.x, factory2.y, factory2.u1, factory2.u2
  lqr_stage_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desired[0], x_desired[1], 0.0, 0.0)
  lqr_stage_jit = jit_expr(lqr_stage_cost_expr)
  cbf10_jit = jit_expr(cbf10.get_expr(substitute=True))
  cbf11_jit = jit_expr(cbf11.get_expr(substitute=True))
  cbf12_jit = jit_expr(cbf12.get_expr(substitute=True))
  cbf13_jit = jit_expr(cbf13.get_expr(substitute=True))

  def stage_cost1(x, y, u1, u2, weight1, weight2, weight3, weight4):
    cbf10_val = cbf10_jit(x=x, y=y)
    return (-weight1 * cbf10_val) if cbf10_val < 0 else 0

  def stage_cost2(x, y, u1, u2, weight1, weight2, weight3, weight4):
    cbf11_val = cbf11_jit(x=x, y=y)
    return (-weight2 * cbf11_val) if cbf11_val < 0 else 0

  def stage_cost3(x, y, u1, u2, weight1, weight2, weight3, weight4):
    cbf12_val = cbf12_jit(x=x, y=y)
    return (-weight3 * cbf12_val) if cbf12_val < 0 else 0

  def stage_cost4(x, y, u1, u2, weight1, weight2, weight3, weight4):
    cbf13_val = cbf13_jit(x=x, y=y)
    return (-weight4 * cbf13_val) if cbf13_val < 0 else 0

  lqr_final_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desired[0], x_desired[1], 0.0, 0.0)
  lqr_final_jit = jit_expr(lqr_final_cost_expr)

  def terminal_cost(x, y, alpha=1.0):
    return lqr_final_jit(x=x, y=y, u1=0.0, u2=0.0)

  mppi_cost_fn1 = get_cost_function_parameterized(
    cost_fn(stage_cost1),
    cost_fn(lambda x, y, weight1, weight2, weight3, weight4: 0.0),
    scalar_args=["weight1", "weight2", "weight3", "weight4"],
    vector_args_ew=[],
    vector_args_vw=[],
  )

  mppi_cost_fn3 = get_cost_function_parameterized(
    cost_fn(stage_cost3),
    cost_fn(lambda x, y, weight1, weight2, weight3, weight4: 0.0),
    scalar_args=["weight1", "weight2", "weight3", "weight4"],
    vector_args_ew=[],
    vector_args_vw=[],
  )

  mppi_cost_fn4 = get_cost_function_parameterized(
    cost_fn(stage_cost4),
    cost_fn(lambda x, y, weight1, weight2, weight3, weight4: 0.0),
    scalar_args=["weight1", "weight2", "weight3", "weight4"],
    vector_args_ew=[],
    vector_args_vw=[],
  )

  lqr_cost_fn = get_cost_function_parameterized(
    cost_fn(lambda x, y, u1, u2, alpha: alpha * lqr_stage_jit(x=x, y=y, u1=0.0, u2=0.0)),
    cost_fn(terminal_cost),
    scalar_args=[],
    vector_args_ew=["alpha"],
    vector_args_vw=[],
  )

  cost1 = MppiCbfCost(mppi_cost_fn1)
  cost2 = cost1
  cost3 = MppiCbfCost(mppi_cost_fn3)
  cost4 = MppiCbfCost(mppi_cost_fn4)
  cost5 = MppiCost(lqr_cost_fn)

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
  flip = False

  def on_key_press(event):
    nonlocal flip, system, fig, ax
    if event.key == "c":
      flip = True
    if event.key == "x":
      system.reset(x_initial, visualizer=(fig, ax), visualize_history=True)
      mppi.reset_relaxation()

  fig.canvas.mpl_connect("key_press_event", on_key_press)

  frames = []

  @save_animation(fig, ax, filename="animation.gif", fps=20, repeat=True)
  def update(_):
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
        (cost1, MppiCbfCostInputs(scalars=[*weights], vectors_ew=[], vectors_vw=[])),
        (cost2, MppiCbfCostInputs(scalars=[*weights], vectors_ew=[], vectors_vw=[])),
        (cost3, MppiCbfCostInputs(scalars=[*weights], vectors_ew=[], vectors_vw=[])),
        (cost4, MppiCbfCostInputs(scalars=[*weights], vectors_ew=[], vectors_vw=[])),
        (cost5, (mppi.relaxation_alphas,)),
      ],
    )
    t2 = time.time()
    print(f"Time taken: {t2 - t1} seconds")
    system.step(control, dt=dt)
    system.visualize()
    time.sleep(0.05)

    fig.canvas.draw()
    frames.append(fig.canvas.copy_from_bbox(ax.bbox))

    if mppi.return_sampled_trajectories:
      for i, sampled_trajectory in enumerate(sampled_trajectories):
        sampled_trajectory.set_data(st[i][:, 0], st[i][:, 1])

    if mppi.return_optimal_trajectory:
      optimal_trajectory_plot.set_data(optimal_trajectory[:, 0], optimal_trajectory[:, 1])

  _ = FuncAnimation(fig, update, interval=33, frames=100)
  plt.show()


if __name__ == "__main__":
  main()
