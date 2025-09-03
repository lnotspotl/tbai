#!/usr/bin/env python3

import time
import functools

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from tbai_safe.systems import SimpleSingleIntegrator2D
from tbai_safe.cbf import (
  ControlBarrierFunctionFactory,
  visualize_cbfs,
  ControlBarrierFunctionMemoryEnvelope,
)
from tbai_safe.symperf import jit_expr
from tbai_safe.mppi import (
  AcceleratedSafetyMPPI,
  get_cost_function_parameterized,
  jit_expr_v2t,
  MppiCbfCost,
  MppiCbfCostInputs,
  cost_fn,
)


def main():
  # Initial and final states
  x_initial = np.array([-2.0, -3.4])
  x_desired = np.array([3.0, 3.0])

  # Soft CBFs
  factory2 = ControlBarrierFunctionFactory()
  d = 0
  sphere = True
  if sphere:
    cbf10 = factory2.get_sphere().substitute(r=1.303)
  else:
    cbf10 = factory2.get_rectangle().substitute(w=2.603, h=2.603)

  noise_scale = 1.0
  get_cbf10_cxcy = lambda: {
    "c_x": np.random.uniform(-1, 1) * noise_scale,
    "c_y": np.random.uniform(-1, 1) * noise_scale - d,
  }

  from tbai_safe.utils import PerformanceTimer

  timer = PerformanceTimer("s")
  scaler = 0.5
  timer.tick("Creating memory envelopes")
  cbf10_memory = ControlBarrierFunctionMemoryEnvelope(cbf10, size=10, P=10, scaler=scaler)
  timer.tock("Creating memory envelopes")
  print(f"Memory envelopes created in {timer.elapsed:.2f} seconds.")

  for i in range(3):
    cbf10_memory.memorize(cbf10.substitute(**get_cbf10_cxcy()))

    if i != 0:
      cbf10_memory.update_subs()

  # Visualize CBFs
  fig, ax = plt.subplots()
  # add minor grid
  ax.minorticks_on()
  ax.grid(which="both", alpha=0.2)
  ax.set_aspect("equal")
  ax.set_xlim(-4, 4)
  ax.set_ylim(-4, 4)
  colors = ["red"]
  weights = [5000.0]
  pcm = visualize_cbfs([cbf10_memory], ax, granularity=200, unsafe_colors=colors, alpha=0.9)

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
  print(cbf10_memory.expr_envelope)
  cbf10_jit = jit_expr_v2t(cbf10_memory.expr_envelope, symbols=cbf10_memory.all_symbols_envelope)

  print(cbf10_memory.expr)
  print(cbf10_memory.expr_envelope)
  print(cbf10_memory.all_symbols)
  print(cbf10_memory.all_symbols_envelope)
  print(cbf10_memory.get_args(x=float(system.state[0]), y=float(system.state[1])))

  @functools.partial(cost_fn, locals=locals(), globals=globals())
  def stage_cost(x, y, u1, u2, weight1, alpha, val1):
    cost = alpha * lqr_stage_jit(x=x, y=y, u1=0.0, u2=0.0)

    cbf10_val = cbf10_jit(x=x, y=y, val=val1)
    cost += (-weight1 * cbf10_val) if cbf10_val < 0 else 0
    return cost

  lqr_final_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desired[0], x_desired[1], 0.0, 0.0)
  lqr_final_jit = jit_expr(lqr_final_cost_expr)

  @functools.partial(cost_fn, locals=locals(), globals=globals())
  def terminal_cost(x, y, weight1, alpha, val1):
    return lqr_final_jit(x=x, y=y, u1=0.0, u2=0.0)

  mppi_cost_fn = get_cost_function_parameterized(
    stage_cost,
    terminal_cost,
    scalar_args=["weight1"],
    vector_args_ew=["alpha"],
    vector_args_vw=["val1"],
  )

  cost = MppiCbfCost(mppi_cost_fn)

  print("Jitted!")

  mppi = AcceleratedSafetyMPPI(
    system=system,
    lqr_Q=np.eye(2),
    lqr_R=np.eye(2),
    dt=0.02,
    horizon=20,
    mc_rollouts=40000,
    lmbda=50.0,
    sigma=np.eye(2) * 4.0,
    transition_time=20,
    x_desired=x_desired,
    return_sampled_trajectories=False,
    return_optimal_trajectory=True,
  )

  sampled_trajectories = []
  for _ in range(10):
    sampled_trajectories.append(ax.plot([], [], "k-")[0])

  (optimal_trajectory_plot,) = ax.plot([], [], "k-", label="Optimal trajectory")
  flip = False
  visualize_memory = False

  def on_key_press(event):
    nonlocal flip, system, fig, ax, visualize_memory
    if event.key == "c":
      flip = True
    if event.key == "x":
      system.reset(x_initial, visualizer=(fig, ax), visualize_history=True)
      mppi.reset_relaxation()
    if event.key == "z":
      visualize_memory = not visualize_memory

  fig.canvas.mpl_connect("key_press_event", on_key_press)

  from tbai_safe.anim import save_animation

  (envelope_plot,) = ax.plot([], [], "k-", label="Envelope")

  @save_animation(fig, ax, filename="animation.gif", fps=20, repeat=True)
  def update(i):
    nonlocal weights, flip, pcm, colors, visualize_memory

    if i % 2 == 0:
      cbf10_memory.memorize(cbf10.substitute(**get_cbf10_cxcy())).update_subs()

    # control = lqr_controller(system.state)
    if flip:
      mppi.reset_relaxation()
      c = colors.pop()
      colors.insert(0, c)
      w = weights.pop()
      weights.insert(0, w)

      print("Flipped")
      flip = False

    if visualize_memory:
      pcm = visualize_cbfs(
        [cbf10_memory],
        ax,
        granularity=200,
        unsafe_colors=colors,
        alpha=0.5,
        pcm=pcm,
      )
    else:
      pcm = visualize_cbfs([cbf10], ax, granularity=200, unsafe_colors=colors, alpha=0.5, pcm=pcm)

    t1 = time.time()
    control, _, optimal_trajectory, st = mppi.calc_control_input(
      system.state,
      [
        (
          cost,
          MppiCbfCostInputs(
            scalars=[*weights],
            vectors_ew=[np.array(mppi.relaxation_alphas)],
            vectors_vw=[cbf10_memory.get_args(x=float(system.state[0]), y=float(system.state[1]))],
          ),
        ),
      ],
    )

    print(cbf10_memory.get_args(x=float(system.state[0]), y=float(system.state[1]))[-2:])
    print()
    t2 = time.time()
    print(f"Time taken: {t2 - t1} seconds")
    system.step(control, dt=dt)
    system.visualize()

    x, y = cbf10_memory.get_envelope_points(P=10, scaler=scaler)
    x = [p + system.state[0] for p in x]
    y = [p + system.state[1] for p in y]
    envelope_plot.set_data(x, y)

    if mppi.return_sampled_trajectories:
      for i, sampled_trajectory in enumerate(sampled_trajectories):
        sampled_trajectory.set_data(st[i][:, 0], st[i][:, 1])

    if mppi.return_optimal_trajectory:
      optimal_trajectory_plot.set_data(optimal_trajectory[:, 0], optimal_trajectory[:, 1])

  _ = FuncAnimation(fig, update, interval=33, frames=100)
  plt.show()


if __name__ == "__main__":
  main()
