#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from tbai_safe.systems import SimpleSingleIntegrator2D
from tbai_safe.cbf import ControlBarrierFunctionFactory, visualize_cbfs
from tbai_safe.control import VanillaSafetyFilterNew


def main():
  # Initial and final states
  x_initial = np.array([-2.0, -3.0])
  x_desired = np.array([3.0, 3.0])

  # Strict CBFs
  factory1 = ControlBarrierFunctionFactory()
  cbf1 = factory1.get_rectangle(method="approximate").substitute(c_x=3, c_y=-0.5, w=2.0, h=2.0)
  cbf2 = factory1.get_rectangle(method="approximate").substitute(c_x=-3, c_y=-0.5, w=2.0, h=2.0)
  cbf3 = factory1.get_sphere().substitute(c_x=1, c_y=0, r=1.3)
  cbf4 = factory1.union([cbf1, cbf2, cbf3], method="approximate").substitute(kappa=10)

  # Soft CBFs
  cbf5 = factory1.get_sphere().substitute(c_x=-1, c_y=0.2, r=1.3)

  # Visualize CBFs
  fig, ax = plt.subplots()
  ax.set_aspect("equal")
  ax.set_xlim(-4, 4)
  ax.set_ylim(-4, 4)
  _ = visualize_cbfs([cbf4, cbf5], ax, granularity=200, unsafe_colors=["red", "green"], alpha=0.99)

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
  K = system.get_lqr_gain(Q, R)
  lqr_controller = lambda x: -K @ (x - x_desired)

  # Get safety filter
  safety_filter = VanillaSafetyFilterNew(system.get_A(), system.get_B(), cbf4, alpha=5.5)

  def update(_):
    control = lqr_controller(system.state)
    control = safety_filter.solve(state=system.state, u_nominal=control)
    system.step(control, dt=dt)
    system.visualize()

  _ = FuncAnimation(fig, update, interval=33, frames=1000)
  plt.show()  # anim.save("animation.gif", writer="pillow") to save


if __name__ == "__main__":
  main()
