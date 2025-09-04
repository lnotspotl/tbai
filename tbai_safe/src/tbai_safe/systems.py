#!/usr/bin/env python3

import numpy as np
import scipy


class SimpleSingleIntegrator2D:
  """
  2D single integrator

  state: [x, y]
  control: [x_dot, y_dot]
  """

  def __init__(self, initial_state=np.zeros(2), visualizer=None):
    self.reset(initial_state, visualizer)

  def reset(self, initial_state, visualizer=None, visualize_history=False):
    # Reset state
    self.set_state(initial_state)
    self.visualizer = visualizer
    self.visualize_history = visualize_history

    self.x_history, self.y_history = None, None
    if self.visualize_history:
      self.x_history = [initial_state[0]]
      self.y_history = [initial_state[1]]

    # Reset visualizer
    self.visualizer = visualizer
    if visualizer is not None:
      com_color = "y"
      self.fig, self.ax = visualizer
      (self.com_plot,) = self.ax.plot([], [], f"{com_color}o", markersize=14, label="COM")
      if self.visualize_history:
        (self.com_history_plot,) = self.ax.plot([], [], "k--", alpha=0.5)
    return self

  def set_state(self, state):
    assert len(state) == 2, "State must be a list of length 2"
    self.state = state

  @staticmethod
  def integrate(state, control, dt):
    assert len(state) == 2, "State must be a list of length 2"
    assert len(control) == 2, "Control must be a list of length 2"

    state_dot = np.empty_like(state)
    state_dot[0] = control[0]
    state_dot[1] = control[1]
    return state + dt * state_dot

  def step(self, control, dt):
    self.state = self.integrate(self.state, control, dt)

  def visualize(self):
    assert self.visualizer is not None, "Visualizer is not set"
    com_x, com_y = [self.state[0]], [self.state[1]]
    self.com_plot.set_data(com_x, com_y)

    if self.visualize_history:
      self.x_history.append(self.state[0])
      self.y_history.append(self.state[1])
      self.com_history_plot.set_data(self.x_history, self.y_history)

  @staticmethod
  def get_A() -> np.ndarray:
    """Return the continuous-time state matrix A"""
    return np.zeros((2, 2))

  @staticmethod
  def get_B() -> np.ndarray:
    """Return the continuous-time input matrix B"""
    return np.eye(2)

  @property
  def dim_x(self) -> int:
    return 2

  @property
  def dim_u(self) -> int:
    return 2

  @staticmethod
  def _lqr_checks(Q: np.ndarray, R: np.ndarray):
    assert Q.shape == (2, 2), "Q matrix must be 2x2"
    assert R.shape == (2, 2), "R matrix must be 2x2"
    try:
      np.linalg.cholesky(Q)
      np.linalg.cholesky(R)
    except np.linalg.LinAlgError:
      raise ValueError("Q and R matrices must be positive definite")

  @staticmethod
  def get_lqr_gain(Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    SimpleSingleIntegrator2D._lqr_checks(Q, R)
    A, B = SimpleSingleIntegrator2D.get_A(), SimpleSingleIntegrator2D.get_B()
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = scipy.linalg.inv(R) @ B.T @ P
    return K

  @staticmethod
  def get_lqr_cost_expr(
    Q: np.ndarray, R: np.ndarray, x, y, u1, u2, x_desired=0.0, y_desired=0.0, u1_desired=0.0, u2_desired=0.0
  ):
    SimpleSingleIntegrator2D._lqr_checks(Q, R)
    v, u = np.array([x - x_desired, y - y_desired]), np.array([u1 - u1_desired, u2 - u2_desired])
    return v.T @ Q @ v + u.T @ R @ u
