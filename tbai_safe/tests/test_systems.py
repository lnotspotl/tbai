#! /usr/bin/env python3


import numpy as np
from tbai_safe.systems import SimpleSingleIntegrator2D
import pytest


def test_single_integrator_2d_reset():
  system = SimpleSingleIntegrator2D()

  initial_state = np.array([1.0, 2.0])
  system.reset(initial_state)
  assert np.allclose(system.state, initial_state)


def test_single_integrator_2d_step():
  system = SimpleSingleIntegrator2D()

  initial_state = np.zeros(2)
  control = np.array([1, 2])
  dt = 1.0

  system.reset(initial_state)
  system.step(control, dt)

  assert np.allclose(system.state, control * dt)


def test_single_integrator_2d_lqr_controller():
  Q = np.eye(2)
  R = np.eye(2)
  K = SimpleSingleIntegrator2D.get_lqr_gain(Q, R)
  controller = lambda x: -K @ x
  assert np.allclose(controller(np.zeros(2)), np.zeros(2))

  # Simulate the system for N steps
  N = 50
  start_state = np.ones(2)
  system = SimpleSingleIntegrator2D()
  system.reset(start_state)

  for _ in range(N):
    control = controller(system.state)
    system.step(control, 1.0)

  assert np.allclose(system.state, np.zeros(2), atol=1e-2)


def test_single_integrator_2d_lqr_controller_raises():
  Q, R = np.eye(2), np.eye(2)
  with pytest.raises(ValueError):
    SimpleSingleIntegrator2D.get_lqr_gain(-Q, R)

  with pytest.raises(ValueError):
    SimpleSingleIntegrator2D.get_lqr_gain(Q, -R)

  with pytest.raises(ValueError):
    SimpleSingleIntegrator2D.get_lqr_gain(-Q, -R)
