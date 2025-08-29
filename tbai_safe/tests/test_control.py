#!/usr/bin/env python3

import numpy as np
from tbai_safe.cbf import ControlBarrierFunctionFactory
from tbai_safe.control import VanillaSafetyFilterNew
from tbai_safe.systems import SimpleSingleIntegrator2D


def test_vanilla_safety_filter_new():
  factory = ControlBarrierFunctionFactory()
  cbf = factory.get_sphere().substitute(c_x=0, c_y=0, r=1)

  system = SimpleSingleIntegrator2D()
  filter = VanillaSafetyFilterNew(system.get_A(), system.get_B(), cbf, alpha=1.0)

  state = np.array([1.0, 1.0])
  u_nominal = np.array([0.0, 0.0])
  u = filter.solve(state=state, u_nominal=u_nominal)

  assert np.allclose(u, u_nominal)

  state = np.array([0.3, 0.0])
  u_nominal = np.array([0.0, 0.0])
  u = filter.solve(state=state, u_nominal=u_nominal)

  # Should be moving away from the sphere
  assert u[0] > 0
  assert np.allclose(u[1], 0.0)


def test_vanilla_safety_filter_new2():
  factory = ControlBarrierFunctionFactory()
  cbf = factory.get_sphere().substitute(c_x=0, c_y=0)

  system = SimpleSingleIntegrator2D()
  filter = VanillaSafetyFilterNew(system.get_A(), system.get_B(), cbf, alpha=1.0)

  state = np.array([1.0, 1.0])
  u_nominal = np.array([0.0, 0.0])
  u = filter.solve(state=state, u_nominal=u_nominal, r_1=1.0)

  assert np.allclose(u, u_nominal)

  state = np.array([0.3, 0.0])
  u_nominal = np.array([0.0, 0.0])
  u = filter.solve(state=state, u_nominal=u_nominal, r_1=1.0)

  # Should be moving away from the sphere
  assert u[0] > 0
  assert np.allclose(u[1], 0.0)

  u = filter.solve(state=state, u_nominal=u_nominal, r_1=0.1)

  # Is outside the sphere
  assert np.allclose(u, u_nominal)
