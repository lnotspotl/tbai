#!/usr/bin/env python3

from tbai_safe.cbf import ControlBarrierFunctionFactory


def test_cbf_sphere():
  factory = ControlBarrierFunctionFactory()
  cbf = factory.get_sphere().substitute(c_x=0, c_y=0, r=1)

  # Inside = unsafe
  assert cbf.evaluate(x=0, y=0) < 0
  assert cbf.evaluate(x=0.1, y=0.2) < 0

  # Outside = safe
  assert cbf.evaluate(x=1, y=1) > 0

  # Boundary = safe
  assert cbf.evaluate(x=1, y=0) == 0
  assert cbf.evaluate(x=0, y=1) == 0


def test_cbf_rectangle_exact():
  factory = ControlBarrierFunctionFactory()
  cbf = factory.get_rectangle(method="exact").substitute(c_x=0, c_y=0, w=2, h=2)

  # Inside = unsafe
  assert cbf.evaluate(x=0, y=0) < 0
  assert cbf.evaluate(x=0.1, y=0.2) < 0

  # Outside = safe
  assert cbf.evaluate(x=2, y=2) > 0

  # Boundary = safe
  assert cbf.evaluate(x=1, y=1) == 0
  assert cbf.evaluate(x=0, y=1) == 0


def test_cbf_rectangle_approx():
  factory = ControlBarrierFunctionFactory()
  cbf = factory.get_rectangle(method="approximate").substitute(c_x=0, c_y=0, w=2, h=2, kappa=10)

  # Inside = unsafe
  assert cbf.evaluate(x=0, y=0) < 0
  assert cbf.evaluate(x=0.1, y=0.2) < 0
  assert cbf.evaluate(x=1, y=1) > 0  # corner rounding

  # Outside = safe
  assert cbf.evaluate(x=2, y=2) > 0


def test_union_exact():
  factory = ControlBarrierFunctionFactory()
  cbf1 = factory.get_sphere().substitute(c_x=0, c_y=0)
  cbf2 = factory.get_sphere().substitute(c_x=2, c_y=0)
  cbf = factory.union([cbf1, cbf2], method="exact")

  for s in cbf1.h_expr.free_symbols | cbf2.h_expr.free_symbols:
    assert s in cbf.h_expr.free_symbols


def test_union_approximate():
  factory = ControlBarrierFunctionFactory()
  cbf1 = factory.get_sphere().substitute(c_x=0, c_y=0)
  cbf2 = factory.get_sphere().substitute(c_x=2, c_y=0)
  cbf = factory.union([cbf1, cbf2], method="approximate")

  for s in cbf1.h_expr.free_symbols | cbf2.h_expr.free_symbols:
    assert s in cbf.h_expr.free_symbols

  assert factory.kappa in cbf.h_expr.free_symbols


def test_intersection_exact():
  factory = ControlBarrierFunctionFactory()
  cbf1 = factory.get_sphere().substitute(c_x=0, c_y=0)
  cbf2 = factory.get_sphere().substitute(c_x=2, c_y=0)
  cbf = factory.intersection([cbf1, cbf2], method="exact")

  for s in cbf1.h_expr.free_symbols | cbf2.h_expr.free_symbols:
    assert s in cbf.h_expr.free_symbols


def test_intersection_approximate():
  factory = ControlBarrierFunctionFactory()
  cbf1 = factory.get_sphere().substitute(c_x=0, c_y=0)
  cbf2 = factory.get_sphere().substitute(c_x=2, c_y=0)
  cbf = factory.intersection([cbf1, cbf2], method="approximate")

  for s in cbf1.h_expr.free_symbols | cbf2.h_expr.free_symbols:
    assert s in cbf.h_expr.free_symbols

  assert factory.kappa in cbf.h_expr.free_symbols
