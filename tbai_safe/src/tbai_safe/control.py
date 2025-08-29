#!/usr/bin/env python3

import casadi as ca
import logging
from tbai_safe.systems import SimpleSingleIntegrator2D
import tbai_safe.cbf


logger = logging.getLogger("safety_filter")


class ProblemType:
  QP = "conic"
  NLP = "nlp"


class Solver:
  OSQP = "osqp"
  QPOASES = "qpoases"
  IPOPT = "ipopt"


def get_default_solver(problem_type: ProblemType):
  mapping = {
    ProblemType.QP: Solver.QPOASES,
    ProblemType.NLP: Solver.IPOPT,
  }
  return mapping[problem_type]


def get_default_solver_options(solver: Solver):
  mapping = {
    Solver.OSQP: {},
    Solver.QPOASES: {
      "print_time": False,
      "print_in": False,
      "print_out": False,
      "print_problem": False,
      "printLevel": "none",
    },
    Solver.IPOPT: {"ipopt.print_level": 0, "print_time": 0},
  }
  return mapping[solver]


class VanillaSafetyFilter:
  def __init__(self, system: SimpleSingleIntegrator2D, cbf: tbai_safe.cbf.ControlBarrierFunctionNew, alpha: float):
    self.system = system
    self.cbf = cbf
    self.alpha = alpha

    self._solve = self._get_safety_filter()

  def solve(self, *args, **kwargs):
    return self._solve(*args, **kwargs)

  def _get_safety_filter(self):
    A, B = self.system.get_A(), self.system.get_B()

    problem_type = ProblemType.QP

    # We know for a fact that this is an instance of a QP problem
    optistack = ca.Opti(problem_type)

    # Parameters
    u_nominal_param = optistack.parameter(2, 1)
    state_param = optistack.parameter(2, 1)

    # Decision variables
    u = optistack.variable(2, 1)

    # Safety constraint
    h = self.cbf.get_h(state_param[0], state_param[1])
    h_dot = self.cbf.get_h_dot(state_param[0], state_param[1], A, B, u)
    optistack.subject_to(h_dot >= -self.alpha * h)

    # Objective
    obj = ca.transpose(u - u_nominal_param) @ (u - u_nominal_param)
    optistack.minimize(obj)

    solver = get_default_solver(problem_type)
    solver_options = get_default_solver_options(solver)
    optistack.solver(solver, solver_options)

    def solve(state, u_nominal):
      # Set solver parameters
      optistack.set_value(state_param, state)
      optistack.set_value(u_nominal_param, u_nominal)

      # Solve
      sol = optistack.solve()
      return sol.value(u)

    return solve


class VanillaSafetyFilterNew:
  def __init__(self, A, B, cbf, alpha):
    assert isinstance(cbf, tbai_safe.cbf.ControlBarrierFunctionNew), "cbf must be a ControlBarrierFunctionNew"
    assert A.shape == (2, 2)
    assert B.shape == (2, 2)
    self.A = A
    self.B = B
    self.cbf = cbf
    self.cbf_casadi = tbai_safe.cbf.CasadiControlBarrierFunction(self.cbf)
    self.alpha = alpha

    self._solve = self._get_safety_filter()

  def solve(self, **kwargs):
    return self._solve(**kwargs)

  def _get_safety_filter(self):
    problem_type = ProblemType.QP
    optistack = ca.Opti(problem_type)

    # Decision variables
    u = optistack.variable(2, 1)
    u_nominal_p = optistack.parameter(2, 1)
    state_p = optistack.parameter(2, 1)

    # Get h and grad_h functions
    h_fn, h_symbols = self.cbf_casadi.get_h(substitute=True)
    grad_h_fn, grad_h_symbols = self.cbf_casadi.get_grad_h(substitute=True)

    # TODO: Refactor this to be more elegant
    state_symbols = {self.cbf.factory.x.name, self.cbf.factory.y.name}
    common_symbols = (h_symbols & grad_h_symbols) - state_symbols
    h_symbols = (h_symbols - common_symbols) - state_symbols
    grad_h_symbols = (grad_h_symbols - common_symbols) - state_symbols

    # Other parameters
    common_params = {s: optistack.parameter(1, 1) for s in common_symbols}
    h_params = {s: optistack.parameter(1, 1) for s in h_symbols}
    grad_h_params = {s: optistack.parameter(1, 1) for s in grad_h_symbols}

    # Safety constraint
    h = h_fn(x=state_p[0], y=state_p[1], **common_params, **h_params)
    grad_h = grad_h_fn(x=state_p[0], y=state_p[1], **common_params, **grad_h_params)
    h_dot = ca.transpose(grad_h) @ (self.A @ state_p + self.B @ u)
    optistack.subject_to(h_dot >= -self.alpha * h)

    # Objective
    obj = ca.transpose(u - u_nominal_p) @ (u - u_nominal_p)
    optistack.minimize(obj)

    solver = get_default_solver(problem_type)
    solver_options = get_default_solver_options(solver)
    optistack.solver(solver, solver_options)

    def solve(state, u_nominal, **other_args):
      # Set solver parameters
      optistack.set_value(state_p, state)
      optistack.set_value(u_nominal_p, u_nominal)

      # Set common parameters
      for k, v in common_params.items():
        optistack.set_value(v, other_args[k])

      # Set h parameters
      for k, v in h_params.items():
        optistack.set_value(v, other_args[k])

      # Set grad_h parameters
      for k, v in grad_h_params.items():
        optistack.set_value(v, other_args[k])

      # Solve QP
      sol = optistack.solve()
      return sol.value(u)

    return solve


class PDRegulator:
  def __init__(self, kp: float, kd: float) -> None:
    self.kp = kp
    self.kd = kd
    self.last_error = None

  def solve(self, desired, current, dt=1.0):
    error = desired - current
    self.last_error = error if self.last_error is None else self.last_error

    control = self.kp * error + self.kd * (error - self.last_error) / dt
    self.last_error = error
    return control

  def solve_error(self, error, dt=1.0):
    self.last_error = error if self.last_error is None else self.last_error
    control = self.kp * error + self.kd * (error - self.last_error) / dt
    self.last_error = error
    return control


if __name__ == "__main__":
  factory = tbai_safe.cbf.ControlBarrierFunctionFactory()
  cbf = factory.get_sphere().substitute(c_x=0, c_y=0)
  cbf = cbf * 10
  alpha = 1.0
  import numpy as np

  A = np.zeros((2, 2))
  B = np.eye(2)
  safety_filter = VanillaSafetyFilterNew(A, B, cbf, alpha)
  u_safe = safety_filter.solve(state=np.array([0.1, 0.0]), u_nominal=np.array([0, 0]), r_1=1.0)
  print(u_safe)
