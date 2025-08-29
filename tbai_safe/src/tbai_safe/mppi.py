#!/usr/bin/env python3

import math
import textwrap
import inspect
import numpy as np
from tbai_safe.systems import SimpleSingleIntegrator2D
from dataclasses import dataclass
import numba

import tbai_safe.logging

import os

try:
  import cupy as cp

  has_cuda = True
except ImportError:
  has_cuda = False

from typing import Callable
import sympy as sp

logger = tbai_safe.logging.get_logger(__name__)


@dataclass
class MppiConfig:
  backend: str = os.environ.get("MPPI_BACKEND", "numpy")
  dtype: str = os.environ.get("MPPI_DTYPE", "float64")

  def __post_init__(self):
    assert self.backend in ["numpy", "cuda"], f"Invalid backend: {self.backend}"
    assert self.dtype in ["float64", "float32"], f"Invalid dtype: {self.dtype}"
    logger.info(f"Setting default backend to {self.backend}")
    logger.info(f"Setting default dtype to {self.dtype}")


_mppi_config = MppiConfig()


def set_default_backend(backend: str):
  assert backend in ["numpy", "cuda"], f"Invalid backend: {backend}"
  _mppi_config.backend = backend


def set_default_dtype(dtype: str):
  assert dtype in ["float64", "float32"], f"Invalid dtype: {dtype}"
  _mppi_config.dtype = dtype


def get_default_backend():
  return _mppi_config.backend


def get_default_dtype():
  return _mppi_config.dtype


def cost_fn(fn: Callable, backend: str = None, dtype: str = None):
  if hasattr(fn, "_tbai_safe_backend"):
    return fn

  backend = get_default_backend() if backend is None else backend
  dtype = get_default_dtype() if dtype is None else dtype

  if backend == "numpy":
    jitted_fn = numba.njit(fn, fastmath=False)
    jitted_fn._tbai_safe_backend = backend
    return jitted_fn
  elif backend == "cuda":
    jitted_fn = numba.cuda.jit(fn, fastmath=False, device=True)
    jitted_fn._tbai_safe_backend = backend
    return jitted_fn
  else:
    raise ValueError(f"Invalid backend: {backend}")


class MppiCost:
  def __init__(self, fn: Callable):
    self.fn = fn

  def evaluate(self, X, U, S, other_inputs):
    """X is of shape (B, T, 2), U is of shape (B, T, 2) - it is states and actions respectively, S is of shape (B,) and is the buffer where the cost should be accumulated"""
    return self.fn(X, U, *other_inputs, S)


@dataclass
class MppiCbfCostInputs:
  scalars: list  # constant for each rollout for every time step
  vectors_vw: list  # vector-wise for each time step (per-rollout, constant for each time step)
  vectors_ew: list  # element-wise for each time step (per-rollout per-time-step)


class MppiCbfCost(MppiCost):
  def __init__(self, fn: Callable, backend: str = None, dtype: str = None):
    super().__init__(fn)
    self.backend = get_default_backend() if backend is None else backend
    self.dtype = get_default_dtype() if dtype is None else dtype
    self.eval_fn = self.evaluate_numpy if self.backend == "numpy" else self.evaluate_cuda

  def evaluate_numpy(self, X, U, S, other_inputs: MppiCbfCostInputs):
    other_inputs.vectors_ew = [
      v if isinstance(v, np.ndarray) else np.array(v, dtype=self.dtype) for v in other_inputs.vectors_ew
    ]
    other_inputs.vectors_vw = [
      v if isinstance(v, np.ndarray) else np.array(v, dtype=self.dtype) for v in other_inputs.vectors_vw
    ]
    return self.fn(X, U, *other_inputs.scalars, *other_inputs.vectors_ew, *other_inputs.vectors_vw, S)

  def evaluate_cuda(self, X, U, S, other_inputs: MppiCbfCostInputs):
    other_inputs.vectors_ew = [
      v if isinstance(v, cp.ndarray) else cp.array(v, dtype=self.dtype) for v in other_inputs.vectors_ew
    ]
    other_inputs.vectors_vw = [
      v if isinstance(v, cp.ndarray) else cp.array(v, dtype=self.dtype) for v in other_inputs.vectors_vw
    ]
    return self.fn(X, U, *other_inputs.scalars, *other_inputs.vectors_ew, *other_inputs.vectors_vw, S)

  def evaluate(self, X, U, S, other_inputs: MppiCbfCostInputs):
    return self.eval_fn(X, U, S, other_inputs)


def discretize_system(A, B, dt):
  """Discretize a linear system

  Args:
    A: The system matrix of shape (n, n)
    B: The control matrix of shape (n, m)
    dt: The time step

  Returns:
    Ad: The discretized system matrix of shape (n, n)
    Bd: The discretized control matrix of shape (n, m)
  """
  assert A.shape[0] == A.shape[1], "A must be a square matrix"
  assert B.shape[0] == A.shape[0], "B must have the same number of rows as A"

  # Euler integration: x_dot = A x + B u
  # x_next = x + x_dot * dt => x_next = (I + A * dt) x + B * dt * u
  Ad = np.eye(A.shape[0]) + A * dt
  Bd = B * dt
  return Ad, Bd


def get_hat_system(A: np.ndarray, B: np.ndarray, T: int, dtype=np.float32):
  """Get the hat system for the given system.

  Args:
    A: The system matrix of shape (2, 2)
    B: The control matrix of shape (2, 2)
    T: The number of time steps
    dtype: The data type of the matrices

  Returns:
    Ahat: The hat system matrix of shape (2 * T, 2)
    Bhat: The hat control matrix of shape (2 * T, 2 * T)
  """
  assert A.shape == (2, 2) and B.shape == (2, 2), (
    f"A and B must be 2x2 and 2x2 matrices, found: {A.shape} and {B.shape}"
  )
  Bhat = np.zeros((2 * T, 2 * T))
  Ahat = np.zeros((2 * T, 2))
  for i in range(T):
    Ahat[2 * i : 2 * i + 2, :] = Ahat[2 * (i - 1) : 2 * i, :] @ A if i > 0 else np.eye(2)

  # [B    0 0]
  # [AB   B 0]
  # [AAB AB B]
  for j in range(0, T):
    Bhat[2 * j : 2 * j + 2, 2 * j : 2 * j + 2] = B
    for i in range(j + 1, T):
      Bhat[2 * i : 2 * i + 2, 2 * j : 2 * j + 2] = A @ Bhat[2 * (i - 1) : 2 * i, 2 * j : 2 * j + 2]
  return Ahat.astype(dtype), Bhat.astype(dtype)


def get_cost_function_parameterized(
  stage_cost,
  terminal_cost,
  scalar_args=[],
  vector_args_vw=[],
  vector_args_ew=[],
  stype=None,
  backend=None,
  threads_per_block=256,
):
  # scalar_args = constant for each rollout for every time step
  # vector_args_ew = element-wise for each time step (per-rollout per-time-step)
  # vector_args_vw = vector-wise for each time step (per-rollout, constant for each time step)
  assert hasattr(stage_cost, "_tbai_safe_backend"), "Stage cost must be jitted with cost_fn"
  assert hasattr(terminal_cost, "_tbai_safe_backend"), "Terminal cost must be jitted with cost_fn"

  stype = get_default_dtype() if stype is None else stype
  backend = get_default_backend() if backend is None else backend

  scalar_args_str = ", ".join([f"{stype}" for _ in scalar_args])
  scalar_args_str = scalar_args_str + ", " if scalar_args_str else ""
  vector_args_ew_str = ", ".join([f"{stype}[:]" for _ in vector_args_ew])
  vector_args_ew_str = vector_args_ew_str + ", " if vector_args_ew_str else ""
  vector_args_vw_str = ", ".join([f"{stype}[:]" for _ in vector_args_vw])
  vector_args_vw_str = vector_args_vw_str + ", " if vector_args_vw_str else ""
  f32an = (
    f"void({stype}[:,:,::1], {stype}[:,:,::1], {scalar_args_str} {vector_args_ew_str} {vector_args_vw_str} {stype}[:])"
  )
  einsum_str = "(b,r,x),(b,r,u){args}{vectors_ev}{vectors_vv}->(b)"
  args = ", ".join(["()" for _ in scalar_args])
  args = (", " if args else "") + args
  vectors_ev = ", ".join(["(r)" for _ in vector_args_ew])
  vectors_ev = (", " if vectors_ev else "") + vectors_ev
  assert len(vector_args_vw) <= len("qweyughf")  # We are running out of characters here :D
  vectors_vv = ", ".join([f"({c})" for (c, _) in zip("qweyughf", vector_args_vw)])
  vectors_vv = (", " if vectors_vv else "") + vectors_vv
  einsum_str = einsum_str.format(args=args, vectors_ev=vectors_ev, vectors_vv=vectors_vv)

  args_str = ", ".join(
    [f"{scalar_args[i]}" for i in range(len(scalar_args))]
    + [f"{vector_args_ew[i]}" for i in range(len(vector_args_ew))]
    + [f"{vector_args_vw[i]}" for i in range(len(vector_args_vw))]
  )
  args_str = args_str + ", " if args_str else ""

  if backend == "numpy":
    use_str = ", ".join(
      [f"{scalar_args[i]}={scalar_args[i]}" for i in range(len(scalar_args))]
      + [f"{vector_args_ew[i]}={vector_args_ew[i]}[t]" for i in range(len(vector_args_ew))]
      + [f"{vector_args_vw[i]}={vector_args_vw[i]}" for i in range(len(vector_args_vw))]
    )

  if backend == "cuda":
    ## Cuda backend does not support keyword arguments
    use_str = ", ".join(
      [f"{scalar_args[i]}" for i in range(len(scalar_args))]
      + [f"{vector_args_ew[i]}[t]" for i in range(len(vector_args_ew))]
      + [f"{vector_args_vw[i]}" for i in range(len(vector_args_vw))]
    )

  use_str = use_str + ", " if use_str else ""

  cc = None

  ### Numpy backend
  if backend == "numpy":
    target = "cpu"

    cc = f"""
    import numba
    TARGET = "{target}"
    if not hasattr(stage_cost, 'signatures'):
      stage_cost = numba.njit(stage_cost, fastmath=False)
    if not hasattr(terminal_cost, 'signatures'):
      terminal_cost = numba.njit(terminal_cost, fastmath=False)
    @numba.guvectorize(['{f32an}'], '{einsum_str}', target=TARGET, fastmath=False)
    def total_cost_vec(x, u, {args_str} out):
      B, T, _ = x.shape
      for i in range(B):
        for t in range(T):
          if t == T - 1:
            out[i] += terminal_cost(x[i, t, 0], x[i, t, 1], {use_str})
          out[i] += stage_cost(x[i, t, 0], x[i, t, 1], u[i, t, 0], u[i, t, 1], {use_str})
    """

  ### CUDA backend
  if backend == "cuda":
    cc = f"""
    import numba
    import numba.cuda
    stage_cost = numba.cuda.jit(stage_cost, fastmath=False, device=True)
    terminal_cost = numba.cuda.jit(terminal_cost, fastmath=False, device=True)
    @numba.cuda.jit
    def total_cost_vec(x, u, {args_str} out):
        thread_id = numba.cuda.threadIdx.x
        block_id = numba.cuda.blockIdx.x
        idx = thread_id + block_id * numba.cuda.blockDim.x
        B = x.shape[0]
        T = x.shape[1]
        if idx < B:
            total = 0.0
            for t in range(T):
              out[idx] = 0.0
              if t == T - 1:
                total += terminal_cost(x[idx, t, 0], x[idx, t, 1], {use_str})
              total += stage_cost(x[idx, t, 0], x[idx, t, 1], u[idx, t, 0], u[idx, t, 1], {use_str})
            out[idx] = total
    """

  assert cc is not None, f"Invalid backend: {backend}"
  cc = textwrap.dedent(cc)

  logger.debug(
    f"Stage cost source:\n{textwrap.dedent(inspect.getsource(stage_cost))}\nTerminal cost source:\n{textwrap.dedent(inspect.getsource(terminal_cost))}\n{cc}"
  )

  exec(cc, locals())

  fn = locals()["total_cost_vec"]
  if backend == "numpy":
    return fn

  ## Create a small wrapper function to call the cuda function
  if backend == "cuda":

    def wrapper(x, u, *args, **kwargs):
      blocks_per_grid = (x.shape[0] + threads_per_block - 1) // threads_per_block
      fn[blocks_per_grid, threads_per_block](x, u, *args, **kwargs)

    return wrapper

  raise ValueError(f"Invalid backend: {backend}")


def jit_expr_v2t(expr: sp.Expr, nbtype=numba.float32, cse=True, parallel=False, symbols=None, backend=None):
  """Jit an expression that takes in x, y and val[i] and returns the expression evaluated at x, y and val[i]

  This function essentially unpacks the val arguments as numba currently does not support passing in a list of arguments.
  """
  backend = get_default_backend() if backend is None else backend

  header = f"@numba.jit(nopython=True, parallel={parallel})" if backend == "numpy" else "@numba.cuda.jit(device=True)"

  if backend == "cuda":
    # Cuda backend does not support keyword arguments
    args = ", ".join(["x", "y"] + [f"val[{i}]" for i in range(len(symbols))])
  if backend == "numpy":
    args = ", ".join(["x=x", "y=y"] + [f"{symbols[i]}=val[{i}]" for i in range(len(symbols))])

  cc = f"""
  import numba
  {"import numba.cuda" if backend == "cuda" else ""}
  from tbai_safe.symperf import jit_expr
  jitted = jit_expr(expr, nbtype, cse, parallel, symbols=["x", "y"] + symbols, backend="{backend}", device=True)
  {header}
  def jitted_wrapper(x, y, val):
    return jitted({args})
  """
  cc = textwrap.dedent(cc)
  logger.debug(f"Jitted expression: {expr}\n\n{cc}")
  exec(cc, locals())
  fn = locals()["jitted_wrapper"]
  return fn


class AcceleratedSafetyMPPI:
  def __init__(
    self,
    system: SimpleSingleIntegrator2D,
    lqr_Q: np.ndarray = np.eye(2),
    lqr_R: np.ndarray = np.eye(2),
    dt: float = 0.02,
    horizon: int = 100,
    mc_rollouts: int = 1000,
    lmbda: float = 50.0,
    sigma: float = np.eye(2) * 2.0,
    transition_time: int = 3,
    x_desired: np.ndarray = np.array([2.0, 2.0]),
    return_optimal_trajectory: bool = False,
    return_sampled_trajectories: bool = False,
    backend: str = None,
    stype: str = None,
  ):
    backend = get_default_backend() if backend is None else backend
    stype = get_default_dtype() if stype is None else stype

    if backend == "numpy":
      self.backend = np
    if backend == "cuda":
      assert has_cuda, "Numba CUDA is not installed"
      self.backend = cp

    if stype == "float64":
      self.stype = self.backend.float64
    if stype == "float32":
      self.stype = self.backend.float32

    assert isinstance(system, SimpleSingleIntegrator2D)
    self.system = system
    self.lqr_matrix = self.backend.array(self.system.get_lqr_gain(lqr_Q, lqr_R)).astype(self.stype)
    self.lqr_controller = lambda x: -self.lqr_matrix @ x

    # Integration time step
    self.dt = dt
    # Number of lookahead steps
    self.T = horizon

    # Number of Monte Carlo rollouts
    self.K = mc_rollouts

    # Lambda parameter
    self.lmbda = lmbda

    # Sigma parameter
    self.sigma = sigma

    self.max_abs_velocity = 2.3

    # Desired state (one that LQR tries to reach)
    self.x_desired = self.backend.asarray(x_desired)

    # Whether to return the optimal trajectory
    self.return_optimal_trajectory = return_optimal_trajectory

    # Whether to return the sampled trajectories
    self.return_sampled_trajectories = return_sampled_trajectories

    # LQR controller
    self.transition_time = transition_time
    self.reset_relaxation(init=True)
    assert hasattr(self, "relaxation_alphas")
    self.u_prev = self.backend.zeros((self.T, self.system.dim_u))

    A, B = self.system.get_A(), self.system.get_B()
    Ad, Bd = discretize_system(A, B, self.dt)
    self.Ahat, self.Bhat = get_hat_system(Ad, Bd, self.T, dtype=self.stype)
    self.Ahat = self.backend.array(self.Ahat).astype(self.stype)
    self.Bhat = self.backend.array(self.Bhat).astype(self.stype)

    self.v = self.backend.zeros((self.K, self.T, 2), dtype=self.stype)
    self.S = self.backend.zeros((self.K,), dtype=self.stype)

  def step_relaxation(self):
    self.relaxation_alphas.pop(0)
    self.relaxation_alphas.append(1)

  def reset_relaxation(self, init: bool = False):
    if init:
      self.relaxation_alphas = [1 for i in range(self.transition_time)]
      while len(self.relaxation_alphas) < self.T:
        self.relaxation_alphas.append(1)
    else:
      self.relaxation_alphas = np.linspace(0, 1, self.transition_time).tolist()
      while len(self.relaxation_alphas) < self.T:
        self.relaxation_alphas.append(1)

  def calc_control_input(self, observed_x: np.ndarray, cost_fn_args):
    # Set u to be the LQR controller response
    observed_x = self.backend.asarray(observed_x, dtype=self.stype)
    current_state = observed_x
    a = 0.3 * self.relaxation_alphas[0]
    for i in range(self.T):
      u = self.lqr_controller(current_state - self.x_desired)
      current_state = self.integrate(current_state, u)
      self.u_prev[i] = u * a + (1 - a) * self.u_prev[i]

    u = self.u_prev

    # set initial x value from observation
    x0 = observed_x

    # sample noise
    epsilon = self.calculate_epsilon(self.sigma, self.K, self.T, self.system.dim_u)  # size is self.K x self.T

    self.v[:, :, :] = u + epsilon
    # v = self.v.astype(np.float64)
    v = self.v
    v = self.backend.clip(v, -self.max_abs_velocity, self.max_abs_velocity)

    self.S[:] = 0.0
    out = (
      self.backend.matmul(
        self.Bhat[self.backend.newaxis, :, :], v.reshape(self.K, self.T * 2)[:, :, self.backend.newaxis]
      ).squeeze(-1)
      + self.backend.matmul(self.Ahat, x0)[self.backend.newaxis, :]
    ).reshape(self.K, self.T, 2)

    for fn, args in cost_fn_args:
      fn.evaluate(out, v, self.S, args)

    w = self.compute_weights(self.S)
    w_epsilon = self.backend.sum(w[:, self.backend.newaxis, self.backend.newaxis] * epsilon, axis=0)

    w_epsilon = self.moving_average_filter(xx=w_epsilon, window_size=self.T)
    u += w_epsilon * max(self.relaxation_alphas[0], 0.1)

    self.step_relaxation()
    optimal_traj = None
    if self.return_optimal_trajectory:
      optimal_traj = self.backend.zeros((self.T, self.system.dim_x), dtype=self.stype)
      x = x0
      for t in range(0, self.T):  # loop for time step t = 0 ~ T-1
        x = self.integrate(x, self.clamp_input(u[t]))
        optimal_traj[t] = x

    sampled_traj_list = None
    if self.return_sampled_trajectories:
      sampled_traj_list = self.backend.zeros((self.K, self.T, self.system.dim_x), dtype=self.stype)
      sorted_idx = self.backend.argsort(self.S)  # sort samples by state cost, 0th is the best sample
      for k in sorted_idx:
        x = x0
        for t in range(0, self.T):  # loop for time step t = 0 ~ T-1
          x = self.integrate(x, self.clamp_input(v[k, t]))
          sampled_traj_list[k, t] = x

    self.u_prev[:-1] = u[1:]
    self.u_prev[-1] = u[-1]

    if self.backend == cp:
      return (
        u[0].get(),
        u.get(),
        optimal_traj.get() if optimal_traj is not None else None,
        sampled_traj_list.get() if sampled_traj_list is not None else None,
      )

    if self.backend == np:
      return u[0], u, optimal_traj, sampled_traj_list

  def calculate_epsilon(self, sigma: np.ndarray, size_sample: int, size_time_step: int, size_dim_u: int):
    mu = self.backend.zeros((size_dim_u))
    epsilon = self.backend.random.multivariate_normal(mu, sigma, (size_sample, size_time_step))
    return epsilon

  def clamp_input(self, v: np.ndarray):
    v = self.backend.clip(v, -self.max_abs_velocity, self.max_abs_velocity)
    return v

  def integrate(self, x_t: np.ndarray, u_t: np.ndarray):
    return self.system.integrate(x_t, u_t, self.dt)

  def compute_weights(self, S: np.ndarray):
    rho = S.min()
    exp_terms = self.backend.exp((-1.0 / self.lmbda) * (S - rho))
    eta = self.backend.sum(exp_terms)
    w = (1.0 / eta) * exp_terms
    return w

  def moving_average_filter(self, xx: np.ndarray, window_size: int):
    b = self.backend.ones(window_size) / window_size
    xx_mean = self.backend.zeros_like(xx)

    xx_mean = self.backend.apply_along_axis(lambda x: self.backend.convolve(x, b, mode="same"), axis=0, arr=xx)

    n_conv = math.ceil(window_size / 2)

    xx_mean[0, :] *= window_size / n_conv

    for i in range(1, n_conv):
      scale_start = window_size / (i + n_conv)
      scale_end = window_size / (i + n_conv - (window_size % 2))

      xx_mean[i, :] *= scale_start
      xx_mean[-i, :] *= scale_end

    return xx_mean
