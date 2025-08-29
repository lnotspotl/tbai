#!/usr/bin/env python3

import numpy as np
import sympy as sp
import casadi as ca
import itertools
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.pyplot as plt

from tbai_safe.utils import get_ellipse_points
from collections import OrderedDict, deque


class ControlBarrierFunctionNew:
  def __init__(self, h_expr, factory, aliases={}, subs={}):
    self.factory = factory
    self.h_expr = h_expr
    self.aliases = aliases
    self.other_subs = {}
    self.substitute(**subs)

  def get_subs_dict(self, *args, **kwargs):
    subs = self.other_subs

    # Substitute from other CBFs
    for item in args:
      if isinstance(item, ControlBarrierFunctionNew):
        subs = subs | self.get_subs_dict(**item.other_subs)

    # Substitute from symbols
    free_symbol_names = [s.name for s in self.h_expr.free_symbols]
    for key, value in kwargs.items():
      if isinstance(key, sp.Symbol):
        key = key.name
      if key in self.aliases:
        key = self.aliases[key]
      if key in free_symbol_names:
        subs[key] = value
    return subs

  def substitute(self, *args, **kwargs):
    self.other_subs = self.get_subs_dict(*args, **kwargs)
    return self

  def get_expr(self, substitute=True):
    if substitute:
      return self.h_expr.subs(self.other_subs)
    return self.h_expr

  def get_grad(self, substitute=True):
    h_expr = self.get_expr(substitute=substitute)
    dh_dx = sp.diff(h_expr, self.factory.x)
    dh_dy = sp.diff(h_expr, self.factory.y)
    return sp.Matrix([dh_dx, dh_dy])

  def evaluate(self, x, y):
    expr = self.get_expr(substitute=False)
    free_symbols = [s.name for s in expr.free_symbols]
    assert self.factory.x.name in free_symbols, f"{self.factory.x.name} must be a free symbol"
    assert self.factory.y.name in free_symbols, f"{self.factory.y.name} must be a free symbol"
    fn = sp.lambdify(free_symbols, expr, "numpy")
    if isinstance(x, np.ndarray):
      assert x.shape == y.shape, f"x and y must have the same shape! Found shapes {x.shape} and {y.shape}"
      return fn(**{self.factory.x.name: x.reshape(-1), self.factory.y.name: y.reshape(-1)}, **self.other_subs).reshape(
        x.shape
      )
    else:
      return fn(**{self.factory.x.name: x, self.factory.y.name: y}, **self.other_subs)

  def shift_xy(self, x_shift, y_shift):
    x_temp, y_temp = self.factory.get_unique_symbols("x_temp, y_temp", add=False)
    x, y = self.factory.x, self.factory.y
    subs = {x: x_temp, y: y_temp}
    self.h_expr = self.h_expr.subs(subs)

    x_new = x - x_shift  # minus because we want to shift "back" to the original orientation
    y_new = y - y_shift  # minus because we want to shift "back" to the original orientation
    subs = {x_temp: x_new, y_temp: y_new}
    self.h_expr = self.h_expr.subs(subs)
    return self

  def rotate(self, angle):
    x_temp, y_temp = self.factory.get_unique_symbols("x_temp, y_temp", add=False)
    x, y = self.factory.x, self.factory.y
    subs = {x: x_temp, y: y_temp}
    self.h_expr = self.h_expr.subs(subs)

    ca, sa = sp.cos(-angle), sp.sin(-angle)  # minus because we want to rotate "back" to the original orientation
    x_new = x * ca - y * sa
    y_new = x * sa + y * ca
    subs = {x_temp: x_new, y_temp: y_new}
    self.h_expr = self.h_expr.subs(subs)
    return self

  def __mul__(self, other):
    return ControlBarrierFunctionNew(self.h_expr * other, self.factory, aliases=self.aliases, subs=self.other_subs)

  def __rmul__(self, other):
    return self.__mul__(other)


def visualize_cbfs(cbfs, ax, granularity=100, unsafe_colors=["red"], safe_color="white", pcm=None, alpha=1.0):
  x_lims, y_lims = ax.get_xlim(), ax.get_ylim()
  x_coords = np.linspace(x_lims[0], x_lims[1], granularity)
  y_coords = np.linspace(y_lims[0], y_lims[1], granularity)
  x_grid, y_grid = np.meshgrid(x_coords, y_coords)
  cbfs = cbfs if isinstance(cbfs, list) else [cbfs]
  unsafe_colors = unsafe_colors if len(cbfs) == len(unsafe_colors) else [unsafe_colors] * len(cbfs)
  unique_unsafe_colors = sorted(list(set(unsafe_colors)))
  colors = [(0, safe_color)] + [
    (1 / len(unique_unsafe_colors) * (idx + 1), uc) for idx, uc in enumerate(unique_unsafe_colors)
  ]
  c2f = {color: f for (f, color) in colors}
  cmap = LinearSegmentedColormap.from_list("custom_cmap", colors)
  num_visits = np.zeros_like(x_grid, dtype=np.int32)
  z_grid = np.zeros_like(x_grid, dtype=np.float64)
  for cbf, unsafe_color in zip(cbfs, unsafe_colors):
    grid = cbf.evaluate(x_grid, y_grid)
    grid_unsafe = grid <= 0
    grid_color = c2f[unsafe_color] * grid_unsafe
    z_grid += grid_color
    num_visits += np.where(grid_unsafe, 1, 0)
  num_visits = np.where(num_visits > 0, num_visits, 1)
  z_grid = z_grid / num_visits
  if pcm is not None:
    pcm.set_array(z_grid.ravel())
    pcm.set_cmap(cmap)
    return pcm
  pcm = ax.pcolormesh(x_grid, y_grid, z_grid, cmap=cmap, alpha=alpha)
  return pcm


class ControlBarrierFunctionFactory:
  def __init__(self):
    self.x, self.y, self.kappa = sp.symbols("x, y, kappa")
    self.u1, self.u2 = sp.symbols("u1, u2")
    self.all_symbols = set([self.x.name, self.y.name, self.kappa.name, self.u1.name, self.u2.name])
    self.all_symbols_copy = None

  def disable_symbol_tracking(self):
    assert self.all_symbols_copy is None, "Symbol tracking must be enabled before disabling"
    self.all_symbols_copy = self.all_symbols.copy()

  def enable_symbol_tracking(self):
    assert self.all_symbols_copy is not None, "Symbol tracking must be disabled before enabling"
    self.all_symbols = self.all_symbols_copy.copy()
    self.all_symbols_copy = None

  def get_unique_symbol(self, name: str, add: bool = True) -> sp.Symbol:
    for idx in itertools.count(start=1):
      candidate = f"{name}_{idx}"
      if candidate not in self.all_symbols:
        if add:
          self.all_symbols.add(candidate)
        return sp.Symbol(candidate)

  def get_unique_symbols(self, names: str, add: bool = True):
    return [self.get_unique_symbol(n.strip(), add=add) for n in names.split(",")]

  def get_sphere(self):
    center_x, center_y, radius = self.get_unique_symbols("c_x, c_y, r")
    h_expr = (self.x - center_x) ** 2 + (self.y - center_y) ** 2 - radius**2
    aliases = dict(c_x=center_x.name, c_y=center_y.name, r=radius.name)
    return ControlBarrierFunctionNew(h_expr, self, aliases=aliases)

  def get_halfplane(self):
    point_x, point_y, normal_x, normal_y = self.get_unique_symbols("p_x, p_y, n_x, n_y")
    normal_expr = normal_x * (self.x - point_x) + normal_y * (self.y - point_y)
    aliases = dict(p_x=point_x.name, p_y=point_y.name, n_x=normal_x.name, n_y=normal_y.name)
    return ControlBarrierFunctionNew(normal_expr, self, aliases=aliases)

  def get_rectangle(self, method="exact"):
    center_x, center_y, width, height = self.get_unique_symbols("c_x, c_y, w, h")
    center = np.array([center_x, center_y])
    n1, n2, n3, n4 = (1, 0), (-1, 0), (0, 1), (0, -1)
    p1 = center + np.array([width / 2, 0])
    p2 = center + np.array([-width / 2, 0])
    p3 = center + np.array([0, height / 2])
    p4 = center + np.array([0, -height / 2])
    self.disable_symbol_tracking()
    h1 = self.get_halfplane().substitute(p_x=p1[0], p_y=p1[1], n_x=n1[0], n_y=n1[1])
    h2 = self.get_halfplane().substitute(p_x=p2[0], p_y=p2[1], n_x=n2[0], n_y=n2[1])
    h3 = self.get_halfplane().substitute(p_x=p3[0], p_y=p3[1], n_x=n3[0], n_y=n3[1])
    h4 = self.get_halfplane().substitute(p_x=p4[0], p_y=p4[1], n_x=n4[0], n_y=n4[1])
    self.enable_symbol_tracking()
    h1234 = self.intersection([h1, h2, h3, h4], method=method)
    aliases = dict(c_x=center_x.name, c_y=center_y.name, w=width.name, h=height.name)
    return ControlBarrierFunctionNew(h1234.get_expr(substitute=True), self, aliases=aliases)

  def union(self, cbfs, method="exact", substitute=False):
    if method not in ["exact", "approximate"]:
      raise ValueError(f"Invalid method: {method}")

    # Combine all substituted values
    combined_subs = dict(itertools.chain.from_iterable([cbf.other_subs.items() for cbf in cbfs]))

    if method == "exact":
      h_expr = sp.Min(*[cbf.get_expr(substitute=substitute) for cbf in cbfs])
      return ControlBarrierFunctionNew(h_expr, self, subs=combined_subs)

    if method == "approximate":
      exps = [sp.exp(-self.kappa * cbf.get_expr(substitute=substitute)) for cbf in cbfs]
      new_expr = -(1 / self.kappa) * sp.log(sum(exps))
      return ControlBarrierFunctionNew(new_expr, self, subs=combined_subs)

    raise ValueError(f"Invalid method: {method}")

  def intersection(self, cbfs, method="exact", substitute=False):
    if method not in ["exact", "approximate"]:
      raise ValueError(f"Invalid method: {method}")

    # Combine all substituted values
    combined_subs = dict(itertools.chain.from_iterable([cbf.other_subs.items() for cbf in cbfs]))

    if method == "exact":
      h_expr = sp.Max(*[cbf.get_expr(substitute=substitute) for cbf in cbfs])
      return ControlBarrierFunctionNew(h_expr, self, subs=combined_subs)

    if method == "approximate":
      exps = [sp.exp(self.kappa * cbf.get_expr(substitute=substitute)) for cbf in cbfs]
      new_expr = (1 / self.kappa) * sp.log(sum(exps))
      return ControlBarrierFunctionNew(new_expr, self, subs=combined_subs)

    raise ValueError(f"Invalid method: {method}")


class CasadiControlBarrierFunction:
  def __init__(self, cbf: ControlBarrierFunctionNew):
    self.cbf = cbf

  def get_h(self, substitute=True):
    expr = self.cbf.get_expr(substitute=substitute)
    expr_fn = sp.lambdify(list(expr.free_symbols), expr, modules="numpy")
    caargs = OrderedDict({s.name: ca.SX.sym(s.name) for s in expr.free_symbols})
    fn = ca.Function("h", caargs.values(), [expr_fn(**caargs)], caargs.keys(), ["h"])
    return lambda **kwargs: fn(**kwargs)["h"], {s.name for s in expr.free_symbols}

  def get_grad_h(self, substitute=True):
    expr = self.cbf.get_grad(substitute=substitute)
    expr_fn = sp.lambdify(list(expr.free_symbols), expr, modules="numpy")
    caargs = OrderedDict({s.name: ca.SX.sym(s.name) for s in expr.free_symbols})
    fn = ca.Function("grad_h", caargs.values(), [expr_fn(**caargs)], caargs.keys(), ["grad_h"])
    return lambda **kwargs: fn(**kwargs)["grad_h"], {s.name for s in expr.free_symbols}


class ControlBarrierFunctionMemory(ControlBarrierFunctionNew):
  def __init__(self, cbf: ControlBarrierFunctionNew, size: int = 2, method: str = "exact"):
    assert method in ["exact", "approximate"], f"Invalid method: {method}"

    # Keep track of factory
    self.cbf = cbf

    # Extract expression from CBF
    expr = cbf.get_expr(substitute=True)

    # These are the three symbols that are almost always present
    self.x_included = cbf.factory.x in expr.free_symbols and False
    self.y_included = cbf.factory.y in expr.free_symbols and False
    self.kappa_included = cbf.factory.kappa in expr.free_symbols

    # Keep track of all present symbols
    self.all_symbols: list[str] = list()
    if self.x_included:
      self.all_symbols.append(cbf.factory.x.name)
    if self.y_included:
      self.all_symbols.append(cbf.factory.y.name)
    if self.kappa_included:
      self.all_symbols.append(cbf.factory.kappa.name)

    # Initialize memory
    self.memory = deque(maxlen=size)

    cbfs = list()
    self.expr_symbols = sorted([s.name for s in expr.free_symbols - {cbf.factory.x, cbf.factory.y, cbf.factory.kappa}])

    # Generate all unique symbols
    for _ in range(size):
      syms = self.cbf.factory.get_unique_symbols(",".join(self.expr_symbols))
      self.all_symbols.extend([s.name for s in syms])
      new_expr = expr.subs({s: sym for s, sym in zip(self.expr_symbols, syms)})
      new_cbf = ControlBarrierFunctionNew(new_expr, factory=self.cbf.factory)
      cbfs.append(new_cbf)
    self.memory_cbf = self.cbf.factory.union(cbfs, method=method)
    self.expr = self.memory_cbf.get_expr(substitute=True)

    # Initialize parent class
    super().__init__(self.expr, factory=self.cbf.factory)

  def update_subs(self, kappa=10):
    """Update other_subs for all symbols in the memory"""
    args = self.get_args(x=10, y=10, kappa=kappa)  # dummy values
    for symbol, val in zip(self.all_symbols, args):
      if symbol in [self.cbf.factory.x.name, self.cbf.factory.y.name]:
        continue
      self.other_subs[symbol] = val
    return self

  def memorize(self, cbf: ControlBarrierFunctionNew):
    """Memorize a new CBF by storing its substituted values in the memory. This function does not update the other_subs. That is done in update_subs."""
    vals = list()
    for symbol in self.expr_symbols:
      vals.append(cbf.other_subs[symbol])
    self.memory.append(vals)
    return self

  def get_args(self, **kwargs):
    assert len(self.memory) > 0, "Nothing in the memory yet!"
    out = list()
    if self.x_included:
      out.append(kwargs[self.cbf.factory.x.name])
    if self.y_included:
      out.append(kwargs[self.cbf.factory.y.name])
    if self.kappa_included:
      out.append(kwargs[self.cbf.factory.kappa.name])

    temp = (len(self.all_symbols) - self.x_included - self.y_included - self.kappa_included) // len(self.memory[0])
    for i in range(temp):
      vals = self.memory[i] if i < len(self.memory) else self.memory[-1]
      out.extend(vals)
    return np.array(out)


class ControlBarrierFunctionMemoryEnvelope(ControlBarrierFunctionMemory):
  def __init__(
    self, cbf: ControlBarrierFunctionNew, size: int = 2, method: str = "exact", P: int = 10, scaler: float = 1.0
  ):
    super().__init__(cbf, size, method)
    x_symbols, y_symbols = self.turn_to_envelope(P)
    self.all_symbols_envelope = self.all_symbols + x_symbols + y_symbols
    self.expr_envelope = self.memory_cbf.get_expr(substitute=False)
    self.P = P
    self.scaler = scaler

  def get_args(self, **kwargs):
    out = super().get_args(**kwargs)
    x, y = self.get_envelope_points(self.P, self.scaler)
    out = np.concatenate([out, x, y])
    return out

  def turn_to_envelope(self, N):
    factory = self.factory

    x_symbols, y_symbols = list(), list()

    final_expr = sp.Min(0.5 * self.memory_cbf.h_expr, 0)
    for i in range(N):
      shift_x, shift_y = factory.get_unique_symbols("shift_x, shift_y")
      x_symbols.append(shift_x.name)
      y_symbols.append(shift_y.name)
      restored_expr = self.memory_cbf.get_expr(substitute=False)
      self.memory_cbf.shift_xy(-shift_x, -shift_y)
      new_expr = self.memory_cbf.h_expr
      self.memory_cbf.h_expr = restored_expr
      final_expr += sp.Min((0.5 / N) * new_expr, 0)
    self.memory_cbf.h_expr = final_expr
    return x_symbols, y_symbols

  def get_cov(self, symbols=None):
    symbols = symbols if symbols is not None else self.expr_symbols
    assert len(symbols) > 0, "No symbols to compute covariance for!"
    data = list()
    for vals in self.memory:
      point = list()
      for symbol in symbols:
        idx = self.expr_symbols.index(symbol)
        point.append(vals[idx])
      data.append(point)
    data = np.array(data)
    return np.cov(data, rowvar=False)

  def get_mean(self, symbols=None):
    symbols = symbols if symbols is not None else self.expr_symbols
    assert len(symbols) > 0, "No symbols to compute mean for!"
    data = list()
    for vals in self.memory:
      point = list()
      for symbol in symbols:
        idx = self.expr_symbols.index(symbol)
        point.append(vals[idx])
      data.append(point)
    data = np.array(data)
    return np.mean(data, axis=0)

  def get_envelope_points(self, P=20, scaler=1.0):
    mean = self.get_mean()
    cov = self.get_cov()
    return get_ellipse_points(mean * 0, scaler * cov, P)


if __name__ == "__main__":
  factory = ControlBarrierFunctionFactory()
  cbf = factory.get_sphere()
  N = 40
  cbf_mem = ControlBarrierFunctionMemory(cbf, size=N, method="exact")
  scaling = 0.01
  for _ in range(N):
    cx = np.random.uniform(-0.1, 0.1) * scaling
    cy = np.random.uniform(-0.1, 0.1) * scaling
    # cy = np.random.uniform(-1, 1) * 1 + cx
    r = np.random.uniform(0, 10)
    cbf_mem.memorize(cbf.substitute(c_x=cx, c_y=cy, r=r))
  print(cbf_mem.get_cov(symbols=["c_x_1", "c_y_1"]))

  fig, ax = plt.subplots()
  ax.set_xlim(-10, 10)
  ax.set_ylim(-10, 10)

  (ellipse_pts,) = ax.plot([], [], "r--", color="red", alpha=0.5)

  from tbai_safe.utils import get_ellipse_points

  def update(i):
    global cbf_mem, N
    for _ in range(1):
      cx = np.random.uniform(-0.1, 0.1) * scaling
      cy = np.random.uniform(-0.1, 0.1) * scaling

      # cy = np.random.uniform(-1, 1) * 1 + cx
      r = np.random.uniform(0, 10)
      cbf_mem.memorize(cbf.substitute(c_x=cx, c_y=cy, r=r))
    mean, cov = cbf_mem.get_mean(symbols=["c_x_1", "c_y_1"]), cbf_mem.get_cov(symbols=["c_x_1", "c_y_1"])
    x, y = get_ellipse_points(mean, 100 * cov)
    ellipse_pts.set_data(x, y)

  from matplotlib.animation import FuncAnimation

  anim = FuncAnimation(fig, update, interval=100)
  plt.show()
