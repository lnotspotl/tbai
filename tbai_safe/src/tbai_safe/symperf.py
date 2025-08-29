#! /usr/bin/env python3

import sympy as sp
import numpy as np
import numba

from tbai_safe.mppi import get_default_backend, get_default_dtype

try:
  import numba.cuda as nbcuda

  has_cuda = True
except ImportError:
  has_cuda = False


def jit_expr(
  expr: sp.Expr, nbtype=numba.float32, cse=True, parallel=False, symbols=None, device=True, backend=None, stype=None
):
  """Take an expression, convert it into a callable function and jit it. Works for both numpy and cuda backends."""
  backend = get_default_backend() if backend is None else backend
  stype = get_default_dtype() if stype is None else stype

  expr_symbols = [s.name for s in sorted(expr.free_symbols, key=lambda x: x.name)]
  symbols = symbols if symbols is not None else expr_symbols
  assert sorted(symbols) == expr_symbols, (
    f"Used symbols must be a permutation of the expression symbols: {symbols} != {expr_symbols}"
  )
  if isinstance(expr, sp.MatrixBase):  # For matrices, we let numba infer types
    fn = sp.lambdify(symbols, expr, modules="numpy", cse=cse)
    if backend == "numpy":
      return numba.jit(nopython=True, parallel=parallel)(fn)
    elif backend == "cuda":
      assert has_cuda, "Numba CUDA is not installed"
      return nbcuda.jit(nopython=True, device=device)(fn)
    else:
      raise ValueError(f"Invalid backend: {backend}, supported backends are: numpy, cuda")
  else:
    fn = sp.lambdify(symbols, expr, modules="math", cse=cse)
    annotation = nbtype(*[nbtype for _ in symbols])
    if backend == "numpy":
      return numba.jit([annotation], nopython=True, parallel=parallel)(fn)
    elif backend == "cuda":
      assert has_cuda, "Numba CUDA is not installed"
      return nbcuda.jit([annotation], nopython=True, device=device)(fn)
    else:
      raise ValueError(f"Invalid backend: {backend}, supported backends are: numpy, cuda")


def vectorize_expr(expr: sp.Expr, nbtype=numba.float32, target="cpu", cse=True, symbols=None):
  """Take an expression, convert it into a callable function and vectorize it. Works only for numpy backend."""
  expr_symbols = [s.name for s in sorted(expr.free_symbols, key=lambda x: x.name)]
  symbols = symbols if symbols is not None else expr_symbols
  assert sorted(symbols) == expr_symbols, (
    f"Used symbols must be a permutation of the expression symbols: {symbols} != {expr_symbols}"
  )
  fn = sp.lambdify(symbols, expr, modules="math", cse=cse)
  fn_annotation = nbtype(*[nbtype for _ in symbols])
  fn_vectorized = numba.vectorize([fn_annotation], nopython=True, target=target)(fn)
  return fn_vectorized


if __name__ == "__main__":
  xs, ys = sp.symbols("x, y")
  expr = sp.Min(sp.sqrt(xs**2 + ys**2), 1)
  fn_vectorized = vectorize_expr(expr, numba.float64)

  fn_jitted = jit_expr(expr, numba.float64)

  x = np.linspace(0, 1, 100)
  y = np.linspace(0, 1, 100)
  a = fn_vectorized(x, y)
  b = fn_jitted(x=0, y=0)
  print(a)
  print(b)
