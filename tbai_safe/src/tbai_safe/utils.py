#!/usr/bin/env python3

import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import seaborn as sns


def remove_outliers(data: list, lower_percentile: float = 5, upper_percentile: float = 95):
  """Remove outliers from a list based on the lower and upper percentiles.

  Args:
      data: The list of data to remove outliers from.
      lower_percentile: The lower percentile to use for the outliers. Defaults to 5.
      upper_percentile: The upper percentile to use for the outliers. Defaults to 95.

  Returns:
      The list of data with outliers removed.
  """
  assert lower_percentile >= 0 and lower_percentile <= 100, "Lower percentile must be between 0 and 100"
  assert upper_percentile >= 0 and upper_percentile <= 100, "Upper percentile must be between 0 and 100"
  assert lower_percentile <= upper_percentile, "Lower percentile must be less than upper percentile"

  lower_bound = np.percentile(data, lower_percentile)
  upper_bound = np.percentile(data, upper_percentile)
  return [t for t in data if (t > lower_bound and t < upper_bound)]


def plot_histogram(
  fig,
  ax,
  data: list[float],
  title: str,
  x_label: str,
  y_label: str = "Frequency",
  bins: int = 100,
  store_fig: bool = False,
):
  ax.hist(data, bins=bins)
  ax.set_title(title)
  ax.set_xlabel(x_label)
  ax.set_ylabel(y_label)
  if store_fig:
    fig.savefig(f"{title}.png", dpi=300)
  return ax


class PerformanceTimer:
  def __init__(self, unit: str = "s"):
    self.start_time = None
    self.stop_time = None
    self.execution_times = list()

    # multiplier for the unit
    multipliers = dict(s=1.0, ms=1e3, us=1e6, ns=1e9)
    assert unit in multipliers, f"Unit must be one of {list(multipliers.keys())}"
    self.unit = unit
    self.multiplier = multipliers[unit]

  def __str__(self):
    return f"PerformanceTimer(unit={self.unit}, num_samples={len(self.execution_times)})"

  def __repr__(self):
    return str(self)

  def tick(self, message: str = ""):
    self.start_time = time.perf_counter()
    if message:
      print(f"{message}...")

  def tock(self, message: str = ""):
    self.stop_time = time.perf_counter()
    self.elapsed = (self.stop_time - self.start_time) * self.multiplier
    self.execution_times.append(self.elapsed)
    if message:
      print(f"{message}...")

  @property
  def mean(self):
    return np.mean(self.execution_times)

  @property
  def std(self):
    return np.std(self.execution_times)

  @property
  def min(self):
    return np.min(self.execution_times)

  @property
  def max(self):
    return np.max(self.execution_times)

  @property
  def median(self):
    return np.median(self.execution_times)

  def remove_outliers_(self, lower_percentile: float = 5, upper_percentile: float = 95):
    self.execution_times = remove_outliers(self.execution_times, lower_percentile, upper_percentile)
    return self

  def get_data(self):
    return self.execution_times


def draw_ellipse(ax, mean, cov, color="red", alpha=0.5):
  eig_vals, eig_vecs = np.linalg.eig(cov)
  angle = np.arctan2(eig_vecs[1, :], eig_vecs[0, :])
  width, height = 2 * np.sqrt(eig_vals)
  import matplotlib.patches as patches

  ax.add_patch(patches.Ellipse(mean, width, height, angle=np.degrees(angle[0]), color=color, alpha=alpha))


def get_ellipse_points(mean, cov, num_points=100):
  eig_vals, eig_vecs = np.linalg.eig(cov)
  angle = np.arctan2(eig_vecs[1, :][0], eig_vecs[0, :][0])
  ca, sa = np.cos(angle), np.sin(angle)
  R = np.array([[ca, -sa], [sa, ca]])
  width, height = 2 * np.sqrt(np.abs(eig_vals))
  theta = np.linspace(0, 2 * np.pi, num_points)
  x = mean[0] + width * np.cos(theta)
  y = mean[1] + height * np.sin(theta)
  x, y = R @ np.vstack([x, y])
  return x, y


class Rate:
  def __init__(self, rate: float):
    self.iteration_time = 1 / rate
    self.last_time = time.perf_counter()

  def sleep(self):
    current_time = time.perf_counter()
    time_to_sleep = self.iteration_time - (current_time - self.last_time)
    if time_to_sleep > 0:
      time.sleep(time_to_sleep)
    self.last_time = time.perf_counter()


def is_notebook():
  return hasattr(__builtins__, "__IPYTHON__")


if is_notebook():
  from IPython import display


class Plotter:
  def __init__(self, fig, ax, maxsize=None):
    self.x_data = deque(maxlen=maxsize) if maxsize is not None else list()
    self.y_data = deque(maxlen=maxsize) if maxsize is not None else list()

    self.fig, self.ax = fig, ax
    (self.line,) = self.ax.plot(self.x_data, self.y_data)

  def update(self, x, y):
    self.x_data.append(x)
    self.y_data.append(y)
    self.line.set_data(self.x_data, self.y_data)
    self.ax.relim()
    self.ax.autoscale_view()
    self.fig.canvas.draw()

    if is_notebook():
      display.clear_output(wait=True)
      display.display(self.fig)

  def save(self, filename: str):
    self.fig.savefig(filename, dpi=300)


class MultiPlotter:
  def __init__(
    self, rows: int, cols: int, maxsize: int = None, names: list[str] = None, figsize: tuple[int, int] = (10, 10)
  ):
    sns.set_theme(style="darkgrid")
    self.fig, self.axes = plt.subplots(rows, cols, figsize=figsize)
    self.plots = dict()
    names = names or [f"Plot {i}" for i in range(rows * cols)]
    for name, ax in zip(names, self.axes.flatten()):
      self.plots[name] = Plotter(self.fig, ax, maxsize)

  def update(self, name: str, x: float, y: float):
    self.plots[name].update(x, y)

  def show(self):
    plt.show()

  def save(self, filename: str):
    self.fig.savefig(filename, dpi=300)


if __name__ == "__main__":
  plotter = MultiPlotter(rows=1, cols=2, names=["Plot 1", "Plot 2"], figsize=(20, 5))
  for i in range(50):
    plotter.update("Plot 1", i, i + np.random.randn())
    plotter.update("Plot 2", i, i + np.random.randn())
    plt.pause(0.01)
  plotter.save("plot.png")
  plotter.show()
