#!/usr/bin/env python3

import numpy as np
import functools
import atexit
import imageio
import matplotlib.pyplot as plt
from typing import Callable

from tbai_safe.logging import get_logger

logger = get_logger("anim")


def anim_callback(fig: plt.Figure, ax: plt.Axes, callback: Callable[[np.ndarray], None]):
  """
  Decorator to call the given callback with the current frame of the animation.
  """

  def wrapper(func):
    @functools.wraps(func)
    def _internal(*args, **kwargs):
      ret = func(*args, **kwargs)
      frame = np.array(fig.canvas.copy_from_bbox(ax.bbox))[:, :, :3]  # Remove the alpha channel: RGBA -> RGB
      callback(frame)
      return ret

    return _internal

  return wrapper


def save_animation(
  fig: plt.Figure,
  ax: plt.Axes,
  filename: str = "animation.gif",
  fps: float = 20.0,
  repeat: bool = True,
  include_all: bool = False,
):
  """Save a matplotlib animation at the end of the program.

  Args:
    fig: The matplotlib figure to save.
    ax: The matplotlib axes to save.
    filename: The filename of the animation to save. Defaults to 'animation.gif'.
    fps: The frames per second of the animation. Defaults to 20.0.
    repeat: Whether the animation should repeat. Only used for GIFs. Defaults to True.
  """

  # Supported formats and their options
  options = dict()
  options["gif"] = dict(fps=fps, loop=0 if repeat else 1)  # 0 means loop indefinitely, 1 means play once
  options["mp4"] = dict(fps=fps, codec="h264")
  options["webm"] = dict(fps=fps, codec="libvpx")

  # Extract the file extension
  extension = filename.split(".")[-1]

  # Make sure the file format is supported
  if extension not in options:
    raise ValueError(f"Unsupported file format {extension} ({filename}). Supported formats: {list(options.keys())}")

  # List of frames to save
  frames = list()

  # Decorator to save the frame
  def wrapper(func):
    @functools.wraps(func)
    def _internal(*args, **kwargs):
      ret = func(*args, **kwargs)
      if include_all:
        frame = np.array(fig.canvas.copy_from_bbox(ax.figure.bbox))
      else:
        frame = np.array(fig.canvas.copy_from_bbox(ax.bbox))
      frame = frame[:, :, :3]  # Remove the alpha channel: RGBA -> RGB
      frames.append(frame)
      return ret

    return _internal

  # Save animation callback function
  def save():
    nonlocal frames
    if len(frames) >= 2:
      if frames[0].shape != frames[1].shape:
        logger.warning(
          f"First frame has different shape than subsequent frames: {frames[0].shape} != {frames[1].shape}"
        )
        frames = frames[1:]
      logger.info(f"Saving animation to {filename} with {len(frames)} frames. Options: {options[extension]}")
      imageio.mimsave(filename, frames, **options[extension])
    else:
      logger.warning("Not enough frames to save animation")

  # Register the save animation callback function to run at program exit
  atexit.register(save)

  return wrapper


if __name__ == "__main__":
  import matplotlib.pyplot as plt
  from matplotlib.animation import FuncAnimation
  import numpy as np

  fig, ax = plt.subplots()
  ax.set_xlim(0, 2 * np.pi)
  ax.set_ylim(-1, 1)

  (tt,) = plt.plot([], [])

  @save_animation(fig, ax, filename="animation.gif", fps=1, repeat=True)
  def update(i):
    tt.set_data(np.linspace(0, 2 * np.pi, 100), np.sin(np.linspace(0, 2 * np.pi, 100)) + np.random.randn(100) * 0.1)

  anim = FuncAnimation(fig, update, frames=100, interval=100)
  plt.show()
