#!/usr/bin/env python3

import copy
import logging
import os
from dataclasses import dataclass

from typing import Any


@dataclass
class _LoggingConfig:
  stdout_level: str = os.environ.get("LOG_LEVEL", "warn")
  file_level: str = os.environ.get("FILE_LOG_LEVEL", "debug")
  log_folder: str = os.environ.get("LOG_FOLDER", "")
  use_color: bool = os.environ.get("LOG_USE_COLOR", "true").lower() == "true"
  _initialized: bool = False

  def __post_init__(self):
    self.stdout_level = self.stdout_level.upper()
    self.file_level = self.file_level.upper()
    self._initialized = False

  def __setattr__(self, name: str, value: Any):
    if self._initialized:
      raise AttributeError("Logging config can be only set before any logger is created")
    super().__setattr__(name, value)


_logging_config = _LoggingConfig()


def set(attr: str, value: Any):
  setattr(_logging_config, attr, value)


def get_logger(
  name: str, stdout_level: str = None, file_level: str = None, log_folder: str = None, use_color: bool = None
) -> logging.Logger:
  # Load defaults if not provided
  stdout_level = stdout_level or _logging_config.stdout_level
  file_level = file_level or _logging_config.file_level
  log_folder = log_folder or _logging_config.log_folder
  use_color = use_color or _logging_config.use_color

  if not _logging_config._initialized:
    _logging_config._initialized = True

  log_to_file = bool(log_folder)

  logger = logging.getLogger(name)
  logger.setLevel(logging.DEBUG)
  logger.handlers.clear()

  assert name != "all", "Logger name cannot be 'all'"
  if log_to_file:
    os.makedirs(log_folder, exist_ok=True)

  COLORS = {
    "DEBUG": "\033[36m",  # Cyan
    "INFO": "\033[32m",  # Green
    "WARNING": "\033[33m",  # Yellow
    "ERROR": "\033[31m",  # Red
    "CRITICAL": "\033[35m",  # Magenta
    "RESET": "\033[0m",  # Reset
  }

  class FileFormatter(logging.Formatter):
    def format(self, record):
      record_copy = copy.copy(record)
      record_copy.levelname = record_copy.levelname.lower()
      return super().format(record_copy)

  class ConsoleFormatter(logging.Formatter):
    def __init__(self, *args, use_color: bool = True, **kwargs):
      super().__init__(*args, **kwargs)
      self.use_color = use_color

    def format(self, record):
      record_copy = copy.copy(record)
      original_levelname = record_copy.levelname
      record_copy.levelname = record_copy.levelname.lower()
      if self.use_color:
        color = COLORS.get(original_levelname, COLORS["RESET"])
        record_copy.levelname = f"{color}{record_copy.levelname}{COLORS['RESET']}"
      return super().format(record_copy)

  FORMAT = "[%(asctime)s.%(msecs)03d] [%(levelname)s] [%(name)s] %(message)s"
  DATEFMT = "%Y-%m-%d %H:%M:%S"

  # Log to console
  console_handler = logging.StreamHandler()
  console_handler.setFormatter(ConsoleFormatter(FORMAT, datefmt=DATEFMT, use_color=use_color))
  console_handler.setLevel(getattr(logging, stdout_level.upper(), logging.DEBUG))
  logger.addHandler(console_handler)

  # Log to file - each logger into its own file
  if log_to_file:
    file_handler = logging.FileHandler(
      os.path.join(log_folder, f"{name}.log"),
      encoding="utf-8",
      mode="a",
    )
    file_handler.setFormatter(FileFormatter(FORMAT, datefmt=DATEFMT))
    file_handler.setLevel(getattr(logging, file_level.upper(), logging.DEBUG))
    logger.addHandler(file_handler)

  # Log to file - all loggers into the same file
  if log_to_file:
    file_handler = logging.FileHandler(
      os.path.join(log_folder, "all.log"),
      encoding="utf-8",
      mode="a",
    )
    file_handler.setFormatter(FileFormatter(FORMAT, datefmt=DATEFMT))
    file_handler.setLevel(getattr(logging, file_level.upper(), logging.DEBUG))
    logger.addHandler(file_handler)

  return logger


__all__ = ["set", "get_logger"]
