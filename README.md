# Towards Better Athletic Intelligence

[![Tests](https://github.com/lnotspotl/tbai/actions/workflows/tests.yml/badge.svg)](https://github.com/lnotspotl/tbai/actions/workflows/tests.yml)

This repository contains implementations of core algorithms used in the `tbai` ecosystems. For deployment and use with specific robotics frameworks, including ROS and ROS2, thin wrapper repositories are available: [tbai_ros](https://github.com/lnotspotl/tbai_ros) and [tbai_ros2](https://github.com/tbai-lab/tbai_ros2)

- [**tbai_ros**](https://github.com/lnotspotl/tbai_ros) - a ROS-noetic wrapper around tbai, uses [pixi](pixi.sh) for dependency management, so no worries that ROS is past its end of life :) - works on many Ubuntu releases, including Ubuntu 24.04
- [**tbai_ros2**](https://github.com/tbai-lab/tbai_ros2) - a ROS2-jazzy wrapper around tbai, uses [pixi](pixi.sh) for dependency management - works on many Ubuntu releases, including Ubuntu 20.04, Ubuntu 22.04 and Ubuntu 24.04

#### Install `tbai`
```bash
pixi install && pixi shell
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release
make -j10
```

#### Environment Variables used throughout `tbai`

| Environment Variable | Type | Default | Description | Usage |
|---------------------|------|---------|-------------|-------|
| **`TBAI_LOG_LEVEL`** | `string` | `"info"` | Logging level | Log verbosity. Values: `"trace"`, `"debug"`, `"info"`, `"warn"`, `"error"`, `"critical"` |
| **`TBAI_LOG_FOLDER`** | `string` | `""` (empty) | Log file directory. No file logging if empty | Logs saved here if set |
| **`TBAI_LOG_TO_CONSOLE`** | `bool` | `true` | Log to console | Show logs in terminal |
| **`TBAI_GLOBAL_CONFIG_PATH`** | `string` | **Required** | Global config file path | Main YAML config file |
| **`TBAI_ROBOT_DESCRIPTION_PATH`** | `string` | **Required** | Robot URDF path | Robot model file |
| **`TBAI_CACHE_DIR`** | `string` | `"/tmp/tbai_hf_cache"` | Model cache directory | For downloaded models |
