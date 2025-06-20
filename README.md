# tbai2

## Environment Variables Reference

Based on the codebase analysis, here are all the TBAI environment variables that can be configured:

| Environment Variable | Type | Default | Description | Usage |
|---------------------|------|---------|-------------|-------|
| **`TBAI_LOG_LEVEL`** | `string` | `"info"` | Sets the logging level for all TBAI components | Controls verbosity of log output. Valid values: `"trace"`, `"debug"`, `"info"`, `"warn"`, `"error"`, `"critical"` |
| **`TBAI_LOG_FOLDER`** | `string` | `""` (empty) | Directory where log files will be stored | If set, logs will be written to files in this directory. Each logger gets its own file (e.g., `bob_controller.logs`) |
| **`TBAI_LOG_TO_CONSOLE`** | `bool` | `true` | Whether to output logs to console | Controls if logs appear in terminal output |
| **`TBAI_GLOBAL_CONFIG_PATH`** | `string` | **Required** | Path to the global YAML configuration file | Points to the main configuration file containing all TBAI settings (robot parameters, controller settings, etc.) |
| **`TBAI_ROBOT_DESCRIPTION_PATH`** | `string` | **Required** | Path to the robot URDF description file | Used by BobController to load robot model and kinematics |
| **`TBAI_CACHE_DIR`** | `string` | `"/tmp/tbai_hf_cache"` | Directory for caching downloaded models | Used by HuggingFace model download functionality |

## Usage Examples

### Basic Setup
```bash
# Required environment variables
export TBAI_GLOBAL_CONFIG_PATH="/path/to/config.yaml"
export TBAI_ROBOT_DESCRIPTION_PATH="/path/to/robot.urdf"

# Optional logging configuration
export TBAI_LOG_LEVEL="debug"
export TBAI_LOG_FOLDER="/tmp/tbai_logs"
export TBAI_LOG_TO_CONSOLE=true

# Optional cache configuration
export TBAI_CACHE_DIR="/home/user/.cache/tbai"
```

### Logging Configuration Examples

**Console-only logging:**
```bash
export TBAI_LOG_LEVEL="info"
export TBAI_LOG_FOLDER=""  # Empty = no file logging
export TBAI_LOG_TO_CONSOLE=true
```

**File-only logging:**
```bash
export TBAI_LOG_LEVEL="debug"
export TBAI_LOG_FOLDER="/var/log/tbai"
export TBAI_LOG_TO_CONSOLE=false
```

**Both console and file logging:**
```bash
export TBAI_LOG_LEVEL="warn"
export TBAI_LOG_FOLDER="/tmp/tbai_logs"
export TBAI_LOG_TO_CONSOLE=true
```

## Notes

1. **`TBAI_GLOBAL_CONFIG_PATH`** and **`TBAI_ROBOT_DESCRIPTION_PATH`** are **required** - the application will throw an error if they're not set.

2. **Logging variables** have sensible defaults and are optional.

3. **`TBAI_CACHE_DIR`** is used for caching HuggingFace models and has a default fallback.

4. The logging system supports both **global logging** (using `TBAI_GLOBAL_LOG_*` macros) and **per-logger logging** (using `TBAI_LOG_*` macros with a logger instance).

5. All environment variables are read using the `tbai::getEnvAs<T>()` function, which provides type safety and optional default values.

