# Towards Better Athletic Intelligence

## Environment Variables used throughout `tbai`

| Environment Variable | Type | Default | Description | Usage |
|---------------------|------|---------|-------------|-------|
| **`TBAI_LOG_LEVEL`** | `string` | `"info"` | Logging level | Log verbosity. Values: `"trace"`, `"debug"`, `"info"`, `"warn"`, `"error"`, `"critical"` |
| **`TBAI_LOG_FOLDER`** | `string` | `""` (empty) | Log file directory. No file logging if empty | Logs saved here if set |
| **`TBAI_LOG_TO_CONSOLE`** | `bool` | `true` | Log to console | Show logs in terminal |
| **`TBAI_GLOBAL_CONFIG_PATH`** | `string` | **Required** | Global config file path | Main YAML config file |
| **`TBAI_ROBOT_DESCRIPTION_PATH`** | `string` | **Required** | Robot URDF path | Robot model file |
| **`TBAI_CACHE_DIR`** | `string` | `"/tmp/tbai_hf_cache"` | Model cache directory | For downloaded models |

## Combining logs
```bash
sort -t']' -k1 *.logs > combined.txt
```