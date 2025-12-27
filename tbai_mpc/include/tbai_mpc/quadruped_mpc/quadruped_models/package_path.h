#pragma once

#include <string>

namespace anymal {

/**
 * @brief Get the path to the tbai_mpc package resources directory.
 * @return Path string to the package resources directory.
 */
inline std::string getPath() {
  // This should be configured based on installation location
  // For now, return the source directory path which will be set during CMake configuration
  return TBAI_MPC_RESOURCE_PATH;
}

}  // namespace anymal
