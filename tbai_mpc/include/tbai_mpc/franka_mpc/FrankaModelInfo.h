#pragma once

#include <string>
#include <vector>

namespace ocs2 {
namespace franka {

struct FrankaModelInfo {
  size_t stateDim;
  size_t inputDim;
  size_t armDim;
  std::string baseFrame;
  std::string eeFrame;
  std::vector<std::string> dofNames;
};

}  // namespace franka
}  // namespace ocs2
