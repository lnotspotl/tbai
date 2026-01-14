#pragma once

#include <string>
#include <vector>

namespace tbai::mpc::arm {

struct ArmModelInfo {
  size_t stateDim;
  size_t inputDim;
  size_t armDim;
  std::string baseFrame;
  std::string eeFrame;
  std::vector<std::string> dofNames;
};

}  // namespace tbai::mpc::arm
