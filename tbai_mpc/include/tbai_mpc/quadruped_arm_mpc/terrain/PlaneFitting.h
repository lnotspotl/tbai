//
// Created by rgrandia on 27.11.20.
//

#pragma once

#include "tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h"
#include "tbai_mpc/quadruped_arm_mpc/terrain/TerrainPlane.h"

namespace tbai::mpc::quadruped_arm {

NormalAndPosition estimatePlane(const std::vector<vector3_t>& regressionPoints);

}  // namespace tbai::mpc::quadruped_arm
