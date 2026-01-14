//
// Created by rgrandia on 27.11.20.
//

#pragma once

#include "tbai_mpc/quadruped_mpc/core/SwitchedModel.h"
#include "tbai_mpc/quadruped_mpc/terrain/TerrainPlane.h"

namespace tbai::mpc::quadruped {

NormalAndPosition estimatePlane(const std::vector<vector3_t>& regressionPoints);

}  // namespace tbai::mpc::quadruped
