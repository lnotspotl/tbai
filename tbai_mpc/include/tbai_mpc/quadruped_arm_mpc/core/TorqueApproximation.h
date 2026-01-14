//
// Created by rgrandia on 08.12.21.
//

#pragma once

#include "tbai_mpc/quadruped_arm_mpc/core/KinematicsModelBase.h"
#include "tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h"

namespace tbai::mpc::quadruped_arm {

/**
 * Approximate leg joint torques with J(q)^T F, i.e. neglecting leg dynamics.
 * Only computes torques for the 12 leg joints (not arm joints).
 */
template <typename SCALAR_T>
leg_joint_coordinate_s_t<SCALAR_T> torqueApproximation(const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
                                                        const feet_array_t<vector3_s_t<SCALAR_T>>& contactForcesInBase,
                                                        const KinematicsModelBase<SCALAR_T>& kinematics);

// Explicit instantiations
extern template leg_joint_coordinate_s_t<scalar_t> torqueApproximation<scalar_t>(
    const leg_joint_coordinate_s_t<scalar_t>& legJointPositions,
    const feet_array_t<vector3_s_t<scalar_t>>& contactForcesInBase,
    const KinematicsModelBase<scalar_t>& kinematics);
extern template leg_joint_coordinate_s_t<ad_scalar_t> torqueApproximation<ad_scalar_t>(
    const leg_joint_coordinate_s_t<ad_scalar_t>& legJointPositions,
    const feet_array_t<vector3_s_t<ad_scalar_t>>& contactForcesInBase,
    const KinematicsModelBase<ad_scalar_t>& kinematics);

}  // namespace tbai::mpc::quadruped_arm