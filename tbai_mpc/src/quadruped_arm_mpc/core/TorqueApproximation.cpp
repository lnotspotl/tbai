//
// Created by rgrandia on 08.12.21.
//

#include "tbai_mpc/quadruped_arm_mpc/core/TorqueApproximation.h"

namespace tbai::mpc::quadruped_arm {

template <typename SCALAR_T>
leg_joint_coordinate_s_t<SCALAR_T> torqueApproximation(const leg_joint_coordinate_s_t<SCALAR_T> &legJointPositions,
                                                        const feet_array_t<vector3_s_t<SCALAR_T>> &contactForcesInBase,
                                                        const KinematicsModelBase<SCALAR_T> &kinematics) {
    // Pad leg joint positions with zeros for arm joints (kinematics interface expects full joint vector)
    joint_coordinate_s_t<SCALAR_T> fullJointPositions;
    fullJointPositions.template head<LEG_JOINT_COORDINATE_SIZE>() = legJointPositions;
    fullJointPositions.template tail<NUM_ARM_JOINTS>().setZero();

    leg_joint_coordinate_s_t<SCALAR_T> torques;
    for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
        // Part of the foot Jacobian that corresponds to the joints in this leg.
        const auto b_baseToFootJacobianBlock = kinematics.baseToFootJacobianBlockInBaseFrame(leg, fullJointPositions);

        // tau = -J^T * F, (bottom 3 rows = translational part)
        torques.template segment<3>(3 * leg) =
            -b_baseToFootJacobianBlock.bottomRows(3).transpose() * contactForcesInBase[leg];
    }
    return torques;
}

template leg_joint_coordinate_s_t<scalar_t> torqueApproximation<scalar_t>(
    const leg_joint_coordinate_s_t<scalar_t> &legJointPositions,
    const feet_array_t<vector3_s_t<scalar_t>> &contactForcesInBase, const KinematicsModelBase<scalar_t> &kinematics);
template leg_joint_coordinate_s_t<ad_scalar_t> torqueApproximation<ad_scalar_t>(
    const leg_joint_coordinate_s_t<ad_scalar_t> &legJointPositions,
    const feet_array_t<vector3_s_t<ad_scalar_t>> &contactForcesInBase,
    const KinematicsModelBase<ad_scalar_t> &kinematics);

}  // namespace tbai::mpc::quadruped_arm