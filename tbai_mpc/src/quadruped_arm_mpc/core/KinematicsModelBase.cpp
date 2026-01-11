/*
 * KinematicsModelBase.h
 *
 *  Created on: Aug 3, 2017
 *      Author: Farbod
 */

#include "tbai_mpc/quadruped_arm_mpc/core/KinematicsModelBase.h"

#include <tbai_mpc/quadruped_arm_mpc/core/Rotations.h>

namespace tbai::mpc::quadruped_arm {

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::legRootInOriginFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T> &basePose) const {
    const vector3_s_t<SCALAR_T> o_basePosition = getPositionInOrigin(basePose);

    const vector3_s_t<SCALAR_T> b_baseTolegRoot = baseToLegRootInBaseFrame(footIndex);
    const vector3_s_t<SCALAR_T> o_baseTolegRoot =
        rotateVectorBaseToOrigin<SCALAR_T>(b_baseTolegRoot, getOrientation(basePose));
    return o_baseTolegRoot + o_basePosition;
}

template <typename SCALAR_T>
matrix3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::orientationLegRootToOriginFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T> &basePose) const {
    auto rotationLegRootToOrigin = rotationMatrixBaseToOrigin(getOrientation(basePose));
    if (footIndex == 1 ||
        footIndex == 3) {  // Right side, need to point the x-axis backwards and y outwards = 180deg turn around z-axis
        rotationLegRootToOrigin.col(0) = -rotationLegRootToOrigin.col(0);  // flip x-axis
        rotationLegRootToOrigin.col(1) = -rotationLegRootToOrigin.col(1);  // flip y-axis
    }
    return rotationLegRootToOrigin;
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::legRootVelocityInBaseFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T> &baseTwistInBaseFrame) const {
    const auto b_baseTolegRoot = baseToLegRootInBaseFrame(footIndex);
    return getLinearVelocity(baseTwistInBaseFrame) + getAngularVelocity(baseTwistInBaseFrame).cross(b_baseTolegRoot);
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::legRootVelocityInOriginFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T> &basePoseInOriginFrame,
    const base_coordinate_s_t<SCALAR_T> &baseTwistInBaseFrame) const {
    const vector3_s_t<SCALAR_T> b_legRootVelocity = legRootVelocityInBaseFrame(footIndex, baseTwistInBaseFrame);
    return rotateVectorBaseToOrigin<SCALAR_T>(b_legRootVelocity, getOrientation(basePoseInOriginFrame));
}

template <typename SCALAR_T>
feet_array_t<vector3_s_t<SCALAR_T>> KinematicsModelBase<SCALAR_T>::positionBaseToFeetInBaseFrame(
    const joint_coordinate_s_t<SCALAR_T> &jointPositions) const {
    feet_array_t<vector3_s_t<SCALAR_T>> baseToFeetPositions;
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
        baseToFeetPositions[i] = positionBaseToFootInBaseFrame(i, jointPositions);
    }
    return baseToFeetPositions;
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::footPositionInOriginFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T> &basePose,
    const joint_coordinate_s_t<SCALAR_T> &jointPositions) const {
    const vector3_s_t<SCALAR_T> o_basePosition = getPositionInOrigin(basePose);

    const vector3_s_t<SCALAR_T> b_baseToFoot = positionBaseToFootInBaseFrame(footIndex, jointPositions);
    const vector3_s_t<SCALAR_T> o_baseToFoot =
        rotateVectorBaseToOrigin<SCALAR_T>(b_baseToFoot, getOrientation(basePose));
    return o_baseToFoot + o_basePosition;
}

template <typename SCALAR_T>
matrix3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::footOrientationInOriginFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T> &basePose,
    const joint_coordinate_s_t<SCALAR_T> &jointPositions) const {
    const matrix3_s_t<SCALAR_T> o_R_b = rotationMatrixBaseToOrigin<SCALAR_T>(getOrientation(basePose));
    return o_R_b * footOrientationInBaseFrame(footIndex, jointPositions);
}

template <typename SCALAR_T>
feet_array_t<vector3_s_t<SCALAR_T>> KinematicsModelBase<SCALAR_T>::feetPositionsInOriginFrame(
    const base_coordinate_s_t<SCALAR_T> &basePoseInOriginFrame,
    const joint_coordinate_s_t<SCALAR_T> &jointPositions) const {
    const vector3_s_t<SCALAR_T> o_basePosition = getPositionInOrigin(basePoseInOriginFrame);
    const vector3_s_t<SCALAR_T> baseOrientation = getOrientation(basePoseInOriginFrame);

    feet_array_t<vector3_s_t<SCALAR_T>> feetPositionsInOriginFrame;
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
        vector3_s_t<SCALAR_T> b_baseToFoot = positionBaseToFootInBaseFrame(i, jointPositions);
        vector3_s_t<SCALAR_T> o_baseToFoot = rotateVectorBaseToOrigin<SCALAR_T>(b_baseToFoot, baseOrientation);
        feetPositionsInOriginFrame[i] = o_baseToFoot + o_basePosition;
    }
    return feetPositionsInOriginFrame;
}

template <typename SCALAR_T>
typename KinematicsModelBase<SCALAR_T>::joint_jacobian_t KinematicsModelBase<SCALAR_T>::baseToFootJacobianInBaseFrame(
    size_t footIndex, const joint_coordinate_s_t<SCALAR_T> &jointPositions) const {
    joint_jacobian_t footJacobian = joint_jacobian_t::Zero();
    const auto footStartIdx = 3 * footIndex;
    footJacobian.template block<6, 3>(0, footStartIdx) = baseToFootJacobianBlockInBaseFrame(footIndex, jointPositions);
    return footJacobian;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::footVelocityRelativeToBaseInBaseFrame(
    size_t footIndex, const joint_coordinate_s_t<SCALAR_T> &jointPositions,
    const joint_coordinate_s_t<SCALAR_T> &jointVelocities) const {
    const auto b_baseToFootJacobianBlock = baseToFootJacobianBlockInBaseFrame(footIndex, jointPositions);
    const auto legStartIdx = 3 * footIndex;
    return b_baseToFootJacobianBlock.template bottomRows<3>() * jointVelocities.template segment<3>(legStartIdx);
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::footVelocityInBaseFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T> &baseTwistInBaseFrame,
    const joint_coordinate_s_t<SCALAR_T> &jointPositions, const joint_coordinate_s_t<SCALAR_T> &jointVelocities) const {
    const vector3_s_t<SCALAR_T> b_footRelativeVelocity =
        footVelocityRelativeToBaseInBaseFrame(footIndex, jointPositions, jointVelocities);
    const vector3_s_t<SCALAR_T> b_baseToFoot = positionBaseToFootInBaseFrame(footIndex, jointPositions);
    return b_footRelativeVelocity + getLinearVelocity(baseTwistInBaseFrame) +
           getAngularVelocity(baseTwistInBaseFrame).cross(b_baseToFoot);
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::footVelocityInOriginFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T> &basePoseInOriginFrame,
    const base_coordinate_s_t<SCALAR_T> &baseTwistInBaseFrame, const joint_coordinate_s_t<SCALAR_T> &jointPositions,
    const joint_coordinate_s_t<SCALAR_T> &jointVelocities) const {
    const vector3_s_t<SCALAR_T> b_footVelocity =
        footVelocityInBaseFrame(footIndex, baseTwistInBaseFrame, jointPositions, jointVelocities);
    return rotateVectorBaseToOrigin<SCALAR_T>(b_footVelocity, getOrientation(basePoseInOriginFrame));
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::footVelocityInFootFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T> &baseTwistInBaseFrame,
    const joint_coordinate_s_t<SCALAR_T> &jointPositions, const joint_coordinate_s_t<SCALAR_T> &jointVelocities) const {
    const vector3_s_t<SCALAR_T> b_footVelocity =
        footVelocityInBaseFrame(footIndex, baseTwistInBaseFrame, jointPositions, jointVelocities);
    const matrix3_s_t<SCALAR_T> foot_R_b = footOrientationInBaseFrame(footIndex, jointPositions).transpose();
    return foot_R_b * b_footVelocity;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
feet_array_t<vector3_s_t<SCALAR_T>> KinematicsModelBase<SCALAR_T>::feetVelocitiesInOriginFrame(
    const base_coordinate_s_t<SCALAR_T> &basePoseInOriginFrame,
    const base_coordinate_s_t<SCALAR_T> &baseTwistInBaseFrame, const joint_coordinate_s_t<SCALAR_T> &jointPositions,
    const joint_coordinate_s_t<SCALAR_T> &jointVelocities) const {
    std::array<vector3_s_t<SCALAR_T>, 4> feetVelocitiesInOriginFrame;
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
        feetVelocitiesInOriginFrame[i] =
            footVelocityInOriginFrame(i, basePoseInOriginFrame, baseTwistInBaseFrame, jointPositions, jointVelocities);
    }
    return feetVelocitiesInOriginFrame;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
std::vector<typename KinematicsModelBase<SCALAR_T>::CollisionSphere>
KinematicsModelBase<SCALAR_T>::collisionSpheresInOriginFrame(
    const base_coordinate_s_t<SCALAR_T> &basePoseInOriginFrame,
    const joint_coordinate_s_t<SCALAR_T> &jointPositions) const {
    const vector3_s_t<SCALAR_T> o_basePosition = getPositionInOrigin(basePoseInOriginFrame);
    const vector3_s_t<SCALAR_T> baseOrientation = getOrientation(basePoseInOriginFrame);

    auto collisionSpheres = collisionSpheresInBaseFrame(jointPositions);
    for (auto &sphere : collisionSpheres) {
        sphere.position = rotateVectorBaseToOrigin<SCALAR_T>(sphere.position, baseOrientation) + o_basePosition;
    }

    return collisionSpheres;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
std::vector<typename KinematicsModelBase<SCALAR_T>::CollisionSphere>
KinematicsModelBase<SCALAR_T>::collisionSpheresInBaseFrame(const joint_coordinate_s_t<SCALAR_T> &jointPositions) const {
    return {};
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
// Arm End-Effector Kinematics Implementations
///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::armEEPositionInOriginFrame(
    const base_coordinate_s_t<SCALAR_T>& basePose,
    const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
    const arm_joint_s_t<SCALAR_T>& armJointPositions) const {
    const vector3_s_t<SCALAR_T> o_basePosition = getPositionInOrigin(basePose);
    const vector3_s_t<SCALAR_T> b_armEEPosition = armEEPositionInBaseFrame(legJointPositions, armJointPositions);
    const vector3_s_t<SCALAR_T> o_armEEPosition =
        rotateVectorBaseToOrigin<SCALAR_T>(b_armEEPosition, getOrientation(basePose));
    return o_armEEPosition + o_basePosition;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
matrix3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::armEEOrientationInOriginFrame(
    const base_coordinate_s_t<SCALAR_T>& basePose,
    const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
    const arm_joint_s_t<SCALAR_T>& armJointPositions) const {
    const matrix3_s_t<SCALAR_T> o_R_b = rotationMatrixBaseToOrigin<SCALAR_T>(getOrientation(basePose));
    return o_R_b * armEEOrientationInBaseFrame(legJointPositions, armJointPositions);
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::armEEVelocityInBaseFrame(
    const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
    const arm_joint_s_t<SCALAR_T>& armJointPositions,
    const arm_joint_s_t<SCALAR_T>& armJointVelocities) const {
    const arm_jacobian_t jacobian = armEEJacobianInBaseFrame(legJointPositions, armJointPositions);
    // Linear velocity is in top 3 rows of Jacobian
    return jacobian.template topRows<3>() * armJointVelocities;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::armEEAngularVelocityInBaseFrame(
    const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
    const arm_joint_s_t<SCALAR_T>& armJointPositions,
    const arm_joint_s_t<SCALAR_T>& armJointVelocities) const {
    const arm_jacobian_t jacobian = armEEJacobianInBaseFrame(legJointPositions, armJointPositions);
    // Angular velocity is in bottom 3 rows of Jacobian
    return jacobian.template bottomRows<3>() * armJointVelocities;
}

template class KinematicsModelBase<scalar_t>;
template class KinematicsModelBase<ocs2::CppAdInterface::ad_scalar_t>;

}  // end of namespace tbai::mpc::quadruped_arm
