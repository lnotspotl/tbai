//
// Created by rgrandia on 19.07.22.
//

#pragma once

#include <Eigen/Geometry>

namespace tbai::mpc::quadruped_arm {

/**
 * CostElements for quadruped with arm.
 * Contains all tracking targets for motion tracking cost.
 *
 * Structure:
 * - Base: eulerXYZ (3) + comPosition (3) + comAngularVelocity (3) + comLinearVelocity (3) = 12
 * - Legs: per leg [jointPosition (3) + footPosition (3) + jointVelocity (3) + footVelocity (3) + contactForce (3)] = 15
 *         Total for 4 legs = 60
 * - Arm: armEEPosition (3) + armEEOrientation (3) + armEEVelocity (3) + armJointPosition (6) + armJointVelocity (6) = 21
 * Total: 12 + 60 + 21 = 93
 */
template <typename SCALAR_T>
struct CostElements {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Base tracking targets (12D)
  vector3_s_t<SCALAR_T> eulerXYZ{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> comPosition{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> comAngularVelocity{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> comLinearVelocity{vector3_s_t<SCALAR_T>::Zero()};

  // Leg tracking targets (60D: 4 legs x 15D each)
  feet_array_t<vector3_s_t<SCALAR_T>> jointPosition{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> footPosition{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> jointVelocity{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> footVelocity{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> contactForce{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};

  // Arm end-effector tracking targets (9D: position + orientation + velocity)
  vector3_s_t<SCALAR_T> armEEPosition{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> armEEOrientation{vector3_s_t<SCALAR_T>::Zero()};  // Axis-angle representation
  vector3_s_t<SCALAR_T> armEEVelocity{vector3_s_t<SCALAR_T>::Zero()};     // Linear velocity

  // Arm joint tracking targets (12D: 6 position + 6 velocity)
  arm_joint_s_t<SCALAR_T> armJointPosition{arm_joint_s_t<SCALAR_T>::Zero()};
  arm_joint_s_t<SCALAR_T> armJointVelocity{arm_joint_s_t<SCALAR_T>::Zero()};

  static size_t Size() {
    constexpr size_t baseTargets = 12;
    constexpr size_t legTargets = 15;
    constexpr size_t armTargets = 3 + 3 + 3 + NUM_ARM_JOINTS + NUM_ARM_JOINTS;  // EE pos + EE orient + EE vel + joint pos + joint vel = 21
    return baseTargets + NUM_CONTACT_POINTS * legTargets + armTargets;          // 12 + 60 + 21 = 93
  }

  Eigen::Matrix<SCALAR_T, -1, 1> asVector() const {
    constexpr size_t baseTargets = 12;
    constexpr size_t legTargets = 15;
    constexpr size_t armTargets = 3 + 3 + 3 + NUM_ARM_JOINTS + NUM_ARM_JOINTS;
    constexpr size_t costVectorLength = baseTargets + NUM_CONTACT_POINTS * legTargets + armTargets;

    Eigen::Matrix<SCALAR_T, -1, 1> v(costVectorLength);

    // Base
    v.head(baseTargets) << eulerXYZ, comPosition, comAngularVelocity, comLinearVelocity;

    // Legs
    for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
      v.segment(baseTargets + leg * legTargets, legTargets) << jointPosition[leg], footPosition[leg], jointVelocity[leg], footVelocity[leg],
          contactForce[leg];
    }

    // Arm
    const size_t armOffset = baseTargets + NUM_CONTACT_POINTS * legTargets;
    v.segment(armOffset, armTargets) << armEEPosition, armEEOrientation, armEEVelocity, armJointPosition, armJointVelocity;

    return v;
  }
};

}  // namespace tbai::mpc::quadruped_arm
