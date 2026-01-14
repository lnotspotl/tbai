/*
 * KinematicsModelBase.h
 *
 *  Created on: Aug 2, 2017
 *      Author: Farbod
 */

#pragma once

#include <array>

#include "tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h"

namespace tbai::mpc::quadruped_arm {

/**
 * ModelKinematics Base Class
 * @tparam SCALAR_T
 */
template <typename SCALAR_T>
class KinematicsModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using joint_jacobian_t = Eigen::Matrix<SCALAR_T, 6, JOINT_COORDINATE_SIZE>;
  using joint_jacobian_block_t = Eigen::Matrix<SCALAR_T, 6, 3>;

  KinematicsModelBase() = default;

  virtual ~KinematicsModelBase() = default;

  virtual KinematicsModelBase<SCALAR_T>* clone() const = 0;

  virtual vector3_s_t<SCALAR_T> baseToLegRootInBaseFrame(size_t footIndex) const = 0;

  vector3_s_t<SCALAR_T> legRootInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose) const;

  matrix3_s_t<SCALAR_T> orientationLegRootToOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose) const;

  vector3_s_t<SCALAR_T> legRootVelocityInBaseFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame) const;

  vector3_s_t<SCALAR_T> legRootVelocityInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                     const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame) const;

  virtual vector3_s_t<SCALAR_T> positionBaseToFootInBaseFrame(size_t footIndex,
                                                              const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  feet_array_t<vector3_s_t<SCALAR_T>> positionBaseToFeetInBaseFrame(const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  vector3_s_t<SCALAR_T> footPositionInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  feet_array_t<vector3_s_t<SCALAR_T>> feetPositionsInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePose,
                                                                 const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  joint_jacobian_t baseToFootJacobianInBaseFrame(size_t footIndex, const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  virtual joint_jacobian_block_t baseToFootJacobianBlockInBaseFrame(size_t footIndex,
                                                                    const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  virtual matrix3_s_t<SCALAR_T> footOrientationInBaseFrame(size_t footIndex,
                                                           const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  matrix3_s_t<SCALAR_T> footOrientationInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose,
                                                     const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  virtual vector3_s_t<SCALAR_T> footVelocityRelativeToBaseInBaseFrame(size_t footIndex,
                                                                      const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                      const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  vector3_s_t<SCALAR_T> footVelocityInBaseFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  vector3_s_t<SCALAR_T> footVelocityInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                  const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  vector3_s_t<SCALAR_T> footVelocityInFootFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  feet_array_t<vector3_s_t<SCALAR_T>> feetVelocitiesInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                                  const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                  const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  struct CollisionSphere {
    vector3_s_t<SCALAR_T> position;
    SCALAR_T radius;
  };
  std::vector<CollisionSphere> collisionSpheresInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                             const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  virtual std::vector<CollisionSphere> collisionSpheresInBaseFrame(const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  // =============================================
  // Arm End-Effector Kinematics
  // =============================================

  using arm_jacobian_t = Eigen::Matrix<SCALAR_T, 6, NUM_ARM_JOINTS>;

  /**
   * Arm end-effector position relative to base frame.
   * @param legJointPositions: Leg joint angles (12D) - may affect arm mounting
   * @param armJointPositions: Arm joint angles (6D)
   * @return 3D position of arm end-effector in base frame
   */
  virtual vector3_s_t<SCALAR_T> armEEPositionInBaseFrame(const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
                                                          const arm_joint_s_t<SCALAR_T>& armJointPositions) const {
    // Default implementation - to be overridden for robots with arms
    (void)legJointPositions;
    (void)armJointPositions;
    return vector3_s_t<SCALAR_T>::Zero();
  }

  /**
   * Arm end-effector orientation in base frame.
   * @param legJointPositions: Leg joint angles (12D)
   * @param armJointPositions: Arm joint angles (6D)
   * @return 3x3 rotation matrix of arm end-effector in base frame
   */
  virtual matrix3_s_t<SCALAR_T> armEEOrientationInBaseFrame(const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
                                                             const arm_joint_s_t<SCALAR_T>& armJointPositions) const {
    // Default implementation - identity rotation
    (void)legJointPositions;
    (void)armJointPositions;
    return matrix3_s_t<SCALAR_T>::Identity();
  }

  /**
   * Arm end-effector position in world/origin frame.
   * @param basePose: Base pose in origin frame (6D: euler + position)
   * @param legJointPositions: Leg joint angles (12D)
   * @param armJointPositions: Arm joint angles (6D)
   * @return 3D position of arm end-effector in origin frame
   */
  vector3_s_t<SCALAR_T> armEEPositionInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePose,
                                                    const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
                                                    const arm_joint_s_t<SCALAR_T>& armJointPositions) const;

  /**
   * Arm end-effector orientation in world/origin frame.
   * @param basePose: Base pose in origin frame (6D: euler + position)
   * @param legJointPositions: Leg joint angles (12D)
   * @param armJointPositions: Arm joint angles (6D)
   * @return 3x3 rotation matrix of arm end-effector in origin frame
   */
  matrix3_s_t<SCALAR_T> armEEOrientationInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePose,
                                                       const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
                                                       const arm_joint_s_t<SCALAR_T>& armJointPositions) const;

  /**
   * Arm end-effector Jacobian in base frame.
   * Maps arm joint velocities to end-effector spatial velocity.
   * @param legJointPositions: Leg joint angles (12D)
   * @param armJointPositions: Arm joint angles (6D)
   * @return 6xNUM_ARM_JOINTS Jacobian (linear velocity top 3 rows, angular bottom 3 rows)
   */
  virtual arm_jacobian_t armEEJacobianInBaseFrame(const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
                                                   const arm_joint_s_t<SCALAR_T>& armJointPositions) const {
    // Default implementation - zero Jacobian
    (void)legJointPositions;
    (void)armJointPositions;
    return arm_jacobian_t::Zero();
  }

  /**
   * Arm end-effector velocity in base frame.
   * @param legJointPositions: Leg joint angles (12D)
   * @param armJointPositions: Arm joint angles (6D)
   * @param armJointVelocities: Arm joint velocities (6D)
   * @return 3D linear velocity of arm end-effector in base frame
   */
  vector3_s_t<SCALAR_T> armEEVelocityInBaseFrame(const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
                                                  const arm_joint_s_t<SCALAR_T>& armJointPositions,
                                                  const arm_joint_s_t<SCALAR_T>& armJointVelocities) const;

  /**
   * Arm end-effector angular velocity in base frame.
   * @param legJointPositions: Leg joint angles (12D)
   * @param armJointPositions: Arm joint angles (6D)
   * @param armJointVelocities: Arm joint velocities (6D)
   * @return 3D angular velocity of arm end-effector in base frame
   */
  vector3_s_t<SCALAR_T> armEEAngularVelocityInBaseFrame(const leg_joint_coordinate_s_t<SCALAR_T>& legJointPositions,
                                                         const arm_joint_s_t<SCALAR_T>& armJointPositions,
                                                         const arm_joint_s_t<SCALAR_T>& armJointVelocities) const;
};

extern template class KinematicsModelBase<scalar_t>;
extern template class KinematicsModelBase<ocs2::CppAdInterface::ad_scalar_t>;

}  // end of namespace tbai::mpc::quadruped_arm
