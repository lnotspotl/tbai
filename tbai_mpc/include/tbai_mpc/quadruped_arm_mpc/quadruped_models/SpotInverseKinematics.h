//
// Created by lnotspotl on 19.1.2024.
//

#pragma once


#include <tbai_mpc/quadruped_arm_mpc/core/InverseKinematicsModelBase.h>

#include <tbai_mpc/quadruped_arm_mpc/analytical_inverse_kinematics/LegInverseKinematicParameters.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedPinocchioMapping.h"

namespace tbai::mpc::quadruped_arm {

class SpotInverseKinematics final : public tbai::mpc::quadruped_arm::InverseKinematicsModelBase {
 public:
  SpotInverseKinematics(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface);

  ~SpotInverseKinematics() override = default;

  SpotInverseKinematics* clone() const override;

  tbai::mpc::quadruped_arm::vector3_t getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      size_t footIndex, const tbai::mpc::quadruped_arm::vector3_t& positionBaseToFootInBaseFrame) const override;

  tbai::mpc::quadruped_arm::vector3_t getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(
      size_t footIndex, const tbai::mpc::quadruped_arm::vector3_t& footVelocityRelativeToBaseInBaseFrame,
      const joint_jacobian_block_t& jointJacobian, tbai::mpc::quadruped_arm::scalar_t damping) const override;

 private:
  SpotInverseKinematics(const SpotInverseKinematics& other) = default;

  tbai::mpc::quadruped_arm::feet_array_t<tbai::mpc::quadruped_arm::analytical_inverse_kinematics::LegInverseKinematicParameters> parameters_;
};

};  // namespace tbai::mpc::quadruped_arm
