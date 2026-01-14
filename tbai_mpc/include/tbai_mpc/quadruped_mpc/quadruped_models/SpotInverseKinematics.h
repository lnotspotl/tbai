//
// Created by lnotspotl on 19.1.2024.
//

#pragma once


#include <tbai_mpc/quadruped_mpc/core/InverseKinematicsModelBase.h>

#include <tbai_mpc/quadruped_mpc/analytical_inverse_kinematics/LegInverseKinematicParameters.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedPinocchioMapping.h"

namespace tbai::mpc::quadruped {

class SpotInverseKinematics final : public tbai::mpc::quadruped::InverseKinematicsModelBase {
 public:
  SpotInverseKinematics(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface);

  ~SpotInverseKinematics() override = default;

  SpotInverseKinematics* clone() const override;

  tbai::mpc::quadruped::vector3_t getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      size_t footIndex, const tbai::mpc::quadruped::vector3_t& positionBaseToFootInBaseFrame) const override;

  tbai::mpc::quadruped::vector3_t getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(
      size_t footIndex, const tbai::mpc::quadruped::vector3_t& footVelocityRelativeToBaseInBaseFrame,
      const joint_jacobian_block_t& jointJacobian, tbai::mpc::quadruped::scalar_t damping) const override;

 private:
  SpotInverseKinematics(const SpotInverseKinematics& other) = default;

  tbai::mpc::quadruped::feet_array_t<tbai::mpc::quadruped::analytical_inverse_kinematics::LegInverseKinematicParameters> parameters_;
};

};  // namespace tbai::mpc::quadruped
