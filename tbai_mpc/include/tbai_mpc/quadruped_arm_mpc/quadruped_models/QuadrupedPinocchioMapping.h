#pragma once

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h>

#include <tbai_mpc/quadruped_arm_mpc/quadruped_models//FrameDeclaration.h>

namespace tbai::mpc::quadruped_arm {

/**
 * Used to map joint configuration space from OCS2 to Pinocchio. In OCS2, the feet order is {LF, RF, LH, RH}. But in Pinocchio, the feet
 * order depends on the URDF.
 */
class QuadrupedPinocchioMapping {
 public:
  QuadrupedPinocchioMapping(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface);

  tbai::mpc::quadruped_arm::joint_coordinate_t getPinocchioJointVector(const tbai::mpc::quadruped_arm::joint_coordinate_t& jointPositions) const;

  tbai::mpc::quadruped_arm::joint_coordinate_ad_t getPinocchioJointVector(const tbai::mpc::quadruped_arm::joint_coordinate_ad_t& jointPositions) const;

  size_t getPinocchioFootIndex(size_t ocs2FootIdx) const { return mapFeetOrderOcs2ToPinocchio_[ocs2FootIdx]; }

  size_t getFootFrameId(size_t ocs2FootIdx) const { return footFrameIds_[ocs2FootIdx]; }

  size_t getHipFrameId(size_t ocs2FootIdx) const { return hipFrameIds_[ocs2FootIdx]; }

  size_t getArmEEFrameId() const { return armEEFrameId_; }

  const std::vector<size_t>& getCollisionLinkFrameIds() const { return collisionLinkFrameIds_; }
  const std::vector<CollisionDeclaration>& getCollisionDeclaration() const { return collisionDeclaration_; }

  const std::vector<std::string>& getOcs2JointNames() const { return ocs2JointNames_; }

  const std::vector<std::string>& getPinocchioJointNames() const { return pinocchioJointNames_; }

  static size_t getBodyId(const std::string& bodyName, const ocs2::PinocchioInterface& pinocchioInterface);

 private:
  void extractPinocchioJointNames(const ocs2::PinocchioInterface& pinocchioInterface);
  void extractFeetOrdering(const ocs2::PinocchioInterface& pinocchioInterface);

  // Frame Ids
  tbai::mpc::quadruped_arm::feet_array_t<size_t> hipFrameIds_;
  tbai::mpc::quadruped_arm::feet_array_t<size_t> footFrameIds_;
  size_t armEEFrameId_{0};

  // Collisions
  std::vector<size_t> collisionLinkFrameIds_;
  std::vector<CollisionDeclaration> collisionDeclaration_;

  // Feet ordering
  tbai::mpc::quadruped_arm::feet_array_t<size_t> mapFeetOrderOcs2ToPinocchio_;
  tbai::mpc::quadruped_arm::feet_array_t<size_t> mapFeetOrderPinocchioToOcs2_;

  // Frame names
  std::vector<std::string> ocs2JointNames_;
  std::vector<std::string> pinocchioJointNames_;
};

}  // namespace tbai::mpc::quadruped_arm
