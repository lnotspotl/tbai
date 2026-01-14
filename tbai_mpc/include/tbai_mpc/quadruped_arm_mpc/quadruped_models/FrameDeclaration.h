//
// Created by rgrandia on 27.04.22.
//

#pragma once

#include <tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h>

namespace tbai::mpc::quadruped_arm {

struct CollisionDeclaration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string link;
  tbai::mpc::quadruped_arm::scalar_t radius;
  tbai::mpc::quadruped_arm::vector3_t offset;
};

struct LimbFrames {
  std::string root;
  std::string tip;
  std::vector<std::string> joints;
};

struct ArmFrames {
  std::string root;
  std::string tip;  // end-effector frame
  std::vector<std::string> joints;
};

struct FrameDeclaration {
  std::string root;
  tbai::mpc::quadruped_arm::feet_array_t<LimbFrames> legs;
  std::vector<CollisionDeclaration> collisions;
  ArmFrames arm;  // Arm configuration for quadruped with manipulator
};

std::vector<std::string> getJointNames(const FrameDeclaration& frameDeclaration);

LimbFrames limbFramesFromFile(const std::string& file, const std::string& field);

FrameDeclaration frameDeclarationFromFile(const std::string& file);

}  // namespace tbai::mpc::quadruped_arm