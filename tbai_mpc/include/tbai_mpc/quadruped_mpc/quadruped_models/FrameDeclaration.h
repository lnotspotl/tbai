//
// Created by rgrandia on 27.04.22.
//

#pragma once

#include <tbai_mpc/quadruped_mpc/core/SwitchedModel.h>

namespace tbai::mpc::quadruped {

struct CollisionDeclaration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string link;
  tbai::mpc::quadruped::scalar_t radius;
  tbai::mpc::quadruped::vector3_t offset;
};

struct LimbFrames {
  std::string root;
  std::string tip;
  std::vector<std::string> joints;
};

struct FrameDeclaration {
  std::string root;
  tbai::mpc::quadruped::feet_array_t<LimbFrames> legs;
  std::vector<CollisionDeclaration> collisions;
};

std::vector<std::string> getJointNames(const FrameDeclaration& frameDeclaration);

LimbFrames limbFramesFromFile(const std::string& file, const std::string& field);

FrameDeclaration frameDeclarationFromFile(const std::string& file);

}  // namespace tbai::mpc::quadruped