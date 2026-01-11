//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include "tbai_mpc/quadruped_arm_mpc/QuadrupedInterface.h"
#include "tbai_mpc/quadruped_arm_mpc/quadruped_models/FrameDeclaration.h"

namespace tbai::mpc::quadruped_arm {

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getAnymalInterface(const std::string& urdf, const std::string& taskFolder);

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getAnymalInterface(const std::string& urdf,
                                                                       tbai::mpc::quadruped_arm::QuadrupedInterface::Settings settings,
                                                                       const tbai::mpc::quadruped_arm::FrameDeclaration& frameDeclaration);

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getGo2Interface(const std::string& urdf, const std::string& taskFolder);

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getGo2Interface(const std::string& urdf,
                                                                       tbai::mpc::quadruped_arm::QuadrupedInterface::Settings settings,
                                                                       const tbai::mpc::quadruped_arm::FrameDeclaration& frameDeclaration);

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getSpotInterface(const std::string& urdf, const std::string& taskFolder);

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getSpotInterface(const std::string& urdf,
                                                                       tbai::mpc::quadruped_arm::QuadrupedInterface::Settings settings,
                                                                       const tbai::mpc::quadruped_arm::FrameDeclaration& frameDeclaration);

}  // namespace tbai::mpc::quadruped_arm
