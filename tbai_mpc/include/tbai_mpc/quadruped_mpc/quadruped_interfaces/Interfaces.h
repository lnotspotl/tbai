//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include "tbai_mpc/quadruped_mpc/QuadrupedInterface.h"
#include "tbai_mpc/quadruped_mpc/quadruped_models/FrameDeclaration.h"

namespace tbai::mpc::quadruped {

std::unique_ptr<tbai::mpc::quadruped::QuadrupedInterface> getAnymalInterface(const std::string& urdf, const std::string& taskFolder);

std::unique_ptr<tbai::mpc::quadruped::QuadrupedInterface> getAnymalInterface(const std::string& urdf,
                                                                       tbai::mpc::quadruped::QuadrupedInterface::Settings settings,
                                                                       const FrameDeclaration& frameDeclaration);

std::unique_ptr<tbai::mpc::quadruped::QuadrupedInterface> getGo2Interface(const std::string& urdf, const std::string& taskFolder);

std::unique_ptr<tbai::mpc::quadruped::QuadrupedInterface> getGo2Interface(const std::string& urdf,
                                                                       tbai::mpc::quadruped::QuadrupedInterface::Settings settings,
                                                                       const FrameDeclaration& frameDeclaration);

std::unique_ptr<tbai::mpc::quadruped::QuadrupedInterface> getSpotInterface(const std::string& urdf, const std::string& taskFolder);

std::unique_ptr<tbai::mpc::quadruped::QuadrupedInterface> getSpotInterface(const std::string& urdf,
                                                                       tbai::mpc::quadruped::QuadrupedInterface::Settings settings,
                                                                       const FrameDeclaration& frameDeclaration);

}  // end of namespace tbai::mpc::quadruped
