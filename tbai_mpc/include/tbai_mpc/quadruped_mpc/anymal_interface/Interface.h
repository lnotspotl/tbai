//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include "tbai_mpc/quadruped_mpc/QuadrupedInterface.h"

#include "tbai_mpc/quadruped_mpc/quadruped_models/AnymalModels.h"
#include "tbai_mpc/quadruped_mpc/quadruped_models/FrameDeclaration.h"

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf, const std::string& taskFolder);

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf,
                                                                       switched_model::QuadrupedInterface::Settings settings,
                                                                       const FrameDeclaration& frameDeclaration);

std::unique_ptr<switched_model::QuadrupedInterface> getGo2Interface(const std::string& urdf, const std::string& taskFolder);

std::unique_ptr<switched_model::QuadrupedInterface> getGo2Interface(const std::string& urdf,
                                                                       switched_model::QuadrupedInterface::Settings settings,
                                                                       const FrameDeclaration& frameDeclaration);

}  // end of namespace anymal
