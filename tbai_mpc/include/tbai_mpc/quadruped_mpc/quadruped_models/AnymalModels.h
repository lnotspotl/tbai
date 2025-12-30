//
// Created by rgrandia on 22.09.20.
//

#pragma once

#include <tbai_mpc/quadruped_mpc/core/ComModelBase.h>
#include <tbai_mpc/quadruped_mpc/core/InverseKinematicsModelBase.h>
#include <tbai_mpc/quadruped_mpc/core/KinematicsModelBase.h>

#include "tbai_mpc/quadruped_mpc/quadruped_models/FrameDeclaration.h"

namespace anymal {

std::unique_ptr<switched_model::InverseKinematicsModelBase> getAnymalInverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf);

std::unique_ptr<switched_model::InverseKinematicsModelBase> getGo2InverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getAnymalKinematics(const FrameDeclaration& frameDeclaration,
                                                                                         const std::string& urdf);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getAnymalKinematicsAd(const FrameDeclaration& frameDeclaration,
                                                                                              const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getAnymalComModel(const FrameDeclaration& frameDeclaration,
                                                                                const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getAnymalComModelAd(const FrameDeclaration& frameDeclaration,
                                                                                     const std::string& urdf);

}  // namespace anymal
