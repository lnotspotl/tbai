//
// Created by rgrandia on 22.09.20.
//

#pragma once

#include <tbai_mpc/quadruped_arm_mpc/core/ComModelBase.h>
#include <tbai_mpc/quadruped_arm_mpc/core/InverseKinematicsModelBase.h>
#include <tbai_mpc/quadruped_arm_mpc/core/KinematicsModelBase.h>

#include "tbai_mpc/quadruped_arm_mpc/quadruped_models/FrameDeclaration.h"

namespace tbai::mpc::quadruped_arm {

std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase> getAnymalInverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf);

std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase> getGo2InverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf);

std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase> getSpotInverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf);

std::unique_ptr<tbai::mpc::quadruped_arm::KinematicsModelBase<ocs2::scalar_t>> getAnymalKinematics(const FrameDeclaration& frameDeclaration,
                                                                                         const std::string& urdf);

std::unique_ptr<tbai::mpc::quadruped_arm::KinematicsModelBase<ocs2::ad_scalar_t>> getAnymalKinematicsAd(const FrameDeclaration& frameDeclaration,
                                                                                              const std::string& urdf);

std::unique_ptr<tbai::mpc::quadruped_arm::ComModelBase<ocs2::scalar_t>> getAnymalComModel(const FrameDeclaration& frameDeclaration,
                                                                                const std::string& urdf);

std::unique_ptr<tbai::mpc::quadruped_arm::ComModelBase<ocs2::ad_scalar_t>> getAnymalComModelAd(const FrameDeclaration& frameDeclaration,
                                                                                     const std::string& urdf);

}  // namespace tbai::mpc::quadruped_arm
