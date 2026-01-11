//
// Created by rgrandia on 22.09.20.
//

#include <unordered_map>

#include <ocs2_pinocchio_interface/urdf.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/AnymalModels.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/Go2InverseKinematics.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedCom.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedInverseKinematics.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedKinematics.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/SpotInverseKinematics.h>

namespace tbai::mpc::quadruped_arm {

std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase> getAnymalInverseKinematics(
    const FrameDeclaration &frameDeclaration, const std::string &urdf) {
    return std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase>(
        new QuadrupedInverseKinematics(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase> getGo2InverseKinematics(
    const FrameDeclaration &frameDeclaration, const std::string &urdf) {
    return std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase>(
        new Go2InverseKinematics(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase> getSpotInverseKinematics(
    const FrameDeclaration &frameDeclaration, const std::string &urdf) {
    return std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase>(
        new SpotInverseKinematics(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<tbai::mpc::quadruped_arm::KinematicsModelBase<ocs2::scalar_t>> getAnymalKinematics(
    const FrameDeclaration &frameDeclaration, const std::string &urdf) {
    return std::unique_ptr<tbai::mpc::quadruped_arm::KinematicsModelBase<ocs2::scalar_t>>(
        new QuadrupedKinematics(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<tbai::mpc::quadruped_arm::KinematicsModelBase<ocs2::ad_scalar_t>> getAnymalKinematicsAd(
    const FrameDeclaration &frameDeclaration, const std::string &urdf) {
    return std::unique_ptr<tbai::mpc::quadruped_arm::KinematicsModelBase<ocs2::ad_scalar_t>>(
        new QuadrupedKinematicsAd(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<tbai::mpc::quadruped_arm::ComModelBase<ocs2::scalar_t>> getAnymalComModel(
    const FrameDeclaration &frameDeclaration, const std::string &urdf) {
    return std::unique_ptr<tbai::mpc::quadruped_arm::ComModelBase<ocs2::scalar_t>>(
        new QuadrupedCom(frameDeclaration, createQuadrupedPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<tbai::mpc::quadruped_arm::ComModelBase<ocs2::ad_scalar_t>> getAnymalComModelAd(
    const FrameDeclaration &frameDeclaration, const std::string &urdf) {
    return std::unique_ptr<tbai::mpc::quadruped_arm::ComModelBase<ocs2::ad_scalar_t>>(
        new QuadrupedComAd(frameDeclaration, createQuadrupedPinocchioInterfaceFromUrdfString(urdf)));
}

}  // namespace tbai::mpc::quadruped_arm
