//
// Created by rgrandia on 17.02.20.
//

#include "tbai_mpc/quadruped_arm_mpc/quadruped_interfaces/Interfaces.h"

#include "tbai_mpc/quadruped_arm_mpc/QuadrupedPointfootInterface.h"
#include "tbai_mpc/quadruped_arm_mpc/quadruped_models/AnymalModels.h"

namespace tbai::mpc::quadruped_arm {

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getAnymalInterface(const std::string &urdf,
                                                                       const std::string &taskFolder) {
    std::cerr << "Loading task file from: " << taskFolder << std::endl;

    return getAnymalInterface(urdf, tbai::mpc::quadruped_arm::loadQuadrupedSettings(taskFolder + "/task.info"),
                              tbai::mpc::quadruped_arm::frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
}

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getAnymalInterface(
    const std::string &urdf, tbai::mpc::quadruped_arm::QuadrupedInterface::Settings settings,
    const tbai::mpc::quadruped_arm::FrameDeclaration &frameDeclaration) {
    std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase> invKin{nullptr};
    if (settings.modelSettings_.analyticalInverseKinematics_) {
        invKin = tbai::mpc::quadruped_arm::getAnymalInverseKinematics(frameDeclaration, urdf);
    }
    auto kin = tbai::mpc::quadruped_arm::getAnymalKinematics(frameDeclaration, urdf);
    auto kinAd = tbai::mpc::quadruped_arm::getAnymalKinematicsAd(frameDeclaration, urdf);
    auto com = tbai::mpc::quadruped_arm::getAnymalComModel(frameDeclaration, urdf);
    auto comAd = tbai::mpc::quadruped_arm::getAnymalComModelAd(frameDeclaration, urdf);
    auto jointNames = tbai::mpc::quadruped_arm::getJointNames(frameDeclaration);
    auto baseName = frameDeclaration.root;

    return std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface>(new tbai::mpc::quadruped_arm::QuadrupedPointfootInterface(
        *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName)));
}

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getGo2Interface(const std::string &urdf,
                                                                    const std::string &taskFolder) {
    std::cerr << "Loading task file from: " << taskFolder << std::endl;

    return getGo2Interface(urdf, tbai::mpc::quadruped_arm::loadQuadrupedSettings(taskFolder + "/task.info"),
                           tbai::mpc::quadruped_arm::frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
}

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getGo2Interface(
    const std::string &urdf, tbai::mpc::quadruped_arm::QuadrupedInterface::Settings settings,
    const tbai::mpc::quadruped_arm::FrameDeclaration &frameDeclaration) {
    std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase> invKin{nullptr};
    if (settings.modelSettings_.analyticalInverseKinematics_) {
        invKin = tbai::mpc::quadruped_arm::getGo2InverseKinematics(frameDeclaration, urdf);
    }
    auto kin = tbai::mpc::quadruped_arm::getAnymalKinematics(frameDeclaration, urdf);
    auto kinAd = tbai::mpc::quadruped_arm::getAnymalKinematicsAd(frameDeclaration, urdf);
    auto com = tbai::mpc::quadruped_arm::getAnymalComModel(frameDeclaration, urdf);
    auto comAd = tbai::mpc::quadruped_arm::getAnymalComModelAd(frameDeclaration, urdf);
    auto jointNames = tbai::mpc::quadruped_arm::getJointNames(frameDeclaration);
    auto baseName = frameDeclaration.root;

    return std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface>(new tbai::mpc::quadruped_arm::QuadrupedPointfootInterface(
        *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName)));
}

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getSpotInterface(const std::string &urdf,
                                                                     const std::string &taskFolder) {
    std::cerr << "Loading task file from: " << taskFolder << std::endl;

    return getSpotInterface(urdf, tbai::mpc::quadruped_arm::loadQuadrupedSettings(taskFolder + "/task.info"),
                            tbai::mpc::quadruped_arm::frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
}

std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> getSpotInterface(
    const std::string &urdf, tbai::mpc::quadruped_arm::QuadrupedInterface::Settings settings,
    const tbai::mpc::quadruped_arm::FrameDeclaration &frameDeclaration) {
    std::unique_ptr<tbai::mpc::quadruped_arm::InverseKinematicsModelBase> invKin{nullptr};
    if (settings.modelSettings_.analyticalInverseKinematics_) {
        invKin = tbai::mpc::quadruped_arm::getSpotInverseKinematics(frameDeclaration, urdf);
    }
    auto kin = tbai::mpc::quadruped_arm::getAnymalKinematics(frameDeclaration, urdf);
    auto kinAd = tbai::mpc::quadruped_arm::getAnymalKinematicsAd(frameDeclaration, urdf);
    auto com = tbai::mpc::quadruped_arm::getAnymalComModel(frameDeclaration, urdf);
    auto comAd = tbai::mpc::quadruped_arm::getAnymalComModelAd(frameDeclaration, urdf);
    auto jointNames = tbai::mpc::quadruped_arm::getJointNames(frameDeclaration);
    auto baseName = frameDeclaration.root;

    return std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface>(new tbai::mpc::quadruped_arm::QuadrupedPointfootInterface(
        *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName)));
}

}  // namespace tbai::mpc::quadruped_arm
