//
// Created by rgrandia on 17.02.20.
//

#include "tbai_mpc/quadruped_mpc/quadruped_interfaces/Interfaces.h"

#include "tbai_mpc/quadruped_mpc/QuadrupedPointfootInterface.h"
#include "tbai_mpc/quadruped_mpc/quadruped_models/AnymalModels.h"

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string &urdf,
                                                                       const std::string &taskFolder) {
    std::cerr << "Loading task file from: " << taskFolder << std::endl;

    return getAnymalInterface(urdf, switched_model::loadQuadrupedSettings(taskFolder + "/task.info"),
                              frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
}

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(
    const std::string &urdf, switched_model::QuadrupedInterface::Settings settings,
    const FrameDeclaration &frameDeclaration) {
    std::unique_ptr<switched_model::InverseKinematicsModelBase> invKin{nullptr};
    if (settings.modelSettings_.analyticalInverseKinematics_) {
        invKin = getAnymalInverseKinematics(frameDeclaration, urdf);
    }
    auto kin = getAnymalKinematics(frameDeclaration, urdf);
    auto kinAd = getAnymalKinematicsAd(frameDeclaration, urdf);
    auto com = getAnymalComModel(frameDeclaration, urdf);
    auto comAd = getAnymalComModelAd(frameDeclaration, urdf);
    auto jointNames = getJointNames(frameDeclaration);
    auto baseName = frameDeclaration.root;

    return std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedPointfootInterface(
        *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName)));
}

std::unique_ptr<switched_model::QuadrupedInterface> getGo2Interface(const std::string &urdf,
                                                                    const std::string &taskFolder) {
    std::cerr << "Loading task file from: " << taskFolder << std::endl;

    return getGo2Interface(urdf, switched_model::loadQuadrupedSettings(taskFolder + "/task.info"),
                           frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
}

std::unique_ptr<switched_model::QuadrupedInterface> getGo2Interface(
    const std::string &urdf, switched_model::QuadrupedInterface::Settings settings,
    const FrameDeclaration &frameDeclaration) {
    std::unique_ptr<switched_model::InverseKinematicsModelBase> invKin{nullptr};
    if (settings.modelSettings_.analyticalInverseKinematics_) {
        invKin = getGo2InverseKinematics(frameDeclaration, urdf);
    }
    auto kin = getAnymalKinematics(frameDeclaration, urdf);
    auto kinAd = getAnymalKinematicsAd(frameDeclaration, urdf);
    auto com = getAnymalComModel(frameDeclaration, urdf);
    auto comAd = getAnymalComModelAd(frameDeclaration, urdf);
    auto jointNames = getJointNames(frameDeclaration);
    auto baseName = frameDeclaration.root;

    return std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedPointfootInterface(
        *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName)));
}

std::unique_ptr<switched_model::QuadrupedInterface> getSpotInterface(const std::string &urdf,
                                                                     const std::string &taskFolder) {
    std::cerr << "Loading task file from: " << taskFolder << std::endl;

    return getSpotInterface(urdf, switched_model::loadQuadrupedSettings(taskFolder + "/task.info"),
                            frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
}

std::unique_ptr<switched_model::QuadrupedInterface> getSpotInterface(
    const std::string &urdf, switched_model::QuadrupedInterface::Settings settings,
    const FrameDeclaration &frameDeclaration) {
    std::unique_ptr<switched_model::InverseKinematicsModelBase> invKin{nullptr};
    if (settings.modelSettings_.analyticalInverseKinematics_) {
        invKin = getSpotInverseKinematics(frameDeclaration, urdf);
    }
    auto kin = getAnymalKinematics(frameDeclaration, urdf);
    auto kinAd = getAnymalKinematicsAd(frameDeclaration, urdf);
    auto com = getAnymalComModel(frameDeclaration, urdf);
    auto comAd = getAnymalComModelAd(frameDeclaration, urdf);
    auto jointNames = getJointNames(frameDeclaration);
    auto baseName = frameDeclaration.root;

    return std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedPointfootInterface(
        *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName)));
}

}  // namespace anymal
