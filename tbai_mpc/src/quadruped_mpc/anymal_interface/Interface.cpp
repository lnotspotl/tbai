//
// Created by rgrandia on 17.02.20.
//

#include "tbai_mpc/quadruped_mpc/anymal_interface/Interface.h"
#include "tbai_mpc/quadruped_mpc/QuadrupedPointfootInterface.h"

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf, const std::string& taskFolder) {
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  return getAnymalInterface(urdf, switched_model::loadQuadrupedSettings(taskFolder + "/task.info"),
                            frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
}

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf,
                                                                       switched_model::QuadrupedInterface::Settings settings,
                                                                       const FrameDeclaration& frameDeclaration) {
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

std::string getConfigFolder(const std::string& configName) {
  return "/home/kb/Documents/ros/src/tbai_ros/dependencies/ocs2/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_mpc/config/c_series";
}

std::string getTaskFilePath(const std::string& configName) {
  return getConfigFolder(configName) + "/task.info";
}

std::unique_ptr<switched_model::QuadrupedInterface> getGo2Interface(const std::string& urdf, const std::string& taskFolder) {
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  return getGo2Interface(urdf, switched_model::loadQuadrupedSettings(taskFolder + "/task.info"),
                            frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
}

std::unique_ptr<switched_model::QuadrupedInterface> getGo2Interface(const std::string& urdf,
                                                                       switched_model::QuadrupedInterface::Settings settings,
                                                                       const FrameDeclaration& frameDeclaration) {
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

}  // namespace anymal
