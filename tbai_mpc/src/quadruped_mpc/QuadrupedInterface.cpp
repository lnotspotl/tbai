//
// Created by rgrandia on 17.02.20.
//

#include "tbai_mpc/quadruped_mpc/QuadrupedInterface.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <tbai_mpc/quadruped_mpc/constraint/EndEffectorVelocityConstraint.h>
#include <tbai_mpc/quadruped_mpc/constraint/FootNormalConstraint.h>
#include <tbai_mpc/quadruped_mpc/constraint/FrictionConeConstraint.h>
#include <tbai_mpc/quadruped_mpc/constraint/ZeroForceConstraint.h>
#include <tbai_mpc/quadruped_mpc/core/SwitchedModelPrecomputation.h>
#include <tbai_mpc/quadruped_mpc/cost/CollisionAvoidanceCost.h>
#include <tbai_mpc/quadruped_mpc/cost/FootPlacementCost.h>
#include <tbai_mpc/quadruped_mpc/cost/FrictionConeCost.h>
#include <tbai_mpc/quadruped_mpc/cost/MotionTrackingCost.h>
#include <tbai_mpc/quadruped_mpc/cost/MotionTrackingTerminalCost.h>
#include <tbai_mpc/quadruped_mpc/cost/TorqueLimitsSoftConstraint.h>
#include <tbai_mpc/quadruped_mpc/dynamics/ComKinoSystemDynamicsAd.h>
#include <tbai_mpc/quadruped_mpc/foot_planner/SwingTrajectoryPlanner.h>
#include <tbai_mpc/quadruped_mpc/initialization/ComKinoInitializer.h>
#include <tbai_mpc/quadruped_mpc/logic/ModeSequenceTemplate.h>
#include <tbai_mpc/quadruped_mpc/terrain/PlanarTerrainModel.h>

namespace tbai::mpc::quadruped {

QuadrupedInterface::QuadrupedInterface(const kinematic_model_t &kinematicModel,
                                       const ad_kinematic_model_t &adKinematicModel, const com_model_t &comModel,
                                       const ad_com_model_t &adComModel,
                                       const InverseKinematicsModelBase *inverseKinematics, Settings settings,
                                       std::vector<std::string> jointNames, std::string baseName)
    : settings_(std::move(settings)),
      jointNames_(std::move(jointNames)),
      baseName_(std::move(baseName)),
      inverseKinematicModelPtr_(nullptr),
      kinematicModelPtr_(kinematicModel.clone()),
      adKinematicModelPtr_(adKinematicModel.clone()),
      comModelPtr_(comModel.clone()),
      adComModelPtr_(adComModel.clone()),
      problemPtr_(new ocs2::OptimalControlProblem) {
    if (inverseKinematics != nullptr) {
        inverseKinematicModelPtr_.reset(inverseKinematics->clone());
    }

    std::unique_ptr<GaitSchedule> gaitSchedule(new GaitSchedule(0.0, settings_.defaultGait_));

    std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(new SwingTrajectoryPlanner(
        settings_.swingTrajectoryPlannerSettings_, getKinematicModel(), getInverseKinematicModelPtr()));

    std::unique_ptr<TerrainModel> terrainModel(new PlanarTerrainModel(std::move(settings_.terrainPlane_)));

    // Mode schedule manager
    modeScheduleManagerPtr_ = std::make_shared<SwitchedModelModeScheduleManager>(
        std::move(gaitSchedule), std::move(swingTrajectoryPlanner), std::move(terrainModel));
    // Dynamics parameter module
    dynamicsParametersSynchronizedModulePtr_ = std::make_shared<DynamicsParametersSynchronizedModule>();
    appendToSynchronizedModules(dynamicsParametersSynchronizedModulePtr_);
}

std::unique_ptr<ocs2::PreComputation> QuadrupedInterface::createPrecomputation() const {
    return std::unique_ptr<ocs2::PreComputation>(new SwitchedModelPreComputation(
        getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), getKinematicModel(),
        getKinematicModelAd(), getComModel(), getComModelAd(), modelSettings()));
}

std::unique_ptr<ocs2::StateInputCost> QuadrupedInterface::createMotionTrackingCost() const {
    return std::unique_ptr<ocs2::StateInputCost>(
        new MotionTrackingCost(costSettings(), getKinematicModelAd(), modelSettings()));
}

std::unique_ptr<ocs2::StateCost> QuadrupedInterface::createMotionTrackingTerminalCost(matrix_t Q) const {
    return std::unique_ptr<ocs2::StateCost>(new MotionTrackingTerminalCost(std::move(Q)));
}

std::unique_ptr<ocs2::StateCost> QuadrupedInterface::createFootPlacementCost() const {
    return std::unique_ptr<ocs2::StateCost>(new FootPlacementCost(
        ocs2::RelaxedBarrierPenalty::Config(modelSettings().muFootPlacement_, modelSettings().deltaFootPlacement_)));
}

std::unique_ptr<ocs2::StateCost> QuadrupedInterface::createCollisionAvoidanceCost() const {
    return std::unique_ptr<ocs2::StateCost>(new CollisionAvoidanceCost(
        ocs2::RelaxedBarrierPenalty::Config(modelSettings().muSdf_, modelSettings().deltaSdf_)));
}

std::unique_ptr<ocs2::StateInputCost> QuadrupedInterface::createJointLimitsSoftConstraint() const {
    // Joint position constraints
    std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint> stateConstraints;
    stateConstraints.reserve(JOINT_COORDINATE_SIZE);
    for (int j = 0; j < JOINT_COORDINATE_SIZE; ++j) {
        ocs2::StateInputSoftBoxConstraint::BoxConstraint jointPositionConstraint;
        jointPositionConstraint.index = 2 * BASE_COORDINATE_SIZE + j;
        jointPositionConstraint.penaltyPtr.reset(
            new ocs2::RelaxedBarrierPenalty({modelSettings().muJointsPosition_, modelSettings().deltaJointsPosition_}));
        jointPositionConstraint.lowerBound = modelSettings().lowerJointLimits_[j];
        jointPositionConstraint.upperBound = modelSettings().upperJointLimits_[j];
        stateConstraints.push_back(std::move(jointPositionConstraint));
    }

    // Joint velocity constraints
    std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint> inputConstraints;
    inputConstraints.reserve(JOINT_COORDINATE_SIZE);
    for (int j = 0; j < JOINT_COORDINATE_SIZE; ++j) {
        ocs2::StateInputSoftBoxConstraint::BoxConstraint jointVelocityConstraint;
        jointVelocityConstraint.index = 3 * NUM_CONTACT_POINTS + j;
        jointVelocityConstraint.penaltyPtr.reset(
            new ocs2::RelaxedBarrierPenalty({modelSettings().muJointsVelocity_, modelSettings().deltaJointsVelocity_}));
        jointVelocityConstraint.lowerBound = -modelSettings().jointVelocityLimits[j];
        jointVelocityConstraint.upperBound = modelSettings().jointVelocityLimits[j];
        inputConstraints.push_back(std::move(jointVelocityConstraint));
    }

    std::unique_ptr<ocs2::StateInputSoftBoxConstraint> jointLimitSoftConstraintPtr(
        new ocs2::StateInputSoftBoxConstraint(std::move(stateConstraints), std::move(inputConstraints)));
    jointLimitSoftConstraintPtr->initializeOffset(0.0, getInitialState(), input_vector_t::Zero());
    return jointLimitSoftConstraintPtr;
}

std::unique_ptr<ocs2::StateInputCost> QuadrupedInterface::createTorqueLimitsSoftConstraint(
    const joint_coordinate_t &nominalTorques) const {
    return std::unique_ptr<ocs2::StateInputCost>(new TorqueLimitsSoftConstraint(
        modelSettings().jointTorqueLimits, {modelSettings().muJointsTorque_, modelSettings().deltaJointsTorque_},
        nominalTorques));
}

std::unique_ptr<ocs2::SystemDynamicsBase> QuadrupedInterface::createDynamics() const {
    return std::unique_ptr<ocs2::SystemDynamicsBase>(new ComKinoSystemDynamicsAd(
        getKinematicModelAd(), getComModelAd(), *getDynamicsParametersSynchronizedModulePtr(), modelSettings()));
}

std::unique_ptr<ocs2::StateInputConstraint> QuadrupedInterface::createZeroForceConstraint(size_t leg) const {
    return std::unique_ptr<ocs2::StateInputConstraint>(
        new ZeroForceConstraint(leg, *getSwitchedModelModeScheduleManagerPtr()));
}

std::unique_ptr<ocs2::StateInputConstraint> QuadrupedInterface::createFootNormalConstraint(size_t leg) const {
    return std::unique_ptr<ocs2::StateInputConstraint>(
        new FootNormalConstraint(leg, settings_.swingTrajectoryPlannerSettings_.errorGain));
}

std::unique_ptr<ocs2::StateInputConstraint> QuadrupedInterface::createEndEffectorVelocityConstraint(size_t leg) const {
    return std::unique_ptr<ocs2::StateInputConstraint>(
        new EndEffectorVelocityConstraint(leg, *getSwitchedModelModeScheduleManagerPtr()));
}

std::unique_ptr<ocs2::StateInputCost> QuadrupedInterface::createFrictionConeCost() const {
    friction_cone::Config frictionConfig(modelSettings().frictionCoefficient_, modelSettings().coneRegularization_,
                                         modelSettings().gripperForce_);
    std::unique_ptr<ocs2::PenaltyBase> penalty(
        new ocs2::RelaxedBarrierPenalty({modelSettings().muFrictionCone_, modelSettings().deltaFrictionCone_}));
    return std::unique_ptr<ocs2::StateInputCost>(
        new FrictionConeCost(std::move(frictionConfig), *getSwitchedModelModeScheduleManagerPtr(), std::move(penalty)));
}

QuadrupedInterface::Settings loadQuadrupedSettings(const std::string &pathToConfigFile) {
    QuadrupedInterface::Settings settings;
    settings.rolloutSettings_ = ocs2::rollout::loadSettings(pathToConfigFile, "rollout");
    settings.modelSettings_ = loadModelSettings(pathToConfigFile);
    settings.trackingWeights_ = loadWeightsFromFile(pathToConfigFile, "tracking_cost_weights");

    // initial state of the switched system
    ocs2::loadData::loadEigenMatrix(pathToConfigFile, "initialRobotState", settings.initialState_);

    // Gait Schedule
    const auto defaultModeSequenceTemplate =
        loadModeSequenceTemplate(pathToConfigFile, "defaultModeSequenceTemplate", false);
    settings.defaultGait_ = toGait(defaultModeSequenceTemplate);

    // Swing trajectory planner
    settings.swingTrajectoryPlannerSettings_ = loadSwingTrajectorySettings(pathToConfigFile);

    // Terrain
    settings.terrainPlane_ = loadTerrainPlane(pathToConfigFile, true);

    return settings;
}

}  // namespace tbai::mpc::quadruped
