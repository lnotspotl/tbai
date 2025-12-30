#include "tbai_mpc/reference/ReferenceTrajectoryGenerator.hpp"

#include <ocs2_core/misc/LoadData.h>
#include <tbai_mpc/quadruped_mpc/core/Rotations.h>
#include <tbai_mpc/quadruped_mpc/core/SwitchedModel.h>
#include <tbai_mpc/quadruped_mpc/quadruped_commands/ReferenceExtrapolation.h>

namespace tbai {
namespace mpc {
namespace reference {

using namespace switched_model;

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ReferenceTrajectoryGenerator::ReferenceTrajectoryGenerator(
    const std::string &configFile, std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr,
    std::shared_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> kinematicsPtr, ocs2::scalar_t trajdt,
    size_t trajKnots)
    : velocityGeneratorPtr_(std::move(velocityGeneratorPtr)),
      terrainEstimator_(std::move(kinematicsPtr)),
      trajdt_(trajdt),
      trajKnots_(trajKnots) {
    defaultJointState_.setZero(12);
    loadSettings(configFile);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ocs2::TargetTrajectories ReferenceTrajectoryGenerator::generateReferenceTrajectory(
    ocs2::scalar_t currentTime, const ocs2::SystemObservation &observation) {
    // Update observation
    updateObservation(observation);

    // Get base reference trajectory using terrain plane
    BaseReferenceTrajectory baseReferenceTrajectory = generateExtrapolatedBaseReference(
        getBaseReferenceHorizon(), getBaseReferenceState(), getBaseReferenceCommand(currentTime), getTerrainPlane());

    // Generate target trajectory
    ocs2::scalar_array_t desiredTimeTrajectory = std::move(baseReferenceTrajectory.time);
    const size_t N = desiredTimeTrajectory.size();
    ocs2::vector_array_t desiredStateTrajectory(N);
    ocs2::vector_array_t desiredInputTrajectory(N, ocs2::vector_t::Zero(INPUT_DIM));

    for (size_t i = 0; i < N; ++i) {
        ocs2::vector_t state = ocs2::vector_t::Zero(STATE_DIM);

        // base orientation
        state.head<3>() = baseReferenceTrajectory.eulerXyz[i];

        auto Rt = switched_model::rotationMatrixOriginToBase(baseReferenceTrajectory.eulerXyz[i]);

        // base position
        state.segment<3>(3) = baseReferenceTrajectory.positionInWorld[i];

        // base angular velocity
        state.segment<3>(6) = Rt * baseReferenceTrajectory.angularVelocityInWorld[i];

        // base linear velocity
        state.segment<3>(9) = Rt * baseReferenceTrajectory.linearVelocityInWorld[i];

        // joint angles
        state.segment<12>(12) = defaultJointState_;

        desiredStateTrajectory[i] = std::move(state);
    }

    return ocs2::TargetTrajectories(std::move(desiredTimeTrajectory), std::move(desiredStateTrajectory),
                                    std::move(desiredInputTrajectory));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceTrajectoryGenerator::updateObservation(const ocs2::SystemObservation &observation) {
    latestObservation_ = observation;
    initialized_ = true;

    // Update terrain estimate if not using external terrain
    if (!useExternalTerrain_) {
        terrainEstimator_.updateFootholds(observation);
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceTrajectoryGenerator::setTerrainPlane(const switched_model::TerrainPlane &plane) {
    useExternalTerrain_ = true;
    externalTerrainPlane_ = plane;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
const switched_model::TerrainPlane &ReferenceTrajectoryGenerator::getTerrainPlane() const {
    if (useExternalTerrain_) {
        return externalTerrainPlane_;
    }
    return terrainEstimator_.getPlane();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceHorizon ReferenceTrajectoryGenerator::getBaseReferenceHorizon() const {
    return {trajdt_, trajKnots_};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceCommand ReferenceTrajectoryGenerator::getBaseReferenceCommand(ocs2::scalar_t time) const {
    auto velocityCommand = velocityGeneratorPtr_->getReferenceVelocity(time, trajdt_);
    return {velocityCommand.velocity_x, velocityCommand.velocity_y, velocityCommand.yaw_rate, comHeight_};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceState ReferenceTrajectoryGenerator::getBaseReferenceState() const {
    ocs2::scalar_t observationTime = latestObservation_.time;
    Eigen::Vector3d positionInWorld = latestObservation_.state.segment<3>(3);
    Eigen::Vector3d eulerXyz = latestObservation_.state.head<3>();
    return {observationTime, positionInWorld, eulerXyz};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceTrajectoryGenerator::loadSettings(const std::string &configFile) {
    // Load target COM height
    ocs2::loadData::loadCppDataType<ocs2::scalar_t>(configFile, "comHeight", comHeight_);

    // Load default joint angles
    ocs2::loadData::loadEigenMatrix(configFile, "defaultJointState", defaultJointState_);
}

}  // namespace reference
}  // namespace mpc
}  // namespace tbai
