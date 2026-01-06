#include "tbai_mpc/quadruped_mpc/quadruped_reference/LocalTerrainEstimator.hpp"

#include <tbai_mpc/quadruped_mpc/core/MotionPhaseDefinition.h>
#include <tbai_mpc/quadruped_mpc/core/SwitchedModel.h>
#include <tbai_mpc/quadruped_mpc/terrain/PlaneFitting.h>

namespace tbai {
namespace mpc {
namespace reference {

using namespace switched_model;

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
LocalTerrainEstimator::LocalTerrainEstimator(
    std::shared_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> kinematicsPtr)
    : kinematicsPtr_(std::move(kinematicsPtr)) {
    lastFootholds_.resize(4);
    for (size_t i = 0; i < 4; i++) {
        lastFootholds_[i] = vector3_t::Zero();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void LocalTerrainEstimator::updateFootholds(const ocs2::SystemObservation &observation) {
    // Base position
    auto basePose = getBasePose(observation.state);

    // Joint positions
    auto jointPositions = getJointPositions(observation.state);

    // Compute forward kinematics
    auto footholds = kinematicsPtr_->feetPositionsInOriginFrame(basePose, jointPositions);

    // contact flags
    contact_flag_t contactFlags = modeNumber2StanceLeg(observation.mode);

    // Update last footholds
    for (size_t i = 0; i < 4; i++) {
        if (contactFlags[i]) {
            lastFootholds_[i] = footholds[i];
        }
    }

    updateLocalTerrainEstimate(lastFootholds_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void LocalTerrainEstimator::updateLocalTerrainEstimate(const std::vector<vector3_t> &footholds) {
    const auto normalAndPosition = estimatePlane(footholds);
    terrainPlane_ = TerrainPlane(normalAndPosition.position,
                                 orientationWorldToTerrainFromSurfaceNormalInWorld(normalAndPosition.normal));
}

}  // namespace reference
}  // namespace mpc
}  // namespace tbai
