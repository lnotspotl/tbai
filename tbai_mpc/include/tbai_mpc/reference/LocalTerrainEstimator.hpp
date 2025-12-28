#pragma once

#include <memory>
#include <vector>

#include <ocs2_mpc/SystemObservation.h>
#include <tbai_mpc/quadruped_mpc/terrain/TerrainPlane.h>
#include <tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedKinematics.h>

namespace tbai {
namespace mpc {
namespace reference {

/**
 * Estimates local terrain plane from foothold positions.
 * This is a ROS-independent implementation.
 */
class LocalTerrainEstimator {
   public:
    /**
     * Constructor
     * @param kinematicsPtr: Shared pointer to the kinematics model
     */
    explicit LocalTerrainEstimator(std::shared_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> kinematicsPtr);

    /**
     * Updates the terrain estimate based on current footholds from observation
     * @param observation: The current system observation
     */
    void updateFootholds(const ocs2::SystemObservation& observation);

    /**
     * Gets the estimated terrain plane
     * @return The estimated terrain plane
     */
    const switched_model::TerrainPlane& getPlane() const { return terrainPlane_; }

   private:
    void updateLocalTerrainEstimate(const std::vector<switched_model::vector3_t>& footholds);

    // Local terrain estimate
    switched_model::TerrainPlane terrainPlane_;

    // Last footholds
    std::vector<switched_model::vector3_t> lastFootholds_;

    // Kinematics model
    std::shared_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> kinematicsPtr_;
};

}  // namespace reference
}  // namespace mpc
}  // namespace tbai
