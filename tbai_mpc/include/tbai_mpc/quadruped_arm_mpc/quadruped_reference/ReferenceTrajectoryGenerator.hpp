#pragma once

#include <memory>
#include <string>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_commands/ReferenceExtrapolation.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedKinematics.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_reference/LocalTerrainEstimator.hpp>
#include <tbai_mpc/quadruped_arm_mpc/terrain/TerrainPlane.h>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

namespace tbai {
namespace mpc {
namespace reference {

using tbai::mpc::quadruped_arm::BaseReferenceCommand;
using tbai::mpc::quadruped_arm::BaseReferenceHorizon;
using tbai::mpc::quadruped_arm::BaseReferenceState;

/**
 * ROS-independent reference trajectory generator.
 * Uses TerrainPlane for terrain-aware trajectory generation.
 */
class ReferenceTrajectoryGenerator {
   public:
    /**
     * Constructor
     * @param configFile: Path to the target command configuration file
     * @param velocityGeneratorPtr: Reference velocity generator
     * @param kinematicsPtr: Kinematics model for terrain estimation
     * @param trajdt: Time step for reference trajectory (default: 0.1)
     * @param trajKnots: Number of knots in reference trajectory (default: 20)
     */
    ReferenceTrajectoryGenerator(
        const std::string &configFile,
        std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr,
        std::shared_ptr<tbai::mpc::quadruped_arm::KinematicsModelBase<ocs2::scalar_t>> kinematicsPtr,
        ocs2::scalar_t trajdt = 0.1, size_t trajKnots = 20);

    virtual ~ReferenceTrajectoryGenerator() = default;

    /**
     * Generates a reference trajectory based on current observation and time
     * @param currentTime: Current time
     * @param observation: Current system observation
     * @return Generated target trajectories
     */
    virtual ocs2::TargetTrajectories generateReferenceTrajectory(ocs2::scalar_t currentTime,
                                                                 const ocs2::SystemObservation &observation);

    /**
     * Updates the observation and terrain estimate
     * @param observation: Current system observation
     */
    void updateObservation(const ocs2::SystemObservation &observation);

    /**
     * Sets an external terrain plane (bypasses internal estimation)
     * @param plane: The terrain plane to use
     */
    void setTerrainPlane(const tbai::mpc::quadruped_arm::TerrainPlane &plane);

    /**
     * Gets the current terrain plane
     * @return The current terrain plane
     */
    const tbai::mpc::quadruped_arm::TerrainPlane &getTerrainPlane() const;

    /**
     * Gets the latest observation
     * @return The latest observation
     */
    const ocs2::SystemObservation &getLatestObservation() const { return latestObservation_; }

    /**
     * Checks if a valid observation has been received
     * @return True if observation is initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Resets the generator state
     */
    void reset() { initialized_ = false; }

   protected:
    BaseReferenceHorizon getBaseReferenceHorizon() const;
    BaseReferenceCommand getBaseReferenceCommand(ocs2::scalar_t time) const;
    BaseReferenceState getBaseReferenceState() const;

    void loadSettings(const std::string &configFile);

    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr_;
    LocalTerrainEstimator terrainEstimator_;

    ocs2::vector_t defaultJointState_;
    ocs2::scalar_t comHeight_;
    ocs2::scalar_t trajdt_;  // timestep
    size_t trajKnots_;       // number of timesteps in reference horizon

    ocs2::SystemObservation latestObservation_;
    bool initialized_ = false;

    // External terrain plane (if set, bypasses internal estimation)
    bool useExternalTerrain_ = false;
    tbai::mpc::quadruped_arm::TerrainPlane externalTerrainPlane_;
};

}  // namespace reference
}  // namespace mpc
}  // namespace tbai
