#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h"
#include "tbai_mpc/quadruped_arm_mpc/dynamics/ComKinoDynamicsParameters.h"
#include "tbai_mpc/quadruped_arm_mpc/foot_planner/SwingTrajectoryPlanner.h"
#include "tbai_mpc/quadruped_arm_mpc/logic/GaitSchedule.h"
#include "tbai_mpc/quadruped_arm_mpc/terrain/TerrainPlane.h"

namespace tbai::mpc::quadruped_arm {

/**
 * Manages the Target Trajectories and ModeSchedule for switched model.
 */
class SwitchedModelModeScheduleManager : public ocs2::ReferenceManager {
 public:
  SwitchedModelModeScheduleManager(std::unique_ptr<GaitSchedule> gaitSchedule, std::unique_ptr<SwingTrajectoryPlanner> swingTrajectory,
                                   std::unique_ptr<TerrainModel> terrainModel);

  ~SwitchedModelModeScheduleManager() override = default;

  contact_flag_t getContactFlags(scalar_t time) const;

  ocs2::Synchronized<GaitSchedule>& getGaitSchedule() { return gaitSchedule_; }
  const ocs2::Synchronized<GaitSchedule>& getGaitSchedule() const { return gaitSchedule_; }

  const SwingTrajectoryPlanner& getSwingTrajectoryPlanner() const { return *swingTrajectoryPtr_; }

  ocs2::Synchronized<TerrainModel>& getTerrainModel() { return terrainModel_; }
  const ocs2::Synchronized<TerrainModel>& getTerrainModel() const { return terrainModel_; }

 private:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, ocs2::TargetTrajectories& targetTrajectories,
                        ocs2::ModeSchedule& modeSchedule) override;

  ocs2::Synchronized<GaitSchedule> gaitSchedule_;
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
  ocs2::Synchronized<TerrainModel> terrainModel_;
};

}  // namespace tbai::mpc::quadruped_arm
