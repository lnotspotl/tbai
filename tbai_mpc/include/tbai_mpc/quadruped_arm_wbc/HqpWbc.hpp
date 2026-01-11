#pragma once

#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedCom.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedKinematics.h>
#include <tbai_mpc/quadruped_arm_wbc/WbcBase.hpp>
#include <tbai_mpc/wbc/HqpSolver.hpp>

namespace tbai {
namespace mpc {

class HqpWbc : public WbcBase {
   public:
    HqpWbc(const std::string &configFile, const std::string &urdfString,
           const tbai::mpc::quadruped_arm::ComModelBase<scalar_t> &comModel,
           const tbai::mpc::quadruped_arm::KinematicsModelBase<scalar_t> &kinematics)
        : WbcBase(configFile, urdfString, comModel, kinematics, "hqpWbc.") {
        loadSettings(configFile);
    }

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, const vector_t &currentState,
                                                     const vector_t &currentInput, const size_t currentMode,
                                                     const vector_t &desiredState, const vector_t &desiredInput,
                                                     const size_t desiredMode, const vector_t &desiredJointAcceleration,
                                                     const vector_t &desiredArmEEPosition,
                                                     const vector_t &desiredArmEEOrientation,
                                                     bool &isStable) override;

   private:
    void loadSettings(const std::string &configFile);

    // joint kp and kd for swing legs
    scalar_t jointSwingKp_;
    scalar_t jointSwingKd_;

    // joint kp and kd for stance legs
    scalar_t jointStanceKp_;
    scalar_t jointStanceKd_;

    HqpSolver hqpSolver_;
};

}  // namespace mpc
}  // namespace tbai
