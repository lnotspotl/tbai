#pragma once

#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_mpc/quadruped_mpc/core/SwitchedModel.h>
#include <tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedCom.h>
#include <tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedKinematics.h>
#include <tbai_mpc/quadruped_wbc/WbcBase.hpp>
#include <tbai_mpc/wbc/HqpSolver.hpp>

namespace tbai {
namespace mpc {

class HqpWbc : public WbcBase {
   public:
    HqpWbc(const std::string &configFile, const std::string &urdfString,
           const switched_model::ComModelBase<scalar_t> &comModel,
           const switched_model::KinematicsModelBase<scalar_t> &kinematics, const std::vector<std::string> &jointNames)
        : WbcBase(configFile, urdfString, comModel, kinematics, "hqpWbc."), jointNames_(jointNames) {
        loadSettings(configFile);
    }

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, const vector_t &currentState,
                                                     const vector_t &currentInput, const size_t currentMode,
                                                     const vector_t &desiredState, const vector_t &desiredInput,
                                                     const size_t desiredMode, const vector_t &desiredJointAcceleration,
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
    std::vector<std::string> jointNames_;
};

}  // namespace mpc
}  // namespace tbai
