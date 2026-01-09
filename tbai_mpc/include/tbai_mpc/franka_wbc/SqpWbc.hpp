#pragma once

#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_mpc/franka_mpc/FrankaModelInfo.h>
#include <tbai_mpc/franka_wbc/WbcBase.hpp>
#include <tbai_mpc/wbc/SqpSolver.hpp>

namespace tbai {
namespace mpc {
namespace franka {

class SqpWbc : public WbcBase {
   public:
    SqpWbc(const std::string &configFile, const std::string &urdfString,
           const ocs2::franka::FrankaModelInfo &frankaInfo)
        : WbcBase(configFile, urdfString, frankaInfo, "sqpWbc.") {
        loadSettings(configFile);
    }

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, const vector_t &currentState,
                                                     const vector_t &currentInput, const vector_t &desiredState,
                                                     const vector_t &desiredInput,
                                                     const vector_t &desiredJointAcceleration,
                                                     const vector_t &desiredEEPosition,
                                                     const vector_t &desiredEEOrientation, bool &isStable) override;

   private:
    void loadSettings(const std::string &configFile);

    // Task weights
    scalar_t weightEEPosition_;
    scalar_t weightEEOrientation_;
    scalar_t weightJointAcceleration_;
    scalar_t weightJointCentering_;

    // Joint PD gains for motor commands
    scalar_t jointKp_;
    scalar_t jointKd_;

    SqpSolver sqpSolver_;
};

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
