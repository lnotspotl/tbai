#pragma once

#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_mpc/franka_mpc/FrankaModelInfo.h>
#include <tbai_mpc/franka_wbc/WbcBase.hpp>
#include <tbai_mpc/wbc/HqpSolver.hpp>

namespace tbai {
namespace mpc {
namespace franka {

    class HqpWbc : public WbcBase {
   public:

    HqpWbc(const std::string &configFile, const std::string &urdfString,
           const ocs2::franka::FrankaModelInfo &frankaInfo)
        : WbcBase(configFile, urdfString, frankaInfo, "hqpWbc.") {
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

    // Joint PD gains for motor commands
    scalar_t jointKp_;
    scalar_t jointKd_;

    HqpSolver hqpSolver_;
};

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
