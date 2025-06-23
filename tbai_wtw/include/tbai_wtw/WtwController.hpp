#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <string>

#include <Eigen/Dense>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>
#include <torch/script.h>
#include <tbai_wtw/State.hpp>

namespace tbai {

using namespace torch::indexing;  // NOLINT
using torch::jit::script::Module;

class WtwController : public tbai::Controller {
   public:
    WtwController(const std::string &urdfPathOrString, const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                  const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen);

    WtwController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                  const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen);

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    bool isSupported(const std::string &controllerType) override;

    void stopController() override {}

    scalar_t getRate() const override { return 50.0; }

    bool checkStability() const override;

    virtual void atPositions(matrix_t &positions) = 0;

   protected:
    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    scalar_t kp_;
    scalar_t kd_;

    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen_;

    Module model_;

    void setupPinocchioModel(const std::string &urdfString);
    std::vector<tbai::MotorCommand> getMotorCommands(const vector_t &jointAngles);
    pinocchio::Model pinocchioModel_;
    pinocchio::Data pinocchioData_;
    State getWtwState();

    at::Tensor getNNInput(const State &state, scalar_t currentTime, scalar_t dt);

    constexpr static int GRAVITY_START_INDEX = 0;
    constexpr static int COMMAND_START_INDEX = 3;
    constexpr static int JOINT_RESIDUALS_START_INDEX = 18;
    constexpr static int JOINT_VELOCITIES_START_INDEX = 30;
    constexpr static int LAST_ACTION_START_INDEX = 42;
    constexpr static int LAST_LAST_ACTION_START_INDEX = 54;
    constexpr static int CLOCK_INPUTS_START_INDEX = 66;

    // Observations:
// gravity - 3
// commands - 15
// dof residuals - 12
// dof velocities - 12
// last action - 12
// last last action - 12
// clock inputs - 4

    void fillGravity(vector_t &input, const State &state);
    void fillCommand(vector_t &input, const State &state, scalar_t currentTime, scalar_t dt);
    void fillJointResiduals(vector_t &input, const State &state);
    void fillJointVelocities(vector_t &input, const State &state);
    void fillLastAction(vector_t &input, const State &state);
    void fillLastLastAction(vector_t &input, const State &state);
    void fillClockInputs(vector_t &input, scalar_t currentTime, scalar_t dt);

    std::vector<std::string> jointNames_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace tbai