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
#include <tbai_np3o/HistoryBuffer.hpp>
#include <tbai_np3o/State.hpp>
#include <torch/script.h>

namespace tbai {

using namespace torch::indexing;  // NOLINT
using torch::jit::script::Module;

class Np3oController : public tbai::Controller {
   public:
    Np3oController(const std::string &urdfPathOrString, const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                  const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen);

    Np3oController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                  const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen);

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    bool isSupported(const std::string &controllerType) override;

    void stopController() override {}

    scalar_t getRate() const override { return 50.0; }

    bool checkStability() const override;

    void postStep(scalar_t currentTime, scalar_t dt) override {}

    std::string getName() const override { return "WTW"; }

    virtual void atPositions(matrix_t &positions) = 0;

   protected:
    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    scalar_t kp_;
    scalar_t kd_;

    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen_;

    Module adaptationModule_;
    Module bodyModule_;

    at::Tensor forward(at::Tensor &input) {
        torch::NoGradGuard no_grad;
        input = input.reshape({1, -1});
        auto latent = adaptationModule_.forward({input}).toTensor();
        at::Tensor concatenated = torch::cat({input, latent}, -1);
        at::Tensor action = bodyModule_.forward({concatenated}).toTensor();
        return action;
    }

    void setupPinocchioModel(const std::string &urdfString);
    std::vector<tbai::MotorCommand> getMotorCommands(const vector_t &jointAngles);
    pinocchio::Model pinocchioModel_;
    pinocchio::Data pinocchioData_;
    wtw::State getWtwState();

    at::Tensor getNNInput(const wtw::State &state, scalar_t currentTime, scalar_t dt);

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

    void fillGravity(vector_t &input, const wtw::State &state);
    void fillCommand(vector_t &input, const wtw::State &state, scalar_t currentTime, scalar_t dt);
    void fillJointResiduals(vector_t &input, const wtw::State &state);
    void fillJointVelocities(vector_t &input, const wtw::State &state);
    void fillLastAction(vector_t &input, const wtw::State &state);
    void fillLastLastAction(vector_t &input, const wtw::State &state);
    void fillClockInputs(vector_t &input, scalar_t currentTime, scalar_t dt);

    std::vector<std::string> jointNames_;

    vector_t lastAction_;
    vector_t lastLastAction_;

    wtw::HistoryBuffer historyBuffer_;

    std::shared_ptr<spdlog::logger> logger_;

    vector_t defaultJointAngles_;

    scalar_t gaitIndex_;

    State state_;
};

}  // namespace tbai