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
#include <tbai_np3o/HistoryBuffer.hpp>
#include <tbai_np3o/State.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>
#include <torch/script.h>

namespace tbai {

using namespace torch::indexing;  // NOLINT
using torch::jit::script::Module;

class Np3oController : public tbai::Controller {
   public:
    Np3oController(const std::string &urdfPathOrString,
                   const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                   const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen);

    Np3oController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                   const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen);

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override { state_ = stateSubscriberPtr_->getLatestState(); }

    bool isSupported(const std::string &controllerType) override;

    void stopController() override {}

    scalar_t getRate() const override { return 50.0; }

    bool checkStability() const override;

    void postStep(scalar_t currentTime, scalar_t dt) override {}

    std::string getName() const override { return "NP3O"; }

   protected:
    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    scalar_t kp_;
    scalar_t kd_;

    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen_;

    Module model_;

    at::Tensor forward(at::Tensor &obsProprioceptive, at::Tensor &obsHistory) {
        torch::NoGradGuard no_grad;
        obsProprioceptive = obsProprioceptive.reshape({1, -1});
        obsHistory = obsHistory.reshape({1, 10, -1});  // TODO: history size is hardcoded here
        at::Tensor action = model_.forward({obsProprioceptive.to(torch::kHalf), obsHistory.to(torch::kHalf)}).toTensor().reshape({-1}).to(torch::kFloat);
        return action;
    }

    void setupPinocchioModel(const std::string &urdfString);
    std::vector<tbai::MotorCommand> getMotorCommands(const vector_t &jointAngles);
    pinocchio::Model pinocchioModel_;
    pinocchio::Data pinocchioData_;
    np3o::State getNp3oState();

    tbai::vector_t getObsProprioceptive(const np3o::State &state, scalar_t currentTime, scalar_t dt);
    tbai::vector_t getObsHistory();
    void updateObsHistory(const tbai::vector_t &observation);


    // base angular velocity - 3
    // projected gravity - 3
    // command - 3
    // dof residuals - 12
    // dof velocities - 12
    // last action - 12
    constexpr static int BASE_ANGULAR_VELOCITY_START_INDEX = 0;
    constexpr static int PROJECTED_GRAVITY_START_INDEX = 3;
    constexpr static int COMMAND_START_INDEX = 6;
    constexpr static int DOF_RESIDUALS_START_INDEX = 9;
    constexpr static int DOF_VELOCITIES_START_INDEX = 21;
    constexpr static int LAST_ACTION_START_INDEX = 33;

    void fillBaseAngularVelocity(vector_t &input, const np3o::State &state);
    void fillProjectedGravity(vector_t &input, const np3o::State &state);
    void fillCommand(vector_t &input, const np3o::State &state, scalar_t currentTime, scalar_t dt);
    void fillJointResiduals(vector_t &input, const np3o::State &state);
    void fillJointVelocities(vector_t &input, const np3o::State &state);
    void fillLastAction(vector_t &input, const np3o::State &state);

    std::vector<std::string> jointNames_;

    vector_t lastAction_;
    vector_t filterLastAction_;

    np3o::HistoryBuffer historyBuffer_;

    std::shared_ptr<spdlog::logger> logger_;

    vector_t defaultJointAngles_;

    scalar_t gaitIndex_;

    bool useActionFilter_ = false;

    State state_;
};

}  // namespace tbai