#pragma once

#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_g1_rl/State.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

namespace tbai {

class G1RlController : public tbai::Controller {
   public:
    G1RlController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                   const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen);

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override { state_ = stateSubscriberPtr_->getLatestState(); }

    bool isSupported(const std::string &controllerType) override;

    void stopController() override {}

    scalar_t getRate() const override { return 50.0; }

    bool checkStability() const override;

    void postStep(scalar_t currentTime, scalar_t dt) override {}

    std::string getName() const override { return "G1RL"; }

   protected:
    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    scalar_t kp_;
    scalar_t kd_;

    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen_;

    void setupPinocchioModel(const std::string &urdfString);
    std::vector<tbai::MotorCommand> getMotorCommands(const vector_t &jointAngles);
    g1_rl::State getG1RlState();

    tbai::vector_t getObsProprioceptive(const g1_rl::State &state, scalar_t currentTime, scalar_t dt);
    tbai::vector_t getObsHistory();
    void updateObsHistory(const tbai::vector_t &observation);

    // base angular velocity - 3
    // projected gravity - 3
    // dof residuals - 29
    // dof velocities - 29
    // last action - 29
    // command - 3
    constexpr static int BASE_ANGULAR_VELOCITY_START_INDEX = 0;
    constexpr static int PROJECTED_GRAVITY_START_INDEX = BASE_ANGULAR_VELOCITY_START_INDEX + 3;
    constexpr static int DOF_RESIDUALS_START_INDEX = PROJECTED_GRAVITY_START_INDEX + 3;
    constexpr static int DOF_VELOCITIES_START_INDEX = DOF_RESIDUALS_START_INDEX + 29;
    constexpr static int LAST_ACTION_START_INDEX = DOF_VELOCITIES_START_INDEX + 29;
    constexpr static int COMMAND_START_INDEX = LAST_ACTION_START_INDEX + 29;

    void fillBaseAngularVelocity(vector_t &input, const g1_rl::State &state);
    void fillProjectedGravity(vector_t &input, const g1_rl::State &state);
    void fillJointResiduals(vector_t &input, const g1_rl::State &state);
    void fillJointVelocities(vector_t &input, const g1_rl::State &state);
    void fillLastAction(vector_t &input, const g1_rl::State &state);
    void fillCommand(vector_t &input, const g1_rl::State &state, scalar_t currentTime, scalar_t dt);

    std::vector<std::string> jointNames_;

    vector_t lastAction_;
    vector_t filterLastAction_;

    std::shared_ptr<spdlog::logger> logger_;

    vector_t defaultJointAngles_;

    scalar_t gaitIndex_;

    bool useActionFilter_ = false;

    State state_;
    g1_rl::State g1RlState_;
};

}  // namespace tbai