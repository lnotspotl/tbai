// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <Eigen/Core>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_static/StaticController.hpp>

namespace tbai {
namespace static_ {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
StaticController::StaticController(std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr)
    : stateSubscriberPtr_(stateSubscriberPtr), alpha_(0.0), currentControllerType_("SIT"), first_(true) {
    logger_ = tbai::getLogger("tbai_static");
    loadSettings();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<MotorCommand> StaticController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    // Change to SIT controller on the first call
    if (first_) {
        first_ = false;
        changeController(initialControllerType_, currentTime);
    }

    if (alpha_ != -1.0) {
        return getInterpCommandMessage(dt);
    }

    if (currentControllerType_ == "STAND") {
        return getStandCommandMessage();
    }

    if (currentControllerType_ == "SIT") {
        return getSitCommandMessage();
    }

    TBAI_THROW("Unsupported controller type: {}. Available controllers: STAND, SIT", currentControllerType_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StaticController::changeController(const std::string &controllerType, scalar_t currentTime) {
    stateSubscriberPtr_->disable();
    TBAI_LOG_INFO(logger_, "Disabling state estimator");
    preStep(currentTime, 0.0);
    currentControllerType_ = controllerType;
    alpha_ = 0.0;
    interpFrom_ = jointAnglesFromState(state_);
    if (currentControllerType_ == "STAND") {
        interpTo_ = standJointAngles_;
    } else if (currentControllerType_ == "SIT") {
        interpTo_ = sitJointAngles_;
    } else {
        TBAI_LOG_ERROR(logger_, "Unsupported controller type: {}. Available controllers: STAND, SIT",
                       currentControllerType_);
        return;
    }

    TBAI_LOG_INFO(logger_, "Interpolating from: {}", (std::stringstream() << interpFrom_.transpose()).str());
    TBAI_LOG_INFO(logger_, "Interpolating to: {}", (std::stringstream() << interpTo_.transpose()).str());
    TBAI_LOG_INFO(logger_, "Interpolating time: {}", interpolationTime_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
bool StaticController::isSupported(const std::string &controllerType) {
    if (controllerType == "STAND" || controllerType == "SIT") {
        return true;
    }
    return false;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
scalar_t StaticController::getRate() const {
    return rate_;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StaticController::loadSettings() {
    kp_ = tbai::fromGlobalConfig<scalar_t>("static_controller/kp");
    kd_ = tbai::fromGlobalConfig<scalar_t>("static_controller/kd");
    rate_ = tbai::fromGlobalConfig<scalar_t>("static_controller/rate");

    standJointAngles_ = tbai::fromGlobalConfig<vector_t>("static_controller/stand_controller/joint_angles");
    sitJointAngles_ = tbai::fromGlobalConfig<vector_t>("static_controller/sit_controller/joint_angles");
    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
    interpolationTime_ = tbai::fromGlobalConfig<scalar_t>("static_controller/interpolation_time");
    fixedBase_ = tbai::fromGlobalConfig<bool>("fixed_base", false);

    initialControllerType_ = tbai::fromGlobalConfig<std::string>("static_controller/initial_controller_type", "SIT");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<MotorCommand> StaticController::getInterpCommandMessage(scalar_t dt) {
    // Update alpha
    alpha_ = std::min(alpha_ + dt / interpolationTime_, static_cast<scalar_t>(1.0));

    // Compute new joint angles
    auto jointAngles = (1.0 - alpha_) * interpFrom_ + alpha_ * interpTo_;

    // Finish interpolation
    if (alpha_ == 1.0) {
        alpha_ = -1.0;

        // If the current controller is STAND, enable state estimator
        if (currentControllerType_ == "STAND") {
            TBAI_LOG_INFO(logger_, "Enabling state estimator");
            stateSubscriberPtr_->enable();
        }
    }

    return packCommandMessage(jointAngles);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<MotorCommand> StaticController::getStandCommandMessage() {
    return packCommandMessage(standJointAngles_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<MotorCommand> StaticController::getSitCommandMessage() {
    return packCommandMessage(sitJointAngles_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<MotorCommand> StaticController::packCommandMessage(const vector_t &jointAngles) {
    std::vector<MotorCommand> commandArray;
    commandArray.resize(jointAngles.size());
    for (size_t i = 0; i < jointAngles.size(); ++i) {
        MotorCommand command;
        command.joint_name = jointNames_[i];
        command.desired_position = jointAngles[i];
        command.desired_velocity = 0.0;
        command.kp = kp_;
        command.kd = kd_;
        command.torque_ff = 0.0;
        commandArray[i] = command;
    }
    return commandArray;
}

}  // namespace static_
}  // namespace tbai