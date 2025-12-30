// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_mpc/MpcController.hpp"

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/SqpSettings.h>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_mpc/quadruped_mpc/QuadrupedMpc.h>
#include <tbai_mpc/quadruped_mpc/core/MotionPhaseDefinition.h>
#include <tbai_mpc/quadruped_mpc/quadruped_interfaces/Interfaces.h>
#include <tbai_mpc/wbc/Factory.hpp>

namespace tbai {
namespace mpc {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
MpcController::MpcController(const std::string &robotName,
                             const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                             std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr,
                             std::function<scalar_t()> getCurrentTimeFunction)
    : robotName_(robotName),
      stateSubscriberPtr_(stateSubscriberPtr),
      velocityGeneratorPtr_(std::move(velocityGeneratorPtr)),
      getCurrentTimeFunction_(getCurrentTimeFunction) {
    logger_ = tbai::getLogger("mpc_controller");
    initTime_ = tbai::readInitTime();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
MpcController::~MpcController() {
    stopReferenceThread();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MpcController::initialize(const std::string &urdfString, const std::string &taskSettingsFile,
                               const std::string &frameDeclarationFile, const std::string &controllerConfigFile,
                               const std::string &targetCommandFile, scalar_t trajdt, size_t trajKnots) {
    // Create quadruped interface
    if (robotName_ == "anymal_d" || robotName_ == "anymal_b" || robotName_ == "anymal_c") {
        quadrupedInterfacePtr_ =
            anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                       anymal::frameDeclarationFromFile(frameDeclarationFile));
    } else if (robotName_ == "go2" || robotName_ == "spot") {
        quadrupedInterfacePtr_ =
            anymal::getGo2Interface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                    anymal::frameDeclarationFromFile(frameDeclarationFile));
    } else {
        TBAI_THROW("Robot {} not implemented. Available robots: anymal_d, anymal_b, anymal_c, go2, spot", robotName_);
    }

    // Create WBC
    wbcPtr_ =
        tbai::mpc::getWbcUnique(controllerConfigFile, urdfString, quadrupedInterfacePtr_->getComModel(),
                                quadrupedInterfacePtr_->getKinematicModel(), quadrupedInterfacePtr_->getJointNames());

    // Create reference trajectory generator
    auto kinematicsPtr = std::shared_ptr<switched_model::KinematicsModelBase<scalar_t>>(
        quadrupedInterfacePtr_->getKinematicModel().clone());
    referenceTrajectoryGeneratorPtr_ = std::make_unique<reference::ReferenceTrajectoryGenerator>(
        targetCommandFile, velocityGeneratorPtr_, std::move(kinematicsPtr), trajdt, trajKnots);

    // Create MRT interface
    mpcPtr_ = createMpcInterface();
    mrtPtr_ = createMrtInterface();

    tNow_ = 0.0;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::unique_ptr<ocs2::MPC_BASE> MpcController::createMpcInterface() {
    throw std::runtime_error("createMpcInterface not implemented");
    // auto sqpSettings = ocs2::sqp::Settings();
    // auto mpcSettings = ocs2::mpc::Settings();
    // return getSqpMpc(*quadrupedInterfacePtr_, mpcSettings, sqpSettings);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::unique_ptr<ocs2::MRT_BASE> MpcController::createMrtInterface() {
    // Default implementation: create MPC_MRT_Interface with threaded execution
    return std::make_unique<ocs2::MPC_MRT_Interface>(*mpcPtr_, true);  // threaded = true
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<MotorCommand> MpcController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    // For MPC_MRT_Interface, we need to call updatePolicy to get latest solution
    mrtPtr_->spinMRT();
    mrtPtr_->updatePolicy();

    tNow_ = getCurrentTimeFunction_() - initTime_;

    auto observation = generateSystemObservation();

    ocs2::vector_t desiredState;
    ocs2::vector_t desiredInput;
    size_t desiredMode;
    mrtPtr_->evaluatePolicy(tNow_, observation.state, desiredState, desiredInput, desiredMode);

    constexpr ocs2::scalar_t time_eps = 1e-4;
    ocs2::vector_t dummyState;
    ocs2::vector_t dummyInput;
    size_t dummyMode;
    mrtPtr_->evaluatePolicy(tNow_ + time_eps, observation.state, dummyState, dummyInput, dummyMode);

    ocs2::vector_t joint_accelerations = (dummyInput.tail<12>() - desiredInput.tail<12>()) / time_eps;

    auto commands = wbcPtr_->getMotorCommands(tNow_, observation.state, observation.input, observation.mode,
                                              desiredState, desiredInput, desiredMode, joint_accelerations, isStable_);

    timeSinceLastMpcUpdate_ += dt;
    if (timeSinceLastMpcUpdate_ >= 1.0 / mpcRate_) {
        setObservation();
    }

    return commands;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MpcController::referenceThreadLoop() {
    std::cerr << "Reference trajectory generator reset" << std::endl;
    TBAI_LOG_WARN(logger_, "Reference trajectory generator reset");
    referenceTrajectoryGeneratorPtr_->reset();
    std::cerr << "Reference trajectory generator reset2" << std::endl;
    TBAI_LOG_WARN(logger_, "Reference trajectory generator reset2");

    // Wait for initial observation
    stateSubscriberPtr_->waitTillInitialized();
    ocs2::SystemObservation observation = generateSystemObservation();
    referenceTrajectoryGeneratorPtr_->updateObservation(observation);
    TBAI_LOG_WARN(logger_, "Reference trajectory generator initialized");

    // Reference loop
    auto sleepDuration = std::chrono::milliseconds(static_cast<int>(1000.0 / referenceThreadRate_));
    while (!stopReferenceThread_) {
        std::cerr << "Reference thread loop" << std::endl;
        TBAI_LOG_WARN_THROTTLE(logger_, 0.005, "Reference thread loop");
        // Generate and publish reference trajectory
        auto observation = generateSystemObservation();
        auto targetTrajectories = referenceTrajectoryGeneratorPtr_->generateReferenceTrajectory(tNow_, observation);
        mrtPtr_->setTargetTrajectories(targetTrajectories);

        TBAI_LOG_INFO_THROTTLE(logger_, 5.0, "Publishing reference");
        std::this_thread::sleep_for(sleepDuration);
        observation = generateSystemObservation();
        referenceTrajectoryGeneratorPtr_->updateObservation(observation);
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
bool MpcController::checkStability() const {
    if (!isStable_) {
        TBAI_LOG_ERROR(logger_, "Mpc is unstable");
        return false;
    }

    scalar_t roll = state_.x[0];
    if (roll >= 1.57 || roll <= -1.57) {
        return false;
    }
    scalar_t pitch = state_.x[1];
    if (pitch >= 1.57 || pitch <= -1.57) {
        return false;
    }
    return true;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MpcController::changeController(const std::string &controllerType, scalar_t currentTime) {
    preStep(currentTime, 0.0);

    if (!mrt_initialized_ || currentTime + 0.1 > mrtPtr_->getPolicy().timeTrajectory_.back()) {
        resetMpc();
        mrt_initialized_ = true;
    }
    tNow_ = currentTime - initTime_;

    // Start reference thread
    startReferenceThread();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MpcController::startReferenceThread() {
    // Stop existing thread if running
    stopReferenceThread();

    TBAI_LOG_WARN(logger_, "Starting reference thread");
    stopReferenceThread_ = false;
    referenceThread_ = std::thread(&MpcController::referenceThreadLoop, this);
    TBAI_LOG_WARN(logger_, "Reference thread started");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MpcController::stopReferenceThread() {
    stopReferenceThread_ = true;
    if (referenceThread_.joinable()) {
        referenceThread_.join();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
bool MpcController::isSupported(const std::string &controllerType) {
    return controllerType == "WBC";
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MpcController::resetMpc() {
    // Generate initial observation
    stateSubscriberPtr_->waitTillInitialized();
    auto initialObservation = generateSystemObservation();
    const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {initialObservation.state},
                                                          {initialObservation.input});
    mrtPtr_->resetMpcNode(initTargetTrajectories);

    while (!mrtPtr_->initialPolicyReceived()) {
        TBAI_LOG_INFO(logger_, "Waiting for initial policy...");
        initialObservation = generateSystemObservation();
        mrtPtr_->setCurrentObservation(initialObservation);
        mrtPtr_->spinMRT();
        mrtPtr_->updatePolicy();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    TBAI_LOG_INFO(logger_, "Initial policy received.");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MpcController::setObservation() {
    mrtPtr_->setCurrentObservation(generateSystemObservation());
    timeSinceLastMpcUpdate_ = 0.0;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ocs2::SystemObservation MpcController::generateSystemObservation() const {
    auto state = stateSubscriberPtr_->getLatestState();
    const tbai::vector_t &rbdState = state.x;

    // Set observation time
    ocs2::SystemObservation observation;
    observation.time = state.timestamp - initTime_;

    // Set mode
    const std::vector<bool> &contactFlags = state.contactFlags;
    std::array<bool, 4> contactFlagsArray = {contactFlags[0], contactFlags[1], contactFlags[2], contactFlags[3]};
    observation.mode = switched_model::stanceLeg2ModeNumber(contactFlagsArray);

    // Set state
    observation.state = rbdState.head<3 + 3 + 3 + 3 + 12>();

    // Swap LH and RF
    std::swap(observation.state(3 + 3 + 3 + 3 + 3 + 0), observation.state(3 + 3 + 3 + 3 + 3 + 3));
    std::swap(observation.state(3 + 3 + 3 + 3 + 3 + 1), observation.state(3 + 3 + 3 + 3 + 3 + 4));
    std::swap(observation.state(3 + 3 + 3 + 3 + 3 + 2), observation.state(3 + 3 + 3 + 3 + 3 + 5));

    // Set input
    observation.input.setZero(24);
    observation.input.tail<12>() = rbdState.tail<12>();

    // Swap LH and RF
    std::swap(observation.input(12 + 3), observation.input(12 + 6));
    std::swap(observation.input(12 + 4), observation.input(12 + 7));
    std::swap(observation.input(12 + 5), observation.input(12 + 8));

    return observation;
}

}  // namespace mpc
}  // namespace tbai
