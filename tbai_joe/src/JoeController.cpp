// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_joe/JoeController.hpp"

#include <string>
#include <vector>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_sqp/SqpSettings.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_mpc/quadruped_mpc/QuadrupedMpc.h>
#include <tbai_mpc/quadruped_mpc/core/MotionPhaseDefinition.h>
#include <tbai_mpc/quadruped_mpc/quadruped_commands/TerrainAdaptation.h>
#include <tbai_mpc/quadruped_mpc/terrain/PlaneFitting.h>

namespace tbai {
namespace joe {

namespace LinearInterpolation = ocs2::LinearInterpolation;
using namespace switched_model;

JoeController::JoeController(const std::string &robotName,
                             const std::shared_ptr<tbai::StateSubscriber> &stateSubscriber,
                             std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGenerator,
                             std::function<scalar_t()> getCurrentTimeFunction)
    : stateSubscriberPtr_(stateSubscriber),
      refVelGen_(std::move(velocityGenerator)),
      getCurrentTimeFunction_(std::move(getCurrentTimeFunction)),
      robotName_(robotName) {
    logger_ = tbai::getLogger("joe_controller");
    initTime_ = tbai::readInitTime();
}

JoeController::~JoeController() {
    stopReferenceThread();
}

void JoeController::initialize(const std::string &urdfString, const std::string &taskSettingsFile,
                               const std::string &frameDeclarationFile, const std::string &joeModelPath) {
    // Load default joint angles
    TBAI_LOG_INFO(logger_, "Loading default joint angles");
    defaultJointAngles_ = tbai::fromGlobalConfig<vector_t>("static_controller/stand_controller/joint_angles");

    // Load joint names
    TBAI_LOG_INFO(logger_, "Loading joint names");
    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Load JOE model
    try {
        TBAI_LOG_INFO(logger_, "Loading torch model from: {}", joeModelPath);
        joeModel_ = torch::jit::load(joeModelPath);
    } catch (const c10::Error &e) {
        std::cerr << "Could not load model from: " << joeModelPath << std::endl;
        throw std::runtime_error("Could not load model");
    }
    TBAI_LOG_INFO(logger_, "Torch model loaded");

    // Do basic ff check
    torch::Tensor input = torch::empty({MODEL_INPUT_SIZE});
    for (int i = 0; i < MODEL_INPUT_SIZE; ++i) input[i] = static_cast<float>(i);
    torch::Tensor output = joeModel_.forward({input.view({1, -1})}).toTensor().view({-1});

    horizon_ = 1.0;
    mpcRate_ = 30;
    pastAction_ = vector_t().setZero(12);

    // Initialize quadruped interface
    TBAI_LOG_INFO(logger_, "Initializing quadruped interface");
    quadrupedInterface_ = anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                                     anymal::frameDeclarationFromFile(frameDeclarationFile));
    auto &quadrupedInterface = *quadrupedInterface_;
    comModel_.reset(quadrupedInterface.getComModel().clone());
    kinematicsModel_.reset(quadrupedInterface.getKinematicModel().clone());

    // Create MPC first (needed by MRT), then MRT
    mpc_ = createMpcInterface();
    mrt_ = createMrtInterface();

    TBAI_LOG_INFO(logger_, "Initialization done");
    TBAI_LOG_INFO(logger_, "Default joint angles: {}", (std::stringstream() << defaultJointAngles_.transpose()).str());
}

std::unique_ptr<ocs2::MRT_BASE> JoeController::createMrtInterface() {
    // Default: create local MPC_MRT_Interface with threaded execution
    return std::make_unique<ocs2::MPC_MRT_Interface>(*mpc_, true);
}

std::unique_ptr<ocs2::MPC_BASE> JoeController::createMpcInterface() {
    TBAI_THROW("Not implemented");
}

std::vector<MotorCommand> JoeController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    mrt_->spinMRT();
    mrt_->updatePolicy();

    currentTime = getCurrentTimeFunction_() - initTime_;

    vector3_t linearVelocityObservation = getLinearVelocityObservation(currentTime, dt);
    vector3_t angularVelocityObservation = getAngularVelocityObservation(currentTime, dt);
    vector3_t projectedGravityObservation = getProjectedGravityObservation(currentTime, dt);
    vector3_t commandObservation = getCommandObservation(currentTime, dt);
    vector_t dofPosObservation = getDofPosObservation(currentTime, dt);
    vector_t dofVelObservation = getDofVelObservation(currentTime, dt);
    vector_t pastActionObservation = getPastActionObservation(currentTime, dt);
    vector_t planarFootholdsObservation = getPlanarFootholdsObservation(currentTime, dt);
    vector_t desiredJointAnglesObservation = getDesiredJointAnglesObservation(currentTime, dt);
    vector_t currentDesiredJointAnglesObservation = getCurrentDesiredJointAnglesObservation(currentTime, dt);
    vector_t desiredContactsObservation = getDesiredContactsObservation(currentTime, dt);
    vector_t timeLeftInPhaseObservation = getTimeLeftInPhaseObservation(currentTime, dt);
    vector_t desiredBasePosObservation = getDesiredBasePosObservation(currentTime, dt);
    vector_t orientationDiffObservation = getOrientationDiffObservation(currentTime, dt);
    vector_t desiredBaseLinVelObservation = getDesiredBaseLinVelObservation(currentTime, dt);
    vector_t desiredBaseAngVelObservation = getDesiredBaseAngVelObservation(currentTime, dt);
    vector_t desiredBaseLinAccObservation = getDesiredBaseLinAccObservation(currentTime, dt);
    vector_t desiredBaseAngAccObservation = getDesiredBaseAngAccObservation(currentTime, dt);
    vector_t cpgObservation = getCpgObservation(currentTime, dt);
    vector_t desiredFootPositionsObservation = getDesiredFootPositionsObservation(currentTime, dt);
    vector_t desiredFootVelocitiesObservation = getDesiredFootVelocitiesObservation(currentTime, dt);
    vector_t heightSamplesObservation = getHeightSamplesObservation(currentTime, dt);

    vector_t eigenObservation = vector_t(MODEL_INPUT_SIZE);

    // clang-format off
    eigenObservation << linearVelocityObservation,
                        angularVelocityObservation,
                        projectedGravityObservation,
                        commandObservation,
                        dofPosObservation,
                        dofVelObservation,
                        pastActionObservation,
                        planarFootholdsObservation,
                        desiredJointAnglesObservation,
                        currentDesiredJointAnglesObservation,
                        desiredContactsObservation,
                        timeLeftInPhaseObservation,
                        desiredBasePosObservation,
                        orientationDiffObservation,
                        desiredBaseLinVelObservation,
                        desiredBaseAngVelObservation,
                        desiredBaseLinAccObservation,
                        desiredBaseAngAccObservation,
                        cpgObservation,
                        desiredFootPositionsObservation,
                        desiredFootVelocitiesObservation,
                        heightSamplesObservation;
    // clang-format on

    torch::Tensor torchObservation = tbai::torch_utils::vector2torch(eigenObservation).view({1, -1});
    torch::Tensor torchAction = joeModel_.forward({torchObservation}).toTensor().view({-1});

    pastAction_ = tbai::torch_utils::torch2vector(torchAction);

    vector_t commandedJointAngles = defaultJointAngles_;
    for (int i = 0; i < 12; ++i) {
        commandedJointAngles[i] += torchAction[i].item<float>() * ACTION_SCALE;
    }

    std::vector<MotorCommand> commands;
    commands.resize(jointNames_.size());
    for (size_t i = 0; i < jointNames_.size(); ++i) {
        auto &command = commands[i];
        command.joint_name = jointNames_[i];
        command.desired_position = commandedJointAngles[i];
        command.desired_velocity = 0.0;
        command.torque_ff = 0.0;
        command.kp = 80;
        command.kd = 2;
    }

    timeSinceLastMpcUpdate_ += dt;
    if (timeSinceLastMpcUpdate_ > 1.0 / mpcRate_) {
        setObservation();
        timeSinceLastMpcUpdate_ = 0.0;
    }

    return commands;
}

contact_flag_t JoeController::getDesiredContactFlags(scalar_t currentTime, scalar_t dt) {
    auto &solution = mrt_->getPolicy();
    auto &modeSchedule = solution.modeSchedule_;
    size_t mode = modeSchedule.modeAtTime(currentTime);
    auto contacts = switched_model::modeNumber2StanceLeg(mode);
    // flip
    std::swap(contacts[1], contacts[2]);
    return contacts;
}

vector_t JoeController::getTimeLeftInPhase(scalar_t currentTime, scalar_t dt) {
    auto &solution = mrt_->getPolicy();
    auto &modeSchedule = solution.modeSchedule_;
    auto &eventTimes = modeSchedule.eventTimes;
    auto it = std::lower_bound(eventTimes.begin(), eventTimes.end(), currentTime);
    if (it == eventTimes.end()) {
        return vector_t::Ones(4) * 0.35;
    }
    return vector_t::Ones(4) * (*it - currentTime);
}

TargetTrajectories JoeController::generateTargetTrajectories(scalar_t currentTime, scalar_t dt,
                                                             const vector3_t &command) {
    if (terrain_ == nullptr) {
        TBAI_THROW("Terrain interface not set - JoeController requires terrain for reference generation");
    }

    return terrain_->generateTargetTrajectories(currentTime, getBaseReferenceHorizon(currentTime),
                                                getBaseReferenceState(currentTime),
                                                getBaseReferenceCommand(currentTime, command), *quadrupedInterface_);
}

BaseReferenceState JoeController::getBaseReferenceState(scalar_t time) {
    auto currentObservation = generateSystemObservation();
    auto observationTime = currentObservation.time;

    Eigen::Vector3d positionInWorld;
    if (lastTargetTrajectories_.get() == nullptr) {
        positionInWorld = currentObservation.state.segment<3>(3);
    } else {
        positionInWorld = lastTargetTrajectories_->getDesiredState(time).segment<3>(3);

        auto currentPos = currentObservation.state.segment<3>(3);
        if ((positionInWorld - currentPos).norm() > 0.1) {
            positionInWorld = currentPos;
        }
    }

    Eigen::Vector3d eulerXyz = currentObservation.state.head<3>(0);
    return {observationTime, positionInWorld, eulerXyz};
}

BaseReferenceCommand JoeController::getBaseReferenceCommand(scalar_t time, const vector_t &command) {
    scalar_t vx = command(0) / LIN_VEL_SCALE;
    scalar_t vy = command(1) / LIN_VEL_SCALE;
    scalar_t wz = command(2) / ANG_VEL_SCALE;
    scalar_t comHeight = 0.53;
    return {vx, vy, wz, comHeight};
}

std::vector<vector3_t> JoeController::getCurrentFeetPositions(scalar_t currentTime, scalar_t dt) {
    SystemObservation sysobs = generateSystemObservation();
    vector_t currentState = sysobs.state;
    auto &kin = *kinematicsModel_;
    auto basePoseOcs2 = switched_model::getBasePose(currentState);
    auto jointAnglesOcs2 = switched_model::getJointPositions(currentState);
    std::vector<vector3_t> feetPositions(4);
    for (int legidx = 0; legidx < 4; ++legidx) {
        auto footPosition = kin.footPositionInOriginFrame(legidx, basePoseOcs2, jointAnglesOcs2);
        feetPositions[legidx] = footPosition;
    }
    return feetPositions;
}

std::vector<vector3_t> JoeController::getCurrentFeetVelocities(scalar_t currentTime, scalar_t dt) {
    SystemObservation sysobs = generateSystemObservation();
    vector_t currentState = sysobs.state;
    vector_t currentInput = sysobs.input;
    auto &kin = *kinematicsModel_;
    auto basePoseOcs2 = switched_model::getBasePose(currentState);
    auto baseTwistOcs2 = switched_model::getBaseLocalVelocities(currentState);
    auto jointAnglesOcs2 = switched_model::getJointPositions(currentState);
    auto jointVelocitiesOcs2 = switched_model::getJointVelocities(currentInput);
    std::vector<vector3_t> feetVelocities(4);
    for (int legidx = 0; legidx < 4; ++legidx) {
        auto footVelocity =
            kin.footVelocityInOriginFrame(legidx, basePoseOcs2, baseTwistOcs2, jointAnglesOcs2, jointVelocitiesOcs2);
        feetVelocities[legidx] = footVelocity;
    }
    return feetVelocities;
}

std::vector<vector3_t> JoeController::getDesiredFeetPositions(scalar_t currentTime, scalar_t dt) {
    auto &solution = mrt_->getPolicy();
    vector_t optimizedState =
        LinearInterpolation::interpolate(currentTime, solution.timeTrajectory_, solution.stateTrajectory_);
    auto &kin = *kinematicsModel_;
    auto basePoseOcs2 = switched_model::getBasePose(optimizedState);
    auto jointAnglesOcs2 = switched_model::getJointPositions(optimizedState);
    std::vector<vector3_t> feetPositions(4);
    for (int legidx = 0; legidx < 4; ++legidx) {
        auto footPosition = kin.footPositionInOriginFrame(legidx, basePoseOcs2, jointAnglesOcs2);
        feetPositions[legidx] = footPosition;
    }
    return feetPositions;
}

std::vector<vector3_t> JoeController::getDesiredFeetVelocities(scalar_t currentTime, scalar_t dt) {
    auto &solution = mrt_->getPolicy();
    vector_t optimizedState =
        LinearInterpolation::interpolate(currentTime, solution.timeTrajectory_, solution.stateTrajectory_);
    vector_t optimizedInput =
        LinearInterpolation::interpolate(currentTime, solution.timeTrajectory_, solution.inputTrajectory_);
    auto &kin = *kinematicsModel_;
    auto basePoseOcs2 = switched_model::getBasePose(optimizedState);
    auto baseTwistOcs2 = switched_model::getBaseLocalVelocities(optimizedState);
    auto jointAnglesOcs2 = switched_model::getJointPositions(optimizedState);
    auto jointVelocitiesOcs2 = switched_model::getJointVelocities(optimizedInput);
    std::vector<vector3_t> feetVelocities(4);
    for (int legidx = 0; legidx < 4; ++legidx) {
        auto footVelocity =
            kin.footVelocityInOriginFrame(legidx, basePoseOcs2, baseTwistOcs2, jointAnglesOcs2, jointVelocitiesOcs2);
        feetVelocities[legidx] = footVelocity;
    }
    return feetVelocities;
}

void JoeController::computeBaseKinematicsAndDynamics(scalar_t currentTime, scalar_t dt, vector3_t &basePos,
                                                     vector_t &baseOrientation, vector3_t &baseLinearVelocity,
                                                     vector3_t &baseAngularVelocity, vector3_t &baseLinearAcceleration,
                                                     vector3_t &baseAngularAcceleration) {
    auto &solution = mrt_->getPolicy();

    // compute desired MPC state and input
    vector_t desiredState = LinearInterpolation::interpolate(currentTime + ISAAC_SIM_DT, solution.timeTrajectory_,
                                                             solution.stateTrajectory_);
    vector_t desiredInput = LinearInterpolation::interpolate(currentTime + ISAAC_SIM_DT, solution.timeTrajectory_,
                                                             solution.inputTrajectory_);

    auto *quadcomPtr = dynamic_cast<anymal::QuadrupedCom *>(comModel_.get());
    auto &quadcom = *quadcomPtr;
    auto &kin = *kinematicsModel_;

    auto basePoseOcs2 = switched_model::getBasePose(desiredState);
    auto jointAnglesOcs2 = switched_model::getJointPositions(desiredState);
    auto baseVelocityOcs2 = switched_model::getBaseLocalVelocities(desiredState);
    auto jointVelocitiesOcs2 = switched_model::getJointVelocities(desiredInput);

    auto qPinocchio = quadcom.getPinnochioConfiguration(basePoseOcs2, jointAnglesOcs2);
    auto vPinocchio = quadcom.getPinnochioVelocity(baseVelocityOcs2, jointVelocitiesOcs2);

    // Desired base orinetation as a quaternion
    quaternion_t desiredBaseOrientationQuat = ocs2rpy2quat(basePoseOcs2.head<3>());  // ocs2 xyz to quaternion
    matrix3_t rotationWorldBase = desiredBaseOrientationQuat.toRotationMatrix();

    const vector_t &basePosett = desiredState.head<6>();
    const vector_t &baseVelocitytt = desiredState.segment<6>(6);
    const vector_t &jointPositionstt = desiredState.tail<switched_model::JOINT_COORDINATE_SIZE>();
    const vector_t &jointVelocitiestt = desiredInput.tail<switched_model::JOINT_COORDINATE_SIZE>();
    const vector_t &jointAccelerationstt = vector_t::Zero(switched_model::JOINT_COORDINATE_SIZE);

    // forcesOnBaseInBaseFrame = [torque (3); force (3)]
    vector_t forcesOnBaseInBaseFrame = vector_t::Zero(6);
    for (size_t i = 0; i < 4; ++i) {
        // force at foot expressed in base frame
        const vector3_t &forceAtFoot = desiredInput.segment<3>(3 * i);

        // base force
        forcesOnBaseInBaseFrame.tail<3>() += forceAtFoot;

        // base torque
        vector3_t footPosition = kin.positionBaseToFootInBaseFrame(i, jointPositionstt);
        forcesOnBaseInBaseFrame.head<3>() += footPosition.cross(forceAtFoot);
    }

    vector_t baseAccelerationLocal = quadcom.calculateBaseLocalAccelerations(
        basePosett, baseVelocitytt, jointPositionstt, jointVelocitiestt, jointAccelerationstt, forcesOnBaseInBaseFrame);

    // Unpack data
    vector3_t desiredBasePosition = basePoseOcs2.tail<3>();
    vector_t desiredBaseOrientationVec = desiredBaseOrientationQuat.coeffs();

    vector3_t desiredBaseLinearVelocity = rotationWorldBase * baseVelocityOcs2.tail<3>();
    vector3_t desiredBaseAngularVelocity = rotationWorldBase * baseVelocityOcs2.head<3>();

    vector3_t desiredBaseLinearAcceleration = rotationWorldBase * (baseAccelerationLocal.tail<3>());
    vector3_t desiredBaseAngularAcceleration = rotationWorldBase * baseAccelerationLocal.head<3>();

    // Update desired base
    basePos = desiredBasePosition;
    baseOrientation = desiredBaseOrientationVec;
    baseLinearVelocity = desiredBaseLinearVelocity;
    baseAngularVelocity = desiredBaseAngularVelocity;
    baseLinearAcceleration = desiredBaseLinearAcceleration;
    baseAngularAcceleration = desiredBaseAngularAcceleration;
}

vector3_t JoeController::getLinearVelocityObservation(scalar_t currentTime, scalar_t dt) const {
    const vector_t &rbdState = state_.x;
    return rbdState.segment<3>(9) * LIN_VEL_SCALE;
}

vector3_t JoeController::getAngularVelocityObservation(scalar_t currentTime, scalar_t dt) const {
    const vector_t &rbdState = state_.x;
    return rbdState.segment<3>(6) * ANG_VEL_SCALE;
}

vector3_t JoeController::getProjectedGravityObservation(scalar_t currentTime, scalar_t dt) const {
    const vector_t &rbdState = state_.x;
    const matrix3_t R_base_world = getRotationMatrixBaseWorld(rbdState);
    return R_base_world * (vector3_t() << 0.0, 0.0, -1.0).finished() * GRAVITY_SCALE;
}

vector3_t JoeController::getCommandObservation(scalar_t currentTime, scalar_t dt) {
    tbai::reference::ReferenceVelocity refvel = refVelGen_->getReferenceVelocity(currentTime, 0.1);
    return vector3_t(refvel.velocity_x * LIN_VEL_SCALE, refvel.velocity_y * LIN_VEL_SCALE,
                     refvel.yaw_rate * ANG_VEL_SCALE * 2);
}

vector_t JoeController::getDofPosObservation(scalar_t currentTime, scalar_t dt) const {
    const vector_t &rbdState = state_.x;
    const vector_t jointAngles = rbdState.segment<12>(12);
    return (jointAngles - defaultJointAngles_) * DOF_POS_SCALE;
}

vector_t JoeController::getDofVelObservation(scalar_t currentTime, scalar_t dt) const {
    const vector_t &rbdState = state_.x;
    const vector_t jointVelocities = rbdState.segment<12>(24);
    return jointVelocities * DOF_VEL_SCALE;
}

vector_t JoeController::getPastActionObservation(scalar_t currentTime, scalar_t dt) const {
    return pastAction_ * PAST_ACTION_SCALE;
}

vector_t JoeController::getPlanarFootholdsObservation(scalar_t currentTime, scalar_t dt) {
    auto &solution = mrt_->getPolicy();
    auto timeLeftInPhase = getTimeLeftInPhase(currentTime, dt);

    std::vector<vector3_t> currentFootPositions = getCurrentFeetPositions(currentTime, dt);
    const auto &rbdState = state_.x;
    vector_t out = vector_t().setZero(8);

    for (int legidx = 0; legidx < 4; ++legidx) {
        scalar_t timeLeft = timeLeftInPhase(legidx);
        scalar_t eventTime = currentTime + timeLeft;
        vector_t optimizedState =
            LinearInterpolation::interpolate(eventTime, solution.timeTrajectory_, solution.stateTrajectory_);
        auto &kin = *kinematicsModel_;
        auto basePoseOcs2 = switched_model::getBasePose(optimizedState);
        auto jointAnglesOcs2 = switched_model::getJointPositions(optimizedState);
        vector3_t futureFootPosition = kin.footPositionInOriginFrame(legidx, basePoseOcs2, jointAnglesOcs2);
        vector3_t currentFootPosition = currentFootPositions[legidx];

        // Update desired footholds
        matrix3_t R_base_world = getRotationMatrixBaseWorld(rbdState);
        vector_t footholdInBase = futureFootPosition - currentFootPosition;
        footholdInBase(2) = 0.0;
        footholdInBase = R_base_world * footholdInBase;

        out.segment<2>(2 * legidx) = footholdInBase.head<2>();
    }

    return out;
}

vector_t JoeController::getDesiredJointAnglesObservation(scalar_t currentTime, scalar_t dt) {
    auto &solution = mrt_->getPolicy();
    auto timeLeftInPhase = getTimeLeftInPhase(currentTime, dt);
    vector_t out = vector_t().setZero(12);

    for (int j = 0; j < 4; ++j) {
        const scalar_t timeLeft = timeLeftInPhase(j);
        auto optimizedState = LinearInterpolation::interpolate(currentTime + timeLeft, solution.timeTrajectory_,
                                                               solution.stateTrajectory_);

        auto *quadcomPtr = dynamic_cast<anymal::QuadrupedCom *>(comModel_.get());
        auto &kin = *kinematicsModel_;

        auto basePoseOcs2 = switched_model::getBasePose(optimizedState);
        auto jointAnglesOcs2 = switched_model::getJointPositions(optimizedState);

        std::swap(jointAnglesOcs2(3 + 0), jointAnglesOcs2(3 + 3));
        std::swap(jointAnglesOcs2(3 + 1), jointAnglesOcs2(3 + 4));
        std::swap(jointAnglesOcs2(3 + 2), jointAnglesOcs2(3 + 5));

        auto optimizedJointAngles = jointAnglesOcs2;

        out.segment<3>(3 * j) = optimizedJointAngles.segment<3>(3 * j);
    }

    // Subtract default joint angles - this is different from env.py
    out -= defaultJointAngles_;

    return out;
}

vector_t JoeController::getCurrentDesiredJointAnglesObservation(scalar_t currentTime, scalar_t dt) {
    auto &solution = mrt_->getPolicy();
    auto optimizedState = LinearInterpolation::interpolate(currentTime + ISAAC_SIM_DT, solution.timeTrajectory_,
                                                           solution.stateTrajectory_);

    auto basePoseOcs2 = switched_model::getBasePose(optimizedState);
    auto jointAnglesOcs2 = switched_model::getJointPositions(optimizedState);

    std::swap(jointAnglesOcs2(3 + 0), jointAnglesOcs2(3 + 3));
    std::swap(jointAnglesOcs2(3 + 1), jointAnglesOcs2(3 + 4));
    std::swap(jointAnglesOcs2(3 + 2), jointAnglesOcs2(3 + 5));

    return jointAnglesOcs2 - defaultJointAngles_;
}

vector_t JoeController::getDesiredContactsObservation(scalar_t currentTime, scalar_t dt) {
    auto desiredContacts = getDesiredContactFlags(currentTime, dt);
    vector_t out = vector_t().setZero(4);
    for (int i = 0; i < 4; ++i) {
        out(i) = static_cast<scalar_t>(desiredContacts[i]);
    }
    return out;
}

vector_t JoeController::getTimeLeftInPhaseObservation(scalar_t currentTime, scalar_t dt) {
    return getTimeLeftInPhase(currentTime, dt);
}

vector_t JoeController::getDesiredBasePosObservation(scalar_t currentTime, scalar_t dt) {
    vector3_t basePos, baseLinearVelocity, baseAngularVelocity, baseLinearAcceleration, baseAngularAcceleration;
    vector_t baseOrientation;
    computeBaseKinematicsAndDynamics(currentTime, dt, basePos, baseOrientation, baseLinearVelocity, baseAngularVelocity,
                                     baseLinearAcceleration, baseAngularAcceleration);
    const auto &rbdState = state_.x;
    vector3_t basePosCurrent = rbdState.segment<3>(3);
    vector3_t basePosDesired = basePos;
    matrix3_t R_base_world = getRotationMatrixBaseWorld(rbdState);
    vector3_t out = R_base_world * (basePosDesired - basePosCurrent);
    return out;
}

vector_t JoeController::getOrientationDiffObservation(scalar_t currentTime, scalar_t dt) {
    vector3_t basePos, baseLinearVelocity, baseAngularVelocity, baseLinearAcceleration, baseAngularAcceleration;
    vector_t baseOrientation;
    computeBaseKinematicsAndDynamics(currentTime, dt, basePos, baseOrientation, baseLinearVelocity, baseAngularVelocity,
                                     baseLinearAcceleration, baseAngularAcceleration);
    const auto &rbdState = state_.x;

    vector3_t eulerAnglesZyxCurrent = getOcs2ZyxEulerAngles(rbdState);
    quaternion_t quatCurrent = this->getQuaternionFromEulerAnglesZyx(eulerAnglesZyxCurrent);

    const scalar_t xdes = baseOrientation(0);
    const scalar_t ydes = baseOrientation(1);
    const scalar_t zdes = baseOrientation(2);
    const scalar_t wdes = baseOrientation(3);
    quaternion_t quatDesired = quaternion_t(wdes, xdes, ydes, zdes);

    // Invert current quaternion
    quaternion_t quatCurrentInverse = quatCurrent.conjugate();

    // Compute the difference
    quaternion_t quatDiff = quatDesired * quatCurrentInverse;

    const scalar_t x = quatDiff.x();
    const scalar_t y = quatDiff.y();
    const scalar_t z = quatDiff.z();
    const scalar_t w = quatDiff.w();

    vector_t orientationDiff = (vector_t(4) << x, y, z, w).finished();

    return orientationDiff;
}

vector_t JoeController::getDesiredBaseLinVelObservation(scalar_t currentTime, scalar_t dt) {
    vector3_t basePos, baseLinearVelocity, baseAngularVelocity, baseLinearAcceleration, baseAngularAcceleration;
    vector_t baseOrientation;
    computeBaseKinematicsAndDynamics(currentTime, dt, basePos, baseOrientation, baseLinearVelocity, baseAngularVelocity,
                                     baseLinearAcceleration, baseAngularAcceleration);
    const auto &rbdState = state_.x;
    matrix3_t R_base_world = getRotationMatrixBaseWorld(rbdState);
    vector3_t baseLinVelDesiredWorld = baseLinearVelocity;
    vector3_t baseLinVelDesiredBase = R_base_world * baseLinVelDesiredWorld;
    return baseLinVelDesiredBase;
}

vector_t JoeController::getDesiredBaseAngVelObservation(scalar_t currentTime, scalar_t dt) {
    vector3_t basePos, baseLinearVelocity, baseAngularVelocity, baseLinearAcceleration, baseAngularAcceleration;
    vector_t baseOrientation;
    computeBaseKinematicsAndDynamics(currentTime, dt, basePos, baseOrientation, baseLinearVelocity, baseAngularVelocity,
                                     baseLinearAcceleration, baseAngularAcceleration);
    const auto &rbdState = state_.x;
    matrix3_t R_base_world = getRotationMatrixBaseWorld(rbdState);
    vector3_t baseAngVelDesiredWorld = baseAngularVelocity;
    vector3_t baseAngVelDesiredBase = R_base_world * baseAngVelDesiredWorld;
    return baseAngVelDesiredBase;
}

vector_t JoeController::getDesiredBaseLinAccObservation(scalar_t currentTime, scalar_t dt) {
    vector3_t basePos, baseLinearVelocity, baseAngularVelocity, baseLinearAcceleration, baseAngularAcceleration;
    vector_t baseOrientation;
    computeBaseKinematicsAndDynamics(currentTime, dt, basePos, baseOrientation, baseLinearVelocity, baseAngularVelocity,
                                     baseLinearAcceleration, baseAngularAcceleration);
    const auto &rbdState = state_.x;
    matrix3_t R_base_world = getRotationMatrixBaseWorld(rbdState);
    vector3_t baseLinAccDesiredWorld = baseLinearAcceleration;
    vector3_t baseLinAccDesiredBase = R_base_world * baseLinAccDesiredWorld;
    return baseLinAccDesiredBase;
}

vector_t JoeController::getDesiredBaseAngAccObservation(scalar_t currentTime, scalar_t dt) {
    vector3_t basePos, baseLinearVelocity, baseAngularVelocity, baseLinearAcceleration, baseAngularAcceleration;
    vector_t baseOrientation;
    computeBaseKinematicsAndDynamics(currentTime, dt, basePos, baseOrientation, baseLinearVelocity, baseAngularVelocity,
                                     baseLinearAcceleration, baseAngularAcceleration);
    const auto &rbdState = state_.x;
    matrix3_t R_base_world = getRotationMatrixBaseWorld(rbdState);
    vector3_t baseAngAccDesiredWorld = baseAngularAcceleration;
    vector3_t baseAngAccDesiredBase = R_base_world * baseAngAccDesiredWorld;
    return baseAngAccDesiredBase;
}

vector_t JoeController::getCpgObservation(scalar_t currentTime, scalar_t dt) {
    auto &solution = mrt_->getPolicy();

    auto desiredContacts = getDesiredContactFlags(currentTime, dt);
    auto timeLeftInPhase = getTimeLeftInPhase(currentTime, dt);

    constexpr scalar_t tp = 0.35;

    vector_t phases = vector_t::Zero(4);

    constexpr scalar_t PI = 3.14159265358979323846;
    for (int j = 0; j < 4; ++j) {
        scalar_t phase = 1.0 - timeLeftInPhase(j) / tp;
        if (desiredContacts[j]) {
            phases(j) = phase * PI;
        } else {
            phases(j) = phase * PI + PI;
        }

        if (phases(j) > 2 * PI) phases(j) -= 2 * PI;
        if (phases(j) < 0) phases(j) += 2 * PI;
    }

    // Perform swap - LF, RF, LH, RH -> LF, LH, RF, RH
    std::swap(phases(1), phases(2));

    // Compute cpg obs
    vector_t out = vector_t::Zero(8);
    for (int j = 0; j < 4; ++j) {
        const scalar_t phase = phases(j);
        const scalar_t c = std::cos(phase);
        const scalar_t s = std::sin(phase);
        out(j) = c;
        out(j + 4) = s;
    }

    return out;
}

vector_t JoeController::getDesiredFootPositionsObservation(scalar_t currentTime, scalar_t dt) {
    auto desiredFootPositions = getDesiredFeetPositions(currentTime, dt);
    auto currentFootPositions = getCurrentFeetPositions(currentTime, dt);

    matrix3_t R_base_world = getRotationMatrixBaseWorld(state_.x);

    vector_t lf_pos = -R_base_world * (desiredFootPositions[0] - currentFootPositions[0]);
    vector_t rf_pos = -R_base_world * (desiredFootPositions[1] - currentFootPositions[1]);
    vector_t lh_pos = -R_base_world * (desiredFootPositions[2] - currentFootPositions[2]);
    vector_t rh_pos = -R_base_world * (desiredFootPositions[3] - currentFootPositions[3]);

    vector_t out(4 * 3);
    out << lf_pos, lh_pos, rf_pos, rh_pos;
    return out;
}

vector_t JoeController::getDesiredFootVelocitiesObservation(scalar_t currentTime, scalar_t dt) {
    auto desiredFootVelocities = getDesiredFeetVelocities(currentTime, dt);
    auto currentFootVelocities = getCurrentFeetVelocities(currentTime, dt);

    matrix3_t R_base_world = getRotationMatrixBaseWorld(state_.x);

    vector_t lf_vel = -R_base_world * (desiredFootVelocities[0] - currentFootVelocities[0]);
    vector_t rf_vel = -R_base_world * (desiredFootVelocities[1] - currentFootVelocities[1]);
    vector_t lh_vel = -R_base_world * (desiredFootVelocities[2] - currentFootVelocities[2]);
    vector_t rh_vel = -R_base_world * (desiredFootVelocities[3] - currentFootVelocities[3]);

    vector_t out(4 * 3);
    out << lf_vel, lh_vel, rf_vel, rh_vel;
    return out;
}

vector_t JoeController::getHeightSamplesObservation(scalar_t currentTime, scalar_t dt) {
    auto &solution = mrt_->getPolicy();
    auto currentFootPositions = getCurrentFeetPositions(currentTime, dt);
    auto timeLeftInPhase = getTimeLeftInPhase(currentTime, dt);
    vector_t out(4 * 10);
    for (int legidx = 0; legidx < 4; ++legidx) {
        scalar_t timeLeft = timeLeftInPhase(legidx);
        scalar_t eventTime = currentTime + timeLeft;
        vector_t optimizedState =
            LinearInterpolation::interpolate(eventTime, solution.timeTrajectory_, solution.stateTrajectory_);
        auto &kin = *kinematicsModel_;
        auto basePoseOcs2 = switched_model::getBasePose(optimizedState);
        auto jointAnglesOcs2 = switched_model::getJointPositions(optimizedState);
        vector3_t futureFootPosition = kin.footPositionInOriginFrame(legidx, basePoseOcs2, jointAnglesOcs2);
        vector3_t currentFootPosition = currentFootPositions[legidx];

        vector3_t diff = futureFootPosition - currentFootPosition;
        scalar_t dalpha = 1.0 / (10 - 1);
        for (int i = 0; i < 10; ++i) {
            scalar_t alpha = dalpha * i;
            vector3_t pos = currentFootPosition + diff * alpha;
            scalar_t x = pos(0);
            scalar_t y = pos(1);
            scalar_t height_diff;
            if (terrain_ != nullptr) {
                scalar_t height = terrain_->atPosition(x, y);
                height_diff = height - currentFootPosition(2);
            } else {
                height_diff = 0.0;  // Blind mode
            }
            out(legidx * 10 + i) = std::max(-0.5, std::min(0.5, height_diff));
        }
    }
    return out;
}

void JoeController::postStep(scalar_t currentTime, scalar_t dt) {}

void JoeController::changeController(const std::string &controllerType, scalar_t currentTime) {
    preStep(currentTime, 0.0);
    TBAI_LOG_INFO(logger_, "Changing controller");
    resetMpc();
    TBAI_LOG_INFO(logger_, "Controller changed");
    if (terrain_ != nullptr) {
        terrain_->waitTillInitialized();
    }
    pastAction_ = vector_t::Zero(12);
    lastTargetTrajectories_.reset();

    // Start reference thread
    startReferenceThread();
}

void JoeController::startReferenceThread() {
    stopReferenceThread();

    TBAI_LOG_INFO(logger_, "Starting reference thread");
    stopReferenceThread_ = false;
    referenceThread_ = std::thread(&JoeController::referenceThreadLoop, this);
    TBAI_LOG_INFO(logger_, "Reference thread started");
}

void JoeController::stopReferenceThread() {
    stopReferenceThread_ = true;
    if (referenceThread_.joinable()) {
        referenceThread_.join();
    }
}

void JoeController::referenceThreadLoop() {
    TBAI_LOG_INFO(logger_, "Reference thread loop started");

    auto sleepDuration = std::chrono::milliseconds(static_cast<int>(1000.0 / referenceThreadRate_));
    while (!stopReferenceThread_) {
        scalar_t currentTime = getCurrentTimeFunction_() - initTime_;

        // Get command observation
        tbai::reference::ReferenceVelocity refvel = refVelGen_->getReferenceVelocity(currentTime, 0.1);
        vector3_t commandObservation(refvel.velocity_x * LIN_VEL_SCALE, refvel.velocity_y * LIN_VEL_SCALE,
                                     refvel.yaw_rate * ANG_VEL_SCALE * 2);

        // Generate and publish reference trajectory
        auto reference = generateTargetTrajectories(currentTime, 0.0, commandObservation);
        mrt_->setTargetTrajectories(reference);
        lastTargetTrajectories_ = std::make_unique<TargetTrajectories>(reference);
        publishReference(reference);

        std::this_thread::sleep_for(sleepDuration);
    }

    TBAI_LOG_INFO(logger_, "Reference thread loop stopped");
}

bool JoeController::checkStability() const {
    const auto &state = state_.x;
    scalar_t roll = state[0];
    if (roll >= 1.57 || roll <= -1.57) {
        return false;
    }
    scalar_t pitch = state[1];
    if (pitch >= 1.57 || pitch <= -1.57) {
        return false;
    }
    return true;
}

bool JoeController::isSupported(const std::string &controllerType) {
    return controllerType == "JOE";
}

ocs2::SystemObservation JoeController::generateSystemObservation() {
    State state = stateSubscriberPtr_->getLatestState();
    const tbai::vector_t &rbdState = state.x;

    // Set observation time
    ocs2::SystemObservation observation;
    observation.time = state.timestamp - initTime_;

    // Set mode
    auto contactFlags = state.contactFlags;
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

void JoeController::resetMpc() {
    TBAI_LOG_INFO(logger_, "Waiting for state subscriber to initialize...");

    // Wait to receive observation
    stateSubscriberPtr_->waitTillInitialized();

    TBAI_LOG_INFO(logger_, "State subscriber initialized");

    // Prepare initial observation for MPC
    ocs2::SystemObservation mpcObservation = generateSystemObservation();

    TBAI_LOG_INFO(logger_, "Initial observation generated");

    // Prepare target trajectory
    ocs2::TargetTrajectories initTargetTrajectories({0.0}, {mpcObservation.state}, {mpcObservation.input});

    TBAI_LOG_INFO(logger_, "Resetting MPC...");
    mrt_->resetMpcNode(initTargetTrajectories);

    // Wait for initial policy
    while (!mrt_->initialPolicyReceived()) {
        TBAI_LOG_INFO(logger_, "Waiting for initial policy...");
        mrt_->setCurrentObservation(generateSystemObservation());
        mrt_->spinMRT();
        mrt_->updatePolicy();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    TBAI_LOG_INFO(logger_, "Initial policy received");
}

void JoeController::setObservation() {
    mrt_->setCurrentObservation(generateSystemObservation());
}

}  // namespace joe
}  // namespace tbai
