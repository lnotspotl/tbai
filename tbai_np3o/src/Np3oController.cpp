// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_np3o/EigenTorch.hpp>
#include <tbai_np3o/Np3oController.hpp>

namespace tbai {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
static inline int mod(int a, int b) {
    return (a % b + b) % b;
}

static inline scalar_t clip(scalar_t x, scalar_t min, scalar_t max) {
    return std::max(min, std::min(x, max));
}

Np3oController::Np3oController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                               const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen)
    : Np3oController::Np3oController(tbai::getEnvAs<std::string>("TBAI_ROBOT_DESCRIPTION_PATH"), stateSubscriberPtr,
                                     refVelGen) {}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
Np3oController::Np3oController(const std::string &urdfPathOrString,
                               const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                               const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen)
    : stateSubscriberPtr_(stateSubscriberPtr), refVelGen_(refVelGen), historyBuffer_(45, 10) {
    logger_ = tbai::getLogger("np3o");

    gaitIndex_ = 0.0;

    // Load parameters
    kp_ = tbai::fromGlobalConfig<scalar_t>("np3o/kp");
    kd_ = tbai::fromGlobalConfig<scalar_t>("np3o/kd");
    useActionFilter_ = tbai::fromGlobalConfig<bool>("np3o/use_action_filter");

    TBAI_LOG_INFO(logger_, "Use action filter: {}", useActionFilter_);
    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Load URDF string from file
    bool isFile;
    try {
        isFile = std::filesystem::exists(urdfPathOrString);
    } catch (const std::exception &e) {
        isFile = false;
    }

    std::string urdfString = urdfPathOrString;
    if (isFile) {
        TBAI_LOG_INFO(logger_, "Loading URDF from file: {}", urdfPathOrString);
        std::ifstream urdfFile(urdfPathOrString);
        std::stringstream buffer;
        buffer << urdfFile.rdbuf();
        urdfString = buffer.str();
    } else {
        TBAI_LOG_INFO(logger_, "Loading URDF from string");
    }
    setupPinocchioModel(urdfString);

    // auto hfRepo = tbai::fromGlobalConfig<std::string>("wtw_controller/hf_repo");
    // auto hfModel = tbai::fromGlobalConfig<std::string>("wtw_controller/hf_model");

    // TBAI_LOG_INFO(logger_, "Loading HF model: {}/{}", hfRepo, hfModel);
    // auto modelPath = tbai::downloadFromHuggingFace(hfRepo, hfModel);
    // TBAI_LOG_INFO(logger_, "Model downloaded to: {}", modelPath);

    const std::string modelPath = "/home/user/model.pt";
    try {
        model_ = torch::jit::load(modelPath, torch::kCPU);
    } catch (const c10::Error &e) {
        TBAI_THROW("Could not load model from: {}\nError: {}", modelPath, e.what());
    }

    TBAI_LOG_INFO(logger_, "Model loaded");

    lastAction_ = tbai::vector_t::Zero(12);
    filterLastAction_ = tbai::vector_t::Zero(12);

    defaultJointAngles_ = tbai::vector_t::Zero(12);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::setupPinocchioModel(const std::string &urdfString) {
    pinocchio::urdf::buildModelFromXML(urdfString, pinocchio::JointModelFreeFlyer(), pinocchioModel_);
    pinocchioData_ = pinocchio::Data(pinocchioModel_);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/**********************************************************************************************
 * *************************/
bool Np3oController::isSupported(const std::string &controllerType) {
    if (controllerType == "NP3O") {
        return true;
    }
    return false;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool Np3oController::checkStability() const {
    return true;
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

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
tbai::vector_t Np3oController::getObsProprioceptive(const np3o::State &state, scalar_t currentTime, scalar_t dt) {
    vector_t input(45);
    fillBaseAngularVelocity(input, state);
    fillProjectedGravity(input, state);
    fillCommand(input, state, currentTime, dt);
    fillJointResiduals(input, state);
    fillJointVelocities(input, state);
    fillLastAction(input, state);

    // Clip inputs to be between -100 and 100
    for (int i = 0; i < 45; ++i) {
        input[i] = clip(input[i], -100.0, 100.0);
    }

    return input;
}

tbai::vector_t Np3oController::getObsHistory() {
    return historyBuffer_.getFinalObservation();
}

void Np3oController::updateObsHistory(const tbai::vector_t &observation) {
    historyBuffer_.addObservation(observation);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> Np3oController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    auto state = getNp3oState();

    // Do not keep track of gradients
    torch::NoGradGuard no_grad;

    auto ts1 = std::chrono::high_resolution_clock::now();
    tbai::vector_t obsProprioceptiveEigen = getObsProprioceptive(state, currentTime, dt);
    tbai::vector_t obsHistoryEigen = getObsHistory();

    at::Tensor obsProprioceptive = tbai::vector2torch(obsProprioceptiveEigen);
    at::Tensor obsHistory = tbai::vector2torch(obsHistoryEigen);

    auto t2 = std::chrono::high_resolution_clock::now();

    // perform forward pass
    auto ts3 = std::chrono::high_resolution_clock::now();
    at::Tensor out = forward(obsProprioceptive, obsHistory).squeeze();
    auto t4 = std::chrono::high_resolution_clock::now();

    tbai::vector_t currentAction =
        tbai::torch2vector(out.reshape({12}) *
                           torch::tensor({0.125, 0.25, 0.25, 0.125, 0.25, 0.25, 0.125, 0.25, 0.25, 0.125, 0.25, 0.25}));
    tbai::vector_t filteredAction;

    if (useActionFilter_) {
        filteredAction = 0.2 * filterLastAction_ + 0.8 * currentAction;
        filterLastAction_ = currentAction;
    } else {
        filteredAction = currentAction;
    }
    auto ret = getMotorCommands(filteredAction + defaultJointAngles_);

    lastAction_ = tbai::torch2vector(out.reshape({12}));
    TBAI_LOG_DEBUG_THROTTLE(logger_, 0.5, "Current action: {} Last action: {} Joint angles: {}",
                            (std::stringstream() << currentAction.transpose()).str(),
                            (std::stringstream() << lastAction_.transpose()).str(),
                            (std::stringstream() << (filteredAction + defaultJointAngles_).transpose()).str());

    updateObsHistory(obsProprioceptiveEigen);

    auto t5 = std::chrono::high_resolution_clock::now();
    TBAI_LOG_INFO_THROTTLE(
        logger_, 10.0,
        "NN input preparations took {} ms, NN forward pass took: {} ms. Total controller step took: {} ms",
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - ts1).count() / 1000.0,
        std::chrono::duration_cast<std::chrono::microseconds>(t4 - ts3).count() / 1000.0,
        std::chrono::duration_cast<std::chrono::microseconds>(t5 - ts1).count() / 1000.0);

    return ret;
}

void Np3oController::fillBaseAngularVelocity(vector_t &input, const np3o::State &state) {
    constexpr scalar_t BASE_ANGULAR_VELOCITY_SCALE = 0.25;
    input[BASE_ANGULAR_VELOCITY_START_INDEX + 0] = state.baseAngularVelocityBase[0] * BASE_ANGULAR_VELOCITY_SCALE;
    input[BASE_ANGULAR_VELOCITY_START_INDEX + 1] = state.baseAngularVelocityBase[1] * BASE_ANGULAR_VELOCITY_SCALE;
    input[BASE_ANGULAR_VELOCITY_START_INDEX + 2] = state.baseAngularVelocityBase[2] * BASE_ANGULAR_VELOCITY_SCALE;
}

void Np3oController::fillProjectedGravity(vector_t &input, const np3o::State &state) {
    constexpr scalar_t PROJECTED_GRAVITY_SCALE = 1.0;
    input[PROJECTED_GRAVITY_START_INDEX + 0] = state.normalizedGravityBase[0] * PROJECTED_GRAVITY_SCALE;
    input[PROJECTED_GRAVITY_START_INDEX + 1] = state.normalizedGravityBase[1] * PROJECTED_GRAVITY_SCALE;
    input[PROJECTED_GRAVITY_START_INDEX + 2] = state.normalizedGravityBase[2] * PROJECTED_GRAVITY_SCALE;
}

void Np3oController::fillCommand(vector_t &input, const np3o::State &state, scalar_t currentTime, scalar_t dt) {
    constexpr scalar_t LIN_VEL_SCALE = 2.0;
    constexpr scalar_t ANG_VEL_SCALE = 0.25 * 5;  // 5 because it was too slow

    auto command = refVelGen_->getReferenceVelocity(currentTime, dt);
    const scalar_t velocity_x = command.velocity_x;
    const scalar_t velocity_y = command.velocity_y;
    const scalar_t yaw_rate = command.yaw_rate;

    input[COMMAND_START_INDEX + 0] = velocity_x * LIN_VEL_SCALE;
    input[COMMAND_START_INDEX + 1] = velocity_y * LIN_VEL_SCALE;
    input[COMMAND_START_INDEX + 2] = yaw_rate * ANG_VEL_SCALE;
}

void Np3oController::fillJointResiduals(vector_t &input, const np3o::State &state) {
    constexpr scalar_t JOINT_RESIDUAL_SCALE = 1.00;
    std::map<std::string, scalar_t> defaultJointAngles;
    defaultJointAngles["FL_hip_joint"] = 0.1;
    defaultJointAngles["RL_hip_joint"] = 0.1;
    defaultJointAngles["FR_hip_joint"] = -0.1;
    defaultJointAngles["RR_hip_joint"] = -0.1;

    defaultJointAngles["FL_thigh_joint"] = 0.8;
    defaultJointAngles["RL_thigh_joint"] = 1.0;
    defaultJointAngles["FR_thigh_joint"] = 0.8;
    defaultJointAngles["RR_thigh_joint"] = 1.0;

    defaultJointAngles["FL_calf_joint"] = -1.5;
    defaultJointAngles["RL_calf_joint"] = -1.5;
    defaultJointAngles["FR_calf_joint"] = -1.5;
    defaultJointAngles["RR_calf_joint"] = -1.5;

    scalar_t FL_hip_joint_position = state.jointPositions[0];
    scalar_t FL_thigh_joint_position = state.jointPositions[1];
    scalar_t FL_calf_joint_position = state.jointPositions[2];

    scalar_t RL_hip_joint_position = state.jointPositions[3];
    scalar_t RL_thigh_joint_position = state.jointPositions[4];
    scalar_t RL_calf_joint_position = state.jointPositions[5];

    scalar_t FR_hip_joint_position = state.jointPositions[6];
    scalar_t FR_thigh_joint_position = state.jointPositions[7];
    scalar_t FR_calf_joint_position = state.jointPositions[8];

    scalar_t RR_hip_joint_position = state.jointPositions[9];
    scalar_t RR_thigh_joint_position = state.jointPositions[10];
    scalar_t RR_calf_joint_position = state.jointPositions[11];

    scalar_t FL_hip_joint_residual = FL_hip_joint_position - defaultJointAngles["FL_hip_joint"];
    scalar_t RL_hip_joint_residual = RL_hip_joint_position - defaultJointAngles["RL_hip_joint"];
    scalar_t FR_hip_joint_residual = FR_hip_joint_position - defaultJointAngles["FR_hip_joint"];
    scalar_t RR_hip_joint_residual = RR_hip_joint_position - defaultJointAngles["RR_hip_joint"];

    scalar_t FL_thigh_joint_residual = FL_thigh_joint_position - defaultJointAngles["FL_thigh_joint"];
    scalar_t RL_thigh_joint_residual = RL_thigh_joint_position - defaultJointAngles["RL_thigh_joint"];
    scalar_t FR_thigh_joint_residual = FR_thigh_joint_position - defaultJointAngles["FR_thigh_joint"];
    scalar_t RR_thigh_joint_residual = RR_thigh_joint_position - defaultJointAngles["RR_thigh_joint"];

    scalar_t FL_calf_joint_residual = FL_calf_joint_position - defaultJointAngles["FL_calf_joint"];
    scalar_t RL_calf_joint_residual = RL_calf_joint_position - defaultJointAngles["RL_calf_joint"];
    scalar_t FR_calf_joint_residual = FR_calf_joint_position - defaultJointAngles["FR_calf_joint"];
    scalar_t RR_calf_joint_residual = RR_calf_joint_position - defaultJointAngles["RR_calf_joint"];

    // Unitree ordering: FR, FL, RR, RL
    input[DOF_RESIDUALS_START_INDEX + 0] = FR_hip_joint_residual * JOINT_RESIDUAL_SCALE;
    input[DOF_RESIDUALS_START_INDEX + 1] = FR_thigh_joint_residual * JOINT_RESIDUAL_SCALE;
    input[DOF_RESIDUALS_START_INDEX + 2] = FR_calf_joint_residual * JOINT_RESIDUAL_SCALE;

    input[DOF_RESIDUALS_START_INDEX + 3] = FL_hip_joint_residual * JOINT_RESIDUAL_SCALE;
    input[DOF_RESIDUALS_START_INDEX + 4] = FL_thigh_joint_residual * JOINT_RESIDUAL_SCALE;
    input[DOF_RESIDUALS_START_INDEX + 5] = FL_calf_joint_residual * JOINT_RESIDUAL_SCALE;

    input[DOF_RESIDUALS_START_INDEX + 6] = RR_hip_joint_residual * JOINT_RESIDUAL_SCALE;
    input[DOF_RESIDUALS_START_INDEX + 7] = RR_thigh_joint_residual * JOINT_RESIDUAL_SCALE;
    input[DOF_RESIDUALS_START_INDEX + 8] = RR_calf_joint_residual * JOINT_RESIDUAL_SCALE;

    input[DOF_RESIDUALS_START_INDEX + 9] = RL_hip_joint_residual * JOINT_RESIDUAL_SCALE;
    input[DOF_RESIDUALS_START_INDEX + 10] = RL_thigh_joint_residual * JOINT_RESIDUAL_SCALE;
    input[DOF_RESIDUALS_START_INDEX + 11] = RL_calf_joint_residual * JOINT_RESIDUAL_SCALE;

    defaultJointAngles_[0] = defaultJointAngles["FR_hip_joint"];
    defaultJointAngles_[1] = defaultJointAngles["FR_thigh_joint"];
    defaultJointAngles_[2] = defaultJointAngles["FR_calf_joint"];
    defaultJointAngles_[3] = defaultJointAngles["FL_hip_joint"];
    defaultJointAngles_[4] = defaultJointAngles["FL_thigh_joint"];
    defaultJointAngles_[5] = defaultJointAngles["FL_calf_joint"];
    defaultJointAngles_[6] = defaultJointAngles["RR_hip_joint"];
    defaultJointAngles_[7] = defaultJointAngles["RR_thigh_joint"];
    defaultJointAngles_[8] = defaultJointAngles["RR_calf_joint"];
    defaultJointAngles_[9] = defaultJointAngles["RL_hip_joint"];
    defaultJointAngles_[10] = defaultJointAngles["RL_thigh_joint"];
    defaultJointAngles_[11] = defaultJointAngles["RL_calf_joint"];
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::fillJointVelocities(vector_t &input, const np3o::State &state) {
    constexpr scalar_t JOINT_VELOCITY_SCALE = 0.05;

    scalar_t FL_hip_joint_velocity = state.jointVelocities[0];
    scalar_t FL_thigh_joint_velocity = state.jointVelocities[1];
    scalar_t FL_calf_joint_velocity = state.jointVelocities[2];

    scalar_t RL_hip_joint_velocity = state.jointVelocities[3];
    scalar_t RL_thigh_joint_velocity = state.jointVelocities[4];
    scalar_t RL_calf_joint_velocity = state.jointVelocities[5];

    scalar_t FR_hip_joint_velocity = state.jointVelocities[6];
    scalar_t FR_thigh_joint_velocity = state.jointVelocities[7];
    scalar_t FR_calf_joint_velocity = state.jointVelocities[8];

    scalar_t RR_hip_joint_velocity = state.jointVelocities[9];
    scalar_t RR_thigh_joint_velocity = state.jointVelocities[10];
    scalar_t RR_calf_joint_velocity = state.jointVelocities[11];

    // Manual loop unrolling right here :)
    // Unitree ordering: FR, FL, RR, RL
    input[DOF_VELOCITIES_START_INDEX + 0] = FR_hip_joint_velocity * JOINT_VELOCITY_SCALE;
    input[DOF_VELOCITIES_START_INDEX + 1] = FR_thigh_joint_velocity * JOINT_VELOCITY_SCALE;
    input[DOF_VELOCITIES_START_INDEX + 2] = FR_calf_joint_velocity * JOINT_VELOCITY_SCALE;

    input[DOF_VELOCITIES_START_INDEX + 3] = FL_hip_joint_velocity * JOINT_VELOCITY_SCALE;
    input[DOF_VELOCITIES_START_INDEX + 4] = FL_thigh_joint_velocity * JOINT_VELOCITY_SCALE;
    input[DOF_VELOCITIES_START_INDEX + 5] = FL_calf_joint_velocity * JOINT_VELOCITY_SCALE;

    input[DOF_VELOCITIES_START_INDEX + 6] = RR_hip_joint_velocity * JOINT_VELOCITY_SCALE;
    input[DOF_VELOCITIES_START_INDEX + 7] = RR_thigh_joint_velocity * JOINT_VELOCITY_SCALE;
    input[DOF_VELOCITIES_START_INDEX + 8] = RR_calf_joint_velocity * JOINT_VELOCITY_SCALE;

    input[DOF_VELOCITIES_START_INDEX + 9] = RL_hip_joint_velocity * JOINT_VELOCITY_SCALE;
    input[DOF_VELOCITIES_START_INDEX + 10] = RL_thigh_joint_velocity * JOINT_VELOCITY_SCALE;
    input[DOF_VELOCITIES_START_INDEX + 11] = RL_calf_joint_velocity * JOINT_VELOCITY_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::fillLastAction(vector_t &input, const np3o::State &state) {
    for (int i = 0; i < 12; ++i) {
        input[LAST_ACTION_START_INDEX + i] = clip(lastAction_[i], -100.0, 100.0);
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> Np3oController::getMotorCommands(const vector_t &jointAngles) {
    // TODO: Remove this, this one should be loaded from the config
    // Unitree ordering: FR, FL, RR, RL
    std::vector<std::string> jointNames = {"RF_HAA", "RF_HFE", "RF_KFE", "LF_HAA", "LF_HFE", "LF_KFE",
                                           "RH_HAA", "RH_HFE", "RH_KFE", "LH_HAA", "LH_HFE", "LH_KFE"};

    std::vector<tbai::MotorCommand> motorCommands;
    motorCommands.resize(jointAngles.size());
    for (size_t i = 0; i < jointAngles.size(); ++i) {
        tbai::MotorCommand &command = motorCommands[i];
        command.joint_name = jointNames[i];
        command.desired_position = jointAngles[i];
        command.desired_velocity = 0.0;
        command.kp = kp_;
        command.kd = kd_;
        command.torque_ff = 0.0;
    }
    return motorCommands;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
np3o::State Np3oController::getNp3oState() {
    const vector_t &stateSubscriberState = state_.x;
    np3o::State ret;

    // Base position
    ret.basePositionWorld = stateSubscriberState.segment<3>(3);

    // Base orientation
    // Pinocchio and ocs2 both use a different euler angle logic
    // https://github.com/stack-of-tasks/pinocchio/blob/ac0b1aa6b18931bed60d0657de3c7680a4037dd3/include/pinocchio/math/rpy.hxx#L51
    // https://github.com/leggedrobotics/ocs2/blob/164c26b46bed5d24cd03d90588db8980d03a4951/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_commands/src/TerrainAdaptation.cpp#L20
    vector3_t ocs2rpy = stateSubscriberState.segment<3>(0);
    tbai::quaternion_t q = tbai::ocs2rpy2quat(ocs2rpy);
    auto Rwb = q.toRotationMatrix();
    ret.baseOrientationWorld = (np3o::State::Vector4() << q.x(), q.y(), q.z(), q.w()).finished();

    // Base angular velocity
    ret.baseAngularVelocityBase = stateSubscriberState.segment<3>(6);

    // Base linear velocity
    ret.baseLinearVelocityBase = stateSubscriberState.segment<3>(9);

    // Normalized gravity vector
    ret.normalizedGravityBase = Rwb.transpose() * (vector3_t() << 0, 0, -1).finished();

    // Joint positions
    ret.jointPositions = stateSubscriberState.segment<12>(12);

    // Joint velocities
    ret.jointVelocities = stateSubscriberState.segment<12>(12 + 12);

    // pinocchio state vector

    vector_t pinocchioStateVector(19);
    pinocchioStateVector.segment<3>(0) = ret.basePositionWorld;
    pinocchioStateVector.segment<4>(3) = ret.baseOrientationWorld;
    pinocchioStateVector.segment<12>(7) = ret.jointPositions;

    // Update kinematics
    pinocchio::forwardKinematics(pinocchioModel_, pinocchioData_, pinocchioStateVector);
    pinocchio::updateFramePlacements(pinocchioModel_, pinocchioData_);

    ret.lfFootPositionWorld = pinocchioData_.oMf[pinocchioModel_.getBodyId("LF_FOOT")].translation();
    ret.lhFootPositionWorld = pinocchioData_.oMf[pinocchioModel_.getBodyId("LH_FOOT")].translation();
    ret.rfFootPositionWorld = pinocchioData_.oMf[pinocchioModel_.getBodyId("RF_FOOT")].translation();
    ret.rhFootPositionWorld = pinocchioData_.oMf[pinocchioModel_.getBodyId("RH_FOOT")].translation();

    return ret;
}

}  // namespace tbai