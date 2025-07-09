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
    : stateSubscriberPtr_(stateSubscriberPtr), refVelGen_(refVelGen), historyBuffer_(70, 30) {
    logger_ = tbai::getLogger("wtw_controller");

    gaitIndex_ = 0.0;

    // Load parameters
    kp_ = tbai::fromGlobalConfig<scalar_t>("wtw_controller/kp");
    kd_ = tbai::fromGlobalConfig<scalar_t>("wtw_controller/kd");
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

    const std::string adaptationModulePath =
        "/home/kuba/Documents/walk-these-ways-go2/runs/gait-conditioned-agility/pretrain-go2/train/142238.667503/"
        "checkpoints/adaptation_module_latest.jit";
    try {
        adaptationModule_ = torch::jit::load(adaptationModulePath);
    } catch (const c10::Error &e) {
        TBAI_THROW("Could not load model from: {}\nError: {}", adaptationModulePath, e.what());
    }

    const std::string bodyModulePath =
        "/home/kuba/Documents/walk-these-ways-go2/runs/gait-conditioned-agility/pretrain-go2/train/142238.667503/"
        "checkpoints/body_latest.jit";
    try {
        bodyModule_ = torch::jit::load(bodyModulePath);
    } catch (const c10::Error &e) {
        TBAI_THROW("Could not load model from: {}\nError: {}", bodyModulePath, e.what());
    }
    TBAI_LOG_INFO(logger_, "Model loaded");

    lastAction_ = tbai::vector_t::Zero(12);
    lastLastAction_ = tbai::vector_t::Zero(12);

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
    if (controllerType == "WTW") {
        return true;
    }
    return false;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool Np3oController::checkStability() const {
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
at::Tensor Np3oController::getNNInput(const wtw::State &state, scalar_t currentTime, scalar_t dt) {
    vector_t input(70);
    fillGravity(input, state);
    fillCommand(input, state, currentTime, dt);
    fillJointResiduals(input, state);
    fillJointVelocities(input, state);
    fillLastAction(input, state);
    fillLastLastAction(input, state);
    fillClockInputs(input, currentTime, dt);

    historyBuffer_.addObservation(input);
    return vector2torch(historyBuffer_.getFinalObservation());
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> Np3oController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    auto state = getWtwState();

    // Do not keep track of gradients
    torch::NoGradGuard no_grad;

    auto ts1 = std::chrono::high_resolution_clock::now();
    at::Tensor nnInput = getNNInput(state, currentTime, dt);
    auto t2 = std::chrono::high_resolution_clock::now();

    // perform forward pass
    auto ts3 = std::chrono::high_resolution_clock::now();
    at::Tensor out = forward(nnInput).squeeze();
    auto t4 = std::chrono::high_resolution_clock::now();

    // Send command
    auto ret =
        getMotorCommands(tbai::torch2vector(out.reshape({12}) * torch::tensor({0.125, 0.25, 0.25, 0.125, 0.25, 0.25,
                                                                               0.125, 0.25, 0.25, 0.125, 0.25, 0.25})) +
                         defaultJointAngles_);

    lastLastAction_ = lastAction_;
    lastAction_ = tbai::torch2vector(out.reshape({12}));

    auto t5 = std::chrono::high_resolution_clock::now();
    TBAI_LOG_INFO_THROTTLE(
        logger_, 10.0,
        "NN input preparations took {} ms, NN forward pass took: {} ms. Total controller step took: {} ms",
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - ts1).count() / 1000.0,
        std::chrono::duration_cast<std::chrono::microseconds>(t4 - ts3).count() / 1000.0,
        std::chrono::duration_cast<std::chrono::microseconds>(t5 - ts1).count() / 1000.0);

    return ret;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::fillGravity(vector_t &input, const wtw::State &state) {
    input[GRAVITY_START_INDEX + 0] = state.normalizedGravityBase[0];
    input[GRAVITY_START_INDEX + 1] = state.normalizedGravityBase[1];
    input[GRAVITY_START_INDEX + 2] = state.normalizedGravityBase[2];
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::fillCommand(vector_t &input, const wtw::State &state, scalar_t currentTime, scalar_t dt) {
    constexpr scalar_t LIN_VEL_SCALE = 2.0;
    constexpr scalar_t ANG_VEL_SCALE = 0.25;
    constexpr scalar_t BODY_HEIGHT_SCALE = 2.0;
    constexpr scalar_t SWING_HEIGHT_SCALE = 0.15;
    constexpr scalar_t BODY_PITCH_SCALE = 0.3;
    constexpr scalar_t BODY_ROLL_SCALE = 0.3;
    constexpr scalar_t STANCE_WIDTH_SCALE = 1.0;
    constexpr scalar_t STANCE_LENGTH_SCALE = 1.0;
    constexpr scalar_t AUX_REWARD_SCALE = 1.0;

    auto command = refVelGen_->getReferenceVelocity(currentTime, dt);
    const scalar_t velocity_x = command.velocity_x;
    const scalar_t velocity_y = command.velocity_y;
    const scalar_t yaw_rate = command.yaw_rate;

    input[COMMAND_START_INDEX + 0] = velocity_x * LIN_VEL_SCALE;
    input[COMMAND_START_INDEX + 1] = velocity_y * LIN_VEL_SCALE;
    input[COMMAND_START_INDEX + 2] = yaw_rate;

    // 0.0 is default height
    input[COMMAND_START_INDEX + 3] = 0.0 * BODY_HEIGHT_SCALE;

    // TODO: Fill these
    input[COMMAND_START_INDEX + 4] = 2.0;  // step frequency
    input[COMMAND_START_INDEX + 5] = 0.5;  // gait 1
    input[COMMAND_START_INDEX + 6] = 0.0;  // gait 2 phase
    input[COMMAND_START_INDEX + 7] = 0.0;  // gait 2 offset
    input[COMMAND_START_INDEX + 8] = 0.8;

    input[COMMAND_START_INDEX + 9] = 0.10 * SWING_HEIGHT_SCALE;
    input[COMMAND_START_INDEX + 10] = 0.0 * BODY_PITCH_SCALE;
    input[COMMAND_START_INDEX + 11] = 0.0 * BODY_ROLL_SCALE;
    input[COMMAND_START_INDEX + 12] = 0.18 * STANCE_WIDTH_SCALE;
    input[COMMAND_START_INDEX + 13] = 0.0 * STANCE_LENGTH_SCALE;

    input[COMMAND_START_INDEX + 14] = 0.0 * AUX_REWARD_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::fillJointResiduals(vector_t &input, const wtw::State &state) {
    constexpr scalar_t JOINT_RESIDUAL_SCALE = 1.00;
    // https://github.com/Teddy-Liao/walk-these-ways-go2/blob/ed4cedecfc4f18f4d1cccd1a605cedc5bd111af9/go2_gym/envs/go2/go2_config.py#L12
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

    input[JOINT_RESIDUALS_START_INDEX + 0] = FL_hip_joint_residual * JOINT_RESIDUAL_SCALE;
    input[JOINT_RESIDUALS_START_INDEX + 1] = FL_thigh_joint_residual * JOINT_RESIDUAL_SCALE;
    input[JOINT_RESIDUALS_START_INDEX + 2] = FL_calf_joint_residual * JOINT_RESIDUAL_SCALE;

    input[JOINT_RESIDUALS_START_INDEX + 3] = FR_hip_joint_residual * JOINT_RESIDUAL_SCALE;
    input[JOINT_RESIDUALS_START_INDEX + 4] = FR_thigh_joint_residual * JOINT_RESIDUAL_SCALE;
    input[JOINT_RESIDUALS_START_INDEX + 5] = FR_calf_joint_residual * JOINT_RESIDUAL_SCALE;

    input[JOINT_RESIDUALS_START_INDEX + 6] = RL_hip_joint_residual * JOINT_RESIDUAL_SCALE;
    input[JOINT_RESIDUALS_START_INDEX + 7] = RL_thigh_joint_residual * JOINT_RESIDUAL_SCALE;
    input[JOINT_RESIDUALS_START_INDEX + 8] = RL_calf_joint_residual * JOINT_RESIDUAL_SCALE;

    input[JOINT_RESIDUALS_START_INDEX + 9] = RR_hip_joint_residual * JOINT_RESIDUAL_SCALE;
    input[JOINT_RESIDUALS_START_INDEX + 10] = RR_thigh_joint_residual * JOINT_RESIDUAL_SCALE;
    input[JOINT_RESIDUALS_START_INDEX + 11] = RR_calf_joint_residual * JOINT_RESIDUAL_SCALE;

    defaultJointAngles_[0] = defaultJointAngles["FL_hip_joint"];
    defaultJointAngles_[1] = defaultJointAngles["FL_thigh_joint"];
    defaultJointAngles_[2] = defaultJointAngles["FL_calf_joint"];
    defaultJointAngles_[3] = defaultJointAngles["FR_hip_joint"];
    defaultJointAngles_[4] = defaultJointAngles["FR_thigh_joint"];
    defaultJointAngles_[5] = defaultJointAngles["FR_calf_joint"];
    defaultJointAngles_[6] = defaultJointAngles["RL_hip_joint"];
    defaultJointAngles_[7] = defaultJointAngles["RL_thigh_joint"];
    defaultJointAngles_[8] = defaultJointAngles["RL_calf_joint"];
    defaultJointAngles_[9] = defaultJointAngles["RR_hip_joint"];
    defaultJointAngles_[10] = defaultJointAngles["RR_thigh_joint"];
    defaultJointAngles_[11] = defaultJointAngles["RR_calf_joint"];
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::fillJointVelocities(vector_t &input, const wtw::State &state) {
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

    input[JOINT_VELOCITIES_START_INDEX + 0] = FL_hip_joint_velocity * JOINT_VELOCITY_SCALE;
    input[JOINT_VELOCITIES_START_INDEX + 1] = FL_thigh_joint_velocity * JOINT_VELOCITY_SCALE;
    input[JOINT_VELOCITIES_START_INDEX + 2] = FL_calf_joint_velocity * JOINT_VELOCITY_SCALE;

    input[JOINT_VELOCITIES_START_INDEX + 3] = FR_hip_joint_velocity * JOINT_VELOCITY_SCALE;
    input[JOINT_VELOCITIES_START_INDEX + 4] = FR_thigh_joint_velocity * JOINT_VELOCITY_SCALE;
    input[JOINT_VELOCITIES_START_INDEX + 5] = FR_calf_joint_velocity * JOINT_VELOCITY_SCALE;

    input[JOINT_VELOCITIES_START_INDEX + 6] = RL_hip_joint_velocity * JOINT_VELOCITY_SCALE;
    input[JOINT_VELOCITIES_START_INDEX + 7] = RL_thigh_joint_velocity * JOINT_VELOCITY_SCALE;
    input[JOINT_VELOCITIES_START_INDEX + 8] = RL_calf_joint_velocity * JOINT_VELOCITY_SCALE;

    input[JOINT_VELOCITIES_START_INDEX + 9] = RR_hip_joint_velocity * JOINT_VELOCITY_SCALE;
    input[JOINT_VELOCITIES_START_INDEX + 10] = RR_thigh_joint_velocity * JOINT_VELOCITY_SCALE;
    input[JOINT_VELOCITIES_START_INDEX + 11] = RR_calf_joint_velocity * JOINT_VELOCITY_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::fillLastAction(vector_t &input, const wtw::State &state) {
    for (int i = 0; i < 12; ++i) {
        input[LAST_ACTION_START_INDEX + i] = clip(lastAction_[i], -100.0, 100.0);
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::fillLastLastAction(vector_t &input, const wtw::State &state) {
    for (int i = 0; i < 12; ++i) {
        input[LAST_LAST_ACTION_START_INDEX + i] = clip(lastLastAction_[i], -100.0, 100.0);
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Np3oController::fillClockInputs(vector_t &input, scalar_t currentTime, scalar_t dt) {
    // Update gait indices
    auto frequency = input[COMMAND_START_INDEX + 4];
    auto phase = input[COMMAND_START_INDEX + 5];
    auto offset = input[COMMAND_START_INDEX + 6];
    auto bound = input[COMMAND_START_INDEX + 7];
    auto duration = input[COMMAND_START_INDEX + 8];

    gaitIndex_ = std::remainder(gaitIndex_ + dt * frequency, 1.0);

    auto footIndex0 = gaitIndex_ + phase + offset + bound;
    auto footIndex1 = gaitIndex_ + offset;
    auto footIndex2 = gaitIndex_ + bound;
    auto footIndex3 = gaitIndex_ + phase;

    input[CLOCK_INPUTS_START_INDEX + 0] = std::sin(2.0 * M_PI * footIndex0);
    input[CLOCK_INPUTS_START_INDEX + 1] = std::sin(2.0 * M_PI * footIndex1);
    input[CLOCK_INPUTS_START_INDEX + 2] = std::sin(2.0 * M_PI * footIndex2);
    input[CLOCK_INPUTS_START_INDEX + 3] = std::sin(2.0 * M_PI * footIndex3);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> Np3oController::getMotorCommands(const vector_t &jointAngles) {
    // TODO: Remove this, this one should be loaded from the config
    std::vector<std::string> jointNames = {"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                           "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

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
wtw::State Np3oController::getWtwState() {
    const vector_t &stateSubscriberState = state_.x;
    wtw::State ret;

    // Base position
    ret.basePositionWorld = stateSubscriberState.segment<3>(3);

    // Base orientation
    // Pinocchio and ocs2 both use a different euler angle logic
    // https://github.com/stack-of-tasks/pinocchio/blob/ac0b1aa6b18931bed60d0657de3c7680a4037dd3/include/pinocchio/math/rpy.hxx#L51
    // https://github.com/leggedrobotics/ocs2/blob/164c26b46bed5d24cd03d90588db8980d03a4951/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_commands/src/TerrainAdaptation.cpp#L20
    vector3_t ocs2rpy = stateSubscriberState.segment<3>(0);
    tbai::quaternion_t q = tbai::ocs2rpy2quat(ocs2rpy);
    auto Rwb = q.toRotationMatrix();
    ret.baseOrientationWorld = (wtw::State::Vector4() << q.x(), q.y(), q.z(), q.w()).finished();

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