#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_g1_rl/G1RlController.hpp>
#include <tbai_g1_rl/HistoryBuffer.hpp>

namespace tbai {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
static inline int mod(int a, int b) {
    return (a % b + b) % b;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
static inline scalar_t clip(scalar_t x, scalar_t min, scalar_t max) {
    return std::max(min, std::min(x, max));
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
G1RlController::G1RlController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                               const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen)
    : stateSubscriberPtr_(stateSubscriberPtr), refVelGen_(refVelGen), historyBuffer_(96, 5) {
    logger_ = tbai::getLogger("g1_rl");

    gaitIndex_ = 0.0;

    // Load parameters
    kp_ = tbai::fromGlobalConfig<scalar_t>("g1_rl/kp");
    kd_ = tbai::fromGlobalConfig<scalar_t>("g1_rl/kd");
    useActionFilter_ = tbai::fromGlobalConfig<bool>("g1_rl/use_action_filter");

    TBAI_LOG_INFO(logger_, "Use action filter: {}", useActionFilter_);
    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    lastAction_ = tbai::vector_t::Zero(jointNames_.size());
    filterLastAction_ = tbai::vector_t::Zero(jointNames_.size());

    defaultJointAngles_ = tbai::vector_t::Zero(jointNames_.size());

    // Taken from https://github.com/unitreerobotics/unitree_rl_lab/blob/a270823d76a0d0aae8e20a299795c432138a603b/deploy/robots/g1_29dof/config/policy/velocity/v0/params/deploy.yaml
    joint_ids_map_ = {0,  6,  12, 1,  7,  13, 2,  8,  14, 3,  9,  15, 22, 4, 10,
                      16, 23, 5,  11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28};
    stiffness_ = {100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 200.0, 200.0, 200.0,
                  40.0,  40.0,  40.0,  40.0,  40.0, 40.0, 40.0,  40.0,  40.0,  40.0,  40.0, 40.0, 40.0,  40.0};
    damping_ = {2.0,  2.0,  2.0,  4.0,  2.0,  2.0,  2.0,  2.0,  2.0,  4.0,  2.0,  2.0,  5.0,  5.0, 5.0,
                10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    default_joint_pos_ = {-0.1, -0.1,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.3,   0.3, 0.3, 0.3, -0.2, -0.2,
                          0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.97, 0.97, 0.15, -0.15, 0.0, 0.0, 0.0, 0.0};
    action_scale_ = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25,
                     0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
    action_offset_ = {-0.1, -0.1,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.3,   0.3, 0.3, 0.3, -0.2, -0.2,
                      0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.97, 0.97, 0.15, -0.15, 0.0, 0.0, 0.0, 0.0};
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool G1RlController::isSupported(const std::string &controllerType) {
    return controllerType == "G1RL";
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool G1RlController::checkStability() const {
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
tbai::vector_t G1RlController::getObsProprioceptive(const g1_rl::State &state, scalar_t currentTime, scalar_t dt) {
    vector_t input(96);
    fillBaseAngularVelocity(input, state);
    fillProjectedGravity(input, state);
    fillJointResiduals(input, state);
    fillJointVelocities(input, state);
    fillLastAction(input, state);
    fillCommand(input, state, currentTime, dt);

    return input;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
tbai::vector_t G1RlController::getObsHistory() {
    return historyBuffer_.getFinalObservation();
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void G1RlController::updateObsHistory(const tbai::vector_t &observation) {
    historyBuffer_.addObservation(observation);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> G1RlController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    g1RlState_ = getG1RlState();
    auto &state = g1RlState_;

    vector_t input = getObsProprioceptive(state, currentTime, dt);
    updateObsHistory(input);
    vector_t networkInput = historyBuffer_.getFinalObservation();
    std::cout << "Network input: " << networkInput.transpose() << std::endl << std::endl << std::endl;

    // TODO: Update actions
    vector_t actions = vector_t::Zero(jointNames_.size());
    for (int i = 0; i < jointNames_.size(); ++i) {
        actions[i] = action_offset_[i];
    }

    lastAction_ = actions;

    // Pack actions into motor commands
    std::vector<tbai::MotorCommand> motorCommands;
    motorCommands.resize(jointNames_.size());
    for (int i = 0; i < jointNames_.size(); ++i) {
        tbai::MotorCommand &command = motorCommands[i];
        command.joint_name = jointNames_[joint_ids_map_[i]];
        command.desired_position = actions[i];
        command.desired_velocity = 0.0;
        command.kp = stiffness_[joint_ids_map_[i]];
        command.kd = damping_[joint_ids_map_[i]];
        command.torque_ff = 0.0;
    }
    return motorCommands;
}

g1_rl::State G1RlController::getG1RlState() {
    const vector_t &stateSubscriberState = state_.x;
    g1_rl::State ret;

    // Base position
    ret.basePositionWorld = stateSubscriberState.segment<3>(3);

    // Base orientation
    // Pinocchio and ocs2 both use a different euler angle logic
    // https://github.com/stack-of-tasks/pinocchio/blob/ac0b1aa6b18931bed60d0657de3c7680a4037dd3/include/pinocchio/math/rpy.hxx#L51
    // https://github.com/leggedrobotics/ocs2/blob/164c26b46bed5d24cd03d90588db8980d03a4951/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_commands/src/TerrainAdaptation.cpp#L20
    vector3_t ocs2rpy = stateSubscriberState.segment<3>(0);
    tbai::quaternion_t q = tbai::ocs2rpy2quat(ocs2rpy);
    auto Rwb = q.toRotationMatrix();
    ret.baseOrientationWorld = (g1_rl::State::Vector4() << q.x(), q.y(), q.z(), q.w()).finished();

    // Base angular velocity
    ret.baseAngularVelocityBase = stateSubscriberState.segment<3>(6);

    // Base linear velocity
    ret.baseLinearVelocityBase = stateSubscriberState.segment<3>(9);

    // Normalized gravity vector
    ret.normalizedGravityBase = Rwb.transpose() * (vector3_t() << 0, 0, -1).finished();

    // Joint positions
    for (int i = 0; i < jointNames_.size(); ++i) {
        ret.jointPositions[i] = stateSubscriberState.segment<29>(12)[joint_ids_map_[i]];
    }

    // Joint velocities
    for (int i = 0; i < jointNames_.size(); ++i) {
        ret.jointVelocities[i] = stateSubscriberState.segment<29>(12 + 29)[joint_ids_map_[i]];
    }

    return ret;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void G1RlController::fillBaseAngularVelocity(vector_t &input, const g1_rl::State &state) {
    constexpr scalar_t BASE_ANGULAR_VELOCITY_SCALE = 0.2;
    input[BASE_ANGULAR_VELOCITY_START_INDEX + 0] = state.baseAngularVelocityBase[0] * BASE_ANGULAR_VELOCITY_SCALE;
    input[BASE_ANGULAR_VELOCITY_START_INDEX + 1] = state.baseAngularVelocityBase[1] * BASE_ANGULAR_VELOCITY_SCALE;
    input[BASE_ANGULAR_VELOCITY_START_INDEX + 2] = state.baseAngularVelocityBase[2] * BASE_ANGULAR_VELOCITY_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void G1RlController::fillProjectedGravity(vector_t &input, const g1_rl::State &state) {
    constexpr scalar_t PROJECTED_GRAVITY_SCALE = 1.0;
    input[PROJECTED_GRAVITY_START_INDEX + 0] = state.normalizedGravityBase[0] * PROJECTED_GRAVITY_SCALE;
    input[PROJECTED_GRAVITY_START_INDEX + 1] = state.normalizedGravityBase[1] * PROJECTED_GRAVITY_SCALE;
    input[PROJECTED_GRAVITY_START_INDEX + 2] = state.normalizedGravityBase[2] * PROJECTED_GRAVITY_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void G1RlController::fillJointResiduals(vector_t &input, const g1_rl::State &state) {
    constexpr scalar_t JOINT_RESIDUAL_SCALE = 1.00;
    for (int i = 0; i < jointNames_.size(); ++i) {
        input[DOF_RESIDUALS_START_INDEX + i] = (state.jointPositions[i] - default_joint_pos_[i]) * JOINT_RESIDUAL_SCALE;
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void G1RlController::fillJointVelocities(vector_t &input, const g1_rl::State &state) {
    constexpr scalar_t JOINT_VELOCITY_SCALE = 0.05;
    for (int i = 0; i < jointNames_.size(); ++i) {
        input[DOF_VELOCITIES_START_INDEX + i] = state.jointVelocities[i] * JOINT_VELOCITY_SCALE;
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void G1RlController::fillLastAction(vector_t &input, const g1_rl::State &state) {
    for (int i = 0; i < jointNames_.size(); ++i) {
        input[LAST_ACTION_START_INDEX + i] = lastAction_[i];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void G1RlController::fillCommand(vector_t &input, const g1_rl::State &state, scalar_t currentTime, scalar_t dt) {
    constexpr scalar_t LIN_VEL_SCALE = 2.0;
    constexpr scalar_t ANG_VEL_SCALE = 0.25 * 5;  // 5 because it was too slow

    auto command = refVelGen_->getReferenceVelocity(currentTime, dt);
    const scalar_t velocity_x = command.velocity_x;
    const scalar_t velocity_y = command.velocity_y;
    const scalar_t yaw_rate = command.yaw_rate;

    auto norm = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);

    input[COMMAND_START_INDEX + 0] = norm > 0.2 ? velocity_x * LIN_VEL_SCALE : 0.0;
    input[COMMAND_START_INDEX + 1] = norm > 0.2 ? velocity_y * LIN_VEL_SCALE : 0.0;
    input[COMMAND_START_INDEX + 2] = yaw_rate * ANG_VEL_SCALE;
}

}  // namespace tbai
