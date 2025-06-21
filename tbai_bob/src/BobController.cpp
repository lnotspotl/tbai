// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <tbai_bob/BobController.hpp>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/Env.hpp>

namespace tbai {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
static inline int mod(int a, int b) {
    return (a % b + b) % b;
}

BobController::BobController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                             const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen)
    : BobController::BobController(tbai::getEnvAs<std::string>("TBAI_ROBOT_DESCRIPTION_PATH"),
                                   stateSubscriberPtr, refVelGen) {}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
BobController::BobController(const std::string &urdfString,
                             const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                             const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen)
    : stateSubscriberPtr_(stateSubscriberPtr), refVelGen_(refVelGen) {
    logger_ = tbai::getLogger("bob_controller");

    // Load parameters
    kp_ = tbai::fromGlobalConfig<scalar_t>("bob_controller/kp");
    kd_ = tbai::fromGlobalConfig<scalar_t>("bob_controller/kd");
    blind_ = tbai::fromGlobalConfig<bool>("bob_controller/blind");
    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    ik_ = getInverseKinematicsUnique();
    cpg_ = getCentralPatternGeneratorUnique();

    // Load URDF string from file
    std::ifstream urdfFile(urdfString);
    if (!urdfFile.is_open()) {
        TBAI_THROW("Could not open URDF file: {}", urdfString);
    }
    std::stringstream buffer;
    buffer << urdfFile.rdbuf();
    std::string urdfString_ = buffer.str();

    setupPinocchioModel(urdfString_);
    generateSamplingPositions();

    auto hfRepo = tbai::fromGlobalConfig<std::string>("bob_controller/hf_repo");
    auto hfModel = tbai::fromGlobalConfig<std::string>("bob_controller/hf_model");

    TBAI_LOG_INFO(logger_, "Loading HF model: {}/{}", hfRepo, hfModel);
    auto modelPath = tbai::downloadFromHuggingFace(hfRepo, hfModel);
    TBAI_LOG_INFO(logger_, "Model downloaded to: {}", modelPath);

    try {
        model_ = torch::jit::load(modelPath);
    } catch (const c10::Error &e) {
        TBAI_THROW("Could not load model from: {}\nError: {}", modelPath, e.what());
    }

    TBAI_LOG_INFO(logger_, "Model loaded");

    std::vector<torch::jit::IValue> stack;
    model_.get_method("set_hidden_size")(stack);

    TBAI_LOG_INFO(logger_, "Resetting history");
    resetHistory();
    TBAI_LOG_INFO(logger_, "History reset");

    TBAI_LOG_INFO(logger_, "Solving IK");
    auto legHeights = cpg_->legHeights();
    jointAngles2_ = ik_->solve(legHeights);

    TBAI_LOG_INFO(logger_, "Solved IK");

    standJointAngles_ = tbai::fromGlobalConfig<vector_t>("static_controller/stand_controller/joint_angles");
    TBAI_LOG_INFO(logger_, "Stand joint angles loaded");

    ACTION_SCALE = tbai::fromGlobalConfig<scalar_t>("bob_controller/action_scale");
    TBAI_LOG_INFO(logger_, "Action scale: {}", ACTION_SCALE);

    LIN_VEL_SCALE = tbai::fromGlobalConfig<scalar_t>("bob_controller/lin_vel_scale");
    TBAI_LOG_INFO(logger_, "Lin vel scale: {}", LIN_VEL_SCALE);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::setupPinocchioModel(const std::string &urdfString) {
    pinocchio::urdf::buildModelFromXML(urdfString, pinocchio::JointModelFreeFlyer(), pinocchioModel_);
    pinocchioData_ = pinocchio::Data(pinocchioModel_);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/**********************************************************************************************
 * *************************/
bool BobController::isSupported(const std::string &controllerType) {
    if (controllerType == "BOB") {
        return true;
    }
    return false;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool BobController::checkStability() const {
    const auto &state = stateSubscriberPtr_->getLatestRbdState();
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

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> BobController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    auto state = getBobnetState();

    // Do not keep track of gradients
    torch::NoGradGuard no_grad;

    auto ts1 = std::chrono::high_resolution_clock::now();
    at::Tensor nnInput = getNNInput(state, currentTime, dt);
    auto t2 = std::chrono::high_resolution_clock::now();

    cpg_->step(dt);

    // perform forward pass
    auto ts3 = std::chrono::high_resolution_clock::now();
    at::Tensor out = model_.forward({nnInput.view({1, getNNInputSize()})}).toTensor().squeeze();
    auto t4 = std::chrono::high_resolution_clock::now();

    // action from NN
    at::Tensor action = out.index({Slice(0, COMMAND_SIZE)});

    // reconstructed hidden information
    hidden_ = out.index({Slice(COMMAND_SIZE, None)});

    // unpack action
    at::Tensor phaseOffsets = action.index({Slice(0, 4)});
    vector_t phaseOffsetsVec(4);
    for (size_t i = 0; i < 4; ++i) {
        phaseOffsetsVec[i] = (phaseOffsets[i].item<float>() * ACTION_SCALE);
    }

    at::Tensor jointResiduals = action.index({Slice(0, COMMAND_SIZE)});
    vector_t jointResidualsVec(12);
    for (size_t i = 0; i < 12; ++i) {
        jointResidualsVec[i] = (jointResiduals[i].item<float>() * ACTION_SCALE);
    }

    // compute ik
    auto legHeights = cpg_->legHeights();
    jointAngles2_ = ik_->solve(legHeights) + jointResidualsVec;

    // Send command
    auto ret = getMotorCommands(jointAngles2_);

    // Compute IK again for buffer, TODO: Remove this calculation
    cpg_->step(-dt);
    legHeights = cpg_->legHeights();
    jointAngles2_ = ik_->solve(legHeights);

    // update history buffer
    updateHistory(nnInput, action, state);

    // update central pattern generator
    cpg_->step(dt);
    auto t5 = std::chrono::high_resolution_clock::now();

    TBAI_LOG_INFO_THROTTLE(logger_,
        10.0, "NN input preparations took {} ms, NN forward pass took: {} ms. Total controller step took: {} ms",
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - ts1).count() / 1000.0,
        std::chrono::duration_cast<std::chrono::microseconds>(t4 - ts3).count() / 1000.0,
        std::chrono::duration_cast<std::chrono::microseconds>(t5 - ts1).count() / 1000.0);

    return ret;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> BobController::getMotorCommands(const vector_t &jointAngles) {
    std::vector<tbai::MotorCommand> motorCommands;
    motorCommands.resize(jointAngles.size());
    for (size_t i = 0; i < jointAngles.size(); ++i) {
        tbai::MotorCommand &command = motorCommands[i];
        command.joint_name = jointNames_[i];
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
void BobController::resetHistory() {
    auto reset = [](auto &history, size_t history_size, size_t item_size) {
        history.clear();
        for (size_t i = 0; i < history_size; ++i) {
            history.push_back(torch::zeros({static_cast<long int>(item_size)}));
        }
    };
    reset(historyResiduals_, POSITION_HISTORY_SIZE, POSITION_SIZE);
    reset(historyVelocities_, VELOCITY_HISTORY_SIZE, VELOCITY_SIZE);
    reset(historyActions_, COMMAND_HISTORY_SIZE, COMMAND_SIZE);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
State BobController::getBobnetState() {
    const vector_t &stateSubscriberState = stateSubscriberPtr_->getLatestRbdState();
    State ret;

    // Base position
    ret.basePositionWorld = stateSubscriberState.segment<3>(3);

    // Base orientation
    // Pinocchio and ocs2 both use a different euler angle logic
    // https://github.com/stack-of-tasks/pinocchio/blob/ac0b1aa6b18931bed60d0657de3c7680a4037dd3/include/pinocchio/math/rpy.hxx#L51
    // https://github.com/leggedrobotics/ocs2/blob/164c26b46bed5d24cd03d90588db8980d03a4951/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_commands/src/TerrainAdaptation.cpp#L20
    vector3_t ocs2rpy = stateSubscriberState.segment<3>(0);
    tbai::quaternion_t q = tbai::ocs2rpy2quat(ocs2rpy);
    auto Rwb = q.toRotationMatrix();
    ret.baseOrientationWorld = (State::Vector4() << q.x(), q.y(), q.z(), q.w()).finished();

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

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
at::Tensor BobController::getNNInput(const State &state, scalar_t currentTime, scalar_t dt) {
    at::Tensor input = at::empty(getNNInputSize());

    // Fill individual sections of the nn input tensor
    fillCommand(input, currentTime, dt);
    fillGravity(input, state);
    fillBaseLinearVelocity(input, state);
    fillBaseAngularVelocity(input, state);
    fillJointResiduals(input, state);
    fillJointVelocities(input, state);
    fillHistory(input);
    fillCpg(input);
    fillHeights(input, state);
    return input;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillCommand(at::Tensor &input, scalar_t currentTime, scalar_t dt) {
    auto command = refVelGen_->getReferenceVelocity(currentTime, dt);
    input[0] = command.velocity_x * 2;
    input[1] = command.velocity_y * 2;
    input[2] = command.yaw_rate * 2;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillGravity(at::Tensor &input, const State &state) {
    input[3] = state.normalizedGravityBase[0] * GRAVITY_SCALE;
    input[4] = state.normalizedGravityBase[1] * GRAVITY_SCALE;
    input[5] = state.normalizedGravityBase[2] * GRAVITY_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillBaseLinearVelocity(at::Tensor &input, const State &state) {
    input[6] = state.baseLinearVelocityBase[0] * LIN_VEL_SCALE;
    input[7] = state.baseLinearVelocityBase[1] * LIN_VEL_SCALE;
    input[8] = state.baseLinearVelocityBase[2] * LIN_VEL_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillBaseAngularVelocity(at::Tensor &input, const State &state) {
    input[9] = state.baseAngularVelocityBase[0] * ANG_VEL_SCALE;
    input[10] = state.baseAngularVelocityBase[1] * ANG_VEL_SCALE;
    input[11] = state.baseAngularVelocityBase[2] * ANG_VEL_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillJointResiduals(at::Tensor &input, const State &state) {
    // fill joint residuals
    for (size_t i = 0; i < 12; ++i) {
        input[12 + i] = (state.jointPositions[i] - jointAngles2_[i]) * JOINT_POS_SCALE;
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillJointVelocities(at::Tensor &input, const State &state) {
    // fill joint velocities
    for (size_t i = 0; i < 12; ++i) {
        input[24 + i] = state.jointVelocities[i] * JOINT_VEL_SCALE;
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillHistory(at::Tensor &input) {
    fillHistoryResiduals(input);
    fillHistoryVelocities(input);
    fillHistoryActions(input);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillCpg(at::Tensor &input) {
    const size_t startIdx = 36 + POSITION_HISTORY_SIZE * POSITION_SIZE + VELOCITY_HISTORY_SIZE * VELOCITY_SIZE +
                            COMMAND_HISTORY_SIZE * COMMAND_SIZE;
    auto cpgObservation = cpg_->getObservation();
    for (size_t i = 0; i < 8; ++i) {
        input[startIdx + i] = cpgObservation[i];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillHeights(at::Tensor &input, const State &state) {
    const size_t startIdx = 36 + POSITION_HISTORY_SIZE * POSITION_SIZE + VELOCITY_HISTORY_SIZE * VELOCITY_SIZE +
                            COMMAND_HISTORY_SIZE * COMMAND_SIZE + 8;

    // Find yaw angle
    quaternion_t q(state.baseOrientationWorld[3], state.baseOrientationWorld[0], state.baseOrientationWorld[1],
                   state.baseOrientationWorld[2]);

    scalar_t yaw = pinocchio::rpy::matrixToRpy(q.toRotationMatrix())[2];

    // Rotate sampling points
    matrix3_t Ryaw = angleaxis_t(yaw, vector3_t::UnitZ()).toRotationMatrix();
    matrix_t rotatedSamplingPoints = Ryaw * samplingPositions_;

    // Replicate sampling points
    sampled_ = rotatedSamplingPoints.replicate<1, 4>();

    auto addOffset = [this](scalar_t x_offset, scalar_t y_offset, size_t idx) mutable {
        size_t blockStart = idx * 52;
        sampled_.block<1, 52>(0, blockStart).array() += x_offset;
        sampled_.block<1, 52>(1, blockStart).array() += y_offset;
    };
    addOffset(state.lfFootPositionWorld[0], state.lfFootPositionWorld[1], 0);
    addOffset(state.lhFootPositionWorld[0], state.lhFootPositionWorld[1], 1);
    addOffset(state.rfFootPositionWorld[0], state.rfFootPositionWorld[1], 2);
    addOffset(state.rhFootPositionWorld[0], state.rhFootPositionWorld[1], 3);

    // If blind, height samples are not used, garbage values are fine
    // sampled_ is updated for visualization purposes
    if (!blind_) {
        // sample heights
        this->atPositions(sampled_);

        // clamp third row between -1 and 1
        sampled_.row(2) = (state.basePositionWorld[2] - sampled_.row(2).array()).array() - 0.5;
        sampled_.row(2) = sampled_.row(2).cwiseMax(-1.0).cwiseMin(1.0) * HEIGHT_MEASUREMENTS_SCALE;

        // Eigen -> torch: https://discuss.pytorch.org/t/data-transfer-between-libtorch-c-and-eigen/54156/6
        float *torchPtr = input.index({Slice(startIdx, None)}).data_ptr<float>();
        Eigen::Map<Eigen::VectorXf> ef(torchPtr, 4 * 52, 1);
        ef = sampled_.row(2).cast<float>();
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillHistoryResiduals(at::Tensor &input) {
    int ip = mod(historyResidualsIndex_ - 1, POSITION_HISTORY_SIZE);
    const size_t startIdx = 36;
    for (int i = 0; i < POSITION_HISTORY_SIZE; ++i) {
        int idx = mod(ip + i, POSITION_HISTORY_SIZE);
        input.index({Slice(startIdx + i * POSITION_SIZE, startIdx + (i + 1) * POSITION_SIZE)}) = historyResiduals_[idx];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillHistoryVelocities(at::Tensor &input) {
    int ip = mod(historyVelocitiesIndex_ - 1, VELOCITY_HISTORY_SIZE);
    const size_t startIdx = 36 + POSITION_HISTORY_SIZE * POSITION_SIZE;
    for (int i = 0; i < VELOCITY_HISTORY_SIZE; ++i) {
        int idx = mod(ip + i, VELOCITY_HISTORY_SIZE);
        input.index({Slice(startIdx + i * VELOCITY_SIZE, startIdx + (i + 1) * VELOCITY_SIZE)}) =
            historyVelocities_[idx];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillHistoryActions(at::Tensor &input) {
    int ip = mod(historyActionsIndex_ - 1, COMMAND_HISTORY_SIZE);
    const size_t startIdx = 36 + POSITION_HISTORY_SIZE * POSITION_SIZE + VELOCITY_HISTORY_SIZE * VELOCITY_SIZE;
    for (int i = 0; i < COMMAND_HISTORY_SIZE; ++i) {
        int idx = mod(ip + i, COMMAND_HISTORY_SIZE);
        input.index({Slice(startIdx + i * COMMAND_SIZE, startIdx + (i + 1) * COMMAND_SIZE)}) = historyActions_[idx];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::updateHistory(const at::Tensor &input, const at::Tensor &action, const State &state) {
    // update position history
    for (int i = 0; i < 12; ++i) {
        historyResiduals_[historyResidualsIndex_][i] = (state.jointPositions[i] - jointAngles2_[i]) * JOINT_POS_SCALE;
    }
    historyResidualsIndex_ = (historyResidualsIndex_ + 1) % POSITION_HISTORY_SIZE;

    // update velocity history
    historyVelocities_[historyVelocitiesIndex_] = input.index({jointVelocitiesSlice_});
    historyVelocitiesIndex_ = (historyVelocitiesIndex_ + 1) % VELOCITY_HISTORY_SIZE;

    // update action history
    historyActions_[historyActionsIndex_] = action;
    historyActionsIndex_ = (historyActionsIndex_ + 1) % COMMAND_HISTORY_SIZE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::generateSamplingPositions() {
    std::vector<scalar_t> Ns = {6, 8, 10, 12, 16};
    std::vector<scalar_t> rs = {0.1, 0.3, 0.5, 0.7, 0.9};

    samplingPositions_ = matrix_t::Zero(3, std::accumulate(Ns.begin(), Ns.end(), 0));

    size_t idx = 0;
    for (int i = 0; i < Ns.size(); ++i) {
        scalar_t r = rs[i];
        scalar_t N = Ns[i];
        for (int j = 0; j < N; ++j) {
            scalar_t theta = 2 * M_PI * j / N;
            scalar_t x = r * std::cos(theta);
            scalar_t y = r * std::sin(theta);
            samplingPositions_(0, idx) = x;
            samplingPositions_(1, idx) = y;
            ++idx;
        }
    }
}

}  // namespace tbai