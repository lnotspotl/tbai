// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <tbai_wtw/WtwController.hpp>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>

#include <tbai_wtw/EigenTorch.hpp>

namespace tbai {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
static inline int mod(int a, int b) {
    return (a % b + b) % b;
}

WtwController::WtwController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                             const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen)
    : WtwController::WtwController(tbai::getEnvAs<std::string>("TBAI_ROBOT_DESCRIPTION_PATH"), stateSubscriberPtr,
                                   refVelGen) {}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
WtwController::WtwController(const std::string &urdfPathOrString,
                             const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                             const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen)
    : stateSubscriberPtr_(stateSubscriberPtr), refVelGen_(refVelGen) {
    logger_ = tbai::getLogger("wtw_controller");

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

    auto hfRepo = tbai::fromGlobalConfig<std::string>("wtw_controller/hf_repo");
    auto hfModel = tbai::fromGlobalConfig<std::string>("wtw_controller/hf_model");

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

}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void WtwController::setupPinocchioModel(const std::string &urdfString) {
    pinocchio::urdf::buildModelFromXML(urdfString, pinocchio::JointModelFreeFlyer(), pinocchioModel_);
    pinocchioData_ = pinocchio::Data(pinocchioModel_);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/**********************************************************************************************
 * *************************/
bool WtwController::isSupported(const std::string &controllerType) {
    if (controllerType == "WTW") {
        return true;
    }
    return false;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool WtwController::checkStability() const {
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
at::Tensor WtwController::getNNInput(const State &state, scalar_t currentTime, scalar_t dt) {
    return at::Tensor();
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> WtwController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    auto state = getWtwState();

    // Do not keep track of gradients
    torch::NoGradGuard no_grad;

    auto ts1 = std::chrono::high_resolution_clock::now();
    at::Tensor nnInput = getNNInput(state, currentTime, dt);
    auto t2 = std::chrono::high_resolution_clock::now();

    // perform forward pass
    auto ts3 = std::chrono::high_resolution_clock::now();
    at::Tensor out = model_.forward({nnInput.view({1, 70})}).toTensor().squeeze();
    auto t4 = std::chrono::high_resolution_clock::now();

    // Send command
    auto ret = getMotorCommands(tbai::torch2vector(out));

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
void WtwController::fillGravity(vector_t &input, const State &state) {
    input[GRAVITY_START_INDEX + 0] = state.normalizedGravityBase[0];
    input[GRAVITY_START_INDEX + 1] = state.normalizedGravityBase[1];
    input[GRAVITY_START_INDEX + 2] = state.normalizedGravityBase[2];
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void WtwController::fillCommand(vector_t &input, const State &state, scalar_t currentTime, scalar_t dt) {

}


/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void WtwController::fillJointResiduals(vector_t &input, const State &state) {

}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void WtwController::fillJointVelocities(vector_t &input, const State &state) {

}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void WtwController::fillLastAction(vector_t &input, const State &state) {

}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void WtwController::fillLastLastAction(vector_t &input, const State &state) {

}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void WtwController::fillClockInputs(vector_t &input, scalar_t currentTime, scalar_t dt) {

}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> WtwController::getMotorCommands(const vector_t &jointAngles) {
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
State WtwController::getWtwState() {
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

}  // namespace tbai