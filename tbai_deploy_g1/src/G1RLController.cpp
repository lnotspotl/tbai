#include "tbai_deploy_g1/G1RLController.hpp"

#include <algorithm>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_reference/ReferenceVelocity.hpp>

namespace tbai {
namespace g1 {

G1RLController::G1RLController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                               const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                               const std::string &policyPath)
    : stateSubscriberPtr_(stateSubscriberPtr), refVelGenPtr_(refVelGenPtr) {
    logger_ = tbai::getLogger("G1RLController");
    TBAI_LOG_INFO(logger_, "Initializing G1RLController");

    // Initialize vectors
    lastAction_ = vector_t::Zero(G1_NUM_JOINTS);
    action_ = vector_t::Zero(G1_NUM_JOINTS);
    fullObservation_ = vector_t::Zero(TOTAL_OBS_SIZE);

    // Load configuration
    defaultJointPos_ = vector_t(G1_NUM_JOINTS);
    actionScale_ = vector_t(G1_NUM_JOINTS);
    actionOffset_ = vector_t(G1_NUM_JOINTS);
    stiffness_ = vector_t(G1_NUM_JOINTS);
    damping_ = vector_t(G1_NUM_JOINTS);

    // Load from config
    auto defaultJointPosVec = tbai::fromGlobalConfig<std::vector<double>>("g1_controller/default_joint_pos");
    auto actionScaleVec = tbai::fromGlobalConfig<std::vector<double>>("g1_controller/action_scale");
    auto stiffnessVec = tbai::fromGlobalConfig<std::vector<double>>("g1_controller/stiffness");
    auto dampingVec = tbai::fromGlobalConfig<std::vector<double>>("g1_controller/damping");
    jointIdsMap_ = tbai::fromGlobalConfig<std::vector<int>>("g1_controller/joint_ids_map");

    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        defaultJointPos_[i] = defaultJointPosVec[i];
        actionScale_[i] = actionScaleVec[i];
        actionOffset_[i] = defaultJointPosVec[i];  // Offset is same as default
        stiffness_[i] = stiffnessVec[i];
        damping_[i] = dampingVec[i];
    }

    angVelScale_ = tbai::fromGlobalConfig<double>("g1_controller/ang_vel_scale", 0.2);
    jointVelScale_ = tbai::fromGlobalConfig<double>("g1_controller/joint_vel_scale", 0.05);

    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Initialize observation term histories with zeros
    for (int i = 0; i < HISTORY_LENGTH; ++i) {
        angVelHistory_.push_back(vector3_t::Zero());
        projGravHistory_.push_back(vector3_t::Zero());
        velCmdHistory_.push_back(vector3_t::Zero());
        jointPosRelHistory_.push_back(vector_t::Zero(G1_NUM_JOINTS));
        jointVelRelHistory_.push_back(vector_t::Zero(G1_NUM_JOINTS));
        lastActionHistory_.push_back(vector_t::Zero(G1_NUM_JOINTS));
    }

    // Initialize ONNX runtime
    initOnnxRuntime(policyPath);

    TBAI_LOG_INFO(logger_, "G1RLController initialized successfully");
}

G1RLController::~G1RLController() {
    TBAI_LOG_INFO(logger_, "Destroying G1RLController");
}

void G1RLController::initOnnxRuntime(const std::string &policyPath) {
    TBAI_LOG_INFO(logger_, "Loading ONNX policy from: {}", policyPath);

    ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "G1RLController");

    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, policyPath.c_str(), sessionOptions);
    memoryInfo_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    Ort::AllocatorWithDefaultOptions allocator;

    auto inputNamePtr = ortSession_->GetInputNameAllocated(0, allocator);
    inputNames_.push_back(strdup(inputNamePtr.get()));

    auto outputNamePtr = ortSession_->GetOutputNameAllocated(0, allocator);
    outputNames_.push_back(strdup(outputNamePtr.get()));

    TBAI_LOG_INFO(logger_, "ONNX model loaded. Input: {}, Output: {}", inputNames_[0], outputNames_[0]);
}

void G1RLController::preStep(scalar_t currentTime, scalar_t dt) {
    state_ = stateSubscriberPtr_->getLatestState();
}

vector3_t G1RLController::computeProjectedGravity(const quaternion_t &orientation) const {
    vector3_t gravityWorld(0.0, 0.0, -1.0);
    matrix3_t R = orientation.toRotationMatrix();
    return R.transpose() * gravityWorld;
}

void G1RLController::buildObservation(scalar_t currentTime, scalar_t dt) {
    vector3_t rpy = state_.x.segment<3>(0);
    vector3_t baseAngVel = state_.x.segment<3>(6);
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);

    quaternion_t orientation = tbai::ocs2rpy2quat(rpy);

    auto refVel = refVelGenPtr_->getReferenceVelocity(currentTime, dt);

    vector3_t velCmd;
    velCmd[0] = std::clamp(refVel.velocity_x, -0.5, 1.0);
    velCmd[1] = std::clamp(refVel.velocity_y, -0.3, 0.3);
    velCmd[2] = std::clamp(refVel.yaw_rate, -0.5, 0.5);

    vector3_t angVelScaled = baseAngVel * angVelScale_;

    vector3_t projGrav = computeProjectedGravity(orientation);

    vector_t jointPosRel(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int mappedIdx = jointIdsMap_[i];
        jointPosRel[i] = jointPos[mappedIdx] - defaultJointPos_[i];
    }

    vector_t jointVelRel(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int mappedIdx = jointIdsMap_[i];
        jointVelRel[i] = jointVel[mappedIdx] * jointVelScale_;
    }

    angVelHistory_.push_back(angVelScaled);
    angVelHistory_.pop_front();

    projGravHistory_.push_back(projGrav);
    projGravHistory_.pop_front();

    velCmdHistory_.push_back(velCmd);
    velCmdHistory_.pop_front();

    jointPosRelHistory_.push_back(jointPosRel);
    jointPosRelHistory_.pop_front();

    jointVelRelHistory_.push_back(jointVelRel);
    jointVelRelHistory_.pop_front();

    lastActionHistory_.push_back(lastAction_);
    lastActionHistory_.pop_front();
}

void G1RLController::updateHistory() {
    int idx = 0;

    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment<3>(idx) = angVelHistory_[h];
        idx += 3;
    }

    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment<3>(idx) = projGravHistory_[h];
        idx += 3;
    }

    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment<3>(idx) = velCmdHistory_[h];
        idx += 3;
    }

    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment(idx, G1_NUM_JOINTS) = jointPosRelHistory_[h];
        idx += G1_NUM_JOINTS;
    }

    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment(idx, G1_NUM_JOINTS) = jointVelRelHistory_[h];
        idx += G1_NUM_JOINTS;
    }

    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment(idx, G1_NUM_JOINTS) = lastActionHistory_[h];
        idx += G1_NUM_JOINTS;
    }
}

void G1RLController::runInference() {
    // Prepare input tensor
    std::vector<int64_t> inputShape = {1, TOTAL_OBS_SIZE};
    std::vector<float> inputData(TOTAL_OBS_SIZE);
    for (int i = 0; i < TOTAL_OBS_SIZE; ++i) {
        inputData[i] = static_cast<float>(fullObservation_[i]);
    }

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(*memoryInfo_, inputData.data(), inputData.size(),
                                                             inputShape.data(), inputShape.size());

    // Run inference
    auto outputTensors =
        ortSession_->Run(Ort::RunOptions{nullptr}, inputNames_.data(), &inputTensor, 1, outputNames_.data(), 1);

    // Get actions
    float *outputData = outputTensors[0].GetTensorMutableData<float>();
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        action_[i] = static_cast<scalar_t>(outputData[i]);
    }

    // Store actions for next step
    lastAction_ = action_;
}

std::vector<tbai::MotorCommand> G1RLController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    buildObservation(currentTime, dt);
    updateHistory();
    runInference();

    std::vector<tbai::MotorCommand> commands;
    commands.reserve(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        tbai::MotorCommand cmd;

        // Map from policy order back to DDS order
        int ddsIdx = jointIdsMap_[i];
        cmd.joint_name = jointNames_[ddsIdx];

        // Apply action scaling and offset (scale and offset are in policy order)
        cmd.desired_position = actionScale_[i] * action_[i] + actionOffset_[i];
        cmd.desired_velocity = 0.0;

        // Stiffness and damping are stored in DDS order
        cmd.kp = stiffness_[ddsIdx];
        cmd.kd = damping_[ddsIdx];
        cmd.torque_ff = 0.0;

        commands.push_back(cmd);
    }

    return commands;
}

bool G1RLController::isSupported(const std::string &controllerType) {
    return controllerType == "G1RLController" || controllerType == "g1_rl" || controllerType == "g1";
}

bool G1RLController::ok() const {
    return true;
}

bool G1RLController::checkStability() const {
    constexpr scalar_t maxAngle = 1.0;  // radians
    scalar_t roll = std::abs(state_.x[0]);
    scalar_t pitch = std::abs(state_.x[1]);

    if (roll > maxAngle || pitch > maxAngle) {
        TBAI_LOG_WARN(logger_, "Instability detected: roll={:.2f}, pitch={:.2f}", roll, pitch);
        return false;
    }
    return true;
}

void G1RLController::changeController(const std::string &controllerType, scalar_t currentTime) {
    TBAI_LOG_INFO(logger_, "Changing to G1RLController");

    // Reset action history
    lastAction_.setZero();

    // Get current state
    state_ = stateSubscriberPtr_->getLatestState();

    // Compute current observation terms
    vector3_t rpy = state_.x.segment<3>(0);
    vector3_t baseAngVel = state_.x.segment<3>(6);
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);
    quaternion_t orientation = tbai::ocs2rpy2quat(rpy);

    vector3_t angVelScaled = baseAngVel * angVelScale_;
    vector3_t projGrav = computeProjectedGravity(orientation);
    vector3_t velCmd = vector3_t::Zero();  // Start with zero velocity command

    vector_t jointPosRel(G1_NUM_JOINTS);
    vector_t jointVelRel(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int mappedIdx = jointIdsMap_[i];
        jointPosRel[i] = jointPos[mappedIdx] - defaultJointPos_[i];
        jointVelRel[i] = jointVel[mappedIdx] * jointVelScale_;
    }

    angVelHistory_.clear();
    projGravHistory_.clear();
    velCmdHistory_.clear();
    jointPosRelHistory_.clear();
    jointVelRelHistory_.clear();
    lastActionHistory_.clear();

    for (int i = 0; i < HISTORY_LENGTH; ++i) {
        angVelHistory_.push_back(angVelScaled);
        projGravHistory_.push_back(projGrav);
        velCmdHistory_.push_back(velCmd);
        jointPosRelHistory_.push_back(jointPosRel);
        jointVelRelHistory_.push_back(jointVelRel);
        lastActionHistory_.push_back(lastAction_);
    }

    TBAI_LOG_INFO(logger_, "Observation history initialized with current state");
}

}  // namespace g1
}  // namespace tbai
