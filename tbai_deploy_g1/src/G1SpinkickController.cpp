#include "tbai_deploy_g1/G1SpinkickController.hpp"

#include <algorithm>
#include <cstring>
#include <sstream>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_deploy_g1/MotionLoader.hpp>

namespace tbai {
namespace g1 {

G1SpinkickController::G1SpinkickController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                           const std::string &policyPath, const std::string &controllerName,
                                           bool useModelMetaConfig, float actionBeta)
    : stateSubscriberPtr_(stateSubscriberPtr),
      actionBeta_(actionBeta),
      useModelMetaConfig_(useModelMetaConfig),
      timestep_(0),
      maxTimestep_(-1),
      motionActive_(false),
      anchorBodyIndex_(0),
      controllerName_(controllerName) {
    logger_ = tbai::getLogger(controllerName_);
    TBAI_LOG_INFO(logger_, "Initializing {}", controllerName_);

    // Initialize vectors
    lastAction_ = vector_t::Zero(G1_NUM_JOINTS);
    action_ = vector_t::Zero(G1_NUM_JOINTS);
    rawAction_ = vector_t::Zero(G1_NUM_JOINTS);
    observation_ = vector_t::Zero(SPINKICK_TOTAL_OBS_SIZE);

    // Initialize motion data
    motionJointPos_ = vector_t::Zero(G1_NUM_JOINTS);
    motionJointVel_ = vector_t::Zero(G1_NUM_JOINTS);
    motionBodyPos_.setZero();
    motionBodyQuat_.setZero();

    // Load configuration
    defaultJointPos_ = tbai::fromGlobalConfig<vector_t>("g1_spinkick_controller/default_joint_pos");
    actionScale_ = tbai::fromGlobalConfig<vector_t>("g1_spinkick_controller/action_scale");
    stiffness_ = tbai::fromGlobalConfig<vector_t>("g1_spinkick_controller/stiffness");
    damping_ = tbai::fromGlobalConfig<vector_t>("g1_spinkick_controller/damping");

    // Initialize ONNX Runtime first to potentially get metadata
    initOnnxRuntime(policyPath);

    // Parse model metadata if available and enabled
    if (useModelMetaConfig_) {
        parseModelMetadata();
    }

    jointIdsMap_ = tbai::fromGlobalConfig<std::vector<int>>("g1_spinkick_controller/joint_ids_map");
    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
    maxTimestep_ = tbai::fromGlobalConfig<int>("g1_spinkick_controller/max_timestep", -1);
    actionBeta_ = tbai::fromGlobalConfig<float>("g1_spinkick_controller/action_beta", actionBeta);

    initQuat_ = quaternion_t::Identity();
    worldToInit_.setIdentity();

    TBAI_LOG_INFO(logger_, "{} initialized successfully (actionBeta={:.2f}, maxTimestep={})", controllerName_,
                  actionBeta_, maxTimestep_);
}

G1SpinkickController::~G1SpinkickController() {
    TBAI_LOG_INFO(logger_, "Destroying {}", controllerName_);
    for (auto name : inputNames_) {
        free(const_cast<char *>(name));
    }
    for (auto name : outputNames_) {
        free(const_cast<char *>(name));
    }
}

void G1SpinkickController::initOnnxRuntime(const std::string &policyPath) {
    TBAI_LOG_INFO(logger_, "Loading ONNX policy from: {}", policyPath);

    ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "G1SpinkickController");

    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, policyPath.c_str(), sessionOptions);
    memoryInfo_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    // Get input names
    Ort::AllocatorWithDefaultOptions allocator;
    size_t numInputs = ortSession_->GetInputCount();
    TBAI_LOG_INFO(logger_, "Model has {} inputs", numInputs);

    for (size_t i = 0; i < numInputs; ++i) {
        auto inputNamePtr = ortSession_->GetInputNameAllocated(i, allocator);
        inputNames_.push_back(strdup(inputNamePtr.get()));
        TBAI_LOG_INFO(logger_, "  Input {}: {}", i, inputNames_.back());
    }

    // Get output names
    size_t numOutputs = ortSession_->GetOutputCount();
    TBAI_LOG_INFO(logger_, "Model has {} outputs", numOutputs);

    for (size_t i = 0; i < numOutputs; ++i) {
        auto outputNamePtr = ortSession_->GetOutputNameAllocated(i, allocator);
        outputNames_.push_back(strdup(outputNamePtr.get()));
        TBAI_LOG_INFO(logger_, "  Output {}: {}", i, outputNames_.back());
    }
}

void G1SpinkickController::parseModelMetadata() {
    TBAI_LOG_INFO(logger_, "Parsing model metadata...");

    try {
        Ort::AllocatorWithDefaultOptions allocator;
        auto metadata = ortSession_->GetModelMetadata();

        auto parseFloats = [](const std::string &str) {
            std::vector<double> result;
            std::stringstream ss(str);
            std::string token;
            while (std::getline(ss, token, ',')) {
                result.push_back(std::stod(token));
            }
            return result;
        };

        // Try to get custom metadata
        bool hasMetadata = false;

        auto parseStrings = [](const std::string &str) {
            std::vector<std::string> result;
            std::stringstream ss(str);
            std::string token;
            while (std::getline(ss, token, ',')) {
                result.push_back(token);
            }
            return result;
        };

        std::vector<std::string> metaKeys = {"default_joint_pos", "joint_stiffness", "joint_damping", "action_scale"};

        for (const auto &key : metaKeys) {
            try {
                auto valuePtr = metadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator);
                if (valuePtr) {
                    hasMetadata = true;
                    std::string value = valuePtr.get();

                    if (key == "default_joint_pos") {
                        auto values = parseFloats(value);
                        for (size_t i = 0; i < std::min(values.size(), static_cast<size_t>(G1_NUM_JOINTS)); ++i) {
                            defaultJointPos_[i] = values[i];
                        }
                        TBAI_LOG_INFO(logger_, "Loaded default_joint_pos from metadata ({} values)", values.size());
                    } else if (key == "joint_stiffness") {
                        auto values = parseFloats(value);
                        for (size_t i = 0; i < std::min(values.size(), static_cast<size_t>(G1_NUM_JOINTS)); ++i) {
                            stiffness_[i] = values[i];
                        }
                        TBAI_LOG_INFO(logger_, "Loaded stiffness from metadata ({} values)", values.size());
                    } else if (key == "joint_damping") {
                        auto values = parseFloats(value);
                        for (size_t i = 0; i < std::min(values.size(), static_cast<size_t>(G1_NUM_JOINTS)); ++i) {
                            damping_[i] = values[i];
                        }
                        TBAI_LOG_INFO(logger_, "Loaded damping from metadata ({} values)", values.size());
                    } else if (key == "action_scale") {
                        auto values = parseFloats(value);
                        for (size_t i = 0; i < std::min(values.size(), static_cast<size_t>(G1_NUM_JOINTS)); ++i) {
                            actionScale_[i] = values[i];
                        }
                        TBAI_LOG_INFO(logger_, "Loaded action_scale from metadata ({} values)", values.size());
                    }
                }
            } catch (...) {
                TBAI_LOG_ERROR(logger_, "Failed to parse {} from model metadata!", key);
            }
        }

        try {
            auto bodyNamesPtr = metadata.LookupCustomMetadataMapAllocated("body_names", allocator);
            auto anchorBodyNamePtr = metadata.LookupCustomMetadataMapAllocated("anchor_body_name", allocator);
            if (bodyNamesPtr && anchorBodyNamePtr) {
                hasMetadata = true;
                std::vector<std::string> bodyNames = parseStrings(bodyNamesPtr.get());
                std::string anchorBodyName = anchorBodyNamePtr.get();

                // Log all body names for debugging
                std::string bodyNamesStr;
                for (size_t i = 0; i < bodyNames.size(); ++i) {
                    bodyNamesStr += std::to_string(i) + ":" + bodyNames[i] + " ";
                }
                TBAI_LOG_INFO(logger_, "Body names: {}", bodyNamesStr);
                TBAI_LOG_INFO(logger_, "Looking for anchor_body_name: '{}'", anchorBodyName);

                bool found = false;
                for (size_t i = 0; i < bodyNames.size(); ++i) {
                    if (bodyNames[i] == anchorBodyName) {
                        anchorBodyIndex_ = i;
                        TBAI_LOG_INFO(logger_, "Found anchor_body_name '{}' at index {}", anchorBodyName, i);
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    TBAI_LOG_ERROR(logger_, "anchor_body_name '{}' not found in body_names!", anchorBodyName);
                }
            } else {
                TBAI_LOG_ERROR(logger_, "Missing body_names or anchor_body_name in model metadata!");
            }
        } catch (...) {
            TBAI_LOG_ERROR(logger_, "Failed to parse body_names/anchor_body_name from model metadata!");
        }

        if (!hasMetadata) {
            TBAI_LOG_WARN(logger_, "No custom metadata found in model");
        }

    } catch (const std::exception &e) {
        TBAI_LOG_WARN(logger_, "Failed to parse model metadata: {}", e.what());
    }
}

void G1SpinkickController::preStep(scalar_t currentTime, scalar_t dt) {
    state_ = stateSubscriberPtr_->getLatestState();
}

void G1SpinkickController::postStep(scalar_t currentTime, scalar_t dt) {
    if (motionActive_) {
        timestep_++;
        if (maxTimestep_ > 0 && timestep_ >= maxTimestep_) {
            TBAI_LOG_INFO(logger_, "Motion complete (reached max timestep {})", maxTimestep_);
            motionActive_ = false;
        }
    }
}

quaternion_t G1SpinkickController::computeAnchorOrientation(const quaternion_t &rootQuat,
                                                            const vector_t &jointPos) const {
    // Compute torso orientation from root quaternion and waist joints
    // Waist joints are at indices 12 (yaw), 13 (roll), 14 (pitch) in DDS order
    using aa_t = Eigen::AngleAxis<scalar_t>;
    quaternion_t torsoQuat = rootQuat * tbai::quaternion_t(aa_t(jointPos[12], vector3_t::UnitZ())) *
                             tbai::quaternion_t(aa_t(jointPos[13], vector3_t::UnitX())) *
                             tbai::quaternion_t(aa_t(jointPos[14], vector3_t::UnitY()));

    return torsoQuat;
}

Eigen::Matrix<scalar_t, 6, 1> G1SpinkickController::computeOrientationError(const quaternion_t &targetQuat,
                                                                             const quaternion_t &actualQuat) const {
    quaternion_t alignedMotion = initQuat_ * targetQuat;
    quaternion_t relQuat = actualQuat.conjugate() * alignedMotion;
    matrix3_t rot = relQuat.toRotationMatrix();

    Eigen::Matrix<scalar_t, 6, 1> data;
    data << rot(0, 0), rot(0, 1), rot(1, 0), rot(1, 1), rot(2, 0), rot(2, 1);
    return data;
}

void G1SpinkickController::buildObservation(scalar_t currentTime, scalar_t dt) {
    vector3_t rpy = state_.x.segment<3>(0);
    vector3_t baseAngVel = state_.x.segment<3>(6);
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);

    // Compute root quaternion from RPY (using OCS2 convention to match state estimator)
    quaternion_t rootQuat = tbai::ocs2rpy2quat(rpy);

    // Compute anchor orientations
    quaternion_t robotAnchorQuat = computeAnchorOrientation(rootQuat, jointPos);
    quaternion_t motionAnchorQuat = quaternion_t::Identity();

    if (motionBodyQuat_.row(anchorBodyIndex_).squaredNorm() > 0.5) {
        motionAnchorQuat = quaternion_t(motionBodyQuat_(anchorBodyIndex_, 0), motionBodyQuat_(anchorBodyIndex_, 1),
                                        motionBodyQuat_(anchorBodyIndex_, 2), motionBodyQuat_(anchorBodyIndex_, 3));
    }

    int idx = 0;

    observation_.segment(idx, G1_NUM_JOINTS) = motionJointPos_;
    idx += G1_NUM_JOINTS;
    observation_.segment(idx, G1_NUM_JOINTS) = motionJointVel_;
    idx += G1_NUM_JOINTS;

    auto oriError = computeOrientationError(motionAnchorQuat, robotAnchorQuat);
    observation_.segment<6>(idx) = oriError;
    idx += 6;

    observation_.segment<3>(idx) = baseAngVel;
    idx += 3;

    // Need to map from DDS order to policy order
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int ddsIdx = jointIdsMap_[i];
        observation_[idx + i] = jointPos[ddsIdx] - defaultJointPos_[i];
    }
    idx += G1_NUM_JOINTS;

    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int ddsIdx = jointIdsMap_[i];
        observation_[idx + i] = jointVel[ddsIdx];
    }
    idx += G1_NUM_JOINTS;

    observation_.segment(idx, G1_NUM_JOINTS) = lastAction_;
}

void G1SpinkickController::runInference() {
    std::vector<int64_t> obsShape = {1, SPINKICK_TOTAL_OBS_SIZE};
    std::vector<float> obsData(SPINKICK_TOTAL_OBS_SIZE);
    for (int i = 0; i < SPINKICK_TOTAL_OBS_SIZE; ++i) {
        obsData[i] = static_cast<float>(observation_[i]);
    }

    Ort::Value obsTensor =
        Ort::Value::CreateTensor<float>(*memoryInfo_, obsData.data(), obsData.size(), obsShape.data(), obsShape.size());

    std::vector<int64_t> timestepShape = {1, 1};
    std::vector<float> timestepData = {static_cast<float>(timestep_)};

    Ort::Value timestepTensor = Ort::Value::CreateTensor<float>(*memoryInfo_, timestepData.data(), timestepData.size(),
                                                                timestepShape.data(), timestepShape.size());

    std::vector<Ort::Value> inputTensors;
    inputTensors.push_back(std::move(obsTensor));
    inputTensors.push_back(std::move(timestepTensor));

    auto outputTensors = ortSession_->Run(Ort::RunOptions{nullptr}, inputNames_.data(), inputTensors.data(),
                                          inputTensors.size(), outputNames_.data(), outputNames_.size());

    // Extract actions
    float *actionsData = outputTensors[0].GetTensorMutableData<float>();
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        rawAction_[i] = static_cast<scalar_t>(actionsData[i]);
    }

    // Apply action smoothing
    action_ = (1.0 - actionBeta_) * lastAction_ + actionBeta_ * rawAction_;
    lastAction_ = action_;

    if (outputTensors.size() > 1) {
        // Get joint positions
        float *jointPosData = outputTensors[1].GetTensorMutableData<float>();
        for (int i = 0; i < G1_NUM_JOINTS; ++i) {
            motionJointPos_[i] = static_cast<scalar_t>(jointPosData[i]);
        }

        // Get joint velocities
        if (outputTensors.size() > 2) {
            float *jointVelData = outputTensors[2].GetTensorMutableData<float>();
            for (int i = 0; i < G1_NUM_JOINTS; ++i) {
                motionJointVel_[i] = static_cast<scalar_t>(jointVelData[i]);
            }
        }

        // Get body positions
        if (outputTensors.size() > 3) {
            float *bodyPosData = outputTensors[3].GetTensorMutableData<float>();
            for (int i = 0; i < SPINKICK_NUM_BODIES; ++i) {
                motionBodyPos_(i, 0) = static_cast<scalar_t>(bodyPosData[i * 3 + 0]);
                motionBodyPos_(i, 1) = static_cast<scalar_t>(bodyPosData[i * 3 + 1]);
                motionBodyPos_(i, 2) = static_cast<scalar_t>(bodyPosData[i * 3 + 2]);
            }
        }

        // Get body quaternions
        if (outputTensors.size() > 4) {
            float *bodyQuatData = outputTensors[4].GetTensorMutableData<float>();
            for (int i = 0; i < SPINKICK_NUM_BODIES; ++i) {
                motionBodyQuat_(i, 0) = static_cast<scalar_t>(bodyQuatData[i * 4 + 0]);
                motionBodyQuat_(i, 1) = static_cast<scalar_t>(bodyQuatData[i * 4 + 1]);
                motionBodyQuat_(i, 2) = static_cast<scalar_t>(bodyQuatData[i * 4 + 2]);
                motionBodyQuat_(i, 3) = static_cast<scalar_t>(bodyQuatData[i * 4 + 3]);
            }
        }
    }
}

std::vector<tbai::MotorCommand> G1SpinkickController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    buildObservation(currentTime, dt);
    runInference();

    std::vector<tbai::MotorCommand> commands;
    commands.reserve(G1_NUM_JOINTS);

    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        tbai::MotorCommand cmd;
        int ddsIdx = jointIdsMap_[i];
        cmd.joint_name = jointNames_[ddsIdx];
        cmd.desired_position = actionScale_[i] * action_[i] + defaultJointPos_[i];
        cmd.desired_velocity = 0.0;
        cmd.kp = stiffness_[i];
        cmd.kd = damping_[i];
        cmd.torque_ff = 0.0;
        commands.push_back(cmd);
    }

    return commands;
}

bool G1SpinkickController::isSupported(const std::string &controllerType) {
    return controllerType == controllerName_ || controllerType == "G1SpinkickController" ||
           controllerType == "G1Spinkick" || controllerType == "g1_spinkick" || controllerType == "spinkick";
}

void G1SpinkickController::stopController() {
    motionActive_ = false;
    TBAI_LOG_INFO(logger_, "{} stopped", controllerName_);
}

bool G1SpinkickController::ok() const {
    return true;
}

bool G1SpinkickController::checkStability() const {
    constexpr scalar_t maxAngle = 1.0;  // radians
    scalar_t roll = std::abs(state_.x[0]);
    scalar_t pitch = std::abs(state_.x[1]);

    if (roll > maxAngle || pitch > maxAngle) {
        TBAI_LOG_WARN(logger_, "Instability detected: roll={:.2f}, pitch={:.2f}", roll, pitch);
        return false;
    }
    return true;
}

bool G1SpinkickController::isMotionComplete() const {
    return maxTimestep_ > 0 && timestep_ >= maxTimestep_;
}

void G1SpinkickController::changeController(const std::string &controllerType, scalar_t currentTime) {
    TBAI_LOG_INFO(logger_, "Changing to {}", controllerName_);

    // Reset action
    lastAction_.setZero();
    action_.setZero();
    rawAction_.setZero();

    // Reset timestep
    timestep_ = 0;
    motionActive_ = true;

    // Get current state
    state_ = stateSubscriberPtr_->getLatestState();
    vector3_t rpy = state_.x.segment<3>(0);
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);

    // Get robot's current anchor orientation (torso, using OCS2 RPY convention)
    quaternion_t rootQuat = tbai::ocs2rpy2quat(rpy);
    quaternion_t robotAnchorQuat = computeAnchorOrientation(rootQuat, jointPos);

    // Run initial inference at timestep 0 to get motion's initial anchor
    observation_.setZero();
    runInference();

    // Get motion's initial anchor orientation
    quaternion_t motionInitAnchorQuat = quaternion_t::Identity();
    if (motionBodyQuat_.row(anchorBodyIndex_).squaredNorm() > 0.5) {
        motionInitAnchorQuat = quaternion_t(motionBodyQuat_(anchorBodyIndex_, 0), motionBodyQuat_(anchorBodyIndex_, 1),
                                            motionBodyQuat_(anchorBodyIndex_, 2), motionBodyQuat_(anchorBodyIndex_, 3));
    }

    quaternion_t robotYaw = MotionLoader::yawQuaternion(robotAnchorQuat);
    quaternion_t motionYaw = MotionLoader::yawQuaternion(motionInitAnchorQuat);

    initQuat_ = robotYaw * motionYaw.conjugate();

    TBAI_LOG_DEBUG(logger_, "Anchor body index: {}", anchorBodyIndex_);
    TBAI_LOG_DEBUG(logger_, "Robot anchor quat (full): [{:.3f},{:.3f},{:.3f},{:.3f}]", robotAnchorQuat.w(),
                   robotAnchorQuat.x(), robotAnchorQuat.y(), robotAnchorQuat.z());
    TBAI_LOG_DEBUG(logger_, "Motion anchor quat (full): [{:.3f},{:.3f},{:.3f},{:.3f}]", motionInitAnchorQuat.w(),
                   motionInitAnchorQuat.x(), motionInitAnchorQuat.y(), motionInitAnchorQuat.z());
    TBAI_LOG_DEBUG(logger_, "Robot yaw quat: [{:.3f},{:.3f},{:.3f},{:.3f}]", robotYaw.w(), robotYaw.x(), robotYaw.y(),
                   robotYaw.z());
    TBAI_LOG_DEBUG(logger_, "Motion yaw quat: [{:.3f},{:.3f},{:.3f},{:.3f}]", motionYaw.w(), motionYaw.x(),
                   motionYaw.y(), motionYaw.z());
    TBAI_LOG_DEBUG(logger_, "Init quat (alignment): [{:.3f},{:.3f},{:.3f},{:.3f}]", initQuat_.w(), initQuat_.x(),
                   initQuat_.y(), initQuat_.z());
    TBAI_LOG_DEBUG(logger_, "Motion playback started (timestep=0)");
}

}  // namespace g1
}  // namespace tbai
