#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <onnxruntime_cxx_api.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_deploy_g1/G1Constants.hpp>

namespace tbai {
namespace g1 {

constexpr int SPINKICK_OBS_MOTION_CMD = 58;       // 29 joint_pos + 29 joint_vel
constexpr int SPINKICK_OBS_ANCHOR_ORI = 6;
constexpr int SPINKICK_OBS_BASE_ANG_VEL = 3;
constexpr int SPINKICK_OBS_JOINT_POS_REL = 29;
constexpr int SPINKICK_OBS_JOINT_VEL_REL = 29;
constexpr int SPINKICK_OBS_LAST_ACTION = 29;
constexpr int SPINKICK_TOTAL_OBS_SIZE = SPINKICK_OBS_MOTION_CMD + SPINKICK_OBS_ANCHOR_ORI + SPINKICK_OBS_BASE_ANG_VEL +
                                        SPINKICK_OBS_JOINT_POS_REL + SPINKICK_OBS_JOINT_VEL_REL + SPINKICK_OBS_LAST_ACTION;

constexpr int SPINKICK_NUM_BODIES = 14;

class G1SpinkickController : public tbai::Controller {
   public:
    G1SpinkickController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                         const std::string &policyPath, const std::string &controllerName = "G1SpinkickController",
                         bool useModelMetaConfig = true, float actionBeta = 1.0f);

    ~G1SpinkickController();

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override;

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void postStep(scalar_t currentTime, scalar_t dt) override;

    bool isSupported(const std::string &controllerType) override;

    std::string getName() const override { return controllerName_; }

    void stopController() override;

    bool ok() const override;

    scalar_t getRate() const override { return 50.0; }

    bool checkStability() const override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isMotionComplete() const;

    int getTimestep() const { return timestep_; }

    int getMaxTimestep() const { return maxTimestep_; }

   protected:
    void initOnnxRuntime(const std::string &policyPath);
    void parseModelMetadata();
    void buildObservation(scalar_t currentTime, scalar_t dt);
    void runInference();

    quaternion_t computeAnchorOrientation(const quaternion_t &rootQuat, const vector_t &jointPos) const;

    Eigen::Matrix<scalar_t, 6, 1> computeOrientationError(const quaternion_t &targetQuat,
                                                          const quaternion_t &actualQuat) const;

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    // ONNX Runtime
    std::unique_ptr<Ort::Env> ortEnv_;
    std::unique_ptr<Ort::Session> ortSession_;
    std::unique_ptr<Ort::MemoryInfo> memoryInfo_;
    std::vector<const char *> inputNames_;
    std::vector<const char *> outputNames_;

    // State
    tbai::State state_;
    vector_t lastAction_;

    vector_t observation_;

    // Action output
    vector_t action_;
    vector_t rawAction_;

    // Motion data from ONNX outputs
    vector_t motionJointPos_;
    vector_t motionJointVel_;
    Eigen::Matrix<scalar_t, SPINKICK_NUM_BODIES, 3> motionBodyPos_;
    Eigen::Matrix<scalar_t, SPINKICK_NUM_BODIES, 4> motionBodyQuat_;  // wxyz format

    // Configuration
    vector_t defaultJointPos_;
    vector_t actionScale_;
    vector_t stiffness_;
    vector_t damping_;
    std::vector<int> jointIdsMap_;

    std::vector<std::string> jointNames_;

    float actionBeta_;
    bool useModelMetaConfig_;

    int timestep_;
    int maxTimestep_;
    bool motionActive_;

    // Initial alignment for motion
    quaternion_t initQuat_;
    matrix3_t worldToInit_;

    // Anchor body index in the body array
    int anchorBodyIndex_;

    std::string controllerName_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace g1
}  // namespace tbai
