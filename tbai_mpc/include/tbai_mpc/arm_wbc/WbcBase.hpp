#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <string>
#include <vector>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_mpc/arm_mpc/ArmModelInfo.h>
#include <tbai_mpc/wbc/Task.hpp>

namespace tbai::mpc::arm {

class WbcBase {
   public:
    WbcBase(const std::string &configFile, const std::string &urdfString, const tbai::mpc::arm::ArmModelInfo &armInfo,
            const std::string &configPrefix);

    virtual ~WbcBase() = default;

    virtual std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, const vector_t &currentState,
                                                             const vector_t &currentInput, const vector_t &desiredState,
                                                             const vector_t &desiredInput,
                                                             const vector_t &desiredJointAcceleration,
                                                             const vector_t &desiredEEPosition,
                                                             const vector_t &desiredEEOrientation, bool &isStable) = 0;

   protected:
    Task createDynamicsTask();

    Task createEndEffectorPositionTask(const vector_t &desiredEEPosition);

    Task createEndEffectorOrientationTask(const vector_t &desiredEEOrientation);

    Task createJointAccelerationTask(const vector_t &desiredJointAcceleration);

    Task createJointCenteringTask();

    Task createTorqueLimitTask();

    void updateMeasuredState(const vector_t &currentState, const vector_t &currentInput);

    void updateKinematicsAndDynamics();

    /* Pinocchio interface for kinematics and dynamics */
    ocs2::PinocchioInterface pinocchioInterface_;

    /* End-effector frame ID */
    size_t eeFrameId_;

    /* Number of joints (DOF) */
    size_t nJoints_;

    /* Number of decision variables (joint accelerations) */
    size_t nDecisionVariables_;

    /* Measured joint positions */
    vector_t qMeasured_;

    /* Measured joint velocities */
    vector_t vMeasured_;

    /* End-effector Jacobian (6 x nJoints) - linear and angular */
    matrix_t Jee_;

    /* End-effector Jacobian time derivative */
    matrix_t dJeedt_;

    /* Joint names */
    std::vector<std::string> jointNames_;

    /* Arm model info */
    tbai::mpc::arm::ArmModelInfo armInfo_;

    /* PD gains for end-effector position tracking */
    scalar_t eePositionKp_;
    scalar_t eePositionKd_;

    /* PD gains for end-effector orientation tracking */
    scalar_t eeOrientationKp_;
    scalar_t eeOrientationKd_;

    /* Torque limits for each joint */
    vector_t torqueLimits_;

    /* Home/neutral joint positions for centering task */
    vector_t jointHomePosition_;

    /* PD gains for joint centering task */
    scalar_t jointCenteringKp_;
    scalar_t jointCenteringKd_;

   private:
    void loadSettings(const std::string &configFile, const std::string &configPrefix);
};

}  // namespace tbai::mpc::arm
