#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <string>
#include <vector>

#include <tbai_core/control/Controllers.hpp>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedCom.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedKinematics.h>
#include <tbai_mpc/wbc/Task.hpp>

namespace tbai {
namespace mpc {
namespace quadruped_arm {

/**
 * Whole-Body Controller base class for quadruped with arm.
 *
 * Decision variables: [base_accel (6), leg_accel (12), arm_accel (6), contact_forces (12)]
 * Total: 36 decision variables
 *
 * Generalized coordinates: [base (7 quaternion), leg_joints (12), arm_joints (6)]
 * Total: 25 (but 24 DOF since quaternion has 4 components for 3 DOF)
 */
class WbcBase {
   public:
    WbcBase(const std::string &configFile, const std::string &urdfString,
            const tbai::mpc::quadruped_arm::ComModelBase<scalar_t> &comModel,
            const tbai::mpc::quadruped_arm::KinematicsModelBase<scalar_t> &kinematics, const std::string &configPrefix);

    virtual ~WbcBase() = default;

    /**
     * Get motor commands for all joints (legs + arm).
     *
     * @param currentTime Current time
     * @param currentState Current robot state (30D: base + leg joints + arm joints)
     * @param currentInput Current input (30D: contact forces + leg velocities + arm velocities)
     * @param currentMode Current contact mode
     * @param desiredState Desired state from MPC
     * @param desiredInput Desired input from MPC
     * @param desiredMode Desired contact mode from MPC
     * @param desiredJointAcceleration Desired joint accelerations (18D: legs + arm)
     * @param desiredArmEEPosition Desired arm end-effector position (3D)
     * @param desiredArmEEOrientation Desired arm end-effector orientation as quaternion (4D: w,x,y,z)
     * @param isStable Output stability flag
     * @return Motor commands for 18 joints (12 leg + 6 arm)
     */
    virtual std::vector<tbai::MotorCommand> getMotorCommands(
        scalar_t currentTime, const vector_t &currentState, const vector_t &currentInput, const size_t currentMode,
        const vector_t &desiredState, const vector_t &desiredInput, const size_t desiredMode,
        const vector_t &desiredJointAcceleration, const vector_t &desiredArmEEPosition,
        const vector_t &desiredArmEEOrientation, bool &isStable) = 0;

   protected:
    // =============================================
    // Quadruped Tasks (legs + base)
    // =============================================
    Task createDynamicsTask();
    Task createContactForceTask();
    Task createStanceFootNoMotionTask();
    Task createTorqueLimitTask();

    Task createBaseAccelerationTask(const vector_t &stateCurrent, const vector_t &stateDesired,
                                    const vector_t &inputDesired, const vector_t &desiredJointAcceleration);
    Task createSwingFootAccelerationTask(const vector_t &stateCurrent, const vector_t &inputCurrent,
                                         const vector_t &stateDesired, const vector_t &inputDesired);
    Task createContactForceMinimizationTask(const vector_t &inputDesired);

    // =============================================
    // Arm End-Effector Tasks
    // =============================================

    /**
     * Create arm end-effector position tracking task.
     * Uses Jacobian-based acceleration-level control:
     * J_pos * qddot = Kp*(p_des - p_meas) - Kd*v_meas - dJ/dt * qdot
     */
    Task createArmEndEffectorPositionTask(const vector_t &desiredEEPosition);

    /**
     * Create arm end-effector orientation tracking task.
     * Uses Jacobian-based acceleration-level control with axis-angle error.
     */
    Task createArmEndEffectorOrientationTask(const vector_t &desiredEEOrientation);

    /**
     * Create arm joint centering task (nullspace regularization).
     * Drives arm joints toward home position when not constrained by EE tasks.
     */
    Task createArmJointCenteringTask();

    /**
     * Create arm torque limit constraint.
     * |M_arm * qddot + h_arm| <= tau_max
     */
    Task createArmTorqueLimitTask();

    // =============================================
    // State Update Methods
    // =============================================
    void updateMeasuredState(const vector_t &stateCurrent, const vector_t &inputCurrent);
    void updateKinematicsAndDynamicsCurrent();
    void updateDesiredState(const vector_t &stateDesired, const vector_t &inputDesired);
    void updateKinematicsAndDynamicsDesired(const vector_t &stateDesired, const vector_t &inputDesired);
    void updateContactFlags(const size_t modeCurrent, const size_t modeDesired);

    // =============================================
    // Quadruped Member Variables
    // =============================================

    /* Contact jacobians - stacked on top of each other */
    matrix_t Jcontact_;

    /* Contact jacobians time derivative - stacked on top of each other */
    matrix_t dJcontactdt_;

    /* Measured generalized positions (quaternion for base) */
    vector_t qMeasured_;

    /* Measured generalized velocities */
    vector_t vMeasured_;

    /* Measured rotation matrix from base to world*/
    matrix_t rMeasured_;

    /* Desired rotation matrix from base to world*/
    matrix_t rDesired_;

    /* Number of decision variables: 6 (base) + 12 (legs) + 6 (arm) + 12 (forces) = 36 */
    size_t nDecisionVariables_;

    /* Number of generalized coordinates (DOF): 6 (base) + 12 (legs) + 6 (arm) = 24 */
    size_t nGeneralizedCoordinates_;

    /* Measured pinocchio interface */
    ocs2::PinocchioInterface pinocchioInterfaceMeasured_;

    /* Foot names*/
    std::vector<std::string> footNames_;

    /* Friction cone matrix*/
    matrix_t muMatrix_;

    /* swing kp and kd*/
    scalar_t swingKp_;
    scalar_t swingKd_;

    /* base kp and kd */
    scalar_t baseKp_;
    scalar_t baseKd_;

    /* euler kp and kd*/
    scalar_t eulerKp_;
    scalar_t eulerKd_;

    /* torque limit for leg joints */
    scalar_t torqueLimit_;

    /* Desired contact flags */
    tbai::mpc::quadruped_arm::contact_flag_t contactFlags_;
    size_t nContacts_;

    /* whether stance should be enforced as a constraint or as a cost */
    bool stanceAsConstraint_ = false;

    // =============================================
    // Arm Member Variables
    // =============================================

    /* Arm end-effector frame ID in Pinocchio model */
    size_t armEEFrameId_;

    /* Arm end-effector Jacobian (6 x nGeneralizedCoordinates_) */
    matrix_t Jarm_;

    /* Arm end-effector Jacobian time derivative */
    matrix_t dJarmdt_;

    /* Arm end-effector position PD gains */
    scalar_t armEEPositionKp_;
    scalar_t armEEPositionKd_;

    /* Arm end-effector orientation PD gains */
    scalar_t armEEOrientationKp_;
    scalar_t armEEOrientationKd_;

    /* Arm joint centering (nullspace) PD gains */
    scalar_t armJointCenteringKp_;
    scalar_t armJointCenteringKd_;

    /* Arm joint PD gains for motor commands */
    scalar_t armJointKp_;
    scalar_t armJointKd_;

    /* Arm home position for centering task */
    Eigen::Matrix<scalar_t, 6, 1> armJointHomePosition_;

    /* Arm torque limits per joint */
    Eigen::Matrix<scalar_t, 6, 1> armTorqueLimits_;

    /* Arm joint names */
    std::vector<std::string> armJointNames_;

    /* Joint names for all joints (leg + arm) */
    std::vector<std::string> jointNames_;

   private:
    void loadSettings(const std::string &configFile, const std::string &configPrefix);
    void generateFrictionConeMatrix(const scalar_t mu);

    std::unique_ptr<tbai::mpc::quadruped_arm::ComModelBase<scalar_t>> comModelPtr_;
    std::unique_ptr<tbai::mpc::quadruped_arm::KinematicsModelBase<scalar_t>> kinematicsPtr_;
};
}  // namespace quadruped_arm
}  // namespace mpc
}  // namespace tbai
