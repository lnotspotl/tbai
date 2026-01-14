// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_mpc/arm_wbc/WbcBase.hpp"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>

// pinocchio
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tbai::mpc::arm {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
WbcBase::WbcBase(const std::string &configFile, const std::string &urdfString,
                 const tbai::mpc::arm::ArmModelInfo &armInfo, const std::string &configPrefix)
    : pinocchioInterface_(ocs2::getPinocchioInterfaceFromUrdfString(urdfString)), armInfo_(armInfo) {
    const auto &model = pinocchioInterface_.getModel();

    // Number of joints (DOF)
    nJoints_ = armInfo_.armDim;

    // Decision variables are joint accelerations only (no contact forces for fixed-base arm)
    nDecisionVariables_ = nJoints_;

    // Initialize measured state vectors
    qMeasured_ = vector_t::Zero(nJoints_);
    vMeasured_ = vector_t::Zero(nJoints_);

    // Initialize Jacobians
    Jee_ = matrix_t::Zero(6, nJoints_);
    dJeedt_ = matrix_t::Zero(6, nJoints_);

    // Get end-effector frame ID
    eeFrameId_ = model.getFrameId(armInfo_.eeFrame);

    // Get joint names
    jointNames_ = armInfo_.dofNames;

    // Load settings
    loadSettings(configFile, configPrefix);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createDynamicsTask() {
    // For a fixed-base manipulator, the dynamics are:
    // M * qddot + h = tau
    //
    // In WBC, we solve for qddot, and compute tau afterwards.
    // The dynamics task here is an empty equality constraint (no constraint on dynamics)
    // because we will compute torques from the solution.
    //
    // However, if we want to ensure the solution is dynamically consistent,
    // we can add constraints on the resulting torques via torque limit task.

    // No explicit dynamics constraint - we just track desired accelerations
    // and compute torques from M*qddot + h = tau afterwards
    return Task(matrix_t(), vector_t(), matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createEndEffectorPositionTask(const vector_t &desiredEEPosition) {
    // End-effector position tracking task:
    // J_pos * qddot = a_desired - dJ_pos/dt * qdot
    // where a_desired = Kp * (p_des - p_meas) + Kd * (v_des - v_meas)
    //
    // Since we track position and don't have desired EE velocity, we assume v_des = 0

    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();

    // Get current EE position
    const auto &eeFrame = data.oMf[eeFrameId_];
    const vector3_t eePosActual = eeFrame.translation();

    // Compute position error
    const vector3_t posError = desiredEEPosition.head<3>() - eePosActual;

    // Get current EE velocity (J_pos * qdot)
    const vector3_t eeVelActual = Jee_.topRows<3>() * vMeasured_;

    // Desired EE acceleration with PD control
    const vector3_t eeAccDesired = eePositionKp_ * posError - eePositionKd_ * eeVelActual;

    // Task: J_pos * qddot = a_desired - dJ_pos/dt * qdot
    matrix_t A = Jee_.topRows<3>();
    vector_t b = eeAccDesired - dJeedt_.topRows<3>() * vMeasured_;

    return Task(std::move(A), std::move(b), matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createEndEffectorOrientationTask(const vector_t &desiredEEOrientation) {
    // End-effector orientation tracking task
    // Using quaternion error for orientation control

    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();

    // Get current EE orientation as rotation matrix
    const auto &eeFrame = data.oMf[eeFrameId_];
    const matrix3_t R_actual = eeFrame.rotation();

    // Desired orientation from quaternion (w, x, y, z format)
    Eigen::Quaterniond quatDesired;
    quatDesired.w() = desiredEEOrientation(0);
    quatDesired.x() = desiredEEOrientation(1);
    quatDesired.y() = desiredEEOrientation(2);
    quatDesired.z() = desiredEEOrientation(3);
    quatDesired.normalize();
    const matrix3_t R_desired = quatDesired.toRotationMatrix();

    // Compute orientation error using rotation matrix
    // Error in world frame: R_error = R_desired * R_actual^T
    const matrix3_t R_error = R_desired * R_actual.transpose();

    // Convert to axis-angle for error vector
    Eigen::AngleAxisd angleAxis(R_error);
    vector3_t orientationError = angleAxis.angle() * angleAxis.axis();

    // Get current EE angular velocity (J_ang * qdot)
    const vector3_t eeAngVelActual = Jee_.bottomRows<3>() * vMeasured_;

    // Desired EE angular acceleration with PD control
    const vector3_t eeAngAccDesired = eeOrientationKp_ * orientationError - eeOrientationKd_ * eeAngVelActual;

    // Task: J_ang * qddot = alpha_desired - dJ_ang/dt * qdot
    matrix_t A = Jee_.bottomRows<3>();
    vector_t b = eeAngAccDesired - dJeedt_.bottomRows<3>() * vMeasured_;

    return Task(std::move(A), std::move(b), matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createJointAccelerationTask(const vector_t &desiredJointAcceleration) {
    // Direct joint acceleration tracking task:
    // qddot = qddot_desired

    matrix_t A = matrix_t::Identity(nJoints_, nDecisionVariables_);
    vector_t b = desiredJointAcceleration;

    return Task(std::move(A), std::move(b), matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createJointCenteringTask() {
    // Joint centering task: drive joints towards home position
    // Using PD control: qddot = Kp * (q_home - q_measured) - Kd * v_measured
    //
    // This acts as a regularization term to keep the arm near a comfortable configuration
    // when the end-effector tasks don't fully constrain the solution (null-space behavior)

    // Compute position error from home position
    const vector_t posError = jointHomePosition_ - qMeasured_;

    // Desired joint acceleration with PD control
    const vector_t qddotDesired = jointCenteringKp_ * posError - jointCenteringKd_ * vMeasured_;

    // Task: I * qddot = qddot_desired
    matrix_t A = matrix_t::Identity(nJoints_, nDecisionVariables_);
    vector_t b = qddotDesired;

    return Task(std::move(A), std::move(b), matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createTorqueLimitTask() {
    // Torque limit task: |tau| <= tau_max
    // tau = M * qddot + h
    // => -tau_max <= M * qddot + h <= tau_max
    // => M * qddot <= tau_max - h  (upper bound)
    // => -M * qddot <= tau_max + h (lower bound)

    auto &data = pinocchioInterface_.getData();
    const matrix_t &M = data.M;
    const vector_t &h = data.nle;

    // Inequality constraints: D * x <= f
    // Upper bound: M * qddot <= tau_max - h
    // Lower bound: -M * qddot <= tau_max + h

    matrix_t D(2 * nJoints_, nDecisionVariables_);
    D.topRows(nJoints_) = M;
    D.bottomRows(nJoints_) = -M;

    vector_t f(2 * nJoints_);
    f.head(nJoints_) = torqueLimits_ - h;
    f.tail(nJoints_) = torqueLimits_ + h;

    return Task(matrix_t(), vector_t(), std::move(D), std::move(f));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::updateMeasuredState(const vector_t &currentState, const vector_t &currentInput) {
    // For fixed-base manipulator:
    // currentState = joint positions (7)
    // currentInput = joint velocities (7)

    qMeasured_ = currentState;
    vMeasured_ = currentInput;

    updateKinematicsAndDynamics();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::updateKinematicsAndDynamics() {
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();

    // Forward kinematics
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);

    // Mass matrix
    pinocchio::crba(model, data, qMeasured_);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Nonlinear effects (Coriolis + gravity)
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);

    // End-effector Jacobian
    matrix_t jac_temp = matrix_t::Zero(6, nJoints_);
    pinocchio::getFrameJacobian(model, data, eeFrameId_, pinocchio::LOCAL_WORLD_ALIGNED, jac_temp);
    Jee_ = jac_temp;

    // End-effector Jacobian time derivative
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
    matrix_t djac_temp = matrix_t::Zero(6, nJoints_);
    pinocchio::getFrameJacobianTimeVariation(model, data, eeFrameId_, pinocchio::LOCAL_WORLD_ALIGNED, djac_temp);
    dJeedt_ = djac_temp;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::loadSettings(const std::string &configFile, const std::string &configPrefix) {
    using ocs2::scalar_t;
    using ocs2::loadData::loadCppDataType;

    // End-effector position PD gains
    loadCppDataType<scalar_t>(configFile, configPrefix + "eePositionKp", eePositionKp_);
    loadCppDataType<scalar_t>(configFile, configPrefix + "eePositionKd", eePositionKd_);

    // End-effector orientation PD gains
    loadCppDataType<scalar_t>(configFile, configPrefix + "eeOrientationKp", eeOrientationKp_);
    loadCppDataType<scalar_t>(configFile, configPrefix + "eeOrientationKd", eeOrientationKd_);

    // Joint centering PD gains (with defaults)
    jointCenteringKp_ = 10.0;
    jointCenteringKd_ = 2.0;
    try {
        loadCppDataType<scalar_t>(configFile, configPrefix + "jointCenteringKp", jointCenteringKp_);
    } catch (...) {
    }
    try {
        loadCppDataType<scalar_t>(configFile, configPrefix + "jointCenteringKd", jointCenteringKd_);
    } catch (...) {
    }

    // Joint home position (default arm home configuration)
    jointHomePosition_ = vector_t(nJoints_);
    jointHomePosition_ << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;  // Default arm home

    // Try to load custom home position from config
    try {
        ocs2::loadData::loadEigenMatrix(configFile, configPrefix + "jointHomePosition", jointHomePosition_);
    } catch (...) {
        // Use default if not specified
    }

    // Torque limits
    torqueLimits_ = vector_t(nJoints_);
    scalar_t defaultTorqueLimit = 87.0;  // Default arm torque limit
    loadCppDataType<scalar_t>(configFile, configPrefix + "torqueLimit", defaultTorqueLimit);
    torqueLimits_.setConstant(defaultTorqueLimit);

    // Try to load individual joint torque limits if available
    for (size_t i = 0; i < nJoints_; ++i) {
        try {
            scalar_t jointLimit;
            loadCppDataType<scalar_t>(configFile, configPrefix + "torqueLimits.joint" + std::to_string(i), jointLimit);
            torqueLimits_(i) = jointLimit;
        } catch (...) {
            // Use default if not specified
        }
    }
}

}  // namespace tbai::mpc::arm
