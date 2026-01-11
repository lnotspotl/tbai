#include "tbai_mpc/quadruped_arm_wbc/WbcBase.hpp"

#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/fwd.hpp>
#include <tbai_mpc/quadruped_arm_mpc/core/MotionPhaseDefinition.h>
#include <tbai_mpc/quadruped_arm_mpc/core/Rotations.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedCom.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

// pinocchio
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tbai {
namespace mpc {
/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
WbcBase::WbcBase(const std::string &configFile, const std::string &urdfString,
                 const tbai::mpc::quadruped_arm::ComModelBase<scalar_t> &comModel,
                 const tbai::mpc::quadruped_arm::KinematicsModelBase<scalar_t> &kinematics, const std::string &configPrefix)
    : pinocchioInterfaceMeasured_(tbai::mpc::quadruped_arm::createQuadrupedPinocchioInterfaceFromUrdfString(urdfString)),
      comModelPtr_(comModel.clone()),
      kinematicsPtr_(kinematics.clone()) {
    // Base angular + linear velocity, joint velocities (12 leg + 6 arm = 18)
    nGeneralizedCoordinates_ = 6 + tbai::mpc::quadruped_arm::JOINT_COORDINATE_SIZE;  // 6 + 18 = 24

    // Base angular + linear acceleration, joint accelerations, contact forces
    nDecisionVariables_ = nGeneralizedCoordinates_ + 3 * tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS;  // 24 + 12 = 36

    qMeasured_ = vector_t(nGeneralizedCoordinates_ + 1);  // quaternion (+1 for quat vs euler)
    vMeasured_ = vector_t(nGeneralizedCoordinates_);

    footNames_ = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    // Arm joint names (Spot arm)
    armJointNames_ = {"arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1"};

    // All joint names (legs + arm)
    jointNames_ = {"front_left_hip_x",  "front_left_hip_y",  "front_left_knee",
                   "front_right_hip_x", "front_right_hip_y", "front_right_knee",
                   "rear_left_hip_x",   "rear_left_hip_y",   "rear_left_knee",
                   "rear_right_hip_x",  "rear_right_hip_y",  "rear_right_knee",
                   "arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1"};

    Jcontact_ = matrix_t(3 * tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS, nGeneralizedCoordinates_);
    dJcontactdt_ = matrix_t(3 * tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS, nGeneralizedCoordinates_);

    // Arm end-effector Jacobians (6x24: 6 for SE(3), 24 generalized coordinates)
    Jarm_ = matrix_t(6, nGeneralizedCoordinates_);
    dJarmdt_ = matrix_t(6, nGeneralizedCoordinates_);

    // Get arm end-effector frame ID
    const auto &model = pinocchioInterfaceMeasured_.getModel();
    armEEFrameId_ = model.getBodyId("arm_ee");

    loadSettings(configFile, configPrefix);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createDynamicsTask() {
    auto &data = pinocchioInterfaceMeasured_.getData();

    // Get floating base mass matrix
    const matrix_t Mb = data.M.block(0, 0, 6, nGeneralizedCoordinates_);

    // Nonlinear effects
    const vector_t h = data.nle.head<6>();

    // // Contact jacobians affecting the base
    const matrix_t Jcontact_base = Jcontact_.block(0, 0, tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS * 3, 6);

    matrix_t A(6, nDecisionVariables_);
    A << Mb, -Jcontact_base.transpose();

    vector_t b = -h;

    return Task(std::move(A), std::move(b), matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createContactForceTask() {
    // Create equality constraint matrix and vector
    const size_t Arows = 3 * (tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS - nContacts_);
    matrix_t A = matrix_t::Zero(Arows, nDecisionVariables_);
    vector_t b = vector_t::Zero(Arows);

    // Create inequality constraint matrix and vector
    const size_t Drows = 4 * nContacts_;
    matrix_t D = matrix_t::Zero(Drows, nDecisionVariables_);
    vector_t f = vector_t::Zero(Drows);

    size_t Ai = 0;
    size_t Di = 0;

    // If foot is in contact, apply friction cone constraints
    // If foot is NOT in contact, the contact force is zero
    for (size_t i = 0; i < tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS; ++i) {
        if (contactFlags_[i]) {
            D.block<4, 3>(4 * Di, nGeneralizedCoordinates_ + 3 * i) = muMatrix_;
            ++Di;
        } else {
            A.block<3, 3>(3 * Ai, nGeneralizedCoordinates_ + 3 * i) = matrix_t::Identity(3, 3);
            ++Ai;
        }
    }

    return Task(A, b, D, f);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createStanceFootNoMotionTask() {
    const size_t Arows = 3 * nContacts_;
    matrix_t A = matrix_t::Zero(Arows, nDecisionVariables_);
    vector_t b = vector_t::Zero(Arows);

    size_t Ai = 0;

    // dpdt = Jcontact * dqdt -> ddpdtdt = dJcontact * dqdt + Jcontact * dqddt = 0
    for (size_t i = 0; i < tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS; ++i) {
        if (contactFlags_[i]) {
            A.block(3 * Ai, 0, 3, nGeneralizedCoordinates_) = Jcontact_.block(3 * i, 0, 3, nGeneralizedCoordinates_);
            b.segment<3>(3 * Ai) = -dJcontactdt_.block(3 * i, 0, 3, nGeneralizedCoordinates_) * vMeasured_;
            ++Ai;
        }
    }

    return Task(A, b, matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createTorqueLimitTask() {
    auto &data = pinocchioInterfaceMeasured_.getData();

    // torques = Mj * udot + hj - JjT * Fext;
    // Only for leg joints (12 joints), not arm joints
    constexpr size_t numLegJoints = tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE;  // 12

    // Leg joint rows in mass matrix (rows 6:18, all columns)
    const matrix_t Mj = data.M.block(6, 0, numLegJoints, nGeneralizedCoordinates_);
    const vector_t hj = data.nle.segment(6, numLegJoints);

    // Contact Jacobian transpose for leg joints only (cols 6:18)
    const matrix_t JjT = Jcontact_.block(0, 6, 3 * tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS, numLegJoints).transpose();

    const size_t Drows = 2 * numLegJoints;
    matrix_t D = matrix_t::Zero(Drows, nDecisionVariables_);

    // upper bound: Mj * qddot - JjT * F <= tau_max - hj
    D.block(0, 0, numLegJoints, nGeneralizedCoordinates_) = Mj;
    D.block(0, nGeneralizedCoordinates_, numLegJoints, 3 * tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS) = -JjT;

    // lower bound: -Mj * qddot + JjT * F <= tau_max + hj
    D.block(numLegJoints, 0, numLegJoints, nGeneralizedCoordinates_) = -Mj;
    D.block(numLegJoints, nGeneralizedCoordinates_, numLegJoints, 3 * tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS) = JjT;

    vector_t f = vector_t::Ones(Drows);

    // upper bound
    f.head(numLegJoints) *= torqueLimit_;
    f.head(numLegJoints) -= hj;

    // lower bound
    f.tail(numLegJoints) *= torqueLimit_;
    f.tail(numLegJoints) += hj;

    return Task(matrix_t(), vector_t(), D, f);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createBaseAccelerationTask(const vector_t &stateCurrent, const vector_t &stateDesired,
                                         const vector_t &inputDesired, const vector_t &desiredJointAcceleration) {
    const vector_t &basePose = stateDesired.head<6>();
    const vector_t &baseVelocity = stateDesired.segment<6>(6);
    const vector_t &jointPositions = stateDesired.tail<tbai::mpc::quadruped_arm::JOINT_COORDINATE_SIZE>();
    const vector_t &jointVelocities = inputDesired.tail<tbai::mpc::quadruped_arm::JOINT_COORDINATE_SIZE>();
    const vector_t &jointAccelerations = desiredJointAcceleration;

    // forcesOnBaseInBaseFrame = [torque (3); force (3)]
    vector_t forcesOnBaseInBaseFrame = vector_t::Zero(6);
    for (size_t i = 0; i < 4; ++i) {
        // force at foot expressed in base frame
        const vector3_t &forceAtFoot = inputDesired.segment<3>(3 * i);

        // base force
        forcesOnBaseInBaseFrame.tail<3>() += forceAtFoot;

        // base torque
        vector3_t footPosition = kinematicsPtr_->positionBaseToFootInBaseFrame(i, jointPositions);
        forcesOnBaseInBaseFrame.head<3>() += footPosition.cross(forceAtFoot);
    }

    vector_t baseAccelerationLocal = comModelPtr_->calculateBaseLocalAccelerations(
        basePose, baseVelocity, jointPositions, jointVelocities, jointAccelerations, forcesOnBaseInBaseFrame);

    /* Base position error */
    auto positionError = rDesired_.transpose() * basePose.tail<3>() - rMeasured_.transpose() * qMeasured_.head<3>();

    /* Base linear velocity error */
    auto v_error = stateDesired.segment<3>(9) - stateCurrent.segment<3>(9);

    /* Base orientation error*/
    vector3_t eulerCurrent = stateCurrent.head<3>();
    vector3_t eulerDesired = stateDesired.head<3>();
    auto eulerError = tbai::mpc::quadruped_arm::rotationErrorInLocalEulerXYZ(eulerCurrent, eulerDesired);

    /* Base angular velocity error */
    vector3_t w_current = stateCurrent.segment<3>(6);
    vector3_t w_desired = stateDesired.segment<3>(6);
    vector3_t w_error = w_desired - w_current;

    // "total" desired acceleration is expressed in the base frame - this is the FreeFlyer's property
    // See https://github.com/stack-of-tasks/pinocchio/issues/1140#issuecomment-611878250
    vector_t baseAcceleration = vector_t::Zero(6);

    // desired base linear acceleration (expressed in base frame)
    baseAcceleration.head<3>() = baseAccelerationLocal.tail<3>() + baseKp_ * positionError + baseKd_ * v_error;

    // desired base angular acceleration (expressed in base frame)
    baseAcceleration.tail<3>() = baseAccelerationLocal.head<3>() + eulerKp_ * eulerError + eulerKd_ * w_error;

    matrix_t A = matrix_t::Zero(6, nDecisionVariables_);
    A.block<6, 6>(0, 0) = matrix_t::Identity(6, 6);

    return Task(A, baseAcceleration, matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createSwingFootAccelerationTask(const vector_t &stateCurrent, const vector_t &inputCurrent,
                                              const vector_t &stateDesired, const vector_t &inputDesired) {
    const vector_t &basePoseDesired = stateDesired.head<6>();
    const vector_t &baseVelocityDesired = stateDesired.segment<6>(6);
    const vector_t &jointPositionsDesired = stateDesired.tail<tbai::mpc::quadruped_arm::JOINT_COORDINATE_SIZE>();
    const vector_t &jointVelocitiesDesired = inputDesired.tail<tbai::mpc::quadruped_arm::JOINT_COORDINATE_SIZE>();

    // Desired feet positions
    auto footPositionsDesired = kinematicsPtr_->feetPositionsInOriginFrame(basePoseDesired, jointPositionsDesired);

    // Desired feet velocities
    auto footVelocitiesDesired = kinematicsPtr_->feetVelocitiesInOriginFrame(
        basePoseDesired, baseVelocityDesired, jointPositionsDesired, jointVelocitiesDesired);

    const vector_t &basePoseCurrent = stateCurrent.head<6>();
    const vector_t &baseVelocityCurrent = stateCurrent.segment<6>(6);
    const vector_t &jointPositionsCurrent = stateCurrent.tail<tbai::mpc::quadruped_arm::JOINT_COORDINATE_SIZE>();
    const vector_t &jointVelocitiesCurrent = inputCurrent.tail<tbai::mpc::quadruped_arm::JOINT_COORDINATE_SIZE>();

    // Measured feet positions
    auto footPositionsMeasured = kinematicsPtr_->feetPositionsInOriginFrame(basePoseCurrent, jointPositionsCurrent);

    // Measured feet velocities
    auto footVelocitiesMeasured = kinematicsPtr_->feetVelocitiesInOriginFrame(
        basePoseCurrent, baseVelocityCurrent, jointPositionsCurrent, jointVelocitiesCurrent);

    const size_t Arows = 3 * (stanceAsConstraint_ ? tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS - nContacts_
                                                  : tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS);
    matrix_t A = matrix_t::Zero(Arows, nDecisionVariables_);
    vector_t b = vector_t::Zero(Arows);

    size_t j = 0;
    for (size_t i = 0; i < tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS; ++i) {
        if (!contactFlags_[i] || !stanceAsConstraint_) {
            const vector_t &posDesired = footPositionsDesired[i];
            const vector_t &velDesired = footVelocitiesDesired[i];
            const vector_t &posMeasured = footPositionsMeasured[i];
            const vector_t &velMeasured = footVelocitiesMeasured[i];
            const vector_t footAcceleration =
                swingKp_ * (posDesired - posMeasured) + swingKd_ * (velDesired - velMeasured);
            A.block(3 * j, 0, 3, nGeneralizedCoordinates_) = Jcontact_.block(3 * i, 0, 3, nGeneralizedCoordinates_);
            b.segment(3 * j, 3) =
                footAcceleration - dJcontactdt_.block(3 * i, 0, 3, nGeneralizedCoordinates_) * vMeasured_;
            ++j;
        }
    }

    return Task(A, b, matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createContactForceMinimizationTask(const vector_t &inputDesired) {
    const size_t Arows = 3 * nContacts_;

    matrix_t A = matrix_t::Zero(Arows, nDecisionVariables_);
    vector_t b = vector_t::Zero(Arows);

    size_t Ai = 0;
    for (size_t i = 0; i < tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS; ++i) {
        if (contactFlags_[i]) {
            A.block<3, 3>(3 * Ai, nGeneralizedCoordinates_ + 3 * i) = matrix_t::Identity(3, 3);
            ++Ai;
        }
    }

    return Task(A, b, matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::updateMeasuredState(const vector_t &stateMeasured, const vector_t &inputMeasured) {
    constexpr size_t numLegJoints = tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE;  // 12
    constexpr size_t numArmJoints = tbai::mpc::quadruped_arm::NUM_ARM_JOINTS;             // 6

    // base position expressed in world frame
    qMeasured_.head<3>() = stateMeasured.segment<3>(3);

    // base orientation quaternion - base to world
    const vector3_t &eulerXYZ = stateMeasured.head<3>();
    qMeasured_.segment<4>(3) = tbai::mpc::quadruped_arm::quaternionBaseToOrigin(eulerXYZ).coeffs();

    // joint angles: LF, RF, LH, RH (12 leg joints)
    qMeasured_.segment(7, numLegJoints) = stateMeasured.segment(12, numLegJoints);

    // flip lh and rf (indices adjusted for new indexing)
    std::swap(qMeasured_[10], qMeasured_[13]);
    std::swap(qMeasured_[11], qMeasured_[14]);
    std::swap(qMeasured_[12], qMeasured_[15]);

    // arm joint angles (6 arm joints)
    qMeasured_.tail(numArmJoints) = stateMeasured.tail(numArmJoints);

    // Rotation matrix: base -> world
    rMeasured_ = tbai::mpc::quadruped_arm::rotationMatrixBaseToOrigin(eulerXYZ);

    // base linear velocity - expressed in base frame
    vMeasured_.head<3>() = stateMeasured.segment<3>(9);

    // base angular velocity - expressed in base frame
    vMeasured_.segment<3>(3) = stateMeasured.segment<3>(6);

    // joint velocities: LF, RF, LH, RH (12 leg joints)
    vMeasured_.segment(6, numLegJoints) = inputMeasured.segment(12, numLegJoints);
    std::swap(vMeasured_[9], vMeasured_[12]);
    std::swap(vMeasured_[10], vMeasured_[13]);
    std::swap(vMeasured_[11], vMeasured_[14]);

    // arm joint velocities (6 arm joints)
    vMeasured_.tail(numArmJoints) = inputMeasured.tail(numArmJoints);

    updateKinematicsAndDynamicsCurrent();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::updateKinematicsAndDynamicsCurrent() {
    const auto &model = pinocchioInterfaceMeasured_.getModel();
    auto &data = pinocchioInterfaceMeasured_.getData();

    // Kinematics and joint jacobians
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);

    // Mass matrix
    pinocchio::crba(model, data, qMeasured_);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Nonlinear effects
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);

    // Contact jacobians (for feet)
    matrix_t jac_temp;
    for (size_t i = 0; i < tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS; ++i) {
        size_t frameIndex = model.getBodyId(footNames_[i]);
        jac_temp.setZero(6, nGeneralizedCoordinates_);
        pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL_WORLD_ALIGNED, jac_temp);
        Jcontact_.block(3 * i, 0, 3, nGeneralizedCoordinates_) = jac_temp.template topRows<3>();
    }

    // Contact jacobian time derivatives
    matrix_t djac_temp;
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
    for (size_t i = 0; i < tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS; ++i) {
        size_t frameIndex = model.getBodyId(footNames_[i]);
        djac_temp.setZero(6, nGeneralizedCoordinates_);
        pinocchio::getFrameJacobianTimeVariation(model, data, frameIndex, pinocchio::LOCAL_WORLD_ALIGNED, djac_temp);
        dJcontactdt_.block(3 * i, 0, 3, nGeneralizedCoordinates_) = djac_temp.template topRows<3>();
    }

    // Arm end-effector Jacobian (6x24: position + orientation)
    Jarm_.setZero();
    pinocchio::getFrameJacobian(model, data, armEEFrameId_, pinocchio::LOCAL_WORLD_ALIGNED, Jarm_);

    // Arm end-effector Jacobian time derivative
    dJarmdt_.setZero();
    pinocchio::getFrameJacobianTimeVariation(model, data, armEEFrameId_, pinocchio::LOCAL_WORLD_ALIGNED, dJarmdt_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::updateDesiredState(const vector_t &stateDesired, const vector_t &inputDesired) {
    updateKinematicsAndDynamicsDesired(stateDesired, inputDesired);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::updateKinematicsAndDynamicsDesired(const vector_t &stateDesired, const vector_t &inputDesired) {
    const vector3_t &eulerXYZ = stateDesired.head<3>();
    rDesired_ = tbai::mpc::quadruped_arm::rotationMatrixBaseToOrigin(eulerXYZ);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::updateContactFlags(const size_t modeCurrent, const size_t modeDesired) {
    // Desired contact flags
    contactFlags_ = tbai::mpc::quadruped_arm::modeNumber2StanceLeg(modeDesired);
    nContacts_ = std::accumulate(contactFlags_.begin(), contactFlags_.end(), 0);
}
/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
// Arm End-Effector Tasks
/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createArmEndEffectorPositionTask(const vector_t &desiredEEPosition) {
    // Get current arm EE position from pinocchio data
    const auto &data = pinocchioInterfaceMeasured_.getData();
    const vector3_t measuredEEPosition = data.oMf[armEEFrameId_].translation();

    // Get current arm EE velocity
    const vector3_t measuredEEVelocity = Jarm_.template topRows<3>() * vMeasured_;

    // Position error
    const vector3_t positionError = desiredEEPosition - measuredEEPosition;

    // Desired acceleration: a_des = Kp * (p_des - p_meas) - Kd * v_meas
    const vector3_t desiredAcceleration = armEEPositionKp_ * positionError - armEEPositionKd_ * measuredEEVelocity;

    // Task: J_pos * qddot = a_des - dJ/dt * qdot
    const vector3_t b = desiredAcceleration - dJarmdt_.template topRows<3>() * vMeasured_;

    matrix_t A = matrix_t::Zero(3, nDecisionVariables_);
    A.block(0, 0, 3, nGeneralizedCoordinates_) = Jarm_.template topRows<3>();

    return Task(A, b, matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createArmEndEffectorOrientationTask(const vector_t &desiredEEOrientation) {
    // desiredEEOrientation is quaternion (w, x, y, z)
    const auto &data = pinocchioInterfaceMeasured_.getData();

    // Get current arm EE orientation as rotation matrix
    const matrix3_t measuredEERotation = data.oMf[armEEFrameId_].rotation();

    // Convert desired quaternion to rotation matrix
    Eigen::Quaternion<scalar_t> quatDesired(desiredEEOrientation(0), desiredEEOrientation(1), desiredEEOrientation(2),
                                            desiredEEOrientation(3));
    const matrix3_t desiredEERotation = quatDesired.toRotationMatrix();

    // Compute orientation error using axis-angle representation
    // R_error = R_desired * R_measured^T
    const matrix3_t R_error = desiredEERotation * measuredEERotation.transpose();

    // Extract axis-angle error from rotation matrix
    // For small rotations: axis_angle ≈ 0.5 * (R - R^T) as skew-symmetric
    vector3_t orientationError;
    orientationError(0) = 0.5 * (R_error(2, 1) - R_error(1, 2));
    orientationError(1) = 0.5 * (R_error(0, 2) - R_error(2, 0));
    orientationError(2) = 0.5 * (R_error(1, 0) - R_error(0, 1));

    // Get current arm EE angular velocity
    const vector3_t measuredEEAngularVelocity = Jarm_.template bottomRows<3>() * vMeasured_;

    // Desired angular acceleration: alpha_des = Kp * orientation_error - Kd * omega_meas
    const vector3_t desiredAngularAcceleration =
        armEEOrientationKp_ * orientationError - armEEOrientationKd_ * measuredEEAngularVelocity;

    // Task: J_ang * qddot = alpha_des - dJ/dt * qdot
    const vector3_t b = desiredAngularAcceleration - dJarmdt_.template bottomRows<3>() * vMeasured_;

    matrix_t A = matrix_t::Zero(3, nDecisionVariables_);
    A.block(0, 0, 3, nGeneralizedCoordinates_) = Jarm_.template bottomRows<3>();

    return Task(A, b, matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createArmJointCenteringTask() {
    // Get current arm joint positions and velocities
    constexpr size_t numArmJoints = tbai::mpc::quadruped_arm::NUM_ARM_JOINTS;  // 6
    constexpr size_t armStartIdx = 6 + tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE;  // 18

    // Extract arm joint positions from qMeasured_ (skip 7 for quaternion base)
    const auto armJointPositions = qMeasured_.tail(numArmJoints);

    // Extract arm joint velocities from vMeasured_
    const auto armJointVelocities = vMeasured_.tail(numArmJoints);

    // Joint position error (drive to home position)
    const auto jointError = armJointHomePosition_ - armJointPositions;

    // Desired acceleration: a_des = Kp * (q_home - q_meas) - Kd * dq_meas
    const auto desiredAcceleration = armJointCenteringKp_ * jointError - armJointCenteringKd_ * armJointVelocities;

    // Task: I * qddot_arm = a_des
    // This only affects arm joint accelerations (last 6 DOF of decision variables)
    matrix_t A = matrix_t::Zero(numArmJoints, nDecisionVariables_);
    A.block(0, armStartIdx, numArmJoints, numArmJoints) = matrix_t::Identity(numArmJoints, numArmJoints);

    return Task(A, desiredAcceleration, matrix_t(), vector_t());
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createArmTorqueLimitTask() {
    auto &data = pinocchioInterfaceMeasured_.getData();

    constexpr size_t numArmJoints = tbai::mpc::quadruped_arm::NUM_ARM_JOINTS;  // 6
    constexpr size_t armStartIdx = 6 + tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE;  // 18 (in generalized coords)

    // torques_arm = M_arm * qddot + h_arm
    // Arm joints are in rows 18:24 of the M matrix
    const matrix_t M_arm = data.M.block(armStartIdx, 0, numArmJoints, nGeneralizedCoordinates_);
    const vector_t h_arm = data.nle.segment(armStartIdx, numArmJoints);

    const size_t Drows = 2 * numArmJoints;
    matrix_t D = matrix_t::Zero(Drows, nDecisionVariables_);

    // upper bound: M_arm * qddot <= tau_max - h_arm
    D.block(0, 0, numArmJoints, nGeneralizedCoordinates_) = M_arm;

    // lower bound: -M_arm * qddot <= tau_max + h_arm
    D.block(numArmJoints, 0, numArmJoints, nGeneralizedCoordinates_) = -M_arm;

    vector_t f(Drows);

    // upper bound
    f.head(numArmJoints) = armTorqueLimits_ - h_arm;

    // lower bound
    f.tail(numArmJoints) = armTorqueLimits_ + h_arm;

    return Task(matrix_t(), vector_t(), D, f);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::generateFrictionConeMatrix(const scalar_t mu) {
    muMatrix_ = matrix_t(4, 3);
    muMatrix_ << 1, 0, -mu,  // clang-format off
                -1,  0, -mu,
                 0,  1, -mu,
                 0, -1, -mu;  // clang-format on
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/

void WbcBase::loadSettings(const std::string &configFile, const std::string &configPrefix) {
    using ocs2::scalar_t;
    using ocs2::loadData::loadCppDataType;

    // friction coefficient
    scalar_t mu;
    loadCppDataType<scalar_t>(configFile, configPrefix + "frictionCoefficient", mu);
    generateFrictionConeMatrix(mu);

    // swing kp and kd
    loadCppDataType<scalar_t>(configFile, configPrefix + "swingKp", swingKp_);
    loadCppDataType<scalar_t>(configFile, configPrefix + "swingKd", swingKd_);

    // base kp and kd
    loadCppDataType<scalar_t>(configFile, configPrefix + "baseKp", baseKp_);
    loadCppDataType<scalar_t>(configFile, configPrefix + "baseKd", baseKd_);

    // euler kp and kd
    loadCppDataType<scalar_t>(configFile, configPrefix + "eulerKp", eulerKp_);
    loadCppDataType<scalar_t>(configFile, configPrefix + "eulerKd", eulerKd_);

    // torque limit
    loadCppDataType<scalar_t>(configFile, configPrefix + "torqueLimit", torqueLimit_);

    // stance
    loadCppDataType<bool>(configFile, configPrefix + "stanceAsConstraint", stanceAsConstraint_);

    // =============================================
    // Arm-specific settings
    // =============================================

    // Arm end-effector position PD gains (default values)
    armEEPositionKp_ = 100.0;
    armEEPositionKd_ = 20.0;
    loadCppDataType<scalar_t>(configFile, configPrefix + "armEEPositionKp", armEEPositionKp_);
    loadCppDataType<scalar_t>(configFile, configPrefix + "armEEPositionKd", armEEPositionKd_);

    // Arm end-effector orientation PD gains
    armEEOrientationKp_ = 50.0;
    armEEOrientationKd_ = 10.0;
    loadCppDataType<scalar_t>(configFile, configPrefix + "armEEOrientationKp", armEEOrientationKp_);
    loadCppDataType<scalar_t>(configFile, configPrefix + "armEEOrientationKd", armEEOrientationKd_);

    // Arm joint centering (nullspace) PD gains
    armJointCenteringKp_ = 10.0;
    armJointCenteringKd_ = 2.0;
    loadCppDataType<scalar_t>(configFile, configPrefix + "armJointCenteringKp", armJointCenteringKp_);
    loadCppDataType<scalar_t>(configFile, configPrefix + "armJointCenteringKd", armJointCenteringKd_);

    // Arm joint PD gains for motor commands
    armJointKp_ = 200.0;
    armJointKd_ = 10.0;
    loadCppDataType<scalar_t>(configFile, configPrefix + "armJointKp", armJointKp_);
    loadCppDataType<scalar_t>(configFile, configPrefix + "armJointKd", armJointKd_);

    // Arm home position (stowed position for spot arm)
    armJointHomePosition_ << 0.0, -3.1, 2.1, 0.0, 0.0, 0.0;

    // Try to load from config (if available)
    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_info(configFile, pt);
        auto homePos = pt.get_child_optional(configPrefix + "armJointHomePosition");
        if (homePos) {
            size_t i = 0;
            for (const auto &val : *homePos) {
                if (i < 6) {
                    armJointHomePosition_(i) = val.second.get_value<scalar_t>();
                }
                ++i;
            }
        }
    } catch (...) {
        // Use default home position
    }

    // Arm torque limits (default values for spot arm)
    armTorqueLimits_ << 45.0, 45.0, 30.0, 30.0, 15.0, 15.0;

    // Try to load from config
    try {
        auto torqueLimits = pt.get_child_optional(configPrefix + "armTorqueLimits");
        if (torqueLimits) {
            size_t i = 0;
            for (const auto &val : *torqueLimits) {
                if (i < 6) {
                    armTorqueLimits_(i) = val.second.get_value<scalar_t>();
                }
                ++i;
            }
        }
    } catch (...) {
        // Use default torque limits
    }
}

}  // namespace mpc
}  // namespace tbai
