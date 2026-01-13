#include "tbai_mpc/quadruped_wbc/WbcBase.hpp"

#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/fwd.hpp>
#include <tbai_mpc/quadruped_mpc/core/MotionPhaseDefinition.h>
#include <tbai_mpc/quadruped_mpc/core/Rotations.h>
#include <tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedCom.h>

// pinocchio
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tbai {
namespace mpc {
namespace quadruped {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
WbcBase::WbcBase(const std::string &configFile, const std::string &urdfString,
                 const tbai::mpc::quadruped::ComModelBase<scalar_t> &comModel,
                 const tbai::mpc::quadruped::KinematicsModelBase<scalar_t> &kinematics, const std::string &configPrefix)
    : pinocchioInterfaceMeasured_(tbai::mpc::quadruped::createQuadrupedPinocchioInterfaceFromUrdfString(urdfString)),
      comModelPtr_(comModel.clone()),
      kinematicsPtr_(kinematics.clone()) {
    // Base angular + linear velocity, joint velocities
    nGeneralizedCoordinates_ = 6 + tbai::mpc::quadruped::JOINT_COORDINATE_SIZE;

    // Base angular + linear acceleration, joint accelerations, contact forces
    nDecisionVariables_ = nGeneralizedCoordinates_ + 3 * tbai::mpc::quadruped::NUM_CONTACT_POINTS;

    qMeasured_ = vector_t(nGeneralizedCoordinates_ + 1);  // quaternion
    vMeasured_ = vector_t(nGeneralizedCoordinates_);

    footNames_ = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    Jcontact_ = matrix_t(3 * tbai::mpc::quadruped::NUM_CONTACT_POINTS, nGeneralizedCoordinates_);
    dJcontactdt_ = matrix_t(3 * tbai::mpc::quadruped::NUM_CONTACT_POINTS, nGeneralizedCoordinates_);

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
    const matrix_t Jcontact_base = Jcontact_.block(0, 0, tbai::mpc::quadruped::NUM_CONTACT_POINTS * 3, 6);

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
    const size_t Arows = 3 * (tbai::mpc::quadruped::NUM_CONTACT_POINTS - nContacts_);
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
    for (size_t i = 0; i < tbai::mpc::quadruped::NUM_CONTACT_POINTS; ++i) {
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
    for (size_t i = 0; i < tbai::mpc::quadruped::NUM_CONTACT_POINTS; ++i) {
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
    const matrix_t &Mj = data.M.block<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE, 18>(6, 0);
    const vector_t &hj = data.nle.segment<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>(6);
    const matrix_t &JjT = Jcontact_.block(0, 6, 12, tbai::mpc::quadruped::JOINT_COORDINATE_SIZE).transpose();

    const size_t Drows = 2 * tbai::mpc::quadruped::JOINT_COORDINATE_SIZE;
    matrix_t D = matrix_t::Zero(Drows, nDecisionVariables_);

    // upper bound
    D.block(0, 0, tbai::mpc::quadruped::JOINT_COORDINATE_SIZE, nDecisionVariables_) << Mj, -JjT;

    // lower bound
    D.block(tbai::mpc::quadruped::JOINT_COORDINATE_SIZE, 0, tbai::mpc::quadruped::JOINT_COORDINATE_SIZE,
            nDecisionVariables_)
        << -Mj,
        JjT;

    vector_t f = vector_t::Ones(Drows);

    // upper bound
    f.head<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>() *= torqueLimit_;
    f.head<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>() -= hj;

    // lower bound
    f.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>() *= torqueLimit_;
    f.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>() += hj;

    return Task(matrix_t(), vector_t(), D, f);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Task WbcBase::createBaseAccelerationTask(const vector_t &stateCurrent, const vector_t &stateDesired,
                                         const vector_t &inputDesired, const vector_t &desiredJointAcceleration) {
    const vector_t &basePose = stateDesired.head<6>();
    const vector_t &baseVelocity = stateDesired.segment<6>(6);
    const vector_t &jointPositions = stateDesired.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>();
    const vector_t &jointVelocities = inputDesired.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>();
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
    auto eulerError = tbai::mpc::quadruped::rotationErrorInLocalEulerXYZ(eulerCurrent, eulerDesired);

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
    const vector_t &jointPositionsDesired = stateDesired.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>();
    const vector_t &jointVelocitiesDesired = inputDesired.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>();

    // Desired feet positions
    auto footPositionsDesired = kinematicsPtr_->feetPositionsInOriginFrame(basePoseDesired, jointPositionsDesired);

    // Desired feet velocities
    auto footVelocitiesDesired = kinematicsPtr_->feetVelocitiesInOriginFrame(
        basePoseDesired, baseVelocityDesired, jointPositionsDesired, jointVelocitiesDesired);

    const vector_t &basePoseCurrent = stateCurrent.head<6>();
    const vector_t &baseVelocityCurrent = stateCurrent.segment<6>(6);
    const vector_t &jointPositionsCurrent = stateCurrent.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>();
    const vector_t &jointVelocitiesCurrent = inputCurrent.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>();

    // Measured feet positions
    auto footPositionsMeasured = kinematicsPtr_->feetPositionsInOriginFrame(basePoseCurrent, jointPositionsCurrent);

    // Measured feet velocities
    auto footVelocitiesMeasured = kinematicsPtr_->feetVelocitiesInOriginFrame(
        basePoseCurrent, baseVelocityCurrent, jointPositionsCurrent, jointVelocitiesCurrent);

    const size_t Arows = 3 * (stanceAsConstraint_ ? tbai::mpc::quadruped::NUM_CONTACT_POINTS - nContacts_
                                                  : tbai::mpc::quadruped::NUM_CONTACT_POINTS);
    matrix_t A = matrix_t::Zero(Arows, nDecisionVariables_);
    vector_t b = vector_t::Zero(Arows);

    size_t j = 0;
    for (size_t i = 0; i < tbai::mpc::quadruped::NUM_CONTACT_POINTS; ++i) {
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
    for (size_t i = 0; i < tbai::mpc::quadruped::NUM_CONTACT_POINTS; ++i) {
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
    // base position expressedd in world frame
    qMeasured_.head<3>() = stateMeasured.segment<3>(3);

    // base orientation quaternion - base to world
    const vector3_t &eulerXYZ = stateMeasured.head<3>();
    qMeasured_.segment<4>(3) = tbai::mpc::quadruped::quaternionBaseToOrigin(eulerXYZ).coeffs();

    // joint angles: LF, RF, LH, RH
    qMeasured_.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>() =
        stateMeasured.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>();

    // flip lh and rf
    std::swap(qMeasured_[10], qMeasured_[13]);
    std::swap(qMeasured_[11], qMeasured_[14]);
    std::swap(qMeasured_[12], qMeasured_[15]);

    // Rotation matrix: base -> world
    rMeasured_ = tbai::mpc::quadruped::rotationMatrixBaseToOrigin(eulerXYZ);

    // base linear velocity - expressed in base frame
    vMeasured_.head<3>() = stateMeasured.segment<3>(9);

    // base angular velocity - expressed in base frame
    vMeasured_.segment<3>(3) = stateMeasured.segment<3>(6);

    // joint velocities: LF, RF, LH, RH
    vMeasured_.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>() =
        inputMeasured.tail<tbai::mpc::quadruped::JOINT_COORDINATE_SIZE>();
    std::swap(vMeasured_[9], vMeasured_[12]);
    std::swap(vMeasured_[10], vMeasured_[13]);
    std::swap(vMeasured_[11], vMeasured_[14]);

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

    // Contact jacobians
    matrix_t jac_temp;
    for (size_t i = 0; i < tbai::mpc::quadruped::NUM_CONTACT_POINTS; ++i) {
        size_t frameIndex = model.getBodyId(footNames_[i]);
        jac_temp.setZero(6, nGeneralizedCoordinates_);
        pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL_WORLD_ALIGNED, jac_temp);
        Jcontact_.block(3 * i, 0, 3, nGeneralizedCoordinates_) = jac_temp.template topRows<3>();
    }

    // Contact jacobian time derivatives
    matrix_t djac_temp;
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
    for (size_t i = 0; i < tbai::mpc::quadruped::NUM_CONTACT_POINTS; ++i) {
        size_t frameIndex = model.getBodyId(footNames_[i]);
        djac_temp.setZero(6, nGeneralizedCoordinates_);
        pinocchio::getFrameJacobianTimeVariation(model, data, frameIndex, pinocchio::LOCAL_WORLD_ALIGNED, djac_temp);
        dJcontactdt_.block(3 * i, 0, 3, nGeneralizedCoordinates_) = djac_temp.template topRows<3>();
    }
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
    rDesired_ = tbai::mpc::quadruped::rotationMatrixBaseToOrigin(eulerXYZ);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void WbcBase::updateContactFlags(const size_t modeCurrent, const size_t modeDesired) {
    // Desired contact flags
    contactFlags_ = tbai::mpc::quadruped::modeNumber2StanceLeg(modeDesired);
    nContacts_ = std::accumulate(contactFlags_.begin(), contactFlags_.end(), 0);
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
}

}  // namespace quadruped
}  // namespace mpc
}  // namespace tbai
