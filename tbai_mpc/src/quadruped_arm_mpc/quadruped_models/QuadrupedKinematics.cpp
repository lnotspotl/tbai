#include "tbai_mpc/quadruped_arm_mpc/quadruped_models/QuadrupedKinematics.h"

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace tbai::mpc::quadruped_arm {
namespace tpl {

template <typename SCALAR_T>
QuadrupedKinematics<SCALAR_T>::QuadrupedKinematics(const FrameDeclaration &frameDeclaration,
                                                   const ocs2::PinocchioInterface &pinocchioInterface)
    : pinocchioInterfacePtr_(new PinocchioInterface_s_t(castPinocchioInterface(pinocchioInterface))),
      pinocchioMapping_(frameDeclaration, pinocchioInterface) {
    for (size_t footIndex = 0; footIndex < tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS; ++footIndex) {
        tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> zeroConfiguration;
        zeroConfiguration.setZero();
        baseToLegRootInBaseFrame_[footIndex] =
            relativeTranslationInBaseFrame(zeroConfiguration, pinocchioMapping_.getHipFrameId(footIndex));
    }
}

template <typename SCALAR_T>
QuadrupedKinematics<SCALAR_T>::QuadrupedKinematics(const QuadrupedKinematics &rhs)
    : pinocchioInterfacePtr_(new PinocchioInterface_s_t(*rhs.pinocchioInterfacePtr_)),
      pinocchioMapping_(rhs.pinocchioMapping_),
      baseToLegRootInBaseFrame_(rhs.baseToLegRootInBaseFrame_){};

template <typename SCALAR_T>
QuadrupedKinematics<SCALAR_T> *QuadrupedKinematics<SCALAR_T>::clone() const {
    return new QuadrupedKinematics<SCALAR_T>(*this);
}

template <typename SCALAR_T>
tbai::mpc::quadruped_arm::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::baseToLegRootInBaseFrame(size_t footIndex) const {
    return baseToLegRootInBaseFrame_[footIndex];
}

template <typename SCALAR_T>
tbai::mpc::quadruped_arm::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::positionBaseToFootInBaseFrame(
    size_t footIndex, const tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> &jointPositions) const {
    pinocchio::FrameIndex frameId = pinocchioMapping_.getFootFrameId(footIndex);
    const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
    return relativeTranslationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::baseToFootJacobianBlockInBaseFrame(
    std::size_t footIndex,
    const tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> &jointPositions) const -> joint_jacobian_block_t {
    auto &data = pinocchioInterfacePtr_->getData();
    const auto &model = pinocchioInterfacePtr_->getModel();

    const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
    pinocchio::FrameIndex frameId = pinocchioMapping_.getFootFrameId(footIndex);

    joint_jacobian_t J;
    pinocchio::computeFrameJacobian(model, data, pinocchioJointPositions, frameId,
                                    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    // In Pinocchio, the first three coordinates of J represent linear part and the last three coordinates represent
    // angular part. But it is the other way round in OCS2. Swap linear and angular parts to stick to the originally
    // used OCS2's convention.
    joint_jacobian_block_t res;
    res.template block<3, 3>(0, 0) = J.template block<3, 3>(3, pinocchioMapping_.getPinocchioFootIndex(footIndex) * 3u);
    res.template block<3, 3>(3, 0) = J.template block<3, 3>(0, pinocchioMapping_.getPinocchioFootIndex(footIndex) * 3u);

    return res;
}

template <typename SCALAR_T>
tbai::mpc::quadruped_arm::matrix3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::footOrientationInBaseFrame(
    size_t footIndex, const tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> &jointPositions) const {
    pinocchio::FrameIndex frameId = pinocchioMapping_.getFootFrameId(footIndex);
    const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
    return relativeOrientationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
tbai::mpc::quadruped_arm::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::footVelocityRelativeToBaseInBaseFrame(
    size_t footIndex, const tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> &jointPositions,
    const tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> &jointVelocities) const {
    auto &data = pinocchioInterfacePtr_->getData();
    const auto &model = pinocchioInterfacePtr_->getModel();

    const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
    const auto pinocchioJointVelocities = pinocchioMapping_.getPinocchioJointVector(jointVelocities);

    pinocchio::forwardKinematics(model, data, pinocchioJointPositions, pinocchioJointVelocities);

    pinocchio::FrameIndex frameId = pinocchioMapping_.getFootFrameId(footIndex);

    return getFrameVelocity(model, data, frameId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
}

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::collisionSpheresInBaseFrame(
    const tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> &jointPositions) const -> std::vector<CollisionSphere> {
    auto &data = pinocchioInterfacePtr_->getData();
    const auto &model = pinocchioInterfacePtr_->getModel();
    const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
    pinocchio::forwardKinematics(model, data, pinocchioJointPositions);

    const auto &linkFrames = pinocchioMapping_.getCollisionLinkFrameIds();
    const auto &decl = pinocchioMapping_.getCollisionDeclaration();

    std::vector<CollisionSphere> collisionSpheres;
    collisionSpheres.reserve(linkFrames.size());
    for (int i = 0; i < linkFrames.size(); ++i) {
        const auto &transformation = pinocchio::updateFramePlacement(model, data, linkFrames[i]);
        const tbai::mpc::quadruped_arm::vector3_s_t<SCALAR_T> offset = decl[i].offset.cast<SCALAR_T>();
        collisionSpheres.push_back({transformation.act(offset), SCALAR_T(decl[i].radius)});
    }

    return collisionSpheres;
}

template <typename SCALAR_T>
tbai::mpc::quadruped_arm::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::relativeTranslationInBaseFrame(
    const tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> &jointPositions, pinocchio::FrameIndex frame) const {
    auto &data = pinocchioInterfacePtr_->getData();
    const auto &model = pinocchioInterfacePtr_->getModel();
    pinocchio::forwardKinematics(model, data, jointPositions);

    return pinocchio::updateFramePlacement(model, data, frame).translation();
}

template <typename SCALAR_T>
tbai::mpc::quadruped_arm::matrix3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::relativeOrientationInBaseFrame(
    const tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> &jointPositions, pinocchio::FrameIndex frame) const {
    auto &data = pinocchioInterfacePtr_->getData();
    const auto &model = pinocchioInterfacePtr_->getModel();
    pinocchio::forwardKinematics(model, data, jointPositions);

    return pinocchio::updateFramePlacement(model, data, frame).rotation();
}

template <typename SCALAR_T>
tbai::mpc::quadruped_arm::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::armEEPositionInBaseFrame(
    const tbai::mpc::quadruped_arm::leg_joint_coordinate_s_t<SCALAR_T> &legJointPositions,
    const tbai::mpc::quadruped_arm::arm_joint_s_t<SCALAR_T> &armJointPositions) const {
    // Combine leg and arm joint positions into full joint coordinate vector
    tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> jointPositions;
    jointPositions.template head<tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE>() = legJointPositions;
    jointPositions.template tail<tbai::mpc::quadruped_arm::NUM_ARM_JOINTS>() = armJointPositions;

    const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
    pinocchio::FrameIndex frameId = pinocchioMapping_.getArmEEFrameId();

    return relativeTranslationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
tbai::mpc::quadruped_arm::matrix3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::armEEOrientationInBaseFrame(
    const tbai::mpc::quadruped_arm::leg_joint_coordinate_s_t<SCALAR_T> &legJointPositions,
    const tbai::mpc::quadruped_arm::arm_joint_s_t<SCALAR_T> &armJointPositions) const {
    // Combine leg and arm joint positions into full joint coordinate vector
    tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> jointPositions;
    jointPositions.template head<tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE>() = legJointPositions;
    jointPositions.template tail<tbai::mpc::quadruped_arm::NUM_ARM_JOINTS>() = armJointPositions;

    const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
    pinocchio::FrameIndex frameId = pinocchioMapping_.getArmEEFrameId();

    return relativeOrientationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::armEEJacobianInBaseFrame(
    const tbai::mpc::quadruped_arm::leg_joint_coordinate_s_t<SCALAR_T> &legJointPositions,
    const tbai::mpc::quadruped_arm::arm_joint_s_t<SCALAR_T> &armJointPositions) const -> typename BASE::arm_jacobian_t {
    // Combine leg and arm joint positions into full joint coordinate vector
    tbai::mpc::quadruped_arm::joint_coordinate_s_t<SCALAR_T> jointPositions;
    jointPositions.template head<tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE>() = legJointPositions;
    jointPositions.template tail<tbai::mpc::quadruped_arm::NUM_ARM_JOINTS>() = armJointPositions;

    const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
    pinocchio::FrameIndex frameId = pinocchioMapping_.getArmEEFrameId();

    auto &data = pinocchioInterfacePtr_->getData();
    const auto &model = pinocchioInterfacePtr_->getModel();

    // Compute full Jacobian (6 x nv)
    Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic> fullJacobian(6, model.nv);
    fullJacobian.setZero();
    pinocchio::computeFrameJacobian(model, data, pinocchioJointPositions, frameId,
                                    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, fullJacobian);

    // Extract arm joint columns (arm joints are after leg joints in pinocchio joint vector)
    // Arm joints start at index LEG_JOINT_COORDINATE_SIZE in the pinocchio joint vector
    typename BASE::arm_jacobian_t armJacobian;

    // In Pinocchio: rows 0-2 are linear, rows 3-5 are angular
    // In OCS2 convention: rows 0-2 are angular, rows 3-5 are linear
    // Swap to match OCS2 convention
    armJacobian.template block<3, tbai::mpc::quadruped_arm::NUM_ARM_JOINTS>(0, 0) =
        fullJacobian.template block<3, tbai::mpc::quadruped_arm::NUM_ARM_JOINTS>(3, tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE);
    armJacobian.template block<3, tbai::mpc::quadruped_arm::NUM_ARM_JOINTS>(3, 0) =
        fullJacobian.template block<3, tbai::mpc::quadruped_arm::NUM_ARM_JOINTS>(0, tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE);

    return armJacobian;
}

}  // namespace tpl
}  // namespace tbai::mpc::quadruped_arm

// Explicit instantiation
template class tbai::mpc::quadruped_arm::tpl::QuadrupedKinematics<ocs2::scalar_t>;
template class tbai::mpc::quadruped_arm::tpl::QuadrupedKinematics<ocs2::ad_scalar_t>;
