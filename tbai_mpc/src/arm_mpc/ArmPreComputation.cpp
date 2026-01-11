#include "tbai_mpc/arm_mpc/ArmPreComputation.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>

namespace tbai::mpc::arm {

ArmPreComputation::ArmPreComputation(ocs2::PinocchioInterface pinocchioInterface, const ArmModelInfo &info)
    : pinocchioInterface_(std::move(pinocchioInterface)), pinocchioMapping_(info) {}

ArmPreComputation *ArmPreComputation::clone() const {
    return new ArmPreComputation(pinocchioInterface_, pinocchioMapping_.getArmModelInfo());
}

void ArmPreComputation::request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t &x, const ocs2::vector_t &u) {
    if (!request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
        return;
    }

    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

    if (request.contains(ocs2::Request::Approximation)) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeJointJacobians(model, data);
        pinocchio::updateGlobalPlacements(model, data);
    } else {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
    }
}

void ArmPreComputation::requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t &x) {
    if (!request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
        return;
    }

    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

    if (request.contains(ocs2::Request::Approximation)) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeJointJacobians(model, data);
    } else {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
    }
}

}  // namespace tbai::mpc::arm
