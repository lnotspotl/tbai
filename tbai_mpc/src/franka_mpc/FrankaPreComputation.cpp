#include "tbai_mpc/franka_mpc/FrankaPreComputation.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>

namespace ocs2 {
namespace franka {

FrankaPreComputation::FrankaPreComputation(PinocchioInterface pinocchioInterface,
                                                                 const FrankaModelInfo &info)
    : pinocchioInterface_(std::move(pinocchioInterface)), pinocchioMapping_(info) {}

FrankaPreComputation *FrankaPreComputation::clone() const {
    return new FrankaPreComputation(pinocchioInterface_, pinocchioMapping_.getFrankaModelInfo());
}

void FrankaPreComputation::request(RequestSet request, scalar_t t, const vector_t &x, const vector_t &u) {
    if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
        return;
    }

    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

    if (request.contains(Request::Approximation)) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeJointJacobians(model, data);
        pinocchio::updateGlobalPlacements(model, data);
    } else {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
    }
}

void FrankaPreComputation::requestFinal(RequestSet request, scalar_t t, const vector_t &x) {
    if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
        return;
    }

    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

    if (request.contains(Request::Approximation)) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeJointJacobians(model, data);
    } else {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
    }
}

}  // namespace franka
}  // namespace ocs2
