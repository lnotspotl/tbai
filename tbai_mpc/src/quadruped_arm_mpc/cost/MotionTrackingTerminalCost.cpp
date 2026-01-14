//
// Created by rgrandia on 10.03.22.
//

#include "tbai_mpc/quadruped_arm_mpc/cost/MotionTrackingTerminalCost.h"

#include "tbai_mpc/quadruped_arm_mpc/core/SwitchedModelPrecomputation.h"

namespace tbai::mpc::quadruped_arm {

MotionTrackingTerminalCost::MotionTrackingTerminalCost(matrix_t Q) : Q_(std::move(Q)) {}

MotionTrackingTerminalCost *MotionTrackingTerminalCost::clone() const {
    return new MotionTrackingTerminalCost(*this);
}

MotionTrackingTerminalCost::MotionTrackingTerminalCost(const MotionTrackingTerminalCost &other) : Q_(other.Q_) {}

scalar_t MotionTrackingTerminalCost::getValue(scalar_t time, const vector_t &state,
                                              const ocs2::TargetTrajectories &targetTrajectories,
                                              const ocs2::PreComputation &preComputation) const {
    const vector_t xDeviation = getStateDeviation(state, preComputation);
    return 0.5 * xDeviation.dot(Q_ * xDeviation);
}

ScalarFunctionQuadraticApproximation MotionTrackingTerminalCost::getQuadraticApproximation(
    scalar_t time, const vector_t &state, const ocs2::TargetTrajectories &targetTrajectories,
    const ocs2::PreComputation &preComputation) const {
    const vector_t xDeviation = getStateDeviation(state, preComputation);

    ScalarFunctionQuadraticApproximation Phi;
    Phi.dfdxx = Q_;
    Phi.dfdx.noalias() = Q_ * xDeviation;
    Phi.f = 0.5 * xDeviation.dot(Phi.dfdx);
    return Phi;
}

vector_t MotionTrackingTerminalCost::getStateDeviation(const vector_t &state,
                                                       const ocs2::PreComputation &preComputation) const {
    const auto &switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComputation);
    return state - switchedModelPreComp.getStateReference();
}

}  // namespace tbai::mpc::quadruped_arm
