#pragma once

#include <ocs2_core/cost/QuadraticStateInputCost.h>

namespace tbai::mpc::arm {

class QuadraticInputCost final : public ocs2::QuadraticStateInputCost {
 public:
  QuadraticInputCost(ocs2::matrix_t R, size_t stateDim)
      : ocs2::QuadraticStateInputCost(ocs2::matrix_t::Zero(stateDim, stateDim), std::move(R)), stateDim_(stateDim) {}

  ~QuadraticInputCost() override = default;

  QuadraticInputCost(const QuadraticInputCost& rhs) = default;
  QuadraticInputCost* clone() const override { return new QuadraticInputCost(*this); }

  std::pair<ocs2::vector_t, ocs2::vector_t> getStateInputDeviation(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                                                       const ocs2::TargetTrajectories& targetTrajectories) const override {
    const ocs2::vector_t inputDeviation = input - targetTrajectories.getDesiredInput(time);
    return {ocs2::vector_t::Zero(stateDim_), inputDeviation};
  }

 private:
  const size_t stateDim_;
};

}  // namespace tbai::mpc::arm
