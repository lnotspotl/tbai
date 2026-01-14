#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include "tbai_mpc/arm_mpc/ArmModelInfo.h"

namespace tbai::mpc::arm {

template <typename SCALAR>
class ArmPinocchioMappingTpl;

using ArmPinocchioMapping = ArmPinocchioMappingTpl<ocs2::scalar_t>;
using ArmPinocchioMappingCppAd = ArmPinocchioMappingTpl<ocs2::ad_scalar_t>;

/**
 * Pinocchio state and input mapping.
 */
template <typename SCALAR>
class ArmPinocchioMappingTpl final : public ocs2::PinocchioStateInputMapping<SCALAR> {
 public:
  using Base = ocs2::PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;

  explicit ArmPinocchioMappingTpl(ArmModelInfo info);

  ~ArmPinocchioMappingTpl() override = default;
  ArmPinocchioMappingTpl<SCALAR>* clone() const override;

  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  const ArmModelInfo& getArmModelInfo() const { return modelInfo_; }

 private:
  ArmPinocchioMappingTpl(const ArmPinocchioMappingTpl& rhs) = default;

  const ArmModelInfo modelInfo_;
};

}  // namespace tbai::mpc::arm
