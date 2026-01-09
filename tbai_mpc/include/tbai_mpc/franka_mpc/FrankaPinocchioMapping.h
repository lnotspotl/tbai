#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include "tbai_mpc/franka_mpc/FrankaModelInfo.h"

namespace ocs2 {
namespace franka {

template <typename SCALAR>
class FrankaPinocchioMappingTpl;

using FrankaPinocchioMapping = FrankaPinocchioMappingTpl<scalar_t>;
using FrankaPinocchioMappingCppAd = FrankaPinocchioMappingTpl<ad_scalar_t>;

/**
 * Pinocchio state and input mapping.
 */
template <typename SCALAR>
class FrankaPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR> {
 public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;

  explicit FrankaPinocchioMappingTpl(FrankaModelInfo info);

  ~FrankaPinocchioMappingTpl() override = default;
  FrankaPinocchioMappingTpl<SCALAR>* clone() const override;

  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  const FrankaModelInfo& getFrankaModelInfo() const { return modelInfo_; }

 private:
  FrankaPinocchioMappingTpl(const FrankaPinocchioMappingTpl& rhs) = default;

  const FrankaModelInfo modelInfo_;
};

}  // namespace franka
}  // namespace ocs2
