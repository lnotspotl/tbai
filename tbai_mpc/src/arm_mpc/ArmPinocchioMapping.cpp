#include "tbai_mpc/arm_mpc/ArmPinocchioMapping.h"

namespace tbai::mpc::arm {

template <typename SCALAR>
ArmPinocchioMappingTpl<SCALAR>::ArmPinocchioMappingTpl(ArmModelInfo info) : modelInfo_(std::move(info)) {}

template <typename SCALAR>
ArmPinocchioMappingTpl<SCALAR> *ArmPinocchioMappingTpl<SCALAR>::clone() const {
    return new ArmPinocchioMappingTpl<SCALAR>(*this);
}

template <typename SCALAR>
auto ArmPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t &state) const -> vector_t {
    return state;
}

template <typename SCALAR>
auto ArmPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t &state,
                                                                  const vector_t &input) const -> vector_t {
    return input;
}

template <typename SCALAR>
auto ArmPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t &state, const matrix_t &Jq,
                                                        const matrix_t &Jv) const -> std::pair<matrix_t, matrix_t> {
    return {Jq, Jv};
}

// explicit template instantiation
template class tbai::mpc::arm::ArmPinocchioMappingTpl<ocs2::scalar_t>;
template class tbai::mpc::arm::ArmPinocchioMappingTpl<ocs2::ad_scalar_t>;

}  // namespace tbai::mpc::arm
