#include "tbai_mpc/franka_mpc/FrankaPinocchioMapping.h"

namespace ocs2 {
namespace franka {

template <typename SCALAR>
FrankaPinocchioMappingTpl<SCALAR>::FrankaPinocchioMappingTpl(FrankaModelInfo info)
    : modelInfo_(std::move(info)) {}

template <typename SCALAR>
FrankaPinocchioMappingTpl<SCALAR> *FrankaPinocchioMappingTpl<SCALAR>::clone() const {
    return new FrankaPinocchioMappingTpl<SCALAR>(*this);
}

template <typename SCALAR>
auto FrankaPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t &state) const -> vector_t {
    return state;
}

template <typename SCALAR>
auto FrankaPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t &state,
                                                                             const vector_t &input) const -> vector_t {
    return input;
}

template <typename SCALAR>
auto FrankaPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(
    const vector_t &state, const matrix_t &Jq, const matrix_t &Jv) const -> std::pair<matrix_t, matrix_t> {
    return {Jq, Jv};
}

// explicit template instantiation
template class ocs2::franka::FrankaPinocchioMappingTpl<ocs2::scalar_t>;
template class ocs2::franka::FrankaPinocchioMappingTpl<ocs2::ad_scalar_t>;

}  // namespace franka
}  // namespace ocs2
