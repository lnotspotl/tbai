/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "tbai_mpc/franka_mpc/MobileManipulatorPinocchioMapping.h"

namespace ocs2 {
namespace mobile_manipulator {

template <typename SCALAR>
MobileManipulatorPinocchioMappingTpl<SCALAR>::MobileManipulatorPinocchioMappingTpl(ManipulatorModelInfo info)
    : modelInfo_(std::move(info)) {}

template <typename SCALAR>
MobileManipulatorPinocchioMappingTpl<SCALAR> *MobileManipulatorPinocchioMappingTpl<SCALAR>::clone() const {
    return new MobileManipulatorPinocchioMappingTpl<SCALAR>(*this);
}

template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t &state) const -> vector_t {
    return state;
}

template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t &state,
                                                                             const vector_t &input) const -> vector_t {
    switch (modelInfo_.manipulatorModelType) {
        case ManipulatorModelType::DefaultManipulator:
            return input;
        default:
            throw std::runtime_error("The chosen manipulator model type is not supported!");
    }
}

template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(
    const vector_t &state, const matrix_t &Jq, const matrix_t &Jv) const -> std::pair<matrix_t, matrix_t> {
    switch (modelInfo_.manipulatorModelType) {
        case ManipulatorModelType::DefaultManipulator:
            return {Jq, Jv};
        default:
            throw std::runtime_error("The chosen manipulator model type is not supported!");
    }
}

// explicit template instantiation
template class ocs2::mobile_manipulator::MobileManipulatorPinocchioMappingTpl<ocs2::scalar_t>;
template class ocs2::mobile_manipulator::MobileManipulatorPinocchioMappingTpl<ocs2::ad_scalar_t>;

}  // namespace mobile_manipulator
}  // namespace ocs2
