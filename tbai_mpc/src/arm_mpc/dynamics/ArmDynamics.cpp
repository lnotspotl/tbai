#include "tbai_mpc/arm_mpc/dynamics/ArmDynamics.h"

namespace tbai::mpc::arm {

ArmDynamics::ArmDynamics(const ArmModelInfo &info, const std::string &modelName,
                               const std::string &modelFolder, bool recompileLibraries, bool verbose) {
    this->initialize(info.stateDim, info.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

ocs2::ad_vector_t ArmDynamics::systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t &state, const ocs2::ad_vector_t &input,
                                          const ocs2::ad_vector_t &) const {
    return input;
}

}  // namespace tbai::mpc::arm
