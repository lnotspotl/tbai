#include "tbai_mpc/franka_mpc/dynamics/FrankaDynamics.h"

namespace ocs2 {
namespace franka {

FrankaDynamics::FrankaDynamics(const FrankaModelInfo &info, const std::string &modelName,
                               const std::string &modelFolder, bool recompileLibraries,
                               bool verbose) {
    this->initialize(info.stateDim, info.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

ad_vector_t FrankaDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t &state,
                                          const ad_vector_t &input, const ad_vector_t &) const {
    return input;
}

}  // namespace franka
}  // namespace ocs2
