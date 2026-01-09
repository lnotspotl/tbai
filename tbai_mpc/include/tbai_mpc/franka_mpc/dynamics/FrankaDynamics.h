#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "tbai_mpc/franka_mpc/FrankaModelInfo.h"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2 {
namespace franka {

class FrankaDynamics final : public SystemDynamicsBaseAD {
 public:
  FrankaDynamics(const FrankaModelInfo& modelInfo, const std::string& modelName,
                 const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);

  ~FrankaDynamics() override = default;
  FrankaDynamics* clone() const override { return new FrankaDynamics(*this); }

 private:
  FrankaDynamics(const FrankaDynamics& rhs) = default;

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& /*parameters*/) const override;
};

}  // namespace franka
}  // namespace ocs2
