#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "tbai_mpc/arm_mpc/ArmModelInfo.h"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace tbai::mpc::arm {

class ArmDynamics final : public ocs2::SystemDynamicsBaseAD {
 public:
  ArmDynamics(const ArmModelInfo& modelInfo, const std::string& modelName,
                 const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);

  ~ArmDynamics() override = default;
  ArmDynamics* clone() const override { return new ArmDynamics(*this); }

 private:
  ArmDynamics(const ArmDynamics& rhs) = default;

  ocs2::ad_vector_t systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                            const ocs2::ad_vector_t& /*parameters*/) const override;
};

}  // namespace tbai::mpc::arm
