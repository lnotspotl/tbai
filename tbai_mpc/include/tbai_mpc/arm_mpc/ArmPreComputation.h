#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "tbai_mpc/arm_mpc/ArmModelInfo.h"
#include "tbai_mpc/arm_mpc/ArmPinocchioMapping.h"

namespace tbai::mpc::arm {

/** Callback for caching and reference update */
class ArmPreComputation : public ocs2::PreComputation {
 public:
  ArmPreComputation(ocs2::PinocchioInterface pinocchioInterface, const ArmModelInfo& info);

  ~ArmPreComputation() override = default;

  ArmPreComputation(const ArmPreComputation& rhs) = delete;
  ArmPreComputation* clone() const override;

  void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) override;
  void requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x) override;

  ocs2::PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const ocs2::PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  ocs2::PinocchioInterface pinocchioInterface_;
  ArmPinocchioMapping pinocchioMapping_;
};

}  // namespace tbai::mpc::arm
