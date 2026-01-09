#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "tbai_mpc/franka_mpc/FrankaModelInfo.h"
#include "tbai_mpc/franka_mpc/FrankaPinocchioMapping.h"

namespace ocs2 {
namespace franka {

/** Callback for caching and reference update */
class FrankaPreComputation : public PreComputation {
 public:
  FrankaPreComputation(PinocchioInterface pinocchioInterface, const FrankaModelInfo& info);

  ~FrankaPreComputation() override = default;

  FrankaPreComputation(const FrankaPreComputation& rhs) = delete;
  FrankaPreComputation* clone() const override;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;
  void requestFinal(RequestSet request, scalar_t t, const vector_t& x) override;

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  PinocchioInterface pinocchioInterface_;
  FrankaPinocchioMapping pinocchioMapping_;
};

}  // namespace franka
}  // namespace ocs2
