#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "tbai_mpc/quadruped_mpc/core/ComModelBase.h"
#include "tbai_mpc/quadruped_mpc/core/KinematicsModelBase.h"
#include "tbai_mpc/quadruped_mpc/core/ModelSettings.h"
#include "tbai_mpc/quadruped_mpc/core/SwitchedModel.h"
#include "tbai_mpc/quadruped_mpc/dynamics/ComKinoDynamicsParameters.h"
#include "tbai_mpc/quadruped_mpc/logic/DynamicsParametersSynchronizedModule.h"
#include "tbai_mpc/quadruped_mpc/logic/SwitchedModelModeScheduleManager.h"

namespace tbai::mpc::quadruped {

class ComKinoSystemDynamicsAd : public ocs2::SystemDynamicsBaseAD {
 public:
  using Base = ocs2::SystemDynamicsBaseAD;

  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;
  using parameters_t = ComKinoSystemDynamicsParameters<scalar_t>;
  using ad_parameters_t = ComKinoSystemDynamicsParameters<ad_scalar_t>;

  explicit ComKinoSystemDynamicsAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                   const DynamicsParametersSynchronizedModule& dynamicsParametersModule, ModelSettings settings);

  ComKinoSystemDynamicsAd(const ComKinoSystemDynamicsAd& rhs);

  ~ComKinoSystemDynamicsAd() override = default;

  ComKinoSystemDynamicsAd* clone() const override;

  ocs2::ad_vector_t systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                  const ocs2::ad_vector_t& parameters) const override;

  ocs2::vector_t getFlowMapParameters(scalar_t time, const ocs2::PreComputation& /* preComputation */) const override {
    return dynamicsParametersModulePtr_->getActiveDynamicsParameters().asVector();
  }

  size_t getNumFlowMapParameters() const override { return parameters_t::getNumParameters(); }

  template <typename SCALAR_T>
  static com_state_s_t<SCALAR_T> computeComStateDerivative(
      const ComModelBase<SCALAR_T>& comModel, const KinematicsModelBase<SCALAR_T>& kinematicsModel,
      const comkino_state_s_t<SCALAR_T>& comKinoState, const comkino_input_s_t<SCALAR_T>& comKinoInput,
      const ComKinoSystemDynamicsParameters<SCALAR_T>& parameters = ComKinoSystemDynamicsParameters<SCALAR_T>());

 private:
  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
  const DynamicsParametersSynchronizedModule* dynamicsParametersModulePtr_;
  ModelSettings settings_;
};

//! Explicit instantiations of computeComStateDerivative
extern template com_state_t ComKinoSystemDynamicsAd::computeComStateDerivative(const ComModelBase<scalar_t>& comModel,
                                                                               const KinematicsModelBase<scalar_t>& kinematicsModel,
                                                                               const comkino_state_t& comKinoState,
                                                                               const comkino_input_t& comKinoInput,
                                                                               const parameters_t& parameters);
extern template com_state_ad_t ComKinoSystemDynamicsAd::computeComStateDerivative(const ComModelBase<ad_scalar_t>& comModel,
                                                                                  const KinematicsModelBase<ad_scalar_t>& kinematicsModel,
                                                                                  const comkino_state_ad_t& comKinoState,
                                                                                  const comkino_input_ad_t& comKinoInput,
                                                                                  const ad_parameters_t& parameters);

}  // namespace tbai::mpc::quadruped
