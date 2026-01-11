#pragma once

#include <memory>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace tbai::mpc::arm {

class EndEffectorConstraint final : public ocs2::StateConstraint {
 public:
  using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;

  EndEffectorConstraint(const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics, const ocs2::ReferenceManager& referenceManager);
  ~EndEffectorConstraint() override = default;
  EndEffectorConstraint* clone() const override { return new EndEffectorConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_); }

  size_t getNumConstraints(ocs2::scalar_t time) const override;
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComputation) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                           const ocs2::PreComputation& preComputation) const override;

 private:
  EndEffectorConstraint(const EndEffectorConstraint& other) = default;
  std::pair<ocs2::vector_t, quaternion_t> interpolateEndEffectorPose(ocs2::scalar_t time) const;

  ocs2::PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

  vector3_t eeDesiredPosition_;
  quaternion_t eeDesiredOrientation_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
  const ocs2::ReferenceManager* referenceManagerPtr_;
};

}  // namespace tbai::mpc::arm
