#pragma once

#include <array>
#include <string>
#include <vector>

#include <tbai_mpc/quadruped_mpc/core/KinematicsModelBase.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedPinocchioMapping.h"

namespace tbai::mpc::quadruped {
namespace tpl {

template <typename SCALAR_T>
class QuadrupedKinematics final : public tbai::mpc::quadruped::KinematicsModelBase<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef tbai::mpc::quadruped::KinematicsModelBase<SCALAR_T> BASE;
  using typename BASE::CollisionSphere;
  using typename BASE::joint_jacobian_block_t;
  using typename BASE::joint_jacobian_t;

  QuadrupedKinematics(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface);
  ~QuadrupedKinematics() = default;

  QuadrupedKinematics<SCALAR_T>* clone() const override;

  tbai::mpc::quadruped::vector3_s_t<SCALAR_T> baseToLegRootInBaseFrame(size_t footIndex) const override;

  tbai::mpc::quadruped::vector3_s_t<SCALAR_T> positionBaseToFootInBaseFrame(
      size_t footIndex, const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  joint_jacobian_block_t baseToFootJacobianBlockInBaseFrame(
      size_t footIndex, const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  tbai::mpc::quadruped::matrix3_s_t<SCALAR_T> footOrientationInBaseFrame(
      size_t footIndex, const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  tbai::mpc::quadruped::vector3_s_t<SCALAR_T> footVelocityRelativeToBaseInBaseFrame(
      size_t footIndex, const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions,
      const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointVelocities) const override;

  std::vector<CollisionSphere> collisionSpheresInBaseFrame(
      const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

 private:
  QuadrupedKinematics(const QuadrupedKinematics& rhs);

  using PinocchioInterface_s_t = ocs2::PinocchioInterfaceTpl<SCALAR_T>;

  template <typename T = SCALAR_T, typename std::enable_if<std::is_same<T, ocs2::ad_scalar_t>::value, bool>::type = true>
  PinocchioInterface_s_t castPinocchioInterface(const ocs2::PinocchioInterface& pinocchioInterface) {
    return pinocchioInterface.toCppAd();
  }

  template <typename T = SCALAR_T, typename std::enable_if<!std::is_same<T, ocs2::ad_scalar_t>::value, bool>::type = true>
  PinocchioInterface_s_t castPinocchioInterface(const ocs2::PinocchioInterface& pinocchioInterface) {
    return pinocchioInterface;
  }

  tbai::mpc::quadruped::vector3_s_t<SCALAR_T> relativeTranslationInBaseFrame(const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                       std::size_t frame) const;
  tbai::mpc::quadruped::matrix3_s_t<SCALAR_T> relativeOrientationInBaseFrame(const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                       std::size_t frame) const;

  std::unique_ptr<PinocchioInterface_s_t> pinocchioInterfacePtr_;
  QuadrupedPinocchioMapping pinocchioMapping_;
  tbai::mpc::quadruped::feet_array_t<tbai::mpc::quadruped::vector3_s_t<SCALAR_T>> baseToLegRootInBaseFrame_;
};

}  // namespace tpl

using QuadrupedKinematics = tpl::QuadrupedKinematics<ocs2::scalar_t>;
using QuadrupedKinematicsAd = tpl::QuadrupedKinematics<ocs2::ad_scalar_t>;
}  // namespace tbai::mpc::quadruped

/**
 *  Explicit instantiation, for instantiation additional types, include the implementation file instead of this one.
 */
extern template class tbai::mpc::quadruped::tpl::QuadrupedKinematics<ocs2::scalar_t>;
extern template class tbai::mpc::quadruped::tpl::QuadrupedKinematics<ocs2::ad_scalar_t>;
