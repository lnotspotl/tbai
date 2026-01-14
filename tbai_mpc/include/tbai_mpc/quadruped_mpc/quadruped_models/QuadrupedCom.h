#pragma once

#include <tbai_mpc/quadruped_mpc/core/ComModelBase.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedPinocchioMapping.h"

namespace tbai::mpc::quadruped {
namespace tpl {

template <typename SCALAR_T>
class QuadrupedCom : public tbai::mpc::quadruped::ComModelBase<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadrupedCom(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface);
  ~QuadrupedCom() = default;

  QuadrupedCom<SCALAR_T>* clone() const override;

  SCALAR_T totalMass() const override { return totalMass_; }

  tbai::mpc::quadruped::vector3_s_t<SCALAR_T> centerOfMassInBaseFrame(const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  tbai::mpc::quadruped::vector3_s_t<SCALAR_T> centerOfMassInWorldFrame(const tbai::mpc::quadruped::base_coordinate_s_t<SCALAR_T>& basePose,
                                                     const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  tbai::mpc::quadruped::base_coordinate_s_t<SCALAR_T> calculateBaseLocalAccelerations(
      const tbai::mpc::quadruped::base_coordinate_s_t<SCALAR_T>& basePose,
      const tbai::mpc::quadruped::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
      const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions,
      const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointVelocities,
      const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
      const tbai::mpc::quadruped::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const override;

 private:
  QuadrupedCom(const QuadrupedCom& rhs);

  Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> getPinnochioConfiguration(
      const tbai::mpc::quadruped::base_coordinate_s_t<SCALAR_T>& basePose,
      const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> getPinnochioVelocity(
      const tbai::mpc::quadruped::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
      const tbai::mpc::quadruped::joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  using PinocchioInterface_s_t = ocs2::PinocchioInterfaceTpl<SCALAR_T>;

  template <typename T = SCALAR_T, typename std::enable_if<std::is_same<T, ocs2::ad_scalar_t>::value, bool>::type = true>
  PinocchioInterface_s_t castPinocchioInterface(const ocs2::PinocchioInterface& pinocchioInterface) {
    return pinocchioInterface.toCppAd();
  }

  template <typename T = SCALAR_T, typename std::enable_if<!std::is_same<T, ocs2::ad_scalar_t>::value, bool>::type = true>
  PinocchioInterface_s_t castPinocchioInterface(const ocs2::PinocchioInterface& pinocchioInterface) {
    return pinocchioInterface;
  }

  std::unique_ptr<PinocchioInterface_s_t> pinocchioInterfacePtr_;
  QuadrupedPinocchioMapping pinocchioMapping_;

  SCALAR_T totalMass_;
};

}  // namespace tpl

ocs2::PinocchioInterface createQuadrupedPinocchioInterfaceFromUrdfString(const std::string& urdfString);

using QuadrupedCom = tpl::QuadrupedCom<ocs2::scalar_t>;
using QuadrupedComAd = tpl::QuadrupedCom<ocs2::ad_scalar_t>;

}  // namespace tbai::mpc::quadruped

/**
 *  Explicit instantiation, for instantiation additional types, include the implementation file instead of this one.
 */
extern template class tbai::mpc::quadruped::tpl::QuadrupedCom<ocs2::scalar_t>;
extern template class tbai::mpc::quadruped::tpl::QuadrupedCom<ocs2::ad_scalar_t>;
