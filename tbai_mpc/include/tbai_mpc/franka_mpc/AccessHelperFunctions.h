#pragma once

#include <Eigen/Core>

#include "tbai_mpc/franka_mpc/FrankaModelInfo.h"

namespace ocs2 {
namespace franka {

/** Provides read/write access to the base position. */
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> getBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const FrankaModelInfo& info);

/** Provides read/write access to the base orientation. */
template <typename SCALAR>
Eigen::Quaternion<SCALAR> getBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const FrankaModelInfo& info);

/** Provides read/write access to the arm joint angles. */
template <typename Derived>
Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const FrankaModelInfo& info);

/** Provides read access to the arm joint angles. */
template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getArmJointAngles(const Eigen::MatrixBase<Derived>& state, const FrankaModelInfo& info);

}  // namespace franka
}  // namespace ocs2

#include "implementation/AccessHelperFunctionsImpl.h"
