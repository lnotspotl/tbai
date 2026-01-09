#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace franka {

template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> getBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const FrankaModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  // Fixed-base manipulator: base position is always at origin
  return Eigen::Matrix<SCALAR, 3, 1>::Zero();
}

template <typename SCALAR>
Eigen::Quaternion<SCALAR> getBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const FrankaModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  // Fixed-base manipulator: base orientation is always identity
  return Eigen::Quaternion<SCALAR>::Identity();
}

template <typename Derived>
Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const FrankaModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  const size_t startRow = info.stateDim - info.armDim;
  return Eigen::Block<Derived, -1, 1>(state.derived(), startRow, 0, info.armDim, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getArmJointAngles(const Eigen::MatrixBase<Derived>& state, const FrankaModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  const size_t startRow = info.stateDim - info.armDim;
  return Eigen::Block<const Derived, -1, 1>(state.derived(), startRow, 0, info.armDim, 1);
}

}  // namespace franka
}  // namespace ocs2
