#pragma once

#include <Eigen/Core>
#include <tbai_core/Types.hpp>

namespace tbai {
namespace g1_rl {

struct State {
    using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
    using Vector4 = Eigen::Matrix<scalar_t, 4, 1>;
    using Vector29 = Eigen::Matrix<scalar_t, 29, 1>;

    /** Base position in the world frame */
    Vector3 basePositionWorld;

    /** Base orientation in the world frame - xyzw quaternion */
    Vector4 baseOrientationWorld;

    /** Base linear velocity expressed int the base frame */
    Vector3 baseLinearVelocityBase;

    /** Base angular velocity expressed in the base frame */
    Vector3 baseAngularVelocityBase;

    /** Gravity vector expressed in the base frame */
    Vector3 normalizedGravityBase;

    /** Joint angles */
    Vector29 jointPositions;

    /** Joint velocities */
    Vector29 jointVelocities;
};

}  // namespace g1_rl
}  // namespace tbai