#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/math/rpy.hpp>
#include <tbai/core/Throws.hpp>
#include <tbai/core/Types.hpp>

namespace tbai {

/**
 * @brief Convert roll-pitch-yaw euler angles to quaternion (Z rotation first, then Y, then X)
 *
 * @param rpy : roll-pitch-yaw euler angles
 * @return quaternion_t : quaternion
 */
inline quaternion_t rpy2quat(const vector3_t &rpy) {
    return angleaxis_t(rpy(2), vector3_t::UnitZ()) * angleaxis_t(rpy(1), vector3_t::UnitY()) *
           angleaxis_t(rpy(0), vector3_t::UnitX());
}

// https://github.com/leggedrobotics/ocs2/blob/164c26b46bed5d24cd03d90588db8980d03a4951/ocs2_robotic_tools/include/ocs2_robotic_tools/common/RotationTransforms.h#L126C1-L170C2
template <typename Scalar>
void makeEulerAnglesUnique(Eigen::Matrix<Scalar, 3, 1> &eulerAngles) {
    Scalar tol(1e-9);  // FIXME(jcarius) magic number
    Scalar pi(M_PI);

    if (eulerAngles.y() < -pi / 2 - tol) {
        if (eulerAngles.x() < 0) {
            eulerAngles.x() = eulerAngles.x() + pi;
        } else {
            eulerAngles.x() = eulerAngles.x() - pi;
        }

        eulerAngles.y() = -(eulerAngles.y() + pi);

        if (eulerAngles.z() < 0) {
            eulerAngles.z() = eulerAngles.z() + pi;
        } else {
            eulerAngles.z() = eulerAngles.z() - pi;
        }
    } else if (-pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= -pi / 2 + tol) {
        eulerAngles.x() -= eulerAngles.z();
        eulerAngles.z() = 0;
    } else if (-pi / 2 + tol < eulerAngles.y() && eulerAngles.y() < pi / 2 - tol) {
        // ok
    } else if (pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= pi / 2 + tol) {
        // todo: pi/2 should not be in range, other formula?
        eulerAngles.x() += eulerAngles.z();
        eulerAngles.z() = 0;
    } else  // pi/2 + tol < eulerAngles.y()
    {
        if (eulerAngles.x() < 0) {
            eulerAngles.x() = eulerAngles.x() + pi;
        } else {
            eulerAngles.x() = eulerAngles.x() - pi;
        }

        eulerAngles.y() = -(eulerAngles.y() - pi);

        if (eulerAngles.z() < 0) {
            eulerAngles.z() = eulerAngles.z() + pi;
        } else {
            eulerAngles.z() = eulerAngles.z() - pi;
        }
    }
}

/**
 * @brief Convert quaternion to a 3x3 rotation matrix
 *
 * @param q : quaternion
 * @return matrix3_t : rotation matrix
 */
inline matrix3_t quat2mat(const quaternion_t &q) {
    return q.toRotationMatrix();
}

/**
 * @brief Convert a 3x3 rotation matrix to roll-pitch-yaw euler angles
 *
 * @param R : rotation matrix
 * @return vector3_t : roll-pitch-yaw euler angles (Z rotation first, then Y, then X)
 */
inline vector3_t mat2rpy(const matrix3_t &R) {
    return pinocchio::rpy::matrixToRpy(R);
}

namespace {  // helper functions to select the right modulo
template <typename SCALAR_T>
SCALAR_T scalarMod(SCALAR_T, SCALAR_T);

template <>
float scalarMod<float>(float x, float y) {
    return fmodf(x, y);
}

template <>
double scalarMod<double>(double x, double y) {
    return fmod(x, y);
}
}  // namespace

// TODO: Add documentation
template <typename SCALAR_T>
inline SCALAR_T moduloAngleWithReference(SCALAR_T x, SCALAR_T reference) {
    const SCALAR_T ub = reference + M_PI;  // upper bound
    const SCALAR_T lb = reference - M_PI;  // lower bound

    if (x > ub) {
        x = lb + scalarMod<SCALAR_T>(x - lb, 2.0 * M_PI);
    } else if (x < lb) {
        x = ub - scalarMod<SCALAR_T>(ub - x, 2.0 * M_PI);
    }

    return x;
}

/**
 * @brief Convert a 3x3 rotation matrix to ocs2-style rpy euler angles, assume last yaw angle
 *
 * @param R : rotation matrix
 * @param lastYaw : previous yaw angle
 * @return vector3_t : roll-pitch-yaw euler angles
 */
inline vector3_t mat2oc2rpy(const matrix3_t &R, const scalar_t lastYaw) {
    // https://github.com/leggedrobotics/ocs2/blob/164c26b46bed5d24cd03d90588db8980d03a4951/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_commands/src/TerrainAdaptation.cpp#L19
    // https://github.com/lnotspotl/tbai/blob/main/tbai_core/include/tbai_core/Rotations.hpp
    vector3_t eulerXYZ = R.eulerAngles(0, 1, 2);
    makeEulerAnglesUnique<scalar_t>(eulerXYZ);
    eulerXYZ.z() = moduloAngleWithReference<scalar_t>(eulerXYZ.z(), lastYaw);
    return eulerXYZ;
}

/**
 * @brief Convert ocs2-style rpy angles to quaternion
 *
 * @param rpy : ocs2-style rpy angles
 * @return quaternion_t : quaternion
 */
inline quaternion_t ocs2rpy2quat(const vector3_t &rpy) {
    return angleaxis_t(rpy(0), vector3_t::UnitX()) * angleaxis_t(rpy(1), vector3_t::UnitY()) *
           angleaxis_t(rpy(2), vector3_t::UnitZ());
}

/**
 * @brief Convert roll-pitch-yaw euler angles to a 3x3 rotation matrix
 *
 * @param rpy : roll-pitch-yaw euler angles
 * @return matrix3_t : rotation matrix
 */
inline matrix3_t rpy2mat(const vector3_t &rpy) {
    return pinocchio::rpy::rpyToMatrix(rpy);
}

/**
 * @brief Convert a 3x3 rotation matrix to axis-angle representation
 *
 * @param R : rotation matrix
 * @return vector3_t : axis-angle representation
 */
inline vector3_t mat2aa(const matrix3_t &R) {
    tbai::angleaxis_t aa(R);
    return aa.axis() * aa.angle();
}

}  // namespace tbai