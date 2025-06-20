#pragma once

#include <Eigen/Dense>

namespace tbai {

/** Scalar type */
using scalar_t = double;

/** Vector type */
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;

/** Matrix type */
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;

/** Vector with three elements */
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

/** Vector with four elements */
using vector4_t = Eigen::Matrix<scalar_t, 4, 1>;

/** 3x3 matrix */
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;

/** Angle axis */
using angleaxis_t = Eigen::AngleAxis<scalar_t>;

/** Quaternion */
using quaternion_t = Eigen::Quaternion<scalar_t>;

}  // namespace tbai