#pragma once
#include <tbai_core/Types.hpp>

namespace tbai {
namespace mpc {

using tbai::matrix_t;
using tbai::scalar_t;
using tbai::vector_t;

/**
 * Task for a QP solver: defined as a single equality and a single inequality
 * constraint: Ax - b  = 0 Cx - d <= 0
 */
struct Task {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* Construct task with given matrices A and D and vectors b and f */
    Task(const matrix_t &A, const vector_t &b, const matrix_t &D, const vector_t &f) : A(A), b(b), D(D), f(f) {}

    /* Combine two tasks t1 and t2 into one common task */
    Task operator+(const Task &rhs) const;

    /* Multiplication by a scalar from the right */
    Task operator*(const scalar_t rhs) const;

    /* Multiplication by a scalar from the left */
    friend Task operator*(const scalar_t lhs, const Task &rhs) { return rhs * lhs; }

    /* Print out task information */
    friend std::ostream &operator<<(std::ostream &os, const Task &t);

    /* Equality constraint, Ax - b = 0 */
    matrix_t A;
    vector_t b;

    /* Inequality constraint Dx - f <= 0 */
    matrix_t D;
    vector_t f;
};

/* Stack two matrices vertically */
matrix_t mvstack(const matrix_t &m1, const matrix_t &m2);

/* Stack two vectors vertically */
vector_t vvstack(const vector_t &v1, const vector_t &v2);

}  // namespace mpc
}  // namespace tbai
