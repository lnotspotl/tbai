#pragma once

#include <Eigen/Dense>
#include <tbai_mpc/wbc/Task.hpp>

namespace tbai {
namespace mpc {

using matrix_qp = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

class SqpSolver {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    vector_t solveSqp(const Task &weightedTasks, const Task &constraints, bool &isStable);
};

}  // namespace mpc
}  // namespace tbai
