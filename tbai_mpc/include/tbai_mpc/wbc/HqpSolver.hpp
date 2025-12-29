#pragma once

#include <Eigen/Dense>
#include <tbai_mpc/wbc/Task.hpp>

namespace tbai {
namespace mpc {

using matrix_qp = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/**
 * Note: the used naming convention is the same as in the paper
 *    "Perception-less terrain adaptation through whole body control and hierarchical optimization"
 *    https://ieeexplore.ieee.org/abstract/document/7803330
 */
class HqpSolver {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    HqpSolver() = default;

    /** Solve given Hierarchical Quadratic Program */
    vector_t solveHqp(std::vector<Task *> &tasks, bool &isStable);  // NOLINT

   private:
    /** Number of slack variables for inequality constraints */
    size_t nSlackVariables_;

    /** Number of actual decision variables */
    size_t nDecisionVariables_;

    /** Current task has equality constraints */
    bool hasEqualityConstraints_;

    /** Current task has inequality constraints */
    bool hasInequalityConstraints_;

    /** Current task is the first task in the hierarchy */
    bool firstTask_;

    /** Pointer to current task */
    Task *currentTaskPtr_;

    /** Higher priority task nullspace matrix */
    matrix_t Zp;

    /** H_{p+1]} matrix */
    matrix_qp Hpp1;

    /** c_{p+1} vector */
    vector_t cpp1;

    /** ~D_{p+1} matrix */
    matrix_qp hDpp1;

    /** ~f_{p+1} vector */
    vector_t hfpp1;

    /** D_{acc} accumulation matrix of tasks matrices D*/
    matrix_t Dacc;

    /** f_{acc} accumulation vector of task vectors f */
    vector_t facc;

    /** v_{acc} accumulation vector of inequality constraint slack variable
     * values */
    vector_t vacc;

    /** Intermediate solutions of QPs */
    vector_t solution;

    /** Final solution of the HQP */
    vector_t xStar_;

    /** Frequently used matrices */
    matrix_t identityNs_;
    matrix_t zeroNsNd_;

    /** Setup variables for the upcoming QP */
    void initializeVariables();

    /** Generate QP matrices and vectors */
    void generate_H_matrix();
    void generate_c_vector();
    void generate_D_matrix();
    void generate_f_vector();

    /** Solve current QP */
    void solveQp(bool &isStable);

    /** Update accumulation variables and Zp nullspace matrix */
    void updateSolver();

    /** Update HQP solution after solving another QP */
    inline void updateXstar() { xStar_ += Zp * solution.head(nDecisionVariables_); }
};

}  // namespace mpc
}  // namespace tbai
