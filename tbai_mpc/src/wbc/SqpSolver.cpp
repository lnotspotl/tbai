#include "tbai_mpc/wbc/SqpSolver.hpp"

#include <qpOASES.hpp>

namespace tbai {
namespace mpc {

vector_t SqpSolver::solveSqp(const Task &weightedTasks, const Task &constraints, bool &isStable) {
    // Number of decision variables
    const size_t nDecisionVariables = weightedTasks.A.cols();

    // Number of constraints
    const size_t nConstraints = constraints.A.rows() + constraints.D.rows();

    // Setup constraint lbA <= A*x <= ubA
    matrix_qp A = (matrix_qp(nConstraints, nDecisionVariables) << constraints.A, constraints.D).finished();
    vector_t lbA =
        (vector_t(nConstraints) << constraints.b, -qpOASES::INFTY * vector_t::Ones(constraints.f.size())).finished();
    vector_t ubA = (vector_t(nConstraints) << constraints.b, constraints.f).finished();

    // x.T @ H @ x + x.T @ g <---> (Ax - b).T @ (Ax - b)
    matrix_t weightedAT = weightedTasks.A.transpose();
    matrix_qp H = weightedAT * weightedTasks.A;
    vector_t g = -weightedAT * weightedTasks.b;

    // Setup QPOASES problem
    qpOASES::QProblem qp_problem(nDecisionVariables, nConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    qp_problem.setOptions(options);
    int nWsr = 20;
    qpOASES::returnValue qp_status =
        qp_problem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);

    // Check if QP initialization was successful
    if (qp_status != qpOASES::SUCCESSFUL_RETURN) {
        isStable = false;
        return vector_t::Zero(nDecisionVariables);
    }

    // Solve QP problem
    vector_t solution(nDecisionVariables);
    qpOASES::returnValue sol_status = qp_problem.getPrimalSolution(solution.data());

    // Check if solution was successful
    if (sol_status != qpOASES::SUCCESSFUL_RETURN) {
        isStable = false;
        return vector_t::Zero(nDecisionVariables);
    }

    isStable = true;
    return solution;
}

}  // namespace mpc
}  // namespace tbai
