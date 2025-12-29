#include "tbai_mpc/wbc/HqpSolver.hpp"

#include <qpOASES.hpp>

namespace tbai {
namespace mpc {
vector_t HqpSolver::solveHqp(std::vector<Task *> &tasks, bool &isStable) {
    firstTask_ = true;

    const size_t lastTaskIndex = tasks.size() - 1;
    for (size_t i = 0; i <= lastTaskIndex; ++i) {
        // Pointer to current task
        currentTaskPtr_ = tasks[i];

        // Initialize variables for the upcoming QP problem
        initializeVariables();

        // Generate QP input matrices and vectors
        generate_H_matrix();
        generate_c_vector();
        generate_D_matrix();
        generate_f_vector();

        // Solve QP using the with the generated inputs above
        solveQp(isStable);

        if (!isStable) {
            return vector_t::Zero(nDecisionVariables_);
        }

        // Update optimal solution
        updateXstar();

        // No need to update the solver when we've solved the last task
        if (i != lastTaskIndex) {
            updateSolver();
        }
    }

    return xStar_;
}

void HqpSolver::initializeVariables() {
    /* Determine number of slack decision variables for the current task */
    nSlackVariables_ = currentTaskPtr_->D.rows();

    /* Determine, whether there are equality constraints */
    hasEqualityConstraints_ = (currentTaskPtr_->A.rows() > 0);

    /* Determine, whether there are inequality constraints */
    hasInequalityConstraints_ = (nSlackVariables_ > 0);

    /* Set things up if this is the first task */
    if (firstTask_) {
        /* Determine number of actual decision variables for the first task */
        nDecisionVariables_ = std::max(currentTaskPtr_->A.cols(), currentTaskPtr_->D.cols());

        xStar_ = vector_t::Zero(nDecisionVariables_);
        Zp = matrix_t::Identity(nDecisionVariables_, nDecisionVariables_);
        Dacc = matrix_t(0, nDecisionVariables_);
        facc = vector_t(0);
        vacc = vector_t(0);
        firstTask_ = false;
    } else {
        // Determine the number of decision variables for the current task
        // in the nullspace of the previous tasks
        nDecisionVariables_ = Zp.cols();
    }

    /* Prepare commonly used matrices for use */
    identityNs_ = matrix_t::Identity(nSlackVariables_, nSlackVariables_);
    zeroNsNd_ = matrix_t::Zero(nSlackVariables_, nDecisionVariables_);
}

void HqpSolver::generate_H_matrix() {
    const size_t H_size = nDecisionVariables_ + nSlackVariables_;
    matrix_t Hpp1(H_size, H_size);

    /* Compute H_{p+1,tl} */
    matrix_t Hpp1tl(nDecisionVariables_, nDecisionVariables_);

    if (hasEqualityConstraints_) {
        matrix_t temp = currentTaskPtr_->A * Zp;
        Hpp1tl = temp.transpose() * temp;
        // To prevent numerical issues, add small number to the diagonal
        // (We need the eigen-values to be >= 0 at all times)
        // Reference:
        // https://github.com/bernhardpg/quadruped_locomotion/blob/main/src/control/ho_qp/ho_qp_problem.cpp
        Hpp1tl += matrix_t::Identity(nDecisionVariables_, nDecisionVariables_) * 1e-12;
    } else {
        Hpp1tl.setZero();
    }

    /**
     * Populate Hpp1 with corresponding values
     *
     *             ( Hpp1tl | 0 )
     *      Hpp1 = (--------|---)
     *             (    0   | I )
     */

    Hpp1 << Hpp1tl, zeroNsNd_.transpose(), zeroNsNd_, identityNs_;

    /* Store generated matrix in the object variable */
    this->Hpp1 = Hpp1;
}

void HqpSolver::generate_c_vector() {
    const size_t c_size = nDecisionVariables_ + nSlackVariables_;
    vector_t cpp1(c_size);
    vector_t zero_bottom = vector_t::Zero(nSlackVariables_);

    /* Compute the upper part of the c_{p+1} vector */
    vector_t cpp1u(nDecisionVariables_);
    if (hasEqualityConstraints_) {
        cpp1u = (currentTaskPtr_->A * Zp).transpose() * (currentTaskPtr_->A * xStar_ - currentTaskPtr_->b);
    } else {
        cpp1u.setZero();
    }

    /**
     * Populate cpp1 with corresponding values
     *
     *             ( cpp1u )
     *      cpp1 = (-------)
     *             (   0   )
     */
    cpp1 << cpp1u, zero_bottom;
    this->cpp1 = cpp1;
}

void HqpSolver::generate_D_matrix() {
    const size_t D_rows = 2 * nSlackVariables_ + Dacc.rows();
    const size_t D_cols = nDecisionVariables_ + nSlackVariables_;
    matrix_t hDpp1(D_rows, D_cols);

    // Preallocate zero matrix on the right of the ~D_{p+1} matrix
    matrix_t zero_right = matrix_t::Zero(Dacc.rows(), nSlackVariables_);

    matrix_t DaccZp = Dacc * Zp;

    // Compute matrix D_{p+1} * Zp
    matrix_t Dpp1Zp;
    if (hasInequalityConstraints_) {
        Dpp1Zp = currentTaskPtr_->D * Zp;
    } else {
        Dpp1Zp = matrix_t::Zero(0, nDecisionVariables_);
    }

    /**
     * Populate hDpp1 with corresponding values
     *
     *               ( Dpp1Zp | -I )
     *               (--------|----)
     *       hDpp1 = ( DaccZp |  0 )
     *               (--------|----)
     *               (   0    | -I )
     */
    hDpp1 << Dpp1Zp, -identityNs_, DaccZp, zero_right, zeroNsNd_, -identityNs_;

    this->hDpp1 = hDpp1;
}

void HqpSolver::generate_f_vector() {
    // ~f_{p+1} vector
    const size_t hf_size = 2 * nSlackVariables_ + Dacc.rows();
    /* Preallocate vector ~f_{p+1} on the stack */
    vector_t hfpp1(hf_size);

    vector_t zero_bottom = vector_t::Zero(nSlackVariables_);

    vector_t fdxv = facc - Dacc * xStar_ + vacc;

    vector_t fmdx;
    if (hasInequalityConstraints_) {
        fmdx = currentTaskPtr_->f - currentTaskPtr_->D * xStar_;
    } else {
        fmdx = vector_t::Zero(0);
    }

    /**
     * Populate hfpp1 with corresponding values
     *
     *              ( fmdx )
     *              (------)
     *      hfpp1 = ( fdxv )
     *              (------)
     *              (  00  )
     */
    hfpp1 << fmdx, fdxv, zero_bottom;

    /* Store generated matrix in the object variable */
    this->hfpp1 = hfpp1;
}

void HqpSolver::solveQp(bool &isStable) {
    const size_t nQpDecisionVariables = nDecisionVariables_ + nSlackVariables_;
    const size_t nQpConstraints = hfpp1.size();

    // Create QP problem
    qpOASES::QProblem qpProblem(nQpDecisionVariables, nQpConstraints);

    // Set QP options
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    qpProblem.setOptions(options);

    // Max number of recalculations
    int nWSR = 50;

    // Initialize QP problem
    qpOASES::returnValue qp_status =
        qpProblem.init(Hpp1.data(), cpp1.data(), hDpp1.data(), nullptr, nullptr, nullptr, hfpp1.data(), nWSR);

    if (qp_status != qpOASES::SUCCESSFUL_RETURN) {
        isStable = false;
        solution = vector_t::Zero(nQpDecisionVariables);
        return;
    }

    // Allocate memory for solution
    solution = vector_t(nQpDecisionVariables);

    // Solve QP and store the solution in 'solution'
    qpOASES::returnValue sol_status = qpProblem.getPrimalSolution(solution.data());

    if (sol_status != qpOASES::SUCCESSFUL_RETURN) {
        isStable = false;
        solution = vector_t::Zero(nQpDecisionVariables);
        return;
    }

    // Check stability (errors) and write to isStable
    isStable = (qp_status == qpOASES::SUCCESSFUL_RETURN) && (sol_status == qpOASES::SUCCESSFUL_RETURN);
}

void HqpSolver::updateSolver() {
    /* Update accumulation vectors and matrices */
    Dacc = mvstack(Dacc, currentTaskPtr_->D);
    facc = vvstack(facc, currentTaskPtr_->f);
    vacc = vvstack(vacc, solution.tail(nSlackVariables_));

    /* Update Zp */
    if (hasEqualityConstraints_) {
        Zp = Zp * (currentTaskPtr_->A * Zp).fullPivLu().kernel();
    }
}

}  // namespace mpc
}  // namespace tbai
