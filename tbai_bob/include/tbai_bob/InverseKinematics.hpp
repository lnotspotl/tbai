#pragma once

#include <memory>

#include <tbai_core/Types.hpp>

namespace tbai {

class InverseKinematics {
   public:
    /* Solve inverse kinematics */
    vector_t solve(vector_t &legHeightDiffs);

   protected:
    /* Default foot positons expressed w.r.t hip frame */
    matrix_t defaultStance_;

   private:
    /* Solve inverse kinematics for given foot positions expressed w.r.t. hip frame*/
    virtual vector_t solve_ik(matrix_t &footPositions) = 0;
};

class AnymalDInverseKinematics : public InverseKinematics {
   public:
    AnymalDInverseKinematics(scalar_t d2, scalar_t a3, scalar_t a4, matrix_t defaultStance)
        : d2_(d2), a3_(a3), a4_(a4) {
        defaultStance_ = defaultStance;
    }

    vector_t solve_ik(matrix_t &footPositions) override;

   private:
    scalar_t d2_;
    scalar_t a3_;
    scalar_t a4_;
};

class Go2InverseKinematics : public InverseKinematics {
   public:
    Go2InverseKinematics(scalar_t d2, scalar_t a3, scalar_t a4, matrix_t defaultStance)
        : d2_(d2), a3_(a3), a4_(a4) {
        defaultStance_ = defaultStance;
    }

    vector_t solve_ik(matrix_t &footPositions) override;

   private:
    scalar_t d2_;
    scalar_t a3_;
    scalar_t a4_;
};


/**
 * @brief Get the InverseKinematics unique pointer, initialize with values from config
 *
 * @return std::unique_ptr<InverseKinematics>
 */
std::unique_ptr<InverseKinematics> getInverseKinematicsUnique();

/**
 * @brief Get the InverseKinematics shared pointer, initialize with values from config
 *
 * @return std::shared_ptr<InverseKinematics>
 */
std::shared_ptr<InverseKinematics> getInverseKinematicsShared();

}  // namespace tbai