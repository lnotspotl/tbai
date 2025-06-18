#include <tbai_bob/InverseKinematics.hpp>
#include <tbai_core/config/Config.hpp>

namespace tbai {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
vector_t InverseKinematics::solve(vector_t &legHeightDiffs) {
    matrix_t positions = defaultStance_;
    positions.col(2) += legHeightDiffs;
    auto angles = solve_ik(positions);
    return angles;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
vector_t AnymalDInverseKinematics::solve_ik(matrix_t &footPositions) {
    vector_t joint_angles(12);
    for (int i = 0; i < 4; ++i) {
        const scalar_t d2 = i < 2 ? d2_ : -d2_;  // assume LF LH RF RH leg order
        const scalar_t a3 = a3_;
        const scalar_t a4 = a4_;

        const scalar_t x = footPositions(i, 0);
        const scalar_t y = footPositions(i, 1);
        const scalar_t z = footPositions(i, 2);

        const scalar_t E = y * y + z * z - d2 * d2;
        const scalar_t E_sqrt = sqrt(E);
        scalar_t theta1 = atan2(E_sqrt, d2) + atan2(z, y);

        scalar_t D = (E + x * x - a3 * a3 - a4 * a4) / (2.0 * a3 * a4);
        D = std::max(static_cast<scalar_t>(-1.0), std::min(static_cast<scalar_t>(1.0), D));
        scalar_t theta4 = -atan2(sqrt(1.0 - D * D), D);
        constexpr scalar_t theta4_offset = 0.254601;
        scalar_t theta4_final = theta4 + theta4_offset;

        if (i % 2 == 1) {  // hind legs
            theta4 *= -1.0;
            theta4_final *= -1.0;
        }

        scalar_t theta3 = atan2(-x, E_sqrt) - atan2(a4 * sin(theta4), a3 + a4 * cos(theta4));

        joint_angles.segment<3>(3 * i) = (vector3_t() << theta1, theta3, theta4_final).finished();
    }

    return joint_angles;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
vector_t Go2InverseKinematics::solve_ik(matrix_t &footPositions) {
    vector_t joint_angles(12);
    for (int i = 0; i < 4; ++i) {
        const scalar_t d2 = i < 2 ? d2_ : -d2_;  // assume LF LH RF RH leg order
        const scalar_t a3 = a3_;
        const scalar_t a4 = a4_;

        const scalar_t x = footPositions(i, 0);
        const scalar_t y = footPositions(i, 1);
        const scalar_t z = footPositions(i, 2);

        const scalar_t E = y * y + z * z - d2 * d2;
        const scalar_t E_sqrt = sqrt(E);
        scalar_t theta1 = atan2(E_sqrt, d2) + atan2(z, y);

        scalar_t D = (E + x * x - a3 * a3 - a4 * a4) / (2.0 * a3 * a4);
        D = std::max(static_cast<scalar_t>(-1.0), std::min(static_cast<scalar_t>(1.0), D));
        scalar_t theta4 = -atan2(sqrt(1.0 - D * D), D);

        scalar_t theta3 = atan2(-x, E_sqrt) - atan2(a4 * sin(theta4), a3 + a4 * cos(theta4));

        joint_angles.segment<3>(3 * i) = (vector3_t() << theta1, theta3, theta4).finished();
    }

    return joint_angles;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
std::unique_ptr<InverseKinematics> getInverseKinematicsUnique() {
    auto robotName = tbai::fromGlobalConfig<std::string>("robot_name");
    auto d2 = tbai::fromGlobalConfig<scalar_t>("bob_controller/ik/d2");
    auto a3 = tbai::fromGlobalConfig<scalar_t>("bob_controller/ik/a3");
    auto a4 = tbai::fromGlobalConfig<scalar_t>("bob_controller/ik/a4");
    auto defaultStance = tbai::fromGlobalConfig<matrix_t>("bob_controller/default_stance");

    if (robotName == "anymal_d") {
        return std::make_unique<AnymalDInverseKinematics>(d2, a3, a4, defaultStance);
    }

    if (robotName == "go2") {
        return std::make_unique<Go2InverseKinematics>(d2, a3, a4, defaultStance);
    }

    throw std::runtime_error("Inverse kinematics for robot " + robotName + " not implemented");
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
std::shared_ptr<InverseKinematics> getInverseKinematicsShared() {
    return std::shared_ptr<InverseKinematics>(getInverseKinematicsUnique().release());
}

}  // namespace tbai