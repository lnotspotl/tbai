#include "tbai_mpc/quadruped_arm_mpc/quadruped_models/SpotInverseKinematics.h"

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <tbai_mpc/quadruped_arm_mpc/analytical_inverse_kinematics/AnalyticalInverseKinematics.h>

namespace tbai::mpc::quadruped_arm {

SpotInverseKinematics::SpotInverseKinematics(const FrameDeclaration &frameDeclaration,
                                             const ocs2::PinocchioInterface &pinocchioInterface) {
    auto data = pinocchioInterface.getData();
    const auto &model = pinocchioInterface.getModel();

    tbai::mpc::quadruped_arm::joint_coordinate_t zeroConfiguration(tbai::mpc::quadruped_arm::joint_coordinate_t::Zero());
    pinocchio::forwardKinematics(model, data, zeroConfiguration);
    pinocchio::updateFramePlacements(model, data);

    for (size_t leg = 0; leg < tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS; ++leg) {
        if (frameDeclaration.legs[leg].joints.size() != 3) {
            throw std::runtime_error(
                "[SpotInverseKinematics] analytical inverse kinematics only valid for 3 joints per leg");
        }

        const auto &hipTransform =
            data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].joints[0], pinocchioInterface)];
        const auto &thighTransform =
            data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].joints[1], pinocchioInterface)];
        const auto &shankTransform =
            data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].joints[2], pinocchioInterface)];
        const auto &footTransform =
            data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].tip, pinocchioInterface)];

        parameters_[leg] = tbai::mpc::quadruped_arm::analytical_inverse_kinematics::LegInverseKinematicParameters(
            hipTransform.translation(), thighTransform.translation() - hipTransform.translation(),
            shankTransform.translation() - thighTransform.translation(),
            footTransform.translation() - shankTransform.translation());
    }
}

SpotInverseKinematics *SpotInverseKinematics::clone() const {
    return new SpotInverseKinematics(*this);
}

tbai::mpc::quadruped_arm::vector3_t SpotInverseKinematics::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
    size_t footIndex, const tbai::mpc::quadruped_arm::vector3_t &positionBaseToFootInBaseFrame) const {
    tbai::mpc::quadruped_arm::vector3_t jointAngles{tbai::mpc::quadruped_arm::vector3_t::Zero()};
    tbai::mpc::quadruped_arm::analytical_inverse_kinematics::spot::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
        jointAngles, positionBaseToFootInBaseFrame, parameters_[footIndex], footIndex);
    return jointAngles;
}

tbai::mpc::quadruped_arm::vector3_t SpotInverseKinematics::getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(
    size_t footIndex, const tbai::mpc::quadruped_arm::vector3_t &footVelocityRelativeToBaseInBaseFrame,
    const joint_jacobian_block_t &jointJacobian, tbai::mpc::quadruped_arm::scalar_t damping) const {
    // v = J * dq, (bottom 3 rows = translational part)
    tbai::mpc::quadruped_arm::matrix3_t Jtranslational = jointJacobian.block<3, 3>(3, 0);
    tbai::mpc::quadruped_arm::matrix3_t JTJ = Jtranslational.transpose() * Jtranslational;
    JTJ.diagonal().array() += damping;  // regularize

    return JTJ.ldlt().solve(Jtranslational.transpose() * footVelocityRelativeToBaseInBaseFrame);
}

}  // namespace tbai::mpc::quadruped_arm
