#include "tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedInverseKinematics.h"

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <tbai_mpc/quadruped_mpc/analytical_inverse_kinematics/AnalyticalInverseKinematics.h>

namespace tbai::mpc::quadruped {

QuadrupedInverseKinematics::QuadrupedInverseKinematics(const FrameDeclaration &frameDeclaration,
                                                       const ocs2::PinocchioInterface &pinocchioInterface) {
    auto data = pinocchioInterface.getData();
    const auto &model = pinocchioInterface.getModel();

    tbai::mpc::quadruped::joint_coordinate_t zeroConfiguration(tbai::mpc::quadruped::joint_coordinate_t::Zero());
    pinocchio::forwardKinematics(model, data, zeroConfiguration);
    pinocchio::updateFramePlacements(model, data);

    for (size_t leg = 0; leg < tbai::mpc::quadruped::NUM_CONTACT_POINTS; ++leg) {
        if (frameDeclaration.legs[leg].joints.size() != 3) {
            throw std::runtime_error(
                "[QuadrupedInverseKinematics] analytical inverse kinematics only valid for 3 joints per leg");
        }

        const auto &hipTransform =
            data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].joints[0], pinocchioInterface)];
        const auto &thighTransform =
            data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].joints[1], pinocchioInterface)];
        const auto &shankTransform =
            data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].joints[2], pinocchioInterface)];
        const auto &footTransform =
            data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].tip, pinocchioInterface)];

        parameters_[leg] = tbai::mpc::quadruped::analytical_inverse_kinematics::LegInverseKinematicParameters(
            hipTransform.translation(), thighTransform.translation() - hipTransform.translation(),
            shankTransform.translation() - thighTransform.translation(),
            footTransform.translation() - shankTransform.translation());
    }
}

QuadrupedInverseKinematics *QuadrupedInverseKinematics::clone() const {
    return new QuadrupedInverseKinematics(*this);
}

tbai::mpc::quadruped::vector3_t QuadrupedInverseKinematics::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
    size_t footIndex, const tbai::mpc::quadruped::vector3_t &positionBaseToFootInBaseFrame) const {
    tbai::mpc::quadruped::vector3_t jointAngles{tbai::mpc::quadruped::vector3_t::Zero()};
    tbai::mpc::quadruped::analytical_inverse_kinematics::tbai::mpc::quadruped::
        getLimbJointPositionsFromPositionBaseToFootInBaseFrame(jointAngles, positionBaseToFootInBaseFrame,
                                                               parameters_[footIndex], footIndex);
    return jointAngles;
}

tbai::mpc::quadruped::vector3_t QuadrupedInverseKinematics::getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(
    size_t footIndex, const tbai::mpc::quadruped::vector3_t &footVelocityRelativeToBaseInBaseFrame,
    const joint_jacobian_block_t &jointJacobian, tbai::mpc::quadruped::scalar_t damping) const {
    // v = J * dq, (bottom 3 rows = translational part)
    tbai::mpc::quadruped::matrix3_t Jtranslational = jointJacobian.block<3, 3>(3, 0);
    tbai::mpc::quadruped::matrix3_t JTJ = Jtranslational.transpose() * Jtranslational;
    JTJ.diagonal().array() += damping;  // regularize

    return JTJ.ldlt().solve(Jtranslational.transpose() * footVelocityRelativeToBaseInBaseFrame);
}

}  // namespace tbai::mpc::quadruped