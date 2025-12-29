// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_mpc/wbc/SqpWbc.hpp"

#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/multibody/data.hpp>

namespace tbai {
namespace mpc {

std::vector<tbai::MotorCommand> SqpWbc::getMotorCommands(scalar_t currentTime, const vector_t &currentState,
                                                         const vector_t &currentInput, const size_t currentMode,
                                                         const vector_t &desiredState, const vector_t &desiredInput,
                                                         const size_t desiredMode,
                                                         const vector_t &desiredJointAcceleration, bool &isStable) {
    // Update state information
    updateContactFlags(currentMode, desiredMode);
    updateMeasuredState(currentState, currentInput.tail<12>());
    updateDesiredState(desiredState, desiredInput);

    // QP constraints
    Task constraints =
        createDynamicsTask() + createStanceFootNoMotionTask() + createContactForceTask() + createTorqueLimitTask();

    // QP cost function
    Task weightedTasks =
        createBaseAccelerationTask(currentState, desiredState, desiredInput, desiredJointAcceleration) *
            weightBaseAcceleration_ +
        createContactForceMinimizationTask(desiredInput) * weightContactForce_ +
        createSwingFootAccelerationTask(currentState, currentInput, desiredState, desiredInput) * weightSwingLeg_;

    // solve QP
    vector_t sqpSolution = sqpSolver_.solveSqp(weightedTasks, constraints, isStable);

    // Generalized accelerations
    const vector_t &udot = sqpSolution.segment(0, nGeneralizedCoordinates_);

    // External forces, expressed in the world frame
    const vector_t &Fext = sqpSolution.segment(nGeneralizedCoordinates_, 3 * switched_model::NUM_CONTACT_POINTS);

    // Compute joint torques
    auto &data = pinocchioInterfaceMeasured_.getData();
    const matrix_t Mj = data.M.block<12, 18>(6, 0);
    const vector_t hj = data.nle.segment<12>(6);
    const matrix_t JjT = Jcontact_.block(0, 6, 12, 12).transpose();
    vector_t torques = Mj * udot + hj - JjT * Fext;

    // Desired joint positions and velocities
    const vector_t &qDesired = desiredState.tail<switched_model::JOINT_COORDINATE_SIZE>();
    const vector_t &vDesired = desiredInput.tail<switched_model::JOINT_COORDINATE_SIZE>();

    std::swap(torques(3), torques(6));
    std::swap(torques(4), torques(7));
    std::swap(torques(5), torques(8));

    // Generator command message
    std::vector<tbai::MotorCommand> commands;
    commands.resize(jointNames_.size());
    for (size_t i = 0; i < jointNames_.size(); ++i) {
        tbai::MotorCommand command;
        command.joint_name = jointNames_[i];
        command.desired_position = qDesired[i];
        command.desired_velocity = vDesired[i];
        if (!contactFlags_[i / 3]) {
            command.kp = jointSwingKp_;
            command.kd = jointSwingKd_;
        } else {
            command.kp = jointStanceKp_;
            command.kd = jointStanceKd_;
        }
        command.torque_ff = torques[i];
        commands[i] = std::move(command);
    }

    return commands;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void SqpWbc::loadSettings(const std::string &configFile) {
    using ocs2::scalar_t;
    using ocs2::loadData::loadCppDataType;

    const std::string prefix = "sqpWbc.";

    loadCppDataType<scalar_t>(configFile, prefix + "weightBaseTracking", weightBaseAcceleration_);
    loadCppDataType<scalar_t>(configFile, prefix + "weightSwingLeg", weightSwingLeg_);
    loadCppDataType<scalar_t>(configFile, prefix + "weightContactForce", weightContactForce_);

    loadCppDataType<scalar_t>(configFile, prefix + "jointSwingKp", jointSwingKp_);
    loadCppDataType<scalar_t>(configFile, prefix + "jointSwingKd", jointSwingKd_);

    loadCppDataType<scalar_t>(configFile, prefix + "jointStanceKp", jointStanceKp_);
    loadCppDataType<scalar_t>(configFile, prefix + "jointStanceKd", jointStanceKd_);
}

}  // namespace mpc
}  // namespace tbai
