// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_mpc/franka_wbc/SqpWbc.hpp"

#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/multibody/data.hpp>

namespace tbai {
namespace mpc {
namespace franka {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<tbai::MotorCommand> SqpWbc::getMotorCommands(scalar_t currentTime, const vector_t &currentState,
                                                          const vector_t &currentInput, const vector_t &desiredState,
                                                          const vector_t &desiredInput,
                                                          const vector_t &desiredJointAcceleration,
                                                          const vector_t &desiredEEPosition,
                                                          const vector_t &desiredEEOrientation, bool &isStable) {
    // Update measured state and compute kinematics/dynamics
    updateMeasuredState(currentState, currentInput);

    // Build constraints (torque limits)
    Task constraints = createTorqueLimitTask();

    // Build weighted cost function
    Task weightedTasks =
        createEndEffectorPositionTask(desiredEEPosition) * weightEEPosition_ +
        createEndEffectorOrientationTask(desiredEEOrientation) * weightEEOrientation_ +
        createJointAccelerationTask(desiredJointAcceleration) * weightJointAcceleration_ +
        createJointCenteringTask() * weightJointCentering_;

    // Solve SQP
    vector_t sqpSolution = sqpSolver_.solveSqp(weightedTasks, constraints, isStable);

    // Extract joint accelerations from solution
    const vector_t &qddot = sqpSolution.head(nDecisionVariables_);

    // Compute joint torques: tau = M * qddot + h
    auto &data = pinocchioInterface_.getData();
    const matrix_t &M = data.M;
    const vector_t &h = data.nle;
    vector_t torques = M * qddot + h;

    // Generate motor commands
    std::vector<tbai::MotorCommand> commands;
    commands.resize(nJoints_);

    for (size_t i = 0; i < nJoints_; ++i) {
        tbai::MotorCommand command;
        command.joint_name = jointNames_[i];
        command.desired_position = desiredState(i);
        command.desired_velocity = desiredInput(i);
        command.kp = jointKp_;
        command.kd = jointKd_;
        command.torque_ff = torques(i);
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

    loadCppDataType<scalar_t>(configFile, prefix + "weightEEPosition", weightEEPosition_);
    loadCppDataType<scalar_t>(configFile, prefix + "weightEEOrientation", weightEEOrientation_);
    loadCppDataType<scalar_t>(configFile, prefix + "weightJointAcceleration", weightJointAcceleration_);

    // Joint centering weight (with default)
    weightJointCentering_ = 0.1;
    try {
        loadCppDataType<scalar_t>(configFile, prefix + "weightJointCentering", weightJointCentering_);
    } catch (...) {}

    loadCppDataType<scalar_t>(configFile, prefix + "jointKp", jointKp_);
    loadCppDataType<scalar_t>(configFile, prefix + "jointKd", jointKd_);
}

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
