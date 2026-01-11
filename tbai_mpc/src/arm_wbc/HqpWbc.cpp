// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_mpc/arm_wbc/HqpWbc.hpp"

#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/multibody/data.hpp>

namespace tbai::mpc::arm {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<tbai::MotorCommand> HqpWbc::getMotorCommands(scalar_t currentTime, const vector_t &currentState,
                                                         const vector_t &currentInput, const vector_t &desiredState,
                                                         const vector_t &desiredInput,
                                                         const vector_t &desiredJointAcceleration,
                                                         const vector_t &desiredEEPosition,
                                                         const vector_t &desiredEEOrientation, bool &isStable) {
    // Update measured state and compute kinematics/dynamics
    updateMeasuredState(currentState, currentInput);

    // Build task hierarchy for HQP
    // Task 1 (Highest Priority): Torque limits (hard constraint)
    Task task1 = createTorqueLimitTask();

    // Task 2: End-effector position and orientation tracking
    Task task2 =
        createEndEffectorPositionTask(desiredEEPosition) + createEndEffectorOrientationTask(desiredEEOrientation);

    // Task 3: Joint acceleration tracking + joint centering
    Task task3 = createJointAccelerationTask(desiredJointAcceleration) + createJointCenteringTask();

    // Solve HQP
    std::vector<Task *> tasks = {&task1, &task2, &task3};
    vector_t hqpSolution = hqpSolver_.solveHqp(tasks, isStable);

    // Extract joint accelerations from solution
    const vector_t &qddot = hqpSolution.head(nDecisionVariables_);

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
void HqpWbc::loadSettings(const std::string &configFile) {
    using ocs2::scalar_t;
    using ocs2::loadData::loadCppDataType;

    const std::string prefix = "hqpWbc.";

    loadCppDataType<scalar_t>(configFile, prefix + "jointKp", jointKp_);
    loadCppDataType<scalar_t>(configFile, prefix + "jointKd", jointKd_);
}

}  // namespace tbai::mpc::arm
