// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_mpc/quadruped_arm_wbc/HqpWbc.hpp"

#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/multibody/data.hpp>

namespace tbai {
namespace mpc {

std::vector<tbai::MotorCommand> HqpWbc::getMotorCommands(scalar_t currentTime, const vector_t &currentState,
                                                         const vector_t &currentInput, const size_t currentMode,
                                                         const vector_t &desiredState, const vector_t &desiredInput,
                                                         const size_t desiredMode,
                                                         const vector_t &desiredJointAcceleration,
                                                         const vector_t &desiredArmEEPosition,
                                                         const vector_t &desiredArmEEOrientation, bool &isStable) {
    constexpr size_t numLegJoints = tbai::mpc::quadruped_arm::LEG_JOINT_COORDINATE_SIZE;  // 12
    constexpr size_t numArmJoints = tbai::mpc::quadruped_arm::NUM_ARM_JOINTS;             // 6

    // Update state information
    updateContactFlags(currentMode, desiredMode);
    updateMeasuredState(currentState, currentInput);
    updateDesiredState(desiredState, desiredInput);

    // HQP task hierarchy with arm:
    // Priority 1: Dynamics + contact constraints + leg torque limits + arm torque limits
    // Priority 2: Swing foot tracking (if any swing legs)
    // Priority 3: Base tracking + Arm EE tracking
    // Priority 4: Arm joint centering (nullspace)
    // Priority 5: Contact force minimization

    vector_t hqpSolution;
    if (nContacts_ == 4) {
        // All feet in contact: no swing leg task
        Task task1 = createDynamicsTask() + createStanceFootNoMotionTask() + createContactForceTask() +
                     createTorqueLimitTask() + createArmTorqueLimitTask();
        Task task2 = createBaseAccelerationTask(currentState, desiredState, desiredInput, desiredJointAcceleration) +
                     createArmEndEffectorPositionTask(desiredArmEEPosition) +
                     createArmEndEffectorOrientationTask(desiredArmEEOrientation);
        Task task3 = createArmJointCenteringTask();
        Task task4 = createContactForceMinimizationTask(desiredInput);
        std::vector<Task *> tasks = {&task1, &task2, &task3, &task4};
        hqpSolution = hqpSolver_.solveHqp(tasks, isStable);
    } else {
        // Some feet in swing: include swing foot task
        Task task1 = createDynamicsTask() + createStanceFootNoMotionTask() + createContactForceTask() +
                     createTorqueLimitTask() + createArmTorqueLimitTask();
        Task task2 = createSwingFootAccelerationTask(currentState, currentInput, desiredState, desiredInput);
        Task task3 = createBaseAccelerationTask(currentState, desiredState, desiredInput, desiredJointAcceleration) +
                     createArmEndEffectorPositionTask(desiredArmEEPosition) +
                     createArmEndEffectorOrientationTask(desiredArmEEOrientation);
        Task task4 = createArmJointCenteringTask();
        Task task5 = createContactForceMinimizationTask(desiredInput);
        std::vector<Task *> tasks = {&task1, &task2, &task3, &task4, &task5};
        hqpSolution = hqpSolver_.solveHqp(tasks, isStable);
    }

    // Generalized accelerations (24D: 6 base + 12 leg + 6 arm)
    const vector_t &udot = hqpSolution.segment(0, nGeneralizedCoordinates_);

    // External forces, expressed in the world frame (12D: 4 feet x 3)
    const vector_t &Fext = hqpSolution.segment(nGeneralizedCoordinates_, 3 * tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS);

    // Compute leg joint torques
    auto &data = pinocchioInterfaceMeasured_.getData();
    const matrix_t Mj_leg = data.M.block(6, 0, numLegJoints, nGeneralizedCoordinates_);
    const vector_t hj_leg = data.nle.segment(6, numLegJoints);
    const matrix_t JjT_leg = Jcontact_.block(0, 6, 3 * tbai::mpc::quadruped_arm::NUM_CONTACT_POINTS, numLegJoints).transpose();
    vector_t legTorques = Mj_leg * udot + hj_leg - JjT_leg * Fext;

    // Compute arm joint torques
    const matrix_t Mj_arm = data.M.block(6 + numLegJoints, 0, numArmJoints, nGeneralizedCoordinates_);
    const vector_t hj_arm = data.nle.segment(6 + numLegJoints, numArmJoints);
    vector_t armTorques = Mj_arm * udot + hj_arm;

    // Desired joint positions and velocities (18D: 12 leg + 6 arm)
    const vector_t &qDesired = desiredState.tail(tbai::mpc::quadruped_arm::JOINT_COORDINATE_SIZE);
    const vector_t &vDesired = desiredInput.tail(tbai::mpc::quadruped_arm::JOINT_COORDINATE_SIZE);

    // Swap leg torques for correct ordering (LF, RF, LH, RH)
    std::swap(legTorques(3), legTorques(6));
    std::swap(legTorques(4), legTorques(7));
    std::swap(legTorques(5), legTorques(8));

    // Generate command messages for all 18 joints (12 leg + 6 arm)
    std::vector<tbai::MotorCommand> commands;
    commands.resize(jointNames_.size());

    // Leg joint commands (12 joints)
    for (size_t i = 0; i < numLegJoints; ++i) {
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
        command.torque_ff = legTorques[i];
        commands[i] = std::move(command);
    }

    // Arm joint commands (6 joints)
    for (size_t i = 0; i < numArmJoints; ++i) {
        tbai::MotorCommand command;
        command.joint_name = jointNames_[numLegJoints + i];
        command.desired_position = qDesired[numLegJoints + i];
        command.desired_velocity = vDesired[numLegJoints + i];
        command.kp = armJointKp_;
        command.kd = armJointKd_;
        command.torque_ff = armTorques[i];
        commands[numLegJoints + i] = std::move(command);
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

    loadCppDataType<scalar_t>(configFile, prefix + "jointSwingKp", jointSwingKp_);
    loadCppDataType<scalar_t>(configFile, prefix + "jointSwingKd", jointSwingKd_);

    loadCppDataType<scalar_t>(configFile, prefix + "jointStanceKp", jointStanceKp_);
    loadCppDataType<scalar_t>(configFile, prefix + "jointStanceKd", jointStanceKd_);
}

}  // namespace mpc
}  // namespace tbai
