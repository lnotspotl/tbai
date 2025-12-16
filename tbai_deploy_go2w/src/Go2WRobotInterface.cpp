#include "tbai_deploy_go2w/Go2WRobotInterface.hpp"

#include <stdint.h>

#include <chrono>
#include <string>
#include <thread>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/Rate.hpp>

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
static uint32_t crc32_core(uint32_t *ptr, uint32_t len) {
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }

            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

namespace tbai {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Go2WRobotInterface::Go2WRobotInterface(Go2WRobotInterfaceArgs args) {
    logger_ = tbai::getLogger("tbai_deploy_go2w");
    TBAI_LOG_INFO(logger_, "Go2WRobotInterface constructor");
    TBAI_LOG_INFO(logger_, "Network interface: {}", args.networkInterface());
    TBAI_LOG_INFO(logger_, "Initializing Go2WRobotInterface");
    TBAI_LOG_INFO(logger_, "Unitree channel: {}", args.unitreeChannel());
    TBAI_LOG_INFO(logger_, "Channel init: {}", args.channelInit());

    if (args.channelInit()) {
        TBAI_LOG_INFO(logger_, "Initializing channel factory: {}", args.networkInterface());
        unitree::robot::ChannelFactory::Instance()->Init(args.unitreeChannel(), args.networkInterface());
    } else {
        throw std::runtime_error("Channel init is disabled");
    }

    // Initialize motor 2 id map (16 DOF for Go2W)
    // Real robot order: FR leg (0-2), FL leg (3-5), RR leg (6-8), RL leg (9-11), wheels (12-15)
    // Joint names match URDF naming convention
    motor_id_map["FR_hip_joint"] = 0;
    motor_id_map["FR_thigh_joint"] = 1;
    motor_id_map["FR_calf_joint"] = 2;
    motor_id_map["FL_hip_joint"] = 3;
    motor_id_map["FL_thigh_joint"] = 4;
    motor_id_map["FL_calf_joint"] = 5;
    motor_id_map["RR_hip_joint"] = 6;
    motor_id_map["RR_thigh_joint"] = 7;
    motor_id_map["RR_calf_joint"] = 8;
    motor_id_map["RL_hip_joint"] = 9;
    motor_id_map["RL_thigh_joint"] = 10;
    motor_id_map["RL_calf_joint"] = 11;
    motor_id_map["FR_foot_joint"] = 12;
    motor_id_map["FL_foot_joint"] = 13;
    motor_id_map["RR_foot_joint"] = 14;
    motor_id_map["RL_foot_joint"] = 15;

    TBAI_LOG_INFO(logger_, "Initializing publisher: Topic: {}", GO2W_TOPIC_LOWCMD);
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(GO2W_TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    // Initialize low_cmd with proper header and flags (required for Go2W communication)
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    // Initialize all motor commands to safe default values
    constexpr float PosStopF = 2.146e9f;
    constexpr float VelStopF = 16000.0f;
    for (int i = 0; i < 20; ++i) {  // Go2 has 20 motor slots
        low_cmd.motor_cmd()[i].mode() = 0x01;
        low_cmd.motor_cmd()[i].q() = PosStopF;
        low_cmd.motor_cmd()[i].dq() = VelStopF;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }

    TBAI_LOG_INFO(logger_, "Initializing subscriber - Topic: {}", GO2W_TOPIC_LOWSTATE);
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(GO2W_TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Go2WRobotInterface::lowStateCallback, this, std::placeholders::_1), 1);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Go2WRobotInterface::~Go2WRobotInterface() {
    TBAI_LOG_INFO(logger_, "Destroying Go2WRobotInterface");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void Go2WRobotInterface::lowStateCallback(const void *message) {
    auto t11 = std::chrono::high_resolution_clock::now();
    scalar_t currentTime = tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow();

    // Create a copy of the low level state
    unitree_go::msg::dds_::LowState_ &low_state = *(unitree_go::msg::dds_::LowState_ *)message;

    // Calculate callback rate
    static auto last_time2 = currentTime;
    static int count = 0;
    count++;

    constexpr int N = 200;
    if (count % N == 0) {
        scalar_t time_diff = currentTime - last_time2;
        double rate = N / time_diff;
        TBAI_LOG_INFO_THROTTLE(logger_, 8.0, "Low state callback rate: {} Hz (count: {})", rate, count);
        last_time2 = currentTime;
    }

    // Extract joint positions and velocities from low_state (16 DOF)
    vector_t jointAngles(go2w::GO2W_NUM_JOINTS);
    vector_t jointVelocities(go2w::GO2W_NUM_JOINTS);

    // Map joints in real robot order to state vector
    // Real order: FR(0-2), FL(3-5), RR(6-8), RL(9-11), wheels(12-15)
    for (int i = 0; i < go2w::GO2W_NUM_JOINTS; ++i) {
        jointAngles[i] = low_state.motor_state()[i].q();
        jointVelocities[i] = low_state.motor_state()[i].dq();
    }

    // Extract IMU data
    vector4_t baseOrientation;
    baseOrientation[0] = low_state.imu_state().quaternion()[1];  // x
    baseOrientation[1] = low_state.imu_state().quaternion()[2];  // y
    baseOrientation[2] = low_state.imu_state().quaternion()[3];  // z
    baseOrientation[3] = low_state.imu_state().quaternion()[0];  // w

    vector3_t baseAngVel;
    baseAngVel[0] = low_state.imu_state().gyroscope()[0];
    baseAngVel[1] = low_state.imu_state().gyroscope()[1];
    baseAngVel[2] = low_state.imu_state().gyroscope()[2];

    // Contact states (for wheeled robot, using foot force sensors on wheels)
    std::vector<bool> contactFlags(4, true);  // Assume wheels are always in contact

    // Build state vector
    State state;
    state.x = vector_t::Zero(go2w::GO2W_STATE_DIM);

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    const quaternion_t baseQuaternion(baseOrientation);
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    state.x.segment<3>(0) = rpy;

    // Base position (not available without state estimator, set to zero)
    // TODO(lnotspotl): Add state estimator
    state.x.segment<3>(3).setZero();

    // Base angular velocity (in body frame)
    state.x.segment<3>(6) = baseAngVel;

    // Base linear velocity (not available without state estimator, set to zero)
    // TODO(lnotspotl): Add state estimator
    state.x.segment<3>(9).setZero();

    // Joint positions (16 DOF)
    state.x.segment(12, go2w::GO2W_NUM_JOINTS) = jointAngles;

    // Joint velocities (16 DOF)
    state.x.segment(12 + go2w::GO2W_NUM_JOINTS, go2w::GO2W_NUM_JOINTS) = jointVelocities;

    state.timestamp = currentTime;
    // TODO(lnotspotl): Add state estimator
    state.contactFlags = contactFlags;

    // Update the latest state
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    state_ = std::move(state);

    initialized = true;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void Go2WRobotInterface::publish(std::vector<MotorCommand> commands) {
    static auto last_publish_time = std::chrono::high_resolution_clock::now();
    static int publish_count = 0;
    publish_count++;

    constexpr int PUBLISH_N = 100;
    if (publish_count % PUBLISH_N == 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_diff =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_publish_time).count();
        double rate = (PUBLISH_N * 1000.0) / time_diff;
        TBAI_LOG_INFO(logger_, "Publish frequency: {} Hz (count: {})", rate, publish_count);
        last_publish_time = current_time;
    }

    for (const auto &command : commands) {
        int motor_id = motor_id_map[command.joint_name];
        low_cmd.motor_cmd()[motor_id].mode() = (0x01);  // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[motor_id].q() = (command.desired_position);
        low_cmd.motor_cmd()[motor_id].kp() = command.kp;
        low_cmd.motor_cmd()[motor_id].dq() = (command.desired_velocity);
        low_cmd.motor_cmd()[motor_id].kd() = command.kd;
        low_cmd.motor_cmd()[motor_id].tau() = command.torque_ff;
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);

    // Publish the low level command
    lowcmd_publisher->Write(low_cmd);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void Go2WRobotInterface::waitTillInitialized() {
    TBAI_LOG_INFO(logger_, "Waiting for the robot to initialize...");
    while (!initialized) {
        TBAI_LOG_INFO_THROTTLE(logger_, 1.0, "Waiting for the robot to initialize...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    TBAI_LOG_INFO(logger_, "Robot initialized");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
State Go2WRobotInterface::getLatestState() {
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return state_;
}

}  // namespace tbai
