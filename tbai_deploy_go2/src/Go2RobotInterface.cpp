#include "tbai_deploy_go2/Go2RobotInterface.hpp"

#include <tbai_core/Rotations.hpp>

#include <stdint.h>

#include <chrono>
#include <string>
#include <thread>

#include <tbai_core/control/Rate.hpp>

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

Go2RobotInterface::Go2RobotInterface() {
    std::cout << "Initializing Go2RobotInterface" << std::endl;

    // Initialize motor 2 id map
    motor_id_map["RF_HAA"] = 0;
    motor_id_map["RF_HFE"] = 1;
    motor_id_map["RF_KFE"] = 2;
    motor_id_map["LF_HAA"] = 3;
    motor_id_map["LF_HFE"] = 4;
    motor_id_map["LF_KFE"] = 5;
    motor_id_map["RH_HAA"] = 6;
    motor_id_map["RH_HFE"] = 7;
    motor_id_map["RH_KFE"] = 8;
    motor_id_map["LH_HAA"] = 9;
    motor_id_map["LH_HFE"] = 10;
    motor_id_map["LH_KFE"] = 11;

    foot_id_map["RF_FOOT"] = 0;
    foot_id_map["LF_FOOT"] = 1;
    foot_id_map["RH_FOOT"] = 2;
    foot_id_map["LH_FOOT"] = 3;

    // // Initialize low level command
    // low_cmd.head()[0] = 0xFE;
    // low_cmd.head()[1] = 0xEF;
    // low_cmd.level_flag() = 0xFF;
    // low_cmd.gpio() = 0;

    // std::cout << "Initializing low level command" << std::endl;
    // for (int i = 0; i < 20; i++) {
    //     low_cmd.motor_cmd()[i].mode() = (0x01);  // motor switch to servo (PMSM) mode
    //     low_cmd.motor_cmd()[i].q() = (PosStopF);
    //     low_cmd.motor_cmd()[i].kp() = (0);
    //     low_cmd.motor_cmd()[i].dq() = (VelStopF);
    //     low_cmd.motor_cmd()[i].kd() = (0);
    //     low_cmd.motor_cmd()[i].tau() = (0);
    // }

    // std::cout << "Initializing publisher" << std::endl;
    // lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    // lowcmd_publisher->InitChannel();

    // std::cout << "Initializing subscriber" << std::endl;
    // lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    // lowstate_subscriber->InitChannel(std::bind(&Go2RobotInterface::lowStateCallback, this, std::placeholders::_1),
    // 1);

    // /*init MotionSwitcherClient*/
    // msc.SetTimeout(10.0f);
    // msc.Init();
    // /*Shut down motion control-related service*/
    // while (queryMotionStatus()) {
    //     std::cout << "Try to deactivate the motion control-related service." << std::endl;
    //     int32_t ret = msc.ReleaseMode();
    //     if (ret == 0) {
    //         std::cout << "ReleaseMode succeeded." << std::endl;
    //     } else {
    //         std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
    //     }
    //     sleep(5);
    // }

    // Initialize estimator
    std::cout << "Initializing estimator" << std::endl;
    std::vector<std::string> footNames = {"LF_FOOT", "RF_FOOT", "RH_FOOT", "LH_FOOT"};
    estimator_ = std::make_unique<tbai::inekf::InEKFEstimator>(footNames, "");
    std::cout << "Estimator initialized" << std::endl;
}

Go2RobotInterface::~Go2RobotInterface() {
    std::cout << "Destroying Go2RobotInterface" << std::endl;
}

std::string Go2RobotInterface::queryServiceName(std::string form, std::string name) {
    if (form == "0") {
        if (name == "normal") return "sport_mode";
        if (name == "ai") return "ai_sport";
        if (name == "advanced") return "advanced_sport";
    } else {
        if (name == "ai-w") return "wheeled_sport(go2W)";
        if (name == "normal-w") return "wheeled_sport(b2W)";
    }
    return "";
}

// int Go2RobotInterface::queryMotionStatus() {
//     std::string robotForm, motionName;
//     int motionStatus;
//     int32_t ret = msc.CheckMode(robotForm, motionName);
//     if (ret == 0) {
//         std::cout << "CheckMode succeeded." << std::endl;
//     } else {
//         std::cout << "CheckMode failed. Error code: " << ret << std::endl;
//     }
//     if (motionName.empty()) {
//         std::cout << "The motion control-related service is deactivated." << std::endl;
//         motionStatus = 0;
//     } else {
//         std::string serviceName = queryServiceName(robotForm, motionName);
//         std::cout << "Service: " << serviceName << " is activate" << std::endl;
//         motionStatus = 1;
//     }
//     return motionStatus;
// }

void Go2RobotInterface::lowStateCallback(const void *message) {
    // Create a copy of the low level state
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
    timestamp = tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow();
    initialized = true;

    // Calculate callback rate
    static auto last_time2 = timestamp;
    static int count = 0;
    count++;

    if (count % 100 == 0) {  // Print every 100th callback to avoid spam
        scalar_t time_diff = timestamp - last_time2;
        double rate = 100.0 / time_diff;
        std::cout << "Low state callback rate: " << rate << " Hz (count: " << count << ")" << std::endl;
        last_time2 = timestamp;
    }

    // Extract joint positions and velocities from low_state
    vector_t jointPositions(12);
    vector_t jointVelocities(12);

    // Map joints in order: LF_HAA, LF_HFE, LF_KFE, LH_HAA, LH_HFE, LH_KFE, RF_HAA, RF_HFE, RF_KFE, RH_HAA, RH_HFE, RH_KFE
    jointPositions[0] = low_state.motor_state()[motor_id_map["LF_HAA"]].q();  // LF_HAA
    jointPositions[1] = low_state.motor_state()[motor_id_map["LF_HFE"]].q();  // LF_HFE
    jointPositions[2] = low_state.motor_state()[motor_id_map["LF_KFE"]].q();  // LF_KFE

    // Left Hind leg
    jointPositions[3] = low_state.motor_state()[motor_id_map["LH_HAA"]].q();  // LH_HAA
    jointPositions[4] = low_state.motor_state()[motor_id_map["LH_HFE"]].q();  // LH_HFE
    jointPositions[5] = low_state.motor_state()[motor_id_map["LH_KFE"]].q();  // LH_KFE

    // Right Front leg
    jointPositions[6] = low_state.motor_state()[motor_id_map["RF_HAA"]].q();  // RF_HAA
    jointPositions[7] = low_state.motor_state()[motor_id_map["RF_HFE"]].q();  // RF_HFE
    jointPositions[8] = low_state.motor_state()[motor_id_map["RF_KFE"]].q();  // RF_KFE

    // Right Hind leg
    jointPositions[9] = low_state.motor_state()[motor_id_map["RH_HAA"]].q();    // RH_HAA
    jointPositions[10] = low_state.motor_state()[motor_id_map["RH_HFE"]].q();  // RH_HFE
    jointPositions[11] = low_state.motor_state()[motor_id_map["RH_KFE"]].q();  // RH_KFE

    // Extract joint velocities in the same order
    jointVelocities[0] = low_state.motor_state()[motor_id_map["LF_HAA"]].dq();  // LF_HAA
    jointVelocities[1] = low_state.motor_state()[motor_id_map["LF_HFE"]].dq();  // LF_HFE
    jointVelocities[2] = low_state.motor_state()[motor_id_map["LF_KFE"]].dq();  // LF_KFE

    jointVelocities[3] = low_state.motor_state()[motor_id_map["LH_HAA"]].dq();  // LH_HAA
    jointVelocities[4] = low_state.motor_state()[motor_id_map["LH_HFE"]].dq();  // LH_HFE
    jointVelocities[5] = low_state.motor_state()[motor_id_map["LH_KFE"]].dq();  // LH_KFE

    jointVelocities[6] = low_state.motor_state()[motor_id_map["RF_HAA"]].dq();  // RF_HAA
    jointVelocities[7] = low_state.motor_state()[motor_id_map["RF_HFE"]].dq();  // RF_HFE
    jointVelocities[8] = low_state.motor_state()[motor_id_map["RF_KFE"]].dq();  // RF_KFE

    jointVelocities[9] = low_state.motor_state()[motor_id_map["RH_HAA"]].dq();    // RH_HAA
    jointVelocities[10] = low_state.motor_state()[motor_id_map["RH_HFE"]].dq();  // RH_HFE
    jointVelocities[11] = low_state.motor_state()[motor_id_map["RH_KFE"]].dq();  // RH_KFE

    // Extract IMU data
    vector4_t quatBase;
    quatBase[0] = low_state.imu_state().quaternion()[1];  // x
    quatBase[1] = low_state.imu_state().quaternion()[2];  // y
    quatBase[2] = low_state.imu_state().quaternion()[3];  // z
    quatBase[3] = low_state.imu_state().quaternion()[0];  // w

    vector3_t linearAccBase;
    linearAccBase[0] = low_state.imu_state().accelerometer()[0];
    linearAccBase[1] = low_state.imu_state().accelerometer()[1];
    linearAccBase[2] = low_state.imu_state().accelerometer()[2];

    vector3_t angularVelBase;
    angularVelBase[0] = low_state.imu_state().gyroscope()[0];
    angularVelBase[1] = low_state.imu_state().gyroscope()[1];
    angularVelBase[2] = low_state.imu_state().gyroscope()[2];

    // Contact states (assuming all feet are in contact for now)
    // Determine contact states based on ground reaction forces
    std::vector<bool> contacts(4, false);
    const double contact_threshold = 15.0;  // N, threshold for contact detection
    
    // Extract ground reaction forces from foot sensors
    // Assuming the order is: LF, LH, RF, RH
    std::vector<double> grf = {
        static_cast<double>(low_state.foot_force()[foot_id_map["LF_FOOT"]]),  // LF
        static_cast<double>(low_state.foot_force()[foot_id_map["LH_FOOT"]]),  // LH  
        static_cast<double>(low_state.foot_force()[foot_id_map["RF_FOOT"]]),  // RF
        static_cast<double>(low_state.foot_force()[foot_id_map["RH_FOOT"]])   // RH
    };
    
    // Set contact to true if ground reaction force exceeds threshold
    for (size_t i = 0; i < 4; ++i) {
        contacts[i] = static_cast<bool>(grf[i] > contact_threshold);
    }

    // Calculate dt (assuming this is called at regular intervals)
    static scalar_t last_time3 = timestamp;
    scalar_t dt = timestamp - last_time3;
    last_time3 = timestamp;

    static bool enable_state_estim = false;
    if (enable_state_estim) {

        // Update the state estimator
        if (estimator_) {
            estimator_->update(timestamp, dt, quatBase, jointPositions, jointVelocities, linearAccBase, angularVelBase,
                               contacts);
        }

    }

    // Get the latest state from the estimator
    State state;
    state.x = vector_t::Zero(36);

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    auto orientation_rot = estimator_->getBaseOrientation();
    const quaternion_t baseQuaternion(orientation_rot);
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    state.x.segment<3>(0) = rpy;

    // Base position
    if(enable_state_estim) {
        state.x.segment<3>(3) = estimator_->getBasePosition();
    }

    // Base angular velocity
    state.x.segment<3>(6) = angularVelBase;

    // Base linear velocity
    if(enable_state_estim) {
        state.x.segment<3>(9) = R_base_world * estimator_->getBaseVelocity();
    }

    // Joint positions
    state.x.segment<12>(12) = jointPositions;

    // Joint velocities
    state.x.segment<12>(12 + 12) = jointVelocities;

    state.timestamp = timestamp;
    state.contactFlags = contacts;

    // Update the latest state
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    state_ = std::move(state);
}

void Go2RobotInterface::publish(std::vector<MotorCommand> commands) {
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

void Go2RobotInterface::waitTillInitialized() {
    std::cout << "Waiting for the robot to initialize..." << std::endl;
    while (!initialized) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Robot initialized" << std::endl;
}

State Go2RobotInterface::getLatestState() {
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return state_;
}

}  // namespace tbai