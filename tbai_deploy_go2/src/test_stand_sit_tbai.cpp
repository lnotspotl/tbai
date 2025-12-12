#include <tbai_deploy_go2/Go2RobotInterface.hpp>

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

// Joint names in order: RF, LF, RH, LH (each: HAA, HFE, KFE)
static const std::vector<std::string> JOINT_NAMES = {
    "RF_HAA", "RF_HFE", "RF_KFE",
    "LF_HAA", "LF_HFE", "LF_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE"
};

class StandSitController {
public:
    StandSitController(tbai::Go2RobotInterface& robot) : robot_(robot) {}

    void run() {
        // Wait for robot to initialize (receive first state)
        robot_.waitTillInitialized();

        std::cout << "Robot initialized. Starting stand/sit sequence..." << std::endl;

        // Get initial joint positions from robot state
        tbai::State state = robot_.getLatestState();
        for (int i = 0; i < 12; i++) {
            startPos_[i] = state.x[12 + i];  // Joint positions start at index 12
        }

        // Main control loop at 500 Hz (2ms period)
        const auto period = std::chrono::microseconds(2000);
        auto nextTime = std::chrono::steady_clock::now();

        while (!done_) {
            update();

            nextTime += period;
            std::this_thread::sleep_until(nextTime);
        }

        std::cout << "Stand/sit sequence completed!" << std::endl;

        // Keep publishing final position to maintain control
        while (true) {
            publishCommands(targetPos3_);
            std::this_thread::sleep_for(period);
        }
    }

private:
    void update() {
        motionTime_++;

        // Print sensor data during motion
        if (percent4_ < 1.0f) {
            if (motionTime_ % 500 == 0) {  // Print every second
                tbai::State state = robot_.getLatestState();
                std::cout << "Read sensor data example: " << std::endl;
                std::cout << "Joint 0 (RF_HAA) pos: " << state.x[12] << std::endl;
                std::cout << "Base orientation (roll, pitch, yaw): "
                          << state.x[0] << ", " << state.x[1] << ", " << state.x[2] << std::endl;
                std::cout << std::endl;
            }
        }

        if (percent4_ >= 1.0f && !done_) {
            std::cout << "The example is done!" << std::endl;
            done_ = true;
            return;
        }

        // Wait 500 iterations (1 second) before starting motion
        if (motionTime_ < 500) {
            return;
        }

        // Phase 1: Move from start position to targetPos1 (sit)
        percent1_ += 1.0f / duration1_;
        percent1_ = std::min(percent1_, 1.0f);

        if (percent1_ < 1.0f) {
            float pos[12];
            for (int j = 0; j < 12; j++) {
                pos[j] = (1.0f - percent1_) * startPos_[j] + percent1_ * targetPos1_[j];
            }
            publishCommands(pos);
            return;
        }

        // Phase 2: Move from targetPos1 to targetPos2 (stand)
        if (percent2_ < 1.0f) {
            percent2_ += 1.0f / duration2_;
            percent2_ = std::min(percent2_, 1.0f);

            float pos[12];
            for (int j = 0; j < 12; j++) {
                pos[j] = (1.0f - percent2_) * targetPos1_[j] + percent2_ * targetPos2_[j];
            }
            publishCommands(pos);
            return;
        }

        // Phase 3: Hold targetPos2 position
        if (percent3_ < 1.0f) {
            percent3_ += 1.0f / duration3_;
            percent3_ = std::min(percent3_, 1.0f);

            publishCommands(targetPos2_);
            return;
        }

        // Phase 4: Move from targetPos2 to targetPos3
        if (percent4_ <= 1.0f) {
            percent4_ += 1.0f / duration4_;
            percent4_ = std::min(percent4_, 1.0f);

            float pos[12];
            for (int j = 0; j < 12; j++) {
                pos[j] = (1.0f - percent4_) * targetPos2_[j] + percent4_ * targetPos3_[j];
            }
            publishCommands(pos);
        }
    }

    void publishCommands(const float* positions) {
        std::vector<tbai::MotorCommand> commands;
        commands.reserve(12);

        for (int i = 0; i < 12; i++) {
            tbai::MotorCommand cmd;
            cmd.joint_name = JOINT_NAMES[i];
            cmd.desired_position = positions[i];
            cmd.desired_velocity = 0.0;
            cmd.kp = kp_;
            cmd.kd = kd_;
            cmd.torque_ff = 0.0;
            commands.push_back(cmd);
        }

        robot_.publish(commands);
    }

    tbai::Go2RobotInterface& robot_;

    float kp_ = 60.0f;
    float kd_ = 5.0f;

    int motionTime_ = 0;
    bool done_ = false;

    // Target positions for each phase (RF, LF, RH, LH - each HAA, HFE, KFE)
    float targetPos1_[12] = {0.0f, 1.36f, -2.65f, 0.0f, 1.36f, -2.65f, -0.2f, 1.36f, -2.65f, 0.2f, 1.36f, -2.65f};
    float targetPos2_[12] = {0.0f, 0.67f, -1.3f, 0.0f, 0.67f, -1.3f, 0.0f, 0.67f, -1.3f, 0.0f, 0.67f, -1.3f};
    float targetPos3_[12] = {-0.35f, 1.36f, -2.65f, 0.35f, 1.36f, -2.65f, -0.5f, 1.36f, -2.65f, 0.5f, 1.36f, -2.65f};
    float startPos_[12] = {0};

    // Duration for each phase (in loop iterations at 500 Hz)
    float duration1_ = 500.0f;
    float duration2_ = 500.0f;
    float duration3_ = 1000.0f;
    float duration4_ = 900.0f;

    // Progress percentages for each phase
    float percent1_ = 0.0f;
    float percent2_ = 0.0f;
    float percent3_ = 0.0f;
    float percent4_ = 0.0f;
};

int main(int argc, const char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " networkInterface unitreeChannel (unitreeChannel is 0 for hardware and 1 for simulation)" << std::endl;
        exit(-1);
    }

    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Create Go2RobotInterface with specified network interface
    tbai::Go2RobotInterfaceArgs args;
    args.networkInterface(argv[1]);
    args.unitreeChannel(std::stoi(argv[2]));
    args.channelInit(true);
    args.subscribeLidar(false);  // Don't need lidar for this test

    tbai::Go2RobotInterface robot(args);

    // Run stand/sit controller
    StandSitController controller(robot);
    controller.run();

    return 0;
}
