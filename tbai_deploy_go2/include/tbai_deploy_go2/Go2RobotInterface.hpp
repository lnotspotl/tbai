#pragma once

#include <tbai_core/Args.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>

// TODO: remove this define, the code does not compile without it though (missing dependency?)
#define SCHED_DEADLINE 6

#include <math.h>
#include <stdint.h>

#include <tbai_estim/inekf/InEKFEstimator.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LIDAR "rt/utlidar/cloud"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

namespace tbai {

struct Go2RobotInterfaceArgs {
    TBAI_ARG_DEFAULT(std::string, networkInterface, "eth0");
    TBAI_ARG_DEFAULT(int, unitreeChannel, 0);
    TBAI_ARG_DEFAULT(bool, channelInit, true);
    TBAI_ARG_DEFAULT(bool, enableStateEstim, true);
    TBAI_ARG_DEFAULT(bool, subscribeLidar, true);
};

class Go2RobotInterface : public RobotInterface {
   public:
    Go2RobotInterface(Go2RobotInterfaceArgs args);
    virtual ~Go2RobotInterface();

    // virtual methods from RobotInterface
    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

    virtual void lidarCallback(const void *message) {
        // Do nothing by default, a user is expected to override this method, but does not have to
    };

   private:
    void lowStateCallback(const void *message);

    unitree_go::msg::dds_::LowCmd_ low_cmd{};  // default init
    // unitree_go::msg::dds_::LowState_ low_state{};  // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ChannelSubscriberPtr<sensor_msgs::msg::dds_::PointCloud2_> lidar_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;

    std::unordered_map<std::string, int> motor_id_map;
    std::unordered_map<std::string, int> foot_id_map;
    bool initialized = false;

    std::unique_ptr<tbai::inekf::InEKFEstimator> estimator_;

    scalar_t lastYaw_ = 0.0;
    std::mutex latest_state_mutex_;
    State state_;

    std::shared_ptr<spdlog::logger> logger_;

    bool rectifyOrientation_ = true;
    bool removeGyroscopeBias_ = true;

    bool enable_ = false;  // Enable state estimation
    void enable() override { enable_ = true; }
    void disable() override { enable_ = false; }
};

}  // namespace tbai