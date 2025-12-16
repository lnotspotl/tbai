#pragma once

#include <tbai_core/Args.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_deploy_go2w/Go2WConstants.hpp>

#define SCHED_DEADLINE 6

#include <math.h>
#include <stdint.h>

#include <unitree/common/thread/thread.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define GO2W_TOPIC_LOWCMD "rt/lowcmd"
#define GO2W_TOPIC_LOWSTATE "rt/lowstate"

namespace tbai {

struct Go2WRobotInterfaceArgs {
    TBAI_ARG_DEFAULT(std::string, networkInterface, "eth0");
    TBAI_ARG_DEFAULT(int, unitreeChannel, 0);
    TBAI_ARG_DEFAULT(bool, channelInit, true);
};

class Go2WRobotInterface : public RobotInterface {
   public:
    Go2WRobotInterface(Go2WRobotInterfaceArgs args);
    virtual ~Go2WRobotInterface();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

   private:
    void lowStateCallback(const void *message);
    void initMotorMapping();

    unitree_go::msg::dds_::LowCmd_ low_cmd{};

    // Publisher
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    // Subscriber
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    // Low cmd write thread
    ThreadPtr lowCmdWriteThreadPtr;

    // Motor name to ID mapping (16 DOF for Go2W)
    std::unordered_map<std::string, int> motor_id_map;

    bool initialized = false;

    scalar_t lastYaw_ = 0.0;
    std::mutex latest_state_mutex_;
    State state_;

    std::shared_ptr<spdlog::logger> logger_;

    bool enable_ = false;
    void enable() override { enable_ = true; }
    void disable() override { enable_ = false; }
};

}  // namespace tbai
