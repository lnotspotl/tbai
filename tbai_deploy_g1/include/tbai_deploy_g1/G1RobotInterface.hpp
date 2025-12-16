#pragma once

#include <tbai_core/Args.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>

#define SCHED_DEADLINE 6

#include <math.h>
#include <stdint.h>

#include <unitree/common/thread/thread.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define G1_TOPIC_LOWCMD "rt/lowcmd"
#define G1_TOPIC_LOWSTATE "rt/lowstate"

namespace tbai {

// G1 29DOF joint count
constexpr int G1_NUM_JOINTS = 29;

// State vector size: 3 orientation + 3 position + 3 angular vel + 3 linear vel + 29 joint pos + 29 joint vel
constexpr int G1_STATE_DIM = 3 + 3 + 3 + 3 + G1_NUM_JOINTS + G1_NUM_JOINTS;

struct G1RobotInterfaceArgs {
    TBAI_ARG_DEFAULT(std::string, networkInterface, "eth0");
    TBAI_ARG_DEFAULT(int, unitreeChannel, 0);
    TBAI_ARG_DEFAULT(bool, channelInit, true);
    TBAI_ARG_DEFAULT(bool, enableStateEstim, true);
};

class G1RobotInterface : public RobotInterface {
   public:
    G1RobotInterface(G1RobotInterfaceArgs args);
    virtual ~G1RobotInterface();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

   private:
    void lowStateCallback(const void *message);
    void initMotorMapping();

    unitree_hg::msg::dds_::LowCmd_ low_cmd{};

    ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber;

    // Low cmd write thread
    ThreadPtr lowCmdWriteThreadPtr;

    // Motor name to ID mapping (29 DOF)
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
