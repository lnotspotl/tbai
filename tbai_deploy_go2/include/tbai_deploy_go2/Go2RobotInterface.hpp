#pragma once

#include <tbai_core/Args.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_core/Logging.hpp>

// TODO: remove this define, the code does not compile without it though (missing dependency?)
#define SCHED_DEADLINE 6

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <iostream>

#include <unitree/common/thread/thread.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <tbai_estim/inekf/InEKFEstimator.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);



namespace tbai {

struct Go2RobotInterfaceArgs {
    TBAI_ARG_DEFAULT(std::string, networkInterface, "192.168.123.10");
    TBAI_ARG_DEFAULT(bool, channelInit, true);
    TBAI_ARG_DEFAULT(bool, enableStateEstim, true);
};

class Go2RobotInterface : public RobotInterface {
   public:
    Go2RobotInterface(Go2RobotInterfaceArgs args);
    virtual ~Go2RobotInterface();

    // virtual methods from RobotInterface
    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

   private:
    int queryMotionStatus();
    std::string queryServiceName(std::string form, std::string name);
    void lowStateCallback(const void *message);

    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    // unitree_go::msg::dds_::LowState_ low_state{};  // default init

    std::unique_ptr<MotionSwitcherClient> msc;

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;
    
    std::unordered_map<std::string, int> motor_id_map;
    std::unordered_map<std::string, int> foot_id_map;
    scalar_t timestamp = 0.0;
    bool initialized = false;

    std::unique_ptr<tbai::inekf::InEKFEstimator> estimator_;

    scalar_t lastYaw_ = 0.0;
    std::mutex latest_state_mutex_;
    State state_;

    std::shared_ptr<spdlog::logger> logger_;

    bool enablePositionEstimation_ = false;

    virtual void enable() override {
        enablePositionEstimation_ = true;
    }

    virtual void disable() override {
        enablePositionEstimation_ = false;
    }
};

}  // namespace tbai