#pragma once

#include <stdint.h>
#include <cmath>
#include "unitree/idl/go2/WirelessController_.hpp"
#include <tbai_deploy_go2/Gamepad.hpp>

// #include <go2_idl/WirelessController_.hpp>
#include "unitree/idl/go2/WirelessController_.hpp"
#include "unitree/common/thread/thread.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

#include <tbai_core/Logging.hpp>


using namespace unitree;
using namespace unitree::common;

namespace tbai {
namespace go2 {


#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using namespace unitree::common;
using namespace unitree::robot;

class Go2JoystickInterface
{
public:
    Go2JoystickInterface();
    virtual ~Go2JoystickInterface() = default;

    void InitDdsModel(const std::string &networkInterface = "enp3s0");
    void SetGamepadDeadZone(float deadzone) {gamepad.dead_zone = deadzone;}
    void setGamepadSmooth(float smooth) {gamepad.smooth = smooth;}
    void MessageHandler(const void *message);
    virtual void onPressA() {TBAI_LOG_DEBUG(logger_, "A pressed");}
    virtual void onPressB() {TBAI_LOG_DEBUG(logger_, "B pressed");}
    virtual void onPressX() {TBAI_LOG_DEBUG(logger_, "X pressed");}
    virtual void onPressY() {TBAI_LOG_DEBUG(logger_, "Y pressed");}
    virtual void inReleaseA() {TBAI_LOG_DEBUG(logger_, "A released");}
    virtual void inReleaseB() {TBAI_LOG_DEBUG(logger_, "B released");}
    virtual void inReleaseX() {TBAI_LOG_DEBUG(logger_, "X released");}
    virtual void inReleaseY() {TBAI_LOG_DEBUG(logger_, "Y released");}
    void Step();
    void Start();

protected:
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber;
    unitree_go::msg::dds_::WirelessController_ joystick_msg;

    Gamepad gamepad;

    ThreadPtr control_thread_ptr;

    std::mutex joystick_mutex;

    int press_count = 0;

    std::shared_ptr<spdlog::logger> logger_;
};

} // namespace go2
} // namespace tbai