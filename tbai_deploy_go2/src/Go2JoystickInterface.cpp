#pragma once

#include <stdint.h>

#include <cmath>

#define SCHED_DEADLINE 6

#include <tbai_deploy_go2/Gamepad.hpp>
#include <tbai_deploy_go2/Go2JoystickInterface.hpp>

// #include <go2_idl/WirelessController_.hpp>
#include "unitree/common/thread/thread.hpp"
#include "unitree/idl/go2/WirelessController_.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"



using namespace unitree;
using namespace unitree::common;

namespace tbai {
namespace go2 {

#include <mutex>

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using namespace unitree::common;
using namespace unitree::robot;

Go2JoystickInterface::Go2JoystickInterface() {
    logger_ = getLogger("go2_joystick");
    setGamepadSmooth(0.3);
    SetGamepadDeadZone(0.05);
}

void Go2JoystickInterface::InitDdsModel(const std::string &networkInterface) {
    ChannelFactory::Instance()->Init(0, networkInterface);
    joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));

    joystick_subscriber->InitChannel(std::bind(&Go2JoystickInterface::MessageHandler, this, std::placeholders::_1), 1);
}

void Go2JoystickInterface::MessageHandler(const void *message) {
    std::lock_guard<std::mutex> lock(joystick_mutex);
    joystick_msg = *(unitree_go::msg::dds_::WirelessController_ *)message;
}

void Go2JoystickInterface::Step() {
    {
        std::lock_guard<std::mutex> lock(joystick_mutex);
        gamepad.Update(joystick_msg);
    }
    if (gamepad.A.on_press) onPressA();
    if (gamepad.B.on_press) onPressB();
    if (gamepad.X.on_press) onPressX();
    if (gamepad.Y.on_press) onPressY();
    if (gamepad.A.on_release) inReleaseA();
    if (gamepad.B.on_release) inReleaseB();
    if (gamepad.X.on_release) inReleaseX();
    if (gamepad.Y.on_release) inReleaseY();
}

// start the work thread
void Go2JoystickInterface::Start() {
    InitDdsModel();
    control_thread_ptr =
        CreateRecurrentThreadEx("joystick_ctrl", UT_CPU_ID_NONE, 40000, &Go2JoystickInterface::Step, this);
}

}  // namespace go2
}  // namespace tbai

int main() {
    // create example object
    tbai::go2::Go2JoystickInterface example;
    example.Start();

    while (1) {
        usleep(20000);
    }
    return 0;
}