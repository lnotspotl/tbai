#include <iostream>

#include <chrono>
#include <thread>

#define SCHED_DEADLINE 6

#include <unitree/robot/channel/channel_factory.hpp>

#include <tbai_deploy_go2/Go2RobotInterface.hpp>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    std::cout << "Connecting to the robot... Network interface: " << argv[1] << std::endl;
    // unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    std::cout << "Connection established" << std::endl;

    std::cout << "Initializing Go2RobotInterface" << std::endl;
    tbai::Go2RobotInterface go2;
    std::cout << "Go2RobotInterface initialized" << std::endl;

    // go2.waitTillInitialized();

    // std::cout << "Robot initialized" << std::endl;

    // go2.getLatestState();

    // std::cout << "Sleeping for 10 seconds" << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(10));

    // std::cout << "Done" << std::endl;

    // std::this_thread::sleep_for(std::chrono::seconds(2));


    std::cout << "Hello, World!" << std::endl;
    return 0;
}