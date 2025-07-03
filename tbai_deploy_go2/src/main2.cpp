#include <iostream>

#include <chrono>
#include <thread>
#include <argparse/argparse.hpp>

#define SCHED_DEADLINE 6

#include <tbai_deploy_go2/Go2RobotInterface.hpp>

int main(int argc, char **argv) {
    argparse::ArgumentParser program("tbai_deploy_go2");
    program.add_argument("--networkInterface").default_value("enp3s0");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        exit(1);
    }

    std::cout << "Initializing Go2RobotInterface" << std::endl;
    std::string networkInterface = program.get<std::string>("--networkInterface");
    std::cout << "Network interface: " << networkInterface << std::endl;
    tbai::Go2RobotInterface go2(tbai::Go2RobotInterfaceArgs().networkInterface(std::move(networkInterface)));
    std::cout << "Go2RobotInterface initialized" << std::endl;

    go2.waitTillInitialized();

    std::cout << "Robot initialized" << std::endl;

    go2.getLatestState();

    std::cout << "Sleeping for 10 seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Done" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));

    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        go2.publish(std::vector<tbai::MotorCommand>());
    }

    std::cout << "Program finished." << std::endl;
    return 0;
}