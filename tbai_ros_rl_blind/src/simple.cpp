#include <iostream>
#include <memory>

#include "tbai_ros_static/StaticController.hpp"
#include <ros/ros.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_ros_core/Publishers.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_estim/dfki/DfkiEstimator.hpp>
#include <tbai_ros_rl/BobController.hpp>


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_static");
    ros::NodeHandle nh;

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    auto stateTopic = tbai::fromGlobalConfig<std::string>("state_topic");
    auto commandTopic = tbai::fromGlobalConfig<std::string>("command_topic");
    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    const std::string urdfString = nh.param<std::string>("robot_description", "");

    std::array<std::string, ModelInterface::N_LEGS> feet_frame_names = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};
    
    std::array<std::string, ModelInterface::NUM_JOINTS> joint_names = {
        "LF_HAA", "LF_HFE", "LF_KFE",
        "LH_HAA", "LH_HFE", "LH_KFE", 
        "RF_HAA", "RF_HFE", "RF_KFE",
        "RH_HAA", "RH_HFE", "RH_KFE"
    };
    
    std::array<double, ModelInterface::NUM_JOINTS> joint_default_positions = {
        0.0, 0.4, -0.8, 0.0, -0.4, 0.8, 0.0, 0.4, -0.8, 0.0, -0.4, 0.8
    };

    const std::string base_link_name = "base";

    // Write urdf string to temporary file
    std::string urdf_path = "/tmp/robot_description.urdf";
    std::ofstream urdf_file(urdf_path);
    urdf_file << urdfString;
    urdf_file.close();

    auto quad_model = std::make_shared<QuadModelPino>(urdf_path, feet_frame_names, base_link_name, joint_names, joint_default_positions);
    std::cout << "Hello, World!" << std::endl;


    // https://github.com/dfki-ric-underactuated-lab/dfki-quad/blob/dd3a8a420e868c8af0a9cfcb8db486e6c919fce0/ws/src/state_estimation/config/state_estimation_go2_sim.yaml
    vector3_t init_robot_pose = {0.0, 0.0, 0.0};
    Eigen::Quaterniond initial_orientation = Eigen::Quaterniond::Identity();

    ::KalmanFilter kalman_filter(quad_model, init_robot_pose, initial_orientation);
    tbai::dfki::DfkiEstimator estimator(&kalman_filter);

    std::unique_ptr<tbai::dfki::DfkiEstimator> estimator_ptr;
    estimator_ptr.reset(&estimator);

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber =
        std::shared_ptr<tbai::StateSubscriber>(new tbai::InekfRosStateSubscriber(nh, stateTopic, urdfString, std::move(estimator_ptr)));

    std::shared_ptr<tbai::CommandPublisher> commandPublisher =
        std::shared_ptr<tbai::CommandPublisher>(new tbai::RosCommandPublisher(nh, commandTopic));

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::shared_ptr<tbai::ChangeControllerSubscriber>(
            new tbai::RosChangeControllerSubscriber(nh, changeControllerTopic));

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add static controller
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(stateSubscriber));

    // Add Bob controller
    controller.addController(std::make_unique<tbai::rl::RosBobController>(
        urdfString, stateSubscriber, tbai::reference::getReferenceVelocityGeneratorShared(nh)));

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
