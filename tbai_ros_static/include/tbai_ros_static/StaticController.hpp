#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tbai_ros_core/Subscribers.hpp"
#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_static/StaticController.hpp>
#include <tf/transform_broadcaster.h>

namespace tbai {
namespace static_ {

class RosStaticController : public tbai::static_::StaticController {
   public:
    /**
     * @brief Construct a new StaticController object
     *
     */
    RosStaticController(std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr);
    void postStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override { return ros::ok(); }

    void preStep(scalar_t currentTime, scalar_t dt) override {
        ros::spinOnce();
        state_ = stateSubscriberPtr_->getLatestState();
    }

    void changeController(const std::string &controllerType, scalar_t currentTime) override {
        tbai::static_::StaticController::changeController(controllerType, currentTime);
    }

   private:
    /** Publish odom->base transforms */
    void publishOdomBaseTransforms(const vector_t &currentState, const ros::Time &currentTime);

    /** Publish joint angles */
    void publishJointAngles(const vector_t &currentState, const ros::Time &currentTime);

    /** Visualization */
    tf::TransformBroadcaster tfBroadcaster_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

    /** Time since last visualization step */
    scalar_t timeSinceLastVisualizationUpdate_;
};

}  // namespace static_
}  // namespace tbai
