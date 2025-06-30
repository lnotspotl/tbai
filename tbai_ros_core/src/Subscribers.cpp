#include "tbai_ros_core/Subscribers.hpp"

#include <tbai_core/Rotations.hpp>
#include <tbai_core/Throws.hpp>

namespace tbai {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
RosStateSubscriber::RosStateSubscriber(ros::NodeHandle &nh, const std::string &stateTopic) {
    stateSubscriber_ = nh.subscribe(stateTopic, 1, &RosStateSubscriber::stateMessageCallback, this);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStateSubscriber::waitTillInitialized() {
    while (!stateMessage_ && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
        ROS_INFO_STREAM_THROTTLE(1, "[StateSubscriber] Waiting for state message...");
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
State RosStateSubscriber::getLatestState() {
    State state;
    state.x = vector_t(Eigen::Map<vector_t>(stateMessage_->rbd_state.data(), stateMessage_->rbd_state.size()));
    state.timestamp = stateMessage_->stamp.toSec();
    state.contactFlags = std::vector<bool>(stateMessage_->contact_flags.begin(), stateMessage_->contact_flags.end());
    return state;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStateSubscriber::stateMessageCallback(const tbai_ros_msgs::RbdState::Ptr &msg) {
    stateMessage_ = msg;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
MuseRosStateSubscriber::MuseRosStateSubscriber(ros::NodeHandle &nhtemp, const std::string &stateTopic,
                                               const std::string &urdf) {
    logger_ = tbai::getLogger("MuseRosStateSubscriber");

    ros::NodeHandle nh;
    nh.setCallbackQueue(&thisQueue_);

    TBAI_LOG_INFO(logger_, "Initializing MuseRosStateSubscriber");
    stateSubscriber_ = nh.subscribe(stateTopic, 1, &MuseRosStateSubscriber::stateMessageCallback, this);

    TBAI_LOG_INFO(logger_, "Initialized MuseRosStateSubscriber");

    std::vector<std::string> footNames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    estimator_ = std::make_unique<tbai::muse::MuseEstimator>(footNames, urdf);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::waitTillInitialized() {
    if (!isRunning_) {
        startThreads();
    }

    TBAI_THROW_UNLESS(isRunning_, "MuseRosStateSubscriber not running");
    while (!isInitialized_ && ros::ok()) {
        ros::spinOnce();
        ros::Duration(1.0 / 5.0).sleep();
        TBAI_LOG_INFO(logger_, "Waiting for state message...");
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::startThreads() {
    isRunning_ = true;
    stateThread_ = std::thread(&MuseRosStateSubscriber::threadFunction, this);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::stopThreads() {
    isRunning_ = false;
    if (stateThread_.joinable()) {
        stateThread_.join();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::threadFunction() {
    while (isRunning_ && ros::ok()) {
        thisQueue_.callAvailable(ros::WallDuration(1.0 / 5.0));
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::stateMessageCallback(const tbai_ros_msgs::RobotState::Ptr &msg) {
    // Determine time step since last state
    scalar_t dt = 0.0;
    scalar_t currentTime = msg->stamp.toSec();
    if (firstState_) {
        lastStateTime_ = msg->stamp;
        firstState_ = false;
    } else {
        dt = currentTime - lastStateTime_.toSec();
        lastStateTime_ = msg->stamp;
    }

    // Unpack base orientation
    vector4_t baseOrientation;
    baseOrientation[0] = msg->orientation_xyzw[0];
    baseOrientation[1] = msg->orientation_xyzw[1];
    baseOrientation[2] = msg->orientation_xyzw[2];
    baseOrientation[3] = msg->orientation_xyzw[3];

    const quaternion_t baseQuaternion(baseOrientation[3], baseOrientation[0], baseOrientation[1], baseOrientation[2]);
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    std::vector<bool> contactFlags = {msg->contact_flags[0], msg->contact_flags[1], msg->contact_flags[2],
                                      msg->contact_flags[3]};

    State state;
    state.x = vector_t::Zero(36);

    vector_t jointAngles = vector_t::Zero(12);
    for (int i = 0; i < 12; i++) {
        jointAngles[i] = msg->joint_angles[i];
    }

    vector_t jointVelocities = vector_t::Zero(12);
    for (int i = 0; i < 12; i++) {
        jointVelocities[i] = msg->joint_velocities[i];
    }

    vector3_t baseAcc = vector3_t::Zero();
    baseAcc[0] = msg->lin_acc[0];
    baseAcc[1] = msg->lin_acc[1];
    baseAcc[2] = msg->lin_acc[2];

    vector3_t baseAngVel = vector3_t::Zero();
    baseAngVel[0] = msg->ang_vel[0];
    baseAngVel[1] = msg->ang_vel[1];
    baseAngVel[2] = msg->ang_vel[2];

    estimator_->update(currentTime, dt, baseOrientation, jointAngles, jointVelocities, baseAcc, baseAngVel,
                       contactFlags);

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    state.x.segment<3>(0) = rpy;

    // Base position
    state.x.segment<3>(3) = estimator_->getBasePosition();

    // Base angular velocity
    state.x.segment<3>(6) = baseAngVel;

    // Base linear velocity
    state.x.segment<3>(9) = R_base_world * estimator_->getBaseVelocity();

    // Joint positions
    state.x.segment<12>(12) = jointAngles;

    // Joint velocities
    state.x.segment<12>(12 + 12) = jointVelocities;

    state.timestamp = currentTime;
    state.contactFlags = contactFlags;

    isInitialized_ = true;
    std::lock_guard<std::mutex> lock(stateMutex_);
    state_ = std::move(state);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
InekfRosStateSubscriber::InekfRosStateSubscriber(ros::NodeHandle &nhtemp, const std::string &stateTopic,
                                                 const std::string &urdf, std::unique_ptr<tbai::dfki::DfkiEstimator> estimator) {
    logger_ = tbai::getLogger("InekfRosStateSubscriber");

    ros::NodeHandle nh;
    nh.setCallbackQueue(&thisQueue_);

    TBAI_LOG_INFO(logger_, "Initializing InekfRosStateSubscriber");
    stateSubscriber_ = nh.subscribe(stateTopic, 1, &InekfRosStateSubscriber::stateMessageCallback, this);

    TBAI_LOG_INFO(logger_, "Initialized InekfRosStateSubscriber");

    estimator_ = std::move(estimator);

    TBAI_LOG_INFO(logger_, "Initilization of DfkiEstimator done");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::waitTillInitialized() {
    std::cout << "Waiting for state message..." << std::endl;
    if (!isRunning_) {
        startThreads();
    }

    std::cout << "isRunning_ " << isRunning_ << std::endl;

    TBAI_THROW_UNLESS(isRunning_, "MuseRosStateSubscriber not running");
    while (!isInitialized_ && ros::ok()) {
        ros::spinOnce();
        ros::Duration(1.0 / 5.0).sleep();
        TBAI_LOG_INFO(logger_, "Waiting for state message...");
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::startThreads() {
    std::cout << "Starting threads..." << std::endl;
    isRunning_ = true;
    stateThread_ = std::thread(&InekfRosStateSubscriber::threadFunction, this);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::stopThreads() {
    std::cout << "Stopping threads..." << std::endl;
    isRunning_ = false;
    if (stateThread_.joinable()) {
        stateThread_.join();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::threadFunction() {
    std::cout << "Thread function..." << std::endl;
    while (isRunning_ && ros::ok()) {
        thisQueue_.callAvailable(ros::WallDuration(1.0 / 5.0));
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::stateMessageCallback(const tbai_ros_msgs::RobotState::Ptr &msg) {
    std::cout << "State message callback..." << std::endl;
    // Determine time step since last state
    scalar_t dt = 0.0;
    scalar_t currentTime = msg->stamp.toSec();
    if (firstState_) {
        lastStateTime_ = msg->stamp;
        firstState_ = false;
    } else {
        dt = currentTime - lastStateTime_.toSec();
        lastStateTime_ = msg->stamp;
    }

    // Unpack base orientation
    vector4_t baseOrientation;
    baseOrientation[0] = msg->orientation_xyzw[0];
    baseOrientation[1] = msg->orientation_xyzw[1];
    baseOrientation[2] = msg->orientation_xyzw[2];
    baseOrientation[3] = msg->orientation_xyzw[3];

    std::vector<bool> contactFlags = {msg->contact_flags[0], msg->contact_flags[1], msg->contact_flags[2],
                                      msg->contact_flags[3]};

    State state;
    state.x = vector_t::Zero(36);


    vector_t jointAngles = vector_t::Zero(12);
    for (int i = 0; i < 12; i++) {
        jointAngles[i] = msg->joint_angles[i];
    }


    vector_t jointVelocities = vector_t::Zero(12);
    for (int i = 0; i < 12; i++) {
        jointVelocities[i] = msg->joint_velocities[i];
    }

    vector3_t baseAcc = vector3_t::Zero();
    baseAcc[0] = msg->lin_acc[0];
    baseAcc[1] = msg->lin_acc[1];
    baseAcc[2] = msg->lin_acc[2];

    vector3_t baseAngVel = vector3_t::Zero();
    baseAngVel[0] = msg->ang_vel[0];
    baseAngVel[1] = msg->ang_vel[1];
    baseAngVel[2] = msg->ang_vel[2];


    estimator_->update(currentTime, dt, baseOrientation, jointAngles, jointVelocities, baseAcc, baseAngVel,
                       contactFlags);

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    auto orientation_rot = estimator_->getBaseOrientation();
    const quaternion_t baseQuaternion(orientation_rot);
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    state.x.segment<3>(0) = rpy;

    // Base position
    state.x.segment<3>(3) = estimator_->getBasePosition();

    // Base angular velocity
    state.x.segment<3>(6) = baseAngVel;
    // state.x.segment<3>(6) = baseAngVel - estimator_->getGyroscopeBias();

    // Base linear velocity
    state.x.segment<3>(9) = R_base_world * estimator_->getBaseVelocity();

    // Joint positions
    state.x.segment<12>(12) = jointAngles;

    // Joint velocities
    state.x.segment<12>(12 + 12) = jointVelocities;

    state.timestamp = currentTime;
    state.contactFlags = contactFlags;

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        state_ = std::move(state);
    }

    isInitialized_ = true;
}
}  // namespace tbai
