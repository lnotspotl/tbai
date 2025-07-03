#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_estim/inekf/InEKFEstimator.hpp>

namespace tbai {
namespace inekf {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
InEKFEstimator::InEKFEstimator(std::vector<std::string> footNames, const std::string &urdf) {
    logger_ = tbai::getLogger("inekf_estimator");

    TBAI_LOG_INFO(logger_, "Initializing Pinocchio model");
    setupPinocchioModel(urdf);

    // Set up InEKF parameters
    // TODO: load these from config
    ::inekf::NoiseParams noise_params;
    noise_params.setGyroscopeNoise(Eigen::Vector3d(0.0001, 0.0001, 0.0001));
    noise_params.setAccelerometerNoise(Eigen::Vector3d(0.1, 0.1, 0.1));
    noise_params.setGyroscopeBiasNoise(0.0001);
    noise_params.setAccelerometerBiasNoise(0.0001);
    noise_params.setContactNoise(0.01);
    inekf_.setNoiseParams(noise_params);

    Eigen::Matrix<double, 15, 15> P_init = Eigen::Matrix<double, 15, 15>::Zero();
    P_init.block<3, 3>(0, 0).diagonal().setConstant(pow(0.00001, 2));
    P_init.block<3, 3>(3, 3).diagonal().setConstant(pow(0.00001, 2));
    P_init.block<3, 3>(6, 6).diagonal().setConstant(pow(0.0000001, 2));
    P_init.block<3, 3>(9, 9).diagonal().setConstant(pow(0.001, 2));
    P_init.block<3, 3>(12, 12).diagonal().setConstant(pow(0.001, 2));

    // TODO: Store this for future re-use (state reset)
    ::inekf::RobotState init_state;
    init_state.setRotation(Eigen::Matrix3d::Identity());
    init_state.setVelocity(Eigen::Vector3d::Zero());
    init_state.setPosition(Eigen::Vector3d(0.0, 0.0, 0.35));
    init_state.setGyroscopeBias(Eigen::Vector3d(0.01, 0.01, 0.01));
    init_state.setAccelerometerBias(Eigen::Vector3d(0.01, 0.01, 0.01));
    init_state.setP(P_init);
    inekf_.setState(init_state);

    for (const auto &footName : footNames) {
        bool exists = model_.existBodyName(footName);
        if (!exists) {
            std::vector<std::string> allFrames;
            for (const auto &frame : model_.frames) {
                allFrames.push_back(frame.name);
            }
            TBAI_THROW("Cound not find body {} in model. All frames: {}", footName, allFrames);
        }

        legIndices_.push_back(model_.getBodyId(footName));
    }

    TBAI_LOG_INFO(logger_, "Joint indices from Pinocchio model:");
    for (size_t i = 0; i < model_.frames.size(); ++i) {
        const auto &frame = model_.frames[i];
        TBAI_LOG_INFO(logger_, "Frame '{}': {}", frame.name, i);
    }

    TBAI_LOG_INFO(logger_, "Initialization complete");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InEKFEstimator::update(scalar_t currentTime, scalar_t dt, const vector4_t &quatBase,
                            const vector_t &jointPositions, const vector_t &jointVelocities,
                            const vector3_t &linearAccBase, const vector3_t &angularVelBase, std::vector<bool> contacts,
                            bool rectifyOrientation, bool enablePositionEstimation) {
    // Just store the message on the first call
    if (!firstMessageReceived_) {
        firstMessageReceived_ = true;
        lastAngularVelocity_ = angularVelBase;
        lastLinearAcceleration_ = linearAccBase;
        return;
    }

    if (rectifyOrientation || !enablePositionEstimation) {
        inekf_.getState().setRotation(
            Eigen::Quaterniond(quatBase[3], quatBase[0], quatBase[1], quatBase[2]).toRotationMatrix());
    }

    if (enablePositionEstimation) {
        // Propagate IMU measurements
        Eigen::Matrix<double, 6, 1> imu_measurement;
        imu_measurement << lastAngularVelocity_, lastLinearAcceleration_;
        inekf_.Propagate(imu_measurement, dt);

        // Set contacts
        std::vector<std::pair<int, bool>> contacts_pairs;
        for (size_t i = 0; i < contacts.size(); i++) {
            contacts_pairs.push_back(std::make_pair(i, contacts[i]));
        }
        inekf_.setContacts(contacts_pairs);

        // Update state based on kinematics
        std::vector<vector3_t> footPositions(legIndices_.size());

        vector_t qPinocchio = jointPositions;
        vector_t vPinocchio = jointVelocities;
        pinocchio::forwardKinematics(model_, data_, qPinocchio);

        for (size_t i = 0; i < legIndices_.size(); i++) {
            footPositions[i] = pinocchio::updateFramePlacement(model_, data_, legIndices_[i]).translation();
        }

        ::inekf::vectorKinematics kinematics;
        kinematics.reserve(legIndices_.size());
        for (size_t i = 0; i < legIndices_.size(); i++) {
            constexpr double foot_noise = 0.01;
            kinematics.emplace_back(i, Eigen::Affine3d(Eigen::Translation3d(footPositions[i])).matrix(),
                                    Eigen::Matrix<double, 6, 6>::Identity() * pow(foot_noise, 2));
        }
        inekf_.CorrectKinematics(kinematics);
    }

    lastAngularVelocity_ = angularVelBase;
    lastLinearAcceleration_ = linearAccBase;

    // Done :)
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InEKFEstimator::setupPinocchioModel(const std::string &urdf) {
    if (urdf.empty()) {
        auto urdfPath = tbai::getEnvAs<std::string>("TBAI_ROBOT_DESCRIPTION_PATH");
        pinocchio::urdf::buildModel(urdfPath, model_);
    } else {
        pinocchio::urdf::buildModelFromXML(urdf, model_);
    }
    data_ = pinocchio::Data(model_);
}

}  // namespace inekf
}  // namespace tbai