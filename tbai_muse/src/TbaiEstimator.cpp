#include <pinocchio/parsers/urdf.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_muse/TbaiEstimator.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace tbai {
TbaiEstimator::TbaiEstimator(std::vector<std::string> footNames) {
    logger_ = tbai::getLogger("TbaiEstimator");

    TBAI_LOG_INFO(logger_, "Initializing Pinocchio model");
    setupPinocchioModel();

    TBAI_LOG_INFO(logger_, "Initializing sensor fusion");
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P;
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Q;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R;

    // From config: https://github.com/iit-DLSLab/muse/blob/main/muse_ws/src/state_estimator/config/sensor_fusion.yaml

    // clang-format off
    P << 1.0e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0e-3, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0e-3, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0e-4, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0e-4, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0e-4;
    // clang-format on

    // clang-format off
    Q << 1.0e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0e-9, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0e-9, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0e-9, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0e-9, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0e-9;
    // clang-format on

    // clang-format off
    R << 1.0e-11, 0.0, 0.0,
         0.0, 1.0e-11, 0.0,
         0.0, 0.0, 1.0e-11;
    // clang-format on

    Eigen::Matrix<double, 6, 1> xhat0 = Eigen::Matrix<double, 6, 1>::Zero();
    scalar_t initTime = readInitTime();

    sensorFusion_ = std::make_unique<state_estimator::KFSensorFusion>(initTime, xhat0, P, Q, R, false, false);

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
	for(size_t i=0; i < model_.frames.size(); ++i) {
		const auto& frame = model_.frames[i];
		TBAI_LOG_INFO(logger_, "Frame '{}': {}", frame.name, i);
	}

	TBAI_LOG_INFO(logger_, "Initialization complete");
}

void TbaiEstimator::computeLinPosVel(scalar_t currentTime, scalar_t dt, Eigen::Vector3d &acc, Eigen::Matrix3d &w_R_b,
                                     Eigen::Vector3d &v_b) {
    // reading acceleration from imu
    Eigen::Vector3d f_b = acc;
    Eigen::Vector3d f_w = w_R_b * f_b;

    // input u = w_R_b*f_b - gravity
    Eigen::Vector3d gravity;
    gravity << 0.0, 0.0, -9.81;
    Eigen::Vector3d u = f_w + gravity;

    // prediction
    sensorFusion_->predict(currentTime, u);

    // reading leg odometry
    Eigen::Vector3d w_v_b = v_b;

    Eigen::Vector3d z_proprio;
    z_proprio << w_v_b;

    // correction
    sensorFusion_->update(currentTime, z_proprio);
}

void TbaiEstimator::update(scalar_t currentTime, scalar_t dt, const vector4_t &quatBase, const vector_t &jointPositions,
                           const vector_t &jointVelocities, const vector3_t &linearAccBase,
                           const vector3_t &angularVelBase, std::vector<bool> contacts) {
    Eigen::Vector3d acc = linearAccBase;
    Eigen::Vector3d omega = angularVelBase;

    matrix3_t w_R_b = tbai::quat2mat(quaternion_t(quatBase[3], quatBase[0], quatBase[1], quatBase[2]));

    // Calculate leg odometry
    size_t numContacts = 0;
    for (size_t i = 0; i < contacts.size(); i++) {
        numContacts += static_cast<size_t>(contacts[i]);
    }

    vector_t qPinocchio = jointPositions;
    vector_t vPinocchio = jointVelocities;

    pinocchio::forwardKinematics(model_, data_, qPinocchio, vPinocchio);
    pinocchio::updateFramePlacements(model_, data_);
    std::vector<Eigen::Vector3d> foot_vels;

    for (size_t i = 0; i < legIndices_.size(); ++i) {
        std::size_t frame_id = legIndices_[i];

        // Get spatial velocity in LOCAL_WORLD_ALIGNED frame
        pinocchio::Motion foot_vel_global =
            pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);

        // Get position of the foot in the base frame
        Eigen::Vector3d foot_pos_base = data_.oMf[frame_id].translation();  // Position of foot in world
        Eigen::Vector3d omega_rotated = omega;

        // Compute velocity contribution from base angular motion: ω x r
        Eigen::Vector3d omega_cross_r = omega_rotated.cross(foot_pos_base);

        // Compute linear velocity of the foot relative to base
        Eigen::Vector3d rel_vel = -(foot_vel_global.linear() - omega_cross_r);
        foot_vels.push_back(rel_vel);
    }

    Eigen::Vector3d lin_leg_lf = foot_vels[0];
    Eigen::Vector3d lin_leg_rf = foot_vels[1];
    Eigen::Vector3d lin_leg_lh = foot_vels[2];
    Eigen::Vector3d lin_leg_rh = foot_vels[3];

	double stance_lf = static_cast<double>(contacts[0]);
    double stance_rf = static_cast<double>(contacts[1]);
    double stance_lh = static_cast<double>(contacts[2]);
    double stance_rh = static_cast<double>(contacts[3]);

    double sum_stance = stance_lf + stance_rf + stance_lh + stance_rh;
    Eigen::Vector3d base_velocity =
        (stance_lf * lin_leg_lf + stance_rf * lin_leg_rf + stance_lh * lin_leg_lh + stance_rh * lin_leg_rh) /
        (sum_stance + 1e-5);

    base_velocity = w_R_b * base_velocity;

    computeLinPosVel(currentTime, dt, acc, w_R_b, base_velocity);
}

void TbaiEstimator::setupPinocchioModel() {
    auto urdfPath = tbai::getEnvAs<std::string>("TBAI_ROBOT_DESCRIPTION_PATH");
    pinocchio::urdf::buildModel(urdfPath, model_);
    data_ = pinocchio::Data(model_);
}

}  // namespace tbai