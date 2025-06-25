#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_muse/TbaiEstimator.hpp>

namespace tbai {
namespace muse {
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

    Q << 1.0e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0e-9, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0e-9, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0e-9, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0e-9, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0e-9;

    R << 1.0e-11, 0.0, 0.0,
         0.0, 1.0e-11, 0.0,
         0.0, 0.0, 1.0e-11;
    // clang-format on

    Eigen::Matrix<double, 6, 1> xhat0;
    xhat0 << 0.0, 0.0, 0.27, 0.0, 0.0, 0.0;

    sensorFusion_ = std::make_unique<KFSensorFusion>(xhat0, P, Q, R, false, false);

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
    sensorFusion_->predict(dt, u);

    // reading leg odometry
    Eigen::Vector3d w_v_b = v_b;

    Eigen::Vector3d z_proprio;
    z_proprio << w_v_b;

    // correction
    sensorFusion_->update(dt, z_proprio);
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
    std::vector<Eigen::Vector3d> footVels;

    for (size_t i = 0; i < legIndices_.size(); ++i) {
        // Foot velocity expressed in the world frame
        const size_t frameId = legIndices_[i];
        const auto rf = pinocchio::LOCAL_WORLD_ALIGNED;
        pinocchio::Motion footVelGlobal = pinocchio::getFrameVelocity(model_, data_, frameId, rf);

        // Foot position w.r.t. base frame
        Eigen::Vector3d footPosBase = data_.oMf[frameId].translation();

        // Compute velocity contribution from base angular motion: ω x r
        Eigen::Vector3d omegaCrossR = omega.cross(footPosBase);

        // Compute linear velocity of the foot relative to base
        Eigen::Vector3d relVel = -(footVelGlobal.linear() - omegaCrossR);
        footVels.push_back(relVel);
    }

    double stanceSum = 0.0;
    for (size_t i = 0; i < contacts.size(); i++) {
        stanceSum += static_cast<double>(contacts[i]);
    }

    Eigen::Vector3d baseVelocity = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < contacts.size(); i++) {
        baseVelocity.noalias() += (footVels[i] * static_cast<double>(contacts[i])) / (stanceSum + 1e-5);
    }

    // This is the base velocity expressed w.r.t. to the base frame
    baseVelocity /= (stanceSum + 1e-5);

    // This is the base velocity expressed w.r.t. to the world frame
    baseVelocity = w_R_b * baseVelocity;

    computeLinPosVel(currentTime, dt, acc, w_R_b, baseVelocity);
}

void TbaiEstimator::setupPinocchioModel() {
    auto urdfPath = tbai::getEnvAs<std::string>("TBAI_ROBOT_DESCRIPTION_PATH");
    pinocchio::urdf::buildModel(urdfPath, model_);
    data_ = pinocchio::Data(model_);
}

}  // namespace muse
}  // namespace tbai