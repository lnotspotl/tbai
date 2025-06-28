#pragma once

#include <memory>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_muse/SensorFusion.hpp>

namespace tbai {
namespace muse {
class TbaiEstimator {
   public:
    TbaiEstimator(std::vector<std::string> footNames, const std::string &urdf = "");

    void update(scalar_t currentTime, scalar_t dt, const vector4_t &quatBase, const vector_t &jointPositions,
                const vector_t &jointVelocities, const vector3_t &linearAccBase, const vector3_t &angularVelBase,
                std::vector<bool> contacts);

    void computeLinPosVel(scalar_t currentTime, scalar_t dt, Eigen::Vector3d &acc, Eigen::Matrix3d &w_R_b,
                          Eigen::Vector3d &v_b);

    std::unique_ptr<KFSensorFusion> sensorFusion_;

    void setupPinocchioModel(const std::string &urdf = "");

    vector3_t getBasePosition() { return sensorFusion_->getX().head<3>(); }
    vector3_t getBaseVelocity() { return sensorFusion_->getX().segment<3>(3); }

    pinocchio::Model model_;
    pinocchio::Data data_;

    std::vector<size_t> legIndices_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace muse
}  // namespace tbai