#include "tbai_mpc/quadruped_arm_wbc/Factory.hpp"

#include <tbai_core/config/Config.hpp>

namespace tbai {
namespace mpc {
namespace quadruped_arm {
std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::quadruped_arm::ComModelBase<scalar_t> &comModel,
                                      const tbai::mpc::quadruped_arm::KinematicsModelBase<scalar_t> &kinematics) {
    auto wbcType = tbai::fromGlobalConfig<std::string>("mpc_controller/wbc_type");

    if (wbcType == "hqp") {
        return std::make_unique<HqpWbc>(controllerConfigFile, urdfString, comModel, kinematics);
    }

    if (wbcType == "sqp") {
        return std::make_unique<SqpWbc>(controllerConfigFile, urdfString, comModel, kinematics);
    }

    throw std::runtime_error("Unknown WBC type: " + wbcType);
}

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::quadruped_arm::ComModelBase<scalar_t> &comModel,
                                      const tbai::mpc::quadruped_arm::KinematicsModelBase<scalar_t> &kinematics) {
    return std::shared_ptr<WbcBase>(getWbcUnique(controllerConfigFile, urdfString, comModel, kinematics).release());
}
}  // namespace quadruped_arm
}  // namespace mpc
}  // namespace tbai
