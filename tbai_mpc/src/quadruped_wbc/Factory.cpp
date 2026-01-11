#include "tbai_mpc/quadruped_wbc/Factory.hpp"

#include <tbai_core/config/Config.hpp>

namespace tbai {
namespace mpc {
namespace quadruped {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::quadruped::ComModelBase<scalar_t> &comModel,
                                      const tbai::mpc::quadruped::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames) {
    auto wbcType = tbai::fromGlobalConfig<std::string>("mpc_controller/wbc_type");

    if (wbcType == "hqp") {
        return std::make_unique<HqpWbc>(controllerConfigFile, urdfString, comModel, kinematics, jointNames);
    }

    if (wbcType == "sqp") {
        return std::make_unique<SqpWbc>(controllerConfigFile, urdfString, comModel, kinematics, jointNames);
    }

    throw std::runtime_error("Unknown WBC type: " + wbcType);
}

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::quadruped::ComModelBase<scalar_t> &comModel,
                                      const tbai::mpc::quadruped::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames) {
    return std::shared_ptr<WbcBase>(
        getWbcUnique(controllerConfigFile, urdfString, comModel, kinematics, jointNames).release());
}

}  // namespace quadruped
}  // namespace mpc
}  // namespace tbai
