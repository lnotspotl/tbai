#include "tbai_mpc/franka_wbc/Factory.hpp"

#include <tbai_core/config/Config.hpp>

namespace tbai {
namespace mpc {
namespace franka {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                       const ocs2::franka::FrankaModelInfo &frankaInfo) {
    auto wbcType = tbai::fromGlobalConfig<std::string>("mpc_controller/wbc_type");

    if (wbcType == "hqp") {
        return std::make_unique<HqpWbc>(controllerConfigFile, urdfString, frankaInfo);
    }

    if (wbcType == "sqp") {
        return std::make_unique<SqpWbc>(controllerConfigFile, urdfString, frankaInfo);
    }

    throw std::runtime_error("Unknown Franka WBC type: " + wbcType + ". Available types: hqp, sqp");
}

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                       const ocs2::franka::FrankaModelInfo &frankaInfo) {
    return std::shared_ptr<WbcBase>(getWbcUnique(controllerConfigFile, urdfString, frankaInfo).release());
}

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
