#include "tbai_mpc/arm_wbc/Factory.hpp"

#include <tbai_core/config/Config.hpp>

namespace tbai::mpc::arm {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::arm::ArmModelInfo &armInfo) {
    auto wbcType = tbai::fromGlobalConfig<std::string>("mpc_controller/wbc_type");

    if (wbcType == "hqp") {
        return std::make_unique<HqpWbc>(controllerConfigFile, urdfString, armInfo);
    }

    if (wbcType == "sqp") {
        return std::make_unique<SqpWbc>(controllerConfigFile, urdfString, armInfo);
    }

    throw std::runtime_error("Unknown arm WBC type: " + wbcType + ". Available types: hqp, sqp");
}

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::arm::ArmModelInfo &armInfo) {
    return std::shared_ptr<WbcBase>(getWbcUnique(controllerConfigFile, urdfString, armInfo).release());
}

}  // namespace tbai::mpc::arm
