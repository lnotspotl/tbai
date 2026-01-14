#pragma once

#include <memory>
#include <string>

#include "tbai_mpc/arm_wbc/HqpWbc.hpp"
#include "tbai_mpc/arm_wbc/SqpWbc.hpp"

namespace tbai::mpc::arm {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::arm::ArmModelInfo &armInfo);

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::arm::ArmModelInfo &armInfo);

}  // namespace tbai::mpc::arm
