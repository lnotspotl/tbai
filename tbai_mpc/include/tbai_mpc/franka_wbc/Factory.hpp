#pragma once

#include <memory>
#include <string>

#include "tbai_mpc/franka_wbc/HqpWbc.hpp"
#include "tbai_mpc/franka_wbc/SqpWbc.hpp"

namespace tbai {
namespace mpc {
namespace franka {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                       const ocs2::franka::FrankaModelInfo &frankaInfo);


std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                       const ocs2::franka::FrankaModelInfo &frankaInfo);

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
