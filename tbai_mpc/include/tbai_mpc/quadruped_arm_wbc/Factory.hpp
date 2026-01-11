#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tbai_mpc/quadruped_arm_wbc/HqpWbc.hpp"
#include "tbai_mpc/quadruped_arm_wbc/SqpWbc.hpp"

namespace tbai {
namespace mpc {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::quadruped_arm::ComModelBase<scalar_t> &comModel,
                                      const tbai::mpc::quadruped_arm::KinematicsModelBase<scalar_t> &kinematics);

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const tbai::mpc::quadruped_arm::ComModelBase<scalar_t> &comModel,
                                      const tbai::mpc::quadruped_arm::KinematicsModelBase<scalar_t> &kinematics);

}  // namespace mpc
}  // namespace tbai
