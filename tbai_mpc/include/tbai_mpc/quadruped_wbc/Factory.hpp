#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tbai_mpc/quadruped_wbc/HqpWbc.hpp"
#include "tbai_mpc/quadruped_wbc/SqpWbc.hpp"

namespace tbai {
namespace mpc {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const switched_model::ComModelBase<scalar_t> &comModel,
                                      const switched_model::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames);

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const switched_model::ComModelBase<scalar_t> &comModel,
                                      const switched_model::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames);

}  // namespace mpc
}  // namespace tbai
