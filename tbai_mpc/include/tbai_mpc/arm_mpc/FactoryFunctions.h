#pragma once

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "tbai_mpc/arm_mpc/ArmModelInfo.h"

namespace tbai::mpc::arm {

/** Create an Arm PinocchioInterface from a URDF */
ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath);

/** Create an Arm PinocchioInterface from a URDF with joint removal */
ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath,
                                            const std::vector<std::string>& jointNames);

/** Create an ArmModelInfo */
ArmModelInfo createArmModelInfo(const ocs2::PinocchioInterface& interface,
                                      const std::string& baseFrame, const std::string& eeFrame);

}  // namespace tbai::mpc::arm
