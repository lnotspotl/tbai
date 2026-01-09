#pragma once

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "tbai_mpc/franka_mpc/FrankaModelInfo.h"

namespace ocs2 {
namespace franka {

/** Create a Franka PinocchioInterface from a URDF */
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath);

/** Create a Franka PinocchioInterface from a URDF with joint removal */
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath,
                                            const std::vector<std::string>& jointNames);

/** Create a FrankaModelInfo */
FrankaModelInfo createFrankaModelInfo(const PinocchioInterface& interface,
                                      const std::string& baseFrame, const std::string& eeFrame);

}  // namespace franka
}  // namespace ocs2
