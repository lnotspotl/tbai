#include "CartPoleUrdf.h"
#include <gtest/gtest.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/urdf.h>

TEST(testPinocchioInterface, buildFromXml) {
    auto pinocchio = ocs2::getPinocchioInterfaceFromUrdfString(cartPoleUrdf);
    std::cout << pinocchio;
}
