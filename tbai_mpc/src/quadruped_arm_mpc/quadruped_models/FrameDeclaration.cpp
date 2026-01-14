//
// Created by rgrandia on 27.04.22.
//

#include "tbai_mpc/quadruped_arm_mpc/quadruped_models/FrameDeclaration.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

namespace tbai::mpc::quadruped_arm {

std::vector<std::string> getJointNames(const FrameDeclaration &frameDeclaration) {
    std::vector<std::string> jointNames;
    // Add leg joints (12 joints: 4 legs × 3 joints)
    for (const auto &leg : frameDeclaration.legs) {
        jointNames.insert(jointNames.end(), leg.joints.begin(), leg.joints.end());
    }
    // Add arm joints (6 joints for the arm)
    jointNames.insert(jointNames.end(), frameDeclaration.arm.joints.begin(), frameDeclaration.arm.joints.end());
    return jointNames;
}

LimbFrames limbFramesFromFile(const std::string &file, const std::string &field) {
    LimbFrames frames;
    ocs2::loadData::loadCppDataType(file, field + ".root", frames.root);
    ocs2::loadData::loadCppDataType(file, field + ".tip", frames.tip);
    ocs2::loadData::loadStdVector(file, field + ".joints", frames.joints, false);
    return frames;
}

ArmFrames armFramesFromFile(const std::string &file, const std::string &field) {
    ArmFrames frames;
    ocs2::loadData::loadCppDataType(file, field + ".root", frames.root);
    ocs2::loadData::loadCppDataType(file, field + ".tip", frames.tip);
    ocs2::loadData::loadStdVector(file, field + ".joints", frames.joints, false);
    return frames;
}

FrameDeclaration frameDeclarationFromFile(const std::string &file) {
    FrameDeclaration decl;
    ocs2::loadData::loadCppDataType(file, "root", decl.root);
    decl.legs[0] = limbFramesFromFile(file, "left_front");
    decl.legs[1] = limbFramesFromFile(file, "right_front");
    decl.legs[2] = limbFramesFromFile(file, "left_hind");
    decl.legs[3] = limbFramesFromFile(file, "right_hind");

    // Load arm configuration
    decl.arm = armFramesFromFile(file, "arm");

    std::vector<std::pair<std::string, ocs2::scalar_t>> collisionSpheres;
    ocs2::loadData::loadStdVectorOfPair(file, "collisions.collisionSpheres", collisionSpheres, false);

    ocs2::matrix_t offsets(collisionSpheres.size(), 3);
    ocs2::loadData::loadEigenMatrix(file, "collisions.collisionOffsets", offsets);

    decl.collisions.reserve(collisionSpheres.size());
    for (int i = 0; i < collisionSpheres.size(); ++i) {
        CollisionDeclaration collisionDeclaration;
        collisionDeclaration.link = collisionSpheres[i].first;
        collisionDeclaration.radius = collisionSpheres[i].second;
        collisionDeclaration.offset = offsets.row(i).transpose();
        decl.collisions.push_back(std::move(collisionDeclaration));
    }

    return decl;
}

}  // namespace tbai::mpc::quadruped_arm