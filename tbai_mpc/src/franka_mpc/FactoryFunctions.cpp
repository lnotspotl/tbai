// Boost MPL compatibility fix
#ifndef BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif
#ifndef BOOST_MPL_LIMIT_VECTOR_SIZE
#define BOOST_MPL_LIMIT_VECTOR_SIZE 50
#endif

#include "tbai_mpc/franka_mpc/FactoryFunctions.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ocs2 {
namespace franka {

PinocchioInterface createPinocchioInterface(const std::string &robotUrdfPath) {
    return getPinocchioInterfaceFromUrdfFile(robotUrdfPath);
}

PinocchioInterface createPinocchioInterface(const std::string &robotUrdfPath,
                                            const std::vector<std::string> &jointNames) {
    using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

    const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
    ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
    for (joint_pair_t &jointPair : newModel->joints_) {
        if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) != jointNames.end()) {
            jointPair.second->type = urdf::Joint::FIXED;
        }
    }

    return getPinocchioInterfaceFromUrdfModel(newModel);
}

FrankaModelInfo createFrankaModelInfo(const PinocchioInterface &interface,
                                      const std::string &baseFrame, const std::string &eeFrame) {
    const auto &model = interface.getModel();

    FrankaModelInfo info;
    info.stateDim = model.nq;
    info.inputDim = info.stateDim;
    info.armDim = info.inputDim;
    info.eeFrame = eeFrame;
    info.baseFrame = baseFrame;
    const auto &jointNames = model.names;
    info.dofNames = std::vector<std::string>(jointNames.end() - info.armDim, jointNames.end());

    return info;
}

}  // namespace franka
}  // namespace ocs2
