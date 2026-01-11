// Boost MPL compatibility fix
#ifndef BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif
#ifndef BOOST_MPL_LIMIT_VECTOR_SIZE
#define BOOST_MPL_LIMIT_VECTOR_SIZE 50
#endif

#include "tbai_mpc/arm_mpc/ArmInterface.h"

#include <string>

#include "tbai_mpc/arm_mpc/ArmModelInfo.h"
#include "tbai_mpc/arm_mpc/ArmPreComputation.h"
#include "tbai_mpc/arm_mpc/constraint/EndEffectorConstraint.h"
#include "tbai_mpc/arm_mpc/cost/QuadraticInputCost.h"
#include "tbai_mpc/arm_mpc/dynamics/ArmDynamics.h"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

namespace tbai::mpc::arm {

ArmInterface::ArmInterface(const std::string &taskFile, const std::string &libraryFolder,
                                 const std::string &urdfFile) {
    boost::filesystem::path taskFilePath(taskFile);
    if (boost::filesystem::exists(taskFilePath)) {
        std::cerr << "[ArmInterface] Loading task file: " << taskFilePath << std::endl;
    } else {
        throw std::invalid_argument("[ArmInterface] Task file not found: " + taskFilePath.string());
    }

    boost::filesystem::path urdfFilePath(urdfFile);
    if (boost::filesystem::exists(urdfFilePath)) {
        std::cerr << "[ArmInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
    } else {
        throw std::invalid_argument("[ArmInterface] URDF file not found: " + urdfFilePath.string());
    }

    boost::filesystem::path libraryFolderPath(libraryFolder);
    boost::filesystem::create_directories(libraryFolderPath);
    std::cerr << "[ArmInterface] Generated library path: " << libraryFolderPath << std::endl;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    std::vector<std::string> removeJointNames;
    ocs2::loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
    std::string baseFrame, eeFrame;
    ocs2::loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
    ocs2::loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);

    std::cerr << "\n #### Model Information:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### model_information.removeJoints: ";
    for (const auto &name : removeJointNames) {
        std::cerr << "\"" << name << "\" ";
    }
    std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";
    std::cerr << "\n #### model_information.eeFrame: \"" << eeFrame << "\"" << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;

    pinocchioInterfacePtr_.reset(new ocs2::PinocchioInterface(createPinocchioInterface(urdfFile, removeJointNames)));
    std::cerr << *pinocchioInterfacePtr_;

    manipulatorModelInfo_ = arm::createArmModelInfo(*pinocchioInterfacePtr_, baseFrame, eeFrame);

    bool usePreComputation = true;
    bool recompileLibraries = true;
    std::cerr << "\n #### Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
    ocs2::loadData::loadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
    ocs2::loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
    std::cerr << " #### =============================================================================\n";

    initialState_.setZero(manipulatorModelInfo_.stateDim);
    const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
    const int armStateDim = manipulatorModelInfo_.armDim;

    // Fixed-base manipulator has no base state

    ocs2::vector_t initialArmState = ocs2::vector_t::Zero(armStateDim);
    ocs2::loadData::loadEigenMatrix(taskFile, "initialState.arm", initialArmState);
    initialState_.tail(armStateDim) = initialArmState;

    std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;

    ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp");
    mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc");

    referenceManagerPtr_.reset(new ocs2::ReferenceManager);

    problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));

    problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));
    problem_.stateSoftConstraintPtr->add(
        "endEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector", usePreComputation,
                                                libraryFolder, recompileLibraries));
    problem_.finalSoftConstraintPtr->add(
        "finalEndEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector",
                                                     usePreComputation, libraryFolder, recompileLibraries));

    problem_.dynamicsPtr.reset(
        new ArmDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));

    if (usePreComputation) {
        problem_.preComputationPtr.reset(new ArmPreComputation(*pinocchioInterfacePtr_, manipulatorModelInfo_));
    }

    const auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout");
    rolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

    initializerPtr_.reset(new ocs2::DefaultInitializer(manipulatorModelInfo_.inputDim));
}

std::unique_ptr<ocs2::StateInputCost> ArmInterface::getQuadraticInputCost(const std::string &taskFile) {
    ocs2::matrix_t R = ocs2::matrix_t::Zero(manipulatorModelInfo_.inputDim, manipulatorModelInfo_.inputDim);
    const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
    const int armStateDim = manipulatorModelInfo_.armDim;

    // Fixed-base manipulator has no base input

    ocs2::matrix_t R_arm = ocs2::matrix_t::Zero(armStateDim, armStateDim);
    ocs2::loadData::loadEigenMatrix(taskFile, "inputCost.R.arm", R_arm);
    R.bottomRightCorner(armStateDim, armStateDim) = R_arm;

    std::cerr << "\n #### Input Cost Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "inputCost.R:  \n" << R << '\n';
    std::cerr << " #### =============================================================================\n";

    return std::make_unique<QuadraticInputCost>(std::move(R), manipulatorModelInfo_.stateDim);
}

std::unique_ptr<ocs2::StateCost> ArmInterface::getEndEffectorConstraint(const ocs2::PinocchioInterface &pinocchioInterface,
                                                                     const std::string &taskFile,
                                                                     const std::string &prefix, bool usePreComputation,
                                                                     const std::string &libraryFolder,
                                                                     bool recompileLibraries) {
    ocs2::scalar_t muPosition = 1.0;
    ocs2::scalar_t muOrientation = 1.0;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::cerr << "\n #### " << prefix << " Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    ocs2::loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
    ocs2::loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
    std::cerr << " #### =============================================================================\n";

    if (referenceManagerPtr_ == nullptr) {
        throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
    }

    std::unique_ptr<ocs2::StateConstraint> constraint;
    if (usePreComputation) {
        ArmPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
        ocs2::PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping,
                                                    {manipulatorModelInfo_.eeFrame});
        constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
    } else {
        ArmPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
        ocs2::PinocchioEndEffectorKinematicsCppAd eeKinematics(
            pinocchioInterface, pinocchioMappingCppAd, {manipulatorModelInfo_.eeFrame}, manipulatorModelInfo_.stateDim,
            manipulatorModelInfo_.inputDim, "end_effector_kinematics", libraryFolder, recompileLibraries, false);
        constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
    }

    std::vector<std::unique_ptr<ocs2::PenaltyBase>> penaltyArray(6);
    std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<ocs2::QuadraticPenalty>(muPosition); });
    std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<ocs2::QuadraticPenalty>(muOrientation); });

    return std::make_unique<ocs2::StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

std::unique_ptr<ocs2::StateInputCost> ArmInterface::getJointLimitSoftConstraint(
    const ocs2::PinocchioInterface &pinocchioInterface, const std::string &taskFile) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    bool activateJointPositionLimit = true;
    ocs2::loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

    const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
    const int armStateDim = manipulatorModelInfo_.armDim;
    const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
    const int armInputDim = manipulatorModelInfo_.armDim;
    const auto &model = pinocchioInterface.getModel();

    std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
    if (activateJointPositionLimit) {
        ocs2::scalar_t muPositionLimits = 1e-2;
        ocs2::scalar_t deltaPositionLimits = 1e-3;

        const ocs2::vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
        const ocs2::vector_t upperBound = model.upperPositionLimit.tail(armStateDim);

        std::cerr << "\n #### JointPositionLimits Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
        std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
        ocs2::loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
        ocs2::loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
        std::cerr << " #### =============================================================================\n";

        stateLimits.reserve(armStateDim);
        for (int i = 0; i < armStateDim; ++i) {
            ocs2::StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
            boxConstraint.index = baseStateDim + i;
            boxConstraint.lowerBound = lowerBound(i);
            boxConstraint.upperBound = upperBound(i);
            boxConstraint.penaltyPtr.reset(new ocs2::RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
            stateLimits.push_back(std::move(boxConstraint));
        }
    }

    std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
    {
        ocs2::vector_t lowerBound = ocs2::vector_t::Zero(manipulatorModelInfo_.inputDim);
        ocs2::vector_t upperBound = ocs2::vector_t::Zero(manipulatorModelInfo_.inputDim);
        ocs2::scalar_t muVelocityLimits = 1e-2;
        ocs2::scalar_t deltaVelocityLimits = 1e-3;

        // Fixed-base manipulator has no base velocity limits

        ocs2::vector_t lowerBoundArm = ocs2::vector_t::Zero(armInputDim);
        ocs2::vector_t upperBoundArm = ocs2::vector_t::Zero(armInputDim);
        ocs2::loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
        ocs2::loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
        lowerBound.tail(armInputDim) = lowerBoundArm;
        upperBound.tail(armInputDim) = upperBoundArm;

        std::cerr << "\n #### JointVelocityLimits Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
        std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
        ocs2::loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
        ocs2::loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
        std::cerr << " #### =============================================================================\n";

        inputLimits.reserve(manipulatorModelInfo_.inputDim);
        for (int i = 0; i < manipulatorModelInfo_.inputDim; ++i) {
            ocs2::StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
            boxConstraint.index = i;
            boxConstraint.lowerBound = lowerBound(i);
            boxConstraint.upperBound = upperBound(i);
            boxConstraint.penaltyPtr.reset(new ocs2::RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
            inputLimits.push_back(std::move(boxConstraint));
        }
    }

    auto boxConstraints = std::make_unique<ocs2::StateInputSoftBoxConstraint>(stateLimits, inputLimits);
    boxConstraints->initializeOffset(0.0, ocs2::vector_t::Zero(manipulatorModelInfo_.stateDim),
                                     ocs2::vector_t::Zero(manipulatorModelInfo_.inputDim));
    return boxConstraints;
}

}  // namespace tbai::mpc::arm
