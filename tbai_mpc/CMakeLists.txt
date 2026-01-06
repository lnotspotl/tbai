cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_mpc)

add_library(tbai_quadruped_mpc
    # wbc general
    src/wbc/HqpSolver.cpp
    src/wbc/SqpSolver.cpp
    src/wbc/Task.cpp

    # wbc quadruped specific
    src/quadruped_wbc/Factory.cpp
    src/quadruped_wbc/HqpWbc.cpp
    src/quadruped_wbc/SqpWbc.cpp
    src/quadruped_wbc/WbcBase.cpp

    # mpc
    src/quadruped_mpc/QuadrupedInterface.cpp
    src/quadruped_mpc/QuadrupedMpc.cpp
    src/quadruped_mpc/QuadrupedPointfootInterface.cpp

    src/quadruped_mpc/analytical_inverse_kinematics/AnalyticalInverseKinematics.cpp
    src/quadruped_mpc/analytical_inverse_kinematics/LegInverseKinematicParameters.cpp

    src/quadruped_mpc/constraint/EndEffectorVelocityConstraint.cpp
    src/quadruped_mpc/constraint/FootNormalConstraint.cpp
    src/quadruped_mpc/constraint/FrictionConeConstraint.cpp
    src/quadruped_mpc/constraint/ZeroForceConstraint.cpp

    src/quadruped_mpc/core/ComModelBase.cpp
    src/quadruped_mpc/core/KinematicsModelBase.cpp
    src/quadruped_mpc/core/ModelSettings.cpp
    src/quadruped_mpc/core/SwitchedModelPrecomputation.cpp
    src/quadruped_mpc/core/TorqueApproximation.cpp

    src/quadruped_mpc/cost/CollisionAvoidanceCost.cpp
    src/quadruped_mpc/cost/FootPlacementCost.cpp
    src/quadruped_mpc/cost/FrictionConeCost.cpp
    src/quadruped_mpc/cost/LinearStateInequalitySoftConstraint.cpp
    src/quadruped_mpc/cost/MotionTrackingCost.cpp
    src/quadruped_mpc/cost/MotionTrackingTerminalCost.cpp
    src/quadruped_mpc/cost/TorqueLimitsSoftConstraint.cpp

    src/quadruped_mpc/dynamics/ComKinoDynamicsParameters.cpp
    src/quadruped_mpc/dynamics/ComKinoSystemDynamicsAd.cpp

    src/quadruped_mpc/foot_planner/CubicSpline.cpp
    src/quadruped_mpc/foot_planner/FootPhase.cpp
    src/quadruped_mpc/foot_planner/KinematicFootPlacementPenalty.cpp
    src/quadruped_mpc/foot_planner/QuinticSplineSwing.cpp
    src/quadruped_mpc/foot_planner/SplineCpg.cpp
    src/quadruped_mpc/foot_planner/SwingSpline3d.cpp
    src/quadruped_mpc/foot_planner/SwingTrajectoryPlanner.cpp

    src/quadruped_mpc/initialization/ComKinoInitializer.cpp

    src/quadruped_mpc/logic/DynamicsParametersSynchronizedModule.cpp
    src/quadruped_mpc/logic/Gait.cpp
    src/quadruped_mpc/logic/GaitAdaptation.cpp
    src/quadruped_mpc/logic/GaitSchedule.cpp
    src/quadruped_mpc/logic/ModeSequenceTemplate.cpp
    src/quadruped_mpc/logic/SingleLegLogic.cpp
    src/quadruped_mpc/logic/SwitchedModelModeScheduleManager.cpp

    src/quadruped_mpc/quadruped_models/AnymalModels.cpp
    src/quadruped_mpc/quadruped_models/FrameDeclaration.cpp
    src/quadruped_mpc/quadruped_models/Go2InverseKinematics.cpp
    src/quadruped_mpc/quadruped_models/QuadrupedCom.cpp
    src/quadruped_mpc/quadruped_models/QuadrupedInverseKinematics.cpp
    src/quadruped_mpc/quadruped_models/QuadrupedKinematics.cpp
    src/quadruped_mpc/quadruped_models/QuadrupedPinocchioMapping.cpp

    # concrete interfaces
    src/quadruped_mpc/quadruped_interfaces/Interfaces.cpp

    # terrain
    src/quadruped_mpc/terrain/PlaneFitting.cpp
    src/quadruped_mpc/terrain/PlanarSignedDistanceField.cpp
    src/quadruped_mpc/terrain/PlanarTerrainModel.cpp
    src/quadruped_mpc/terrain/TerrainPlane.cpp

    # reference trajectory generator
    src/quadruped_mpc/quadruped_reference/LocalTerrainEstimator.cpp
    src/quadruped_mpc/quadruped_reference/ReferenceTrajectoryGenerator.cpp

    # quadruped commands
    src/quadruped_mpc/quadruped_commands/ReferenceExtrapolation.cpp
    src/quadruped_mpc/quadruped_commands/TerrainAdaptation.cpp

    # controller
    src/quadruped_mpc/MpcController.cpp
)

target_include_directories(tbai_quadruped_mpc
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_quadruped_mpc
    PUBLIC
        tbai_core
        tbai_reference
        ocs2_core
        ocs2_ddp
        ocs2_sqp
        ocs2_mpc
        ocs2_pinocchio_interface
        ocs2::qpoases
)

## Franka MPC library
add_library(tbai_franka_mpc
    src/franka_mpc/FactoryFunctions.cpp
    src/franka_mpc/MobileManipulatorPinocchioMapping.cpp
    src/franka_mpc/MobileManipulatorPreComputation.cpp
    src/franka_mpc/MobileManipulatorInterface.cpp
    src/franka_mpc/dynamics/DefaultManipulatorDynamics.cpp
    src/franka_mpc/constraint/EndEffectorConstraint.cpp
)
add_library(tbai::franka_mpc ALIAS tbai_franka_mpc)

target_include_directories(tbai_franka_mpc
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_franka_mpc
    PUBLIC
        ocs2_core
        ocs2_ddp
        ocs2_mpc
        ocs2_pinocchio_interface
        ocs2_robotic_tools
)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_quadruped_mpc tbai_franka_mpc
    EXPORT tbai_quadruped_mpc-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_quadruped_mpc-targets
    NAMESPACE tbai::
    FILE tbai-quadruped-mpc-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)