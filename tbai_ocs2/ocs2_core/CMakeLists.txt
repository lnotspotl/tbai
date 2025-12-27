cmake_minimum_required(VERSION 3.16)
project(ocs2_core)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ocs2_cxx_flags.cmake)

# Collect all source files
set(OCS2_CORE_SOURCES
  src/Types.cpp
  src/augmented_lagrangian/AugmentedLagrangian.cpp
  src/augmented_lagrangian/StateAugmentedLagrangian.cpp
  src/augmented_lagrangian/StateInputAugmentedLagrangian.cpp
  src/augmented_lagrangian/StateAugmentedLagrangianCollection.cpp
  src/augmented_lagrangian/StateInputAugmentedLagrangianCollection.cpp
  src/automatic_differentation/CppAdInterface.cpp
  src/automatic_differentation/CppAdSparsity.cpp
  src/automatic_differentation/FiniteDifferenceMethods.cpp
  src/constraint/StateConstraintCppAd.cpp
  src/constraint/StateInputConstraintCppAd.cpp
  src/constraint/StateConstraintCollection.cpp
  src/constraint/StateInputConstraintCollection.cpp
  src/constraint/LinearStateConstraint.cpp
  src/constraint/LinearStateInputConstraint.cpp
  src/control/FeedforwardController.cpp
  src/control/LinearController.cpp
  src/control/StateBasedLinearController.cpp
  src/cost/QuadraticStateCost.cpp
  src/cost/QuadraticStateInputCost.cpp
  src/cost/StateCostCollection.cpp
  src/cost/StateCostCppAd.cpp
  src/cost/StateInputCostCollection.cpp
  src/cost/StateInputCostCppAd.cpp
  src/cost/StateInputGaussNewtonCostAd.cpp
  src/dynamics/ControlledSystemBase.cpp
  src/dynamics/LinearSystemDynamics.cpp
  src/dynamics/SystemDynamicsBase.cpp
  src/dynamics/SystemDynamicsBaseAD.cpp
  src/dynamics/SystemDynamicsLinearizer.cpp
  src/dynamics/TransferFunctionBase.cpp
  src/integration/SensitivityIntegrator.cpp
  src/integration/SensitivityIntegratorImpl.cpp
  src/integration/Integrator.cpp
  src/integration/IntegratorBase.cpp
  src/integration/RungeKuttaDormandPrince5.cpp
  src/integration/OdeBase.cpp
  src/integration/Observer.cpp
  src/integration/StateTriggeredEventHandler.cpp
  src/integration/SystemEventHandler.cpp
  src/reference/ModeSchedule.cpp
  src/reference/TargetTrajectories.cpp
  src/loopshaping/LoopshapingDefinition.cpp
  src/loopshaping/LoopshapingPropertyTree.cpp
  src/loopshaping/LoopshapingFilter.cpp
  src/loopshaping/LoopshapingPreComputation.cpp
  src/loopshaping/cost/LoopshapingCost.cpp
  src/loopshaping/cost/LoopshapingStateCost.cpp
  src/loopshaping/cost/LoopshapingStateInputCost.cpp
  src/loopshaping/cost/LoopshapingCostEliminatePattern.cpp
  src/loopshaping/cost/LoopshapingCostOutputPattern.cpp
  src/loopshaping/soft_constraint/LoopshapingSoftConstraint.cpp
  src/loopshaping/soft_constraint/LoopshapingStateInputSoftConstraint.cpp
  src/loopshaping/soft_constraint/LoopshapingSoftConstraintEliminatePattern.cpp
  src/loopshaping/soft_constraint/LoopshapingSoftConstraintOutputPattern.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangian.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingStateAugmentedLagrangian.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingStateInputAugmentedLagrangian.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangianEliminatePattern.cpp
  src/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangianOutputPattern.cpp
  src/loopshaping/constraint/LoopshapingConstraint.cpp
  src/loopshaping/constraint/LoopshapingStateConstraint.cpp
  src/loopshaping/constraint/LoopshapingStateInputConstraint.cpp
  src/loopshaping/constraint/LoopshapingConstraintEliminatePattern.cpp
  src/loopshaping/constraint/LoopshapingConstraintOutputPattern.cpp
  src/loopshaping/dynamics/LoopshapingDynamics.cpp
  src/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.cpp
  src/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.cpp
  src/loopshaping/dynamics/LoopshapingFilterDynamics.cpp
  src/loopshaping/initialization/LoopshapingInitializer.cpp
  src/model_data/ModelData.cpp
  src/model_data/Metrics.cpp
  src/model_data/Multiplier.cpp
  src/misc/LinearAlgebra.cpp
  src/misc/Log.cpp
  src/soft_constraint/StateSoftConstraint.cpp
  src/soft_constraint/StateInputSoftConstraint.cpp
  src/soft_constraint/StateInputSoftBoxConstraint.cpp
  src/penalties/MultidimensionalPenalty.cpp
  src/penalties/Penalties.cpp
  src/penalties/penalties/RelaxedBarrierPenalty.cpp
  src/penalties/penalties/SquaredHingePenalty.cpp
  src/thread_support/ThreadPool.cpp
)

# Create the main library
add_library(${PROJECT_NAME} ${OCS2_CORE_SOURCES})
add_library(ocs2::core ALIAS ${PROJECT_NAME})

# Set target properties
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    Boost::system
    Boost::filesystem
    Boost::log_setup
    Boost::log
    OpenMP::OpenMP_CXX
    Threads::Threads
    ${CMAKE_DL_LIBS}
    cppadcg
)

# Lint target for clang tooling
add_executable(${PROJECT_NAME}_lintTarget src/lintTarget.cpp)
target_link_libraries(${PROJECT_NAME}_lintTarget
  PRIVATE
    ${PROJECT_NAME}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ${PROJECT_NAME}_lintTarget")
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
      ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif()

#############
## Install ##
#############

include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
  EXPORT ocs2Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(DIRECTORY test/include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(EXPORT ocs2Targets
  FILE ocs2Targets.cmake
  NAMESPACE ocs2::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ocs2
)

#############
## Testing ##
#############

if(OCS2_BUILD_TESTS)
  add_executable(test_control
    test/control/testLinearController.cpp
    test/control/testFeedforwardController.cpp
  )
  target_link_libraries(test_control
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_control)

  add_executable(initialization_unittest
    test/initialization/InitializationTest.cpp
  )
  target_link_libraries(initialization_unittest
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(initialization_unittest)

  add_executable(test_integration
    test/integration/testSensitivityIntegrator.cpp
    test/integration/IntegrationTest.cpp
    test/integration/testRungeKuttaDormandPrince5.cpp
    test/integration/TrapezoidalIntegrationTest.cpp
  )
  target_link_libraries(test_integration
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_integration)

  add_executable(interpolation_unittest
    test/misc/testInterpolation.cpp
  )
  target_link_libraries(interpolation_unittest
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(interpolation_unittest)

  add_executable(${PROJECT_NAME}_cppadcg
    test/cppad_cg/testCppADCG_dynamics.cpp
    test/cppad_cg/testSparsityHelpers.cpp
    test/cppad_cg/testCppAdInterface.cpp
  )
  target_link_libraries(${PROJECT_NAME}_cppadcg
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(${PROJECT_NAME}_cppadcg)

  add_executable(test_transferfunctionbase
    test/dynamics/testTransferfunctionBase.cpp
  )
  target_link_libraries(test_transferfunctionbase
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_transferfunctionbase)

  add_executable(${PROJECT_NAME}_loopshaping
    test/loopshaping/testLoopshapingConfiguration.cpp
    test/loopshaping/testLoopshapingAugmentedLagrangian.cpp
    test/loopshaping/testLoopshapingConstraint.cpp
    test/loopshaping/testLoopshapingCost.cpp
    test/loopshaping/testLoopshapingSoftConstraint.cpp
    test/loopshaping/testLoopshapingDefinition.cpp
    test/loopshaping/testLoopshapingDynamics.cpp
    test/loopshaping/testLoopshapingFilterDynamics.cpp
    test/loopshaping/testLoopshapingPreComputation.cpp
  )
  target_link_libraries(${PROJECT_NAME}_loopshaping
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
      $<$<CXX_COMPILER_ID:GNU>:stdc++fs>
  )
  gtest_discover_tests(${PROJECT_NAME}_loopshaping)

  add_executable(${PROJECT_NAME}_test_misc
    test/misc/testInterpolation.cpp
    test/misc/testLinearAlgebra.cpp
    test/misc/testLogging.cpp
    test/misc/testLoadData.cpp
    test/misc/testLookup.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_misc
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(${PROJECT_NAME}_test_misc)

  add_executable(test_dynamics
    test/dynamics/testSystemDynamicsLinearizer.cpp
    test/dynamics/testSystemDynamicsPreComputation.cpp
  )
  target_link_libraries(test_dynamics
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_dynamics)

  add_executable(test_cost
    test/cost/testCostCollection.cpp
    test/cost/testCostCppAd.cpp
    test/cost/testQuadraticCostFunction.cpp
  )
  target_link_libraries(test_cost
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_cost)

  add_executable(test_constraint
    test/constraint/testConstraintCollection.cpp
    test/constraint/testConstraintCppAd.cpp
    test/constraint/testLinearConstraint.cpp
  )
  target_link_libraries(test_constraint
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_constraint)

  add_executable(test_metrics
    test/model_data/testMetrics.cpp
  )
  target_link_libraries(test_metrics
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_metrics)

  add_executable(test_multiplier
    test/model_data/testMultiplier.cpp
  )
  target_link_libraries(test_multiplier
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_multiplier)

  add_executable(test_ModelData
    test/model_data/testModelData.cpp
  )
  target_link_libraries(test_ModelData
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_ModelData)

  add_executable(test_ModeSchedule
    test/reference/testModeSchedule.cpp
  )
  target_link_libraries(test_ModeSchedule
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_ModeSchedule)

  add_executable(test_softConstraint
    test/soft_constraint/testSoftConstraint.cpp
    test/soft_constraint/testDoubleSidedPenalty.cpp
  )
  target_link_libraries(test_softConstraint
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_softConstraint)

  add_executable(${PROJECT_NAME}_test_thread_support
    test/thread_support/testBufferedValue.cpp
    test/thread_support/testSynchronized.cpp
    test/thread_support/testThreadPool.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_thread_support
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(${PROJECT_NAME}_test_thread_support)

  add_executable(${PROJECT_NAME}_test_core
    test/testPrecomputation.cpp
    test/testTypes.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_core
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(${PROJECT_NAME}_test_core)
endif()
