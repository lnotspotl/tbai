cmake_minimum_required(VERSION 3.16)
project(ocs2_oc)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../ocs2_core/cmake/ocs2_cxx_flags.cmake)

# Collect all source files
set(OCS2_OC_SOURCES
  src/approximate_model/ChangeOfInputVariables.cpp
  src/approximate_model/LinearQuadraticApproximator.cpp
  src/multiple_shooting/Helpers.cpp
  src/multiple_shooting/Initialization.cpp
  src/multiple_shooting/LagrangianEvaluation.cpp
  src/multiple_shooting/MetricsComputation.cpp
  src/multiple_shooting/PerformanceIndexComputation.cpp
  src/multiple_shooting/ProjectionMultiplierCoefficients.cpp
  src/multiple_shooting/Transcription.cpp
  src/oc_data/LoopshapingPrimalSolution.cpp
  src/oc_data/PerformanceIndex.cpp
  src/oc_data/TimeDiscretization.cpp
  src/oc_problem/OptimalControlProblem.cpp
  src/oc_problem/LoopshapingOptimalControlProblem.cpp
  src/oc_problem/OptimalControlProblemHelperFunction.cpp
  src/oc_problem/OcpSize.cpp
  src/oc_problem/OcpToKkt.cpp
  src/oc_solver/SolverBase.cpp
  src/precondition/Ruzi.cpp
  src/rollout/PerformanceIndicesRollout.cpp
  src/rollout/RolloutBase.cpp
  src/rollout/RootFinder.cpp
  src/rollout/InitializerRollout.cpp
  src/rollout/StateTriggeredRollout.cpp
  src/rollout/TimeTriggeredRollout.cpp
  src/rollout/RolloutSettings.cpp
  src/synchronized_module/ReferenceManager.cpp
  src/synchronized_module/LoopshapingReferenceManager.cpp
  src/synchronized_module/LoopshapingSynchronizedModule.cpp
  src/synchronized_module/SolverObserver.cpp
  src/search_strategy/FilterLinesearch.cpp
  src/trajectory_adjustment/TrajectorySpreading.cpp
)

# Create the main library
add_library(${PROJECT_NAME} ${OCS2_OC_SOURCES})
add_library(ocs2::oc ALIAS ${PROJECT_NAME})

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
    ocs2_core
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
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test/include
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

#############
## Testing ##
#############

if(OCS2_BUILD_TESTS)
  add_executable(test_${PROJECT_NAME}_multiple_shooting
    test/multiple_shooting/testProjectionMultiplierCoefficients.cpp
    test/multiple_shooting/testTranscriptionMetrics.cpp
    test/multiple_shooting/testTranscriptionPerformanceIndex.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_multiple_shooting
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_${PROJECT_NAME}_multiple_shooting)

  add_executable(test_${PROJECT_NAME}_data
    test/oc_data/testTimeDiscretization.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_data
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_${PROJECT_NAME}_data)

  add_executable(test_${PROJECT_NAME}_rollout
    test/rollout/testTimeTriggeredRollout.cpp
    test/rollout/testStateTriggeredRollout.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_rollout
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_${PROJECT_NAME}_rollout)

  add_executable(test_change_of_variables
    test/testChangeOfInputVariables.cpp
  )
  target_link_libraries(test_change_of_variables
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_change_of_variables)

  add_executable(test_trajectory_spreading
    test/trajectory_adjustment/TrajectorySpreadingTest.cpp
  )
  target_link_libraries(test_trajectory_spreading
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_trajectory_spreading)

  add_executable(test_ocp_to_kkt
    test/oc_problem/testOcpToKkt.cpp
  )
  target_link_libraries(test_ocp_to_kkt
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_ocp_to_kkt)

  add_executable(test_precondition
    test/precondition/testPrecondition.cpp
  )
  target_link_libraries(test_precondition
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_precondition)
endif()
