cmake_minimum_required(VERSION 3.16)
project(ocs2_ddp)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../ocs2_core/cmake/ocs2_cxx_flags.cmake)

# Collect all source files
set(OCS2_DDP_SOURCES
  src/riccati_equations/ContinuousTimeRiccatiEquations.cpp
  src/riccati_equations/DiscreteTimeRiccatiEquations.cpp
  src/riccati_equations/RiccatiModification.cpp
  src/search_strategy/LevenbergMarquardtStrategy.cpp
  src/search_strategy/LineSearchStrategy.cpp
  src/search_strategy/StrategySettings.cpp
  src/ContinuousTimeLqr.cpp
  src/GaussNewtonDDP.cpp
  src/HessianCorrection.cpp
  src/ILQR.cpp
  src/SLQ.cpp
  src/DDP_Settings.cpp
  src/DDP_HelperFunctions.cpp
)

# Create the main library
add_library(${PROJECT_NAME} ${OCS2_DDP_SOURCES})
add_library(ocs2::ddp ALIAS ${PROJECT_NAME})

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
    ocs2_oc
    ocs2_mpc
    ocs2_qp_solver
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
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
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
  add_executable(exp0_ddp_test
    test/Exp0Test.cpp
  )
  target_link_libraries(exp0_ddp_test
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(exp0_ddp_test)

  add_executable(exp1_ddp_test
    test/Exp1Test.cpp
  )
  target_link_libraries(exp1_ddp_test
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(exp1_ddp_test)

  add_executable(correctness_test
    test/CorrectnessTest.cpp
  )
  target_link_libraries(correctness_test
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(correctness_test)

  add_executable(riccati_ode_test
    test/RiccatiTest.cpp
  )
  target_link_libraries(riccati_ode_test
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(riccati_ode_test)

  add_executable(circular_kinematics_ddp_test
    test/CircularKinematicsTest.cpp
  )
  target_link_libraries(circular_kinematics_ddp_test
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(circular_kinematics_ddp_test)

  # TODO(lnotspotl): Fix this test, re-enable once the issue is resolved
  # add_executable(hybrid_slq_test
  #   test/HybridSlqTest.cpp
  # )
  # target_link_libraries(hybrid_slq_test
  #   PRIVATE
  #     ${PROJECT_NAME}
  #     GTest::gtest_main
  # )
  # gtest_discover_tests(hybrid_slq_test)

  add_executable(bouncing_mass_test
    test/bouncingmass/BouncingMassTest.cpp
    test/bouncingmass/OverallReference.cpp
    test/bouncingmass/Reference.cpp
  )
  target_link_libraries(bouncing_mass_test
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(bouncing_mass_test)

  add_executable(testContinuousTimeLqr
    test/testContinuousTimeLqr.cpp
  )
  target_link_libraries(testContinuousTimeLqr
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(testContinuousTimeLqr)

  add_executable(testDdpHelperFunction
    test/testDdpHelperFunction.cpp
  )
  target_link_libraries(testDdpHelperFunction
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(testDdpHelperFunction)

  add_executable(testReachingTask
    test/testReachingTask.cpp
  )
  target_link_libraries(testReachingTask
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(testReachingTask)
endif()
