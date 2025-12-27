cmake_minimum_required(VERSION 3.16)
project(ocs2_ipm)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../ocs2_core/cmake/ocs2_cxx_flags.cmake)

# Collect all source files
set(OCS2_IPM_SOURCES
  src/IpmHelpers.cpp
  src/IpmInitialization.cpp
  src/IpmPerformanceIndexComputation.cpp
  src/IpmSettings.cpp
  src/IpmSolver.cpp
)

# Create the main library
add_library(${PROJECT_NAME} ${OCS2_IPM_SOURCES})
add_library(ocs2::ipm ALIAS ${PROJECT_NAME})

# Set target properties
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ocs2_core
    ocs2_oc
    ocs2_mpc
    ocs2_hpipm
)

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

#############
## Testing ##
#############

if(OCS2_BUILD_TESTS)
  add_executable(test_${PROJECT_NAME}
    test/Exp0Test.cpp
    test/Exp1Test.cpp
    test/testCircularKinematics.cpp
    test/testSwitchedProblem.cpp
    test/testUnconstrained.cpp
    test/testValuefunction.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_${PROJECT_NAME})
endif()
