cmake_minimum_required(VERSION 3.16)
project(ocs2_sqp)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../ocs2_core/cmake/ocs2_cxx_flags.cmake)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/SqpLogging.cpp
  src/SqpSettings.cpp
  src/SqpSolver.cpp
)
add_library(ocs2::sqp ALIAS ${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ocs2_core
    ocs2_mpc
    ocs2_oc
    ocs2_qp_solver
    ocs2_hpipm
    Eigen3::Eigen
    Boost::system
    Boost::filesystem
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
    test/testCircularKinematics.cpp
    test/testSwitchedProblem.cpp
    test/testUnconstrained.cpp
    test/testValuefunction.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  gtest_discover_tests(test_${PROJECT_NAME})
endif()
