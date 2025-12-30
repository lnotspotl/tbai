cmake_minimum_required(VERSION 3.16)
project(ocs2_pinocchio_interface)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../../ocs2_core/cmake/ocs2_cxx_flags.cmake)

find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add pinocchio configurations
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(pinocchio_config)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/PinocchioInterface.cpp
  src/PinocchioInterfaceCppAd.cpp
  src/PinocchioEndEffectorKinematics.cpp
  src/PinocchioEndEffectorKinematicsCppAd.cpp
  src/urdf.cpp
)
add_library(ocs2::pinocchio_interface ALIAS ${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} SYSTEM
  PUBLIC
    $<BUILD_INTERFACE:${pinocchio_INCLUDE_DIRS}>
)

target_link_directories(${PROJECT_NAME}
  PUBLIC
    ${pinocchio_LIBRARY_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ocs2_core
    ocs2_robotic_tools
    Eigen3::Eigen
    ${pinocchio_LIBRARIES}
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
  add_executable(testPinocchioInterface
    test/testPinocchioInterface.cpp
    test/testPinocchioEndEffectorKinematics.cpp
  )
  target_link_libraries(testPinocchioInterface
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  gtest_discover_tests(testPinocchioInterface)
endif()
