cmake_minimum_required(VERSION 3.16)
project(ocs2_sphere_approximation)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../../ocs2_core/cmake/ocs2_cxx_flags.cmake)

find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tinyxml2 REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

add_library(${PROJECT_NAME}
  src/SphereApproximation.cpp
  src/PinocchioSphereInterface.cpp
  src/PinocchioSphereKinematics.cpp
  src/PinocchioSphereKinematicsCppAd.cpp
)
add_library(ocs2::sphere_approximation ALIAS ${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} SYSTEM
  PUBLIC
    ${pinocchio_INCLUDE_DIRS}
    ${tinyxml2_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME}
  PUBLIC
    ${pinocchio_LIBRARY_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ocs2_core
    ocs2_robotic_tools
    ocs2_pinocchio_interface
    Eigen3::Eigen
    ${pinocchio_LIBRARIES}
    ${tinyxml2_LIBRARIES}
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

# Tests require ocs2_robotic_assets which is not included
# if(OCS2_BUILD_TESTS)
#   add_executable(PinocchioSphereKinematicsTest
#     test/testPinocchioSphereKinematics.cpp
#   )
#   target_link_libraries(PinocchioSphereKinematicsTest
#     ${PROJECT_NAME}
#     GTest::gtest_main
#   )
#   gtest_discover_tests(PinocchioSphereKinematicsTest)
# endif()
