cmake_minimum_required(VERSION 3.16)
project(ocs2_centroidal_model)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../../ocs2_core/cmake/ocs2_cxx_flags.cmake)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)

###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

add_library(${PROJECT_NAME}
  src/PinocchioCentroidalDynamics.cpp
  src/PinocchioCentroidalDynamicsAD.cpp
  src/CentroidalModelRbdConversions.cpp
  src/CentroidalModelPinocchioMapping.cpp
  src/CentroidalModelInfo.cpp
  src/FactoryFunctions.cpp
  src/ModelHelperFunctions.cpp
)
add_library(ocs2::centroidal_model ALIAS ${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} SYSTEM
  PUBLIC
    ${pinocchio_INCLUDE_DIRS}
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
    Boost::system
    Boost::filesystem
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