cmake_minimum_required(VERSION 3.16)
project(ocs2_robotic_tools)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../ocs2_core/cmake/ocs2_cxx_flags.cmake)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/common/RotationTransforms.cpp
  src/common/LoopshapingRobotInterface.cpp
)
add_library(ocs2::robotic_tools ALIAS ${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ocs2_core
    ocs2_oc
    Eigen3::Eigen
)

# Dummy target for clang tooling
add_executable(${PROJECT_NAME}_lintTarget src/lintTarget.cpp)
target_link_libraries(${PROJECT_NAME}_lintTarget ${PROJECT_NAME})

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
  add_executable(rotation_transform_tests
    test/common/TestRotationTransforms.cpp
    test/common/TestRotationDerivativesTransforms.cpp
  )
  target_link_libraries(rotation_transform_tests
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  target_compile_options(rotation_transform_tests PRIVATE ${OCS2_CXX_FLAGS})
  gtest_discover_tests(rotation_transform_tests)
endif()
