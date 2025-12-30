cmake_minimum_required(VERSION 3.16)
project(ocs2_perceptive)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../ocs2_core/cmake/ocs2_cxx_flags.cmake)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/end_effector/EndEffectorDistanceConstraint.cpp
  src/end_effector/EndEffectorDistanceConstraintCppAd.cpp
)
add_library(ocs2::perceptive ALIAS ${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ocs2_core
    ocs2_robotic_tools
    Eigen3::Eigen
    Boost::system
    Boost::filesystem
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
  add_executable(test_bilinear_interpolation
    test/interpolation/testBilinearInterpolation.cpp
  )
  target_link_libraries(test_bilinear_interpolation
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  gtest_discover_tests(test_bilinear_interpolation)

  add_executable(test_trilinear_interpolation
    test/interpolation/testTrilinearInterpolation.cpp
  )
  target_link_libraries(test_trilinear_interpolation
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  gtest_discover_tests(test_trilinear_interpolation)
endif()
