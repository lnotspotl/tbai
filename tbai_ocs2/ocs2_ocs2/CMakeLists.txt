cmake_minimum_required(VERSION 3.16)
project(ocs2_ocs2)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../ocs2_core/cmake/ocs2_cxx_flags.cmake)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem)

###########
## Build ##
###########

# Header-only library
add_library(${PROJECT_NAME} INTERFACE)
add_library(ocs2::ocs2 ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    ocs2_core
    ocs2_ddp
    ocs2_frank_wolfe
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
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

#############
## Testing ##
#############

# Tests are currently disabled in the original package
# if(OCS2_BUILD_TESTS)
#   add_executable(exp0_gddp_test test/exp0_gddp_test.cpp)
#   target_link_libraries(exp0_gddp_test
#     ${PROJECT_NAME}
#     GTest::gtest_main
#   )
#   target_compile_options(exp0_gddp_test PRIVATE ${OCS2_CXX_FLAGS})
#   gtest_discover_tests(exp0_gddp_test)
#
#   add_executable(exp1_gddp_test test/exp1_gddp_test.cpp)
#   target_link_libraries(exp1_gddp_test
#     ${PROJECT_NAME}
#     GTest::gtest_main
#   )
#   target_compile_options(exp1_gddp_test PRIVATE ${OCS2_CXX_FLAGS})
#   gtest_discover_tests(exp1_gddp_test)
#
#   add_executable(exp0_ocs2_test test/exp0_ocs2_test.cpp)
#   target_link_libraries(exp0_ocs2_test
#     ${PROJECT_NAME}
#     GTest::gtest_main
#   )
#   target_compile_options(exp0_ocs2_test PRIVATE ${OCS2_CXX_FLAGS})
#   gtest_discover_tests(exp0_ocs2_test)
#
#   add_executable(exp1_ocs2_test test/exp1_ocs2_test.cpp)
#   target_link_libraries(exp1_ocs2_test
#     ${PROJECT_NAME}
#     GTest::gtest_main
#   )
#   target_compile_options(exp1_ocs2_test PRIVATE ${OCS2_CXX_FLAGS})
#   gtest_discover_tests(exp1_ocs2_test)
# endif()
