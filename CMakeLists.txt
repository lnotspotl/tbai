cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(TBAI_DEV "Build in development mode" OFF)
option(TBAI_BUILD_TESTS "Build the tests" OFF)
option(TBAI_BUILD_BOB "Build the bob" ON)
option(TBAI_BUILD_WTW "Build the wtw" ON)
option(TBAI_BUILD_NP3O "Build the np3o" ON)
option(TBAI_BUILD_MUSE "Build the muse" ON)
option(TBAI_BUILD_PYTHON "Build the Python bindings" OFF)
option(TBAI_BUILD_DEPLOY_GO2 "Build the deploy go2" ON)
option(TBAI_BUILD_DEPLOY_GO2W "Build the deploy go2w (wheeled)" ON)
option(TBAI_BUILD_DEPLOY_G1 "Build the deploy g1" ON)
option(TBAI_BUILD_OCS2 "Build the ocs2" ON)
option(TBAI_BUILD_MPC "Build the mpc" ON)
option(TBAI_BUILD_DTC "Build the dtc" ON)
option(TBAI_BUILD_JOE "Build the joe" ON)

project(tbai VERSION 2.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(TBAI_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(tbai_core)
add_subdirectory(tbai_reference)
add_subdirectory(tbai_static)

if(TBAI_BUILD_MUSE)
  add_subdirectory(tbai_estim)
endif()

if(TBAI_BUILD_BOB)
  add_subdirectory(tbai_bob)
endif()

if(TBAI_BUILD_WTW)
  add_subdirectory(tbai_wtw)
endif()

if(TBAI_BUILD_NP3O)
  add_subdirectory(tbai_np3o)
endif()

if(TBAI_BUILD_PYTHON)
  add_subdirectory(tbai_python)
endif()

set(GET_USE_UNITREE_SDK2 
    ${TBAI_BUILD_DEPLOY_GO2} 
    OR ${TBAI_BUILD_DEPLOY_GO2W} 
    OR ${TBAI_BUILD_DEPLOY_G1}
)

if(GET_USE_UNITREE_SDK2)
  include(cmake/unitree_sdk2.cmake)
endif()

if(TBAI_BUILD_DEPLOY_GO2)
  add_subdirectory(tbai_deploy_go2)
endif()

if(TBAI_BUILD_DEPLOY_GO2W)
  add_subdirectory(tbai_deploy_go2w)
endif()

if(TBAI_BUILD_DEPLOY_G1)
  add_subdirectory(tbai_deploy_g1)
endif()

if(TBAI_BUILD_OCS2)
  add_subdirectory(tbai_ocs2)
endif()

if(TBAI_BUILD_MPC)
  add_subdirectory(tbai_mpc)
endif()

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/tbai-config-version.cmake"
    VERSION ${tbai_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/tbai-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/tbai-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/tbai-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/tbai-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)
