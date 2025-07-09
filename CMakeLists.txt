cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(TBAI_BUILD_TESTS "Build the tests" OFF)
option(TBAI_BUILD_BOB "Build the bob" ON)
option(TBAI_BUILD_WTW "Build the wtw" ON)
option(TBAI_BUILD_NP3O "Build the np3o" ON)
option(TBAI_BUILD_MUSE "Build the muse" ON)
option(TBAI_BUILD_PYTHON "Build the Python bindings" OFF)
option(TBAI_BUILD_DEPLOY_GO2 "Build the deploy go2" ON)

if(TBAI_BUILD_TESTS)
  enable_testing()
  list(APPEND VCPKG_MANIFEST_FEATURES "tests")
endif()

if(TBAI_BUILD_PYTHON)
  list(APPEND VCPKG_MANIFEST_FEATURES "python")
endif()

if(TBAI_BUILD_DEPLOY_GO2)
  list(APPEND VCPKG_MANIFEST_FEATURES "deploy-go2")
endif()

set(VCPKG_VERSION)
include(cmake/vcpkg.cmake)

project(tbai VERSION 2.0.0)

add_subdirectory(tbai_core)
add_subdirectory(tbai_reference)
add_subdirectory(tbai_static)

if(TBAI_BUILD_BOB OR TBAI_BUILD_WTW)
  include(cmake/torch.cmake)
endif()

# # Install vcpkg libraries
# include(cmake/vcpkg-install.cmake)

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

if(TBAI_BUILD_DEPLOY_GO2)
  include(cmake/unitree_sdk2.cmake)
  add_subdirectory(tbai_deploy_go2)
endif()

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/tbai-config-version.cmake"
    VERSION ${tbai_VERSION}
    COMPATIBILITY SameMajorVersion
)

set(tbai_VCPKG_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_FILE}")
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/tbai-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/tbai-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
    PATH_VARS tbai_VCPKG_TOOLCHAIN_FILE
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/tbai-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/tbai-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)