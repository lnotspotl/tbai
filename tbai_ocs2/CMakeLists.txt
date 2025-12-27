cmake_minimum_required(VERSION 3.16)
project(tbai_ocs2
  VERSION 1.0.0
  LANGUAGES CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable Position Independent Code for all targets (needed for linking static libs into shared libs)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build options
option(OCS2_BUILD_TESTS "Build unit tests" OFF)

# Set default build type if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

# Module path for custom cmake files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find required dependencies
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Threads
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

# OpenMP
find_package(OpenMP REQUIRED)

# GTest for testing
if(OCS2_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(GoogleTest)
endif()

# Add subdirectories
add_subdirectory(ocs2_thirdparty)
add_subdirectory(ocs2_core)
add_subdirectory(ocs2_frank_wolfe)
add_subdirectory(ocs2_oc)
add_subdirectory(ocs2_robotic_tools)
add_subdirectory(ocs2_perceptive)
add_subdirectory(ocs2_qp_solver)
add_subdirectory(ocs2_mpc)
add_subdirectory(ocs2_thirdparty/blasfeo)
add_subdirectory(ocs2_thirdparty/hpipm)
add_subdirectory(ocs2_thirdparty/qpoases)
add_subdirectory(ocs2_sqp)
add_subdirectory(ocs2_ddp)
add_subdirectory(ocs2_slp)
add_subdirectory(ocs2_ipm)
add_subdirectory(ocs2_ocs2)
add_subdirectory(ocs2_pinocchio)

# Install cmake config files
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/ocs2ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ocs2Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/ocs2Config.cmake"
  INSTALL_DESTINATION lib/cmake/ocs2
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/ocs2Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/ocs2ConfigVersion.cmake"
  DESTINATION lib/cmake/ocs2
)
