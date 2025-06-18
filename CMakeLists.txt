cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(TBAI_BUILD_TESTS "Build the tests" OFF)

if(TBAI_BUILD_TESTS)
  enable_testing()
  list(APPEND VCPKG_MANIFEST_FEATURES "tests")
endif()

set(VCPKG_VERSION)
include(cmake/vcpkg.cmake)

project(tbai VERSION 2.0.0)

add_subdirectory(tbai_core)
