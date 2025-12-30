cmake_minimum_required(VERSION 3.16)
project(ocs2_thirdparty)

# CppAD - header-only library
add_library(cppad INTERFACE)
add_library(ocs2::cppad ALIAS cppad)

target_include_directories(cppad
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# CppADCodeGen - header-only library (depends on CppAD)
add_library(cppadcg INTERFACE)
add_library(ocs2::cppadcg ALIAS cppadcg)

target_include_directories(cppadcg
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cppadcg INTERFACE cppad)

# Installation
include(GNUInstallDirs)

install(TARGETS cppad cppadcg
  EXPORT ocs2ThirdpartyTargets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ocs2ThirdpartyTargets
  FILE ocs2ThirdpartyTargets.cmake
  NAMESPACE ocs2::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ocs2
)
