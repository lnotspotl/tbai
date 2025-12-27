cmake_minimum_required(VERSION 3.16)
project(ocs2_slp)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../ocs2_core/cmake/ocs2_cxx_flags.cmake)

# Collect all source files
set(OCS2_SLP_SOURCES
  src/pipg/PipgSettings.cpp
  src/pipg/PipgSolver.cpp
  src/pipg/SingleThreadPipg.cpp
  src/Helpers.cpp
  src/SlpSettings.cpp
  src/SlpSolver.cpp
)

# Create the main library
add_library(${PROJECT_NAME} ${OCS2_SLP_SOURCES})
add_library(ocs2::slp ALIAS ${PROJECT_NAME})

# Set target properties
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ocs2_core
    ocs2_oc
    ocs2_mpc
    ocs2_qp_solver
)

# Lint target for clang tooling
add_executable(${PROJECT_NAME}_lintTarget src/lintTarget.cpp)
target_link_libraries(${PROJECT_NAME}_lintTarget
  PRIVATE
    ${PROJECT_NAME}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ${PROJECT_NAME}_lintTarget")
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
      ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif()

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
  add_executable(test_${PROJECT_NAME}
    test/testHelpers.cpp
    test/testPipgSolver.cpp
    test/testSlpSolver.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    PRIVATE
      ${PROJECT_NAME}
      GTest::gtest_main
  )
  gtest_discover_tests(test_${PROJECT_NAME})
endif()
