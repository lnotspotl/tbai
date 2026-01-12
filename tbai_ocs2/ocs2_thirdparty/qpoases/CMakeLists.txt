cmake_minimum_required(VERSION 3.16)
project(ocs2_qpoases)

include(ExternalProject)

# =============================================================================
# qpOASES - Quadratic Programming solver
# =============================================================================
ExternalProject_Add(
  qpOASES_external
  GIT_REPOSITORY https://github.com/humanoid-path-planner/qpoases.git
  GIT_TAG c7e5ff50a770cf3a2b1b56eb375941be6088b26b
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DBUILD_SHARED_LIBS=ON
    -DCMAKE_CXX_FLAGS="-w"
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
)

ExternalProject_Get_Property(qpOASES_external SOURCE_DIR BINARY_DIR)

# Create an interface library that exports qpOASES
add_library(${PROJECT_NAME} INTERFACE)
add_library(ocs2::qpoases ALIAS ${PROJECT_NAME})

# Make sure qpOASES is built before this target
add_dependencies(${PROJECT_NAME} qpOASES_external)

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${BINARY_DIR}/libs/libqpOASES.so>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_LIBDIR}/libqpOASES.so>
)

#############
## Install ##
#############

include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
  EXPORT ocs2qpoasesTargets
)

install(DIRECTORY ${SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${BINARY_DIR}/libs/libqpOASES.so
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT ocs2qpoasesTargets
  FILE tbai-ocs2-qpoases-targets.cmake
  NAMESPACE ocs2::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)