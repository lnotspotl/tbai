cmake_minimum_required(VERSION 3.16)
project(ocs2_qpoases)

include(ExternalProject)

# =============================================================================
# qpOASES - Quadratic Programming solver
# =============================================================================
set(QPOASES_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/qpoases_install)

ExternalProject_Add(
  qpOASES_external
  GIT_REPOSITORY https://github.com/humanoid-path-planner/qpoases.git
  GIT_TAG c7e5ff50a770cf3a2b1b56eb375941be6088b26b
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${QPOASES_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DBUILD_SHARED_LIBS=ON
    -DCMAKE_CXX_FLAGS="-w"
  UPDATE_COMMAND ""
  BUILD_COMMAND ${CMAKE_COMMAND} --build . --target install -- -j1
)

message(STATUS "qpOASES_external installed in: ${QPOASES_INSTALL_PREFIX}")

# Create an interface library that exports qpOASES
add_library(${PROJECT_NAME} INTERFACE)
add_library(ocs2::qpoases ALIAS ${PROJECT_NAME})

# Make sure qpOASES is built before this target
add_dependencies(${PROJECT_NAME} qpOASES_external)

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${QPOASES_INSTALL_PREFIX}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    ${QPOASES_INSTALL_PREFIX}/lib/libqpOASES.so
)

#############
## Install ##
#############

include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
  EXPORT ocs2Targets
)

install(DIRECTORY ${QPOASES_INSTALL_PREFIX}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${QPOASES_INSTALL_PREFIX}/lib/libqpOASES.so
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
