cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_deploy_g1)

# Fetch ONNX Runtime automatically
include(FetchContent)

if(NOT UNIX OR APPLE)
    message(FATAL_ERROR "tbai_deploy_g1 currently only supports Linux platforms")
endif()

set(ONNXRUNTIME_VERSION "1.22.0")
set(ONNXRUNTIME_URL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz")

message(STATUS "Fetching ONNX Runtime v${ONNXRUNTIME_VERSION}...")

FetchContent_Declare(
    onnxruntime
    URL ${ONNXRUNTIME_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(onnxruntime)

FetchContent_GetProperties(onnxruntime SOURCE_DIR ONNXRUNTIME_SOURCE_DIR)

set(ONNXRUNTIME_INCLUDE_DIR "${onnxruntime_SOURCE_DIR}/include")
set(ONNXRUNTIME_LIB_DIR "${onnxruntime_SOURCE_DIR}/lib")

add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
)

message(STATUS "ONNX Runtime configured: ${onnxruntime_SOURCE_DIR}")

add_library(tbai_deploy_g1
    src/G1RobotInterface.cpp
    src/G1RLController.cpp
    src/G1MimicController.cpp
    src/MotionLoader.cpp
)
set_target_properties(tbai_deploy_g1 PROPERTIES EXPORT_NAME deploy_g1)

target_include_directories(tbai_deploy_g1
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${ONNXRUNTIME_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_deploy_g1
    PUBLIC
        tbai_core
        tbai_reference
        unitree_sdk2
        onnxruntime
)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_deploy_g1
    EXPORT tbai_deploy_g1-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_deploy_g1-targets
    NAMESPACE tbai::
    FILE tbai-deploy_g1-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)

# Install ONNX Runtime headers
install(DIRECTORY "${ONNXRUNTIME_INCLUDE_DIR}/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install ONNX Runtime library
install(FILES "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.${ONNXRUNTIME_VERSION}"
              "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1"
              "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
