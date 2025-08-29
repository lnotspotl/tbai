cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_g1_rl)

add_library(tbai_g1_rl
    src/G1RlController.cpp
    src/HistoryBuffer.cpp
)

include(FetchContent)

if(NOT UNIX OR APPLE)
    message(FATAL_ERROR "This project currently only supports Linux platforms")
endif()

set(ONNXRUNTIME_VERSION "1.22.0")
set(ONNXRUNTIME_URL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz")

# Download and extract ONNX Runtime
FetchContent_Declare(
    onnxruntime
    URL ${ONNXRUNTIME_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(onnxruntime)

# Get the extracted directory path
FetchContent_GetProperties(onnxruntime SOURCE_DIR ONNXRUNTIME_SOURCE_DIR)

# Set up include and library directories
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_SOURCE_DIR}/include")
set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_SOURCE_DIR}/lib")

# Create imported target for ONNX Runtime
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.${ONNXRUNTIME_VERSION}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
)

find_package(pinocchio REQUIRED)

target_include_directories(tbai_g1_rl
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_g1_rl
    PUBLIC
        tbai_core
        tbai_reference
        pinocchio::pinocchio
        onnxruntime
)

if(TBAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_g1_rl
    EXPORT tbai_g1_rl-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES
    "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.${ONNXRUNTIME_VERSION}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY "${ONNXRUNTIME_INCLUDE_DIR}/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

