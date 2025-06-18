cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_bob)

### Download torch
include(FetchContent)

# Download and configure libtorch CPU
set(PYTORCH_VERSION "2.1.0")
set(PYTORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip")

FetchContent_Declare(
    libtorch
    URL ${PYTORCH_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(libtorch)

# Add libtorch to path
list(APPEND CMAKE_PREFIX_PATH ${libtorch_SOURCE_DIR})
find_package(Torch REQUIRED)
message(STATUS "[tbai_bob] Torch found: ${Torch_DIR}")
message(STATUS "[tbai_bob] Torch include dir: ${TORCH_INCLUDE_DIRS}")
message(STATUS "[tbai_bob] Torch library dir: ${TORCH_LIBRARY_DIRS}")
message(STATUS "[tbai_bob] Torch libraries: ${Torch_LIBRARIES}")
message(STATUS "[tbai_bob] Torch version: ${Torch_VERSION}")
message(STATUS "[tbai_bob] Torch config: ${Torch_CONFIG}")
message(STATUS "[tbai_bob] Torch module: ${Torch_MODULE_PATH}")
message(STATUS "[tbai_bob] Torch package: ${Torch_PACKAGE_DIR}")
message(STATUS "[tbai_bob] Torch package version: ${Torch_PACKAGE_VERSION}")
message(STATUS "[tbai_bob] Torch package include dir: ${Torch_PACKAGE_INCLUDE_DIRS}")

add_library(tbai_bob
    src/BobController.cpp
    src/CentralPatternGenerator.cpp
    src/InverseKinematics.cpp
)

target_include_directories(tbai_bob
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_bob
    PUBLIC
        tbai_core
        tbai_reference
        ${TORCH_LIBRARIES}
)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_bob
    EXPORT tbai_bob-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

## Install torch
install(DIRECTORY ${libtorch_SOURCE_DIR}/lib/
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILES_MATCHING PATTERN "*.so*"
)

install(DIRECTORY ${libtorch_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_bob-targets
    NAMESPACE tbai::
    FILE tbai-bob-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)