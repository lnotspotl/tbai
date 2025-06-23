### Download torch
include(FetchContent)

# Download and configure libtorch CPU
set(PYTORCH_VERSION "2.7.0")
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