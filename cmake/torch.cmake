include(FetchContent)

option(TBAI_FORCE_TORCH_FETCH "Force fetching libtorch from PyTorch website" OFF)


if(TBAI_FORCE_TORCH_FETCH)
    message(STATUS "[TBAI] Force fetching libtorch from PyTorch website.")
endif()

find_package(Python3 COMPONENTS Interpreter)

if(NOT Python3_Interpreter_FOUND)
    message(FATAL_ERROR "[TBAI] Python3 interpreter not found. Please install Python3.")
endif()

execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pip show torch
    RESULT_VARIABLE PIP_SHOW_RESULT
    OUTPUT_QUIET
    ERROR_QUIET
)

if(PIP_SHOW_RESULT EQUAL 0 AND NOT TBAI_FORCE_TORCH_FETCH)
    message(STATUS "[TBAI] PyTorch found via pip, skipping fetch.")
    # Set Torch_DIR to the Python torch package location
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import torch; import os; print(os.path.dirname(torch.__file__))"
        OUTPUT_VARIABLE TORCH_PYTHON_PACKAGE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(Torch_DIR "${TORCH_PYTHON_PACKAGE_DIR}/share/cmake/Torch")
    list(APPEND CMAKE_PREFIX_PATH "${Torch_DIR}")
    set(TORCH_SOURCE_DIR "${TORCH_PYTHON_PACKAGE_DIR}")
else()
    # Download and configure libtorch CPU
    set(PYTORCH_VERSION "2.7.0")
    set(PYTORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip")
    message(STATUS "[TBAI] PyTorch not found via pip, fetching libtorch version ${PYTORCH_VERSION} from ${PYTORCH_URL}.")

    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        message(FATAL_ERROR "Fetching libtorch not supported on aarch64 architecture. Please install PyTorch via pip or build libtorch from source.")
    endif()


    FetchContent_Declare(
        libtorch
        URL ${PYTORCH_URL}
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

    FetchContent_MakeAvailable(libtorch)

    # Add libtorch to path
    set(TORCH_SOURCE_DIR "${libtorch_SOURCE_DIR}")
    list(APPEND CMAKE_PREFIX_PATH ${libtorch_SOURCE_DIR})
endif()

find_package(Torch REQUIRED)

macro(_tbai_print_var label var)
    if(DEFINED ${var} AND NOT "${${var}}" STREQUAL "")
        message(STATUS "[TBAI] ${label}: ${${var}}")
    else()
        message(STATUS "[TBAI] ${label}: <not set>")
    endif()
endmacro()

_tbai_print_var("Torch found" Torch_DIR)
_tbai_print_var("Torch libraries" TORCH_LIBRARIES)
_tbai_print_var("Torch include dir" TORCH_INCLUDE_DIRS)
_tbai_print_var("Torch library dir" TORCH_LIBRARY_DIRS)
_tbai_print_var("Torch source dir" TORCH_SOURCE_DIR)
_tbai_print_var("Torch libraries" Torch_LIBRARIES)
_tbai_print_var("Torch version" Torch_VERSION)
_tbai_print_var("Torch config" Torch_CONFIG)
_tbai_print_var("Torch module" Torch_MODULE_PATH)
_tbai_print_var("Torch package" Torch_PACKAGE_DIR)
_tbai_print_var("Torch package version" Torch_PACKAGE_VERSION)
_tbai_print_var("Torch package include dir" Torch_PACKAGE_INCLUDE_DIRS)

if(NOT DEFINED TORCH_SOURCE_DIR OR "${TORCH_SOURCE_DIR}" STREQUAL "")
    message(FATAL_ERROR "TORCH_SOURCE_DIR is not set or is empty. PyTorch/LibTorch was not found or configured properly.")
endif()


install(DIRECTORY ${TORCH_SOURCE_DIR}/lib/
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        FILES_MATCHING PATTERN "*.so*"
    )

install(DIRECTORY ${TORCH_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)