cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_torch)

find_package(Torch REQUIRED)

add_library(tbai_torch INTERFACE)

target_include_directories(tbai_torch
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_torch
    INTERFACE
        tbai_core
        ${TORCH_LIBRARIES}
)

if(TBAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_torch
    EXPORT tbai_torch-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_torch-targets
    NAMESPACE tbai::
    FILE tbai-torch-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)
