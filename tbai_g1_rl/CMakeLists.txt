cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_g1_rl)

find_package(Torch REQUIRED)

add_library(tbai_g1_rl
    src/G1RlController.cpp
)

target_include_directories(tbai_g1_rl
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_g1_rl
    PUBLIC
        tbai_core
        tbai_reference
        ${TORCH_LIBRARIES}
)

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
