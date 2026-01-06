cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_joe)

find_package(Torch REQUIRED)

add_library(tbai_joe
    src/JoeController.cpp
)

target_include_directories(tbai_joe
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_joe
    PUBLIC
        tbai_core
        tbai_torch
        tbai_quadruped_mpc
        tbai_reference
        ocs2_core
        ocs2_mpc
        ocs2_sqp
	ocs2_centroidal_model
        ocs2_pinocchio_interface
        ${TORCH_LIBRARIES}
)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_joe
    EXPORT tbai_joe-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_joe-targets
    NAMESPACE tbai::
    FILE tbai-joe-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)
