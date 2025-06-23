cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_bob)

find_package(pinocchio REQUIRED)

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
        pinocchio::pinocchio
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
