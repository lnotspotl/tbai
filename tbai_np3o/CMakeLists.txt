cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_np3o)

add_library(tbai_np3o
    src/Np3oController.cpp
    src/HistoryBuffer.cpp
)

find_package(pinocchio REQUIRED)

if(TBAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()

target_include_directories(tbai_np3o
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_np3o
    PUBLIC
        tbai_core
        tbai_reference
        ${TORCH_LIBRARIES}
        pinocchio::pinocchio
)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_np3o
    EXPORT tbai_np3o-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
