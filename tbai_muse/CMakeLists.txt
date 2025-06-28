cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_muse)

add_library(tbai_muse src/TbaiEstimator.cpp)
target_link_libraries(tbai_muse PUBLIC tbai_core)
target_include_directories(tbai_muse
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_executable(main src/main.cpp)
target_link_libraries(main PUBLIC tbai_muse)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_muse
    EXPORT tbai_muse-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_muse-targets
    NAMESPACE tbai::
    FILE tbai-muse-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)