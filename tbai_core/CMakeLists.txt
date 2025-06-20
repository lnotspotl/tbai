cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_core)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED) 
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(pinocchio REQUIRED)

add_library(tbai_core
    src/Files.cpp
    src/Utils.cpp
    src/Env.cpp
    src/Logging.cpp
)

target_include_directories(tbai_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_core 
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    fmt::fmt
    spdlog::spdlog
    pinocchio::pinocchio
)
target_include_directories(tbai_core INTERFACE
    ${fmt_INCLUDE_DIRS}
)

## Tests

if(TBAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()


## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_core
    EXPORT tbai_core-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_core-targets
    NAMESPACE tbai::
    FILE tbai-core-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)