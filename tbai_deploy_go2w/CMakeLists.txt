cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

project(tbai_deploy_go2w)

find_package(Torch REQUIRED)

add_library(tbai_deploy_go2w
    src/Go2WRobotInterface.cpp
    src/Go2WDreamWaQController.cpp
)
set_target_properties(tbai_deploy_go2w PROPERTIES EXPORT_NAME deploy_go2w)

target_include_directories(tbai_deploy_go2w
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_deploy_go2w
    PUBLIC
        tbai_core
        tbai_reference
        unitree_sdk2
        ${TORCH_LIBRARIES}
)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_deploy_go2w
    EXPORT tbai_deploy_go2w-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_deploy_go2w-targets
    NAMESPACE tbai::
    FILE tbai-deploy_go2w-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)
