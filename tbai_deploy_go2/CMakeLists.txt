cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_deploy_go2)

add_library(tbai_deploy_go2
    src/Go2RobotInterface.cpp
    src/Go2JoystickInterface.cpp
)
set_target_properties(tbai_deploy_go2 PROPERTIES EXPORT_NAME deploy_go2)

target_include_directories(tbai_deploy_go2
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_deploy_go2
    PUBLIC
        tbai_core
        tbai_estim
        unitree_sdk2
)


add_executable(test_stand_sit_tbai src/test_stand_sit_tbai.cpp)
target_link_libraries(test_stand_sit_tbai
    PUBLIC
    unitree_sdk2
    tbai_deploy_go2
)

add_executable(test_stand_sit_unitree src/test_stand_sit_unitree.cpp)
target_link_libraries(test_stand_sit_unitree
    PUBLIC
    unitree_sdk2
)

add_executable(test_lidar src/test_lidar.cpp)
target_link_libraries(test_lidar
    PUBLIC
        tbai_deploy_go2
)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_deploy_go2
    EXPORT tbai_deploy_go2-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_deploy_go2-targets
    NAMESPACE tbai::
    FILE tbai-deploy_go2-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)
