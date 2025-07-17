cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_deploy_go2)

add_library(tbai_deploy_go2
    src/Go2RobotInterface.cpp
    src/Go2JoystickInterface.cpp
)

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


add_executable(main src/main.cpp)
target_link_libraries(main
    PUBLIC
    unitree_sdk2
)

add_executable(main2 src/main2.cpp)
target_link_libraries(main2
    PUBLIC
        tbai_deploy_go2
)

add_executable(joystick_main src/joystick_main.cpp)
target_link_libraries(joystick_main
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
