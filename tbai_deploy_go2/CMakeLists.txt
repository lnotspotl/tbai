cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_deploy_go2)

add_library(tbai_deploy_go2
    src/Go2RobotInterface.cpp
)

target_include_directories(tbai_deploy_go2
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_deploy_go2
    PUBLIC
        tbai_core
)




