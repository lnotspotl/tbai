cmake_minimum_required(VERSION 3.0.2)
project(tbai_core)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  tf
  tbai_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS 
  DEPENDS Eigen
)

## Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/CentralController.cpp
  src/StateSubscriber.cpp
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)