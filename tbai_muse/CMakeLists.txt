cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_muse)

add_library(tbai_muse src/TbaiEstimator.cpp)
target_include_directories(tbai_muse PUBLIC include)
target_link_libraries(tbai_muse PUBLIC tbai_core)

add_executable(main src/main.cpp)
target_link_libraries(main PUBLIC tbai_muse)