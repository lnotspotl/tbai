cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_wtw)

find_package(Torch REQUIRED)

add_library(tbai_wtw
    src/WtwController.cpp
    src/HistoryBuffer.cpp
)
target_link_libraries(tbai_wtw PUBLIC tbai_core tbai_reference tbai_torch)
target_include_directories(tbai_wtw PUBLIC include)

if(TBAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()
