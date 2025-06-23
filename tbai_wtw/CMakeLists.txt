cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_wtw)

add_library(tbai_wtw src/HistoryBuffer.cpp)
target_link_libraries(tbai_wtw PUBLIC tbai_core ${TORCH_LIBRARIES})
target_include_directories(tbai_wtw PUBLIC include)

if(TBAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()
