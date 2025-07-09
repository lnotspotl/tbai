cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_np3o)

add_library(tbai_np3o
    src/Np3oController.cpp
    src/HistoryBuffer.cpp
)
target_link_libraries(tbai_np3o PUBLIC tbai_core tbai_reference ${TORCH_LIBRARIES})
target_include_directories(tbai_np3o PUBLIC include)

if(TBAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()
