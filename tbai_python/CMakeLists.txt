cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_python)

find_package(pybind11 CONFIG REQUIRED)
find_package(pinocchio CONFIG REQUIRED)

pybind11_add_module(tbai_python 
    src/bindings.cpp
)
target_link_libraries(tbai_python
    PUBLIC
        tbai_core
        tbai_bob
        tbai_reference
        tbai_static
        tbai_muse
        pinocchio::pinocchio
)
target_include_directories(tbai_python
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)