cmake_minimum_required(VERSION 3.16)
project(ocs2_hpipm)

include(FetchContent)

# Load ocs2 compile flags
include(${CMAKE_CURRENT_SOURCE_DIR}/../../ocs2_core/cmake/ocs2_cxx_flags.cmake)

# =============================================================================
# HPIPM - High-Performance Interior Point Method
# =============================================================================
# HPIPM depends on BLASFEO, which is provided by ocs2_blasfeo
set(BLASFEO_PATH ${OCS2_BLASFEO_SOURCE_DIR} CACHE STRING "BLASFEO source path" FORCE)
set(BLASFEO_INCLUDE_DIR ${OCS2_BLASFEO_SOURCE_DIR}/include CACHE STRING "BLASFEO include path" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "HPIPM testing")

FetchContent_Declare(hpipm
  GIT_REPOSITORY https://github.com/giaf/hpipm
  GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf
)
FetchContent_MakeAvailable(hpipm)

# =============================================================================
# HPIPM Interface Library
# =============================================================================
add_library(${PROJECT_NAME}
  src/HpipmInterface.cpp
  src/HpipmInterfaceSettings.cpp
)
add_library(ocs2::hpipm ALIAS ${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${hpipm_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ocs2_core
    ocs2_oc
    hpipm
    ocs2_blasfeo
)

#############
## Install ##
#############

include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
  EXPORT ocs2Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
