cmake_minimum_required(VERSION 3.16)
project(ocs2_blasfeo)

include(FetchContent)

# =============================================================================
# BLASFEO - Basic Linear Algebra Subroutines for Embedded Optimization
# =============================================================================
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "BLASFEO examples")

FetchContent_Declare(blasfeo
  GIT_REPOSITORY https://github.com/giaf/blasfeo
  GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
)
FetchContent_MakeAvailable(blasfeo)

# Create an interface library that exports blasfeo
add_library(${PROJECT_NAME} INTERFACE)
add_library(ocs2::blasfeo ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${blasfeo_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    blasfeo
)

# Export blasfeo source dir for hpipm to use
set(OCS2_BLASFEO_SOURCE_DIR ${blasfeo_SOURCE_DIR} CACHE INTERNAL "BLASFEO source directory")

#############
## Install ##
#############

include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
  EXPORT ocs2Targets
)
