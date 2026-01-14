cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_estim)

add_library(tbai_estim
    # Muse estimator
    src/muse/MuseEstimator.cpp

    # InEKF
    src/inekf/InEKF.cpp
    src/inekf/LieGroup.cpp
    src/inekf/NoiseParams.cpp
    src/inekf/RobotState.cpp
    src/inekf/InEKFEstimator.cpp

)
set_target_properties(tbai_estim PROPERTIES EXPORT_NAME estim)
target_link_libraries(tbai_estim PUBLIC tbai_core)
target_include_directories(tbai_estim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_estim
    EXPORT tbai_estim-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_estim-targets
    NAMESPACE tbai::
    FILE tbai-estimator-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)