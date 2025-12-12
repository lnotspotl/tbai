### Download unitree_sdk2
include(FetchContent)

## == Unitree Legged SDK2 ==
set(BUILD_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)
FetchContent_Declare(
    unitree_sdk2
    GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
    GIT_TAG 4f8aca7bbc269d275d45a7917d23b0386f74fb1c
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
)
FetchContent_MakeAvailable(unitree_sdk2)
message(STATUS "[TBAI] Unitree SDK2 source dir: ${unitree_sdk2_SOURCE_DIR}")

## == Install unitree_sdk2 ==
install(DIRECTORY ${unitree_sdk2_SOURCE_DIR}/lib/
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILES_MATCHING PATTERN "*.so*" PATTERN "*.a"
)

install(DIRECTORY ${unitree_sdk2_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${unitree_sdk2_SOURCE_DIR}/thirdparty/include/ddscxx/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

message(STATUS "[TBAI] Unitree SDK2 installation configured successfully")
