### Download unitree_sdk2
include(FetchContent)

## == Unitree Legged SDK2 ==
set(BUILD_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)
FetchContent_Declare(
    unitree_sdk2
    GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
    GIT_TAG eed0898b8d63d83406f7f460a827fa378dd3e631
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
)
FetchContent_MakeAvailable(unitree_sdk2)
message(STATUS "[tbai_bob] Unitree SDK2 built: ${unitree_sdk2_SOURCE_DIR}")

## == Unitree Legged SDK2 ==
message(STATUS "[tbai_bob] Unitree SDK2 found: ${unitree_sdk2_DIR}")
    
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


