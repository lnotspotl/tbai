vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO unitreerobotics/unitree_sdk2
    REF 794e0c2116be7a5a7e71af84660b6858c0c40cbe
    SHA512 efd7f177aa2e0284cab3558a8bb75eb45cfd0532ad05fde47376a9f001836581590e5ec453d00cd424dfbc07b84ab9925b3846940f0365d4a89f8deb9e2a9957
    HEAD_REF main
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DBUILD_EXAMPLES=OFF
)

vcpkg_cmake_install()