### Install vcpkg libraries
# This file installs all vcpkg libraries to the system installation directory

# Get vcpkg installed directory
if(DEFINED VCPKG_INSTALLED_DIR)
    set(VCPKG_INSTALLED_DIRECTORY "${VCPKG_INSTALLED_DIR}")
elseif(DEFINED CMAKE_TOOLCHAIN_FILE)
    # Derive from toolchain file path
    get_filename_component(VCPKG_TOOLCHAIN_DIR "${CMAKE_TOOLCHAIN_FILE}" DIRECTORY)
    get_filename_component(VCPKG_SCRIPTS_DIR "${VCPKG_TOOLCHAIN_DIR}" DIRECTORY)
    get_filename_component(VCPKG_ROOT_DIR "${VCPKG_SCRIPTS_DIR}" DIRECTORY)
    set(VCPKG_INSTALLED_DIRECTORY "${VCPKG_ROOT_DIR}/installed")
elseif(DEFINED VCPKG_DIRECTORY)
    set(VCPKG_INSTALLED_DIRECTORY "${VCPKG_DIRECTORY}/installed")
else()
    # Fallback to common vcpkg installation paths
    if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/vcpkg/installed")
        set(VCPKG_INSTALLED_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/vcpkg/installed")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/installed")
        set(VCPKG_INSTALLED_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/installed")
    else()
        message(WARNING "Could not find vcpkg installed directory. Skipping vcpkg library installation.")
        return()
    endif()
endif()

# Get the target triplet
if(DEFINED VCPKG_TARGET_TRIPLET)
    set(VCPKG_TRIPLET "${VCPKG_TARGET_TRIPLET}")
elseif(DEFINED ENV{VCPKG_DEFAULT_TRIPLET})
    set(VCPKG_TRIPLET "$ENV{VCPKG_DEFAULT_TRIPLET}")
elseif(DEFINED ENV{VCPKG_DEFAULT_HOST_TRIPLET})
    set(VCPKG_TRIPLET "$ENV{VCPKG_DEFAULT_HOST_TRIPLET}")
else()
    # Try to detect triplet
    if(WIN32)
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(VCPKG_TRIPLET "x64-windows")
        else()
            set(VCPKG_TRIPLET "x86-windows")
        endif()
    elseif(APPLE)
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(VCPKG_TRIPLET "x64-osx")
        else()
            set(VCPKG_TRIPLET "x86-osx")
        endif()
    else()
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(VCPKG_TRIPLET "x64-linux")
        else()
            set(VCPKG_TRIPLET "x86-linux")
        endif()
    endif()
endif()

message(STATUS "[TBAI] Installing vcpkg libraries from: ${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}")

# Check if the installed directory exists
if(NOT EXISTS "${VCPKG_INSTALLED_DIRECTORY}")
    message(WARNING "[TBAI] Vcpkg installed directory does not exist yet: ${VCPKG_INSTALLED_DIRECTORY}")
    message(WARNING "[TBAI] This is normal during first build. Libraries will be installed after vcpkg packages are built.")
    return()
endif()

if(NOT EXISTS "${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}")
    message(WARNING "[TBAI] Vcpkg triplet directory does not exist yet: ${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}")
    message(WARNING "[TBAI] This is normal during first build. Libraries will be installed after vcpkg packages are built.")
    return()
endif()

# Install libraries
if(EXISTS "${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/lib")
    install(DIRECTORY ${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/lib/
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        FILES_MATCHING PATTERN "*.so*" PATTERN "*.dll" PATTERN "*.dylib" PATTERN "*.lib"
    )
endif()

# Install debug libraries if they exist
if(EXISTS "${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/debug/lib" AND NOT EXISTS "${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/lib/")
    install(DIRECTORY ${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/debug/lib/
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        FILES_MATCHING PATTERN "*.so*" PATTERN "*.dll" PATTERN "*.dylib"  PATTERN "*.lib"
    )
endif()

# Install include files
if(EXISTS "${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/include")
    install(DIRECTORY ${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif()

# Install debug include files if they exist
if(EXISTS "${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/debug/include" AND NOT EXISTS "${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/include/")
    install(DIRECTORY ${VCPKG_INSTALLED_DIRECTORY}/${VCPKG_TRIPLET}/debug/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif()


message(STATUS "[TBAI] Vcpkg libraries installation configured successfully")
message(STATUS "[TBAI] Libraries will be installed to: ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "[TBAI] Headers will be installed to: ${CMAKE_INSTALL_INCLUDEDIR}")