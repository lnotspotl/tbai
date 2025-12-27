# OCS2 C++ Compile Flags
#
# Additional flags can be passed via OCS2_CXX_FLAGS cache variable:
#   cmake -DOCS2_CXX_FLAGS="-march=native;-mtune=native" ..

# Initialize the flags list
set(OCS2_CXX_FLAGS "" CACHE STRING "Additional OCS2 compile flags")

# Core flags
list(APPEND OCS2_CXX_FLAGS
  "-pthread"
  "-Wfatal-errors"
)

# Linker flags (only for shared libraries on Linux)
if(UNIX AND NOT APPLE)
  list(APPEND OCS2_CXX_FLAGS
    "-Wl,--no-as-needed"
  )
endif()

# Force Boost dynamic linking
list(APPEND OCS2_CXX_FLAGS
  "-DBOOST_ALL_DYN_LINK"
)

# Add OpenMP flags if not already found
if(NOT DEFINED OpenMP_CXX_FOUND)
  find_package(OpenMP REQUIRED)
endif()
if(OpenMP_CXX_FLAGS)
  list(APPEND OCS2_CXX_FLAGS
    ${OpenMP_CXX_FLAGS}
  )
endif()
