# Find F2C
# 
# This sets the following variables:
# F2C_FOUND - system has libf2c
# F2C_INCLUDE_DIR - the libf2c include directories
# F2C_LIBRARY - link these to use libf2c

# Include dir
find_path(F2C_INCLUDE_DIR NAMES f2c.h)
set(F2C_INCLUDE_DIRS ${F2C_INCLUDE_DIR})

# Finally the library itself
find_library(F2C_LIBRARY NAMES f2c)
set(F2C_INCLUDE_DIRS ${F2C_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(F2C DEFAULT_MSG F2C_LIBRARY F2C_INCLUDE_DIR)
