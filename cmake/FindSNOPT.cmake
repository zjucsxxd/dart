# Find SNOPT
#
# This sets the following variables:
# SNOPT_FOUND
# SNOPT_LIBRARIES
# SNOPT_DEFINITIONS

find_package(PkgConfig QUIET)
pkg_check_modules(PC_SNOPT snopt)
set(SNOPT_DEFINITIONS ${PC_SNOPT_CFLAGS_OTHER})

set(SNOPT_LIBRARIES snopt_c snprint_c blas_c f2c)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SNOPT DEFAULT_MSG
                                  SNOPT_LIBRARY)

mark_as_advanced(SNOPT_LIBRARY)
