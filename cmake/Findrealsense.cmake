# Try to find Part4 library
# Once done this will define
#  REALSENSE_FOUND - if system found Part4 library
#  REALSENSE_INCLUDE_DIRS - The Part4 include directories
#  REALSENSE_LIBRARIES - The libraries needed to use Part4
#  REALSENSE_DEFINITIONS - Compiler switches required for using Part4

find_path(REALSENSE_INCLUDE_DIR "librealsense/rs.h")
find_library(REALSENSE_LIBRARY "librealsense.so")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(REALSENSE DEFAULT_MSG REALSENSE_INCLUDE_DIR REALSENSE_LIBRARY)

if(REALSENSE_FOUND)
    set(REALSENSE_INCLUDE_DIRS "${REALSENSE_INCLUDE_DIR}")
    message(STATUS "REALSENSE_INCLUDE_DIR = ${REALSENSE_INCLUDE_DIR}")
    set(REALSENSE_LIBRARIES "${REALSENSE_LIBRARY}")
    message(STATUS "REALSENSE_LIBRARY = ${REALSENSE_LIBRARY}")
endif(REALSENSE_FOUND)

