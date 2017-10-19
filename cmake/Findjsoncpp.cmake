# - Try to find JSONCPP
# Once done this will define
#  JSONCPP_FOUND - System has JSONCPP
#  JSONCPP_INCLUDE_DIRS - The JSONCPP include directories
#  JSONCPP_LIBRARIES - The libraries needed to use JSONCPP

find_path(JSONCPP_INCLUDE_DIR json/json.h
          /usr/include
          /usr/include/jsoncpp
          /usr/local/include
          /usr/local/include/jsoncpp )

find_library(JSONCPP_LIBRARY NAMES libjsoncpp.so
             PATHS /usr/lib /usr/local/lib )

set(JSONCPP_LIBRARIES ${JSONCPP_LIBRARY} )
set(JSONCPP_INCLUDE_DIRS ${JSONCPP_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set JSONCPP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(JSONCPP  DEFAULT_MSG
                                  JSONCPP_LIBRARY JSONCPP_INCLUDE_DIR)

mark_as_advanced(JSONCPP_INCLUDE_DIR JSONCPP_LIBRARY )
