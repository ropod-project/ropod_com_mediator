cmake_minimum_required(VERSION 2.8.6)
project (ropod-communication-mediator)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(INSTALL_DIR /usr/lib CACHE PATH "Installation directory for mediator")


########################################################################
# LIBZMQ dependency
########################################################################
find_package(libzmq REQUIRED)
IF (LIBZMQ_FOUND)
    include_directories(${LIBZMQ_INCLUDE_DIRS})
    list(APPEND LIBS ${LIBZMQ_LIBRARIES})
ELSE (LIBZMQ_FOUND)
    message( FATAL_ERROR "libzmq not found." )
ENDIF (LIBZMQ_FOUND)

########################################################################
# CZMQ dependency
########################################################################
find_package(czmq REQUIRED)
IF (CZMQ_FOUND)
    include_directories(${CZMQ_INCLUDE_DIRS})
    list(APPEND LIBS ${CZMQ_LIBRARIES})
ELSE (CZMQ_FOUND)
    message( FATAL_ERROR "czmq not found." )
ENDIF (CZMQ_FOUND)

########################################################################
# Zyre dependency
########################################################################
find_package(zyre REQUIRED)
IF (ZYRE_FOUND)
    include_directories(${ZYRE_INCLUDE_DIRS})
    list(APPEND LIBS ${ZYRE_LIBRARIES})
ELSE (ZYRE_FOUND)
    message( FATAL_ERROR "zyre not found." )
ENDIF (ZYRE_FOUND)

########################################################################
# JSON dependency
########################################################################

#pkg_check_modules(JSONCPP jsoncpp)
#target_link_libraries(${JSONCPP_LIBRARIES})
#include_directories(${JSONCPP_INCLUDE_DIRS})

find_package(jsoncpp REQUIRED)
IF (JSONCPP_FOUND)
    include_directories(${JSONCPP_INCLUDE_DIRS})
    list(APPEND LIBS ${JSONCPP_LIBRARIES})
ELSE (JSONCPP_FOUND)
    message( FATAL_ERROR "JSONCPP not found." )
ENDIF (JSONCPP_FOUND)


########################################################################
# Mediator
########################################################################
#include_directories(${PROJECT_SOURCE_DIR}/include)
#set(HEADER_FILES ${PROJECT_SOURCE_DIR}/include/mediator.h)


add_executable(ropod_comm_mediator_test ${PROJECT_SOURCE_DIR}/src/test.cpp ${HEADER_FILES})
target_link_libraries(ropod_comm_mediator_test ${LIBS})

#add_executable(ropod_comm_mediator ${PROJECT_SOURCE_DIR}/src/ropod_comm_mediator.c ${HEADER_FILES})
#target_link_libraries(ropod_comm_mediator ${LIBS} jansson)

