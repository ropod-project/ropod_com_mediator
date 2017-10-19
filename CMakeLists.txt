cmake_minimum_required(VERSION 2.8.6)
project(ropod_wm_mediator)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ropod_wm_mediator/cmake")

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(INSTALL_DIR /usr/lib CACHE PATH "Installation directory for mediator")


########################################################################
# ROS dependency
########################################################################
find_package(catkin REQUIRED COMPONENTS roscpp message_generation ropod_ros_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

#catkin_package(
##   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp ropod_ros_msgs
#   DEPENDS message_runtime
#)

catkin_package(
  CATKIN_DEPENDS roscpp ropod_ros_msgs
)

########################################################################
# JSON dependency
########################################################################

#pkg_check_modules(JSONCPP jsoncpp)
#target_link_libraries(${JSONCPP_LIBRARIES})
#include_directories(${JSONCPP_INCLUDE_DIRS})

#find_package(jsoncpp REQUIRED)
#IF (JSONCPP_FOUND)
#    include_directories(${JSONCPP_INCLUDE_DIRS})
#    list(APPEND LIBS ${JSONCPP_LIBRARIES})
#ELSE (JSONCPP_FOUND)
#    message( FATAL_ERROR "JSONCPP not found." )
#ENDIF (JSONCPP_FOUND)


########################################################################
# Mediator
########################################################################
#include_directories(${PROJECT_SOURCE_DIR}/include)
#set(HEADER_FILES ${PROJECT_SOURCE_DIR}/include/mediator.h)


add_executable(ropod_wm_mediator_demo ${PROJECT_SOURCE_DIR}/src/demo_mediator.cpp ${HEADER_FILES})
target_link_libraries(ropod_wm_mediator_demo ${LIBS} ${catkin_LIBRARIES})

add_executable(test_wm_mediator ${PROJECT_SOURCE_DIR}/src/test.cpp ${HEADER_FILES})
target_link_libraries(test_wm_mediator ${LIBS} ${catkin_LIBRARIES})

