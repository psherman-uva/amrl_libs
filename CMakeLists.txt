cmake_minimum_required(VERSION 3.0.2)
project(amrl_libs)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_compile_options(
  $<$<COMPILE_LANGUAGE:C>:-std=c17>
  -pthread
  -O3
)

set(JSON_Install OFF CACHE INTERNAL "")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  tf2_ros
  std_msgs
  geometry_msgs
  message_generation
  amrl_common
  amrl_logging
  ## Optional sub-packages. If not found, CMake will just ignore any
  ## directories with that particular dependencies
  OPTIONAL_COMPONENTS
  amrl_display
)
find_package(Threads) # Find c++ threads library
find_package(Boost)   # Boost software libraries

# catkin_ALL_FOUND_COMPONENTS = roscpp;rospy;roslaunch;tf2_ros;std_msgs;geometry_msgs;message_generation;amrl_common;amrl_display

## Specify additional locations of header files
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(DIRECTORY ${PROJECT_SOURCE_DIR}/ros_msg
  FILES
  Circle.msg
  CircleList.msg
  FloatVector.msg
  PointVector.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  INCLUDE_DIRS include
)

#########################
## Project directories ##
#########################

add_subdirectory(include)

