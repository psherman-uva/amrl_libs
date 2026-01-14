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
  amrl_display
  amrl_logging
  amrl_common
)
find_package(Threads) # Find c++ threads library
find_package(Boost)   # Boost software libraries

## Specify additional locations of header files
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS})


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

