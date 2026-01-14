
add_library(amrl_psherman_util 
  src/ros_util.cpp)
  # src/prob_util.cpp)
target_link_libraries(amrl_psherman_util 
  amrl_util
  amrl_mapping
  amrl_lidar
  amrl_obstacle_util
  amrl_logging_util
  # approx_cdf
  ${catkin_LIBRARIES})
add_dependencies(amrl_psherman_util amrl_logging_generate_messages)
