
add_library(amrl_robot_data src/RobotData.cpp)
target_link_libraries(amrl_robot_data
  amrl_lidar
  amrl_mapping
  amrl_obstacle
  amrl_display_helpers
  amrl_obstacle_util
  amrl_psherman_util
  ${catkin_LIBRARIES}
)