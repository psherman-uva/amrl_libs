add_library(amrl_robots
  src/RobotBase.cpp
  src/Uav.cpp
  src/Ugv.cpp)
  # src/UgvRviz.cpp
  # src/UnicycleModel.cpp)
target_link_libraries(
  amrl_robots  
  amrl_obstacle
  ${catkin_LIBRARIES})
add_dependencies(amrl_robots amrl_display_generate_messages)


add_library(amrl_robot_helpers
  src/RobotHelpers.cpp)
target_link_libraries(
  amrl_robot_helpers
  amrl_robots  
  amrl_psherman_util
  ${catkin_LIBRARIES})