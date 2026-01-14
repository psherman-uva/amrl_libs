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