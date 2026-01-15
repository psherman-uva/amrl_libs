
add_library(amrl_display_helpers
  src/FigureDisplay.cpp
  src/FrontierDisplay.cpp
  src/ObstacleDisplay.cpp
  src/OccupancyGridDisplay.cpp
  src/RobotDisplay.cpp
  src/TaskDisplay.cpp
  src/TerrainGridDisplay.cpp
)
target_link_libraries(
  amrl_display_helpers
  amrl_robot_helpers
  amrl_mapping
  amrl_obstacle_util
  ${catkin_LIBRARIES}
)

add_dependencies(amrl_display_helpers amrl_display_generate_messages)
