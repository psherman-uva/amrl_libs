
add_library(amrl_mapping
  src/Grid.cpp
  src/GridInfo.cpp
  src/GridGraph.cpp
  src/OccupancyGrid.cpp
  src/Frontier.cpp
  src/TerrainGrid.cpp)
target_link_libraries(amrl_mapping
  amrl_util
  amrl_obstacle
  ${catkin_LIBRARIES})
