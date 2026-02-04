
add_library(amrl_path_planning 
  src/AStar.cpp
  src/AStarDynObs.cpp
  src/Dijkstra.cpp
  src/PathPlanningFunctions.cpp)
target_link_libraries(amrl_path_planning amrl_mapping amrl_obstacle)