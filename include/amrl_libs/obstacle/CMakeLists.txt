
add_library(amrl_obstacle
  src/Obstacle.cpp
  src/ObstacleShape.cpp
  src/DynamicObstacle.cpp
  src/VibrationObstacle.cpp
  src/CircleObstacle.cpp
  src/PolygonObstacle.cpp)
target_link_libraries(amrl_obstacle)

add_library(amrl_obstacle_util
  src/ObstacleHelpers.cpp)
target_link_libraries(amrl_obstacle_util
  amrl_obstacle)