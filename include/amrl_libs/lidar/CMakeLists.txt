add_library(amrl_lidar
  src/LidarSimulated.cpp
  src/LidarSimulatedMap.cpp
  src/LidarSimulatedObstacles.cpp)
target_link_libraries(amrl_lidar 
  amrl_obstacle
  amrl_mapping)