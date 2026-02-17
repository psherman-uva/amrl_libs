add_library(amrl_mppi_controller
  src/MppiController.cpp
  src/MppiCost.cpp
  src/MppiHelpers.cpp)
target_link_libraries(amrl_mppi_controller
  amrl_robots
  amrl_filters
  amrl_mapping
  amrl_func_timer
  amrl_log_data_handler
  amrl_psherman_util)

add_library(amrl_mppi_path
  src/MppiPathPlanner.cpp)
target_link_libraries(amrl_mppi_path
  amrl_path_planning
  amrl_mapping)