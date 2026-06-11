add_library(amrl_trajectory_gen
    src/Trajectory.cpp
    src/TrajectoryGenSimple.cpp)
# target_link_libraries(amrl_trajectory_gen amrl_mapping amrl_obstacle)

add_executable(amrl_traj_test tests/trajectory_test.cpp)
target_link_libraries(amrl_traj_test amrl_trajectory_gen)