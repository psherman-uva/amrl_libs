
#pragma once

#include <amrl_libs/trajectory_generation/Trajectory.hpp>
#include <amrl_common/point/Point.hpp>

namespace amrl {

class TrajectoryGenSimple
{
public:
  /// Constructor
  /// @param  
  TrajectoryGenSimple(void);

  /// Destructor
  ~TrajectoryGenSimple(void) = default;


  Trajectory generate(
    const std::vector<Point<double>> &path, 
    double v_ref,
    double dt) const;

private:

  double _vel_des;


};

} // namespace amrl
