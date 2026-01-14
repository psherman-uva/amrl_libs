
#pragma once

#include <amrl_libs/pid/PIDGains.hpp>
#include <vector>

namespace amrl {

class PID 
{
public:

  /// Constructor
  /// @param gains PID gains object
  PID(const PIDGains &gains, const bool use_prev_u = false);

  /// Constructor
  /// @param  Kp Proportional gain for PID loop
  /// @param  Ki Integral gain for PID loop
  /// @param  Kd Derivative gain for PID loop
  PID(const double Kp, const double Ki, const double Kd, const bool use_prev_u = false);
    
  // Destructor
  ~PID(void) = default;

  /// Run a single loop of a PID controller
  /// @param  setpoint The reference or desired setpoint of the system
  /// @param  measurement The current measured value of the systems
  /// @param  dt The time step since the last loop update
  double loop(const double setpoint, const double measurement, const double dt);

  /// Resets PID controller to an initial state
  void reset(void);

  void integral_windup_enable(const bool enable);
  void integral_windup_limit(const double limit);
  double intregral_windup_value(void) const;

  void derivative_limit_enable(const bool enable);
  void derivative_limit_set(const double limit);

  Eigen::Vector3d output_components(void) const;

private:

  PIDGains _gains;

  double _output_p;
  double _output_i;
  double _output_d;

  double _previous_err; // Error from last PID cycle
  double _integral_sum; // Integral of error
  double _previous_u;   // Last output from PID cycle
  bool   _use_prev_u;   // For some situations, you want to build upon the previous u. Others no.
  
  bool _integral_windup_active;
  double _integral_windup_limit;

  bool _derivative_limit_active;
  double _derivative_limit;
};

} // namespace amrl