
#include <amrl_libs/pid/PID.hpp>

#include <limits>
#include <cmath>

namespace amrl  {

PID::PID(const PIDGains &gains, const bool use_prev_u) :
  _gains(gains),
  _previous_err(0.0),
  _integral_sum(0.0),
  _previous_u(0.0),
  _use_prev_u(use_prev_u),
  _integral_windup_active(false),
  _integral_windup_limit(std::numeric_limits<double>::max()),
  _derivative_limit_active(false),
  _derivative_limit(std::numeric_limits<double>::max())
{
  reset();
}

PID::PID(const double Kp, const double Ki, const double Kd, const bool use_prev_u) :
  PID(PIDGains(Kp, Ki, Kd), use_prev_u)
{
}

double PID::loop(const double setpoint, const double measurement, const double dt)
{
  // Calculate error values
  double error = setpoint - measurement;
  _integral_sum += dt*(error + _previous_err)/2.0;
  double derivative = (error - _previous_err)/dt;

  // Adjustments
  if(_integral_windup_active) {
    _integral_sum = std::fmin(_integral_windup_limit, 
      std::fmax(-_integral_windup_limit, _integral_sum));
  }

  // PID control output
  _output_p = _gains.Kp*error;
  _output_i = _gains.Ki*_integral_sum;
  
  _output_d = _gains.Kd*derivative;
  if(_derivative_limit_active) {
    _output_d = std::fmin(_derivative_limit, std::fmax(-_derivative_limit, _output_d));
  }
  
  double u = 0;
  if(_use_prev_u) {
    u = _previous_u + _output_p + _output_i + _output_d;
  } else {
    u = _output_p + _output_i + _output_d;
  }

  // Cache values for next loop
  _previous_err = error;
  _previous_u   = u;

  return u;
}

void PID::reset(void)
{
  _previous_err = 0.0;
  _previous_u   = 0.0;
  _integral_sum = 0.0;
}

void PID::integral_windup_enable(const bool enable)
{
  _integral_windup_active = enable;
}

void PID::integral_windup_limit(const double limit)
{
  if(limit > 0) { 
    _integral_windup_limit = limit;
  }
}

double PID::intregral_windup_value(void) const
{
  return _integral_sum;
}

void PID::derivative_limit_enable(const bool enable)
{
  _derivative_limit_active = enable;
}

void PID::derivative_limit_set(const double limit)
{
  if(limit > 0) {
    _derivative_limit = limit;
  }
}

Eigen::Vector3d PID::output_components(void) const
{
  return Eigen::Vector3d({_output_p, _output_i, _output_d});
}

} // namespace amrl