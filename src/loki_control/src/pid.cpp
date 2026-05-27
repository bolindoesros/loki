/**
 * @file pid.cpp
 * @brief 1D PID controller implementation.
 */

#include "loki_control/pid.hpp"

namespace loki
{

PID::PID(const PIDParams & p) { set_params(p); }

void PID::set_params(const PIDParams & p)
{
  params_       = p;
  params_.alpha = std::clamp(p.alpha, 0.0, 1.0);
  if (params_.output_max <= params_.output_min) {
    params_.output_max =  1e9;
    params_.output_min = -1e9;
  }
}

void PID::reset()
{
  first_run_ = true;
  integral_  = 0.0;
  prev_err_  = 0.0;
  filt_d_    = 0.0;
  p_term_ = i_term_ = d_term_ = output_ = 0.0;
}

double PID::compute(double dt, double error)
{
  if (first_run_) {
    first_run_ = false;
    prev_err_  = error;
  }
  p_term_ = params_.kp * error;
  i_term_ = compute_integral(dt, error);
  d_term_ = compute_derivative_filtered(dt, error);
  output_ = saturate(p_term_ + i_term_ + d_term_);
  prev_err_ = error;
  return output_;
}

double PID::compute(double dt, double error, double error_dot)
{
  if (first_run_) {
    first_run_ = false;
    prev_err_  = error;
  }
  p_term_ = params_.kp * error;
  i_term_ = compute_integral(dt, error);
  d_term_ = compute_derivative_direct(error_dot);
  output_ = saturate(p_term_ + i_term_ + d_term_);
  prev_err_ = error;
  return output_;
}

double PID::compute_integral(double dt, double error)
{
  if (params_.reset_on_sign &&
      params_.antiwindup > 0.0 &&
      std::abs(integral_) > params_.antiwindup &&
      std::signbit(integral_) != std::signbit(error)) {
    integral_ = 0.0;
  }

  integral_ += error * dt;

  if (params_.antiwindup > 0.0) {
    integral_ = std::clamp(integral_, -params_.antiwindup, params_.antiwindup);
  }

  return params_.ki * integral_;
}

double PID::compute_derivative_filtered(double dt, double error)
{
  if (dt < std::numeric_limits<double>::epsilon()) return d_term_;
  double raw = (error - prev_err_) / dt;
  filt_d_    = params_.alpha * raw + (1.0 - params_.alpha) * filt_d_;
  return params_.kd * filt_d_;
}

double PID::compute_derivative_direct(double error_dot)
{
  return params_.kd * error_dot;
}

double PID::saturate(double v) const
{
  return std::clamp(v, params_.output_min, params_.output_max);
}

}  // namespace loki
