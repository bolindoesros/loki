/**
 * @file pid.hpp
 * @brief 1D PID controller with derivative filter, anti-windup, output saturation.
 *
 * Based on Rafael Pérez Seguí (Universidad Politécnica de Madrid).
 * Adapted for the Loki AUV.
 */

#ifndef LOKI_CONTROL__PID_HPP_
#define LOKI_CONTROL__PID_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

namespace loki
{

struct PIDParams
{
  double kp             = 0.0;
  double ki             = 0.0;
  double kd             = 0.0;
  double alpha          = 1.0;   ///< Derivative low-pass (0=max filter, 1=none)
  double antiwindup     = 0.0;   ///< Integral clamp limit (0=disabled)
  double output_max     =  1e9;
  double output_min     = -1e9;
  bool   reset_on_sign  = false;
};

class PID
{
public:
  PID() = default;
  explicit PID(const PIDParams & p);

  void   set_params(const PIDParams & p);
  void   reset();

  /// Compute from error — derivative estimated internally via filter
  double compute(double dt, double error);

  /// Compute with explicit derivative — preferred when clean rate available
  double compute(double dt, double error, double error_dot);

  // Diagnostics
  double p_term()   const { return p_term_; }
  double i_term()   const { return i_term_; }
  double d_term()   const { return d_term_; }
  double output()   const { return output_; }
  double integral() const { return integral_; }

private:
  PIDParams params_;

  bool   first_run_ = true;
  double integral_  = 0.0;
  double prev_err_  = 0.0;
  double filt_d_    = 0.0;

  double p_term_ = 0.0;
  double i_term_ = 0.0;
  double d_term_ = 0.0;
  double output_ = 0.0;

  double compute_integral(double dt, double error);
  double compute_derivative_filtered(double dt, double error);
  double compute_derivative_direct(double error_dot);
  double saturate(double v) const;
};

}  // namespace loki

#endif  // LOKI_CONTROL__PID_HPP_
