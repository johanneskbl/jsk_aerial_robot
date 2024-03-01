//
// Created by li-jinjie on 24-3-1.
//

#ifndef AERIAL_ROBOT_CONTROL_I_TERM_H
#define AERIAL_ROBOT_CONTROL_I_TERM_H

#endif  // AERIAL_ROBOT_CONTROL_I_TERM_H

#pragma once

#include <algorithm>

namespace aerial_robot_control
{
class ITerm
{
public:
  ITerm() = default;
  ~ITerm() = default;

  void initialize(double i_gain, double i_limit, double dt)
  {
    i_gain_ = i_gain;
    i_limit_ = i_limit;
    dt_ = dt;
    i_term_ = 0;
    error_last_round_ = 0;
  }

  double update(double y_ref, double y)
  {
    double error = y_ref - y;

    // update the integrator using trapazoidal rule
    i_term_ += (dt_ / 2) * (error + error_last_round_);

    // limit the integrator  TODO: add anti-windup
    i_term_ = std::max(std::min(i_term_, i_limit_), -i_limit_);

    error_last_round_ = error;

    return i_gain_ * i_term_;
  }

  void reset()
  {
    i_term_ = 0;
    error_last_round_ = 0;
  }

private:
  double i_gain_;
  double i_limit_;
  double dt_;
  double i_term_;
  double error_last_round_;
};
}  // namespace aerial_robot_control