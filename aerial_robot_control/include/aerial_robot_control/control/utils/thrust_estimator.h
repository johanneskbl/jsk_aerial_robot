#ifndef AERIAL_ROBOT_CONTROL_THRUST_ESTIMATOR_H
#define AERIAL_ROBOT_CONTROL_THRUST_ESTIMATOR_H

#include <ros/ros.h>
#include "aerial_robot_control/control/utils/esc_telemetry_handler.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace aerial_robot_control
{

/**
 * @brief Estimation of the current thrust and thrust rate of each rotor from the delayed ESC telemetry and the history
 * of the thrust commands
 * Rotor model:       dF/dt = (F_cmd(t - d_in) - F(t)) / t_rotor
 * Measurement model: F_meas(t) = F(t - d_meas)
 *                    dF_meas/dt(t) = dF/dt(t - d_meas - d_rate)
 * Each measurement is propagated over its age with the exact solution of the rotor model for each thrust command that
 * acted in the meantime:
 *   F = F_cmd + (F - F_cmd) * exp(-duration / t_rotor)
 * The age is the measurement delay d_meas of the ESC plus the time since the measurement was received, for the thrust
 * rate additionally the delay d_rate of the rate estimation on the ESC.
 * The rotor model links the thrust rate to the thrust by dF/dt = (F_cmd - F) / t_rotor, so the measured thrust rate is
 * propagated as its equivalent thrust F_cmd - t_rotor * dF/dt.
 */
class ThrustEstimator
{
public:
  ThrustEstimator() = default;

  void initialize(const ros::NodeHandle& nh, int motor_num)
  {
    /* Get parameters */
    ros::NodeHandle physical_nh(nh, "physical");
    ros::NodeHandle motor_nh(nh, "motor_info");
    physical_nh.param<double>("t_rotor", t_rotor_, 0.0);
    motor_nh.param<std::vector<double>>("delay_thrust_meas", delay_thrust_meas_, std::vector<double>());
    motor_nh.param<double>("delay_thrust_rate_meas", delay_thrust_rate_meas_, 0.0);
    motor_nh.param<double>("delay_thrust_cmd", delay_thrust_cmd_, 0.0);

    /* Initialize command history */
    // The command history covers published commands for the duration until the last useful command
    // It is used to propagate the measurements from the delayed time to current time
    history_duration_ = *std::max_element(delay_thrust_meas_.begin(), delay_thrust_meas_.end()) +
                        delay_thrust_rate_meas_ + ESC_TELEMETRY_DEADTIME;
    last_call_time_ = ros::Time::now();;

    /* Sanity checks */
    if (t_rotor_ <= 0.0 || motor_num <= 0 || static_cast<int>(delay_thrust_meas_.size()) != motor_num)
    {
      std::string err_msg = "[Thrust estimator] Rotor time constant must be positive and delay vector "
                            "must have " + std::to_string(motor_num) + " entries " +
                            "(found " + std::to_string(delay_thrust_meas_.size()) + ")!";
      ROS_ERROR("%s", err_msg.c_str());
      throw std::invalid_argument(err_msg.c_str());
    }
    for (int i = 0; i < motor_num; i++)
    {
      if (delay_thrust_meas_.at(i) < 0.0 || delay_thrust_rate_meas_ < 0.0 || delay_thrust_cmd_ < 0.0)
      {
        std::string err_msg = "[Thrust estimator] Delays of measurement and command must not be negative!";
        ROS_ERROR("%s", err_msg.c_str());
        throw std::invalid_argument(err_msg.c_str());
      }
    }
  }

  /**
   * @brief Estimate the current thrust and thrust rate of each rotor
   * @param esc_telem ESC telemetry handler
   * @param thrust_meas Measured thrust of each rotor [N]
   * @param thrust_cmd Last thrust command of each rotor [N]
   * @param thrust Returned estimated thrust of each rotor at current time [N]
   * @param thrust_rate_meas (optional) Measured thrust rate of each rotor [N/s]
   * @param thrust_rate (optional) Returned estimated thrust rate of each rotor at current time [N/s]
   */
  void estimate(const ESCTelemetryHandler& esc_telem, const std::vector<double>& thrust_meas,
                const std::vector<float>& thrust_cmd, std::vector<double>& thrust,
                const std::vector<double>* thrust_rate_meas = nullptr, std::vector<double>* thrust_rate = nullptr)
  {
    const ros::Time now = ros::Time::now();

    /* Update the command history */
    // The last command was calculated from the state of the previous call and acts after the actuation delay
    thrust_cmd_history_.emplace_back(last_call_time_ + ros::Duration(delay_thrust_cmd_),
                                     std::vector<double>(thrust_cmd.begin(), thrust_cmd.end()));
    last_call_time_ = now;
    // Keep the command acting at the beginning of the history and all later ones until history window is covered
    while (thrust_cmd_history_.size() > 1 && (thrust_cmd_history_.at(1).first - now).toSec() <= -history_duration_)
      thrust_cmd_history_.pop_front();

    /* Propagate the measurements until now */
    for (int i = 0; i < static_cast<int>(delay_thrust_meas_.size()); i++)
    {
      // Get time since the measurement was taken
      const double age = delay_thrust_meas_[i] + (now - esc_telem.getLastValidTime(i)).toSec();  // now - esc_telem.getLastValidTime(i) ~= 0
      
      // Propagate thrust state forward to current time, i.e., estimate its value based on past command and rotor model
      thrust.at(i) = propagate(i, thrust_meas.at(i), age, now);

      // Propagate thrust rate state forward to current time, i.e., estimate its value based on past command and rotor model
      if (thrust_rate_meas != nullptr && thrust_rate != nullptr)
      {
        // Thrust rate has an additional and wider filter, i.e., the measurement corresponds to an older timestamp
        const double age_rate = age + delay_thrust_rate_meas_;

        // Compute the equivalent thrust that would have caused the measured thrust rate at the time of the measurement based on the published command
        // tau * dF/dt + F = command <=> F = command - tau * dF/dt
        const double thrust_equivalent = getActiveCommand(i, -age_rate, now) - t_rotor_ * thrust_rate_meas->at(i);

        // Estimate the current thrust rate from propagating the equivalent thrust and current command
        // tau * dF/dt + F = command <=> dF/dt = (command - F) / tau
        thrust_rate->at(i) = (getActiveCommand(i, 0.0, now) - propagate(i, thrust_equivalent, age_rate, now)) / t_rotor_;
      }
    }
  }

private:
  static constexpr double ESC_TELEMETRY_DEADTIME = 0.1;  // Longest potential tolerated ESC deadtime without a valid frame [s] 

  double t_rotor_ = 0.0;                   // Time constant of the rotor [s]
  std::vector<double> delay_thrust_meas_;  // Measurement delay d_meas of each ESC [s]
  double delay_thrust_rate_meas_ = 0.0;    // Measurement delay d_rate of the thrust rate [s]
  double delay_thrust_cmd_ = 0.0;          // Actuation delay d_in from the state of the MPC until the ESC acts [s]
  double history_duration_ = 0.0;          // Duration covered by the command history [s]
  ros::Time last_call_time_;               // Time of the previous call of estimate()
  std::deque<std::pair<ros::Time, std::vector<double>>> thrust_cmd_history_;  // Structured array to hold previous thrust commands and their time of actuation

  /**
   * @brief Get the thrust command acting on a rotor at a time relative to now, before the history the oldest command
   * @param i Index of the rotor
   * @param t Time relative to now [s]
   * @param now Current time
   */
  double getActiveCommand(int i, double t, const ros::Time& now) const
  {
    double cmd = thrust_cmd_history_.front().second.at(i);
    for (const auto& entry : thrust_cmd_history_)
    {
      if ((entry.first - now).toSec() <= t)  // entry.first is the publish time
        cmd = entry.second.at(i);            // entry.second is the actual thrust command published at entry.first
    }
    return cmd;
  }

  /**
   * @brief Propagate the thrust of a rotor until now with the exact solution of the rotor model for each command
   * @param i Index of the rotor
   * @param thrust Thrust at the time age ago [N]
   * @param age Duration of the propagation [s]
   * @param now Current time
   */
  double propagate(int i, double thrust, double age, const ros::Time& now) const
  {
    double t = -age;
    double cmd = getActiveCommand(i, t, now);
    for (const auto& entry : thrust_cmd_history_)
    {
      const double t_cmd = (entry.first - now).toSec();  // Time relative to now from which the command acts
      if (t_cmd <= t || t_cmd > 0.0)
        continue;
      thrust = cmd + (thrust - cmd) * std::exp(-(t_cmd - t) / t_rotor_);
      cmd = entry.second.at(i);
      t = t_cmd;
    }
    return cmd + (thrust - cmd) * std::exp(t / t_rotor_);  // Last command until now
  }
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_THRUST_ESTIMATOR_H
