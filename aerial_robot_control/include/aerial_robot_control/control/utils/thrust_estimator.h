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
 * @brief Estimation of the thrust and thrust rate of each rotor at the actuation time of the next thrust command from
 * the delayed ESC telemetry and the history of the thrust commands
 * Rotor thrust model:       t_rotor * dF(t)/dt + F(t) = F_cmd(t - d_in)
 * Rotor thrust rate model:  t_thrust_rate * d^2F(t)/dt^2 + dF(t)/dt = K * dF_cmd(t - d_in)/dt
 * Measurement model:        F_meas(t) = F(t - d_meas)
 *                           dF_meas(t)/dt = dF(t - d_meas - d_rate)/dt
 * The actuation delay d_in is compensated by a state predictor as in B. Barros Carlos et al., "An Efficient Real-Time
 * NMPC for Quadrotor Position Control under Communication Time-Delay", ICARCV 2020: The command calculated from the
 * state read at time acts from the actuation time t_act = now + d_in on, so the thrust is not only compensated for the
 * measurement delay d_meas but further predicted until t_act.
 * Then the MPC needs no model of the actuation delay since its first control input acts from the delayed initial state onwards.
 * All commands acting until t_act are known since they were calculated before `now`.
 * NOTE: The rigid body states are NOT predicted since they change little within d_in, in contrast to the rotors with t_rotor ~ d_in.
 * Each thrust measurement is propagated over its age with the exact solution of its model for the piecewise constant thrust
 * commands that acted in the meantime. The exact solution of the thrust model is:
 * 
 *    F = F_cmd + (F_meas - F_cmd) * exp(-duration / t_rotor)
 * 
 * Each thrust rate measurement is propagated over its age with the exact solution of its model (!) for the piecewise constant
 * thrust commands that acted in the meantime. The rate model is the derivative of the first-order thrust model, but
 * with its own gain K_rate and time constant t_thrust_rate. The exact solution of the rate model is:
 * 
 *   dF/dt = dF_meas/dt * exp(-age / t_thrust_rate) + K / t_thrust_rate * sum_j step_j * exp(-(t_act - t_j) / t_thrust_rate)
 * 
 * where step_j is the change of the thrust command that acts from t_j on. The age is the measurement delay d_meas of
 * the ESC plus the time from the reception of the measurement until t_act, for the thrust rate additionally the delay
 * d_rate of the rate estimation on the ESC. Since the rate measurement is highly filtered its delay (~33 ms) covers
 * multiple command steps.
 *  - K_rate: With the gain K_rate < 1 the real rotor's rate reacts to a command step less strongly, and more
 *    smoothly, than an instantaneous first-order jump which fits the real behavior better
 *  - t_thrust_rate > t_rotor: The measured thrust rate decays slower than the thrust
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
    physical_nh.param<double>("t_thrust_rate", t_thrust_rate_, 0.0);
    physical_nh.param<double>("gain_thrust_rate", gain_thrust_rate_, 1.0);
    motor_nh.param<std::vector<double>>("delay_thrust_meas", delay_thrust_meas_, std::vector<double>());
    motor_nh.param<double>("delay_thrust_rate_meas", delay_thrust_rate_meas_, 0.0);
    motor_nh.param<double>("delay_thrust_cmd", delay_thrust_cmd_, 0.0);

    /* Sanity checks */
    if (t_rotor_ <= 0.0 || t_thrust_rate_ <= 0.0 || motor_num <= 0 ||
        static_cast<int>(delay_thrust_meas_.size()) != motor_num)
    {
      std::string err_msg = "[Thrust estimator] Time constants of rotor and thrust rate must be positive and delay vector "
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

    /* Initialize command history */
    // The command history covers published commands for the duration until the last useful command
    // It is used to propagate the measurements from the delayed time to the actuation time
    history_duration_ = *std::max_element(delay_thrust_meas_.begin(), delay_thrust_meas_.end()) +
                        delay_thrust_rate_meas_ + ESC_TELEMETRY_DEADTIME;
    last_call_time_ = ros::Time::now();;
  }

  /**
   * @brief Estimate the thrust and thrust rate of each rotor at the actuation time of the next thrust command
   * @param esc_telem ESC telemetry handler
   * @param thrust_meas Measured thrust of each rotor [N]
   * @param thrust_cmd Last thrust command of each rotor [N]
   * @param thrust Returned estimated thrust of each rotor at the actuation time [N]
   * @param thrust_rate_meas (optional) Measured thrust rate of each rotor [N/s]
   * @param thrust_rate (optional) Returned estimated thrust rate of each rotor at the actuation time [N/s]
   */
  void estimate(const ESCTelemetryHandler& esc_telem, const std::vector<double>& thrust_meas,
                const std::vector<float>& thrust_cmd, std::vector<double>& thrust,
                const std::vector<double>* thrust_rate_meas = nullptr, std::vector<double>* thrust_rate = nullptr)
  {
    const ros::Time now = ros::Time::now();

    /* Update the command history */
    // The last command was calculated from the state of the previous call and acts after the actuation delay
    // NOTE here we compensate for the actual actuation delay (NOT the delay with respect to the last publishing time!) which is more accurate
    thrust_cmd_history_.emplace_back(last_call_time_ + ros::Duration(delay_thrust_cmd_),
                                     std::vector<double>(thrust_cmd.begin(), thrust_cmd.end()));
    last_call_time_ = now;
    // Keep the command acting at the beginning of the history and all later ones until history window is covered
    while (thrust_cmd_history_.size() > 1 && (thrust_cmd_history_.at(1).first - now).toSec() <= -history_duration_)
      thrust_cmd_history_.pop_front();

    /* Propagate the measurements until the actuation time of the next command */
    const ros::Time t_act = now + ros::Duration(delay_thrust_cmd_);
    for (int i = 0; i < static_cast<int>(delay_thrust_meas_.size()); i++)
    {
      // Get time from the measurement until the actuation time
      const double age = delay_thrust_meas_[i] + (t_act - esc_telem.getLastValidTime(i)).toSec();  // now - esc_telem.getLastValidTime(i) ~= 0

      // Propagate thrust state forward to the actuation time, i.e., estimate its value based on past command and rotor model
      thrust.at(i) = propagate(i, thrust_meas.at(i), age, t_act);

      // Propagate thrust rate state forward to the actuation time with the exact solution of the rate model
      if (thrust_rate_meas != nullptr && thrust_rate != nullptr)
      {
        // Thrust rate has an additional and wider filter, i.e., the measurement corresponds to an older timestamp
        const double age_rate = age + delay_thrust_rate_meas_;

        // The measured rate decays and each command step since the measurement adds a decaying jump
        // The first command of the history has no known previous command, but acts before the age of the measurement
        double rate = thrust_rate_meas->at(i) * std::exp(-age_rate / t_thrust_rate_);
        for (size_t j = 1; j < thrust_cmd_history_.size(); j++)
        {
          const double t_cmd = (thrust_cmd_history_[j].first - t_act).toSec();  // Time relative to t_act from which the command acts
          if (t_cmd <= -age_rate || t_cmd > 0.0)
            continue;
          // NOTE: The thrust rate model is defined by dF_cmd/dt such that we need the difference in the command step history!
          const double cmd_step = thrust_cmd_history_[j].second.at(i) - thrust_cmd_history_[j - 1].second.at(i);
          rate += gain_thrust_rate_ / t_thrust_rate_ * cmd_step * std::exp(t_cmd / t_thrust_rate_);
        }
        thrust_rate->at(i) = rate;
      }
    }
  }

private:
  static constexpr double ESC_TELEMETRY_DEADTIME = 0.1;  // Longest potential tolerated ESC deadtime without a valid frame [s] 

  double t_rotor_ = 0.0;                   // Time constant of the rotor [s]
  std::vector<double> delay_thrust_meas_;  // Measurement delay d_meas of each ESC [s]
  double delay_thrust_rate_meas_ = 0.0;    // Measurement delay d_rate of the thrust rate [s]
  double delay_thrust_cmd_ = 0.0;          // Actuation delay d_in from the state of the MPC until the ESC acts [s]
  double gain_thrust_rate_ = 1.0;          // Short-term gain K of the rate model [-]
  double t_thrust_rate_ = 0.0;             // Time constant of the rate model [s]
  double history_duration_ = 0.0;          // Duration covered by the command history [s]
  ros::Time last_call_time_;               // Time of the previous call of estimate()
  std::deque<std::pair<ros::Time, std::vector<double>>> thrust_cmd_history_;  // Structured array to hold previous thrust commands and their time of actuation

  /**
   * @brief Get the thrust command acting on a rotor at a time relative to t_act, before the history the oldest command
   * @param i Index of the rotor
   * @param t Time relative to t_act [s]
   * @param t_act Actuation time of the next command
   */
  double getActiveCommand(int i, double t, const ros::Time& t_act) const
  {
    double cmd = thrust_cmd_history_.front().second.at(i);
    for (const auto& entry : thrust_cmd_history_)
    {
      if ((entry.first - t_act).toSec() <= t)  // entry.first is the actuation time, i.e., the publish time + actuation delay
        cmd = entry.second.at(i);              // entry.second is the actual thrust command published at entry.first
    }
    return cmd;
  }

  /**
   * @brief Propagate the thrust of a rotor until t_act with the exact solution of the rotor model for each command
   * @param i Index of the rotor
   * @param thrust Thrust at the time age before t_act [N]
   * @param age Duration of the propagation [s]
   * @param t_act Actuation time of the next command
   */
  double propagate(int i, double thrust, double age, const ros::Time& t_act) const
  {
    double t = -age;
    double cmd = getActiveCommand(i, t, t_act);
    for (const auto& entry : thrust_cmd_history_)
    {
      const double t_cmd = (entry.first - t_act).toSec();  // Time relative to t_act from which the command acts
      if (t_cmd <= t || t_cmd > 0.0)
        continue;
      thrust = cmd + (thrust - cmd) * std::exp(-(t_cmd - t) / t_rotor_);
      cmd = entry.second.at(i);
      t = t_cmd;
    }
    return cmd + (thrust - cmd) * std::exp(t / t_rotor_);  // Last command until t_act
  }
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_THRUST_ESTIMATOR_H
