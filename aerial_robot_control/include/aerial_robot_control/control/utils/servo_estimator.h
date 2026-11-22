#ifndef AERIAL_ROBOT_CONTROL_SERVO_ESTIMATOR_H
#define AERIAL_ROBOT_CONTROL_SERVO_ESTIMATOR_H

#include <ros/ros.h>

#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace aerial_robot_control
{

/**
 * @brief Estimation of the angle and angular velocity of each servo at the actuation time of the next angle command
 * from the delayed servo states and the history of the angle commands, analogous to the ThrustEstimator
 * Servo angle model:     t_servo * da(t)/dt + a(t) = a_cmd(t - d_in)
 * Servo velocity model:  t_servo * d^2a(t)/dt^2 + da(t)/dt = da_cmd(t - d_in)/dt
 * Measurement model:     a_meas(t) = a(t - d_meas)
 *                        da_meas(t)/dt = da(t - d_meas - d_vel)/dt
 * As in the ThrustEstimator, the actuation delay d_in is compensated by predicting the servo state until the actuation
 * time t_act = now + d_in, from which the next command acts. All commands acting until t_act are known since they were
 * calculated before `now`.
 * Each angle measurement is propagated over its age with the exact solution of its model for the piecewise constant
 * angle commands that acted in the meantime:
 *
 *    a = a_cmd + (a_meas - a_cmd) * exp(-duration / t_servo)
 *
 * The velocity model is the derivative of the angle model. Each velocity measurement is propagated over its age with
 * the exact solution of the velocity model for the piecewise constant angle commands that acted in the meantime:
 *
 *    da/dt = da_meas/dt * exp(-age / t_servo) + 1 / t_servo * sum_j step_j * exp(-(t_act - t_j) / t_servo)
 *
 * where step_j is the change of the angle command that acts from t_j on. The age is the measurement delay d_meas of
 * the servo states plus the time from their reception until t_act, for the velocity additionally the delay d_vel of the
 * velocity estimation in spinal. Since the velocity is estimated over a window of past angles, its delay covers multiple
 * command steps.
 */
class ServoEstimator
{
public:
  ServoEstimator() = default;

  void initialize(const ros::NodeHandle& nh)
  {
    /* Get parameters */
    ros::NodeHandle physical_nh(nh, "physical");
    ros::NodeHandle servo_nh(nh, "servo_controller/gimbals");
    physical_nh.param<double>("t_servo", t_servo_, 0.0);
    servo_nh.param<double>("delay_servo_meas", delay_servo_meas_, 0.0);
    servo_nh.param<double>("delay_servo_vel_meas", delay_servo_vel_meas_, 0.0);
    servo_nh.param<double>("delay_servo_cmd", delay_servo_cmd_, 0.0);

    /* Sanity checks */
    if (t_servo_ <= 0.0 || delay_servo_meas_ < 0.0 || delay_servo_vel_meas_ < 0.0 || delay_servo_cmd_ < 0.0)
    {
      std::string err_msg = "[Servo estimator] Time constant of the servo must be positive and delays of measurement "
                            "and command must not be negative!";
      ROS_ERROR("%s", err_msg.c_str());
      throw std::invalid_argument(err_msg.c_str());
    }

    /* Initialize command history */
    // The command history covers published commands for the duration until the last useful command
    // It is used to propagate the measurements from the delayed time to the actuation time
    history_duration_ = delay_servo_meas_ + delay_servo_vel_meas_ + SERVO_STATES_DEADTIME;
    last_call_time_ = ros::Time::now();
  }

  /**
   * @brief Estimate the angle and angular velocity of each servo at the actuation time of the next angle command
   * @param meas_time Receipt time of the servo states
   * @param servo_meas Measured angle of each servo [rad]
   * @param servo_cmd Last angle command of each servo [rad]
   * @param servo Returned estimated angle of each servo at the actuation time [rad]
   * @param servo_vel_meas (optional) Measured angular velocity of each servo [rad/s]
   * @param servo_vel (optional) Returned estimated angular velocity of each servo at the actuation time [rad/s]
   */
  void estimate(const ros::Time& meas_time, const std::vector<double>& servo_meas, const std::vector<double>& servo_cmd,
                std::vector<double>& servo, const std::vector<double>* servo_vel_meas = nullptr,
                std::vector<double>* servo_vel = nullptr)
  {
    const ros::Time now = ros::Time::now();

    /* Update the command history */
    // The last command was calculated from the state of the previous call and acts after the actuation delay
    servo_cmd_history_.emplace_back(last_call_time_ + ros::Duration(delay_servo_cmd_), servo_cmd);
    last_call_time_ = now;
    // Keep the command acting at the beginning of the history and all later ones until history window is covered
    while (servo_cmd_history_.size() > 1 && (servo_cmd_history_.at(1).first - now).toSec() <= -history_duration_)
      servo_cmd_history_.pop_front();

    /* Propagate the measurements until the actuation time of the next command */
    const ros::Time t_act = now + ros::Duration(delay_servo_cmd_);
    // Get time from the measurement until the actuation time, the same for all servos of one message
    const double age = delay_servo_meas_ + (t_act - meas_time).toSec();
    for (int i = 0; i < static_cast<int>(servo_meas.size()); i++)
    {
      // Propagate angle state forward to the actuation time, i.e., estimate its value based on past command and servo model
      servo.at(i) = propagate(i, servo_meas.at(i), age, t_act);

      // Propagate velocity state forward to the actuation time with the exact solution of the velocity model
      if (servo_vel_meas != nullptr && servo_vel != nullptr)
      {
        // Velocity is estimated over a window of past angles, i.e., the measurement corresponds to an older timestamp
        const double age_vel = age + delay_servo_vel_meas_;

        // The measured velocity decays and each command step since the measurement adds a decaying jump
        // The first command of the history has no known previous command, but acts before the age of the measurement
        double vel = servo_vel_meas->at(i) * std::exp(-age_vel / t_servo_);
        for (size_t j = 1; j < servo_cmd_history_.size(); j++)
        {
          const double t_cmd = (servo_cmd_history_[j].first - t_act).toSec();  // Time relative to t_act from which the command acts
          if (t_cmd <= -age_vel || t_cmd > 0.0)
            continue;
          const double cmd_step = servo_cmd_history_[j].second.at(i) - servo_cmd_history_[j - 1].second.at(i);
          vel += cmd_step / t_servo_ * std::exp(t_cmd / t_servo_);
        }
        servo_vel->at(i) = vel;
      }
    }
  }

private:
  static constexpr double SERVO_STATES_DEADTIME = 0.1;  // Longest potential tolerated deadtime without servo states [s]

  double t_servo_ = 0.0;               // Time constant of the servo [s]
  double delay_servo_meas_ = 0.0;      // Measurement delay d_meas of the servo states [s]
  double delay_servo_vel_meas_ = 0.0;  // Measurement delay d_vel of the servo velocity [s]
  double delay_servo_cmd_ = 0.0;       // Actuation delay d_in from the state of the MPC until the servo acts [s]
  double history_duration_ = 0.0;      // Duration covered by the command history [s]
  ros::Time last_call_time_;           // Time of the previous call of estimate()
  std::deque<std::pair<ros::Time, std::vector<double>>> servo_cmd_history_;  // Previous angle commands and their time of actuation

  /**
   * @brief Get the angle command acting on a servo at a time relative to t_act, before the history the oldest command
   * @param i Index of the servo
   * @param t Time relative to t_act [s]
   * @param t_act Actuation time of the next command
   */
  double getActiveCommand(int i, double t, const ros::Time& t_act) const
  {
    double cmd = servo_cmd_history_.front().second.at(i);
    for (const auto& entry : servo_cmd_history_)
    {
      if ((entry.first - t_act).toSec() <= t)  // entry.first is the actuation time, i.e., the publish time + actuation delay
        cmd = entry.second.at(i);              // entry.second is the actual angle command published at entry.first
    }
    return cmd;
  }

  /**
   * @brief Propagate the angle of a servo until t_act with the exact solution of the servo model for each command
   * @param i Index of the servo
   * @param servo Angle at the time age before t_act [rad]
   * @param age Duration of the propagation [s]
   * @param t_act Actuation time of the next command
   */
  double propagate(int i, double servo, double age, const ros::Time& t_act) const
  {
    double t = -age;
    double cmd = getActiveCommand(i, t, t_act);
    for (const auto& entry : servo_cmd_history_)
    {
      const double t_cmd = (entry.first - t_act).toSec();  // Time relative to t_act from which the command acts
      if (t_cmd <= t || t_cmd > 0.0)
        continue;
      servo = cmd + (servo - cmd) * std::exp(-(t_cmd - t) / t_servo_);
      cmd = entry.second.at(i);
      t = t_cmd;
    }
    return cmd + (servo - cmd) * std::exp(t / t_servo_);  // Last command until t_act
  }
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_SERVO_ESTIMATOR_H
