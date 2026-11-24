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
 * Servo angle model:     t_servo * da(t)/dt + a(t) = a_cmd(t - d_in) + b
 * Servo velocity model:  t_servo_rate * d^2a(t)/dt^2 + da(t)/dt = K_rate * da_cmd(t - d_in)/dt
 * Measurement model:     a_meas(t) = a(t - d_meas)
 *                        da_meas(t)/dt = da(t - d_meas - d_vel)/dt
 * As in the ThrustEstimator, the actuation delay d_in is compensated by predicting the servo state until the actuation
 * time t_act = now + d_in, from which the next command acts. All commands acting until t_act are known since they were
 * calculated before `now`.
 * Each angle measurement is propagated over its age with the exact solution of its model for the piecewise constant
 * angle commands that acted in the meantime:
 *
 *    a = a_cmd + b + (a_meas - a_cmd - b) * exp(-duration / t_servo)
 *
 * The bias b is the steady-state offset of each servo, i.e., its position error under load, which varies slowly. It is
 * the low-pass (time constant t_servo_bias) of the difference between the measured angle and an open-loop model of the
 * sent commands without b, both at the time of the measurement. The controller subtracts b from the sent command, so
 * that the servo settles at the command of the MPC. With t_servo_bias = 0 the bias is not estimated and stays zero.
 *
 * The velocity model is the derivative of the angle model. Each velocity measurement is propagated over its age with
 * the exact solution of the velocity model for the piecewise constant angle commands that acted in the meantime:
 *
 *    da/dt = da_meas/dt * exp(-age / t_servo_rate)
 *            + K_rate / t_servo_rate * sum_j step_j * exp(-(t_act - t_j) / t_servo_rate)
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
    physical_nh.param<double>("t_servo_rate", t_servo_rate_, t_servo_);
    physical_nh.param<double>("gain_servo_rate", gain_servo_rate_, 1.0);
    servo_nh.param<double>("delay_servo_meas", delay_servo_meas_, 0.0);
    servo_nh.param<double>("delay_servo_vel_meas", delay_servo_vel_meas_, 0.0);
    servo_nh.param<double>("delay_servo_cmd", delay_servo_cmd_, 0.0);
    servo_nh.param<double>("t_servo_bias", t_servo_bias_, 0.0);

    /* Sanity checks */
    if (!std::isfinite(t_servo_) || t_servo_ <= 0.0 || !std::isfinite(t_servo_rate_) || t_servo_rate_ <= 0.0 ||
        !std::isfinite(gain_servo_rate_) || gain_servo_rate_ < 0.0 ||
        delay_servo_meas_ < 0.0 || delay_servo_vel_meas_ < 0.0 || delay_servo_cmd_ < 0.0 ||
        t_servo_bias_ < 0.0)
    {
      std::string err_msg = "[Servo estimator] Servo angle and velocity time constants must be finite and positive, "
                            "velocity gain must be finite and nonnegative, and measurement and command delays "
                            "as well as the bias time constant must not be negative!";
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
   * @param adapt_bias (optional) Update the bias with this measurement, only while the commands are sent
   */
  void estimate(const ros::Time& meas_time, const std::vector<double>& servo_meas, const std::vector<double>& servo_cmd,
                std::vector<double>& servo, const std::vector<double>* servo_vel_meas = nullptr,
                std::vector<double>* servo_vel = nullptr, bool adapt_bias = false)
  {
    const ros::Time now = ros::Time::now();

    /* Update the command history */
    // The last command was calculated from the state of the previous call and acts after the actuation delay
    servo_cmd_history_.emplace_back(last_call_time_ + ros::Duration(delay_servo_cmd_), servo_cmd);
    last_call_time_ = now;
    // Keep the command acting at the beginning of the history and all later ones until history window is covered
    while (servo_cmd_history_.size() > 1 && (servo_cmd_history_.at(1).first - now).toSec() <= -history_duration_)
      servo_cmd_history_.pop_front();

    /* Update the bias of each servo with a new measurement */
    if (t_servo_bias_ > 0.0 && adapt_bias && meas_time > last_meas_time_)
      updateBias(meas_time - ros::Duration(delay_servo_meas_), servo_meas);
    last_meas_time_ = meas_time;

    /* Propagate the measurements until the actuation time of the next command */
    const ros::Time t_act = now + ros::Duration(delay_servo_cmd_);
    // Get time from the measurement until the actuation time, the same for all servos of one message
    const double age = delay_servo_meas_ + (t_act - meas_time).toSec();
    for (int i = 0; i < static_cast<int>(servo_meas.size()); i++)
    {
      // Propagate angle state forward to the actuation time, i.e., estimate its value based on past command and servo model
      servo.at(i) = propagate(i, servo_meas.at(i), age, t_act, servo_bias_.at(i));

      // Propagate velocity state forward to the actuation time with the exact solution of the velocity model
      if (servo_vel_meas != nullptr && servo_vel != nullptr)
      {
        // Velocity is estimated over a window of past angles, i.e., the measurement corresponds to an older timestamp
        const double age_vel = age + delay_servo_vel_meas_;

        // The measured velocity decays and each command step since the measurement adds a decaying jump
        // The first command of the history has no known previous command, but acts before the age of the measurement
        double vel = servo_vel_meas->at(i) * std::exp(-age_vel / t_servo_rate_);
        for (size_t j = 1; j < servo_cmd_history_.size(); j++)
        {
          const double t_cmd = (servo_cmd_history_[j].first - t_act).toSec();  // Time relative to t_act from which the command acts
          if (t_cmd <= -age_vel || t_cmd > 0.0)
            continue;
          const double cmd_step = servo_cmd_history_[j].second.at(i) - servo_cmd_history_[j - 1].second.at(i);
          vel += gain_servo_rate_ / t_servo_rate_ * cmd_step * std::exp(t_cmd / t_servo_rate_);
        }
        servo_vel->at(i) = vel;
      }
    }
  }

  /**
   * @brief Get the bias of a servo, i.e., its steady-state offset to the sent command [rad]
   * @param i Index of the servo
   */
  double getBias(int i) const
  {
    return i < static_cast<int>(servo_bias_.size()) ? servo_bias_.at(i) : 0.0;
  }

private:
  static constexpr double SERVO_STATES_DEADTIME = 0.1;  // Longest potential tolerated deadtime without servo states [s]
  static constexpr double MAX_SERVO_BIAS = 0.035;        // Limit of bias (2 deg) [rad]

  double t_servo_ = 0.0;               // Time constant of the servo [s]
  double t_servo_rate_ = 0.0;          // Time constant of the velocity predictor [s]
  double gain_servo_rate_ = 1.0;       // Gain K_rate of the velocity predictor [-]
  double delay_servo_meas_ = 0.0;      // Measurement delay d_meas of the servo states [s]
  double delay_servo_vel_meas_ = 0.0;  // Measurement delay d_vel of the servo velocity [s]
  double delay_servo_cmd_ = 0.0;       // Actuation delay d_in from the state of the MPC until the servo acts [s]
  double history_duration_ = 0.0;      // Duration covered by the command history [s]
  double t_servo_bias_ = 0.0;          // Time constant of the bias estimation, should be >> 1 to only compensate static offsets; 0: no bias [s]
  ros::Time last_call_time_;           // Time of the previous call of estimate()
  ros::Time last_meas_time_;           // Receipt time of the previous servo states
  ros::Time open_loop_time_;           // Time of the open-loop model state
  std::vector<double> servo_bias_;     // Bias of each servo [rad]
  std::vector<double> servo_open_loop_;  // Open-loop model of the sent commands without bias at open_loop_time_ [rad]
  std::deque<std::pair<ros::Time, std::vector<double>>> servo_cmd_history_;  // Previous angle commands and their time of actuation

  /**
   * @brief Update the bias with the difference between the measured angle and the open-loop model at the measurement time
   * @param t_meas Time of the measured angles, i.e., receipt time - measurement delay
   * @param servo_meas Measured angle of each servo [rad]
   */
  void updateBias(const ros::Time& t_meas, const std::vector<double>& servo_meas)
  {
    const double duration = (t_meas - open_loop_time_).toSec();
    const bool restart = servo_open_loop_.size() != servo_meas.size() || duration <= 0.0 || duration > history_duration_;
    if (restart)
    {
      servo_open_loop_.resize(servo_meas.size());
      for (int i = 0; i < static_cast<int>(servo_meas.size()); i++)
      {
        // Start the open-loop model consistent with the current bias, e.g., after a pause of the adaptation
        servo_open_loop_.at(i) = servo_meas.at(i) - servo_bias_.at(i);
      }
    }
    else
    {
      for (int i = 0; i < static_cast<int>(servo_meas.size()); i++)
      {
        servo_open_loop_.at(i) = propagate(i, servo_open_loop_.at(i), duration, t_meas);
        const double residual = servo_meas.at(i) - servo_open_loop_.at(i);
        const double bias = servo_bias_.at(i) + (1.0 - std::exp(-duration / t_servo_bias_)) * (residual - servo_bias_.at(i));
        servo_bias_.at(i) = std::max(-MAX_SERVO_BIAS, std::min(MAX_SERVO_BIAS, bias));
      }
    }
    open_loop_time_ = t_meas;
  }

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
   * @param bias Steady-state offset of the servo to the command [rad]
   */
  double propagate(int i, double servo, double age, const ros::Time& t_act, double bias = 0.0) const
  {
    double t = -age;
    double cmd = getActiveCommand(i, t, t_act) + bias;
    for (const auto& entry : servo_cmd_history_)
    {
      const double t_cmd = (entry.first - t_act).toSec();  // Time relative to t_act from which the command acts
      if (t_cmd <= t || t_cmd > 0.0)
        continue;
      servo = cmd + (servo - cmd) * std::exp(-(t_cmd - t) / t_servo_);
      cmd = entry.second.at(i) + bias;
      t = t_cmd;
    }
    return cmd + (servo - cmd) * std::exp(t / t_servo_);  // Last command until t_act
  }
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_SERVO_ESTIMATOR_H
