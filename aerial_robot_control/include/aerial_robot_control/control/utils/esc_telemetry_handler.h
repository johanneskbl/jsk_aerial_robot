#ifndef AERIAL_ROBOT_CONTROL_ESC_TELEMETRY_HANDLER_H
#define AERIAL_ROBOT_CONTROL_ESC_TELEMETRY_HANDLER_H

#include <ros/ros.h>
#include "spinal/ESCTelemetryArray.h"

#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace aerial_robot_control
{

/**
 * @brief Shared validity check and thrust conversion of the ESC telemetry from spinal
 * Spinal reads one ESC after the other and keeps the last values of an ESC without a valid frame in its round.
 * This is flagged by a non-zero crc_error (0xFF for no valid frame, otherwise the CRC mismatch of a corrupt frame).
 * Only valid frames update the measurement. An ESC is stale if it had no valid frame within the timeout, e.g., for a
 * dead ESC or a lost telemetry stream.
 * The thrust follows from the measured RPM -> thrust = ratio * krpm^2 + bias.
 */
class ESCTelemetryHandler
{
public:
  static constexpr int MAX_ESC_NUM = 4;  // spinal::ESCTelemetryArray has the fixed fields esc_telemetry_1 to _4

  ESCTelemetryHandler() = default;

  /**
   * @brief Read the thrust model from the motor_info parameters and check the configuration
   * @param nh Node handle in the robot namespace
   * @param esc_num Number of ESCs, i.e., motors
   * @param motor_num Number of motors
   * @param timeout Time without a valid frame after which the measurement of an ESC is stale [s]
   */
  void initialize(const ros::NodeHandle& nh, int esc_num, double timeout = 0.1)
  {
    ros::NodeHandle motor_nh(nh, "motor_info");
    motor_nh.param<double>("krpm_square_to_thrust_ratio", krpm_square_to_thrust_ratio_, 0.0);
    motor_nh.param<double>("krpm_square_to_thrust_bias", krpm_square_to_thrust_bias_, 0.0);
    if (krpm_square_to_thrust_ratio_ == 0.0)
    {
      std::string err_msg = "[ESC telemetry] motor_info/krpm_square_to_thrust_ratio is 0 or not set, so the thrust measured by the "
                "ESC telemetry is invalid! Load the motor config of the RPM mode.";
      ROS_ERROR("%s", err_msg.c_str());
      throw std::invalid_argument(err_msg.c_str());
    }

    if (esc_num != MAX_ESC_NUM)
    {
      std::string err_msg = "[ESC telemetry] The telemetry contains " + std::to_string(esc_num) + " ESCs, but " +
                            std::to_string(MAX_ESC_NUM) + " are expected.";
      ROS_ERROR("%s", err_msg.c_str());
      throw std::invalid_argument(err_msg.c_str());
    }
    esc_num_ = std::min(esc_num, MAX_ESC_NUM);
    timeout_ = timeout;
    last_valid_time_.assign(esc_num_, ros::Time(0));
  }

  /**
   * @brief Update the thrust and its rate of each ESC with a valid frame, to be called in the telemetry callback
   * @param msg ESC telemetry from spinal
   * @param thrust Thrust of each rotor in [N]
   * @param crc_error CRC error field for each ESC (0 for valid telemetry, non-zero for error)
   * @param thrust_rate Thrust rate of each rotor in [N/s] (optional)
   */
  void update(const spinal::ESCTelemetryArray& msg, std::vector<double>& thrust, std::vector<uint8_t>& crc_error,
              std::vector<double>* thrust_rate = nullptr)
  {
    const std::array<const spinal::ESCTelemetry*, MAX_ESC_NUM> escs = { &msg.esc_telemetry_1, &msg.esc_telemetry_2,
                                                                        &msg.esc_telemetry_3, &msg.esc_telemetry_4 };
    const ros::Time now = ros::Time::now();
    for (int i = 0; i < esc_num_; i++)
    {
      const spinal::ESCTelemetry& esc = *escs[i];
      if (esc.crc_error != 0)
      {
        crc_error.at(i) = esc.crc_error;
        continue;  // Values held by spinal from the last valid frame
      }

      // Models:
      //    thrust = ratio * krpm^2 + bias
      //    thrust_rate = d(thrust)/dt = 2 * ratio * krpm * d(krpm)/dt
      const double krpm = static_cast<double>(esc.rpm) * 0.001;
      const double krpm_rate = static_cast<double>(esc.rpm_rate) * 0.001;
      const double thrust_value = krpm_square_to_thrust_ratio_ * krpm * krpm + krpm_square_to_thrust_bias_;
      const bool is_thrust_positive = thrust_value > 0.0;

      if (thrust_value < 0.0)
      {
        ROS_ERROR_THROTTLE(1.0, "[ESC telemetry] Negative thrust detected for ESC %d: %.2f N", i + 1, thrust_value);
        thrust.at(i) = 0.0;
        if (thrust_rate != nullptr)
          thrust_rate->at(i) = 0.0;
      }
      else
      {
        thrust.at(i) = thrust_value;
        if (thrust_rate != nullptr)
          thrust_rate->at(i) = 2.0 * krpm_square_to_thrust_ratio_ * krpm * krpm_rate;
      }
      crc_error.at(i) = esc.crc_error;
      last_valid_time_[i] = now;
    }
  }

  /**
   * @brief Check if the measurement of an ESC is alive, i.e., it had a valid frame within the timeout
   * @param i Index of the ESC, starting from 0
   */
  bool isAlive(int i) const
  {
    return i < esc_num_ && (ros::Time::now() - last_valid_time_[i]).toSec() < timeout_;
  }

  /**
   * @brief Check if the measurements of all ESCs are alive and report the stale ones
   * @return true if all are alive, false otherwise
   */
  bool checkAlive() const
  {
    const ros::Time now = ros::Time::now();
    std::string stale_escs;
    for (int i = 0; i < esc_num_; i++)
    {
      if ((now - last_valid_time_[i]).toSec() >= timeout_)
        stale_escs += " " + std::to_string(i + 1);
    }
    if (stale_escs.empty())
      return true;

    ROS_ERROR_THROTTLE(1.0, "[ESC telemetry] No valid frame for %.0f ms from ESC%s! Holding the last valid thrust.",
                       timeout_ * 1000.0, stale_escs.c_str());
    return false;
  }

private:
  int esc_num_ = 0;
  double timeout_ = 0.1;
  double krpm_square_to_thrust_ratio_ = 0.0;  // [N/kRPM^2]
  double krpm_square_to_thrust_bias_ = 0.0;   // [N]
  std::vector<ros::Time> last_valid_time_;    // of each ESC
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_ESC_TELEMETRY_HANDLER_H
