#ifndef AERIAL_ROBOT_CONTROL_BODY_STATE_ESTIMATOR_H
#define AERIAL_ROBOT_CONTROL_BODY_STATE_ESTIMATOR_H

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

#include <stdexcept>
#include <string>

namespace aerial_robot_control
{

/**
 * @brief Estimation of the Body state at the actuation time of the next command from the delayed measurement
 * and the MPC model
 * Rigid body model:   d(pos)/dt = vel
 *                     d(vel)/dt = acc_w
 *                     dq/dt = q * omega / 2
 *                     d(omega)/dt = ang_acc_b
 * Measurement model:  x_meas(t) = x(t - d_meas)
 * 
 * The thrust is the first command that acts on the body, so the body is predicted to the thrust's actuation time.
 * The actuation delay d_in is compensated by predicting the state until the actuation time t_act = now + d_in,
 * from which the next command acts.
 * The state estimate is propagated over its age with the model for constant accelerations:
 *
 *    pos = pos_meas + vel_meas * age + acc_w * age^2 / 2
 *    vel = vel_meas + acc_w * age
 *    q = q_meas * exp((omega_meas * age + ang_acc_b * age^2 / 2) / 2)
 *    omega = omega_meas + ang_acc_b * age
 *
 * where the rotation of q is exact for a fixed axis of the angular velocity. The linear acceleration acc_w (in World
 * frame) and the angular acceleration ang_acc_b (in Body frame) are those of the MPC model over the first shooting
 * interval of its last solution, i.e., they contain the actuator states and commands, gravity and (optionally) disturbances.
 * Otherwise, if the delay is not compensated, there will be damping of the closed loop.
 */
class BodyStateEstimator
{
public:
  double age_ = 0.0;

  BodyStateEstimator() = default;

  void initialize(const ros::NodeHandle& nh)
  {
    /* Get parameters */
    ros::NodeHandle physical_nh(nh, "physical");
    physical_nh.param<double>("delay_body_meas", delay_body_meas_, 0.0);
    physical_nh.param<double>("delay_thrust_cmd", delay_thrust_cmd_, 0.0);

    /* Sanity checks */
    if (delay_body_meas_ < 0.0 || delay_thrust_cmd_ < 0.0)
    {
      std::string err_msg = "[Body state estimator] Delays of measurement and command must not be negative!";
      ROS_ERROR("%s", err_msg.c_str());
      throw std::invalid_argument(err_msg.c_str());
    }

    /* Compute the age of the state estimate at the actuation time */
    age_ = delay_body_meas_ + delay_thrust_cmd_;
  }

  /**
   * @brief Estimate the Body state at the actuation time of the next command
   * @param pos Measured position, returned as the estimated position at the actuation time [m]
   * @param vel Measured linear velocity in World frame, returned as the estimate at the actuation time [m/s]
   * @param acc_w Linear acceleration in World frame [m/s^2]
   * @param q Measured orientation of the Body frame w.r.t. World frame, returned as the estimate at the actuation time
   * @param omega Measured angular velocity in Body frame, returned as the estimate at the actuation time [rad/s]
   * @param ang_acc_b Angular acceleration in Body frame [rad/s^2]
   */
  void estimate(tf::Vector3& pos, tf::Vector3& vel, const tf::Vector3& acc_w,
                tf::Quaternion& q, tf::Vector3& omega, const tf::Vector3& ang_acc_b) const
  {
    /* Propagate the state forward to the actuation time with the model */
    pos += vel * age_ + acc_w * (age_ * age_ / 2.0);
    vel += acc_w * age_;
    const tf::Vector3 rotation_b = omega * age_ + ang_acc_b * (age_ * age_ / 2.0);  // Rotation vector in Body frame
    if (rotation_b.length() > 0.0)
      q *= tf::Quaternion(rotation_b, rotation_b.length());  // Rotation about the (normalized) axis by the angle
    omega += ang_acc_b * age_;
  }

private:
  double delay_body_meas_ = 0.0;   // Measurement delay d_meas of the state estimate [s]
  double delay_thrust_cmd_ = 0.0;  // Actuation delay d_in from the state of the MPC until the rotor acts [s]
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_BODY_STATE_ESTIMATOR_H
