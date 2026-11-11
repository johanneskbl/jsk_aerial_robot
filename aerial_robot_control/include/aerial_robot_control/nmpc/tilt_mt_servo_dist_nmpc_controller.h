//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_MT_SERVO_DIST_NMPC_CONTROLLER_H
#define TILT_MT_SERVO_DIST_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_nmpc_controller.h"
#include "aerial_robot_control/wrench_est/wrench_est_actuator_meas_base.h"
#include "aerial_robot_control/wrench_est/wrench_est_i_term.h"

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

namespace aerial_robot_control
{

namespace nmpc
{

/**
 * @brief MPC controller with a servo angle model and disturbance model
 * The state is [p, v, q, w, alpha, f_w, tau_b] with the external wrench estimated from a wrench estimator plugin,
 * and the model parameters by the model error from an I-term.
 */
class TiltMtServoDistNMPC : public TiltMtServoNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

  aerial_robot_control::WrenchEstITerm wrench_est_i_term_;  // I-term is indispensable to eliminate steady error.

  boost::shared_ptr<pluginlib::ClassLoader<aerial_robot_control::WrenchEstActuatorMeasBase>> wrench_est_loader_ptr_;
  boost::shared_ptr<aerial_robot_control::WrenchEstActuatorMeasBase> wrench_est_ptr_;

protected:
  bool if_use_est_wrench_4_control_ = false;

  ros::Publisher pub_disturb_wrench_;  // for disturbance wrench

  int idx_p_dist_end_ = 0;

  /* Initialization methods */
  void initPlugins() override;

  /* Reset methods */
  void resetPlugins() override;

  /* Activate methods */
  void initMPCPhysicalParams() override;
  void initAllocMat() override;

  /* State methods */
  using TiltMtServoNMPC::meas2VecX;
  std::vector<double> meas2VecX(bool is_ee_centric) override;

  /* Update methods */
  void publishRecording() override;

  /* Control methods */
  void prepareMPCParams() override;

  /* External wrench estimation */
  void updateDisturbWrench() const;
  void pubDisturbWrench() const;

  /* I-term */
  void updateITerm();
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_DIST_NMPC_CONTROLLER_H
