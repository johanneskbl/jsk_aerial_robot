//
// Created by jinjie on 24/07/31.
//

#ifndef TILT_MT_SERVO_THRUST_DIST_NMPC_CONTROLLER_H
#define TILT_MT_SERVO_THRUST_DIST_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_dist_nmpc_controller.h"
#include "aerial_robot_control/control/utils/esc_telemetry_handler.h"

#include "spinal/ESCTelemetryArray.h"

namespace aerial_robot_control
{

namespace nmpc
{

/**
 * @brief MPC controller with a servo angle model, thrust model and disturbance model
 * The state is [p, v, q, w, alpha, thrust, external wrench], the thrust is measured by the ESC telemetry.
 * The input is [thrust, alpha]. 
 */
class TiltMtServoThrustDistNMPC : public TiltMtServoDistNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  ros::Subscriber sub_esc_telem_;
  ESCTelemetryHandler esc_telem_;

  std::vector<double> thrust_meas_;
  std::vector<u_int8_t> crc_errors_;  // record if a frame was valid

  /* Initialization methods */
  inline void initActuatorStates() override
  {
    TiltMtServoNMPC::initActuatorStates();
    thrust_meas_.resize(motor_num_, 0.0);
    crc_errors_.resize(motor_num_, 0);
  }
  void initMPCCostW() override;
  void initMPCConstraints() override;

  /* State methods */
  using TiltMtServoDistNMPC::meas2VecX;
  std::vector<double> meas2VecX(bool is_ee_centric) override;

  /* Control methods */
  void allocateToXU(const tf::Vector3& ref_pos_w, const tf::Vector3& ref_vel_w, const tf::Quaternion& ref_quat_wt,
                    const tf::Vector3& ref_omega_t, const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                    std::vector<double>& u) override;

  /* Callback methods */
  void callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg);
  void callbackMPCcfg(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_THRUST_DIST_NMPC_CONTROLLER_H
