#ifndef TILT_MT_SERVO_THRUST_DIFF_MPC_CONTROLLER_H
#define TILT_MT_SERVO_THRUST_DIFF_MPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_nmpc_controller.h"

#include "spinal/ESCTelemetryArray.h"

namespace aerial_robot_control
{

namespace nmpc
{

class TiltMtServoThrustDiffMPC : public TiltMtServoNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  double thrust_vel_min_;
  double thrust_vel_max_;
  double servo_vel_min_;
  double servo_vel_max_;
  double krpm_square_to_thrust_ratio_;
  double krpm_square_to_thrust_bias_;
  std::vector<double> thrust_meas_;
  ros::Subscriber sub_esc_telem_;

  int servo_start_idx_, servo_vel_start_idx_, thrust_start_idx_, thrust_vel_start_idx_, wrench_state_start_idx_;
  std::vector<double> uo_prev_;
  Eigen::VectorXd internal_wrench_b_;

  virtual void initActuatorStates();
  
  void initGeneralParams() override;

  virtual void initNMPCCostW();

  virtual void initNMPCConstraints();

  void callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg);

  virtual std::vector<double> meas2VecX(bool is_modified_by_traj_frame);

  virtual void allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i, const tf::Quaternion& ref_quat_ib,
                            const tf::Vector3& ref_omega_b, const VectorXd& ref_wrench_b, vector<double>& x,
                            vector<double>& u);

  void allocateToXUwOneFixedRotor(int fix_rotor_idx, double fix_ft, double fix_alpha, const VectorXd& ref_wrench_b,
                                  vector<double>& x, vector<double>& u) override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;

  void computeInternalWrenchB();

  double getCommand(int idx_u, double T_horizon) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_THRUST_DIFF_MPC_CONTROLLER_H
