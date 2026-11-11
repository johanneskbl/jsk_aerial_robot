#ifndef TILT_MT_SERVO_THRUST_DIFF_MPC_CONTROLLER_H
#define TILT_MT_SERVO_THRUST_DIFF_MPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_nmpc_controller.h"
#include "aerial_robot_control/control/utils/esc_telemetry_handler.h"

#include "spinal/ESCTelemetryArray.h"

namespace aerial_robot_control
{

namespace nmpc
{

/**
 * @brief Differential MPC controller with actuator velocity control and nullspace control
 * The servo angles and thrusts are states and the inputs are their velocities.
 * The state is [p, v, q, w, alpha, thrust, (optional) internal wrench].
 * The input is [thrust velocity, alpha velocity].
 * The velocity commands are integrated to the thrust and servo angle (alpha) commands.
 */
class TiltMtServoThrustDiffMPC : public TiltMtServoNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;
  void reset() override;

protected:
  double thrust_vel_min_;
  double thrust_vel_max_;
  double servo_vel_min_;
  double servo_vel_max_;
  std::vector<double> thrust_meas_;
  std::vector<double> thrust_rate_meas_;  // from the rpm rate estimated on the ESC
  std::vector<u_int8_t> crc_errors_;  // record if a frame was valid
  ros::Subscriber sub_esc_telem_;
  ESCTelemetryHandler esc_telem_;

  int servo_start_idx_;
  int servo_vel_start_idx_;
  int thrust_start_idx_;
  int thrust_vel_start_idx_;
  int wrench_state_start_idx_;
  std::vector<double> uo_prev_;
  Eigen::VectorXd internal_wrench_b_;
  bool include_nullspace_control_;

  /* Initialization methods */
  void initActuatorStates() override;
  void initMPCCostW() override;
  void initMPCConstraints() override;

  /* Reset methods */
  void resetSolver() override;

  /* State methods */
  using TiltMtServoNMPC::meas2VecX;
  std::vector<double> meas2VecX(bool is_ee_centric) override;
  // Guard for accessing wrench indices because nullspace control is optional
  inline bool hasWrenchState() const { return mpc_solver_ptr_->NX_ == wrench_state_start_idx_ + 6; }
  void computeInternalWrenchB();

  /* Update methods */
  double getCommand(int idx_u, double T_horizon) override;
  void publishRecording() override;

  /* Control methods */
  void allocateToXU(const tf::Vector3& ref_pos_w, const tf::Vector3& ref_vel_w, const tf::Quaternion& ref_quat_wt,
                    const tf::Vector3& ref_omega_t, const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                    std::vector<double>& u) override;
  void allocateToXUwOneFixedRotor(int fix_rotor_idx, double fix_ft, double fix_alpha,
                                  const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                                  std::vector<double>& u) override;

  /* Callback methods */
  void callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg);
  void callbackMPCcfg(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_THRUST_DIFF_MPC_CONTROLLER_H
