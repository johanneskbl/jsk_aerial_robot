#ifndef TILT_MT_SERVO_THRUST_DIFF_SECOND_ORDER_MPC_CONTROLLER_H
#define TILT_MT_SERVO_THRUST_DIFF_SECOND_ORDER_MPC_CONTROLLER_H

#include "aerial_robot_control/differential_mpc/tilt_mt_servo_thrust_diff_mpc_controller.h"

#include "spinal/ESCTelemetryArray.h"

namespace aerial_robot_control
{
namespace nmpc
{

/**
 * @brief Differential MPC controller with actuator velocity control, nullspace control and second-order actuator dynamics
 * The state is [p, v, q, w, alpha, alpha velocity, thrust, thrust velocity, (optional) internal wrench].
 * The input is [thrust velocity, alpha velocity].
 * The velocity commands are integrated to the thrust and servo angle (alpha) commands.
 */
class TiltMtServoThrustDiffSecondOrderMPC : public TiltMtServoThrustDiffMPC
{
protected:
  std::vector<double> prev_servo_angle_;
  std::vector<double> prev_servo_angle_vel_estimate_;
  std::vector<double> prev_thrust_meas_;
  std::vector<double> prev_thrust_vel_estimate_;

  /* Initialization methods */
  void initActuatorStates() override;
  void initMPCCostW() override;
  void initMPCConstraints() override;

  /* Reset methods */
  void resetSolver() override;

  /* Update methods */
  double getCommand(int idx_u, double T_horizon) override;
  void publishRecording() override;

  /* State methods */
  using TiltMtServoThrustDiffMPC::meas2VecX;
  std::vector<double> meas2VecX(bool is_ee_centric) override;

  /* Control methods */
  void allocateToXU(const tf::Vector3& ref_pos_w, const tf::Vector3& ref_vel_w, const tf::Quaternion& ref_quat_wt,
                    const tf::Vector3& ref_omega_t, const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                    std::vector<double>& u) override;

  /* Callback methods */
  void callbackMPCcfg(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_THRUST_DIFF_SECOND_ORDER_MPC_CONTROLLER_H
