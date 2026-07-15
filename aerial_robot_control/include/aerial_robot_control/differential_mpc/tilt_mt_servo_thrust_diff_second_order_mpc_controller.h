#ifndef TILT_MT_SERVO_THRUST_DIFF_SECOND_ORDER_MPC_CONTROLLER_H
#define TILT_MT_SERVO_THRUST_DIFF_SECOND_ORDER_MPC_CONTROLLER_H

#include "aerial_robot_control/differential_mpc/tilt_mt_servo_thrust_diff_mpc_controller.h"

#include "spinal/ESCTelemetryArray.h"

namespace aerial_robot_control
{
namespace nmpc
{

class TiltMtServoThrustDiffSecondOrderMPC : public TiltMtServoThrustDiffMPC
{
protected:
  double thrust_vel_min_;
  double thrust_vel_max_;
  double servo_vel_min_;
  double servo_vel_max_;

  std::vector<double> prev_servo_angle_;
  std::vector<double> prev_servo_angle_vel_estimate_;
  std::vector<double> prev_thrust_meas_;
  std::vector<double> prev_thrust_vel_estimate_;

  void initActuatorStates() override;

  void initNMPCCostW() override;

  void initNMPCConstraints() override;

  std::vector<double> meas2VecX(bool is_modified_by_traj_frame) override;

  void allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i, const tf::Quaternion& ref_quat_ib,
                    const tf::Vector3& ref_omega_b, const VectorXd& ref_wrench_b, vector<double>& x,
                    vector<double>& u) override;

  double getCommand(int idx_u, double T_horizon) override;

  void publishRecording() override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_THRUST_DIFF_SECOND_ORDER_MPC_CONTROLLER_H
