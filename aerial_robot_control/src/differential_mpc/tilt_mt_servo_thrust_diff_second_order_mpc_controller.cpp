#include "aerial_robot_control/differential_mpc/tilt_mt_servo_thrust_diff_second_order_mpc_controller.h"

namespace aerial_robot_control
{
namespace nmpc
{
void TiltMtServoThrustDiffSecondOrderMPC::initActuatorStates()
{
  TiltMtServoThrustDiffMPC::initActuatorStates();

  // Override starting indices for this controller's state-space
  servo_start_idx_ = 13;
  servo_vel_start_idx_ = 13 + joint_num_;
  thrust_start_idx_ = 13 + joint_num_ * 2;
  thrust_vel_start_idx_ = 13 + joint_num_ * 2 + motor_num_;
  wrench_state_start_idx_ = 13 + joint_num_ * 2 + motor_num_ * 2;

  prev_servo_angle_.resize(joint_num_, 0.0);
  prev_servo_angle_vel_estimate_.resize(joint_num_, 0.0);
  prev_thrust_meas_.resize(motor_num_, 0.0);
  prev_thrust_vel_estimate_.resize(motor_num_, 0.0);
}

void TiltMtServoThrustDiffSecondOrderMPC::initNMPCCostW()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  /* control parameters with dynamic reconfigure */
  double Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Qad, Qt, Qtd, Qfu, Qtau, Rtd_c, Rad_c;
  getParam<double>(nmpc_nh, "Qp_xy", Qp_xy, 300);
  getParam<double>(nmpc_nh, "Qp_z", Qp_z, 400);
  getParam<double>(nmpc_nh, "Qv_xy", Qv_xy, 10);
  getParam<double>(nmpc_nh, "Qv_z", Qv_z, 10);
  getParam<double>(nmpc_nh, "Qq_xy", Qq_xy, 300);
  getParam<double>(nmpc_nh, "Qq_z", Qq_z, 300);
  getParam<double>(nmpc_nh, "Qw_xy", Qw_xy, 5);
  getParam<double>(nmpc_nh, "Qw_z", Qw_z, 5);
  getParam<double>(nmpc_nh, "Qa", Qa, 1);
  getParam<double>(nmpc_nh, "Qad", Qad, 1.0);
  getParam<double>(nmpc_nh, "Qt", Qt, 1);
  getParam<double>(nmpc_nh, "Qtd", Qtd, 1);
  getParam<double>(nmpc_nh, "Qfu", Qfu, 0);
  getParam<double>(nmpc_nh, "Qtau", Qtau, 0);

  getParam<double>(nmpc_nh, "Rtd_c", Rtd_c, 1);
  getParam<double>(nmpc_nh, "Rad_c", Rad_c, 250);

  // State cost
  mpc_solver_ptr_->setCostWDiagElement(0, Qp_xy);
  mpc_solver_ptr_->setCostWDiagElement(1, Qp_xy);
  mpc_solver_ptr_->setCostWDiagElement(2, Qp_z);
  mpc_solver_ptr_->setCostWDiagElement(3, Qv_xy);
  mpc_solver_ptr_->setCostWDiagElement(4, Qv_xy);
  mpc_solver_ptr_->setCostWDiagElement(5, Qv_z);
  mpc_solver_ptr_->setCostWDiagElement(6, 0);
  mpc_solver_ptr_->setCostWDiagElement(7, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(8, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(9, Qq_z);
  mpc_solver_ptr_->setCostWDiagElement(10, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(11, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(12, Qw_z);
  // Servo state
  for (int i = servo_start_idx_; i < servo_start_idx_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  // Servo velocity state
  for (int i = servo_vel_start_idx_; i < servo_vel_start_idx_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qad);
  // Thrust state
  for (int i = thrust_start_idx_; i < thrust_start_idx_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);
  // Thrust velocity cost
  for (int i = thrust_vel_start_idx_; i < thrust_vel_start_idx_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qtd);

  // Wrench state cost
  // NOTE: Include if `self.include_differential_allocation` is set to True in Python file
  mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 0, Qfu);
  mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 1, Qfu);
  mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 2, Qfu);
  mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 3, Qtau);
  mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 4, Qtau);
  mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 5, Qtau);

  // Control input cost
  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rtd_c, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rad_c, false);

  ROS_INFO("MPC cost W initialized:\n"
           "\tQp_xy=%f, Qp_z=%f, Qv_xy=%f, Qv_z=%f, Qq_xy=%f, Qq_z=%f, Qw_xy=%f, Qw_z=%f,\n"
           "\tQa=%f, Qad=%f, Qt=%f, Qtd=%f, Qfu=%f, Qtau=%f, Rtd_c=%f, Rad_c=%f",
           Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Qad, Qt, Qtd, Qfu, Qtau, Rtd_c, Rad_c);
}

void TiltMtServoThrustDiffSecondOrderMPC::initNMPCConstraints()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  double body_rate_max, body_rate_min;
  getParam<double>(nmpc_nh, "w_min", body_rate_min, -6.0);
  getParam<double>(nmpc_nh, "w_max", body_rate_max, 6.0);
  getParam<double>(nmpc_nh, "v_min", vel_min_, -1.0);
  getParam<double>(nmpc_nh, "v_max", vel_max_, 1.0);
  getParam<double>(nmpc_nh, "thrust_min", thrust_ctrl_min_, 0.0);
  getParam<double>(nmpc_nh, "thrust_max", thrust_ctrl_max_, 0.0);
  getParam<double>(nmpc_nh, "thrust_vel_min", thrust_vel_min_, -100.0);
  getParam<double>(nmpc_nh, "thrust_vel_max", thrust_vel_max_, 100.0);
  getParam<double>(nmpc_nh, "a_min", servo_angle_min_, -3.1416);
  getParam<double>(nmpc_nh, "a_max", servo_angle_max_, 3.1416);
  getParam<double>(nmpc_nh, "a_vel_min", servo_vel_min_, -4.0);
  getParam<double>(nmpc_nh, "a_vel_max", servo_vel_max_, 4.0);

  // TODO: this should be set in flight_navigation
  getParam<double>(control_nh, "vel_limit_takeoff", vel_limit_takeoff_, 1.0);  // m/s

  // lbx and ubx
  std::vector<int> idxbx = mpc_solver_ptr_->getConstraintsIdxbx();
  std::vector<int> idxbx_desired = { 3, 4, 5, 10, 11, 12 };
  idxbx_desired.resize(6 + joint_num_ * 2 + motor_num_ * 2);
  // Servo state
  for (int i = 0; i < joint_num_; i++)
  {
    idxbx_desired[6 + i] = 13 + i;
  }
  // Servo velocity state
  for (int i = 0; i < joint_num_; i++)
  {
    idxbx_desired[6 + joint_num_ + i] = 13 + joint_num_ + i;
  }
  // Thrust state
  for (int i = 0; i < motor_num_; i++)
  {
    idxbx_desired[6 + joint_num_ * 2 + i] = 13 + joint_num_ * 2 + i;
  }
  // Thrust velocity state
  for (int i = 0; i < motor_num_; i++)
  {
    idxbx_desired[6 + joint_num_ * 2 + motor_num_ + i] = 13 + joint_num_ * 2 + motor_num_ + i;
  }


  if (idxbx.size() != idxbx_desired.size() || !std::equal(idxbx.begin(), idxbx.end(), idxbx_desired.begin()))
  {
    ROS_ERROR("idxbx is not equal to idxbx_desired, we cannot set constraints lbx and ubx!");
  }

  std::vector<double> lbx = { vel_min_, vel_min_, vel_min_, body_rate_min, body_rate_min, body_rate_min };
  std::vector<double> ubx = { vel_max_, vel_max_, vel_max_, body_rate_max, body_rate_max, body_rate_max };
  lbx.resize(6 + joint_num_ * 2 + motor_num_ * 2);
  ubx.resize(6 + joint_num_ * 2 + motor_num_ * 2);
  // Servo state
  for (int i = 0; i < joint_num_; i++)
  {
    lbx[6 + i] = servo_angle_min_;
    ubx[6 + i] = servo_angle_max_;
  }
  // Servo velocity state
  for (int i = 0; i < joint_num_; i++)
  {
    lbx[6 + joint_num_ + i] = servo_vel_min_;
    ubx[6 + joint_num_ + i] = servo_vel_max_;
  }
  // Thrust state
  for (int i = 0; i < motor_num_; i++)
  {
    lbx[6 + joint_num_ * 2 + i] = thrust_ctrl_min_;
    ubx[6 + joint_num_ * 2 + i] = thrust_ctrl_max_;
  }
  // Thrust velocity state
  for (int i = 0; i < motor_num_; i++)
  {
    lbx[6 + joint_num_ * 2 + motor_num_ + i] = thrust_vel_min_;
    ubx[6 + joint_num_ * 2 + motor_num_ + i] = thrust_vel_max_;
  }
  mpc_solver_ptr_->setConstraintsLbx(lbx);
  mpc_solver_ptr_->setConstraintsUbx(ubx);

  // lbxe and ubxe
  std::vector<int> idxbxe = mpc_solver_ptr_->getConstraintsIdxbxe();
  std::vector<int> idxbxe_desired = idxbx_desired;
  if (idxbxe.size() != idxbxe_desired.size() || !std::equal(idxbxe.begin(), idxbxe.end(), idxbxe_desired.begin()))
  {
    ROS_ERROR("idxbx_end is not equal to idxbx_end_desired, we cannot set constraints lbxe and ubxe!");
  }
  mpc_solver_ptr_->setConstraintsLbxe(lbx);
  mpc_solver_ptr_->setConstraintsUbxe(ubx);

  // lbu and ubu
  std::vector<int> idxbu = mpc_solver_ptr_->getConstraintsIdxbu();
  std::vector<int> idxbu_desired(motor_num_ + joint_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    idxbu_desired[i] = i;
  }
  for (int i = 0; i < joint_num_; i++)
  {
    idxbu_desired[motor_num_ + i] = motor_num_ + i;
  }
  if (idxbu.size() != idxbu_desired.size() || !std::equal(idxbu.begin(), idxbu.end(), idxbu_desired.begin()))
  {
    ROS_ERROR("idxbu is not equal to idxbu_desired, we cannot set constraints lbu and ubu!");
  }

  std::vector<double> lbu(motor_num_ + joint_num_, 0.0);
  std::vector<double> ubu(motor_num_ + joint_num_, 0.0);
  for (int i = 0; i < motor_num_; i++)
  {
    lbu[i] = thrust_vel_min_;
    ubu[i] = thrust_vel_max_;
  }
  for (int i = 0; i < joint_num_; i++)
  {
    lbu[motor_num_ + i] = servo_vel_min_;
    ubu[motor_num_ + i] = servo_vel_max_;
  }
  mpc_solver_ptr_->setConstraintsLbu(lbu);
  mpc_solver_ptr_->setConstraintsUbu(ubu);

  ROS_INFO("MPC constraints are set: \n",
           "\tvel_min: %f, vel_max: %f, w_min: %f, w_max: %f,\n",
           "\tthrust_min: %f, thrust_max: %f, thrust_vel_min: %f, thrust_vel_max: %f,\n",
           "\tservo_angle_min: %f, servo_angle_max: %f, servo_angle_vel_min: %f, servo_angle_vel_max: %f",
           vel_min_, vel_max_, body_rate_min, body_rate_max,
           thrust_ctrl_min_, thrust_ctrl_max_, thrust_vel_min_, thrust_vel_max_,
           servo_angle_min_, servo_angle_max_, servo_vel_min_, servo_vel_max_);
}

void TiltMtServoThrustDiffSecondOrderMPC::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                                       const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                                                       const Eigen::VectorXd& ref_wrench_b, vector<double>& x,
                                                       vector<double>& u)
{
  TiltMtServoThrustDiffMPC::allocateToXU(ref_pos_i, ref_vel_i, ref_quat_ib, ref_omega_b, ref_wrench_b, x, u);

  // ############ Singularity point incentive ############
  // Fix singularity point rotation by penalizing pi/2 angle in servo velocity reference
  // Servo angle velocity reference
  for (int i = 0; i < joint_num_; i++)
  {
    // x.at(servo_vel_start_idx_ + i) = - 0.5*std::sin(joint_angles_[i]);  // Alternative
    x.at(servo_vel_start_idx_ + i) = 0.0;
  }

  // Thrust velocity
  for (int i = 0; i < motor_num_; i++)
  {
    x.at(thrust_vel_start_idx_ + i) = 0.0;
  }
}

std::vector<double> TiltMtServoThrustDiffSecondOrderMPC::meas2VecX(bool is_ee_centric)
{
  auto bx0 = TiltMtServoThrustDiffMPC::meas2VecX(is_ee_centric);

  if (!is_warmup_)
  {
    // Servo velocity estimate
    for (int i = 0; i < joint_num_; i++)
    {
      // ==== MODEL ====
      double last_servo_cmd = uo_prev_.at(motor_num_ + i);
      // TODO is this really correctly computing the velocity???
      double servo_angle_velocity = (last_servo_cmd - joint_angles_[i]) / t_servo_;

      // ==== NUMERICAL DERIVATIVE ====
      // double servo_angle_velocity = (joint_angles_[i] - prev_servo_angles_[i]) / du_;
      // prev_servo_angles_[i] = servo_angles_[i];

      // ==== FILTERED NUMERICAL DERIVATIVE ====
      // double weight = 0.9;
      // auto servo_angle_velocity = weight * prev_servo_angle_vel_estimate_.at(i) + (1-weight) * (joint_angles_[i] - prev_servo_angle_.at(i)) * ctrl_loop_du_;
      // prev_servo_angle_.at(i) = joint_angles_[i];
      // prev_servo_angle_vel_estimate_.at(i) = servo_angle_velocity;

      bx0[13 + joint_num_ + i] = servo_angle_velocity;
    }

    // Thrust velocity estimate
    for (int i = 0; i < motor_num_; i++)
    {
      // ==== MODEL ====
      double last_thrust_c = uo_prev_.at(i);
      // double thrust_velocity = (last_thrust_c - thrust_meas_[i] + 0.2) / t_rotor_;  // 0.2 correction factor for model mismatch
      double thrust_velocity = (last_thrust_c - thrust_meas_[i]) / t_rotor_;

      // ==== NUMERICAL DERIVATIVE ====
      // double thrust_velocity = (thrust_meas_[i] - prev_thrust_meas_[i]) / du_;
      // prev_thrust_meas_[i] = thrust_meas_[i];

      // ==== FILTERED NUMERICAL DERIVATIVE ====
      // auto thrust_velocity = 0.7 * prev_thrust_vel_estimate_.at(i) + 0.3 * (thrust_meas_[i] - prev_thrust_meas_[i]) / ctrl_loop_du_;
      // prev_thrust_meas_.at(i) = thrust_meas_[i];
      // prev_thrust_vel_estimate_.at(i) = thrust_velocity;

      bx0[13 + joint_num_ * 2 + motor_num_ + i] = thrust_velocity;
    }
  }
  else
  {
    for (int i = 0; i < joint_num_; i++)
      bx0[13 + joint_num_ + i] = 0.0;
    for (int i = 0; i < motor_num_; i++)
      bx0[13 + joint_num_ * 2 + motor_num_ + i] = 0.0;
  }

  return bx0;
}

double TiltMtServoThrustDiffSecondOrderMPC::getCommand(int idx_u, double T_horizon)
{
  // ENTIRELY DIFFERENT IDEA:
  // Instead of using control input here, get the predicted STATE of thrust and servo inside the MODEL!
  // This makes sense since in SQP-RTI the state is also an optimization variable
  // Interpolate to next sample time based on current and next state in the horizon
  // if (idx_u < motor_num_)
  // {
  //   // Interpolate one control period into the horizon WITH rotor time constant
  //   double x0 = mpc_solver_ptr_->xo_.at(0).at(thrust_start_idx_ + idx_u);
  //   double x1 = mpc_solver_ptr_->xo_.at(1).at(thrust_start_idx_ + idx_u);
  //   double uo_ = x0 + (x1 - x0) * t_rotor_ / t_nmpc_step_;
  // }
  // else
  // {
  //   // Interpolate one control period into the horizon WITH servo time constant
  //   double x0 = mpc_solver_ptr_->xo_.at(0).at(servo_start_idx_ + (idx_u - motor_num_));
  //   double x1 = mpc_solver_ptr_->xo_.at(1).at(servo_start_idx_ + (idx_u - motor_num_));
  //   double uo_ = x0 + (x1 - x0) * t_servo_ / t_nmpc_step_;
  // }

  // ------- OR -------

  // // Integrate control input since it is defined as the servo angle and thrust velocity
  // double uo_derivative = mpc_solver_ptr_->uo_.at(0).at(idx_u);
  // // RESET?!?! to avoid blindly integrating?
  // double uo = uo_prev_.at(idx_u) + uo_derivative * t_nmpc_dt_;

  // ------- OR -------

  // // Equivalent to direct integration if model is correct, else correct for drift
  // double uo_derivative = mpc_solver_ptr_->uo_.at(0).at(idx_u);
  // double uo;
  // if (idx_u < motor_num_) {
  //   // For thrust: use measured thrust as ground truth
  //   uo = thrust_meas_[idx_u] + uo_derivative * t_rotor_;
  // } else {
  //   // For servo: use measured angle as ground truth
  //   uo = joint_angles_[idx_u - motor_num_] + uo_derivative * t_servo_;
  // }

  // ------- OR -------

  double uo_derivative = mpc_solver_ptr_->uo_.at(0).at(idx_u);
  double tau  = (idx_u < motor_num_) ? t_rotor_ : t_servo_;
  double meas = (idx_u < motor_num_) ? thrust_meas_[idx_u] : joint_angles_[idx_u - motor_num_];

  // Blind integration realizes the planned rate exactly; the slow leak pins the command
  // to the measurement-consistent lead (meas + tau*u_dot) so open-loop drift stays bounded.
  const double T_leak = 0.4;  // s; must be >> tau - leaking at 1/tau halves the realized velocity
  double uo = uo_prev_.at(idx_u) + uo_derivative * t_nmpc_dt_;
  uo += (t_nmpc_dt_ / T_leak) * ((meas + tau * uo_derivative) - uo);

  // -----------------

  // Clamp to constraints
  if (idx_u < motor_num_)
  {
    uo = std::clamp(uo, thrust_ctrl_min_, thrust_ctrl_max_);
  }
  else
  {
    uo = std::clamp(uo, servo_angle_min_, servo_angle_max_);
  }

  if (!is_warmup_)
  {
    uo_prev_.at(idx_u) = uo;
  }
  return uo;
}

void TiltMtServoThrustDiffSecondOrderMPC::cfgNMPCCallback(aerial_robot_control::NMPCConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (config.nmpc_flag)
  {
    try
    {
      switch (level)
      {
        case Levels::RECONFIGURE_NMPC_Q_P_XY: {
          mpc_solver_ptr_->setCostWDiagElement(0, config.Qp_xy);
          mpc_solver_ptr_->setCostWDiagElement(1, config.Qp_xy);

          ROS_INFO_STREAM("change Qp_xy for NMPC '" << config.Qp_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_P_Z: {
          mpc_solver_ptr_->setCostWDiagElement(2, config.Qp_z);
          ROS_INFO_STREAM("change Qp_z for NMPC '" << config.Qp_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_V_XY: {
          mpc_solver_ptr_->setCostWDiagElement(3, config.Qv_xy);
          mpc_solver_ptr_->setCostWDiagElement(4, config.Qv_xy);
          ROS_INFO_STREAM("change Qv_xy for NMPC '" << config.Qv_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_V_Z: {
          mpc_solver_ptr_->setCostWDiagElement(5, config.Qv_z);
          ROS_INFO_STREAM("change Qv_z for NMPC '" << config.Qv_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_Q_XY: {
          mpc_solver_ptr_->setCostWDiagElement(7, config.Qq_xy);
          mpc_solver_ptr_->setCostWDiagElement(8, config.Qq_xy);
          ROS_INFO_STREAM("change Qq_xy for NMPC '" << config.Qq_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_Q_Z: {
          mpc_solver_ptr_->setCostWDiagElement(9, config.Qq_z);
          ROS_INFO_STREAM("change Qq_z for NMPC '" << config.Qq_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_W_XY: {
          mpc_solver_ptr_->setCostWDiagElement(10, config.Qw_xy);
          mpc_solver_ptr_->setCostWDiagElement(11, config.Qw_xy);
          ROS_INFO_STREAM("change Qw_xy for NMPC '" << config.Qw_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_W_Z: {
          mpc_solver_ptr_->setCostWDiagElement(12, config.Qw_z);
          ROS_INFO_STREAM("change Qw_z for NMPC '" << config.Qw_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_A: {
          for (int i = servo_start_idx_; i < servo_start_idx_ + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qa);
          ROS_INFO_STREAM("change Qa for NMPC '" << config.Qa << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_AD: {
          for (int i = servo_vel_start_idx_; i < servo_vel_start_idx_ + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qad);
          ROS_INFO_STREAM("change Qad for NMPC '" << config.Qad << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_T: {
          for (int i = thrust_start_idx_; i < thrust_start_idx_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qt);
          ROS_INFO_STREAM("change Qt for NMPC '" << config.Qt << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_TD: {
          for (int i = thrust_vel_start_idx_; i < thrust_vel_start_idx_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qtd);
          ROS_INFO_STREAM("change Qtd for NMPC '" << config.Qtd << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_FU: {
          mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 0, config.Qfu);
          mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 1, config.Qfu);
          mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 2, config.Qfu);
          ROS_INFO_STREAM("change Qfu for NMPC '" << config.Qfu << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_TAU: {
          mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 3, config.Qtau);
          mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 4, config.Qtau);
          mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 5, config.Qtau);
          ROS_INFO_STREAM("change Qtau for NMPC '" << config.Qtau << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_TC_D: {
          for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rtc_d, false);
          ROS_INFO_STREAM("change Rtc_d for NMPC '" << config.Rtc_d << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_AD_C: {
          for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rad_c, false);
          ROS_INFO_STREAM("change Rad_c for NMPC '" << config.Rad_c << "'");
          break;
        }
        default: {
          ROS_INFO_STREAM("The setting variable is not in the list!");
          break;
        }
      }
    }
    catch (std::invalid_argument& e)
    {
      ROS_ERROR_STREAM("NMPC config failed: " << e.what());
    }
  }
}
}
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustDiffSecondOrderMPC, aerial_robot_control::ControlBase);
