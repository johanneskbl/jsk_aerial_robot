#include "aerial_robot_control/differential_mpc/tilt_mt_servo_thrust_diff_second_order_mpc_controller.h"

namespace aerial_robot_control
{
namespace nmpc
{
void TiltMtServoThrustDiffSecondOrderMPC::initActuatorStates()
{
  TiltMtServoThrustDiffMPC::initActuatorStates();

  /* Initialize joint velocity, thrust rate and command integrator */
  joint_vel_meas_.resize(joint_num_, 0.0);
  joint_vel_est_.resize(joint_num_, 0.0);
  thrust_rate_meas_.resize(motor_num_, 0.0);
  thrust_rate_est_.resize(motor_num_, 0.0);
  uo_prev_ = std::vector<double>(motor_num_ + joint_num_, 0.0);

  // Override starting indices for this controller's state-space
  servo_start_idx_ = 13;
  servo_vel_start_idx_ = 13 + joint_num_;
  thrust_start_idx_ = 13 + joint_num_ * 2;
  thrust_vel_start_idx_ = 13 + joint_num_ * 2 + motor_num_;
  wrench_state_start_idx_ = 13 + joint_num_ * 2 + motor_num_ * 2;
}

/**
 * @brief Reset the command integrator based on the measured actuator states
 */
void TiltMtServoThrustDiffSecondOrderMPC::resetSolver()
{
  for (int i = 0; i < motor_num_; i++)
    uo_prev_.at(i) = std::clamp(thrust_meas_[i], thrust_ctrl_min_, thrust_ctrl_max_);
  for (int i = 0; i < joint_num_; i++)
    uo_prev_.at(motor_num_ + i) = std::clamp(joint_angles_est_[i], servo_angle_min_, servo_angle_max_);

  TiltMtServoThrustDiffMPC::resetSolver();
}

void TiltMtServoThrustDiffSecondOrderMPC::initMPCCostW()
{
  /* Solver weights set with dynamic reconfigure */
  TiltMtServoThrustDiffMPC::initMPCCostW();

  // Actuator velocity state related cost weights
  double Qad, Qtd;
  getParam<double>(mpc_nh_, "Qad", Qad, 1.0);
  getParam<double>(mpc_nh_, "Qtd", Qtd, 1.0);
  for (int i = servo_vel_start_idx_; i < servo_vel_start_idx_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qad);
  for (int i = thrust_vel_start_idx_; i < thrust_vel_start_idx_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qtd);

  ROS_INFO("MPC cost W initialized:\n\tQad=%f, Qtd=%f", Qad, Qtd);
}

void TiltMtServoThrustDiffSecondOrderMPC::initMPCConstraints()
{
  double body_rate_max, body_rate_min;
  getParam<double>(mpc_nh_, "w_min", body_rate_min, -6.0);
  getParam<double>(mpc_nh_, "w_max", body_rate_max, 6.0);
  getParam<double>(mpc_nh_, "v_min", vel_min_, -1.0);
  getParam<double>(mpc_nh_, "v_max", vel_max_, 1.0);
  getParam<double>(mpc_nh_, "thrust_min", thrust_ctrl_min_, 0.0);
  getParam<double>(mpc_nh_, "thrust_max", thrust_ctrl_max_, 0.0);
  getParam<double>(mpc_nh_, "thrust_vel_min", thrust_vel_min_, -100.0);
  getParam<double>(mpc_nh_, "thrust_vel_max", thrust_vel_max_, 100.0);
  getParam<double>(mpc_nh_, "a_min", servo_angle_min_, -3.1416);
  getParam<double>(mpc_nh_, "a_max", servo_angle_max_, 3.1416);
  getParam<double>(mpc_nh_, "a_vel_min", servo_vel_min_, -4.0);
  getParam<double>(mpc_nh_, "a_vel_max", servo_vel_max_, 4.0);

  // TODO: this should be set in flight_navigation
  getParam<double>(mpc_nh_, "vel_limit_takeoff", vel_limit_takeoff_, 1.0);  // m/s

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

  ROS_INFO("MPC constraints are set: \n"
           "\tvel_min: %f, vel_max: %f, w_min: %f, w_max: %f,\n"
           "\tthrust_min: %f, thrust_max: %f, thrust_vel_min: %f, thrust_vel_max: %f,\n"
           "\tservo_angle_min: %f, servo_angle_max: %f, servo_angle_vel_min: %f, servo_angle_vel_max: %f",
           vel_min_, vel_max_, body_rate_min, body_rate_max,
           thrust_ctrl_min_, thrust_ctrl_max_, thrust_vel_min_, thrust_vel_max_,
           servo_angle_min_, servo_angle_max_, servo_vel_min_, servo_vel_max_);
}

/**
 * @brief Construct and allocate the state and control inputs from the referenced wrench
 * In addition to TiltMtServoThrustDiffMPC::allocateToXU(), set the actuator velocity state references.
 * @param ref_pos_w Position of the origin of T, expressed in W
 * @param ref_vel_w Linear velocity of the origin of T, expressed in W
 * @param ref_quat_wt Orientation of T w.r.t. W
 * @param ref_omega_t Angular velocity of T w.r.t. W, expressed in T
 * @param ref_wrench_b Reference wrench in the Body frame
 * @param x State vector to be filled
 * @param u Control vector to be filled
 */
void TiltMtServoThrustDiffSecondOrderMPC::allocateToXU(const tf::Vector3& ref_pos_w, const tf::Vector3& ref_vel_w,
                                                       const tf::Quaternion& ref_quat_wt,
                                                       const tf::Vector3& ref_omega_t,
                                                       const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                                                       std::vector<double>& u)
{
  TiltMtServoThrustDiffMPC::allocateToXU(ref_pos_w, ref_vel_w, ref_quat_wt, ref_omega_t, ref_wrench_b, x, u);

  // ############ Singularity point incentive ############
  // Fix singularity point rotation by penalizing pi/2 angle in servo velocity reference
  // Servo angle velocity reference

  if (include_nullspace_control_)
  {
    // When using nullspace control, we MUST set the reference to zero as we already have a P-term difference in the cost function!
    // DO !
    // NOT !
    // CHANGE !
    for (int i = 0; i < joint_num_; i++)
    {
      x.at(servo_vel_start_idx_ + i) = 0.0;
    }
    for (int i = 0; i < motor_num_; i++)
    {
      x.at(thrust_vel_start_idx_ + i) = 0.0;
    }
  }
  else
  {
    for (int i = 0; i < joint_num_; i++)
    {
      // Incentivize servo angle to go back to 0 rad (or pi rad) to avoid singularity point FOR LOW THRUSTS
      // x.at(servo_vel_start_idx_ + i) = - std::sin(joint_angles_est_[i] / 2.0) * std::cos((std::max(thrust_meas_[i], 0.0) + 1e-3) / thrust_ctrl_max_ * M_PI_2);  // Alternative
      x.at(servo_vel_start_idx_ + i) = 0.0;
    }

    // Thrust velocity
    for (int i = 0; i < motor_num_; i++)
    {
      x.at(thrust_vel_start_idx_ + i) = 0.0;
    }
  }
}

void TiltMtServoThrustDiffSecondOrderMPC::callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
  if (static_cast<int>(msg->position.size()) == joint_num_ && static_cast<int>(msg->velocity.size()) == joint_num_)
  {
    for (int i = 0; i < joint_num_; i++)
    {
      joint_angles_[i] = msg->position[i];
      joint_vel_meas_[i] = msg->velocity[i];
    }
    joint_states_time_ = ros::Time::now();  // Receipt time for delay compensation
  }
  else
  {
    ROS_WARN_THROTTLE(1, "Received joint states with size %zu and joint velocities size %zu, but expected %d and %d. "
                      "Ignoring this message.",
                      msg->position.size(), msg->velocity.size(), joint_num_, joint_num_);
  }
}

/**
 * @brief Update the measured thrust (clamped to >= 0) and its rate from the valid frames of the ESC telemetry
 */
void TiltMtServoThrustDiffSecondOrderMPC::callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg)
{
  esc_telem_.update(*msg, thrust_meas_, crc_errors_, &thrust_rate_meas_);
}

/**
 * @brief Estimate the servo angle AND velocity at the actuation time of the next command
 * NOTE: Overrides the angle-only estimation of TiltMtServoNMPC
 */
void TiltMtServoThrustDiffSecondOrderMPC::estimateJointStates()
{
  // The servo bias is only adapted while the commands are sent
  bool adapt_servo_bias = !is_warmup_ && !has_triggered_force_landing_;
  joint_estimator_.estimate(joint_states_time_, joint_angles_, gimbal_ctrl_cmd_.position, joint_angles_est_,
                            &joint_vel_meas_, &joint_vel_est_, adapt_servo_bias);
}

/**
 * @brief Estimate the thrust AND thrust rate at the actuation time of the next command
 * NOTE: Overrides the thrust-only estimation of TiltMtServoThrustDistNMPC
 */
void TiltMtServoThrustDiffSecondOrderMPC::estimateThrust()
{
  esc_telem_.checkAlive();  // Only logging
  thrust_estimator_.estimate(esc_telem_, thrust_meas_, flight_cmd_.base_thrust, thrust_est_, &thrust_rate_meas_,
                             &thrust_rate_est_);
}

std::vector<double> TiltMtServoThrustDiffSecondOrderMPC::meas2VecX(bool is_ee_centric)
{
  auto bx0 = TiltMtServoThrustDiffMPC::meas2VecX(is_ee_centric);

  /* Servo velocity */
  // Measured in spinal and estimated to read time to compensate its age with the servo model and the command history
  for (int i = 0; i < joint_num_; i++)
    bx0[servo_vel_start_idx_ + i] = joint_vel_est_[i];

  /* Thrust velocity estimate */
  // Measured on the ESCs and estimated to read time to compensate its age with the rate model and the command history
  for (int i = 0; i < motor_num_; i++)
    bx0[thrust_vel_start_idx_ + i] = thrust_rate_est_[i];

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
  //   double uo_ = x0 + (x1 - x0) * t_rotor_ / t_mpc_step_;
  // }
  // else
  // {
  //   // Interpolate one control period into the horizon WITH servo time constant
  //   double x0 = mpc_solver_ptr_->xo_.at(0).at(servo_start_idx_ + (idx_u - motor_num_));
  //   double x1 = mpc_solver_ptr_->xo_.at(1).at(servo_start_idx_ + (idx_u - motor_num_));
  //   double uo_ = x0 + (x1 - x0) * t_servo_ / t_mpc_step_;
  // }

  // ------- OR -------

  // // Integrate control input since it is defined as the servo angle and thrust velocity
  // double uo_derivative = mpc_solver_ptr_->uo_.at(0).at(idx_u);
  // // RESET?!?! to avoid blindly integrating?
  // double uo = uo_prev_.at(idx_u) + uo_derivative * t_mpc_dt_;

  // ------- OR -------

  // // Equivalent to direct integration if model is correct, else correct for drift
  // double uo_derivative = mpc_solver_ptr_->uo_.at(0).at(idx_u);
  // double uo;
  // if (idx_u < motor_num_) {
  //   // For thrust: use measured thrust as ground truth
  //   uo = thrust_meas_[idx_u] + uo_derivative * t_rotor_;
  // } else {
  //   // For servo: use measured angle as ground truth
  //   uo = joint_angles_est_[idx_u - motor_num_] + uo_derivative * t_servo_;
  // }

  // ------- OR -------

  // double uo_derivative = mpc_solver_ptr_->uo_.at(0).at(idx_u);
  // double tau  = (idx_u < motor_num_) ? t_rotor_ : t_servo_;
  // double meas = (idx_u < motor_num_) ? thrust_meas_[idx_u] : joint_angles_est_[idx_u - motor_num_];

  // // Blind integration realizes the planned rate exactly; the slow leak pins the command
  // // to the measurement-consistent lead (meas + tau*u_dot) so open-loop drift stays bounded.
  // const double T_leak = 0.4;  // s; must be >> tau - leaking at 1/tau halves the realized velocity
  // double uo = uo_prev_.at(idx_u) + uo_derivative * t_mpc_dt_;
  // uo += (t_mpc_dt_ / T_leak) * ((meas + tau * uo_derivative) - uo);

  // // -----------------

  // // Clamp to constraints
  // if (idx_u < motor_num_)
  // {
  //   uo = std::clamp(uo, thrust_ctrl_min_, thrust_ctrl_max_);
  // }
  // else
  // {
  //   uo = std::clamp(uo, servo_angle_min_, servo_angle_max_);
  // }

  // ------- OR -------

  // Integrated command with ZOH-exact hold:
  // The actuator velocity model
  //   tau * d(x_vel)/dt + x_vel = uo_vel
  // is the derivative of the first-order actuator model
  //   tau * dx/dt + x = x_cmd            (Note: dx/dt = x_vel and d(x_cmd)/dt = uo_vel)
  // Therefore, the MPC integrates the optimized velocity to the command, i.e., the command ramp s (uo_prev_)
  // advances by uo_vel * T_samp each control period.
  // The command is held for one control period T_samp, so the ramp itself is not realized. Instead, the held command
  //   uo_cmd = s + (lead - t) * uo_vel,
  //   lead = T_samp / (1 - exp(-T_samp / t))
  // moves the first-order actuator exactly as the model with the ramp over the period. This holds the ramp at about
  // the middle of the period, whereas holding its end s + uo_vel * T_samp leads the model by ~(T_samp / 2).
  // A slow leak pulls the ramp towards the estimated command x0 + tau * x_vel0 of the initial state of the MPC to avoid drift.
  
  // Get actuator position, velocity and command at the start of the horizon
  double t_actuator, x0, x_vel0, x_min, x_max;
  if (idx_u < motor_num_)
  {
    t_actuator = t_rotor_;
    x0 = mpc_solver_ptr_->xo_.at(0).at(thrust_start_idx_ + idx_u);
    x_vel0 = mpc_solver_ptr_->xo_.at(0).at(thrust_vel_start_idx_ + idx_u);
    x_min = thrust_ctrl_min_;
    x_max = thrust_ctrl_max_;
  }
  else
  {
    t_actuator = t_servo_;
    x0 = mpc_solver_ptr_->xo_.at(0).at(servo_start_idx_ + (idx_u - motor_num_));
    x_vel0 = mpc_solver_ptr_->xo_.at(0).at(servo_vel_start_idx_ + (idx_u - motor_num_));
    x_min = servo_angle_min_;
    x_max = servo_angle_max_;
  }
  const double uo_vel = mpc_solver_ptr_->uo_.at(0).at(idx_u);

  // Compute command that needs to be held to realize uo_vel over T_samp
  const double lead = t_mpc_samp_ / (1.0 - std::exp(-t_mpc_samp_ / t_actuator));
  const double uo_cmd = std::clamp(uo_prev_.at(idx_u) + (lead - t_actuator) * uo_vel, x_min, x_max);

  // Advance the command ramp to the next control period with the leak towards the estimated command
  const double T_leak = 0.4;  // in seconds; must be >> t_actuator, otherwise the leak reduces the realized velocity
  double s_next = uo_prev_.at(idx_u) + uo_vel * t_mpc_dt_;
  s_next += (t_mpc_dt_ / T_leak) * ((x0 + t_actuator * x_vel0) - s_next);
  if (!is_warmup_ && std::isfinite(s_next))  // Only integrate valid commands
    uo_prev_.at(idx_u) = std::clamp(s_next, x_min, x_max);

  return uo_cmd;
}

/**
 * @brief Populate the recording including the actuator velocity states
 */
void TiltMtServoThrustDiffSecondOrderMPC::populateRecordingMsgs(aerial_robot_msgs::MPCTrajectory& ref_msg,
                                                                aerial_robot_msgs::MPCTrajectory& pred_msg)
{
  TiltMtServoThrustDiffMPC::populateRecordingMsgs(ref_msg, pred_msg);

  /* Append servo angle velocity and thrust velocity states */
  for (int i = 0; i < static_cast<int>(ref_msg.states.size()); ++i)
  {
    const std::vector<double>& xr = mpc_solver_ptr_->xr_[i];
    const std::vector<double>& xo = mpc_solver_ptr_->xo_[i];
    ref_msg.states[i].servo_angle_velocity.assign(xr.begin() + servo_vel_start_idx_,
                                                  xr.begin() + servo_vel_start_idx_ + joint_num_);
    pred_msg.states[i].servo_angle_velocity.assign(xo.begin() + servo_vel_start_idx_,
                                                   xo.begin() + servo_vel_start_idx_ + joint_num_);
    ref_msg.states[i].thrust_velocity.assign(xr.begin() + thrust_vel_start_idx_,
                                             xr.begin() + thrust_vel_start_idx_ + motor_num_);
    pred_msg.states[i].thrust_velocity.assign(xo.begin() + thrust_vel_start_idx_,
                                              xo.begin() + thrust_vel_start_idx_ + motor_num_);
  }
}

void TiltMtServoThrustDiffSecondOrderMPC::callbackMPCcfg(aerial_robot_control::NMPCConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (!config.nmpc_flag)
    return;

  switch (level)
  {
    case Levels::RECONFIGURE_NMPC_Q_AD: {
      for (int i = servo_vel_start_idx_; i < servo_vel_start_idx_ + joint_num_; ++i)
        mpc_solver_ptr_->setCostWDiagElement(i, config.Qad);
      ROS_INFO_STREAM("change Qad for NMPC '" << config.Qad << "'");
      break;
    }
    case Levels::RECONFIGURE_NMPC_Q_TD: {
      for (int i = thrust_vel_start_idx_; i < thrust_vel_start_idx_ + motor_num_; ++i)
        mpc_solver_ptr_->setCostWDiagElement(i, config.Qtd);
      ROS_INFO_STREAM("change Qtd for NMPC '" << config.Qtd << "'");
      break;
    }
    default: {
      TiltMtServoThrustDiffMPC::callbackMPCcfg(config, level);
      break;
    }
  }
}
}
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustDiffSecondOrderMPC, aerial_robot_control::ControlBase);
