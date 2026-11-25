#include "aerial_robot_control/differential_mpc/tilt_mt_servo_thrust_diff_mpc_controller.h"

namespace aerial_robot_control
{
namespace nmpc
{
void TiltMtServoThrustDiffMPC::initActuatorStates()
{
  TiltMtServoThrustDistNMPC::initActuatorStates();

  servo_start_idx_ = 13;
  thrust_start_idx_ = 13 + joint_num_;
  wrench_state_start_idx_ = 13 + joint_num_ + motor_num_;

  internal_wrench_b_ = Eigen::VectorXd::Zero(6);
}

void TiltMtServoThrustDiffMPC::initMPCCostW()
{
  /* Solver weights set with dynamic reconfigure */
  TiltMtServoNMPC::initMPCCostW();

  // Thrust, wrench and actuator velocity related cost weights
  double Qt, Qfu, Qtau, Rtd_c, Rad_c;
  getParam<double>(mpc_nh_, "Qt", Qt, 1);
  getParam<double>(mpc_nh_, "Qfu", Qfu, 0);
  getParam<double>(mpc_nh_, "Qtau", Qtau, 0);

  getParam<double>(mpc_nh_, "Rtd_c", Rtd_c, 1);
  getParam<double>(mpc_nh_, "Rad_c", Rad_c, 250);

  for (int i = thrust_start_idx_; i < thrust_start_idx_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);

  // Wrench state cost
  // NOTE: Include if `self.include_differential_allocation` is set to True in Python file
  if (hasWrenchState())
  {
    mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 0, Qfu);
    mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 1, Qfu);
    mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 2, Qfu);
    mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 3, Qtau);
    mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 4, Qtau);
    mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 5, Qtau);
  }

  // Control input cost
  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rtd_c, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rad_c, false);

  // Nullspace control
  double nullspace_servo_gain, nullspace_thrust_gain;
  getParam<double>(mpc_nh_, "nullspace_servo_gain", nullspace_servo_gain, 0.0);
  getParam<double>(mpc_nh_, "nullspace_thrust_gain", nullspace_thrust_gain, 0.0);
  // NOTE: The nullspace control of the model requires the differential allocation, i.e., the wrench state
  if ((nullspace_servo_gain == 0.0 && nullspace_thrust_gain == 0.0) || !hasWrenchState())
  {
    include_nullspace_control_ = false;
  }
  else
  {
    include_nullspace_control_ = true;

    // For nullspace control, the reference for the servo and thrust state should NOT be tracked
    double Qa;
    getParam<double>(mpc_nh_, "Qa", Qa, 1.0);
    if (Qa != 0.0 || Qt != 0.0)
    {
      ROS_FATAL("Nullspace control is enabled, but Qa or Qt is not zero. Since the reference must be set to zero this is badly conditioned.");
    }
  }

  ROS_INFO("MPC cost W initialized:\n"
           "\tQt=%f, Qfu=%f, Qtau=%f, Rtd_c=%f, Rad_c=%f,\n"
           "\tnullspace_servo_gain=%f, nullspace_thrust_gain=%f",
           Qt, Qfu, Qtau, Rtd_c, Rad_c, nullspace_servo_gain, nullspace_thrust_gain);
}

void TiltMtServoThrustDiffMPC::initMPCConstraints()
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

  //  TODO: this should be set in flight_navigation; don't know why set 0.2 results solver failure
  getParam<double>(mpc_nh_, "vel_limit_takeoff", vel_limit_takeoff_, 1.0);  // m/s

  // lbx and ubx
  std::vector<int> idxbx = mpc_solver_ptr_->getConstraintsIdxbx();
  std::vector<int> idxbx_desired = { 3, 4, 5, 10, 11, 12 };
  idxbx_desired.resize(6 + joint_num_ + motor_num_);
  for (int i = 0; i < joint_num_; i++)
  {
    idxbx_desired[6 + i] = 13 + i;
  }
  for (int i = 0; i < motor_num_; i++)
  {
    idxbx_desired[6 + joint_num_ + i] = 13 + joint_num_ + i;
  }

  if (idxbx.size() != idxbx_desired.size() || !std::equal(idxbx.begin(), idxbx.end(), idxbx_desired.begin()))
  {
    ROS_ERROR("idxbx is not equal to idxbx_desired, we cannot set constraints lbx and ubx!");
  }

  std::vector<double> lbx = { vel_min_, vel_min_, vel_min_, body_rate_min, body_rate_min, body_rate_min };
  std::vector<double> ubx = { vel_max_, vel_max_, vel_max_, body_rate_max, body_rate_max, body_rate_max };
  lbx.resize(6 + joint_num_ + motor_num_);
  ubx.resize(6 + joint_num_ + motor_num_);
  for (int i = 0; i < joint_num_; i++)
  {
    lbx[6 + i] = servo_angle_min_;
    ubx[6 + i] = servo_angle_max_;
  }
  for (int i = 0; i < motor_num_; i++)
  {
    lbx[6 + joint_num_ + i] = thrust_ctrl_min_;
    ubx[6 + joint_num_ + i] = thrust_ctrl_max_;
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
 * @brief Reset the solver iterate with the current state and zero actuator velocities as input guess
 * The inputs are the thrust and servo angle velocities, so the hover guess of TiltMtServoNMPC does not apply.
 */
void TiltMtServoThrustDiffMPC::resetSolver()
{
  std::vector<double> x_vec = meas2VecX(false);            // Always in Body frame
  std::vector<double> u_guess(mpc_solver_ptr_->NU_, 0.0);  // Zero velocities
  mpc_solver_ptr_->resetSolverByX0U0(x_vec, u_guess);
}

/**
 * @brief Construct and allocate the state and control inputs from the referenced wrench
 * The servo angles and thrusts are states, so their references are set in x and the control input reference
 * (actuator velocities) is zero.
 * @param ref_pos_w Position of the origin of T, expressed in W
 * @param ref_vel_w Linear velocity of the origin of T, expressed in W
 * @param ref_quat_wt Orientation of T w.r.t. W
 * @param ref_omega_t Angular velocity of T w.r.t. W, expressed in T
 * @param ref_wrench_b Reference wrench in the Body frame
 * @param x State vector to be filled
 * @param u Control vector to be filled
 */
void TiltMtServoThrustDiffMPC::allocateToXU(const tf::Vector3& ref_pos_w, const tf::Vector3& ref_vel_w,
                                            const tf::Quaternion& ref_quat_wt, const tf::Vector3& ref_omega_t,
                                            const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                                            std::vector<double>& u)
{
  TiltMtServoNMPC::allocateToXU(ref_pos_w, ref_vel_w, ref_quat_wt, ref_omega_t, ref_wrench_b, x, u);

  // Map allocated thrust to the thrust state
  for (int i = 0; i < motor_num_; i++)
    x.at(thrust_start_idx_ + i) = u.at(i);

  // Internal wrench state reference
  if (hasWrenchState())
  {
    x.at(wrench_state_start_idx_ + 0) = ref_wrench_b(0);
    x.at(wrench_state_start_idx_ + 1) = ref_wrench_b(1);
    x.at(wrench_state_start_idx_ + 2) = ref_wrench_b(2);
    x.at(wrench_state_start_idx_ + 3) = ref_wrench_b(3);
    x.at(wrench_state_start_idx_ + 4) = ref_wrench_b(4);
    x.at(wrench_state_start_idx_ + 5) = ref_wrench_b(5);
  }

  // Zero-out control input since control input is thrust and servo derivative
  for (int i = 0; i < motor_num_ + joint_num_; i++)
    u.at(i) = 0.0;
}

std::vector<double> TiltMtServoThrustDiffMPC::meas2VecX(bool is_ee_centric)
{
  auto bx0 = TiltMtServoNMPC::meas2VecX(is_ee_centric);

  estimateThrust();
  for (int i = 0; i < motor_num_; i++)
    bx0[thrust_start_idx_ + i] = thrust_est_[i];  // A stale ESC converges to its last thrust command

  // Internal wrench as state
  if (hasWrenchState())
  {
    computeInternalWrenchB();
    bx0[wrench_state_start_idx_ + 0] = internal_wrench_b_(0);
    bx0[wrench_state_start_idx_ + 1] = internal_wrench_b_(1);
    bx0[wrench_state_start_idx_ + 2] = internal_wrench_b_(2);
    bx0[wrench_state_start_idx_ + 3] = internal_wrench_b_(3);
    bx0[wrench_state_start_idx_ + 4] = internal_wrench_b_(4);
    bx0[wrench_state_start_idx_ + 5] = internal_wrench_b_(5);
  }

  return bx0;
}

void TiltMtServoThrustDiffMPC::computeInternalWrenchB()
{
  if (alloc_mat_.size() == 0 || static_cast<int>(thrust_est_.size()) != motor_num_)
  {
    ROS_WARN("[NMPC] Allocation matrix is not set or thrust measurement size is not equal to motor number, we cannot compute internal wrench, set it to zero!");
    internal_wrench_b_ = Eigen::VectorXd::Zero(6);
    return;
  }

  // Compute current force and torque from current servo angles and forces
  Eigen::VectorXd allocation_matrix_input = Eigen::VectorXd::Zero(2 * motor_num_);
  for (int i = 0; i < motor_num_; ++i)
  {
    allocation_matrix_input(2 * i) = thrust_est_[i] * std::sin(joint_angles_est_[i]);
    allocation_matrix_input(2 * i + 1) = thrust_est_[i] * std::cos(joint_angles_est_[i]);
  }

  // Compute current body wrench from current force and torque
  internal_wrench_b_ = alloc_mat_ * allocation_matrix_input;
}

/**
 * @brief Main control loop with a regularized solver during warm-up
 * During warm-up (ARM_ON) the commands are not sent, so the solver iterates on the same problem every control period
 * (robot on the ground, takeoff reference). On it, the full Gauss-Newton steps of SQP-RTI alternate between two
 * iterates, e.g., in the servo plan. The Levenberg-Marquardt term damps the steps such that the iterate converges and
 * is a good warm start for the takeoff. After warm-up it is zero, i.e., the full steps are used.
 */
void TiltMtServoThrustDiffMPC::controlCore()
{
  if (is_warmup_)
  {
    const double levenberg_marquardt_warmup = 10.0;  // converges for 3 - 100 in simulation
    mpc_solver_ptr_->setLevenbergMarquardt(levenberg_marquardt_warmup);
  }
  else
  {
    mpc_solver_ptr_->setLevenbergMarquardt(0.0);
  }

  TiltMtServoThrustDistNMPC::controlCore();
}

double TiltMtServoThrustDiffMPC::getCommand(int idx_u, double T_horizon)
{
  // ZOH-exact lead:
  // The MPC plans the actuator state x(t) = x0 + uo_vel * t from the estimated state x0 at the actuation
  // time. Therefore, the state follows exactly
  //   x1 - x0 = T_step * uo_vel, and
  //   x(T_samp) - x0 = T_samp * uo_vel,
  // since the actuator states are pure integrators of uo_vel. The command uo_cmd is held for
  // one control period T_samp, in which the first-order actuator
  //   tau * dx/dt + x = uo_cmd
  // moves by
  //   x(T_samp) - x0 = (uo_cmd - x0) * (1 - exp(-T_samp / tau)).
  // So the command
  //   uo_cmd = x0 + uo_vel * T_samp / (1 - exp(-T_samp / tau))
  // realizes exactly the planned increment of uo_vel * T_samp.
  // The lead satisfies the following approximation for small T_samp / tau:
  //    T_samp / (1 - exp(-T_samp / tau)) ~ tau + T_samp / 2,
  // whereas the continuous-time inversion x0 + tau * uo_vel neglects the hold and realizes only 73 %
  // of the planned increment (with tau = 15 ms, T_samp = 10 ms).
  if (idx_u < motor_num_)
  {
    const double x0 = mpc_solver_ptr_->xo_.at(0).at(thrust_start_idx_ + idx_u);
    const double uo_vel = mpc_solver_ptr_->uo_.at(0).at(idx_u);

    // Lead factor for ZOH-exact inversion of first-order actuator dynamics
    const double lead = t_mpc_samp_ / (1.0 - std::exp(-t_mpc_samp_ / t_rotor_));

    // Compute the command based on the estimated initial state and the lead factor
    const double uo_cmd = x0 + lead * uo_vel;

    // The command is hard constrained here to stay in actuator limits.
    return std::clamp(uo_cmd, thrust_ctrl_min_, thrust_ctrl_max_);
  }
  else
  {
    const double x0 = mpc_solver_ptr_->xo_.at(0).at(servo_start_idx_ + (idx_u - motor_num_));
    const double uo_vel = mpc_solver_ptr_->uo_.at(0).at(idx_u);

    // Lead factor for ZOH-exact inversion of first-order actuator dynamics
    const double lead = t_mpc_samp_ / (1.0 - std::exp(-t_mpc_samp_ / t_servo_));

    // Compute the command based on the estimated initial state and the lead factor
    const double uo_cmd = x0 + lead * uo_vel;

    // The command is hard constrained here to stay in actuator limits.
    return std::clamp(uo_cmd, servo_angle_min_, servo_angle_max_);
  }

  // ENTIRELY DIFFERENT IDEA:
  // Instead of using control input here, get the predicted STATE of thrust and servo inside the MODEL!
  // This makes sense since in SQP-RTI the state is also an optimization variable
  // Interpolate to next sample time based on current and next state in the horizon

  // ------- OR -------

  // // Integrate control input since it is defined as the servo angle and thrust velocity
  // double uo_derivative = mpc_solver_ptr_->uo_.at(0).at(idx_u);
  // // RESET?!?! to avoid blindly integrating?
  // double uo = uo_prev_.at(idx_u) + uo_derivative / ctrl_loop_du_;

  // ------- OR -------

  // double uo_derivative = mpc_solver_ptr_->uo_.at(0).at(idx_u);
  // double uo;
  // if (idx_u < motor_num_) {
  //   // For thrust: use measured thrust as ground truth
  //   uo = thrust_meas_[idx_u] + uo_derivative / ctrl_loop_du_;
  // } else {
  //   // For servo: use measured angle as ground truth
  //   uo = joint_angles_est_[idx_u - motor_num_] + uo_derivative / ctrl_loop_du_;
  // }

  // ------- OR -------

  // // Integrate control input since it is defined as the servo angle and thrust velocity
  // double uo_derivative = mpc_solver_ptr_->uo_.at(0).at(idx_u);
  // // RESET?!?! to avoid blindly integrating?
  // double uo = uo_prev_.at(idx_u) + uo_derivative / ctrl_loop_du_;

  // double measurement;
  // double feedback_gain = 0.1;

  // if (idx_u < motor_num_) {
  //   measurement = thrust_meas_[idx_u];
  // } else {
  //   measurement = joint_angles_est_[idx_u - motor_num_];
  // }
  // // Blend: favor MPC integration but correct for divergence
  // uo += feedback_gain * (measurement - uo_prev_[idx_u]);

  // -----------------

  // // Clamp to constraints
  // if (idx_u < motor_num_)
  // {
  //   uo = std::clamp(uo, thrust_ctrl_min_, thrust_ctrl_max_);
  // }
  // else
  // {
  //   uo = std::clamp(uo, servo_angle_min_, servo_angle_max_);
  // }

  // if (!is_warmup_)
  // {
  //   uo_prev_.at(idx_u) = uo;
  // }
  // return uo;
}

/**
 * @brief Publish the recording of the populated trajectories
 * NOTE: Skip TiltMtServoThrustDistNMPC::publishRecording() since the thrust states are appended in
 * populateRecordingMsgs() and the model has no external wrench states
 */
void TiltMtServoThrustDiffMPC::publishRecording()
{
  TiltMtServoNMPC::publishRecording();
}

/**
 * @brief Populate the recording including the thrust (and internal wrench) states
 * The predicted controls are the optimized actuator velocities and the commands computed in getCommand().
 */
void TiltMtServoThrustDiffMPC::populateRecordingMsgs(aerial_robot_msgs::MPCTrajectory& ref_msg,
                                                     aerial_robot_msgs::MPCTrajectory& pred_msg)
{
  TiltMtServoNMPC::populateRecordingMsgs(ref_msg, pred_msg);

  /* Append thrust and internal wrench states */
  for (int i = 0; i < static_cast<int>(ref_msg.states.size()); ++i)
  {
    const std::vector<double>& xr = mpc_solver_ptr_->xr_[i];
    const std::vector<double>& xo = mpc_solver_ptr_->xo_[i];
    ref_msg.states[i].thrust.assign(xr.begin() + thrust_start_idx_, xr.begin() + thrust_start_idx_ + motor_num_);
    pred_msg.states[i].thrust.assign(xo.begin() + thrust_start_idx_, xo.begin() + thrust_start_idx_ + motor_num_);

    if (hasWrenchState())
    {
      ref_msg.states[i].wrench_forces.x = xr[wrench_state_start_idx_ + 0];
      ref_msg.states[i].wrench_forces.y = xr[wrench_state_start_idx_ + 1];
      ref_msg.states[i].wrench_forces.z = xr[wrench_state_start_idx_ + 2];
      ref_msg.states[i].wrench_torques.x = xr[wrench_state_start_idx_ + 3];
      ref_msg.states[i].wrench_torques.y = xr[wrench_state_start_idx_ + 4];
      ref_msg.states[i].wrench_torques.z = xr[wrench_state_start_idx_ + 5];
      pred_msg.states[i].wrench_forces.x = xo[wrench_state_start_idx_ + 0];
      pred_msg.states[i].wrench_forces.y = xo[wrench_state_start_idx_ + 1];
      pred_msg.states[i].wrench_forces.z = xo[wrench_state_start_idx_ + 2];
      pred_msg.states[i].wrench_torques.x = xo[wrench_state_start_idx_ + 3];
      pred_msg.states[i].wrench_torques.y = xo[wrench_state_start_idx_ + 4];
      pred_msg.states[i].wrench_torques.z = xo[wrench_state_start_idx_ + 5];
    }
  }

  /* Predicted controls: actuator velocities and the sent commands */
  for (int i = 0; i < static_cast<int>(pred_msg.controls.size()); ++i)
  {
    const std::vector<double>& uo = mpc_solver_ptr_->uo_[i];
    pred_msg.controls[i].thrust_velocity_uo.assign(uo.begin(), uo.begin() + motor_num_);
    pred_msg.controls[i].servo_angle_velocity_uo.assign(uo.begin() + motor_num_, uo.begin() + motor_num_ + joint_num_);
    pred_msg.controls[i].thrust_cmd.assign(flight_cmd_.base_thrust.begin(), flight_cmd_.base_thrust.end());
    pred_msg.controls[i].servo_angle_cmd = gimbal_ctrl_cmd_.position;
  }
}

void TiltMtServoThrustDiffMPC::callbackMPCcfg(aerial_robot_control::NMPCConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (!config.nmpc_flag)
    return;

  switch (level)
  {
    case Levels::RECONFIGURE_NMPC_Q_T: {
      for (int i = thrust_start_idx_; i < thrust_start_idx_ + motor_num_; ++i)
        mpc_solver_ptr_->setCostWDiagElement(i, config.Qt);
      ROS_INFO_STREAM("change Qt for NMPC '" << config.Qt << "'");
      break;
    }
    case Levels::RECONFIGURE_NMPC_Q_FU: {
      if (!hasWrenchState())
      {
        ROS_WARN_STREAM("The MPC model has no wrench state, Qfu is not changed!");
        break;
      }
      mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 0, config.Qfu);
      mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 1, config.Qfu);
      mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 2, config.Qfu);
      ROS_INFO_STREAM("change Qfu for NMPC '" << config.Qfu << "'");
      break;
    }
    case Levels::RECONFIGURE_NMPC_Q_TAU: {
      if (!hasWrenchState())
      {
        ROS_WARN_STREAM("The MPC model has no wrench state, Qtau is not changed!");
        break;
      }
      mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 3, config.Qtau);
      mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 4, config.Qtau);
      mpc_solver_ptr_->setCostWDiagElement(wrench_state_start_idx_ + 5, config.Qtau);
      ROS_INFO_STREAM("change Qtau for NMPC '" << config.Qtau << "'");
      break;
    }
    case Levels::RECONFIGURE_NMPC_R_TD_C: {
      for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
        mpc_solver_ptr_->setCostWDiagElement(i, config.Rtd_c, false);
      ROS_INFO_STREAM("change Rtd_c for NMPC '" << config.Rtd_c << "'");
      break;
    }
    case Levels::RECONFIGURE_NMPC_R_AD_C: {
      for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
        mpc_solver_ptr_->setCostWDiagElement(i, config.Rad_c, false);
      ROS_INFO_STREAM("change Rad_c for NMPC '" << config.Rad_c << "'");
      break;
    }
    case Levels::RECONFIGURE_NMPC_R_TC_D:
    case Levels::RECONFIGURE_NMPC_R_AC_D: {
      // Rtc_d and Rac_d are replaced by Rtd_c and Rad_c in this model, so they must not overwrite the weight of the
      // actuator velocity inputs
      ROS_INFO_STREAM("The setting variable is not in the list!");
      break;
    }
    default: {
      TiltMtServoThrustDistNMPC::callbackMPCcfg(config, level);
      break;
    }
  }
}
}
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustDiffMPC, aerial_robot_control::ControlBase);
