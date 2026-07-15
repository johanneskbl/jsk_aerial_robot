#include "aerial_robot_control/differential_mpc/tilt_mt_servo_thrust_diff_mpc_controller.h"

namespace aerial_robot_control
{
namespace nmpc
{
void TiltMtServoThrustDiffMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                double ctrl_loop_du)
{
  TiltMtServoNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  /* Subscribers */
  sub_esc_telem_ = nh_.subscribe("esc_telem", 1, &TiltMtServoThrustDiffMPC::callbackESCTelem, this);

  /* Publishers */
  pub_record_ref_ = nh_.advertise<aerial_robot_msgs::MPCTrajectory>("nmpc/record_ref", 1);
  pub_record_pred_ = nh_.advertise<aerial_robot_msgs::MPCTrajectory>("nmpc/record_pred", 1);
}

void TiltMtServoThrustDiffMPC::initGeneralParams()
{
  TiltMtServoNMPC::initGeneralParams();

  ros::NodeHandle motor_nh(nh_, "motor_info");
  getParam<double>(motor_nh, "krpm_square_to_thrust_ratio", krpm_square_to_thrust_ratio_, 0.0);
  getParam<double>(motor_nh, "krpm_square_to_thrust_bias", krpm_square_to_thrust_bias_, 0.0);
}

void TiltMtServoThrustDiffMPC::initActuatorStates()
{
  TiltMtServoNMPC::initActuatorStates();
  thrust_meas_.resize(motor_num_, 0.0);
  
  servo_start_idx_ = 13;
  thrust_start_idx_ = 13 + joint_num_;
  wrench_state_start_idx_ = 13 + joint_num_ + motor_num_;

  internal_wrench_b_ = Eigen::VectorXd::Zero(6);
  
  uo_prev_ = std::vector<double>(motor_num_ + joint_num_, 0.0);
}

void TiltMtServoThrustDiffMPC::initNMPCCostW()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  /* control parameters with dynamic reconfigure */
  double Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Qt, Qfu, Qtau, Rtd_c, Rad_c;
  getParam<double>(nmpc_nh, "Qp_xy", Qp_xy, 300);
  getParam<double>(nmpc_nh, "Qp_z", Qp_z, 400);
  getParam<double>(nmpc_nh, "Qv_xy", Qv_xy, 10);
  getParam<double>(nmpc_nh, "Qv_z", Qv_z, 10);
  getParam<double>(nmpc_nh, "Qq_xy", Qq_xy, 300);
  getParam<double>(nmpc_nh, "Qq_z", Qq_z, 300);
  getParam<double>(nmpc_nh, "Qw_xy", Qw_xy, 5);
  getParam<double>(nmpc_nh, "Qw_z", Qw_z, 5);
  getParam<double>(nmpc_nh, "Qa", Qa, 1);
  getParam<double>(nmpc_nh, "Qt", Qt, 1);
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
  for (int i = servo_start_idx_; i < servo_start_idx_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = thrust_start_idx_; i < thrust_start_idx_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);

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

  
  // Nullspace control
  double nullspace_servo_gain, nullspace_thrust_gain;
  getParam<double>(nmpc_nh, "nullspace_servo_gain", nullspace_servo_gain, 0.0);
  getParam<double>(nmpc_nh, "nullspace_thrust_gain", nullspace_thrust_gain, 0.0);
  if (nullspace_servo_gain == 0.0 && nullspace_thrust_gain == 0.0)
  {
    include_nullspace_control_ = false;
  }
  else
  {
    include_nullspace_control_ = true;
    if (Qa != 0.0 || Qt != 0.0)
    {
      ROS_FATAL("Nullspace control is enabled, but Qa or Qt is not zero. Since the reference must be set to zero this is badly conditioned.");
    }
  }

  ROS_INFO("MPC cost W initialized:\n"
           "\tQp_xy=%f, Qp_z=%f, Qv_xy=%f, Qv_z=%f, Qq_xy=%f, Qq_z=%f, Qw_xy=%f, Qw_z=%f,\n"
           "\tQa=%f, Qt=%f, Qfu=%f, Qtau=%f, Rtd_c=%f, Rad_c=%f,\n"
           "\tnullspace_servo_gain=%f, nullspace_thrust_gain=%f",
           Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Qt, Qfu, Qtau, Rtd_c, Rad_c,
           nullspace_servo_gain, nullspace_thrust_gain);
}

void TiltMtServoThrustDiffMPC::initNMPCConstraints()
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

  //  TODO: this should be set in flight_navigation; don't know why set 0.2 results solver failure
  getParam<double>(control_nh, "vel_limit_takeoff", vel_limit_takeoff_, 1.0);  // m/s

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

  ROS_INFO("MPC constraints are set: \n",
           "\tvel_min: %f, vel_max: %f, w_min: %f, w_max: %f,\n",
           "\tthrust_min: %f, thrust_max: %f, thrust_vel_min: %f, thrust_vel_max: %f,\n",
           "\tservo_angle_min: %f, servo_angle_max: %f, servo_angle_vel_min: %f, servo_angle_vel_max: %f",
           vel_min_, vel_max_, body_rate_min, body_rate_max,
           thrust_ctrl_min_, thrust_ctrl_max_, thrust_vel_min_, thrust_vel_max_,
           servo_angle_min_, servo_angle_max_, servo_vel_min_, servo_vel_max_);
}

void TiltMtServoThrustDiffMPC::callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg)
{  // TODO: support different motor number
  double krpm = (double)msg->esc_telemetry_1.rpm * 0.001;
  thrust_meas_[0] = krpm * krpm * krpm_square_to_thrust_ratio_ + krpm_square_to_thrust_bias_;

  krpm = (double)msg->esc_telemetry_2.rpm * 0.001;
  thrust_meas_[1] = krpm * krpm * krpm_square_to_thrust_ratio_ + krpm_square_to_thrust_bias_;

  krpm = (double)msg->esc_telemetry_3.rpm * 0.001;
  thrust_meas_[2] = krpm * krpm * krpm_square_to_thrust_ratio_ + krpm_square_to_thrust_bias_;

  krpm = (double)msg->esc_telemetry_4.rpm * 0.001;
  thrust_meas_[3] = krpm * krpm * krpm_square_to_thrust_ratio_ + krpm_square_to_thrust_bias_;
}

bool TiltMtServoThrustDiffMPC::update()
{
  bool control_ready = ControlBase::update();

  // after press activate button, but before takeoff
  if (!control_ready)
  {
    if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE)
    {
      // Warmup the solver before actual takeoff
      is_warmup_ = true;
      controlCore();
      publishRecording();
    }
    return false;
  }
  else
  {
    is_warmup_ = false;
    controlCore();
    sendCmd();
    publishRecording();
  }
  return true;
}

void TiltMtServoThrustDiffMPC::reset()
{
  TiltMtServoNMPC::reset();
  uo_prev_ = std::vector<double>(motor_num_ + joint_num_, 0.0);
}

/**
 * @brief calXrUrRef: calculate the reference state and control input
 * @param ref_pos_i
 * @param ref_vel_i
 * @param ref_acc_i - the acceleration is in the inertial frame, no including the gravity
 * @param ref_quat_ib
 * @param ref_omega_b
 * @param ref_ang_acc_b
 * @param horizon_idx - set -1 for adding the target point to the end of the reference trajectory, 0 ~ NN for adding
 * the target point to the horizon_idx interval
 */
void TiltMtServoThrustDiffMPC::setXrUrRef(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                          const tf::Vector3& ref_acc_i, const tf::Quaternion& ref_quat_ib,
                                          const tf::Vector3& ref_omega_b, const tf::Vector3& ref_ang_acc_b,
                                          const int& horizon_idx)
{
  int& NX = mpc_solver_ptr_->NX_;
  int& NU = mpc_solver_ptr_->NU_;
  int& NN = mpc_solver_ptr_->NN_;

  /* calculate the reference wrench in the body frame */
  Eigen::VectorXd acc_with_g_i(3);
  acc_with_g_i(0) = ref_acc_i.x();
  acc_with_g_i(1) = ref_acc_i.y();
  acc_with_g_i(2) = ref_acc_i.z() + gravity_const_;  // add gravity

  // coordinate transformation
  tf::Quaternion q_bi = ref_quat_ib.inverse();
  Eigen::Matrix3d rot_bi;
  tf::matrixTFToEigen(tf::Transform(q_bi).getBasis(), rot_bi);
  Eigen::VectorXd ref_acc_b = rot_bi * acc_with_g_i;

  Eigen::VectorXd ref_wrench_b(6);
  ref_wrench_b(0) = ref_acc_b(0) * mass_;
  ref_wrench_b(1) = ref_acc_b(1) * mass_;
  ref_wrench_b(2) = ref_acc_b(2) * mass_;
  ref_wrench_b(3) = ref_ang_acc_b.x() * inertia_.at(0);
  ref_wrench_b(4) = ref_ang_acc_b.y() * inertia_.at(1);
  ref_wrench_b(5) = ref_ang_acc_b.z() * inertia_.at(2);

  /* calculate X U from ref, aka. control allocation */
  std::vector<double> x(NX, 0.0);
  std::vector<double> u(NU, 0.0);
  allocateToXU(ref_pos_i, ref_vel_i, ref_quat_ib, ref_omega_b, ref_wrench_b, x, u);

  /* set values */
  if (horizon_idx == -1)
  {
    // Aim: gently add the target point to the end of the reference trajectory
    // - x: NN + 1, u: NN
    // - for 0 ~ NN-2 x and u, shift
    // - copy x to x: NN-1 and NN, copy u to u: NN-1
    for (int i = 0; i < NN - 1; i++)
    {
      // shift one step
      std::copy(x_u_ref_.x.data.begin() + NX * (i + 1), x_u_ref_.x.data.begin() + NX * (i + 2),
                x_u_ref_.x.data.begin() + NX * i);
      std::copy(x_u_ref_.u.data.begin() + NU * (i + 1), x_u_ref_.u.data.begin() + NU * (i + 2),
                x_u_ref_.u.data.begin() + NU * i);
    }
    std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * (NN - 1));
    std::copy(u.begin(), u.begin() + NU, x_u_ref_.u.data.begin() + NU * (NN - 1));

    std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * NN);

    return;
  }

  if (horizon_idx < 0 || horizon_idx > NN)
  {
    ROS_WARN("horizon_idx is out of range! CalXrUrRef failed!");
    return;
  }

  std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * horizon_idx);
  if (horizon_idx < NN)
    std::copy(u.begin(), u.begin() + NU, x_u_ref_.u.data.begin() + NU * horizon_idx);
}

void TiltMtServoThrustDiffMPC::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                            const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                                            const Eigen::VectorXd& ref_wrench_b, vector<double>& x,
                                            vector<double>& u)
{
  x.at(0) = ref_pos_i.x();
  x.at(1) = ref_pos_i.y();
  x.at(2) = ref_pos_i.z();
  x.at(3) = ref_vel_i.x();
  x.at(4) = ref_vel_i.y();
  x.at(5) = ref_vel_i.z();
  x.at(6) = ref_quat_ib.w();
  x.at(7) = ref_quat_ib.x();
  x.at(8) = ref_quat_ib.y();
  x.at(9) = ref_quat_ib.z();
  x.at(10) = ref_omega_b.x();
  x.at(11) = ref_omega_b.y();
  x.at(12) = ref_omega_b.z();

  // Servo angle and thrust reference
  // NOTE: Not to be used when including differential allocation because the wrench can be directly used as 
  // reference without problematic servo angle mapping 
  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * ref_wrench_b;
  std::vector<double> ft_ref_vec(motor_num_);
  std::vector<double> a_ref_vec(joint_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    a_ref_vec[i] = atan2(x_lambda(2 * i), x_lambda(2 * i + 1));  // Contains discontinuities
    ft_ref_vec[i] = sqrt(x_lambda(2 * i) * x_lambda(2 * i) + x_lambda(2 * i + 1) * x_lambda(2 * i + 1));
    x.at(servo_start_idx_ + i) = a_ref_vec[i];
    x.at(thrust_start_idx_ + i) = ft_ref_vec[i];
  }

  // Internal wrench state reference
  // NOTE: Include if `self.include_differential_allocation` is set to True in Python file
  x.at(wrench_state_start_idx_ + 0) = ref_wrench_b(0);
  x.at(wrench_state_start_idx_ + 1) = ref_wrench_b(1);
  x.at(wrench_state_start_idx_ + 2) = ref_wrench_b(2);
  x.at(wrench_state_start_idx_ + 3) = ref_wrench_b(3);
  x.at(wrench_state_start_idx_ + 4) = ref_wrench_b(4);
  x.at(wrench_state_start_idx_ + 5) = ref_wrench_b(5);
  
  // Zero-out control input since control input is thrust and servo derivative
  for (int i = 0; i < motor_num_ + joint_num_; i++)
  {
    u.at(i) = 0.0;
  }

  // ======================================================================================
  // ############ Heuristic allocation ############
  if (alloc_type_ == 0)
    return;

  // 1) Check if one rotor's thrust is less than threshold and flipped backwards
  std::vector<int> rotor_idx_vec;
  for (int i = 0; i < motor_num_; i++)
  {
    if (ft_ref_vec[i] > ft_thresh_)
      continue;

    if (a_ref_vec[i] >= -M_PI_2 && a_ref_vec[i] <= M_PI_2)
      continue;

    rotor_idx_vec.push_back(i);
  }

  if (rotor_idx_vec.empty())
    return;

  ROS_INFO_THROTTLE(1.0, "-------------------------------------------------");

  int rotor_idx;
  if (rotor_idx_vec.size() > 1)
  {
    double max_ft = 0.0;
    int max_rotor_idx = -1;
    for (const auto& idx : rotor_idx_vec)
    {
      if (ft_ref_vec[idx] > max_ft)
      {
        max_ft = ft_ref_vec[idx];
        max_rotor_idx = idx;
      }
    }
    rotor_idx = max_rotor_idx;
    ROS_WARN_THROTTLE(1.0, "[NMPC] More than one rotor is below threshold and flipped backwards! Select rotor %d with "
      "largest thrust %.2f and angle %.2f as the fixed rotor.", rotor_idx, max_ft, a_ref_vec[max_rotor_idx]);
  }
  else
  {
    rotor_idx = rotor_idx_vec.at(0);
    ROS_WARN_THROTTLE(1.0, "[NMPC] Rotor %d is below threshold and flipped backwards! Select this rotor with thrust %.2f and "
      "angle %.2f as the fixed rotor.", rotor_idx, ft_ref_vec[rotor_idx], a_ref_vec[rotor_idx]);
  }

  // 2) If rotor_idx is not empty, maintain the thrust and modify the angle
  double ft_stop_rotor = ft_ref_vec[rotor_idx];
  double alpha_stop_rotor = M_PI_2 - acos(x_lambda(2 * rotor_idx) / ft_thresh_);
  
  // 3) Re-allocate with this rotor's thrust and angle fixed
  ROS_INFO_THROTTLE(1.0, "[NMPC] Changed reference:");
  for (int i = 0; i < motor_num_; i++)
  {
    ROS_INFO_THROTTLE(1.0, "\t[BEFORE] Rotor %d: thrust %.2f,  \tangle %.2f", i, ft_ref_vec[i], a_ref_vec[i]);
  }
  allocateToXUwOneFixedRotor(rotor_idx, ft_stop_rotor, alpha_stop_rotor, ref_wrench_b, x, u);
  for (int i = 0; i < motor_num_; i++)
  {
    ROS_INFO_THROTTLE(1.0, "\t[AFTER]  Rotor %d: thrust %.2f,  \tangle %.2f", i, x.at(thrust_start_idx_ + i), x.at(servo_start_idx_ + i));
  }
}

void TiltMtServoThrustDiffMPC::allocateToXUwOneFixedRotor(int fix_rotor_idx, double fix_ft, double fix_alpha,
                                                          const VectorXd& ref_wrench_b, vector<double>& x,
                                                          vector<double>& u)
{
  double fix_ft_x = fix_ft * sin(fix_alpha);
  double fix_ft_y = fix_ft * cos(fix_alpha);

  // 1) Construct target_wrench from z_from_rotor
  Eigen::VectorXd z_from_rotor = Eigen::VectorXd::Zero(motor_num_ * 2);
  z_from_rotor(2 * fix_rotor_idx) = fix_ft_x;
  z_from_rotor(2 * fix_rotor_idx + 1) = fix_ft_y;
  Eigen::VectorXd target_wrench_from_rotor = alloc_mat_ * z_from_rotor;

  // 2) Calculate alloc_mat with this rotor's contribution
  Eigen::VectorXd ref_wrench_modified = ref_wrench_b - target_wrench_from_rotor;

  // 3) Calculate the allocation matrix without this rotor, which is 6*6
  if (fix_rotor_idx != rotor_idx_prev_)
  {
    Eigen::MatrixXd alloc_mat_del_rotor(alloc_mat_.rows(), alloc_mat_.cols() - 2);
    int j = 0;
    for (int k = 0; k < alloc_mat_.cols(); ++k)
    {
      if (k == 2 * fix_rotor_idx || k == 2 * fix_rotor_idx + 1)
        continue;
      alloc_mat_del_rotor.col(j++) = alloc_mat_.col(k);
    }
    alloc_mat_del_rotor_inv_ = alloc_mat_del_rotor.inverse();
  }

  // 4) Reconstruct the z output
  Eigen::VectorXd z_except_rotor = alloc_mat_del_rotor_inv_ * ref_wrench_modified;

  // 5) At the index of 2*fix_rotor_idx, insert 2 numbers to z_except_rotor
  Eigen::VectorXd z_final(motor_num_ * 2);
  z_final.head(2 * fix_rotor_idx) = z_except_rotor.head(2 * fix_rotor_idx);
  z_final(2 * fix_rotor_idx) = fix_ft_x;
  z_final(2 * fix_rotor_idx + 1) = fix_ft_y;
  z_final.tail(z_except_rotor.size() - 2 * fix_rotor_idx) =
      z_except_rotor.tail(z_except_rotor.size() - 2 * fix_rotor_idx);

  // 6) Reconstruct the thrust and servo angle
  for (int i = 0; i < motor_num_; i++)
  {
    const double ft = sqrt(z_final(2 * i) * z_final(2 * i) + z_final(2 * i + 1) * z_final(2 * i + 1));
    const double alpha = atan2(z_final(2 * i), z_final(2 * i + 1));
    x.at(servo_start_idx_ + i) = ensureOneServoContinuity(alpha, i);
    x.at(thrust_start_idx_ + i) = ft;
  }

  // If the fixed rotor is the same with previous one, no need to recalculate the allocation matrix.
  rotor_idx_prev_ = fix_rotor_idx;
}

std::vector<double> TiltMtServoThrustDiffMPC::meas2VecX(bool is_ee_centric)
{
  auto bx0 = TiltMtServoNMPC::meas2VecX(is_ee_centric);

  for (int i = 0; i < motor_num_; i++)
    bx0[thrust_start_idx_ + i] = std::max(thrust_meas_[i], 0.0);

  // Internal wrench as state
  // NOTE: Include if `self.include_differential_allocation` is set to True in Python file
  computeInternalWrenchB();
  bx0[wrench_state_start_idx_ + 0] = internal_wrench_b_(0);
  bx0[wrench_state_start_idx_ + 1] = internal_wrench_b_(1);
  bx0[wrench_state_start_idx_ + 2] = internal_wrench_b_(2);
  bx0[wrench_state_start_idx_ + 3] = internal_wrench_b_(3);
  bx0[wrench_state_start_idx_ + 4] = internal_wrench_b_(4);
  bx0[wrench_state_start_idx_ + 5] = internal_wrench_b_(5);

  return bx0;
}

void TiltMtServoThrustDiffMPC::computeInternalWrenchB()
{
  if (alloc_mat_.size() == 0 || thrust_meas_.size() != motor_num_)
  {
    ROS_WARN("[NMPC] Allocation matrix is not set or thrust measurement size is not equal to motor number, we cannot compute internal wrench, set it to zero!");
    internal_wrench_b_ = Eigen::VectorXd::Zero(6);
    return;
  }

  // Compute current force and torque from current servo angles and forces
  Eigen::VectorXd allocation_matrix_input = Eigen::VectorXd::Zero(2 * motor_num_);
  for (int i = 0; i < motor_num_; ++i)
  {
    allocation_matrix_input(2 * i) = thrust_meas_[i] * std::sin(joint_angles_[i]);
    allocation_matrix_input(2 * i + 1) = thrust_meas_[i] * std::cos(joint_angles_[i]);
  }

  // Compute current body wrench from current force and torque
  internal_wrench_b_ = alloc_mat_ * allocation_matrix_input;
}

double TiltMtServoThrustDiffMPC::getCommand(int idx_u, double T_horizon)
{
  // ENTIRELY DIFFERENT IDEA:
  // Instead of using control input here, get the predicted STATE of thrust and servo inside the MODEL!
  // This makes sense since in SQP-RTI the state is also an optimization variable
  // Interpolate to next sample time based on current and next state in the horizon
  if (idx_u < motor_num_)
  {
    // Interpolate one control period into the horizon WITH rotor time constant
    double x0 = mpc_solver_ptr_->xo_.at(0).at(thrust_start_idx_ + idx_u);
    double x1 = mpc_solver_ptr_->xo_.at(1).at(thrust_start_idx_ + idx_u);
    return x0 + (x1 - x0) * t_rotor_ / t_nmpc_step_;
  }
  else
  {
    // Interpolate one control period into the horizon WITH servo time constant
    double x0 = mpc_solver_ptr_->xo_.at(0).at(servo_start_idx_ + (idx_u - motor_num_));
    double x1 = mpc_solver_ptr_->xo_.at(1).at(servo_start_idx_ + (idx_u - motor_num_));
    return x0 + (x1 - x0) * t_servo_ / t_nmpc_step_;
  }

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
  //   uo = joint_angles_[idx_u - motor_num_] + uo_derivative / ctrl_loop_du_;
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
  //   measurement = joint_angles_[idx_u - motor_num_];
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

void TiltMtServoThrustDiffMPC::publishRecording()
{
  // int& NN = mpc_solver_ptr_->NN_;
  int NN = 1;  // Only for next prediction to avoid high runtime overhead
  auto stamp = ros::Time::now();

  // Publish reference states
  aerial_robot_msgs::MPCTrajectory ref_msg;
  ref_msg.header.frame_id = "world";
  ref_msg.header.stamp = stamp;
  ref_msg.states.resize(NN + 1);
  ref_msg.controls.resize(NN);

  for (int i = 0; i <= NN; ++i)
  {
    // Position
    ref_msg.states[i].position.x = mpc_solver_ptr_->xr_[i][0];
    ref_msg.states[i].position.y = mpc_solver_ptr_->xr_[i][1];
    ref_msg.states[i].position.z = mpc_solver_ptr_->xr_[i][2];
    
    // Linear velocity
    ref_msg.states[i].linear_velocity.x = mpc_solver_ptr_->xr_[i][3];
    ref_msg.states[i].linear_velocity.y = mpc_solver_ptr_->xr_[i][4];
    ref_msg.states[i].linear_velocity.z = mpc_solver_ptr_->xr_[i][5];
    
    // Quaternion
    ref_msg.states[i].orientation.w = mpc_solver_ptr_->xr_[i][6];
    ref_msg.states[i].orientation.x = mpc_solver_ptr_->xr_[i][7];
    ref_msg.states[i].orientation.y = mpc_solver_ptr_->xr_[i][8];
    ref_msg.states[i].orientation.z = mpc_solver_ptr_->xr_[i][9];
    
    // Angular velocity
    ref_msg.states[i].angular_velocity.x = mpc_solver_ptr_->xr_[i][10];
    ref_msg.states[i].angular_velocity.y = mpc_solver_ptr_->xr_[i][11];
    ref_msg.states[i].angular_velocity.z = mpc_solver_ptr_->xr_[i][12];
    
    // Servo angle state
    ref_msg.states[i].servo_angles.resize(joint_num_);
    for (int j = 0; j < joint_num_; ++j)
    {
      ref_msg.states[i].servo_angles[j] = mpc_solver_ptr_->xr_[i][servo_start_idx_ + j];
    }

    // Thrust state
    ref_msg.states[i].thrust.resize(motor_num_);
    for (int j = 0; j < motor_num_; ++j)
    {
      ref_msg.states[i].thrust[j] = mpc_solver_ptr_->xr_[i][thrust_start_idx_ + j];
    }

    // Wrench state
    ref_msg.states[i].wrench_forces.x = mpc_solver_ptr_->xr_[i][wrench_state_start_idx_ + 0];
    ref_msg.states[i].wrench_forces.y = mpc_solver_ptr_->xr_[i][wrench_state_start_idx_ + 1];
    ref_msg.states[i].wrench_forces.z = mpc_solver_ptr_->xr_[i][wrench_state_start_idx_ + 2];
    ref_msg.states[i].wrench_torques.x = mpc_solver_ptr_->xr_[i][wrench_state_start_idx_ + 3];
    ref_msg.states[i].wrench_torques.y = mpc_solver_ptr_->xr_[i][wrench_state_start_idx_ + 4];
    ref_msg.states[i].wrench_torques.z = mpc_solver_ptr_->xr_[i][wrench_state_start_idx_ + 5];
  }

  for (int i = 0; i < NN; ++i)
  {
    // Thrust commands
    ref_msg.controls[i].thrust_commands.resize(motor_num_);
    for (int j = 0; j < motor_num_; ++j)
    {
      ref_msg.controls[i].thrust_commands[j] = mpc_solver_ptr_->ur_[i][j];
    }

    // Servo angle commands
    ref_msg.controls[i].servo_angle_commands.resize(joint_num_);
    for (int j = 0; j < joint_num_; ++j)
    {
      ref_msg.controls[i].servo_angle_commands[j] = mpc_solver_ptr_->ur_[i][j+motor_num_];
    }
  }
  pub_record_ref_.publish(ref_msg);

  // Predicted states
  aerial_robot_msgs::MPCTrajectory pred_msg;
  pred_msg.header.frame_id = "world";
  pred_msg.header.stamp = stamp;
  pred_msg.states.resize(NN + 1);
  pred_msg.controls.resize(NN);

  for (int i = 0; i <= NN; ++i)
  {
    // Position
    pred_msg.states[i].position.x = mpc_solver_ptr_->xo_[i][0];
    pred_msg.states[i].position.y = mpc_solver_ptr_->xo_[i][1];
    pred_msg.states[i].position.z = mpc_solver_ptr_->xo_[i][2];
    
    // Linear velocity
    pred_msg.states[i].linear_velocity.x = mpc_solver_ptr_->xo_[i][3];
    pred_msg.states[i].linear_velocity.y = mpc_solver_ptr_->xo_[i][4];
    pred_msg.states[i].linear_velocity.z = mpc_solver_ptr_->xo_[i][5];
    
    // Quaternion
    pred_msg.states[i].orientation.w = mpc_solver_ptr_->xo_[i][6];
    pred_msg.states[i].orientation.x = mpc_solver_ptr_->xo_[i][7];
    pred_msg.states[i].orientation.y = mpc_solver_ptr_->xo_[i][8];
    pred_msg.states[i].orientation.z = mpc_solver_ptr_->xo_[i][9];
    
    // Angular velocity
    pred_msg.states[i].angular_velocity.x = mpc_solver_ptr_->xo_[i][10];
    pred_msg.states[i].angular_velocity.y = mpc_solver_ptr_->xo_[i][11];
    pred_msg.states[i].angular_velocity.z = mpc_solver_ptr_->xo_[i][12];
    
    // Servo angle state
    pred_msg.states[i].servo_angles.resize(joint_num_);
    for (int j = 0; j < joint_num_; ++j)
    {
      pred_msg.states[i].servo_angles[j] = mpc_solver_ptr_->xo_[i][servo_start_idx_ + j];
    }

    // Thrust state
    pred_msg.states[i].thrust.resize(motor_num_);
    for (int j = 0; j < motor_num_; ++j)
    {
      pred_msg.states[i].thrust[j] = mpc_solver_ptr_->xo_[i][thrust_start_idx_ + j];
    }

    // Wrench state
    pred_msg.states[i].wrench_forces.x = mpc_solver_ptr_->xo_[i][wrench_state_start_idx_ + 0];
    pred_msg.states[i].wrench_forces.y = mpc_solver_ptr_->xo_[i][wrench_state_start_idx_ + 1];
    pred_msg.states[i].wrench_forces.z = mpc_solver_ptr_->xo_[i][wrench_state_start_idx_ + 2];
    pred_msg.states[i].wrench_torques.x = mpc_solver_ptr_->xo_[i][wrench_state_start_idx_ + 3];
    pred_msg.states[i].wrench_torques.y = mpc_solver_ptr_->xo_[i][wrench_state_start_idx_ + 4];
    pred_msg.states[i].wrench_torques.z = mpc_solver_ptr_->xo_[i][wrench_state_start_idx_ + 5];
  }
  
  for (int i = 0; i < NN; ++i)
  {
    // Thrust commands
    pred_msg.controls[i].thrust_commands.resize(motor_num_);
    for (int j = 0; j < motor_num_; ++j)
    {
      pred_msg.controls[i].thrust_commands[j] = mpc_solver_ptr_->uo_[i][j];
    }

    // Servo angle commands
    pred_msg.controls[i].servo_angle_commands.resize(joint_num_);
    for (int j = 0; j < joint_num_; ++j)
    {
      pred_msg.controls[i].servo_angle_commands[j] = mpc_solver_ptr_->uo_[i][j+motor_num_];
    }
  }
  pub_record_pred_.publish(pred_msg);
}

void TiltMtServoThrustDiffMPC::cfgNMPCCallback(aerial_robot_control::NMPCConfig& config, uint32_t level)
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
        case Levels::RECONFIGURE_NMPC_Q_T: {
          for (int i = thrust_start_idx_; i < thrust_start_idx_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qt);
          ROS_INFO_STREAM("change Qt for NMPC '" << config.Qt << "'");
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
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustDiffMPC, aerial_robot_control::ControlBase);
