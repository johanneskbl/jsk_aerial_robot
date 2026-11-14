//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                       double ctrl_loop_du)
{
  BaseMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  /* Initialize plugins */
  initPlugins();

  /* Initialize general parameters from ROS node */
  initGeneralParams();

  /* Initialize actuator states */
  initActuatorStates();

  /* Initialize cost weights */
  initMPCCostW();

  /* Initialize dynamic reconfigure server to allow changing parameters at runtime */
  mpc_reconf_servers_.push_back(boost::make_shared<MPCControlDynamicConfig>(mpc_nh_));
  mpc_reconf_servers_.back()->setCallback(boost::bind(&TiltMtServoNMPC::callbackMPCcfg, this, _1, _2));

  /* Initialize constraints */
  initMPCConstraints();

  /* Initialize reference object */
  initPredXU(x_u_ref_);

  /* Publishers */
  pub_flight_cmd_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  pub_gimbal_control_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  pub_record_ref_ = nh_.advertise<aerial_robot_msgs::MPCTrajectory>("nmpc/record_ref", 1);
  pub_record_pred_ = nh_.advertise<aerial_robot_msgs::MPCTrajectory>("nmpc/record_pred", 1);

  /* Subscribers */
  sub_joint_states_ = nh_.subscribe("joint_states", 5, &TiltMtServoNMPC::callbackJointStates, this);
  sub_set_rpy_ = nh_.subscribe("set_rpy", 5, &TiltMtServoNMPC::callbackSetRPY, this);
  sub_set_ref_x_u_ = nh_.subscribe("set_ref_x_u", 5, &TiltMtServoNMPC::callbackSetRefXU, this);
  sub_set_traj_ = nh_.subscribe("set_ref_traj", 5, &TiltMtServoNMPC::callbackSetRefTraj, this);
  sub_set_fixed_rotor_ = nh_.subscribe("set_fixed_rotor", 5, &TiltMtServoNMPC::callbackSetFixedRotor, this);

  /* Set mode in spinal controller */
  setControlMode();

  ROS_INFO("MPC Controller initialized!");
}

/**
 * @brief Activate the controller and run initializations
 */
void nmpc::TiltMtServoNMPC::activate()
{
  /* Generic activation: warm-up flag, sanity checks, inertial parameters and flight state to spinal */
  BaseMPC::activate();

  /* Initialize allocation matrix */
  initAllocMat();

  /* Initialize MPC parameters */
  initMPCPhysicalParams();

  /* Initialize quaternions */
  q_prev_.setValue(0, 0, 0, 1);                 // (x,y,z,w)
  tracked_target_q_wt_prev_.setValue(0, 0, 0, 1);  // (x,y,z,w)
  quat_ref_prev_.assign(mpc_solver_ptr_->NN_ + 1, tf::Quaternion(0, 0, 0, 1));  // (x,y,z,w)

  /* Set takeoff velocity constraints */
  modifyVelConstraints(-vel_limit_takeoff_, vel_limit_takeoff_);
  has_restored_vel_ = false;  // reset the flag, so that we can restore the velocity after hovering
}

void nmpc::TiltMtServoNMPC::reset()
{
  BaseMPC::reset();

  resetPlugins();

  /* Reset reference x_u_ref_ to the current state of the Tracking frame */
  std::vector<double> x_vec_ref = meas2VecX(isTrackingEE());
  std::vector<double> u_vec(mpc_solver_ptr_->NU_, 0);

  int &NX = mpc_solver_ptr_->NX_, &NU = mpc_solver_ptr_->NU_, &NN = mpc_solver_ptr_->NN_;
  for (int i = 0; i < NN; i++)
  {
    std::copy(x_vec_ref.begin(), x_vec_ref.begin() + NX, x_u_ref_.x.data.begin() + NX * i);
    std::copy(u_vec.begin(), u_vec.begin() + NU, x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x_vec_ref.begin(), x_vec_ref.begin() + NX, x_u_ref_.x.data.begin() + NX * NN);

  mpc_solver_ptr_->resetXrUrByX0U0(x_vec_ref, u_vec);

  /* Reset control input */
  // [!] Don't publish actuation here
  flight_cmd_.base_thrust = std::vector<float>(motor_num_, 0.0);

  gimbal_ctrl_cmd_.name.clear();
  gimbal_ctrl_cmd_.position.clear();
  for (int i = 0; i < joint_num_; i++)
  {
    gimbal_ctrl_cmd_.name.emplace_back("gimbal" + std::to_string(i + 1));
    gimbal_ctrl_cmd_.position.push_back(0.0);
  }

  pub_gimbal_control_.publish(gimbal_ctrl_cmd_);

  /* Reset solver and set initial guess to hovering */
  joint_angles_.assign(joint_num_, 0.0);  // Reset joint angle state to zero
  resetSolver();
}

/**
 * @brief Reset the solver iterate with the current state and a hover guess for the input
 * Used on reset and to clear a corrupted (e.g., NaN) iterate after solver failures.
 * NOTE: Derived classes with a different input (e.g., actuator velocities) or internal command states must override.
 */
void nmpc::TiltMtServoNMPC::resetSolver()
{
  std::vector<double> x_vec = meas2VecX(false);  // Always in Body frame
  std::vector<double> u_guess(mpc_solver_ptr_->NU_, 0.0);
  for (int i = 0; i < motor_num_ && i < static_cast<int>(u_guess.size()); ++i)
    u_guess[i] = mass_ * gravity_const_ / motor_num_;  // in [N]
  for (int i = 0; i < joint_num_ && motor_num_ + i < static_cast<int>(u_guess.size()); ++i)
    u_guess[motor_num_ + i] = joint_angles_[i];  // in [rad]
  mpc_solver_ptr_->resetSolverByX0U0(x_vec, u_guess);
}

void nmpc::TiltMtServoNMPC::initGeneralParams()
{
  /* General MPC parameters */
  BaseMPC::initGeneralParams();

  /* Physical and allocation parameters */
  ros::NodeHandle physical_nh(nh_, "physical");
  ros::NodeHandle alloc_nh(control_nh_, "alloc");

  getParam<int>(alloc_nh, "type", alloc_type_, 0);
  getParam<double>(alloc_nh, "ft_thresh", ft_thresh_, 0.5);

  getParam<int>(physical_nh, "num_servos", joint_num_, 0);
  getParam<double>(physical_nh, "t_servo", t_servo_, 0.01);
  getParam<int>(physical_nh, "num_rotors", motor_num_, 0);
  getParam<double>(physical_nh, "t_rotor", t_rotor_, 0.01);

  /* Sanity checks */
  if (ft_thresh_ <= 0.0)
  {
    std::string err_msg =
        "ft_thresh must be greater than zero! Please set a positive value for ft_thresh in the parameter server.";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }
  if (motor_num_ != joint_num_)
  {
    std::string err_msg = "motor_num_ is not equal to joint_num_! Currently allocation indexing assume they are equal.";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }
  if (motor_num_ <= 0)
  {
    std::string err_msg =
        "motor_num_ must be greater than zero! Please set a positive value for motor_num_ in the parameter server.";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }
  if (motor_num_ != static_cast<int>(robot_model_->getRotorNum()))
  {
    std::string err_msg = "motor_num_ is not equal to the number of rotors in the robot model! Please set motor_num_ "
                          "to match the robot model.";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }
}

void nmpc::TiltMtServoNMPC::initMPCCostW()
{
  /* Solver weights set with dynamic reconfigure */
  double Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Rt, Rac_d;
  getParam<double>(mpc_nh_, "Qp_xy", Qp_xy, 300);
  getParam<double>(mpc_nh_, "Qp_z", Qp_z, 400);
  getParam<double>(mpc_nh_, "Qv_xy", Qv_xy, 10);
  getParam<double>(mpc_nh_, "Qv_z", Qv_z, 10);
  getParam<double>(mpc_nh_, "Qq_xy", Qq_xy, 300);
  getParam<double>(mpc_nh_, "Qq_z", Qq_z, 300);
  getParam<double>(mpc_nh_, "Qw_xy", Qw_xy, 5);
  getParam<double>(mpc_nh_, "Qw_z", Qw_z, 5);
  getParam<double>(mpc_nh_, "Qa", Qa, 1);
  getParam<double>(mpc_nh_, "Rt", Rt, 1);
  getParam<double>(mpc_nh_, "Rac_d", Rac_d, 250);

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
  for (int i = 13; i < 13 + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rt, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);
}

void nmpc::TiltMtServoNMPC::initMPCConstraints()
{
  double body_rate_max, body_rate_min;
  getParam<double>(mpc_nh_, "w_max", body_rate_max, 6.0);
  getParam<double>(mpc_nh_, "w_min", body_rate_min, -6.0);
  getParam<double>(mpc_nh_, "v_max", vel_max_, 1.0);
  getParam<double>(mpc_nh_, "v_min", vel_min_, -1.0);
  getParam<double>(mpc_nh_, "thrust_max", thrust_ctrl_max_, 0.0);
  getParam<double>(mpc_nh_, "thrust_min", thrust_ctrl_min_, 0.0);
  getParam<double>(mpc_nh_, "a_max", servo_angle_max_, 3.1416);
  getParam<double>(mpc_nh_, "a_min", servo_angle_min_, -3.1416);

  /* Sanity checks */
  if (vel_max_ < 0.0 || vel_min_ > 0.0 || vel_min_ > vel_max_ || body_rate_max < 0.0 || body_rate_min > 0.0 ||
      body_rate_min > body_rate_max || servo_angle_max_ <= 0.0 || servo_angle_min_ >= 0.0 ||
      servo_angle_min_ > servo_angle_max_)
  {
    std::string err_msg = "Velocity and servo angle limits must be set such that max > 0 and min < 0 and min < max!";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }
  if (thrust_ctrl_min_ < 0.0 || thrust_ctrl_max_ <= thrust_ctrl_min_)
  {
    std::string err_msg = "Thrust limits must be set such that 0 <= min < max!";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }

  //  TODO: this should be set in flight_navigation
  getParam<double>(mpc_nh_, "vel_limit_takeoff", vel_limit_takeoff_, 1.0);  // m/s

  // lbx and ubx
  std::vector<int> idxbx = mpc_solver_ptr_->getConstraintsIdxbx();
  std::vector<int> idxbx_desired = { 3, 4, 5, 10, 11, 12 };  // vx, vy, vz, wx, wy, wz
  idxbx_desired.resize(6 + joint_num_);
  for (int i = 0; i < joint_num_; i++)
  {
    idxbx_desired[6 + i] = 13 + i;  // joint angles
  }
  if (idxbx.size() != idxbx_desired.size() || !std::equal(idxbx.begin(), idxbx.end(), idxbx_desired.begin()))
  {
    ROS_ERROR("idxbx is not equal to idxbx_desired, we cannot set constraints lbx and ubx!");
  }

  std::vector<double> lbx = { vel_min_, vel_min_, vel_min_, body_rate_min, body_rate_min, body_rate_min };
  std::vector<double> ubx = { vel_max_, vel_max_, vel_max_, body_rate_max, body_rate_max, body_rate_max };
  lbx.resize(6 + joint_num_);
  ubx.resize(6 + joint_num_);
  for (int i = 0; i < joint_num_; i++)
  {
    lbx[6 + i] = servo_angle_min_;
    ubx[6 + i] = servo_angle_max_;
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
    lbu[i] = thrust_ctrl_min_;
    ubu[i] = thrust_ctrl_max_;
  }
  for (int i = 0; i < joint_num_; i++)
  {
    lbu[motor_num_ + i] = servo_angle_min_;
    ubu[motor_num_ + i] = servo_angle_max_;
  }
  mpc_solver_ptr_->setConstraintsLbu(lbu);
  mpc_solver_ptr_->setConstraintsUbu(ubu);
}

void nmpc::TiltMtServoNMPC::initAllocMat()
{
  /* Get model configuration */
  int rotor_num = robot_model_->getRotorNum();  // For tilt-rotor, rotor_num = servo_num
  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::map<int, int> rotor_dr = robot_model_->getRotorDirection();
  double kq_d_kt = std::abs(robot_model_->getMFRate());  // PAY ATTENTION: should be positive value

  /* Construct allocation matrix */
  alloc_mat_ = Eigen::MatrixXd::Zero(6, 2 * rotor_num);

  for (int i = 0; i < rotor_num; i++)
  {
    Eigen::Vector3d p_b = rotor_p[i];
    int dr = rotor_dr.find(i + 1)->second;  // PAY ATTENTION: the rotor index starts from 1!!!!!!!!!!!!!!!!!!!!!

    double sqrt_p_xy = sqrt(p_b.x() * p_b.x() + p_b.y() * p_b.y());

    // Force
    alloc_mat_(0, 2 * i) = p_b.y() / sqrt_p_xy;
    alloc_mat_(1, 2 * i) = -p_b.x() / sqrt_p_xy;
    alloc_mat_(2, 2 * i + 1) = 1;

    // Torque
    alloc_mat_(3, 2 * i) = -dr * kq_d_kt * p_b.y() / sqrt_p_xy + p_b.x() * p_b.z() / sqrt_p_xy;
    alloc_mat_(4, 2 * i) = dr * kq_d_kt * p_b.x() / sqrt_p_xy + p_b.y() * p_b.z() / sqrt_p_xy;
    alloc_mat_(5, 2 * i) = -p_b.x() * p_b.x() / sqrt_p_xy - p_b.y() * p_b.y() / sqrt_p_xy;

    alloc_mat_(3, 2 * i + 1) = p_b.y();
    alloc_mat_(4, 2 * i + 1) = -p_b.x();
    alloc_mat_(5, 2 * i + 1) = -dr * kq_d_kt;
  }

  /* Compute the pseudoinverse of the allocation matrix */
  alloc_mat_pinv_ = aerial_robot_model::pseudoinverse(alloc_mat_);
}

void nmpc::TiltMtServoNMPC::initMPCPhysicalParams()
{
  /* Construct acados parameters */
  std::vector<double> acados_p(mpc_solver_ptr_->NP_, 0.0);

  acados_p[0] = 1.0;  // qw
  idx_p_quat_end_ = 3;

  int idx;
  // TODO: this condition is temporary for drones that don't use these phys param (e.g., bi, tri, fix-qd)
  if (mpc_solver_ptr_->NP_ > 4 + 6)
  {
    std::vector<double> phys_p = getPhysicalParams();
    std::copy(phys_p.begin(), phys_p.end(), acados_p.begin() + idx_p_quat_end_ + 1);
    idx = idx_p_quat_end_ + static_cast<int>(phys_p.size());

    ROS_INFO("Set physical parameters for MPC solver.");
  }
  else
  {
    idx = idx_p_quat_end_;
  }

  // Store idx_p_phys_end_ for setting other parameters later
  idx_p_phys_end_ = idx;

  /* Set acados parameters */
  mpc_solver_ptr_->setParameters(acados_p);
}

void nmpc::TiltMtServoNMPC::modifyVelConstraints(double vel_min, double vel_max) const
{
  // Hardcoded: velocity constraints are the first three elements
  // TODO: consider making it more general
  std::vector<double> lbx = mpc_solver_ptr_->getConstraintsLbx();
  lbx[0] = vel_min;
  lbx[1] = vel_min;
  lbx[2] = vel_min;
  mpc_solver_ptr_->setConstraintsLbx(lbx);

  std::vector<double> lbxe = mpc_solver_ptr_->getConstraintsLbxe();
  lbxe[0] = vel_min;
  lbxe[1] = vel_min;
  lbxe[2] = vel_min;
  mpc_solver_ptr_->setConstraintsLbxe(lbxe);

  std::vector<double> ubx = mpc_solver_ptr_->getConstraintsUbx();
  ubx[0] = vel_max;
  ubx[1] = vel_max;
  ubx[2] = vel_max;
  mpc_solver_ptr_->setConstraintsUbx(ubx);

  std::vector<double> ubxe = mpc_solver_ptr_->getConstraintsUbxe();
  ubxe[0] = vel_max;
  ubxe[1] = vel_max;
  ubxe[2] = vel_max;
  mpc_solver_ptr_->setConstraintsUbxe(ubxe);

  ROS_INFO("Velocity constraints modified: [%f, %f, %f] for lbx and [%f, %f, %f] for ubx", lbx[0], lbx[1], lbx[2],
           ubx[0], ubx[1], ubx[2]);
}

/**
 * @brief Construct the physical parameters vector for the current robot configuration
 * @return Physical parameters as a std::vector<double>
 */
std::vector<double> nmpc::TiltMtServoNMPC::getPhysicalParams() const
{
  /* Get model configuration */
  int rotor_num = robot_model_->getRotorNum();  // For tilt-rotor, rotor_num = servo_num
  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::map<int, int> rotor_dr = robot_model_->getRotorDirection();
  double kq_d_kt = std::abs(robot_model_->getMFRate());  // PAY ATTENTION: should be positive value

  /* Construct physical parameters vector */
  std::vector<double> phys_p(2 + 3 + 1 + 4 * rotor_num + 2 + 7, 0);
  // Order of parameters:
  // mass, gravity, Ixx, Iyy, Izz, kq_d_kt, dr1, p1_b, dr2, p2_b, dr3, p3_b, dr4, p4_b,
  // t_rotor, t_servo, ee_p, ee_qwxyz
  phys_p[0] = mass_;
  phys_p[1] = gravity_const_;
  phys_p[2] = inertia_[0];
  phys_p[3] = inertia_[1];
  phys_p[4] = inertia_[2];
  phys_p[5] = kq_d_kt;
  int idx = 6;
  for (int i = 0; i < rotor_num; i++)
  {
    phys_p[idx] = rotor_dr.find(i + 1)->second;
    idx++;
    phys_p[idx] = rotor_p[i].x();
    idx++;
    phys_p[idx] = rotor_p[i].y();
    idx++;
    phys_p[idx] = rotor_p[i].z();
    idx++;
  }
  phys_p[idx] = t_rotor_;
  idx++;
  phys_p[idx] = t_servo_;
  idx++;

  /* Pose of the Tracking frame in the Body frame (ee_p, ee_qwxyz) */
  // For CoG tracking the cached pose is the identity, so that a solver generated for EE tracking also tracks the
  // Body frame and robots without an "ee_contact" frame are supported
  tf::Vector3 t_p_b;
  tf::Quaternion q_bt;
  getTrackingFramePose(t_p_b, q_bt);

  phys_p[idx++] = t_p_b.x();
  phys_p[idx++] = t_p_b.y();
  phys_p[idx++] = t_p_b.z();

  phys_p[idx++] = q_bt.w();
  phys_p[idx++] = q_bt.x();
  phys_p[idx++] = q_bt.y();
  phys_p[idx++] = q_bt.z();

  return phys_p;
}

/**
 * @brief Get the measurement as state vector
 * @param is_ee_centric Flag to transform the measured state from the Body frame to the Tracking frame.
 * The MPC model and thus the internal solver state always describes the Body frame, so only
 * use true to build an EE centric reference.
 * @return State vector as a std::vector<double>
 */
std::vector<double> nmpc::TiltMtServoNMPC::meas2VecX(bool is_ee_centric)
{
  std::vector<double> bx0(mpc_solver_ptr_->NBX0_, 0);

  /* Measured state of the Body frame */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Quaternion q = estimator_->getQuat(Frame::COG, estimate_mode_);
  tf::Vector3 omega = estimator_->getAngularVel(Frame::COG, estimate_mode_);

  // Check the sign of the quaternion to avoid flipping
  ensureQuaternionContinuity(q, q_prev_);

  /* Transform the state from the Body frame to the Tracking frame */
  if (is_ee_centric)
    convertFromBToT(pos, vel, q, omega, pos, vel, q, omega);

  /* Fill the state vector */
  bx0.at(0) = pos.x();
  bx0.at(1) = pos.y();
  bx0.at(2) = pos.z();
  bx0.at(3) = vel.x();
  bx0.at(4) = vel.y();
  bx0.at(5) = vel.z();
  bx0.at(6) = q.w();
  bx0.at(7) = q.x();
  bx0.at(8) = q.y();
  bx0.at(9) = q.z();
  bx0.at(10) = omega.x();
  bx0.at(11) = omega.y();
  bx0.at(12) = omega.z();
  for (int i = 0; i < joint_num_; i++)
    bx0[13 + i] = joint_angles_[i];

  return bx0;
}

/**
 * @brief Main control loop
 * NOTE: When integrating (e.g., I-term), we need to skip the update during warm-up
 */
void nmpc::TiltMtServoNMPC::controlCore()
{
  /* Restore velocity constraints after hovering */
  if (navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE && has_restored_vel_ == false)
  {
    modifyVelConstraints(vel_min_, vel_max_);
    has_restored_vel_ = true;
  }

  /* Prepare parameters and reference for MPC */
  prepareMPCParams();

  prepareMPCRef();

  /* Get current state from estimator */
  std::vector<double> bx0 = meas2VecX();

  /* Solve the optimization problem */
  bool is_valid = solve(bx0);

  /* Get result from solver and check validity */
  std::vector<float> thrust_cmd(motor_num_, 0.0);
  std::vector<double> servo_cmd(joint_num_, 0.0);
  if (is_valid)
  {
    for (int i = 0; i < motor_num_; i++)
      thrust_cmd[i] = static_cast<float>(getCommand(i));
    for (int i = 0; i < joint_num_; i++)
      servo_cmd[i] = getCommand(motor_num_ + i);

    is_valid = std::all_of(thrust_cmd.begin(), thrust_cmd.end(), [](float v) { return std::isfinite(v); }) &&
               std::all_of(servo_cmd.begin(), servo_cmd.end(), [](double v) { return std::isfinite(v); });
    if (!is_valid)
      ROS_ERROR_THROTTLE(1.0, "[MPC] Solver returned a non-finite command (NaN or Inf).");
  }

  /* Update command */
  if (is_valid)
  {
    flight_cmd_.base_thrust = thrust_cmd;

    gimbal_ctrl_cmd_.header.stamp = ros::Time::now();
    for (int i = 0; i < joint_num_; i++)
    {
      gimbal_ctrl_cmd_.position.at(i) = servo_cmd[i];
    }
  }
  else
  {
    handleSolverFailure();
  }

  /* Logging */
  ROS_INFO_THROTTLE(1.0, "[MPC] ==============================================");
  ROS_INFO_THROTTLE(1.0, "[MPC] Closed-loop: \t%.3f ms (%0.3f Hz)", t_mpc_dt_ * 1000.0, 1.0 / t_mpc_dt_);
  publishSolverTime();
}

/**
 * @brief Send the command to spinal
 */
void nmpc::TiltMtServoNMPC::sendCmd()
{
  if (motor_num_ > 0)
    pub_flight_cmd_.publish(flight_cmd_);
  if (joint_num_ > 0 && !has_triggered_force_landing_)
    pub_gimbal_control_.publish(gimbal_ctrl_cmd_);
}

/**
 * @brief Set the reference for the MPC solver depending on the current mode: trajectory tracking / target point
 */
void nmpc::TiltMtServoNMPC::prepareMPCRef()
{
  /* Trajectory tracking mode */
  // The reference is set by callbackSetRefXU() and assumed to be already described in the Tracking frame.
  // So here we only check if the trajectory is still received. If not, we turn off the tracking mode.
  if (is_traj_tracking_)
  {
    const ros::Duration t_since_traj = ros::Time::now() - x_u_ref_.header.stamp;
    if (t_since_traj > ros::Duration(0.5))
    {
      ROS_INFO("No traj msg for 0.5s. Trajectory tracking mode is off! Return to the hovering!");
      is_traj_tracking_ = false;
      tf::Vector3 current_pos = estimator_->getPos(Frame::COG, estimate_mode_);
      tf::Vector3 current_rpy = estimator_->getEuler(Frame::COG, estimate_mode_);

      navigator_->setTargetPosX((float)current_pos.x());
      navigator_->setTargetPosY((float)current_pos.y());
      navigator_->setTargetPosZ((float)current_pos.z());
      navigator_->setTargetVelX(0.0);
      navigator_->setTargetVelY(0.0);
      navigator_->setTargetVelZ(0.0);
      navigator_->setTargetRoll(0.0);
      navigator_->setTargetPitch(0.0);
      navigator_->setTargetYaw((float)current_rpy.z());
      navigator_->setTargetOmegaX(0.0);
      navigator_->setTargetOmegaY(0.0);
      navigator_->setTargetOmegaZ(0.0);
    }
    else if (t_since_traj > ros::Duration(0.1))
    {
      ROS_INFO_THROTTLE(1, "No traj msg for 0.1s. Try to track current pose.");
      tf::Vector3 current_pos = estimator_->getPos(Frame::COG, estimate_mode_);
      tf::Vector3 current_rpy = estimator_->getEuler(Frame::COG, estimate_mode_);

      navigator_->setTargetPosX((float)current_pos.x());
      navigator_->setTargetPosY((float)current_pos.y());
      navigator_->setTargetPosZ((float)current_pos.z());
      navigator_->setTargetVelX(0.0);
      navigator_->setTargetVelY(0.0);
      navigator_->setTargetVelZ(0.0);
      navigator_->setTargetRoll((float)current_rpy.x());
      navigator_->setTargetPitch((float)current_rpy.y());
      navigator_->setTargetYaw((float)current_rpy.z());
      navigator_->setTargetOmegaX(0.0);
      navigator_->setTargetOmegaY(0.0);
      navigator_->setTargetOmegaZ(0.0);

      last_traj_msg_.points.clear();  // Clear the traj msg every time when ending traj tracking
    }
    else
    {
      // The received trajectory is tracked
      return;
    }
  }

  /* Target mode */
  // The target of the navigator always describes the Body frame (origin at the CoG)
  tf::Vector3 target_pos_w = navigator_->getTargetPos();
  tf::Vector3 target_vel_w = navigator_->getTargetVel();
  tf::Vector3 target_rpy = navigator_->getTargetRPY();
  tf::Quaternion target_q_wb;
  target_q_wb.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
  tf::Vector3 target_omega_b = navigator_->getTargetOmega();

  // Transform the target to the Tracking frame
  tf::Vector3 tracked_target_pos_w, tracked_target_vel_w, tracked_target_omega_t;
  tf::Quaternion tracked_target_q_wt;
  convertFromBToT(target_pos_w, target_vel_w, target_q_wb, target_omega_b, tracked_target_pos_w, tracked_target_vel_w,
                  tracked_target_q_wt, tracked_target_omega_t);

  // Ensure quaternion continuity to avoid discontinuity in reference signal
  ensureQuaternionContinuity(tracked_target_q_wt, tracked_target_q_wt_prev_);

  /* Allocate the target to a full state and control input reference */
  setXrUrRef(tracked_target_pos_w, tracked_target_vel_w, tf::Vector3(0, 0, 0), tracked_target_q_wt,
             tracked_target_omega_t, tf::Vector3(0, 0, 0), -1);

  /* Send reference to the solver */
  rosXU2VecXU(x_u_ref_, mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_);
  mpc_solver_ptr_->setReference(mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_, true);
}

void nmpc::TiltMtServoNMPC::prepareMPCParams()
{
  updateTrackingFramePose();

  updateInertialParams();

  // TODO: this condition is temporary for drones that don't pass in phys param (bi, tri, fix-qd)
  if (mpc_solver_ptr_->NP_ > 4 + 6)
  {
    std::vector<double> phys_p = getPhysicalParams();

    std::vector<int> idx(phys_p.size());
    std::iota(idx.begin(), idx.end(), idx_p_quat_end_ + 1);

    mpc_solver_ptr_->setParamSparseAllStages(idx, phys_p);
  }
}

/**
 * @brief Allocate the full state and control input reference from a target reference and
 * set to the class reference variables at the horizon index.
 * NOTE: All arguments are assumed to describe the Tracking frame T, i.e.,
 * the Body frame B (origin at the CoG) or the EE frame.
 * @param ref_pos_w Position of the origin of T, expressed in W
 * @param ref_vel_w Linear velocity of the origin of T, expressed in W
 * @param ref_acc_w Linear acceleration of the origin of T, expressed in W, not including gravity
 * @param ref_quat_wt Orientation of T w.r.t. W (rotates vectors from T to W)
 * @param ref_omega_t Angular velocity of T w.r.t. W, expressed in T
 * @param ref_ang_acc_t Angular acceleration of T w.r.t. W, expressed in T
 * @param horizon_idx Index of the horizon interval to add the reference point to:
 * Set to -1 for smoothly adding the reference point to the end of the reference trajectory
 * Set to i in [0, NN] for adding the reference point to the ith node
 */
void nmpc::TiltMtServoNMPC::setXrUrRef(const tf::Vector3& ref_pos_w, const tf::Vector3& ref_vel_w,
                                       const tf::Vector3& ref_acc_w, const tf::Quaternion& ref_quat_wt,
                                       const tf::Vector3& ref_omega_t, const tf::Vector3& ref_ang_acc_t,
                                       const int& horizon_idx)
{
  int& NX = mpc_solver_ptr_->NX_;
  int& NU = mpc_solver_ptr_->NU_;
  int& NN = mpc_solver_ptr_->NN_;

  /* Calculate the reference wrench in Body frame */
  Eigen::VectorXd ref_wrench_b = computeRefWrenchB(ref_acc_w, ref_quat_wt, ref_omega_t, ref_ang_acc_t);

  /* Allocate state and control inputs from the reference wrench */
  std::vector<double> x(NX, 0.0);
  std::vector<double> u(NU, 0.0);
  allocateToXU(ref_pos_w, ref_vel_w, ref_quat_wt, ref_omega_t, ref_wrench_b, x, u);

  /* Set reference to the solver */
  // For horizon_idx == -1: add a single target point from the navigator (not a full trajectory)
  if (horizon_idx == -1)
  {
    if (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
    {
      /* Set the reference over the entire horizon */
      // During takeoff, set the reference over the entire horizon to avoid large jump
      // from idle only at the last node
      for (int i = 0; i <= NN; i++)
      {
        std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * i);
        if (i < NN)
          std::copy(u.begin(), u.begin() + NU, x_u_ref_.u.data.begin() + NU * i);
      }
    }
    else
    {
      /* Add a new reference point only to the end of the horizon */
      // Aim: gently add the new reference point to the end of the horizon,
      // and shift the previous reference points backwards
      // - x: NN + 1, u: NN
      // - for 0 ~ NN-2 x and u, shift
      // - copy x to x: NN-1 and NN, copy u to u: NN-1
      for (int i = 0; i < NN - 1; i++)
      {
        // Shift existing reference points one step back
        std::copy(x_u_ref_.x.data.begin() + NX * (i + 1), x_u_ref_.x.data.begin() + NX * (i + 2),
                  x_u_ref_.x.data.begin() + NX * i);
        std::copy(x_u_ref_.u.data.begin() + NU * (i + 1), x_u_ref_.u.data.begin() + NU * (i + 2),
                  x_u_ref_.u.data.begin() + NU * i);
      }
      // Add the new reference point to the end of the horizon (actually at NN-1 and NN for x)
      std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * (NN - 1));
      std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * NN);
      std::copy(u.begin(), u.begin() + NU, x_u_ref_.u.data.begin() + NU * (NN - 1));
    }
    return;
  }

  // For horizon_idx in [0, NN]: add reference at the horizon node idx
  if (horizon_idx < 0 || horizon_idx > NN)
  {
    ROS_WARN("horizon_idx %d is out of range [0, %d]! Failed to set reference in setXrUrRef().", horizon_idx, NN);
    return;
  }

  std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * horizon_idx);
  if (horizon_idx < NN)
    std::copy(u.begin(), u.begin() + NU, x_u_ref_.u.data.begin() + NU * horizon_idx);
}

/**
 * @brief Construct and allocate the state and control inputs from the referenced wrench
 * @param ref_pos_w Position of the origin of T, expressed in W
 * @param ref_vel_w Linear velocity of the origin of T, expressed in W
 * @param ref_quat_wt Orientation of T w.r.t. W
 * @param ref_omega_t Angular velocity of T w.r.t. W, expressed in T
 * @param ref_wrench_b Reference wrench in the Body frame
 * NOTE: The wrench must be expressed in the Body frame to be allocated to the actuator states and controls
 * @param x State vector to be filled
 * @param u Control vector to be filled
 */
void nmpc::TiltMtServoNMPC::allocateToXU(const tf::Vector3& ref_pos_w, const tf::Vector3& ref_vel_w,
                                         const tf::Quaternion& ref_quat_wt, const tf::Vector3& ref_omega_t,
                                         const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                                         std::vector<double>& u)
{
  x.at(0) = ref_pos_w.x();
  x.at(1) = ref_pos_w.y();
  x.at(2) = ref_pos_w.z();
  x.at(3) = ref_vel_w.x();
  x.at(4) = ref_vel_w.y();
  x.at(5) = ref_vel_w.z();
  x.at(6) = ref_quat_wt.w();
  x.at(7) = ref_quat_wt.x();
  x.at(8) = ref_quat_wt.y();
  x.at(9) = ref_quat_wt.z();
  x.at(10) = ref_omega_t.x();
  x.at(11) = ref_omega_t.y();
  x.at(12) = ref_omega_t.z();

  // 0) If one rotor is fixed, allocate without this rotor's contribution
  if (is_set_fix_rotor_)
  {
    if (ros::Time::now() - fix_rotor_msg_.header.stamp > ros::Duration(0.1))
    {
      ROS_INFO_THROTTLE(1, "No FixRotor msg for 0.1s. Recover to the normal allocation state.");
      is_set_fix_rotor_ = false;
    }

    allocateToXUwOneFixedRotor(fix_rotor_msg_.rotor_id, fix_rotor_msg_.fix_ft, fix_rotor_msg_.fix_alpha, ref_wrench_b,
                               x, u);
    return;
  }

  // 1) Allocate (motor_num_ == joint_num_ assumed)
  std::vector<double> ft_ref_vec(motor_num_);
  std::vector<double> a_ref_vec(joint_num_);
  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * ref_wrench_b;
  for (int i = 0; i < motor_num_; i++)
  {
    ft_ref_vec[i] = sqrt(x_lambda(2 * i) * x_lambda(2 * i) + x_lambda(2 * i + 1) * x_lambda(2 * i + 1));
    a_ref_vec[i] = atan2(x_lambda(2 * i), x_lambda(2 * i + 1));

    u.at(i) = ft_ref_vec[i];
    x.at(13 + i) = ensureOneServoContinuity(a_ref_vec[i], i);
  }

  if (alloc_type_ == 0)
    // Normal allocation, no further action needed
    return;

  /* The following allocation method contains a heuristic mapping from the referenced
     wrench to actuation necessary to achieve the desired pose and wrench. */
  // 2) Check if one rotor's thrust is less than threshold and flip backwards
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

    ROS_WARN(
        "More than one rotor is below threshold and flip backwards! Select rotor %d with thrust %.2f as the "
        "fixed rotor.",
        rotor_idx, max_ft);
  }
  else
  {
    rotor_idx = rotor_idx_vec.at(0);
  }

  // 3) If rotor_idx is not empty, maintain the thrust and manually modify
  // the angle of the stopped rotor
  double ft_stop_rotor = ft_ref_vec[rotor_idx];
  double alpha_stop_rotor = M_PI_2 - acos(x_lambda(2 * rotor_idx) / ft_thresh_);

  // 4) Re-allocate with this fixed rotor's contribution removed from the wrench
  allocateToXUwOneFixedRotor(rotor_idx, ft_stop_rotor, alpha_stop_rotor, ref_wrench_b, x, u);
}

/**
 * @brief Allocate the state and control inputs with one rotor's contribution fixed
 * @param fix_rotor_idx Index of the fixed rotor
 * @param fix_ft Fixed thrust of the rotor
 * @param fix_alpha Fixed angle of the rotor
 * @param ref_wrench_b Reference wrench in the Body frame
 * @param x State vector to be filled
 * @param u Control vector to be filled
 */
void nmpc::TiltMtServoNMPC::allocateToXUwOneFixedRotor(int fix_rotor_idx, double fix_ft, double fix_alpha,
                                                       const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                                                       std::vector<double>& u)
{
  double fix_ft_x = fix_ft * sin(fix_alpha);
  double fix_ft_y = fix_ft * cos(fix_alpha);

  // 1) Construct target_wrench from z_from_rotor
  Eigen::VectorXd z_from_rotor = Eigen::VectorXd::Zero(motor_num_ * 2);
  z_from_rotor(2 * fix_rotor_idx) = fix_ft_x;
  z_from_rotor(2 * fix_rotor_idx + 1) = fix_ft_y;
  Eigen::VectorXd target_wrench_from_rotor = alloc_mat_ * z_from_rotor;

  // 2) Isolate target wrench without this rotor's contribution
  Eigen::VectorXd target_wrench_modified = ref_wrench_b - target_wrench_from_rotor;

  // 3) Calculate the allocation matrix without this rotor (dimension 6x6)
  if (fix_rotor_idx != rotor_idx_prev_)  // skip if already calculated for this rotor
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
  rotor_idx_prev_ = fix_rotor_idx;

  // 4) Reconstruct the z output
  Eigen::VectorXd z_except_rotor = alloc_mat_del_rotor_inv_ * target_wrench_modified;

  // 5) At the place of 2*fix_rotor_idx, insert 2 numbers to z_except_rotor
  Eigen::VectorXd z_final(motor_num_ * 2);
  z_final.head(2 * fix_rotor_idx) = z_except_rotor.head(2 * fix_rotor_idx);
  z_final(2 * fix_rotor_idx) = fix_ft_x;
  z_final(2 * fix_rotor_idx + 1) = fix_ft_y;
  z_final.tail(z_except_rotor.size() - 2 * fix_rotor_idx) =
      z_except_rotor.tail(z_except_rotor.size() - 2 * fix_rotor_idx);

  // 6) Reconstruct the thrust and servo angle (motor_num_ == joint_num_ assumed)
  for (int i = 0; i < motor_num_; i++)
  {
    const double ft = sqrt(z_final(2 * i) * z_final(2 * i) + z_final(2 * i + 1) * z_final(2 * i + 1));
    u.at(i) = ft;
    const double alpha = atan2(z_final(2 * i), z_final(2 * i + 1));
    x.at(13 + i) = ensureOneServoContinuity(alpha, i);
  }
}

void nmpc::TiltMtServoNMPC::callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
  if (static_cast<int>(msg->position.size()) == joint_num_)
  {
    for (int i = 0; i < joint_num_; i++)
      joint_angles_[i] = msg->position[i];
  }
  else
  {
    ROS_WARN_THROTTLE(1, "Received joint states with size %zu, but expected %d. Ignoring this message.",
                      msg->position.size(), joint_num_);
  }
}

/**
 * @brief Track only the desired orientation of the robot (mainly for testing)
 */
void nmpc::TiltMtServoNMPC::callbackSetRPY(const spinal::DesireCoordConstPtr& msg)
{
  // add a check to avoid the singular point for euler angle
  if (msg->pitch == M_PI / 2.0 or msg->pitch == -M_PI / 2.0)
  {
    ROS_WARN(
        "The pitch angle is set to PI/2 or -PI/2, which is a singular point for euler angle."
        " Please set other values for the pitch angle.");
    return;
  }

  navigator_->setTargetRoll(msg->roll);
  navigator_->setTargetPitch(msg->pitch);
  navigator_->setTargetYaw(msg->yaw);
}

/**
 * @brief Track a reference trajectory from a message
 * The reference must describe the Tracking frame (Body or EE frame, see tracking_frame_) and name it in
 * header.frame_id ("cog" or "ee_contact").
 * @param msg Message of type aerial_robot_msgs::PredXU containing the reference point
 * TODO: This function and other reference callbacks should be combined with the inner planning framework
 */
void nmpc::TiltMtServoNMPC::callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg)
{
  /* Sanity check */
  if (navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE)
  {
    ROS_WARN_THROTTLE(1, "The robot has not hovered, so the reference trajectory will be ignored!");
    return;
  }
  if (!isRefInTrackingFrame(msg->header.frame_id))
    return;
  if (static_cast<int>(msg->x.data.size()) != mpc_solver_ptr_->NX_ * (mpc_solver_ptr_->NN_ + 1) ||
      static_cast<int>(msg->u.data.size()) != mpc_solver_ptr_->NU_ * mpc_solver_ptr_->NN_)
  {
    ROS_WARN_THROTTLE(1, "The size of the reference state and control input is not equal to the prediction horizon! "
                         "Cannot use the reference!");
    return;
  }

  /* Switch tracking mode */
  if (!is_traj_tracking_)
  {
    ROS_INFO("Trajectory tracking mode is on!");
    is_traj_tracking_ = true;
  }

  /* Read message */
  x_u_ref_ = *msg;

  /* Ensure quaternion continuity by preventing flips at each node */
  for (int i = 0; i <= mpc_solver_ptr_->NN_; i++)
  {
    const int base = i * mpc_solver_ptr_->NX_;

    tf::Quaternion ref_q_wt(x_u_ref_.x.data[base + 7],  // x
                            x_u_ref_.x.data[base + 8],  // y
                            x_u_ref_.x.data[base + 9],  // z
                            x_u_ref_.x.data[base + 6]   // w
    );

    ensureQuaternionContinuity(ref_q_wt, quat_ref_prev_[i]);

    x_u_ref_.x.data[base + 6] = ref_q_wt.w();
    x_u_ref_.x.data[base + 7] = ref_q_wt.x();
    x_u_ref_.x.data[base + 8] = ref_q_wt.y();
    x_u_ref_.x.data[base + 9] = ref_q_wt.z();
  }

  /* Set the reference state and control input to the solver */
  // NOTE: The reference trajectory is assumed to already include proper allocation
  // NOTE: The reference trajectory must describe the Tracking frame and is therefore not transformed here
  rosXU2VecXU(x_u_ref_, mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_);
  mpc_solver_ptr_->setReference(mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_, true);
}

/**
 * @brief Track a reference trajectory from a message
 * The trajectory must describe the Tracking frame (Body or EE frame, see tracking_frame_) and name it in
 * header.frame_id ("cog" or "ee_contact").
 * @param msg Message of type trajectory_msgs::MultiDOFJointTrajectory containing the reference trajectory
 */
void nmpc::TiltMtServoNMPC::callbackSetRefTraj(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  /* Sanity checks */
  if (static_cast<int>(msg->points.size()) != mpc_solver_ptr_->NN_ + 1)
  {
    ROS_WARN("The length of the trajectory is not equal to the prediction horizon! Cannot use the trajectory!");
    return;
  }

  if (navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE)
  {
    ROS_WARN_THROTTLE(1, "The robot has not hovered, so the reference trajectory will be ignored!");
    return;
  }

  if (!isRefInTrackingFrame(msg->header.frame_id))
    return;

  /* Check if the trajectory from the planner is the same as the last one to avoid unnecessary allocation effort */
  int max_same_idx = 0;
  if (!last_traj_msg_.points.empty())  // check if the last trajectory is empty
  {
    for (int i = 0; i < static_cast<int>(msg->points.size()); i++)
    {
      if (isMulDOFJointTrajPtEqual(msg->points[i], last_traj_msg_.points[i], false))  // time is not equal
        max_same_idx = i;
      else
        break;
    }
  }
  last_traj_msg_ = *msg;

  /* If the trajectory is different from the last one, or if a fixed rotor is set, update the reference trajectory */
  if (max_same_idx != static_cast<int>(msg->points.size()) - 1 || is_set_fix_rotor_ == true)
  {
    for (int i = 0; i < mpc_solver_ptr_->NN_ + 1; i++)
    {
      // Read message
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& point = msg->points[i];
      geometry_msgs::Vector3 pos = point.transforms[0].translation;
      geometry_msgs::Vector3 vel = point.velocities[0].linear;
      geometry_msgs::Vector3 acc = point.accelerations[0].linear;
      geometry_msgs::Quaternion quat = point.transforms[0].rotation;
      geometry_msgs::Vector3 omega = point.velocities[0].angular;
      geometry_msgs::Vector3 ang_acc = point.accelerations[0].angular;
      // Allocate reference state and control input
      setXrUrRef(tf::Vector3(pos.x, pos.y, pos.z), tf::Vector3(vel.x, vel.y, vel.z), tf::Vector3(acc.x, acc.y, acc.z),
                 tf::Quaternion(quat.x, quat.y, quat.z, quat.w), tf::Vector3(omega.x, omega.y, omega.z),
                 tf::Vector3(ang_acc.x, ang_acc.y, ang_acc.z), i);
    }
  }

  /* Set reference to solver */
  x_u_ref_.header.stamp = msg->header.stamp;
  x_u_ref_.header.frame_id = msg->header.frame_id;
  callbackSetRefXU(boost::make_shared<const aerial_robot_msgs::PredXU>(x_u_ref_));
}

/**
 * @brief Set a fixed rotor's thrust and angle for allocation
 * @param msg Message of type aerial_robot_msgs::FixRotorConst containing the fixed rotor information
 */
void nmpc::TiltMtServoNMPC::callbackSetFixedRotor(const aerial_robot_msgs::FixRotorConstPtr& msg)
{
  /* Failsafe */
  if (msg->rotor_id < 0 || msg->rotor_id >= motor_num_)
  {
    ROS_WARN_STREAM("The rotor_id " << static_cast<int>(msg->rotor_id)
                                    << " is incorrect. Note that the id starts from 0.");
    return;
  }

  if (msg->fix_ft < thrust_ctrl_min_ || msg->fix_ft > thrust_ctrl_max_)
  {
    ROS_WARN_STREAM("The fix_ft value " << msg->fix_ft << " is out of range. It should be between " << thrust_ctrl_min_
                                        << " and " << thrust_ctrl_max_ << ".");
    return;
  }

  if (msg->fix_alpha < servo_angle_min_ || msg->fix_alpha > servo_angle_max_)
  {
    ROS_WARN_STREAM("The fix_alpha value " << msg->fix_alpha << " is out of range. It should be between "
                                           << servo_angle_min_ << " and " << servo_angle_max_ << ".");
    return;
  }

  /* Store fixed rotor message */
  is_set_fix_rotor_ = true;
  fix_rotor_msg_ = *msg;
}

/**
 * @brief Publish the predicted and reference trajectory with
 * full state and controls vectors for dataset recording.
 * NOTE: Both trajectories describe the Body frame. For EE tracking, the reference (in the EE frame for the solver) is
 * transformed back to the Body frame here, only for the recording.
 */
void nmpc::TiltMtServoNMPC::publishRecording()
{
  int& NN = mpc_solver_ptr_->NN_;
  ros::Time stamp = ros::Time::now();

  // Publish reference states
  aerial_robot_msgs::MPCTrajectory ref_msg;
  ref_msg.header.frame_id = "world";
  ref_msg.header.stamp = stamp;
  ref_msg.states.resize(NN + 1);
  ref_msg.controls.resize(NN);

  for (int i = 0; i <= NN; ++i)
  {
    const std::vector<double>& xr = mpc_solver_ptr_->xr_[i];
    tf::Vector3 pos(xr[0], xr[1], xr[2]);
    tf::Vector3 vel(xr[3], xr[4], xr[5]);
    tf::Quaternion quat(xr[7], xr[8], xr[9], xr[6]);  // (x, y, z, w)
    tf::Vector3 omega(xr[10], xr[11], xr[12]);

    // Map the reference from the Tracking frame back to the Body frame to match the predicted states
    // (only for the recording)
    convertFromTToB(pos, vel, quat, omega, pos, vel, quat, omega);

    // Position
    ref_msg.states[i].position.x = pos.x();
    ref_msg.states[i].position.y = pos.y();
    ref_msg.states[i].position.z = pos.z();

    // Linear velocity
    ref_msg.states[i].linear_velocity.x = vel.x();
    ref_msg.states[i].linear_velocity.y = vel.y();
    ref_msg.states[i].linear_velocity.z = vel.z();

    // Quaternion
    ref_msg.states[i].orientation.w = quat.w();
    ref_msg.states[i].orientation.x = quat.x();
    ref_msg.states[i].orientation.y = quat.y();
    ref_msg.states[i].orientation.z = quat.z();

    // Angular velocity
    ref_msg.states[i].angular_velocity.x = omega.x();
    ref_msg.states[i].angular_velocity.y = omega.y();
    ref_msg.states[i].angular_velocity.z = omega.z();
    // Servo angle state
    ref_msg.states[i].servo_angles.resize(joint_num_);
    for (int j = 0; j < joint_num_; ++j)
    {
      ref_msg.states[i].servo_angles[j] = mpc_solver_ptr_->xr_[i][13 + j];
    }
  }

  for (int i = 0; i < NN; ++i)
  {
    // Thrust commands
    ref_msg.controls[i].thrust_cmd.resize(motor_num_);
    for (int j = 0; j < motor_num_; ++j)
    {
      ref_msg.controls[i].thrust_cmd[j] = mpc_solver_ptr_->ur_[i][j];
    }

    // Servo angle commands
    ref_msg.controls[i].servo_angle_cmd.resize(joint_num_);
    for (int j = 0; j < joint_num_; ++j)
    {
      ref_msg.controls[i].servo_angle_cmd[j] = mpc_solver_ptr_->ur_[i][j + motor_num_];
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
      pred_msg.states[i].servo_angles[j] = mpc_solver_ptr_->xo_[i][13 + j];
    }
  }

  for (int i = 0; i < NN; ++i)
  {
    // Thrust commands
    pred_msg.controls[i].thrust_cmd.resize(motor_num_);
    for (int j = 0; j < motor_num_; ++j)
    {
      pred_msg.controls[i].thrust_cmd[j] = mpc_solver_ptr_->uo_[i][j];
    }

    // Servo angle commands
    pred_msg.controls[i].servo_angle_cmd.resize(joint_num_);
    for (int j = 0; j < joint_num_; ++j)
    {
      pred_msg.controls[i].servo_angle_cmd[j] = mpc_solver_ptr_->uo_[i][j + motor_num_];
    }
  }
  pub_record_pred_.publish(pred_msg);
}

/**
 * @brief Callback function for dynamic reconfigure of MPC parameters
 * @param config Config object containing the new configuration
 * @param level Level of the configuration change indicating the specific
 * parameter to be changed
 * (see aerial_robot_msgs/msg/DynamicReconfigureLevels.msg for the levels)
 */
void nmpc::TiltMtServoNMPC::callbackMPCcfg(NMPCConfig& config, uint32_t level)
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
          for (int i = 13; i < 13 + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qa);
          ROS_INFO_STREAM("change Qa for NMPC '" << config.Qa << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_T: {
          for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rt, false);
          ROS_INFO_STREAM("change Rt for NMPC '" << config.Rt << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_AC_D: {
          for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rac_d, false);
          ROS_INFO_STREAM("change Rac_d for NMPC '" << config.Rac_d << "'");
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

double nmpc::TiltMtServoNMPC::ensureOneServoContinuity(double a_ref, int idx) const
{
  double a_now = gimbal_ctrl_cmd_.position[idx];
  if (a_ref - a_now > M_PI)
    a_ref -= 2 * M_PI;
  else if (a_ref - a_now < -M_PI)
    a_ref += 2 * M_PI;

  return a_ref;
}

std::vector<double> nmpc::TiltMtServoNMPC::ensureAllServoContinuity(std::vector<double>& a_ref_vec) const
{
  for (int i = 0; i < joint_num_; i++)
    a_ref_vec[i] = ensureOneServoContinuity(a_ref_vec[i], i);

  return a_ref_vec;
}

/* Plugin registration */
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoNMPC, aerial_robot_control::ControlBase)
