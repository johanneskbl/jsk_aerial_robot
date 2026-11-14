#include "aerial_robot_control/nmpc/base_mpc_controller.h"

using namespace aerial_robot_control;

/**
 * @brief Base class for MPC controllers
 * Includes the MPC solver plugin, ROS communication, and other common functionalities
 * shared by all MPC controllers
 */
void nmpc::BaseMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                               boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                               boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                               double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);
  mpc_nh_ = ros::NodeHandle(control_nh_, "nmpc");

  /* Initialize MPC solver plugin */
  mpc_solver_loader_ptr_ = boost::make_shared<pluginlib::ClassLoader<aerial_robot_control::mpc_solver::BaseMPCSolver>>(
      "aerial_robot_control", "aerial_robot_control::mpc_solver::BaseMPCSolver");

  // Read the plugin name from the parameter server
  std::string mpc_solver_name;
  if (!nh_.getParam("mpc_solver_name", mpc_solver_name))
  {
    std::string err_msg =
        "mpc_solver_name for mpc_solver_plugin is not loaded. You must specify the mpc_solver_name in the launch file.";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }

  // Load the plugin
  try
  {
    mpc_solver_ptr_ = mpc_solver_loader_ptr_->createInstance(mpc_solver_name);
    mpc_solver_ptr_->initialize();
    ROS_INFO("load mpc solver plugin: %s", mpc_solver_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    std::string err_msg = "mpc_solver_plugin: The plugin failed to load for some reason. Error: " + std::string(ex.what());
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }

  /* Initialize inertial parameters */
  updateInertialParams();

  /* Store solver dimensions */
  NN_ = mpc_solver_ptr_->NN_;
  NX_ = mpc_solver_ptr_->NX_;
  NU_ = mpc_solver_ptr_->NU_;
  NP_ = mpc_solver_ptr_->NP_;

  /* Publishers */
  pub_solver_time_ = nh_.advertise<std_msgs::Float64>("nmpc/solver_time", 1);
  pub_viz_ref_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_ref", 1);
  pub_viz_pred_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_pred", 1);
  pub_flight_config_cmd_spinal_ = nh_.advertise<spinal::FlightConfigCmd>("flight_config_cmd", 1);

  /* Services */
  srv_set_control_mode_ = nh_.serviceClient<spinal::SetControlMode>("set_control_mode");

  /* Timers */
  tmr_viz_ = nh_.createTimer(ros::Duration(0.05), &BaseMPC::callbackViz, this);
  t_mpc_timestep_ = t_mpc_prev_timestep_ = ros::Time::now().toSec();
}

/**
 * @brief Activate the controller when motor arming is issued and run the generic initializations
 */
void nmpc::BaseMPC::activate()
{
  ControlBase::activate();

  /* Warmup before takeoff: Flag to prevent integrators or similar from winding up */
  is_warmup_ = true;

  /* Sanity check */
  if (isTrackingEE() && !robot_model_->hasFrame("ee_contact"))
  {
    std::string err_msg = "Tracking frame is ee_tracking, but the robot model has no frame 'ee_contact'!";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }

  /* Initialize the pose of the Tracking frame */
  updateTrackingFramePose();

  /* Initialize inertial parameters */
  updateInertialParams();
  if (is_print_phys_params_)
    printPhysicalParams();

  /* Send flight state to spinal */
  // NOTE: This should run at least a couple times (e.g. in 10Hz) to ensure that spinal receives it
  spinal::FlightConfigCmd flight_config_cmd;
  flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
  pub_flight_config_cmd_spinal_.publish(flight_config_cmd);
}

/**
 * @brief Update loop
 * @return true if the controller is ready to control, false otherwise
 */
bool nmpc::BaseMPC::update()
{
  bool is_ready = ControlBase::update();
  if (!is_ready)
  {
    // After activate/arming, but before takeoff warm-up solver
    if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE)
    {
      is_warmup_ = true;
      controlCore();

      // Pulish full state and controls at reduced rate to avoid excessive rosbag size
      t_stamp_record_ = ros::Time::now();
      if ((t_stamp_record_ - t_last_record_).toSec() > 1.0 / record_rate_)
        publishRecording();
      t_last_record_ = t_stamp_record_;
    }
    return false;
  }
  else
  {
    is_warmup_ = false;
    double t_stamp_start = ros::Time::now().toSec();
    controlCore();
    double t_stamp_core = ros::Time::now().toSec();
    ROS_INFO_THROTTLE(1.0, "[MPC] controlCore(): \t%.3f ms", (t_stamp_core - t_stamp_start) * 1000.0);
    
    // Publish command to spinal
    sendCmd();

    // Pulish full state and controls at reduced rate to avoid excessive rosbag size
    t_stamp_record_ = ros::Time::now().toSec();
    if (t_stamp_record_ - t_last_record_ > 1.0 / record_rate_)
    {
      publishRecording();
      ROS_INFO_THROTTLE(1.0, "[MPC] publishRecording(): \t%.3f ms", (t_stamp_record_ - t_stamp_core) * 1000.0);
    }
    t_last_record_ = t_stamp_record_;
  }

  return true;
}

/**
 * @brief Reset the generic states of the MPC controller
 */
void nmpc::BaseMPC::reset()
{
  ControlBase::reset();
  has_triggered_force_landing_ = false;
}

/**
 * @brief Read the general parameters shared by all MPC controllers from the ROS parameter server
 */
void nmpc::BaseMPC::initGeneralParams()
{
  getParam<double>(mpc_nh_, "T_samp", t_mpc_samp_, 0.025);
  getParam<double>(mpc_nh_, "T_step", t_mpc_step_, 0.1);
  getParam<double>(mpc_nh_, "T_horizon", t_mpc_horizon_, 2.0);

  getParam<bool>(mpc_nh_, "is_attitude_ctrl", is_attitude_ctrl_, true);
  getParam<bool>(mpc_nh_, "is_body_rate_ctrl", is_body_rate_ctrl_, false);
  getParam<bool>(mpc_nh_, "is_print_phys_params", is_print_phys_params_, false);
  getParam<bool>(mpc_nh_, "is_debug", is_debug_, false);

  /* Set acados dimensions */
  mpc_nh_.setParam("NN", mpc_solver_ptr_->NN_);
  mpc_nh_.setParam("NX", mpc_solver_ptr_->NX_);
  mpc_nh_.setParam("NU", mpc_solver_ptr_->NU_);

  /* Frame tracked by the MPC cost */
  std::string tracking_frame;
  getParam<std::string>(mpc_nh_, "tracking_frame", tracking_frame, std::string("cog_tracking"));
  if (tracking_frame == "cog_tracking")
    tracking_frame_ = TrackingFrame::CoG;
  else if (tracking_frame == "ee_tracking")
    tracking_frame_ = TrackingFrame::EE;
  else
  {
    std::string err_msg = "Invalid tracking_frame '" + tracking_frame + "'! Choose cog_tracking or ee_tracking.";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }
  ROS_INFO("MPC tracking frame: %s", tracking_frame.c_str());

  if (is_debug_)
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
}

/**
 * @brief Set the control mode for the low-level controller on spinal
 */
void nmpc::BaseMPC::setControlMode()
{
  bool res = ros::service::waitForService("set_control_mode", ros::Duration(5));
  if (!res)
  {
    ROS_ERROR("Cannot find service named set_control_mode");
  }
  ros::Duration(2.0).sleep();
  spinal::SetControlMode set_control_mode_srv;
  set_control_mode_srv.request.is_attitude = is_attitude_ctrl_;
  set_control_mode_srv.request.is_body_rate = is_body_rate_ctrl_;
  while (!srv_set_control_mode_.call(set_control_mode_srv))
    ROS_WARN_THROTTLE(1,
                      "Waiting for set_control_mode service.... If you always see this message, the robot cannot fly.");

  ROS_INFO("Set control mode: attitude = %d and body rate = %d", set_control_mode_srv.request.is_attitude,
           set_control_mode_srv.request.is_body_rate);
}

/**
 * @brief Get the mass, gravity and the diagonal of the inertia of the Body frame from the robot model
 */
void nmpc::BaseMPC::updateInertialParams()
{
  mass_ = robot_model_->getMass();
  gravity_const_ = robot_model_->getGravity()[2];
  Eigen::Matrix3d inertia_mtx = robot_model_->getInertia<Eigen::Matrix3d>();
  inertia_.resize(3);
  inertia_[0] = inertia_mtx(0, 0);
  inertia_[1] = inertia_mtx(1, 1);
  inertia_[2] = inertia_mtx(2, 2);
}

/**
 * @brief Solve the MPC problem for the current state
 * Also measures the actual closed-loop timestep to the previous call (t_mpc_dt_), used by derived classes.
 * @param bx0 Current state, always described in the Body frame
 * @return True if the solver succeeded, false otherwise
 * NOTE: The result is stored in mpc_solver_ptr_->xo_ and uo_. If the solver fails, they still hold the previous
 * solution.
 */
bool nmpc::BaseMPC::solve(const std::vector<double>& bx0)
{
  /* Measure actual closed-loop timestep to previous solve() call */
  t_mpc_timestep_ = ros::Time::now().toSec();
  t_mpc_dt_ = t_mpc_timestep_ - t_mpc_prev_timestep_;
  t_mpc_prev_timestep_ = t_mpc_timestep_;

  /* Call solver to solve the optimization problem */
  try
  {
    mpc_solver_ptr_->solve(bx0, is_debug_);
  }
  catch (mpc_solver::AcadosSolveException& e)
  {
    ROS_ERROR_THROTTLE(1.0, "[MPC] Solver failed. Details: %s", e.what());
    return false;
  }

  return true;
}

/**
 * @brief Handle a failed control loop (solver exception or non-finite command)
 */
void nmpc::BaseMPC::handleSolverFailure()
{
  // Force land
  if ((navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE ||
       navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE ||
       navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE) &&
      !is_warmup_ && !has_triggered_force_landing_)
  {
    spinal::FlightConfigCmd flight_config_cmd;
    flight_config_cmd.cmd = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
    pub_flight_config_cmd_spinal_.publish(flight_config_cmd);
    has_triggered_force_landing_ = true;
    ROS_FATAL("[MPC] Solver failed. Triggering force landing!");
  }
}

/**
 * @brief Get the command as optimal control input from the MPC solver
 * @param idx_u Index of the control input
 * @param T_horizon Time into the horizon to interpolate the control input to, starting at 0.0
 * @return Optimal control input at the specified index and time as double
 */
double nmpc::BaseMPC::getCommand(int idx_u, double T_horizon)
{
  if (T_horizon == 0)
    return mpc_solver_ptr_->uo_.at(0).at(idx_u);

  return mpc_solver_ptr_->uo_.at(0).at(idx_u) +
         T_horizon / t_mpc_step_ * (mpc_solver_ptr_->uo_.at(1).at(idx_u) - mpc_solver_ptr_->uo_.at(0).at(idx_u));
}

/**
 * @brief Publish the time of the last solve in ms
 */
void nmpc::BaseMPC::publishSolverTime() const
{
  std_msgs::Float64 msg;
  msg.data = mpc_solver_ptr_->getSolveTime() * 1000.0;
  pub_solver_time_.publish(msg);
}

/**
 * @brief Build the ROS msg object shared for the MPC state and input with the solver dimensions
 * @param x_u Message to be initialized (zero state with unit quaternion and zero input)
 */
void nmpc::BaseMPC::initPredXU(aerial_robot_msgs::PredXU& x_u)
{
  x_u.x.layout.dim.clear();
  x_u.x.layout.dim.emplace_back();
  x_u.x.layout.dim.emplace_back();
  x_u.x.layout.dim[0].label = "horizon";
  x_u.x.layout.dim[0].size = NN_ + 1;
  x_u.x.layout.dim[0].stride = (NN_ + 1) * NX_;
  x_u.x.layout.dim[1].label = "state";
  x_u.x.layout.dim[1].size = NX_;
  x_u.x.layout.dim[1].stride = NX_;
  x_u.x.layout.data_offset = 0;
  x_u.x.data.resize((NN_ + 1) * NX_);
  std::fill(x_u.x.data.begin(), x_u.x.data.end(), 0.0);
  // quaternion
  for (int i = 6; i < (NN_ + 1) * NX_; i += NX_)
    x_u.x.data[i] = 1.0;

  x_u.u.layout.dim.clear();
  x_u.u.layout.dim.emplace_back();
  x_u.u.layout.dim.emplace_back();
  x_u.u.layout.dim[0].label = "horizon";
  x_u.u.layout.dim[0].size = NN_;
  x_u.u.layout.dim[0].stride = NN_ * NU_;
  x_u.u.layout.dim[1].label = "input";
  x_u.u.layout.dim[1].size = NU_;
  x_u.u.layout.dim[1].stride = NU_;
  x_u.u.layout.data_offset = 0;
  x_u.u.data.resize(NN_ * NU_);
  std::fill(x_u.u.data.begin(), x_u.u.data.end(), 0.0);
}

/**
 * @brief Convert the ROS msg object of the MPC state and input to the vectors used by the solver
 * @param x_u Message containing the state and input over the horizon
 * @param x_vec State vectors to be filled (NN + 1 nodes)
 * @param u_vec Input vectors to be filled (NN nodes)
 */
void nmpc::BaseMPC::rosXU2VecXU(const aerial_robot_msgs::PredXU& x_u, std::vector<std::vector<double>>& x_vec,
                                std::vector<std::vector<double>>& u_vec)
{
  if (static_cast<int>(x_u.x.layout.dim[1].stride) != NX_ || static_cast<int>(x_u.u.layout.dim[1].stride) != NU_)
    ROS_ERROR("The dimension of x_u: nx, nu is not correct!");

  if (static_cast<int>(x_u.x.layout.dim[0].size) != NN_ + 1 || static_cast<int>(x_u.u.layout.dim[0].size) != NN_)
    ROS_ERROR("The dimension of x_u: nn for x, nn for u is not correct!");

  // Convert x_u to xr_ and ur_
  for (int i = 0; i < NN_; i++)
  {
    std::copy(x_u.x.data.begin() + i * NX_, x_u.x.data.begin() + (i + 1) * NX_, x_vec[i].begin());
    std::copy(x_u.u.data.begin() + i * NU_, x_u.u.data.begin() + (i + 1) * NU_, u_vec[i].begin());
  }
  std::copy(x_u.x.data.begin() + NN_ * NX_, x_u.x.data.begin() + (NN_ + 1) * NX_, x_vec[NN_].begin());
}

/**
 * @brief Compute the reference wrench in Body frame
 * The arguments are assumed to describe T. The allocation needs the wrench acting on the Body,
 * so for T = B they are used as they are and for T = EE they are first transformed from EE to B.
 * @param ref_acc_w Linear acceleration of the origin of T, expressed in W, not including gravity
 * @param ref_quat_wt Orientation of T w.r.t. W (rotates vectors from T to W)
 * @param ref_omega_t Angular velocity of T w.r.t. W, expressed in T
 * @param ref_ang_acc_t Angular acceleration of T w.r.t. W, expressed in T
 * @return Reference wrench in B as Eigen::VectorXd(6)
 */
Eigen::VectorXd nmpc::BaseMPC::computeRefWrenchB(const tf::Vector3& ref_acc_w, const tf::Quaternion& ref_quat_wt,
                                                 const tf::Vector3& ref_omega_t,
                                                 const tf::Vector3& ref_ang_acc_t) const
{
  tf::Vector3 acc_w, omega_b, ang_acc_b;
  tf::Quaternion quat_wb;

  /* Transform the reference from the Tracking frame T to the Body frame B */
  if (!isTrackingEE())
  {
    // For T = B the reference already describes the Body
    acc_w = ref_acc_w;          // linear acceleration of the origin of B (CoG), expressed in W
    quat_wb = ref_quat_wt;      // orientation of B w.r.t. W
    omega_b = ref_omega_t;      // angular velocity, expressed in B
    ang_acc_b = ref_ang_acc_t;  // angular acceleration, expressed in B
  }
  else
  {
    // For T = EE the reference needs to be transformed into Body frame first
    // Pose of EE in B: origin ee_p_b (expressed in B) and orientation ee_q_be (rotates vectors from EE to B)
    tf::Vector3 ee_p_b;
    tf::Quaternion ee_q_be;
    getTrackingFramePose(ee_p_b, ee_q_be);
    const tf::Matrix3x3 rot_be(ee_q_be);

    // EE and B belong to the same rigid body, so they share the angular velocity and acceleration:
    // q_we = q_wb * q_be (quaternion multiplication), omega_ee = R_be^T * omega_b, alpha_ee = R_be^T * alpha_b
    quat_wb = ref_quat_wt * ee_q_be.inverse();
    omega_b = rot_be * ref_omega_t;
    ang_acc_b = rot_be * ref_ang_acc_t;

    // Acceleration of a point fixed on the body:
    // a_ee_w = a_b_w + R_wb * (alpha_b x ee_p_b + omega_b x (omega_b x ee_p_b))
    const tf::Vector3 acc_rel_b = ang_acc_b.cross(ee_p_b) + omega_b.cross(omega_b.cross(ee_p_b));
    acc_w = ref_acc_w - tf::Matrix3x3(quat_wb) * acc_rel_b;
  }

  /* Linear acceleration including gravity compensation, transformed from W to B */
  Eigen::Vector3d acc_with_g_w(acc_w.x(), acc_w.y(), acc_w.z() + gravity_const_);
  Eigen::Matrix3d rot_bw;
  tf::matrixTFToEigen(tf::Matrix3x3(quat_wb.inverse()), rot_bw);
  Eigen::Vector3d acc_with_g_b = rot_bw * acc_with_g_w;
  Eigen::Vector3d f_b = acc_with_g_b * mass_;

  /* Torque in B */
  // I_b * alpha_b = tau_b - omega_b x (I_b * omega_b)
  const tf::Vector3 inertia_diag(inertia_.at(0), inertia_.at(1), inertia_.at(2));
  const tf::Vector3 tau_b = inertia_diag * ang_acc_b + omega_b.cross(inertia_diag * omega_b);

  /* Reference wrench in B */
  Eigen::VectorXd ref_wrench_b(6);
  ref_wrench_b(0) = f_b(0);
  ref_wrench_b(1) = f_b(1);
  ref_wrench_b(2) = f_b(2);
  ref_wrench_b(3) = tau_b.x();
  ref_wrench_b(4) = tau_b.y();
  ref_wrench_b(5) = tau_b.z();
  return ref_wrench_b;
}

/**
 * @brief Update the pose of the Tracking frame T in the Body frame B from the robot model
 * The pose of EE in B depends on the joint angles. It is read once per control loop and cached, so that all
 * transformations within one loop use the same pose.
 */
void nmpc::BaseMPC::updateTrackingFramePose()
{
  if (!isTrackingEE())
    // For T = B, the pose stays the identity.
    return;

  std::vector<double> ee_p, ee_q;
  robot_model_->getCoGtoFramePosQuat("ee_contact", ee_p, ee_q);
    
  // The robot model locks the frames, since they are updated by the joint states in another thread
  std::lock_guard<std::mutex> lock(t_pose_mutex_);
  t_p_b_.setValue(ee_p[0], ee_p[1], ee_p[2]);
  q_bt_.setValue(ee_q[1], ee_q[2], ee_q[3], ee_q[0]);
}

/**
 * @brief Get the cached pose of the Tracking frame T in the Body frame B
 * @param t_p_b Origin of T, expressed in B (output)
 * @param q_bt Orientation of T w.r.t. B, rotates vectors from T to B (output)
 */
void nmpc::BaseMPC::getTrackingFramePose(tf::Vector3& t_p_b, tf::Quaternion& q_bt) const
{
  std::lock_guard<std::mutex> lock(t_pose_mutex_);
  t_p_b = t_p_b_;
  q_bt = q_bt_;
}

/**
 * @brief Transform a state of the Body frame B to the Tracking frame T
 * B and T belong to the same rigid body. With the pose t_p_b, q_bt of T in B:
 * q_wt = q_wb * q_bt, omega_t = R_bt^T * omega_b,
 * t_pos_w = pos_w + R_wb * t_p_b, t_vel_w = vel_w + R_wb * (omega_b x t_p_b)
 * NOTE: The quaternion product keeps the sign of q_wb, so the continuity of q_wb carries over to q_wt.
 * NOTE: Inputs and outputs may be the same objects. For T = B, the state is copied.
 * @param pos_w Position of the origin of B, expressed in W
 * @param vel_w Linear velocity of the origin of B, expressed in W
 * @param q_wb Orientation of B w.r.t. W
 * @param omega_b Angular velocity of B w.r.t. W, expressed in B
 * @param t_pos_w Position of the origin of T, expressed in W (output)
 * @param t_vel_w Linear velocity of the origin of T, expressed in W (output)
 * @param q_wt Orientation of T w.r.t. W (output)
 * @param omega_t Angular velocity of T w.r.t. W, expressed in T (output)
 */
void nmpc::BaseMPC::convertFromBToT(const tf::Vector3& pos_w, const tf::Vector3& vel_w, const tf::Quaternion& q_wb,
                                    const tf::Vector3& omega_b, tf::Vector3& t_pos_w, tf::Vector3& t_vel_w,
                                    tf::Quaternion& q_wt, tf::Vector3& omega_t) const
{
  if (!isTrackingEE())
  {
    t_pos_w = pos_w;
    t_vel_w = vel_w;
    q_wt = q_wb;
    omega_t = omega_b;
    return;
  }

  tf::Vector3 t_p_b;
  tf::Quaternion q_bt;
  getTrackingFramePose(t_p_b, q_bt);

  // Compute into locals first, so that inputs and outputs may be the same objects
  const tf::Matrix3x3 rot_wb(q_wb);
  const tf::Quaternion out_q_wt = q_wb * q_bt;
  const tf::Vector3 out_omega_t = tf::Matrix3x3(q_bt).transpose() * omega_b;
  const tf::Vector3 out_pos_w = pos_w + rot_wb * t_p_b;
  const tf::Vector3 out_vel_w = vel_w + rot_wb * omega_b.cross(t_p_b);

  t_pos_w = out_pos_w;
  t_vel_w = out_vel_w;
  q_wt = out_q_wt;
  omega_t = out_omega_t;
}

/**
 * @brief Check if a reference describes the Tracking frame
 * The publisher of a reference sets the name of the described robot frame ("cog" or "ee_contact") as header.frame_id.
 * @param frame_id Name of the robot frame described by the reference
 * @return True if the reference describes the Tracking frame, false otherwise
 */
bool nmpc::BaseMPC::isRefInTrackingFrame(const std::string& frame_id) const
{
  if (frame_id == getTrackingFrameName())
    return true;

  ROS_ERROR_THROTTLE(1.0, "[MPC] The reference describes the frame '%s', but the MPC tracks the frame '%s'! Ignoring "
                     "the reference.",
                     frame_id.c_str(), getTrackingFrameName().c_str());
  return false;
}

/**
 * @brief Transform a state of the Tracking frame T back to the Body frame B
 * Exact inverse of convertFromBToT(). With the pose t_p_b, q_bt of T in B:
 * q_wb = q_wt * q_bt^-1, omega_b = R_bt * omega_t,
 * pos_w = t_pos_w - R_wb * t_p_b, vel_w = t_vel_w - R_wb * (omega_b x t_p_b)
 * NOTE: Inputs and outputs may be the same objects. For T = B, the state is copied.
 * @param t_pos_w Position of the origin of T, expressed in W
 * @param t_vel_w Linear velocity of the origin of T, expressed in W
 * @param q_wt Orientation of T w.r.t. W
 * @param omega_t Angular velocity of T w.r.t. W, expressed in T
 * @param pos_w Position of the origin of B, expressed in W (output)
 * @param vel_w Linear velocity of the origin of B, expressed in W (output)
 * @param q_wb Orientation of B w.r.t. W (output)
 * @param omega_b Angular velocity of B w.r.t. W, expressed in B (output)
 */
void nmpc::BaseMPC::convertFromTToB(const tf::Vector3& t_pos_w, const tf::Vector3& t_vel_w, const tf::Quaternion& q_wt,
                                    const tf::Vector3& omega_t, tf::Vector3& pos_w, tf::Vector3& vel_w,
                                    tf::Quaternion& q_wb, tf::Vector3& omega_b) const
{
  if (!isTrackingEE())
  {
    pos_w = t_pos_w;
    vel_w = t_vel_w;
    q_wb = q_wt;
    omega_b = omega_t;
    return;
  }

  tf::Vector3 t_p_b;
  tf::Quaternion q_bt;
  getTrackingFramePose(t_p_b, q_bt);

  // Compute into locals first, so that inputs and outputs may be the same objects
  const tf::Quaternion out_q_wb = q_wt * q_bt.inverse();
  const tf::Vector3 out_omega_b = tf::Matrix3x3(q_bt) * omega_t;
  const tf::Matrix3x3 rot_wb(out_q_wb);
  const tf::Vector3 out_pos_w = t_pos_w - rot_wb * t_p_b;
  const tf::Vector3 out_vel_w = t_vel_w - rot_wb * out_omega_b.cross(t_p_b);

  pos_w = out_pos_w;
  vel_w = out_vel_w;
  q_wb = out_q_wb;
  omega_b = out_omega_b;
}

/**
 * @brief Publish the reference and predicted trajectory for visualization in RViz
 * NOTE: The reference poses describe the Tracking frame T, the predicted poses the Body frame B.
 * @param event Timer event
 */
void nmpc::BaseMPC::callbackViz(const ros::TimerEvent& event)
{
  /* Convert from mpc_solver_ptr_->x_u_out to PoseArray */
  geometry_msgs::PoseArray pred_poses;
  geometry_msgs::PoseArray ref_poses;

  for (int i = 0; i < NN_ + 1; ++i)
  {
    geometry_msgs::Pose pred_pose;
    pred_pose.position.x = mpc_solver_ptr_->xo_[i][0];
    pred_pose.position.y = mpc_solver_ptr_->xo_[i][1];
    pred_pose.position.z = mpc_solver_ptr_->xo_[i][2];
    pred_pose.orientation.w = mpc_solver_ptr_->xo_[i][6];
    pred_pose.orientation.x = mpc_solver_ptr_->xo_[i][7];
    pred_pose.orientation.y = mpc_solver_ptr_->xo_[i][8];
    pred_pose.orientation.z = mpc_solver_ptr_->xo_[i][9];
    pred_poses.poses.push_back(pred_pose);

    geometry_msgs::Pose ref_pose;
    ref_pose.position.x = mpc_solver_ptr_->xr_[i][0];
    ref_pose.position.y = mpc_solver_ptr_->xr_[i][1];
    ref_pose.position.z = mpc_solver_ptr_->xr_[i][2];
    ref_pose.orientation.w = mpc_solver_ptr_->xr_[i][6];
    ref_pose.orientation.x = mpc_solver_ptr_->xr_[i][7];
    ref_pose.orientation.y = mpc_solver_ptr_->xr_[i][8];
    ref_pose.orientation.z = mpc_solver_ptr_->xr_[i][9];
    ref_poses.poses.push_back(ref_pose);
  }

  pred_poses.header.frame_id = "world";
  pred_poses.header.stamp = ros::Time::now();
  pub_viz_pred_.publish(pred_poses);

  ref_poses.header.frame_id = "world";
  ref_poses.header.stamp = ros::Time::now();
  pub_viz_ref_.publish(ref_poses);
}

/**
 * @brief Ensure the continuity of a quaternion w.r.t. the previous one by preventing sign flips
 * This is quite important for dataset recordings.
 * @param quat Quaternion to be checked and flipped if necessary
 * @param quat_prev Previous quaternion, updated to quat
 */
void nmpc::BaseMPC::ensureQuaternionContinuity(tf::Quaternion& quat, tf::Quaternion& quat_prev)
{
  double qe_c_w =
      quat.w() * quat_prev.w() + quat.x() * quat_prev.x() + quat.y() * quat_prev.y() + quat.z() * quat_prev.z();
  if (qe_c_w < 0)
  {
    quat = quat.operator-();
  }
  quat_prev = quat;
}

/**
 * @brief Check if two MultiDOFJointTrajectoryPoint messages are equal
 * @param a First MultiDOFJointTrajectoryPoint message
 * @param b Second MultiDOFJointTrajectoryPoint message
 * @param if_compare_time Whether to compare the time from start
 * @param epsilon The tolerance for the comparison
 * @return True if the messages are equal, false otherwise
 */
bool nmpc::BaseMPC::isMulDOFJointTrajPtEqual(const trajectory_msgs::MultiDOFJointTrajectoryPoint& a,
                                             const trajectory_msgs::MultiDOFJointTrajectoryPoint& b,
                                             bool if_compare_time, double epsilon)
{
  if (a.transforms.size() != b.transforms.size() || a.velocities.size() != b.velocities.size() ||
      a.accelerations.size() != b.accelerations.size())
    return false;

  for (size_t i = 0; i < a.transforms.size(); ++i)
  {
    const auto& ta = a.transforms[i];
    const auto& tb = b.transforms[i];
    if (!isAlmostEqual(ta.translation.x, tb.translation.x, epsilon) ||
        !isAlmostEqual(ta.translation.y, tb.translation.y, epsilon) ||
        !isAlmostEqual(ta.translation.z, tb.translation.z, epsilon) ||
        !isAlmostEqual(ta.rotation.x, tb.rotation.x, epsilon) ||
        !isAlmostEqual(ta.rotation.y, tb.rotation.y, epsilon) ||
        !isAlmostEqual(ta.rotation.z, tb.rotation.z, epsilon) || !isAlmostEqual(ta.rotation.w, tb.rotation.w, epsilon))
      return false;
  }

  for (size_t i = 0; i < a.velocities.size(); ++i)
  {
    const auto& va = a.velocities[i];
    const auto& vb = b.velocities[i];
    if (!isAlmostEqual(va.linear.x, vb.linear.x, epsilon) || !isAlmostEqual(va.linear.y, vb.linear.y, epsilon) ||
        !isAlmostEqual(va.linear.z, vb.linear.z, epsilon) || !isAlmostEqual(va.angular.x, vb.angular.x, epsilon) ||
        !isAlmostEqual(va.angular.y, vb.angular.y, epsilon) || !isAlmostEqual(va.angular.z, vb.angular.z, epsilon))
      return false;
  }

  for (size_t i = 0; i < a.accelerations.size(); ++i)
  {
    const auto& aa = a.accelerations[i];
    const auto& ab = b.accelerations[i];
    if (!isAlmostEqual(aa.linear.x, ab.linear.x, epsilon) || !isAlmostEqual(aa.linear.y, ab.linear.y, epsilon) ||
        !isAlmostEqual(aa.linear.z, ab.linear.z, epsilon) || !isAlmostEqual(aa.angular.x, ab.angular.x, epsilon) ||
        !isAlmostEqual(aa.angular.y, ab.angular.y, epsilon) || !isAlmostEqual(aa.angular.z, ab.angular.z, epsilon))
      return false;
  }

  // ROS duration comparison
  if (if_compare_time)
  {
    if (!isAlmostEqual(a.time_from_start.toSec(), b.time_from_start.toSec(), epsilon))
      return false;
  }

  return true;
}

/**
 * @brief Print the physical parameters of the robot model
 */
void nmpc::BaseMPC::printPhysicalParams() const
{
  std::cout << "mass: " << robot_model_->getMass() << std::endl;
  std::cout << "gravity: " << robot_model_->getGravity() << std::endl;
  std::cout << "inertia: " << robot_model_->getInertia<Eigen::Matrix3d>() << std::endl;
  std::cout << "rotor num: " << robot_model_->getRotorNum() << std::endl;
  for (const auto& dir : robot_model_->getRotorDirection())
  {
    std::cout << "Key: " << dir.first << ", Value: " << dir.second << std::endl;
  }
  for (const auto& vec : robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>())
  {
    std::cout << "rotor origin from cog: " << vec << std::endl;
  }
  std::cout << "thrust lower limit: " << robot_model_->getThrustLowerLimit() << std::endl;
  std::cout << "thrust upper limit: " << robot_model_->getThrustUpperLimit() << std::endl;

  std::cout << "kq_kt_rate" << robot_model_->getMFRate() << std::endl;
  std::cout << "abs(kq_kt_rate)" << std::abs(robot_model_->getMFRate()) << std::endl;
}
