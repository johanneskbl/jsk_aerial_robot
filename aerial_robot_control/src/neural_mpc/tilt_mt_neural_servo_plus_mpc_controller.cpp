//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/neural_mpc/tilt_mt_neural_servo_plus_mpc_controller.h"

using namespace aerial_robot_control;

namespace data_utils
{
  std::vector<int> parseIntArray(const std::string& s) {
    // Expected input:
    // std::string "[1, 2, 3]"
    std::vector<int> result;
    std::string trimmed = s.substr(1, s.size() - 2); // strip [ and ]
    std::stringstream ss(trimmed);
    std::string token;
    while (std::getline(ss, token, ',')) {
        result.push_back(std::stoi(token));
    }
    return result;
  }
}  // namespace data_utils

void nmpc::TiltMtNeuralServoPlusMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                double ctrl_loop_du)
{
  BaseMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);
  
  /* init plugins */
  initPlugins();

  /* init neural model and load its metadata */
  initNeuralModel();

  /* init general parameters */
  initGeneralParams();

  /* init cost weight parameters */
  initMPCCostW();

  /* init constraints */
  initMPCConstraints();

  /* init dynamic reconfigure */
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle mpc_nh(control_nh, "nmpc");
  mpc_reconf_servers_.push_back(boost::make_shared<MPCControlDynamicConfig>(mpc_nh));
  mpc_reconf_servers_.back()->setCallback(boost::bind(&TiltMtNeuralServoPlusMPC::cfgMPCCallback, this, _1, _2));

  /* set ROS parameters */
  mpc_nh.setParam("NN", NN_);
  mpc_nh.setParam("NX", NX_);
  mpc_nh.setParam("NU", NU_);

  /* timers */
  tmr_viz_ = nh_.createTimer(ros::Duration(0.05), &TiltMtNeuralServoPlusMPC::callbackViz, this);

  /* publishers */
  pub_record_curr_ = nh_.advertise<aerial_robot_msgs::MPCState>("nmpc/record_curr", 1);
  pub_record_ref_ = nh_.advertise<aerial_robot_msgs::MPCTrajectory>("nmpc/record_ref", 1);
  pub_record_pred_ = nh_.advertise<aerial_robot_msgs::MPCTrajectory>("nmpc/record_pred", 1);
  pub_viz_ref_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_ref", 1);
  pub_viz_pred_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_pred", 1);
  pub_flight_cmd_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  pub_gimbal_control_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  pub_flight_config_cmd_spinal_ = nh_.advertise<spinal::FlightConfigCmd>("flight_config_cmd", 1);

  /* services */
  srv_set_control_mode_ = nh_.serviceClient<spinal::SetControlMode>("set_control_mode");

  /* subscribers */
  sub_joint_states_ = nh_.subscribe("joint_states", 5, &TiltMtNeuralServoPlusMPC::callbackJointStates, this);
  sub_set_rpy_ = nh_.subscribe("set_rpy", 5, &TiltMtNeuralServoPlusMPC::callbackSetRPY, this);
  sub_set_ref_x_u_ = nh_.subscribe("set_ref_x_u", 5, &TiltMtNeuralServoPlusMPC::callbackSetRefXU, this);
  sub_set_traj_ = nh_.subscribe("set_ref_traj", 5, &TiltMtNeuralServoPlusMPC::callbackSetRefTraj, this);
  sub_set_fixed_rotor_ = nh_.subscribe("set_fixed_rotor", 5, &TiltMtNeuralServoPlusMPC::callbackSetFixedRotor, this);

  /* init some values */
  setControlMode();

  initActuatorStates();
  initPredXU(x_u_ref_);

  quat_prev_.setW(1.0);

  reset();
  ROS_INFO("[CONTROL] MPC Controller initialized!");
}

void nmpc::TiltMtNeuralServoPlusMPC::activate()
{
  ControlBase::activate();

  initAllocMat();
  updateInertialParams();
  initMPCParams();

  if (is_print_phys_params_)
    printPhysicalParams();

  // make takeoff slow
  modifyVelConstraints(-vel_limit_takeoff_, vel_limit_takeoff_);
  has_restored_vel_ = false;  // reset the flag, so that we can restore the velocity after hovering

  /* also for some commands that should be sent after takeoff */
  // enable imu sending, only works in simulation. TODO: check its compatibility with real robot
  spinal::FlightConfigCmd flight_config_cmd;
  flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
  pub_flight_config_cmd_spinal_.publish(flight_config_cmd);
}

bool nmpc::TiltMtNeuralServoPlusMPC::update()
{
  if (!ControlBase::update())
  {
    // After press activate button, but before takeoff
    // Warm-up the solver & network (if any) before actual takeoff
    if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE)
      controlCore(true);
    return false;
  }
  else
  {
    /* Runtime analysis
    Nominal:
        - controlCore: 1.2-2.0 ms
    Neural (neuralmodel_185):
        - controlCore: 10.0-10.8 ms
    Neural (neuralmodel_185 & linearized order 1):
        - controlCore: 2.0-3.0 ms
            - construction: 0.00 ms
            - convert: 0.03 ms
            - forward pass: 0.13-0.2 ms (peak 0.7 ms)
            - linearization: 0.11-0.2 ms
            - flatten: 0.03-0.06 ms
            - set params: 0.001 ms
    Auxiliary:
        - sendCmd: 0.01 ms
        - recording: 0.02 ms
    */
    double start_time = ros::WallTime::now().toSec();
    controlCore();
    double core_time = ros::WallTime::now().toSec();
    ROS_INFO_THROTTLE(1.0, "[MPC] Runtime - controlCore(): %.3f ms", (core_time - start_time) * 1000.0);
    
    sendCmd();

    // *** RECORDING ***
    // Only needed to record dataset but else its computationally expensive, so we can comment it out when not needed.
    publishRecording();
  }

  return true;
}

void nmpc::TiltMtNeuralServoPlusMPC::reset()
{
  ControlBase::reset();

  resetPlugins();

  // reset x_u_ref_
  std::vector<double> x_vec_ee = meas2VecX(true);
  std::vector<double> u_vec(NU_, 0);

  for (int i = 0; i < NN_; i++)
  {
    std::copy(x_vec_ee.begin(), x_vec_ee.begin() + NX_, x_u_ref_.x.data.begin() + NX_ * i);
    std::copy(u_vec.begin(), u_vec.begin() + NU_, x_u_ref_.u.data.begin() + NU_ * i);
  }
  std::copy(x_vec_ee.begin(), x_vec_ee.begin() + NX_, x_u_ref_.x.data.begin() + NX_ * NN_);

  mpc_solver_ptr_->resetXrUrByX0U0(x_vec_ee, u_vec);
  
  // reset solver & set initial guess to hovering
  std::vector<double> x_vec = meas2VecX();
  std::vector<double> u_hover(NU_, 0.0);
  for (int i = 0; i < motor_num_ && i < static_cast<int>(u_hover.size()); ++i)
    u_hover[i] = 8.0;  // in [N]
  mpc_solver_ptr_->resetSolverByX0U0(x_vec, u_hover);

  /* reset control input */
  flight_cmd_.base_thrust = std::vector<float>(motor_num_, 0.0);

  gimbal_ctrl_cmd_.name.clear();
  gimbal_ctrl_cmd_.position.clear();
  for (int i = 0; i < joint_num_; i++)
  {
    gimbal_ctrl_cmd_.name.emplace_back("gimbal" + std::to_string(i + 1));
    gimbal_ctrl_cmd_.position.push_back(0.0);
  }

  pub_gimbal_control_.publish(gimbal_ctrl_cmd_);
}

void nmpc::TiltMtNeuralServoPlusMPC::initGeneralParams()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle mpc_nh(control_nh, "nmpc");
  ros::NodeHandle physical_nh(nh_, "physical");
  ros::NodeHandle alloc_nh(control_nh, "alloc");

  getParam<int>(alloc_nh, "type", alloc_type_, 0);
  getParam<double>(alloc_nh, "ft_thresh", ft_thresh_, 0.5);
  if (ft_thresh_ <= 0.0)
    throw std::runtime_error(
        "ft_thresh must be greater than zero! Please set a positive value for ft_thresh in the parameter server.");

  getParam<int>(physical_nh, "num_servos", joint_num_, 0);
  getParam<double>(physical_nh, "t_servo", t_servo_, 0.01);
  getParam<int>(physical_nh, "num_rotors", motor_num_, 0);
  getParam<double>(physical_nh, "t_rotor", t_rotor_, 0.01);

  getParam<double>(mpc_nh, "T_samp", t_mpc_samp_, 0.025);
  getParam<double>(mpc_nh, "T_step", t_mpc_step_, 0.1);
  getParam<double>(mpc_nh, "T_horizon", t_mpc_horizon_, 2.0);

  getParam<bool>(mpc_nh, "is_attitude_ctrl", is_attitude_ctrl_, true);
  getParam<bool>(mpc_nh, "is_body_rate_ctrl", is_body_rate_ctrl_, false);
  getParam<bool>(mpc_nh, "is_print_phys_params", is_print_phys_params_, false);
  getParam<bool>(mpc_nh, "is_debug", is_debug_, false);

  if (is_debug_)
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
}

void nmpc::TiltMtNeuralServoPlusMPC::initNeuralModel()
{
  ROS_INFO("==========================");
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle mpc_nh(control_nh, "nmpc");
  results_dir_ = ros::package::getPath("aerial_robot_control") + "/scripts/neural_mpc/results/model_fitting/";
  metadata_json_path_ = results_dir_ + "metadata.json";
  
  try
  {
    std::ifstream metadata_json_(metadata_json_path_);
    metadata_ = json::parse(metadata_json_);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("[NEURAL][MPC] Failed to load neural model metadata from '%s'", metadata_json_path_.c_str());
    linearize_mlp_ = false;
    return;
  }
  
  // Extract neural model metadata
  // NOTE: At build time, the solver is rebuilt with options from scripts/neural_mpc/config/configurations.py
  // These options are written AT BUILD TIME to metadata.json in Python, and we read them here AT RUNTIME in C++
  try
  {
    // General
    neural_model_name_ = metadata_["runtime_options"]["model_name"];
    neural_model_instance_ = metadata_["runtime_options"]["model_instance"];
    
    linearize_mlp_ = metadata_["runtime_options"]["linearize_mlp"];
    linearize_order_ = metadata_["runtime_options"]["linearize_order"];
    linearize_start_idx_ = metadata_["runtime_options"]["linearize_start_idx"];
    linearize_end_idx_ = metadata_["runtime_options"]["linearize_end_idx"];
    use_gpu_ = metadata_["runtime_options"]["use_gpu"];

    const auto& cfg_ = metadata_[neural_model_name_][neural_model_instance_]["ModelFitConfig"];
    state_feats_ = data_utils::parseIntArray(cfg_["state_feats"]);
    u_feats_ = data_utils::parseIntArray(cfg_["u_feats"]);
    y_reg_dims_ = data_utils::parseIntArray(cfg_["y_reg_dims"]);
    input_transform_ = cfg_["input_transform"];
    batch_size_ = static_cast<int64_t>(NN_ + 1);
    N_in_ = static_cast<int64_t>(state_feats_.size() + u_feats_.size());
    N_out_ = static_cast<int64_t>(y_reg_dims_.size());

    // Linearization
    if (linearize_mlp_)
    {
      num_lin_params_ = linearize_end_idx_ - linearize_start_idx_;
      lin_param_idx_.resize(num_lin_params_);
      lin_params_.resize(num_lin_params_, 0.0);
      std::iota(lin_param_idx_.begin(), lin_param_idx_.end(), linearize_start_idx_);

      x0_vec_f32_.resize(static_cast<size_t>(batch_size_) * N_in_,  0.0f);
      x0_vec_.resize(static_cast<size_t>(batch_size_) * N_in_,  0.0);
      y0_vec_.resize(static_cast<size_t>(batch_size_) * N_out_, 0.0);
      J0_vec_.resize(static_cast<size_t>(batch_size_) * N_out_ * N_in_, 0.0);
      H0_vec_.resize(static_cast<size_t>(batch_size_) * N_out_ * N_in_ * N_in_, 0.0);
      off_x0_ = 0;
      off_y0_ = off_x0_ + static_cast<size_t>(N_in_);
      off_J0_ = off_y0_ + static_cast<size_t>(N_out_);
      off_H0_ = off_J0_ + static_cast<size_t>(N_out_) * N_in_;
      stride_x0_ = static_cast<size_t>(N_in_);
      stride_y0_ = static_cast<size_t>(N_out_);
      stride_J0_ = static_cast<size_t>(N_out_) * static_cast<size_t>(N_in_);
      stride_H0_ = static_cast<size_t>(N_out_) * static_cast<size_t>(N_in_) * static_cast<size_t>(N_in_);

      if (off_H0_ + (linearize_order_ >= 2 ? stride_H0_ : 0) != static_cast<size_t>(num_lin_params_))
      {
        ROS_ERROR("[NEURAL][MPC] Mismatch of linearization parameters size. Expected %zu but got %zu. Disabling linearization.", 
                  off_H0_ + (linearize_order_ >= 2 ? stride_H0_ : 0), static_cast<size_t>(num_lin_params_));
        linearize_mlp_ = false;
      }
    }

    // Temporal model
    delay_horizon_ = metadata_[neural_model_name_][neural_model_instance_]["NetworkConfig"]["delay_horizon"];

    ROS_INFO("[NEURAL][MPC] Using neural model %s/%s.", neural_model_name_.c_str(), neural_model_instance_.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("[NEURAL][MPC] Invalid/unsupported metadata format for %s/%s: %s", neural_model_name_.c_str(),
              neural_model_instance_.c_str(), e.what());
    linearize_mlp_ = false;
  }

  // Load neural model using LibTorch
  if (linearize_mlp_)
  {
    try
    {
      if (use_gpu_)
      {
        if (torch::cuda::is_available())
        {
          device_mlp_ = torch::Device(torch::kCUDA);
        }
        else
        {
          ROS_WARN("[NEURAL][MPC] GPU requested but CUDA is not available. Falling back to CPU.");
        }
      }

      torch::jit::script::Module neural_model_ = torch::jit::load(results_dir_ + neural_model_name_ + "/" + neural_model_instance_ + "_scripted.pt", device_mlp_);
      neural_model_.eval();
      neural_module_ = std::make_shared<torch::jit::script::Module>(std::move(neural_model_));
      ROS_INFO("[NEURAL][MPC] Successfully loaded neural model!");
      ROS_INFO("[NEURAL][MPC] Linearization is ENABLED with order %d", linearize_order_);
      if (input_transform_)
        ROS_INFO("[NEURAL][MPC] Input transform is ENABLED.");
      else
        ROS_INFO("[NEURAL][MPC] Input transform is DISABLED.");
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[NEURAL][MPC] Error loading the neural model: %s. Disabling linearization. Exception: %s", neural_model_instance_.c_str(), e.what());
      linearize_mlp_ = false;
    }
  }
  else
  {
    ROS_INFO("[NEURAL][MPC] Linearization is DISABLED.");
  }

  // Temporal model
  if (delay_horizon_ > 0)
  {
    ROS_INFO("[NEURAL][MPC] Loaded a temporal model (delay_horizon = %d).", delay_horizon_);
  }

  if (linearize_mlp_)
  {
    if (linearize_start_idx_ < 0 || linearize_end_idx_ < 0 || linearize_end_idx_ <= linearize_start_idx_ || linearize_end_idx_ > NP_)
    {
      ROS_ERROR("[NEURAL][MPC] Invalid linearization index range [%d,%d). Disabling linearization.", linearize_start_idx_,
                linearize_end_idx_);
      linearize_mlp_ = false;
    }
    if (linearize_order_ < 1 || linearize_order_ > 2)
    {
      ROS_ERROR("[NEURAL][MPC] Invalid linearization order %d. Only 1 (Jacobian) or 2 (Hessian) are supported. Disabling linearization.",
                linearize_order_);
      linearize_mlp_ = false;
    }
  }
  ROS_INFO("==========================");
}

void nmpc::TiltMtNeuralServoPlusMPC::initMPCCostW()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle mpc_nh(control_nh, "nmpc");

  /* control parameters with dynamic reconfigure */
  double Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Rt, Rac_d;
  getParam<double>(mpc_nh, "Qp_xy", Qp_xy, 300);
  getParam<double>(mpc_nh, "Qp_z", Qp_z, 400);
  getParam<double>(mpc_nh, "Qv_xy", Qv_xy, 10);
  getParam<double>(mpc_nh, "Qv_z", Qv_z, 10);
  getParam<double>(mpc_nh, "Qq_xy", Qq_xy, 300);
  getParam<double>(mpc_nh, "Qq_z", Qq_z, 300);
  getParam<double>(mpc_nh, "Qw_xy", Qw_xy, 5);
  getParam<double>(mpc_nh, "Qw_z", Qw_z, 5);
  getParam<double>(mpc_nh, "Qa", Qa, 1);
  getParam<double>(mpc_nh, "Rt", Rt, 1);
  getParam<double>(mpc_nh, "Rac_d", Rac_d, 250);

  // diagonal matrix
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
  for (int i = NX_; i < NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rt, false);
  for (int i = NX_ + motor_num_; i < NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);
}

void nmpc::TiltMtNeuralServoPlusMPC::initMPCConstraints()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle mpc_nh(control_nh, "nmpc");

  double body_rate_max, body_rate_min;
  getParam<double>(mpc_nh, "w_max", body_rate_max, 6.0);
  getParam<double>(mpc_nh, "w_min", body_rate_min, -6.0);
  getParam<double>(mpc_nh, "v_max", vel_max_, 1.0);
  getParam<double>(mpc_nh, "v_min", vel_min_, -1.0);
  getParam<double>(mpc_nh, "thrust_max", thrust_ctrl_max_, 0.0);
  getParam<double>(mpc_nh, "thrust_min", thrust_ctrl_min_, 0.0);
  getParam<double>(mpc_nh, "a_max", servo_angle_max_, 3.1416);
  getParam<double>(mpc_nh, "a_min", servo_angle_min_, -3.1416);

  //  TODO: this should be set in flight_navigation; don't know why set 0.2 results solver failure
  getParam<double>(mpc_nh, "vel_limit_takeoff", vel_limit_takeoff_, 1.0);  // m/s

  // lbx and ubx
  std::vector<int> idxbx = mpc_solver_ptr_->getConstraintsIdxbx();
  std::vector<int> idxbx_desired = { 3, 4, 5, 10, 11, 12 };
  idxbx_desired.resize(6 + joint_num_);
  for (int i = 0; i < joint_num_; i++)
  {
    idxbx_desired[6 + i] = 13 + i;
  }
  if (idxbx.size() != idxbx_desired.size() || !std::equal(idxbx.begin(), idxbx.end(), idxbx_desired.begin()))
  {
    ROS_ERROR("[MPC] idxbx is not equal to idxbx_desired, we cannot set constraints lbx and ubx!");
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
    ROS_ERROR("[MPC] idxbx_end is not equal to idxbx_end_desired, we cannot set constraints lbxe and ubxe!");
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
    ROS_ERROR("[MPC] idxbu is not equal to idxbu_desired, we cannot set constraints lbu and ubu!");
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

void nmpc::TiltMtNeuralServoPlusMPC::setControlMode()
{
  bool res = ros::service::waitForService("set_control_mode", ros::Duration(5));
  if (!res)
  {
    ROS_ERROR("[MPC] cannot find service named set_control_mode");
  }
  ros::Duration(2.0).sleep();
  spinal::SetControlMode set_control_mode_srv;
  set_control_mode_srv.request.is_attitude = is_attitude_ctrl_;
  set_control_mode_srv.request.is_body_rate = is_body_rate_ctrl_;
  while (!srv_set_control_mode_.call(set_control_mode_srv))
    ROS_WARN_THROTTLE(1,
                      "[MPC] Waiting for set_control_mode service.... If you always see this message, the robot cannot fly.");

  ROS_INFO("[MPC] Set control mode: attitude = %d and body rate = %d", set_control_mode_srv.request.is_attitude,
           set_control_mode_srv.request.is_body_rate);
}

void nmpc::TiltMtNeuralServoPlusMPC::initAllocMat()
{
  /* get physical param */
  int rotor_num = robot_model_->getRotorNum();  // For tilt-rotor, rotor_num = servo_num
  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const map<int, int> rotor_dr = robot_model_->getRotorDirection();
  double kq_d_kt = abs(robot_model_->getMFRate());  // PAY ATTENTION: should be positive value

  /* alloc mat */
  alloc_mat_.resize(0, 0);
  alloc_mat_pinv_.resize(0, 0);

  // construct alloc_mat_
  alloc_mat_ = Eigen::MatrixXd::Zero(6, 2 * rotor_num);

  for (int i = 0; i < rotor_num; i++)
  {
    Eigen::Vector3d p_b = rotor_p[i];
    int dr = rotor_dr.find(i + 1)->second;  // PAY ATTENTION: the rotor index starts from 1!!!!!!!!!!!!!!!!!!!!!

    double sqrt_p_xy = sqrt(p_b.x() * p_b.x() + p_b.y() * p_b.y());

    // - force
    alloc_mat_(0, 2 * i) = p_b.y() / sqrt_p_xy;
    alloc_mat_(1, 2 * i) = -p_b.x() / sqrt_p_xy;
    alloc_mat_(2, 2 * i + 1) = 1;

    // - torque
    alloc_mat_(3, 2 * i) = -dr * kq_d_kt * p_b.y() / sqrt_p_xy + p_b.x() * p_b.z() / sqrt_p_xy;
    alloc_mat_(4, 2 * i) = dr * kq_d_kt * p_b.x() / sqrt_p_xy + p_b.y() * p_b.z() / sqrt_p_xy;
    alloc_mat_(5, 2 * i) = -p_b.x() * p_b.x() / sqrt_p_xy - p_b.y() * p_b.y() / sqrt_p_xy;

    alloc_mat_(3, 2 * i + 1) = p_b.y();
    alloc_mat_(4, 2 * i + 1) = -p_b.x();
    alloc_mat_(5, 2 * i + 1) = -dr * kq_d_kt;
  }

  alloc_mat_pinv_ = aerial_robot_model::pseudoinverse(alloc_mat_);
}

void nmpc::TiltMtNeuralServoPlusMPC::initMPCParams()
{
  /* construct acados parameters */
  std::vector<double> acados_p(NP_, 0.0);

  acados_p[0] = 1.0;  // qw
  idx_p_quat_end_ = 3;

  int idx;
  // TODO: this condition is temporary for drones that don't pass in phys param (bi, tri, fix-qd)
  if (NP_ > 4 + 6)
  {
    ROS_INFO("[MPC] Set physical parameters for MPC solver");

    std::vector<double> phys_p = PhysToMPCParams();
    std::copy(phys_p.begin(), phys_p.end(), acados_p.begin() + idx_p_quat_end_ + 1);
    idx = idx_p_quat_end_ + phys_p.size();
  }
  else
  {
    idx = idx_p_quat_end_;
  }

  // set idx_p_phys_end_ for setting other parameters later
  idx_p_phys_end_ = idx;

  /* set acados parameters */
  mpc_solver_ptr_->setParameters(acados_p);
}

void nmpc::TiltMtNeuralServoPlusMPC::updateInertialParams()
{
  mass_ = robot_model_->getMass();
  gravity_const_ = robot_model_->getGravity()[2];
  Eigen::Matrix3d inertia_mtx = robot_model_->getInertia<Eigen::Matrix3d>();
  inertia_.resize(3);
  inertia_[0] = inertia_mtx(0, 0);
  inertia_[1] = inertia_mtx(1, 1);
  inertia_[2] = inertia_mtx(2, 2);
}

void nmpc::TiltMtNeuralServoPlusMPC::modifyVelConstraints(double vel_min, double vel_max) const
{
  // Hardcoded: the vel idx is 3,4,5, which are the first three elements. TODO: consider to make it more general

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

  ROS_INFO("[MPC] Velocity constraints modified: [%f, %f, %f] for lbx and [%f, %f, %f] for ubx", lbx[0], lbx[1], lbx[2],
           ubx[0], ubx[1], ubx[2]);
}

std::vector<double> nmpc::TiltMtNeuralServoPlusMPC::PhysToMPCParams() const
{
  int rotor_num = robot_model_->getRotorNum();  // For tilt-rotor, rotor_num = servo_num
  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const map<int, int> rotor_dr = robot_model_->getRotorDirection();
  double kq_d_kt = abs(robot_model_->getMFRate());  // PAY ATTENTION: should be positive value

  std::vector<double> phys_p(2 + 3 + 1 + 4 * rotor_num + 2 + 7, 0);
  // order: mass, gravity, Ixx, Iyy, Izz, kq_d_kt, dr1, p1_b, dr2, p2_b, dr3, p3_b, dr4, p4_b, t_rotor, t_servo
  // ee_p, ee_qwxyz
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

  std::vector<double> contact_frame_p, contact_frame_q;
  robot_model_->getCoGtoFramePosQuat("ee_contact", contact_frame_p, contact_frame_q);

  std::copy(contact_frame_p.begin(), contact_frame_p.end(), phys_p.begin() + idx);
  idx += static_cast<int>(contact_frame_p.size());

  std::copy(contact_frame_q.begin(), contact_frame_q.end(), phys_p.begin() + idx);
  idx += static_cast<int>(contact_frame_q.size());

  return phys_p;
}

void nmpc::TiltMtNeuralServoPlusMPC::controlCore(bool is_warmup)
// When integrating an I-term in the future, we need to skip the I-term update during warm-up
{
  // restore velocity constraints after hovering
  if (navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE and has_restored_vel_ == false)
  {
    modifyVelConstraints(vel_min_, vel_max_);
    has_restored_vel_ = true;
  }

  prepareMPCRef();

  prepareMPCParams();

  /* Get current state from estimator */
  bx0_ = meas2VecX();

  /* Call solver to solve the optimization problem */
  try
  {
    mpc_solver_ptr_->solve(bx0_, is_debug_);
  }
  catch (mpc_solver::AcadosSolveException& e)
  {
    ROS_FATAL("[MPC] Solver failed. Details: %s", e.what());
  }
  // Note: The result is stored in mpc_solver_ptr_->uo_

  /* Get result from solver */
  // - thrust
  for (int i = 0; i < motor_num_; i++)
  {
    flight_cmd_.base_thrust[i] = (float)getCommand(i);
  }

  // - servo angle
  gimbal_ctrl_cmd_.header.stamp = ros::Time::now();
  gimbal_ctrl_cmd_.name.clear();
  gimbal_ctrl_cmd_.position.clear();
  for (int i = 0; i < joint_num_; i++)
  {
    gimbal_ctrl_cmd_.name.emplace_back("gimbal" + std::to_string(i + 1));
    gimbal_ctrl_cmd_.position.push_back(getCommand(motor_num_ + i));
  }
}

void nmpc::TiltMtNeuralServoPlusMPC::sendCmd()
{
  /* Publish commands */
  if (motor_num_ > 0)
    pub_flight_cmd_.publish(flight_cmd_);
  if (joint_num_ > 0)
    pub_gimbal_control_.publish(gimbal_ctrl_cmd_);
}

void nmpc::TiltMtNeuralServoPlusMPC::prepareMPCRef()
{
  /* if in trajectory tracking mode, the ref is set by callbackSetRefXU.
   * So here we check if the traj info is still received. If not, we turn off the tracking mode */
  if (is_traj_tracking_)
  {
    if (ros::Time::now() - x_u_ref_.header.stamp > ros::Duration(0.5))
    {
      ROS_INFO("[MPC] No traj msg for 0.5s. Trajectory tracking mode is off! Return to the hovering!");
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
    else if (ros::Time::now() - x_u_ref_.header.stamp > ros::Duration(0.1))
    {
      ROS_INFO_THROTTLE(1, "[MPC] No traj msg for 0.1s. Try to track current pose.");
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

      last_traj_msg_.points.clear();  // every time end the traj tracking, clear the traj msg
    }

    return;
  }

  /* if not in tracking mode, we use point mode --> set target */
  // Added on 2025-07-17: Note: in this mode we should always track the CoG point. So if the reference of MPC
  // is assumed in tool frame, we need to do a conversion. On the contrary, for traj. tracking, we directly track tool
  // frame.
  tf::Vector3 target_cog_pos_in_w = navigator_->getTargetPos();
  tf::Vector3 target_cog_vel_in_w = navigator_->getTargetVel();
  tf::Vector3 target_cog_rpy = navigator_->getTargetRPY();
  tf::Quaternion target_cog_quat;
  target_cog_quat.setRPY(target_cog_rpy.x(), target_cog_rpy.y(), target_cog_rpy.z());
  tf::Vector3 target_cog_omega = navigator_->getTargetOmega();

  // make conversion
  tf::Vector3 target_ee_pos_in_w, target_ee_vel_in_w, target_ee_omega;
  tf::Quaternion target_ee_quat;
  robot_model_->convertFromCoGToEEContact(target_cog_pos_in_w, target_cog_vel_in_w, target_cog_quat, target_cog_omega,
                                          target_ee_pos_in_w, target_ee_vel_in_w, target_ee_quat, target_ee_omega);

  // set the reference state and control input
  setXrUrRef(target_ee_pos_in_w, target_ee_vel_in_w, tf::Vector3(0, 0, 0), target_ee_quat, target_ee_omega,
             tf::Vector3(0, 0, 0), -1);
  rosXU2VecXU(x_u_ref_, mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_);
  mpc_solver_ptr_->setReference(mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_, true);
}

void nmpc::TiltMtNeuralServoPlusMPC::prepareMPCParams()
{
  updateInertialParams();

  // TODO: this condition is temporary for drones that don't pass in phys param (bi, tri, fix-qd)
  if (NP_ > 4 + 6)
  {
    std::vector<double> phys_p = PhysToMPCParams();

    std::vector<int> idx(phys_p.size());
    std::iota(idx.begin(), idx.end(), idx_p_quat_end_ + 1);

    mpc_solver_ptr_->setParamSparseAllStages(idx, phys_p);
  }

  // Update linearization parameters
  if (linearize_mlp_)
  {
    updateLinearizationParams();
  }
}

void nmpc::TiltMtNeuralServoPlusMPC::updateLinearizationParams()
{
  try
  {
    // 1) Assemble batched neural network input x0
    // NOTE: batch_size_ = NN_ + 1
    for (int j = 0; j < batch_size_; ++j)
    {
      const auto& state_j_raw = mpc_solver_ptr_->xo_.at(j);

      // For terminal node, use the control value from N-1
      const auto& u_cmd_j = mpc_solver_ptr_->uo_.at(j < NN_ ? j : NN_ - 1);

      // Input transform
      std::vector<double> state_j = state_j_raw;
      if (input_transform_)
      {
        tf::Quaternion q(state_j[6], state_j[7], state_j[8], state_j[9]);  // (w,x,y,z) order
        tf::Vector3 v_w(state_j[3], state_j[4], state_j[5]);
        tf::Vector3 v_b = tf::quatRotate(q.inverse(), v_w);
        state_j[3] = v_b.x();
        state_j[4] = v_b.y();
        state_j[5] = v_b.z();
      }

      float* row_j = x0_vec_f32_.data() + j * N_in_;
      int idx = 0;
      for (int feat_idx : state_feats_) row_j[idx++] = static_cast<float>(state_j[feat_idx]);
      for (int feat_idx : u_feats_)     row_j[idx++] = static_cast<float>(u_cmd_j[feat_idx]);
    }

    // 2) Create torch::Tensor
    // NOTE: from_blob() does NOT copy; the buffer must remain valid for the lifetime of the tensor
    torch::Tensor x0 = torch::from_blob(
      x0_vec_f32_.data(),
      { batch_size_, N_in_ },
      torch::TensorOptions()
        .dtype(torch::kFloat32))
      .to(device_mlp_);  // First construct on CPU and then move to device to avoid issues with split memory-access

    // Repeat x0 such that each "virtual batch element" corresponds to one (b, i) pair
    // IDEA: A single extended forward call is cheaper than N_out_ grad() calls
    // Basically we repeat every row of x0 N_out_ times. From the model forward call we receive corresponding N_out_ duplicate outputs for each set of N_out_ rows
    torch::Tensor x0_rep = x0.repeat_interleave(N_out_, /*dim=*/0)  // (B*N_out_, N_in_)
                             .requires_grad_(true);

    // Wrap the Tensor in an IValue vector
    std::vector<torch::jit::IValue> inputs_rep;
    inputs_rep.push_back(x0_rep);
    
    // 3) Forward pass
    // NOTE: We need to call forward() on the entire repeated input to ensure that the gradients are correctly tracked w.r.t. the entire repeated input
    // NOTE: y0_rep does NOT need to be label transformed since mlp_out is transformed inside the MPC formulation
    torch::Tensor y0_rep = neural_module_->forward(inputs_rep).toTensor();
    TORCH_CHECK(y0_rep.dim() == 2 && y0_rep.size(0) == batch_size_ * N_out_ && y0_rep.size(1) == N_out_,
          "Expected y0_rep shape (", batch_size_ * N_out_, ", ", N_out_, "), got ", y0_rep.sizes());

    // 4) Linearize
    auto [J0, H0] = linearize(x0_rep, y0_rep);

    // 5) Cast and Flatten Input, Output, Jacobian and Hessian to 1-D std::vector<double> in Fortran order to send to acados
    // NOTE: acados expects double
    x0_vec_.assign(x0_vec_f32_.begin(), x0_vec_f32_.end());  // move to std::vector<double>
    torch::Tensor y0 = y0_rep.view({batch_size_, N_out_, N_out_}).select(1, 0);  // extract (B, N_out_)
    auto y0_f64 = y0.detach().cpu().to(torch::kFloat64).contiguous();  // cast to double
    std::memcpy(y0_vec_.data(),  // move to std::vector<double>
                y0_f64.data_ptr<double>(),
                y0_vec_.size() * sizeof(double));
    flattenTensors(J0, H0);

    // 6) Set per-stage parameters from the batched linearization
    for (int j = 0; j <= NN_; ++j)
    {
      std::memcpy(lin_params_.data() + off_x0_,
                  x0_vec_.data() + j * stride_x0_,
                  stride_x0_ * sizeof(double));

      std::memcpy(lin_params_.data() + off_y0_,
                  y0_vec_.data() + j * stride_y0_,
                  stride_y0_ * sizeof(double));

      std::memcpy(lin_params_.data() + off_J0_,
                  J0_vec_.data() + j * stride_J0_,
                  stride_J0_ * sizeof(double));

      if (linearize_order_ >= 2)
        std::memcpy(lin_params_.data() + off_H0_,
                    H0_vec_.data() + j * stride_H0_,
                    stride_H0_ * sizeof(double));

      mpc_solver_ptr_->setParamSparseOneStage(j, lin_param_idx_, lin_params_);
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_THROTTLE(1.0, "[NEURAL][MPC] Torch error during batched linearization: %s", e.what());
    return;
  }
  return;
}

std::pair<torch::Tensor, torch::Tensor> nmpc::TiltMtNeuralServoPlusMPC::linearize(const torch::Tensor& x0_rep,
                                                                                  const torch::Tensor& y0_rep)
{
  TORCH_CHECK(x0_rep.requires_grad(), "[TORCH] Input tensor must have requires_grad=true");
  TORCH_CHECK(y0_rep.requires_grad(), "[TORCH] Output tensor must have requires_grad=true");

  bool compute_graph = (linearize_order_ >= 2);

  // ── Jacobian ──────────────────────────────────────────────
  // Compute full batched Jacobian in a single backward pass.
  //
  // Build a (B*N_out_, N_out_) identity with batch_size_ one-hot
  // vectors as columns so a single grad() returns all rows at once.
  //
  // With this the "repeated-input" work-around we avoid retain_graph
  // completely and only call grad() once.
  // ──────────────────────────────────────────────────────────

  // grad_outputs: select output i for virtual-batch element (b*N_out_+i).
  // Shape: (B*N_out_, N_out_) - block-diagonal identity tiled B times.
  torch::Tensor eye_block = torch::eye(N_out_,
      torch::TensorOptions().dtype(torch::kFloat32).device(device_mlp_))
      .repeat({batch_size_, 1});  // (B*N_out_, N_out_)

  auto J_flat = torch::autograd::grad(  // (B*N_out_, N_in_)
      { y0_rep },
      { x0_rep },
      { eye_block },
      /*retain_graph=*/compute_graph,  // keep only if Hessian needed
      /*create_graph=*/compute_graph)[0];

  torch::Tensor J0 = J_flat.view({batch_size_, N_out_, N_in_});

  // ── Hessian ───────────────────────────────────────────────
  // We need H[b,i,j,k] = d^2 y[b,i] / dx[b,j]dx[b,k].
  // J_flat already contains dJ[b,i] / dx which is treated as a scalar
  // function of x0_rep. We now differentiate each column k of
  // J_flat w.r.t. x0_rep using the same identity trick.
  // ──────────────────────────────────────────────────────────
  torch::Tensor H0;
  if (linearize_order_ >= 2)
  {
    // eye_k: (B*N_out_, N_in_) - for each k, selects column k of J_flat
    // We tile torch::eye(N_in_) for all B*N_out_ virtual elements:
    //   shape: (N_in_, B*N_out_, N_in_)  -> iterate k in outer loop

    std::vector<torch::Tensor> H_cols;
    H_cols.reserve(N_in_);

    torch::Tensor eye_in = torch::eye(N_in_,
        torch::TensorOptions().dtype(torch::kFloat32).device(device_mlp_));

    for (int k = 0; k < N_in_; ++k)
    {
      // grad_outputs selects column k of J_flat for every (b,i) element.
      // Shape: (B*N_out_, N_in_) - k-th unit vector, tiled.
      torch::Tensor grad_outputs_k = eye_in[k]
          .unsqueeze(0)                              // (1, N_in_)
          .expand({batch_size_ * N_out_, N_in_});    // (B*N_out_, N_in_)

      bool last_k = (k == N_in_ - 1);
      auto H_k = torch::autograd::grad(  // (B*N_out_, N_in_) 
          { J_flat },
          { x0_rep },
          { grad_outputs_k },
          /*retain_graph=*/!last_k,
          /*create_graph=*/false)[0];

      H_cols.push_back(H_k.view({batch_size_, N_out_, N_in_}));
    }
    H0 = torch::stack(H_cols, /*dim=*/3);
  }
  else
  {
    H0 = torch::zeros({batch_size_, N_out_, N_in_, N_in_},
         torch::TensorOptions().dtype(torch::kFloat32).device(device_mlp_));
  }

  return {J0, H0};
}

// =========== NAIVE APPROACH (loop over N_out_ dim) =============
//   std::vector<torch::Tensor> J_cols;
//   J_cols.reserve(N_out_);
  
//   for (int i = 0; i < N_out_; ++i)
//   {
//     torch::Tensor jac_grad_outputs = torch::zeros_like(y0); // (B, N_out_)
//     jac_grad_outputs.select(1, i).fill_(1.0f);

//     bool last_i = (i == N_out_ - 1);
//     auto grads = torch::autograd::grad(
//         { y0 },
//         { x0 },
//         { jac_grad_outputs },
//         /*retain_graph=*/(!last_i || compute_graph),
//         /*create_graph=*/compute_graph);

//     J_cols.push_back(grads[0]);  // (B, N_in_)
//   }
//   // J0 shape: (B, N_out_, N_in_)
//   torch::Tensor J0 = torch::stack(J_cols, /*dim=*/1);

//   torch::Tensor H0;
//   if (linearize_order_ >= 2)
//   {
//     std::vector<torch::Tensor> H_out_cols;
//     H_out_cols.reserve(N_out_);

//     torch::Tensor hess_grad_outputs = torch::ones({batch_size_}, 
//         torch::TensorOptions().dtype(torch::kFloat32).device(device_mlp_));

//     for (int i = 0; i < N_out_; ++i)
//     {
//       std::vector<torch::Tensor> H_in_cols;
//       H_in_cols.reserve(N_in_);
      
//       for (int k = 0; k < N_in_; ++k)
//       {
//         bool last_grad = (i == N_out_ - 1) && (k == N_in_ - 1);
        
//         // Differentiate J0[:, i, k] wrt x0
//         // J0.select(1, i) gives (B, N_in_), then .select(1, k) gives (B)
//         auto H_k = torch::autograd::grad(  // (B, N_in_)
//             { J0.select(1, i).select(1, k) }, 
//             { x0 },
//             { hess_grad_outputs },
//             /*retain_graph=*/!last_grad,
//             /*create_graph=*/false)[0];
//         H_in_cols.push_back(H_k);
//       }
//       // Stack along dim=1 to get (B, N_in_, N_in_)
//       H_out_cols.push_back(torch::stack(H_in_cols, 1));
//     }
//     // Stack along dim=1 to get (B, N_out_, N_in_, N_in_)
//     H0 = torch::stack(H_out_cols, 1);
//   }

/**
 * Flatten J0 and H0 to 1-D std::vector<float> using CasADi / Fortran
 * (column-major) order.
 *
 * J0 shape: (B, N_out, N_in)
 *   CasADi sees a (N_out x N_in) matrix -> column-major -> i (output) varies
 *   fastest.
 *   Flat index: i + N_out_ * j
 *   Equivalent torch op: J0[b].t().contiguous().flatten()
 *
 * H0 shape: (B, N_out, N_in, N_in)
 *   CasADi sees a (N_out*N_in x N_in) matrix -> column-major.
 *   The "row" axis is the composite (i, j) pair, with i varying fastest.
 *   Flat index: i + N_out_*j + N_out_*N_in_*k
 *   Equivalent torch op: H0[b].permute({2,1,0}).contiguous().flatten()
 *     (permute k->dim0, j->dim1, i->dim2, then flatten with i varying fastest)
 */
void nmpc::TiltMtNeuralServoPlusMPC::flattenTensors(const torch::Tensor& J0,
                                                    const torch::Tensor& H0)
{
    // ---- Jacobian: (B, N_out, N_in) -> B × (N_out*N_in,) Fortran-flat ----
    // Transpose last two dims so memory order is [j, i] -> flatten gives
    // column-major [i0, i1, …, iN | next col …]
    torch::Tensor J0_flat = J0
        .transpose(1, 2)           // (B, N_in, N_out) - i varies fastest
        .contiguous()              // force contiguous before data_ptr
        .reshape({batch_size_, N_out_ * N_in_});  // (B, N_out*N_in)

    // ---- Hessian: (B, N_out, N_in, N_in) -> B × (N_out*N_in*N_in,) Fortran-flat ----
    // CasADi matrix shape: rows = N_out*N_in  (i fast, j slow)
    //                      cols = N_in        (k)
    // Column-major: i varies fastest, then j, then k.
    // Achieve with permute [b, i, j, k] -> [b, i, k, j] then flatten.
    // keep the N_out axis (i) in place as dim-1 so each output block stays contiguous,
    // and only swap the two N_in axes so j varies fastest within each block.
    torch::Tensor H0_flat = H0
        .permute({0, 1, 3, 2})    // (B, N_out_i, N_in_k, N_in_j) 
        .contiguous()
        .reshape({batch_size_, N_out_ * N_in_ * N_in_});  // (B, N_out*N_in*N_in)

    const int64_t j_numel = J0_flat.numel();
    const int64_t h_numel = H0_flat.numel();

    if (j_numel != J0_vec_.size() || h_numel != H0_vec_.size())
    {
        ROS_ERROR("[NEURAL][MPC] Size mismatch when flattening Jacobian/Hessian: J0 numel = %ld, "
                  "H0 numel = %ld, but J0_vec_ size = %zu, H0_vec_ size = %zu",
                  j_numel, h_numel, J0_vec_.size(), H0_vec_.size());
        return;
    }

    std::memcpy(J0_vec_.data(), J0_flat.cpu().to(torch::kFloat64).data_ptr<double>(), j_numel * sizeof(double));
    std::memcpy(H0_vec_.data(), H0_flat.cpu().to(torch::kFloat64).data_ptr<double>(), h_numel * sizeof(double));
    return;
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
void nmpc::TiltMtNeuralServoPlusMPC::setXrUrRef(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                                const tf::Vector3& ref_acc_i, const tf::Quaternion& ref_quat_ib,
                                                const tf::Vector3& ref_omega_b, const tf::Vector3& ref_ang_acc_b,
                                                const int& horizon_idx)
{
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
  std::vector<double> x(NX_);
  std::vector<double> u(NU_);
  allocateToXU(ref_pos_i, ref_vel_i, ref_quat_ib, ref_omega_b, ref_wrench_b, x, u);

  /* set values */
  if (horizon_idx == -1)
  {
    for (int i = 0; i <= NN_; i++)
    {
      std::copy(x.begin(), x.begin() + NX_, x_u_ref_.x.data.begin() + NX_ * i);
      if (i < NN_)
        std::copy(u.begin(), u.begin() + NU_, x_u_ref_.u.data.begin() + NU_ * i);
    }
    return;
  }

  if (horizon_idx < 0 || horizon_idx > NN_)
  {
    ROS_WARN("[MPC] horizon_idx is out of range! CalXrUrRef failed!");
    return;
  }

  std::copy(x.begin(), x.begin() + NX_, x_u_ref_.x.data.begin() + NX_ * horizon_idx);
  if (horizon_idx < NN_)
    std::copy(u.begin(), u.begin() + NU_, x_u_ref_.u.data.begin() + NU_ * horizon_idx);
}

void nmpc::TiltMtNeuralServoPlusMPC::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                                  const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                                                  const VectorXd& ref_wrench_b, vector<double>& x, vector<double>& u)
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

  // ========= 0) if one rotor is fixed, do it and finish. ======
  if (is_set_fix_rotor_)
  {
    if (ros::Time::now() - fix_rotor_msg_.header.stamp > ros::Duration(0.1))
    {
      ROS_INFO_THROTTLE(1, "[MPC] No FixRotor msg for 0.1s. Recover to the normal allocation state.");
      is_set_fix_rotor_ = false;
    }

    allocateToXUwOneFixedRotor(fix_rotor_msg_.rotor_id, fix_rotor_msg_.fix_ft, fix_rotor_msg_.fix_alpha, ref_wrench_b,
                               x, u);
    return;
  }
  // =============================================================

  // 1) do one allocation
  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * ref_wrench_b;
  std::vector<double> ft_ref_vec(motor_num_);
  std::vector<double> a_ref_vec(joint_num_);

  if (motor_num_ != joint_num_)
  {
    ROS_FATAL("[MPC] motor_num_ is not equal to joint_num_! Cannot allocate to X and U!");
    throw std::runtime_error("[MPC] motor_num_ is not equal to joint_num_! Cannot allocate to X and U!");
  }
  for (int i = 0; i < motor_num_; i++)
  {
    ft_ref_vec[i] = sqrt(x_lambda(2 * i) * x_lambda(2 * i) + x_lambda(2 * i + 1) * x_lambda(2 * i + 1));
    a_ref_vec[i] = atan2(x_lambda(2 * i), x_lambda(2 * i + 1));

    u.at(i) = ft_ref_vec[i];
    x.at(13 + i) = ensureOneServoContinuity(a_ref_vec[i], i);
  }

  if (alloc_type_ == 0)
    return;

  // 2) check if one rotor's thrust is less than threshold and flip backwards
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
        "[MPC] More than one rotor is below threshold and flip backwards! Select rotor %d with thrust %.2f as the "
        "fixed rotor.",
        rotor_idx, max_ft);
  }
  else
  {
    rotor_idx = rotor_idx_vec.at(0);
  }

  // 3) if rotor_idx is not empty, maintain the thrust and modify the angle
  double ft_stop_rotor = ft_ref_vec[rotor_idx];
  double alpha_stop_rotor = M_PI_2 - acos(x_lambda(2 * rotor_idx) / ft_thresh_);

  // 4) re-alloc
  allocateToXUwOneFixedRotor(rotor_idx, ft_stop_rotor, alpha_stop_rotor, ref_wrench_b, x, u);
}

void nmpc::TiltMtNeuralServoPlusMPC::allocateToXUwOneFixedRotor(int fix_rotor_idx, double fix_ft, double fix_alpha,
                                                                const VectorXd& ref_wrench_b, vector<double>& x,
                                                                vector<double>& u)
{
  double fix_ft_x = fix_ft * sin(fix_alpha);
  double fix_ft_y = fix_ft * cos(fix_alpha);

  // 1) construct tgt_wrench from z_from_rotor
  Eigen::VectorXd z_from_rotor = Eigen::VectorXd::Zero(motor_num_ * 2);
  z_from_rotor(2 * fix_rotor_idx) = fix_ft_x;
  z_from_rotor(2 * fix_rotor_idx + 1) = fix_ft_y;
  Eigen::VectorXd tgt_wrench_from_rotor = alloc_mat_ * z_from_rotor;

  // 2) calculate alloc_mat with this rotor's contribution
  Eigen::VectorXd tgt_wrench_modified = ref_wrench_b - tgt_wrench_from_rotor;

  // 3) calculate the allocation matrix without this rotor, which is 6*6
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

  // 4) reconstruct the z output
  Eigen::VectorXd z_except_rotor = alloc_mat_del_rotor_inv_ * tgt_wrench_modified;

  // 5) at the place of 2*fix_rotor_idx, insert 2 numbers to z_except_rotor
  Eigen::VectorXd z_final(motor_num_ * 2);
  z_final.head(2 * fix_rotor_idx) = z_except_rotor.head(2 * fix_rotor_idx);
  z_final(2 * fix_rotor_idx) = fix_ft_x;
  z_final(2 * fix_rotor_idx + 1) = fix_ft_y;
  z_final.tail(z_except_rotor.size() - 2 * fix_rotor_idx) =
      z_except_rotor.tail(z_except_rotor.size() - 2 * fix_rotor_idx);

  // 6) reconstruct the thrust and servo angle
  // check motor_num_ == joint_num_ before this function is called
  if (motor_num_ != joint_num_)
  {
    ROS_FATAL("[MPC] motor_num_ is not equal to joint_num_! Cannot allocate to X and U!");
    throw std::runtime_error("[MPC] motor_num_ is not equal to joint_num_! Cannot allocate to X and U!");
  }
  for (int i = 0; i < motor_num_; i++)
  {
    const double ft = sqrt(z_final(2 * i) * z_final(2 * i) + z_final(2 * i + 1) * z_final(2 * i + 1));
    u.at(i) = ft;
    const double alpha = atan2(z_final(2 * i), z_final(2 * i + 1));
    x.at(13 + i) = ensureOneServoContinuity(alpha, i);
  }

  // if the fixed rotor is the same with previous one, no need to recalculate the allocation matrix.
  rotor_idx_prev_ = fix_rotor_idx;
}

/**
 * @brief publishRecording: publish the current, predicted and reference trajectory with full state and controls vectors for dataset recording
 */
void nmpc::TiltMtNeuralServoPlusMPC::publishRecording()
{
  stamp = ros::Time::now();

  // Current state (input to MPC at this timestep)
  aerial_robot_msgs::MPCState curr_mpc_state;
  curr_mpc_state.header.frame_id = "world";
  curr_mpc_state.header.stamp = stamp;
  
  // Position
  curr_mpc_state.position.x = bx0_[0];
  curr_mpc_state.position.y = bx0_[1];
  curr_mpc_state.position.z = bx0_[2];
  
  // Linear velocity
  curr_mpc_state.linear_velocity.x = bx0_[3];
  curr_mpc_state.linear_velocity.y = bx0_[4];
  curr_mpc_state.linear_velocity.z = bx0_[5];
  
  // Quaternion
  curr_mpc_state.orientation.w = bx0_[6];
  curr_mpc_state.orientation.x = bx0_[7];
  curr_mpc_state.orientation.y = bx0_[8];
  curr_mpc_state.orientation.z = bx0_[9];
  
  // Angular velocity
  curr_mpc_state.angular_velocity.x = bx0_[10];
  curr_mpc_state.angular_velocity.y = bx0_[11];
  curr_mpc_state.angular_velocity.z = bx0_[12];
  
  // Servo angle state
  curr_mpc_state.servo_angles.resize(joint_num_);
  for (int i = 0; i < joint_num_; ++i)
  {
    curr_mpc_state.servo_angles[i] = bx0_[13 + i];
  }
  
  pub_record_curr_.publish(curr_mpc_state);

  // Publish reference states
  aerial_robot_msgs::MPCTrajectory ref_msg;
  ref_msg.header.frame_id = "world";
  ref_msg.header.stamp = stamp;
  ref_msg.states.resize(NN_ + 1);
  ref_msg.controls.resize(NN_);

  for (int i = 0; i <= NN_; ++i)
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
      ref_msg.states[i].servo_angles[j] = mpc_solver_ptr_->xr_[i][13 + j];
    }
  }

  for (int i = 0; i < NN_; ++i)
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
  pred_msg.states.resize(NN_ + 1);
  pred_msg.controls.resize(NN_);

  for (int i = 0; i <= NN_; ++i)
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
  
  for (int i = 0; i < NN_; ++i)
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

/**
 * @brief callbackViz: publish the predicted trajectory and reference trajectory
 * @param [ros::TimerEvent&] event
 */
void nmpc::TiltMtNeuralServoPlusMPC::callbackViz(const ros::TimerEvent& event)
{
  // from mpc_solver_ptr_->x_u_out to PoseArray
  geometry_msgs::PoseArray pred_poses;
  geometry_msgs::PoseArray ref_poses;

  for (int i = 0; i < NN_; ++i)
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

void nmpc::TiltMtNeuralServoPlusMPC::callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
  for (int i = 0; i < joint_num_; i++)
    joint_angles_[i] = msg->position[i];
}

/* TODO: this function is just for test. We may need a more general function to set all kinds of state */
void nmpc::TiltMtNeuralServoPlusMPC::callbackSetRPY(const spinal::DesireCoordConstPtr& msg)
{
  // add a check to avoid the singular point for euler angle
  if (msg->pitch == M_PI / 2.0 or msg->pitch == -M_PI / 2.0)
  {
    ROS_WARN(
        "[MPC] The pitch angle is set to PI/2 or -PI/2, which is a singular point for euler angle."
        " Please set other values for the pitch angle.");
    return;
  }

  navigator_->setTargetRoll(msg->roll);
  navigator_->setTargetPitch(msg->pitch);
  navigator_->setTargetYaw(msg->yaw);
}

/* TODO: this function should be combined with the inner planning framework */
void nmpc::TiltMtNeuralServoPlusMPC::callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg)
{
  /* failsafe check */
  if (navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE)
  {
    ROS_WARN_THROTTLE(1, "[MPC] The robot has not hovered, so the reference trajectory will be ignored!");
    return;
  }

  /* switch tracking mode */
  if (!is_traj_tracking_)
  {
    ROS_INFO("[MPC] Trajectory tracking mode is on!");
    is_traj_tracking_ = true;
  }

  /* receive info */
  x_u_ref_ = *msg;

  /* set reference */
  rosXU2VecXU(x_u_ref_, mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_);
  mpc_solver_ptr_->setReference(mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_, true);
}

void nmpc::TiltMtNeuralServoPlusMPC::callbackSetRefTraj(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  if (msg->points.size() != NN_ + 1)
    ROS_WARN("[MPC] The length of the trajectory is not equal to the prediction horizon! Cannot use the trajectory!");

  if (navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE)
  {
    ROS_WARN_THROTTLE(1, "[MPC] The robot has not hovered, so the reference trajectory will be ignored!");
    return;
  }

  /* For set-point regulation, if the traj planner sends the same traj, we can skip the calculation of allocation. */
  // check if two trajectories are the same
  int max_same_idx = 0;
  if (!last_traj_msg_.points.empty())  // check if the last trajectory is empty
  {
    for (int i = 0; i < msg->points.size(); i++)  // only check the first NN points
    {
      if (isMulDOFJointTrajPtEqual(msg->points[i], last_traj_msg_.points[i], false))  // time is not equal
        max_same_idx = i;
      else
        break;
    }
  }

  if (max_same_idx != msg->points.size() - 1 || is_set_fix_rotor_ == true)
  {
    for (int i = 0; i < NN_ + 1; i++)
    {
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& point = msg->points[i];
      geometry_msgs::Vector3 pos = point.transforms[0].translation;
      geometry_msgs::Vector3 vel = point.velocities[0].linear;
      geometry_msgs::Vector3 acc = point.accelerations[0].linear;
      geometry_msgs::Quaternion quat = point.transforms[0].rotation;
      geometry_msgs::Vector3 omega = point.velocities[0].angular;
      geometry_msgs::Vector3 ang_acc = point.accelerations[0].angular;
      setXrUrRef(tf::Vector3(pos.x, pos.y, pos.z), tf::Vector3(vel.x, vel.y, vel.z), tf::Vector3(acc.x, acc.y, acc.z),
                 tf::Quaternion(quat.x, quat.y, quat.z, quat.w), tf::Vector3(omega.x, omega.y, omega.z),
                 tf::Vector3(ang_acc.x, ang_acc.y, ang_acc.z), i);
    }
  }

  x_u_ref_.header.stamp = msg->header.stamp;
  callbackSetRefXU(boost::make_shared<const aerial_robot_msgs::PredXU>(x_u_ref_));

  last_traj_msg_ = *msg;
}

void nmpc::TiltMtNeuralServoPlusMPC::callbackSetFixedRotor(const aerial_robot_msgs::FixRotorConstPtr& msg)
{
  // failsafe
  if (msg->rotor_id < 0 || msg->rotor_id >= motor_num_)
  {
    ROS_WARN_STREAM("[MPC] The rotor_id " << static_cast<int>(msg->rotor_id)
                                    << " is incorrect. Note that the id starts from 0.");
    return;
  }

  if (msg->fix_ft < thrust_ctrl_min_ || msg->fix_ft > thrust_ctrl_max_)
  {
    ROS_WARN_STREAM("[MPC] The fix_ft value " << msg->fix_ft << " is out of range. It should be between " << thrust_ctrl_min_
                                        << " and " << thrust_ctrl_max_ << ".");
    return;
  }

  if (msg->fix_alpha < servo_angle_min_ || msg->fix_alpha > servo_angle_max_)
  {
    ROS_WARN_STREAM("[MPC] The fix_alpha value " << msg->fix_alpha << " is out of range. It should be between "
                                           << servo_angle_min_ << " and " << servo_angle_max_ << ".");
    return;
  }

  // set values
  is_set_fix_rotor_ = true;
  fix_rotor_msg_ = *msg;
}

void nmpc::TiltMtNeuralServoPlusMPC::cfgMPCCallback(NMPCConfig& config, uint32_t level)
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

          ROS_INFO_STREAM("[MPC] Change Qp_xy for NMPC '" << config.Qp_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_P_Z: {
          mpc_solver_ptr_->setCostWDiagElement(2, config.Qp_z);
          ROS_INFO_STREAM("[MPC] Change Qp_z for NMPC '" << config.Qp_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_V_XY: {
          mpc_solver_ptr_->setCostWDiagElement(3, config.Qv_xy);
          mpc_solver_ptr_->setCostWDiagElement(4, config.Qv_xy);
          ROS_INFO_STREAM("[MPC] Change Qv_xy for NMPC '" << config.Qv_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_V_Z: {
          mpc_solver_ptr_->setCostWDiagElement(5, config.Qv_z);
          ROS_INFO_STREAM("[MPC] Change Qv_z for NMPC '" << config.Qv_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_Q_XY: {
          mpc_solver_ptr_->setCostWDiagElement(7, config.Qq_xy);
          mpc_solver_ptr_->setCostWDiagElement(8, config.Qq_xy);
          ROS_INFO_STREAM("[MPC] Change Qq_xy for NMPC '" << config.Qq_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_Q_Z: {
          mpc_solver_ptr_->setCostWDiagElement(9, config.Qq_z);
          ROS_INFO_STREAM("[MPC] Change Qq_z for NMPC '" << config.Qq_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_W_XY: {
          mpc_solver_ptr_->setCostWDiagElement(10, config.Qw_xy);
          mpc_solver_ptr_->setCostWDiagElement(11, config.Qw_xy);
          ROS_INFO_STREAM("[MPC] Change Qw_xy for NMPC '" << config.Qw_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_W_Z: {
          mpc_solver_ptr_->setCostWDiagElement(12, config.Qw_z);
          ROS_INFO_STREAM("[MPC] Change Qw_z for NMPC '" << config.Qw_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_A: {
          for (int i = 13; i < 13 + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qa);
          ROS_INFO_STREAM("[MPC] Change Qa for NMPC '" << config.Qa << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_T: {
          for (int i = NX_; i < NX_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rt, false);
          ROS_INFO_STREAM("[MPC] Change Rt for NMPC '" << config.Rt << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_AC_D: {
          for (int i = NX_ + motor_num_; i < NX_ + motor_num_ + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rac_d, false);
          ROS_INFO_STREAM("[MPC] Change Rac_d for NMPC '" << config.Rac_d << "'");
          break;
        }
        default: {
          ROS_INFO_STREAM("[MPC] The setting variable is not in the list!");
          break;
        }
      }
    }
    catch (std::invalid_argument& e)
    {
      ROS_ERROR_STREAM("[MPC] Config failed: " << e.what());
    }
  }
}

double nmpc::TiltMtNeuralServoPlusMPC::getCommand(int idx_u, double T_horizon) const
{
  if (T_horizon == 0)
    return mpc_solver_ptr_->uo_.at(0).at(idx_u);

  return mpc_solver_ptr_->uo_.at(0).at(idx_u) +
         T_horizon / t_mpc_step_ * (mpc_solver_ptr_->uo_.at(1).at(idx_u) - mpc_solver_ptr_->uo_.at(0).at(idx_u));
}

std::vector<double> nmpc::TiltMtNeuralServoPlusMPC::meas2VecX(bool is_ee_centric)
{
  vector<double> bx0(mpc_solver_ptr_->NBX0_, 0);

  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Quaternion quat = estimator_->getQuat(Frame::COG, estimate_mode_);
  tf::Vector3 ang_vel = estimator_->getAngularVel(Frame::COG, estimate_mode_);

  // === check the sign of the quaternion, avoid the flip of the quaternion. ===
  // This is quite important because of the warm-starting of the MPC solver. The quaternion should be continuous.
  double qe_c_w =
      quat.w() * quat_prev_.w() + quat.x() * quat_prev_.x() + quat.y() * quat_prev_.y() + quat.z() * quat_prev_.z();
  if (qe_c_w < 0)
  {
    quat = quat.operator-();
  }

  quat_prev_ = quat;

  // === for reference, we may need to convert the position and velocity to the end-effector frame ===
  if (is_ee_centric)
  {
    // convert the position and velocity from CoG to end-effector frame
    tf::Vector3 target_pos, target_ee_vel, target_ee_omega;
    tf::Quaternion target_ee_quat;
    robot_model_->convertFromCoGToEEContact(pos, vel, quat, ang_vel, target_pos, target_ee_vel, target_ee_quat,
                                            target_ee_omega);

    pos = target_pos;
    vel = target_ee_vel;
    quat = target_ee_quat;
    ang_vel = target_ee_omega;
  }

  // === fill the vector ===
  bx0[0] = pos.x();
  bx0[1] = pos.y();
  bx0[2] = pos.z();
  bx0[3] = vel.x();
  bx0[4] = vel.y();
  bx0[5] = vel.z();
  bx0[6] = quat.w();
  bx0[7] = quat.x();
  bx0[8] = quat.y();
  bx0[9] = quat.z();
  bx0[10] = ang_vel.x();
  bx0[11] = ang_vel.y();
  bx0[12] = ang_vel.z();
  for (int i = 0; i < joint_num_; i++)
    bx0[13 + i] = joint_angles_[i];
  return bx0;
}

double nmpc::TiltMtNeuralServoPlusMPC::ensureOneServoContinuity(double a_ref, int idx) const
{
  double a_now = gimbal_ctrl_cmd_.position[idx];
  // ensure the servo angle is continuous
  if (a_ref - a_now > M_PI)
    a_ref -= 2 * M_PI;
  else if (a_ref - a_now < -M_PI)
    a_ref += 2 * M_PI;

  return a_ref;
}

std::vector<double> nmpc::TiltMtNeuralServoPlusMPC::ensureAllServoContinuity(std::vector<double>& a_ref_vec) const
{
  for (int i = 0; i < joint_num_; i++)
    a_ref_vec[i] = ensureOneServoContinuity(a_ref_vec[i], i);

  return a_ref_vec;
}

void nmpc::TiltMtNeuralServoPlusMPC::printPhysicalParams()
{
  cout << "mass: " << robot_model_->getMass() << endl;
  cout << "gravity: " << robot_model_->getGravity() << endl;
  cout << "inertia: " << robot_model_->getInertia<Eigen::Matrix3d>() << endl;
  cout << "rotor num: " << robot_model_->getRotorNum() << endl;
  for (const auto& dir : robot_model_->getRotorDirection())
  {
    std::cout << "Key: " << dir.first << ", Value: " << dir.second << std::endl;
  }
  for (const auto& vec : robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>())
  {
    std::cout << "rotor origin from cog: " << vec << std::endl;
  }
  cout << "thrust lower limit: " << robot_model_->getThrustLowerLimit() << endl;
  cout << "thrust upper limit: " << robot_model_->getThrustUpperLimit() << endl;

  cout << "kq_kt_rate" << robot_model_->getMFRate() << endl;
  cout << "abs(kq_kt_rate)" << abs(robot_model_->getMFRate()) << endl;
}

bool nmpc::TiltMtNeuralServoPlusMPC::isMulDOFJointTrajPtEqual(const trajectory_msgs::MultiDOFJointTrajectoryPoint& a,
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

/* plugin registration */
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtNeuralServoPlusMPC, aerial_robot_control::ControlBase)
