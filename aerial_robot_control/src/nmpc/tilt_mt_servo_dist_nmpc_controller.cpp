//
// Created by lijinjie on 24/07/18.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_dist_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoDistNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_du)
{
  TiltMtServoNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  getParam<bool>(control_nh_, "if_use_est_wrench_4_control", if_use_est_wrench_4_control_, false);

  /* Publishers */
  pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("ext_wrench_est/value", 1);
}

void nmpc::TiltMtServoDistNMPC::initPlugins()
{
  /* Control I-term as disturbance estimation of the model error */
  wrench_est_i_term_.initialize(nh_, robot_model_, estimator_, ctrl_loop_du_);

  /* Plugin: External Wrench Estimator */
  wrench_est_loader_ptr_ = boost::make_shared<pluginlib::ClassLoader<aerial_robot_control::WrenchEstActuatorMeasBase>>(
      "aerial_robot_control", "aerial_robot_control::WrenchEstActuatorMeasBase");
  try
  {
    // 1. Read the plugin name from the parameter server
    std::string wrench_estimator_name;
    nh_.param("wrench_estimator_name", wrench_estimator_name, std::string("aerial_robot_control::WrenchEstNone"));

    // 2. Load the plugin
    wrench_est_ptr_ = wrench_est_loader_ptr_->createInstance(wrench_estimator_name);
    wrench_est_ptr_->initialize(nh_, robot_model_, estimator_, ctrl_loop_du_);
    ROS_INFO("load wrench estimator plugin: %s", wrench_estimator_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    std::string err_msg = "wrench_est_plugin: The plugin failed to load for some reason. Error: " + std::string(ex.what());
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }
}

void nmpc::TiltMtServoDistNMPC::initAllocMat()
{
  TiltMtServoNMPC::initAllocMat();
  wrench_est_ptr_->init_alloc_mtx(alloc_mat_, alloc_mat_pinv_);
}

void nmpc::TiltMtServoDistNMPC::initMPCPhysicalParams()
{
  TiltMtServoNMPC::initMPCPhysicalParams();
  idx_p_dist_end_ = idx_p_phys_end_ + 6;
}

void nmpc::TiltMtServoDistNMPC::resetPlugins()
{
  wrench_est_i_term_.reset();
  wrench_est_ptr_->reset();
  was_warmup_ = true;
}

std::vector<double> nmpc::TiltMtServoDistNMPC::meas2VecX(bool is_ee_centric)
{
  std::vector<double> bx0 = TiltMtServoNMPC::meas2VecX(is_ee_centric);

  /* Disturbance rejection */
  geometry_msgs::Vector3 external_force_w;   // default constructor: 0, 0, 0
  geometry_msgs::Vector3 external_torque_b;  // default constructor: 0, 0, 0

  if (if_use_est_wrench_4_control_)
  {
    external_force_w = wrench_est_ptr_->getDistForceW();
    external_torque_b = wrench_est_ptr_->getDistTorqueCOG();
  }

  const int disturbance_index = 13 + joint_num_;
  bx0[disturbance_index + 0] = external_force_w.x;
  bx0[disturbance_index + 1] = external_force_w.y;
  bx0[disturbance_index + 2] = external_force_w.z;
  bx0[disturbance_index + 3] = external_torque_b.x;
  bx0[disturbance_index + 4] = external_torque_b.y;
  bx0[disturbance_index + 5] = external_torque_b.z;

  return bx0;
}

/**
 * @brief Update the I-term with the residual between the measured and the predicted velocities
 * The I-term estimates the model error of the wrench from the velocity residual (a_real - a_model) * T over one control
 * period T.
 * NOTE: The predicted state of the solver always describes the Body frame, independent of the Tracking frame.
 * NOTE: Modified from position error to velocity error since velocity captures the planned motion and model error better
 * With position error and Body state estimation, the I-term introduces instability!
 */
void nmpc::TiltMtServoDistNMPC::updateITerm()
{
  /* Get the measured velocities */
  tf::Vector3 vel_w = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Vector3 omega_b = estimator_->getAngularVel(Frame::COG, estimate_mode_);

  /* Get the time of the current measurement relative to the first node of the last solution */
  // The last solution starts at the actuation time of the last solve() call, i.e., the age of the state estimate after
  // its call. So the current measurement is the time since that call minus the age. For a negative time, the first
  // shooting interval is extrapolated backwards.
  const double t_since_solve = ros::Time::now().toSec() - t_mpc_prev_timestep_;
  if (t_since_solve <= 0.0 || t_since_solve > 2.0 * t_mpc_samp_)
  {
    ROS_WARN_THROTTLE(1.0, "[MPC] I-term is not updated, since the last solution is %.3f s old.", t_since_solve);
    return;
  }
  const double t_target = t_since_solve - body_estimator_.age_;

  /* Find the shooting interval [t_k, t_k + dt_k] of the last solution that contains the current measurement */
  int k = 0;
  double t_k = 0.0;
  while (k < mpc_solver_ptr_->NN_ - 1 && t_target > t_k + mpc_solver_ptr_->getTimeStep(k))
  {
    t_k += mpc_solver_ptr_->getTimeStep(k);
    k++;
  }
  const double ratio = (t_target - t_k) / mpc_solver_ptr_->getTimeStep(k);  // Negative for backward extrapolation

  /* Get the predicted velocities at the time of the current measurement */
  // The velocities are linear in time for a constant acceleration on the shooting interval, so the interpolation
  // between its two nodes is exact in that case
  const std::vector<double>& xo_0 = mpc_solver_ptr_->xo_.at(k);
  const std::vector<double>& xo_1 = mpc_solver_ptr_->xo_.at(k + 1);
  tf::Vector3 vel_w_x0(xo_0.at(3), xo_0.at(4), xo_0.at(5));  // idx: 3,4,5 = vx,vy,vz
  tf::Vector3 vel_w_x1(xo_1.at(3), xo_1.at(4), xo_1.at(5));
  tf::Vector3 target_vel_w = vel_w_x0 + (vel_w_x1 - vel_w_x0) * ratio;

  tf::Vector3 omega_b_x0(xo_0.at(10), xo_0.at(11), xo_0.at(12));  // idx: 10,11,12 = wx,wy,wz
  tf::Vector3 omega_b_x1(xo_1.at(10), xo_1.at(11), xo_1.at(12));
  tf::Vector3 target_omega_b = omega_b_x0 + (omega_b_x1 - omega_b_x0) * ratio;

  /* Update I-term */
  wrench_est_i_term_.update(target_vel_w, target_omega_b, vel_w, omega_b);
}

void nmpc::TiltMtServoDistNMPC::updateDisturbWrench() const
{
  auto vel_w = estimator_->getVel(Frame::COG, estimate_mode_);
  auto omega_b = estimator_->getAngularVel(Frame::COG, estimate_mode_);

  wrench_est_ptr_->update(vel_w, omega_b);
}

/**
 * @brief Prepare the parameters of the MPC solver and update the estimators
 */
void nmpc::TiltMtServoDistNMPC::prepareMPCParams()
{
  TiltMtServoNMPC::prepareMPCParams();

  if (!is_warmup_)
  {
    // Estimators are not updated during warm-up to avoid wind-up
    updateDisturbWrench();

    // This form of I-term can always be activated, no need to be shut down when the external wrench appears.
    if (!was_warmup_)  // Skip one solution after warmup ended since its state is not predicted properly yet
      updateITerm();
  }
  was_warmup_ = is_warmup_;
  auto mdl_error_force_w = wrench_est_i_term_.getDistForceW();
  auto mdl_error_torque_b = wrench_est_i_term_.getDistTorqueCOG();

  std::vector<int> idx = { idx_p_phys_end_ + 1, idx_p_phys_end_ + 2, idx_p_phys_end_ + 3,
                           idx_p_phys_end_ + 4, idx_p_phys_end_ + 5, idx_p_phys_end_ + 6 };
  std::vector<double> p = { mdl_error_force_w.x,  mdl_error_force_w.y,  mdl_error_force_w.z,
                            mdl_error_torque_b.x, mdl_error_torque_b.y, mdl_error_torque_b.z };
  mpc_solver_ptr_->setParamSparseAllStages(idx, p);
}

/**
 * @brief Publish the recording and the estimated external wrench used by the controller
 */
void nmpc::TiltMtServoDistNMPC::publishRecording()
{
  TiltMtServoNMPC::publishRecording();

  if (!is_warmup_)
    pubDisturbWrench();
}

void nmpc::TiltMtServoDistNMPC::pubDisturbWrench() const
{
  geometry_msgs::WrenchStamped dist_wrench;
  dist_wrench.header.frame_id = nh_.getNamespace().substr(1) + "/cog";

  auto ext_force_w = wrench_est_ptr_->getDistForceW();
  dist_wrench.wrench.torque = wrench_est_ptr_->getDistTorqueCOG();

  tf::Matrix3x3 rot_mtx_cog2w = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 tf_dist_force_w = tf::Vector3(ext_force_w.x, ext_force_w.y, ext_force_w.z);
  tf::Vector3 tf_dist_force_cog = rot_mtx_cog2w.inverse() * tf_dist_force_w;
  dist_wrench.wrench.force.x = tf_dist_force_cog.x();
  dist_wrench.wrench.force.y = tf_dist_force_cog.y();
  dist_wrench.wrench.force.z = tf_dist_force_cog.z();

  dist_wrench.header.stamp = ros::Time::now();
  pub_disturb_wrench_.publish(dist_wrench);
}

/* Plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoDistNMPC, aerial_robot_control::ControlBase);
