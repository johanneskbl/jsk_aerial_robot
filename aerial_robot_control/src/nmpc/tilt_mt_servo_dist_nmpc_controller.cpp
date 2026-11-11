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

void nmpc::TiltMtServoDistNMPC::resetPlugins()
{
  wrench_est_i_term_.reset();
  wrench_est_ptr_->reset();
}

void nmpc::TiltMtServoDistNMPC::initPlugins()
{
  /* I-term is always loaded */
  wrench_est_i_term_.initialize(nh_, robot_model_, estimator_, ctrl_loop_du_);

  /* Plugin: wrench estimator */
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

/**
 * @brief Update the I-term with the error between the predicted and the measured state in Body frame
 * NOTE: The predicted state of the solver always describes the Body frame, independent of the Tracking frame.
 */
void nmpc::TiltMtServoDistNMPC::updateITerm()
{
  /* HANDLING MODEL ERROR */
  /* Get the current state */
  tf::Vector3 pos_w = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Quaternion q_wb = estimator_->getQuat(Frame::COG, estimate_mode_);

  /* Get the target state */
  // Note that the target state is the estimated state from the MPC solver.
  tf::Vector3 pos_w_x0(mpc_solver_ptr_->xo_.at(0).at(0), mpc_solver_ptr_->xo_.at(0).at(1),
                       mpc_solver_ptr_->xo_.at(0).at(2));
  tf::Vector3 pos_w_x1(mpc_solver_ptr_->xo_.at(1).at(0), mpc_solver_ptr_->xo_.at(1).at(1),
                       mpc_solver_ptr_->xo_.at(1).at(2));
  tf::Vector3 target_pos_w = pos_w_x0 + (pos_w_x1 - pos_w_x0) * t_mpc_samp_ / t_mpc_step_;

  tf::Quaternion q_wb_x0(mpc_solver_ptr_->xo_.at(0).at(7), mpc_solver_ptr_->xo_.at(0).at(8),
                         mpc_solver_ptr_->xo_.at(0).at(9), mpc_solver_ptr_->xo_.at(0).at(6));
  q_wb_x0.normalize();
  tf::Quaternion q_wb_x1(mpc_solver_ptr_->xo_.at(1).at(7), mpc_solver_ptr_->xo_.at(1).at(8),
                         mpc_solver_ptr_->xo_.at(1).at(9), mpc_solver_ptr_->xo_.at(1).at(6));
  q_wb_x1.normalize();
  tf::Quaternion target_q_wb = q_wb_x0.slerp(q_wb_x1, t_mpc_samp_ / t_mpc_step_);

  /* Update I-term */
  wrench_est_i_term_.update(target_pos_w, target_q_wb, pos_w, q_wb);
}

void nmpc::TiltMtServoDistNMPC::initMPCPhysicalParams()
{
  TiltMtServoNMPC::initMPCPhysicalParams();
  idx_p_dist_end_ = idx_p_phys_end_ + 6;
}

/**
 * @brief Prepare the parameters of the MPC solver and update the estimators
 */
void nmpc::TiltMtServoDistNMPC::prepareMPCParams()
{
  TiltMtServoNMPC::prepareMPCParams();

  if (!is_warmup_)
  {
    // Estimators are not updated during warm-up, to avoid wind-up
    updateDisturbWrench();

    // This form of I-term can always be activated, no need to be shut down when the external wrench appears.
    updateITerm();
  }
  auto mdl_error_force_w = wrench_est_i_term_.getDistForceW();
  auto mdl_error_torque_b = wrench_est_i_term_.getDistTorqueCOG();

  std::vector<int> idx = { idx_p_phys_end_ + 1, idx_p_phys_end_ + 2, idx_p_phys_end_ + 3,
                           idx_p_phys_end_ + 4, idx_p_phys_end_ + 5, idx_p_phys_end_ + 6 };
  std::vector<double> p = { mdl_error_force_w.x,  mdl_error_force_w.y,  mdl_error_force_w.z,
                            mdl_error_torque_b.x, mdl_error_torque_b.y, mdl_error_torque_b.z };
  mpc_solver_ptr_->setParamSparseAllStages(idx, p);
}

std::vector<double> nmpc::TiltMtServoDistNMPC::meas2VecX(bool is_ee_centric)
{
  std::vector<double> bx0 = TiltMtServoNMPC::meas2VecX(is_ee_centric);

  /* Disturbance rejection */
  geometry_msgs::Vector3 external_force_w;   // default: 0, 0, 0
  geometry_msgs::Vector3 external_torque_b;  // default: 0, 0, 0

  if (if_use_est_wrench_4_control_)
  {
    external_force_w = wrench_est_ptr_->getDistForceW();
    external_torque_b = wrench_est_ptr_->getDistTorqueCOG();
  }

  bx0[13 + joint_num_ + 0] = external_force_w.x;
  bx0[13 + joint_num_ + 1] = external_force_w.y;
  bx0[13 + joint_num_ + 2] = external_force_w.z;
  bx0[13 + joint_num_ + 3] = external_torque_b.x;
  bx0[13 + joint_num_ + 4] = external_torque_b.y;
  bx0[13 + joint_num_ + 5] = external_torque_b.z;

  return bx0;
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

void nmpc::TiltMtServoDistNMPC::updateDisturbWrench() const
{
  auto vel_w = estimator_->getVel(Frame::COG, estimate_mode_);
  auto omega_b = estimator_->getAngularVel(Frame::COG, estimate_mode_);

  wrench_est_ptr_->update(vel_w, omega_b);
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

void nmpc::TiltMtServoDistNMPC::initAllocMat()
{
  TiltMtServoNMPC::initAllocMat();
  wrench_est_ptr_->init_alloc_mtx(alloc_mat_, alloc_mat_pinv_);
}

/* Plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoDistNMPC, aerial_robot_control::ControlBase);
