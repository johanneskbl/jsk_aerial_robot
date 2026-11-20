//
// Created by jinjie on 24/07/31.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_thrust_dist_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoThrustDistNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                 boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                 double ctrl_loop_du)
{
  TiltMtServoDistNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  /* Thrust measurement */
  sub_esc_telem_ = nh_.subscribe("esc_telem", 1, &TiltMtServoThrustDistNMPC::callbackESCTelem, this);
  esc_telem_.initialize(nh_, motor_num_);
  thrust_estimator_.initialize(nh_, motor_num_);
}

void nmpc::TiltMtServoThrustDistNMPC::initActuatorStates()
{
  TiltMtServoDistNMPC::initActuatorStates();

  thrust_meas_.resize(motor_num_, 0.0);
  thrust_est_.resize(motor_num_, 0.0);
  crc_errors_.resize(motor_num_, 0);
}

void nmpc::TiltMtServoThrustDistNMPC::initMPCCostW()
{
  TiltMtServoDistNMPC::initMPCCostW();

  /* Thrust-related cost weights */
  // NOTE: Rtc_d now describes the difference between the thrust command and the measured thrust, so it is not the same as Rt in TiltMtServoDistNMPC.
  double Qt, Rtc_d;
  getParam<double>(mpc_nh_, "Qt", Qt, 1);
  getParam<double>(mpc_nh_, "Rtc_d", Rtc_d, 1);

  for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);

  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rtc_d, false);
}

void nmpc::TiltMtServoThrustDistNMPC::initMPCConstraints()
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
  idxbx_desired.resize(6 + joint_num_ + motor_num_);
  for (int i = 0; i < joint_num_; i++)
  {
    idxbx_desired[6 + i] = 13 + i;  // Joint angles
  }
  for (int i = 0; i < motor_num_; i++)
  {
    idxbx_desired[6 + joint_num_ + i] = 13 + joint_num_ + i;  // Thrusts
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

/**
 * @brief Update the measured thrust from the valid frames of the ESC telemetry
 */
void nmpc::TiltMtServoThrustDistNMPC::callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg)
{
  esc_telem_.update(*msg, thrust_meas_, crc_errors_);
}

/**
 * @brief Construct and allocate the state and control inputs from the referenced wrench
 * The servo angles and thrusts are states, so their references are set in x and the control input reference is zero.
 * @param ref_pos_w Position of the origin of T, expressed in W
 * @param ref_vel_w Linear velocity of the origin of T, expressed in W
 * @param ref_quat_wt Orientation of T w.r.t. W
 * @param ref_omega_t Angular velocity of T w.r.t. W, expressed in T
 * @param ref_wrench_b Reference wrench in the Body frame
 * @param x State vector to be filled
 * @param u Control vector to be filled
 */
void nmpc::TiltMtServoThrustDistNMPC::allocateToXU(const tf::Vector3& ref_pos_w, const tf::Vector3& ref_vel_w,
                                                   const tf::Quaternion& ref_quat_wt, const tf::Vector3& ref_omega_t,
                                                   const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                                                   std::vector<double>& u)
{
  TiltMtServoDistNMPC::allocateToXU(ref_pos_w, ref_vel_w, ref_quat_wt, ref_omega_t, ref_wrench_b, x, u);

  // Thrust is a state, so its reference is set in x and the control input reference is zeroed
  for (int i = 0; i < motor_num_; i++)
  {
    x.at(13 + joint_num_ + i) = u.at(i);
    u.at(i) = 0.0;
  }
}

std::vector<double> nmpc::TiltMtServoThrustDistNMPC::meas2VecX(bool is_ee_centric)
{
  /* Disturbance rejection */
  geometry_msgs::Vector3 external_force_w;   // default: 0, 0, 0
  geometry_msgs::Vector3 external_torque_b;  // default: 0, 0, 0

  if (if_use_est_wrench_4_control_)
  {
    external_force_w = wrench_est_ptr_->getDistForceW();
    external_torque_b = wrench_est_ptr_->getDistTorqueCOG();
  }

  // NOTE: Skip TiltMtServoDistNMPC::meas2VecX() since the thrust states come before the external wrench states
  auto bx0 = TiltMtServoNMPC::meas2VecX(is_ee_centric);

  esc_telem_.checkAlive();  // Only logging
  thrust_estimator_.estimate(esc_telem_, thrust_meas_, flight_cmd_.base_thrust, thrust_est_);
  for (int i = 0; i < motor_num_; i++)
    bx0[13 + joint_num_ + i] = thrust_est_[i];

  bx0[13 + joint_num_ + motor_num_ + 0] = external_force_w.x;
  bx0[13 + joint_num_ + motor_num_ + 1] = external_force_w.y;
  bx0[13 + joint_num_ + motor_num_ + 2] = external_force_w.z;
  bx0[13 + joint_num_ + motor_num_ + 3] = external_torque_b.x;
  bx0[13 + joint_num_ + motor_num_ + 4] = external_torque_b.y;
  bx0[13 + joint_num_ + motor_num_ + 5] = external_torque_b.z;
  return bx0;
}

void nmpc::TiltMtServoThrustDistNMPC::callbackMPCcfg(aerial_robot_control::NMPCConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (!config.nmpc_flag)
    return;

  switch (level)
  {
    case Levels::RECONFIGURE_NMPC_Q_T: {
          for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
        mpc_solver_ptr_->setCostWDiagElement(i, config.Qt);
      ROS_INFO_STREAM("Change Qt for NMPC '" << config.Qt << "'");
      break;
    }
    case Levels::RECONFIGURE_NMPC_R_TC_D: {
      for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
        mpc_solver_ptr_->setCostWDiagElement(i, config.Rtc_d, false);
      ROS_INFO_STREAM("Change Rtc_d for NMPC '" << config.Rtc_d << "'");
      break;
    }
    case Levels::RECONFIGURE_NMPC_R_T: {
      // Rt is replaced by Rtc_d in this model, so it must not overwrite the weight of the thrust command inputs
      ROS_INFO_STREAM("The setting variable is not in the list!");
      break;
    }
    default: {
      TiltMtServoDistNMPC::callbackMPCcfg(config, level);
      break;
    }
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustDistNMPC, aerial_robot_control::ControlBase);
