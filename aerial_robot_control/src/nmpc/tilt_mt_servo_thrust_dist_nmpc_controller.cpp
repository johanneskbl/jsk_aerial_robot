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
  esc_telem_.initialize(nh_, motor_num_);

  sub_esc_telem_ = nh_.subscribe("esc_telem", 1, &TiltMtServoThrustDistNMPC::callbackESCTelem, this);
}

void nmpc::TiltMtServoThrustDistNMPC::initMPCCostW()
{
  /* control parameters with dynamic reconfigure */
  double Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Qt, Rtc_d, Rac_d;
  getParam<double>(mpc_nh_, "Qp_xy", Qp_xy, 300);
  getParam<double>(mpc_nh_, "Qp_z", Qp_z, 400);
  getParam<double>(mpc_nh_, "Qv_xy", Qv_xy, 10);
  getParam<double>(mpc_nh_, "Qv_z", Qv_z, 10);
  getParam<double>(mpc_nh_, "Qq_xy", Qq_xy, 300);
  getParam<double>(mpc_nh_, "Qq_z", Qq_z, 300);
  getParam<double>(mpc_nh_, "Qw_xy", Qw_xy, 5);
  getParam<double>(mpc_nh_, "Qw_z", Qw_z, 5);
  getParam<double>(mpc_nh_, "Qa", Qa, 1);
  getParam<double>(mpc_nh_, "Qt", Qt, 1);

  getParam<double>(mpc_nh_, "Rtc_d", Rtc_d, 1);
  getParam<double>(mpc_nh_, "Rac_d", Rac_d, 250);

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
  for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);

  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rtc_d, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);
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

  //  TODO: this should be set in flight_navigation
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

  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * ref_wrench_b;
  for (int i = 0; i < x_lambda.size() / 2; i++)
  {
    double a_ref = atan2(x_lambda(2 * i), x_lambda(2 * i + 1));
    x.at(13 + i) = a_ref;
    double ft_ref = sqrt(x_lambda(2 * i) * x_lambda(2 * i) + x_lambda(2 * i + 1) * x_lambda(2 * i + 1));
    x.at(13 + x_lambda.size() / 2 + i) = ft_ref;
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
  for (int i = 0; i < motor_num_; i++)
    bx0[13 + joint_num_ + i] = thrust_meas_[i];

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
        case Levels::RECONFIGURE_NMPC_Q_T: {
          for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qt);
          ROS_INFO_STREAM("change Qt for NMPC '" << config.Qt << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_TC_D: {
          for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rtc_d, false);
          ROS_INFO_STREAM("change Rtc_d for NMPC '" << config.Rtc_d << "'");
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

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustDistNMPC, aerial_robot_control::ControlBase);
