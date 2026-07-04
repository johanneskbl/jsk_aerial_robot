#include "aerial_robot_control/differential_mpc/tilt_mt_servo_thrust_diff_mpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoThrustDiffMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                             boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                 boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                 double ctrl_loop_du)
{
  TiltMtServoNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  sub_esc_telem_ = nh_.subscribe("esc_telem", 1, &TiltMtServoThrustDiffMPC::callbackESCTelem, this);
}

void nmpc::TiltMtServoThrustDiffMPC::initGeneralParams()
{
  TiltMtServoNMPC::initGeneralParams();

  ros::NodeHandle motor_nh(nh_, "motor_info");
  getParam<double>(motor_nh, "krpm_square_to_thrust_ratio", krpm_square_to_thrust_ratio_, 0.0);
  getParam<double>(motor_nh, "krpm_square_to_thrust_bias", krpm_square_to_thrust_bias_, 0.0);

  thrust_meas_.resize(motor_num_, 0.0);
  internal_wrench_w_ = Eigen::VectorXd::Zero(6);
}

void nmpc::TiltMtServoThrustDiffMPC::initNMPCCostW()
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
  // getParam<double>(nmpc_nh, "Qfu", Qfu, 0);
  // getParam<double>(nmpc_nh, "Qtau", Qtau, 0);

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
  for (int i = 13; i < 13 + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);

  // Wrench state cost
  // NOTE: Include if `self.include_differential_allocation` is set to True in Python file
  // mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 0, Qfu);
  // mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 1, Qfu);
  // mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 2, Qfu);
  // mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 3, Qtau);
  // mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 4, Qtau);
  // mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 5, Qtau);

  // Control input cost
  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rtd_c, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rad_c, false);

  ROS_INFO("MPC cost W initialized:\n",
          "\tQp_xy=%f, Qp_z=%f, Qv_xy=%f, Qv_z=%f, Qq_xy=%f, Qq_z=%f, Qw_xy=%f, Qw_z=%f,\n",
          "\tQa=%f, Qt=%f, Rtd_c=%f, Rad_c=%f",
           Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Qt, Rtd_c, Rad_c);
}

void nmpc::TiltMtServoThrustDiffMPC::initNMPCConstraints()
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
  getParam<double>(nmpc_nh, "thrust_vel_min", thrust_ctrl_vel_min_, -100.0);
  getParam<double>(nmpc_nh, "thrust_vel_max", thrust_ctrl_vel_max_, 100.0);
  getParam<double>(nmpc_nh, "a_min", servo_angle_min_, -3.1416);
  getParam<double>(nmpc_nh, "a_max", servo_angle_max_, 3.1416);
  getParam<double>(nmpc_nh, "a_vel_min", servo_angle_vel_min_, -4.0);
  getParam<double>(nmpc_nh, "a_vel_max", servo_angle_vel_max_, 4.0);

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
    lbu[i] = thrust_ctrl_vel_min_;
    ubu[i] = thrust_ctrl_vel_max_;
  }
  for (int i = 0; i < joint_num_; i++)
  {
    lbu[motor_num_ + i] = servo_angle_vel_min_;
    ubu[motor_num_ + i] = servo_angle_vel_max_;
  }
  mpc_solver_ptr_->setConstraintsLbu(lbu);
  mpc_solver_ptr_->setConstraintsUbu(ubu);

  ROS_INFO("MPC constraints are set: \n",
           "\tvel_min: %f, vel_max: %f, w_min: %f, w_max: %f,\n",
           "\tthrust_min: %f, thrust_max: %f, thrust_vel_min: %f, thrust_vel_max: %f,\n",
           "\tservo_angle_min: %f, servo_angle_max: %f, servo_angle_vel_min: %f, servo_angle_vel_max: %f",
           vel_min_, vel_max_, body_rate_min, body_rate_max,
           thrust_ctrl_min_, thrust_ctrl_max_, thrust_ctrl_vel_min_, thrust_ctrl_vel_max_,
           servo_angle_min_, servo_angle_max_, servo_angle_vel_min_, servo_angle_vel_max_);
}

void nmpc::TiltMtServoThrustDiffMPC::callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg)
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

void nmpc::TiltMtServoThrustDiffMPC::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
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
  for (int i = 0; i < x_lambda.size() / 2; i++)
  {
    double a_ref = atan2(x_lambda(2 * i), x_lambda(2 * i + 1));  // Contains discontinuities
    x.at(13 + i) = a_ref;
    double ft_ref = sqrt(x_lambda(2 * i) * x_lambda(2 * i) + x_lambda(2 * i + 1) * x_lambda(2 * i + 1));
    x.at(13 + x_lambda.size() / 2 + i) = ft_ref;
  }

  // Internal wrench state reference
  // NOTE: Include if `self.include_differential_allocation` is set to True in Python file
  // x.at(13 + joint_num_ + motor_num_ + 0) = ref_wrench_b(0);
  // x.at(13 + joint_num_ + motor_num_ + 1) = ref_wrench_b(1);
  // x.at(13 + joint_num_ + motor_num_ + 2) = ref_wrench_b(2);
  // x.at(13 + joint_num_ + motor_num_ + 3) = ref_wrench_b(3);
  // x.at(13 + joint_num_ + motor_num_ + 4) = ref_wrench_b(4);
  // x.at(13 + joint_num_ + motor_num_ + 5) = ref_wrench_b(5);
}

std::vector<double> nmpc::TiltMtServoThrustDiffMPC::meas2VecX(bool is_ee_centric)
{
  auto bx0 = TiltMtServoNMPC::meas2VecX(is_ee_centric);

  for (int i = 0; i < motor_num_; i++)
    bx0[13 + joint_num_ + i] = thrust_meas_[i];

  // Internal wrench as state
  // NOTE: Include if `self.include_differential_allocation` is set to True in Python file
  // computeInternalWrenchW();
  // bx0[13 + joint_num_ + motor_num_ + 0] = internal_wrench_w_(0);
  // bx0[13 + joint_num_ + motor_num_ + 1] = internal_wrench_w_(1);
  // bx0[13 + joint_num_ + motor_num_ + 2] = internal_wrench_w_(2);
  // bx0[13 + joint_num_ + motor_num_ + 3] = internal_wrench_w_(3);
  // bx0[13 + joint_num_ + motor_num_ + 4] = internal_wrench_w_(4);
  // bx0[13 + joint_num_ + motor_num_ + 5] = internal_wrench_w_(5);

  return bx0;
}

void nmpc::TiltMtServoThrustDiffMPC::computeInternalWrenchW()
{
  if (alloc_mat_.size() == 0 || thrust_meas_.size() != motor_num_)
  {
    ROS_WARN("[NMPC] Allocation matrix is not set or thrust measurement size is not equal to motor number, we cannot compute internal wrench w, set it to zero!");
    internal_wrench_w_ = Eigen::VectorXd::Zero(6);
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
  internal_wrench_w_ = alloc_mat_ * allocation_matrix_input;
}


void nmpc::TiltMtServoThrustDiffMPC::cfgNMPCCallback(aerial_robot_control::NMPCConfig& config, uint32_t level)
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
        case Levels::RECONFIGURE_NMPC_Q_FU: {
          mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 0, config.Qfu);
          mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 1, config.Qfu);
          mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 2, config.Qfu);
          ROS_INFO_STREAM("change Qfu for NMPC '" << config.Qfu << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_TAU: {
          mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 3, config.Qtau);
          mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 4, config.Qtau);
          mpc_solver_ptr_->setCostWDiagElement(13 + joint_num_ + motor_num_ + 5, config.Qtau);
          ROS_INFO_STREAM("change Qtau for NMPC '" << config.Qtau << "'");
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
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustDiffMPC, aerial_robot_control::ControlBase);
