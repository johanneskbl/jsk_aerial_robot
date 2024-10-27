// -*- mode: c++ -*-

#include "beetle/beetle_omni_navigation.h"

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

void BeetleOmniNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  NMPCManager::initialize(nh, nhp, robot_model, estimator);

  /* subscriber for trajectory*/
  sub_set_rpy_ = nh_.subscribe("set_rpy", 5, &BeetleOmniNavigator::callbackSetRPY, this);
  sub_set_ref_x_u_ = nh_.subscribe("set_ref_x_u", 5, &BeetleOmniNavigator::callbackSetRefXU, this);
  sub_set_traj_ = nh_.subscribe("set_ref_traj", 5, &BeetleOmniNavigator::callbackSetRefTraj, this);
}

void BeetleOmniNavigator::update()
{
  NMPCManager::update();

  if (!trajectory_mode_)
  {
    /* point mode --> set target */
    tf::Vector3 target_pos = getTargetPos();
    tf::Vector3 target_vel = getTargetVel();
    tf::Vector3 target_rpy = getTargetRPY();
    tf::Quaternion target_quat;
    target_quat.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
    tf::Vector3 target_omega = getTargetOmega();

    setXrUrRef(target_pos, target_vel, tf::Vector3(0, 0, 0), target_quat, target_omega, tf::Vector3(0, 0, 0), -1);

    return;
  }

  if (trajectory_mode_)
  {
    // check if the trajectory info is received. If not, set the target to the current position with no attitude.
    if (ros::Time::now() - receive_time_ > ros::Duration(0.1))
    {
      ROS_INFO("Trajectory tracking mode is off!");
      trajectory_mode_ = false;
      tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);

      setTargetPosX((float)pos.x());
      setTargetPosY((float)pos.y());
      setTargetPosZ((float)pos.z());
      setTargetVelX(0.0);
      setTargetVelY(0.0);
      setTargetVelZ(0.0);
      setTargetRoll(0.0);
      setTargetPitch(0.0);
      setTargetYaw(0.0);
      setTargetOmegaX(0.0);
      setTargetOmegaY(0.0);
      setTargetOmegaZ(0.0);
    }
  }
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
void BeetleOmniNavigator::setXrUrRef(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                     const tf::Vector3& ref_acc_i, const tf::Quaternion& ref_quat_ib,
                                     const tf::Vector3& ref_omega_b, const tf::Vector3& ref_ang_acc_b,
                                     const int& horizon_idx)
{
  int& NX = NX_;
  int& NU = NU_;
  int& NN = NN_;

  /* calculate the reference wrench in the body frame */
  Eigen::VectorXd acc_with_g_i(3);
  acc_with_g_i(0) = ref_acc_i.x();
  acc_with_g_i(1) = ref_acc_i.y();
  acc_with_g_i(2) = ref_acc_i.z() + robot_model_->getGravity()[2];  // add gravity

  // coordinate transformation
  tf::Quaternion q_bi = ref_quat_ib.inverse();
  Eigen::Matrix3d rot_bi;
  tf::matrixTFToEigen(tf::Transform(q_bi).getBasis(), rot_bi);
  Eigen::VectorXd ref_acc_b = rot_bi * acc_with_g_i;

  Eigen::VectorXd ref_wrench_b(6);
  ref_wrench_b(0) = ref_acc_b(0) * robot_model_->getMass();
  ref_wrench_b(1) = ref_acc_b(1) * robot_model_->getMass();
  ref_wrench_b(2) = ref_acc_b(2) * robot_model_->getMass();

  auto inertia_mtx = robot_model_->getInertia<Eigen::Matrix3d>();
  ref_wrench_b(3) = ref_ang_acc_b.x() * inertia_mtx(0, 0);
  ref_wrench_b(4) = ref_ang_acc_b.y() * inertia_mtx(1, 1);
  ref_wrench_b(5) = ref_ang_acc_b.z() * inertia_mtx(2, 2);

  /* calculate X U from ref, aka. control allocation */
  std::vector<double> x(NX);
  std::vector<double> u(NU);
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

void BeetleOmniNavigator::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                       const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                                       const VectorXd& ref_wrench_b, vector<double>& x, vector<double>& u) const
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
  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * ref_wrench_b;
  for (int i = 0; i < x_lambda.size() / 2; i++)
  {
    double a_ref = atan2(x_lambda(2 * i), x_lambda(2 * i + 1));
    x.at(13 + i) = a_ref;
    double ft_ref = sqrt(x_lambda(2 * i) * x_lambda(2 * i) + x_lambda(2 * i + 1) * x_lambda(2 * i + 1));
    u.at(i) = ft_ref;
  }
}

void BeetleOmniNavigator::callbackSetRPY(const spinal::DesireCoordConstPtr& msg)
{
  // add a check to avoid the singular point for euler angle
  if (msg->pitch == M_PI / 2.0 or msg->pitch == -M_PI / 2.0)
  {
    ROS_WARN(
        "The pitch angle is set to PI/2 or -PI/2, which is a singular point for euler angle."
        " Please set other values for the pitch angle.");
    return;
  }

  setTargetRoll(msg->roll);
  setTargetPitch(msg->pitch);
  setTargetYaw(msg->yaw);
}

void BeetleOmniNavigator::callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg)
{
  if (getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
  {
    ROS_WARN_THROTTLE(1, "The robot is taking off, so the reference trajectory will be ignored!");
    return;
  }

  x_u_ref_ = *msg;
  receive_time_ = ros::Time::now();

  if (!trajectory_mode_)
  {
    ROS_INFO("Trajectory tracking mode is on!");
    trajectory_mode_ = true;
  }
}

void BeetleOmniNavigator::callbackSetRefTraj(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  if (msg->points.size() != NN_ + 1)
    ROS_WARN("The length of the trajectory is not equal to the prediction horizon! Cannot use the trajectory!");

  if (getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
  {
    ROS_WARN_THROTTLE(1, "The robot is taking off, so the reference trajectory will be ignored!");
    return;
  }

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

  callbackSetRefXU(aerial_robot_msgs::PredXUConstPtr(new aerial_robot_msgs::PredXU(x_u_ref_)));
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::BeetleOmniNavigator, aerial_robot_navigation::BaseNavigator);