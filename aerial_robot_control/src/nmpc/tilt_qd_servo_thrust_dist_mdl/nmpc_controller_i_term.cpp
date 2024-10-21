//
// Created by li-jinjie on 24-9-28.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_thrust_dist_mdl/nmpc_controller_i_term.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoThrustNMPCwITerm::initParams()
{
  TiltQdServoThrustDistNMPC::initParams();

  ros::NodeHandle control_nh(nh_, "controller");

  wrench_est_i_term_.initParams(control_nh, ctrl_loop_du_);
}

void nmpc::TiltQdServoThrustNMPCwITerm::calcDisturbWrench()
{
  /* get the current state */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Quaternion q = estimator_->getQuat(Frame::COG, estimate_mode_);

  /* get the target state */
  tf::Vector3 target_pos;
  tf::Quaternion target_q;
  if (is_traj_tracking_)
  {
    target_pos.setX(x_u_ref_.x.data.at(0));
    target_pos.setY(x_u_ref_.x.data.at(1));
    target_pos.setZ(x_u_ref_.x.data.at(2));

    target_q.setW(x_u_ref_.x.data.at(6));
    target_q.setX(x_u_ref_.x.data.at(7));
    target_q.setY(x_u_ref_.x.data.at(8));
    target_q.setZ(x_u_ref_.x.data.at(9));
  }
  else
  {
    target_pos = navigator_->getTargetPos();

    tf::Vector3 target_rpy = navigator_->getTargetRPY();
    target_q.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
  }

  /* update I term */
  wrench_est_i_term_.update(pos, target_pos, q, target_q);

  /* get the disturbance wrench */
  dist_force_w_ = wrench_est_i_term_.getDistForceW();
  dist_torque_cog_ = wrench_est_i_term_.getDistTorqueCOG();
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoThrustNMPCwITerm, aerial_robot_control::ControlBase);
