//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_QD_SERVO_DIST_NMPC_CONTROLLER_H
#define TILT_QD_SERVO_DIST_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

// plugin
#include <pluginlib/class_loader.h>
#include "aerial_robot_control/nmpc/wrench_est/wrench_est_base.h"

#include "geometry_msgs/WrenchStamped.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoDistNMPC : public nmpc::TiltQdServoNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  ros::Publisher pub_disturb_wrench_;  // for disturbance wrench
  geometry_msgs::Vector3 dist_force_w_ = geometry_msgs::Vector3();
  geometry_msgs::Vector3 dist_torque_cog_ = geometry_msgs::Vector3();


  /* Nesting plugin */
  // For normal class, we can define the ClassLoader in the class constructor like
  // controller_loader_("aerial_robot_control", "aerial_robot_control::ControlBase"),
  // which is the way in aerial_robot_base.
  // But for the nested class, because the outer class is also a plugin, we cannot define any parameters for constructor.
  // Hence, please refer to the following code for a solution.
  boost::shared_ptr<pluginlib::ClassLoader<aerial_robot_control::nmpc::WrenchEstBase>> wrench_est_loader_ptr_;
  boost::shared_ptr<aerial_robot_control::nmpc::WrenchEstBase> wrench_est_ptr_;

  inline void initMPCSolverPtr() override
  {
    mpc_solver_ptr_ = std::make_unique<mpc_solver::TiltQdServoDistMdlMPCSolver>();
  }

  std::vector<double> meas2VecX() override;

  virtual void calcDisturbWrench() {
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

//    /* update */
//    wrench_est_ptr_->update(pos, target_pos, q, target_q);
//
//    /* get the disturbance wrench */
//    dist_force_w_ = wrench_est_ptr_->getDistForceW();
//    dist_torque_cog_ = wrench_est_ptr_->getDistTorqueCOG();

  };

  void callbackViz(const ros::TimerEvent& event) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_DIST_NMPC_CONTROLLER_H