//
// Created by li-jinjie on 24-10-21.
//

#ifndef WRENCH_EST_BASE_H
#define WRENCH_EST_BASE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>

#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>

#include <pluginlib/class_list_macros.h>

namespace aerial_robot_control
{
namespace nmpc
{

class WrenchEstBase
{
public:
  virtual ~WrenchEstBase() = default;

  virtual inline void initParams(ros::NodeHandle& nh_ctrl, double ctrl_loop_du)
  {
    setCtrlLoopDu(ctrl_loop_du);
  }

  virtual inline void update(const tf::Vector3& pos, const tf::Vector3& pos_ref, const tf::Quaternion& q,
                      const tf::Quaternion& q_ref) {};

  /* getter */
  inline geometry_msgs::Vector3 getDistForceW()
  {
    return dist_force_w_;
  }
  inline geometry_msgs::Vector3 getDistTorqueCOG()
  {
    return dist_torque_cog_;
  }
  inline double getCtrlLoopDu() const
  {
    return ctrl_loop_du_;
  }

  /* setter */
  inline void setDistForceW(double x, double y, double z)
  {
    dist_force_w_.x = x;
    dist_force_w_.y = y;
    dist_force_w_.z = z;
  }
  inline void setDistTorqueCOG(double x, double y, double z)
  {
    dist_torque_cog_.x = x;
    dist_torque_cog_.y = y;
    dist_torque_cog_.z = z;
  }
  inline void setParamVerbose(bool param_verbose)
  {
    param_verbose_ = param_verbose;
  }
  inline void setCtrlLoopDu(double ctrl_loop_du)
  {
    ctrl_loop_du_ = ctrl_loop_du;
  }

protected:
  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);

    if (param_verbose_)
      ROS_INFO_STREAM("[" << nh.getNamespace() << "] " << param_name << ": " << param);
  }

private:
  double ctrl_loop_du_ = 0.01;
  bool param_verbose_ = false;

  geometry_msgs::Vector3 dist_force_w_;     // disturbance force in world frame
  geometry_msgs::Vector3 dist_torque_cog_;  // disturbance torque in cog frame
};

}  // namespace nmpc
}  // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::WrenchEstBase, aerial_robot_control::nmpc::WrenchEstBase);

#endif  // WRENCH_EST_BASE_H
