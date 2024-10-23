//
// Created by li-jinjie on 24-10-21.
//

#include "aerial_robot_control/nmpc/wrench_est/wrench_est_base.h"

namespace aerial_robot_control
{
namespace nmpc
{

void WrenchEstBase::initParams(ros::NodeHandle& nh_ctrl, double ctrl_loop_du)
{
  setCtrlLoopDu(ctrl_loop_du);
}

geometry_msgs::Vector3 WrenchEstBase::getDistForceW()
{
  return dist_force_w_;
}

geometry_msgs::Vector3 WrenchEstBase::getDistTorqueCOG()
{
  return dist_torque_cog_;
}

double WrenchEstBase::getCtrlLoopDu() const
{
  return ctrl_loop_du_;
}

void WrenchEstBase::setDistForceW(double x, double y, double z)
{
  dist_force_w_.x = x;
  dist_force_w_.y = y;
  dist_force_w_.z = z;
}

void WrenchEstBase::setDistTorqueCOG(double x, double y, double z)
{
  dist_torque_cog_.x = x;
  dist_torque_cog_.y = y;
  dist_torque_cog_.z = z;
}

void WrenchEstBase::setParamVerbose(bool param_verbose)
{
  param_verbose_ = param_verbose;
}

void WrenchEstBase::setCtrlLoopDu(double ctrl_loop_du)
{
  ctrl_loop_du_ = ctrl_loop_du;
}

template <class T>
void WrenchEstBase::getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
{
  nh.param<T>(param_name, param, default_value);

  if (param_verbose_)
    ROS_INFO_STREAM("[" << nh.getNamespace() << "] " << param_name << ": " << param);
}

}  // namespace nmpc
}  // namespace aerial_robot_control