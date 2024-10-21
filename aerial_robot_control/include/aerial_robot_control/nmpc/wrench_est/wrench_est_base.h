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

namespace aerial_robot_control
{
namespace nmpc
{

class WrenchEstBase
{
public:
  virtual ~WrenchEstBase() = default;

  virtual void initParams(ros::NodeHandle& nh_ctrl, double ctrl_loop_du) = 0;
  virtual void update(const tf::Vector3& pos, const tf::Vector3& pos_ref, const tf::Quaternion& q,
                      const tf::Quaternion& q_ref) = 0;

  /* getter */
  geometry_msgs::Vector3 getDistForceW()
  {
    return dist_force_w_;
  }
  geometry_msgs::Vector3 getDistTorqueCOG()
  {
    return dist_torque_cog_;
  }

  /* setter */
  inline void setParamVerbose(bool param_verbose)
  {
    param_verbose_ = param_verbose;
  }

protected:
  double ctrl_loop_du_ = 0.01;
  bool param_verbose_ = false;

  geometry_msgs::Vector3 dist_force_w_;     // disturbance force in world frame
  geometry_msgs::Vector3 dist_torque_cog_;  // disturbance torque in cog frame

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);

    if (param_verbose_)
      ROS_INFO_STREAM("[" << nh.getNamespace() << "] " << param_name << ": " << param);
  }
};

}  // namespace nmpc
}  // namespace aerial_robot_control

#endif  // WRENCH_EST_BASE_H
