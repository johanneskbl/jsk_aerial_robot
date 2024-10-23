//
// Created by li-jinjie on 24-10-21.
//

#ifndef WRENCH_EST_BASE_H
#define WRENCH_EST_BASE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>

namespace aerial_robot_control
{
namespace nmpc
{

class WrenchEstBase
{
public:
  WrenchEstBase() = default;
  virtual ~WrenchEstBase() = default;

  virtual void initParams(ros::NodeHandle& nh_ctrl, double ctrl_loop_du);

  virtual void update(const tf::Vector3& pos, const tf::Vector3& pos_ref, const tf::Quaternion& q,
                      const tf::Quaternion& q_ref) {};

  /* getter */
  geometry_msgs::Vector3 getDistForceW();
  geometry_msgs::Vector3 getDistTorqueCOG();
  double getCtrlLoopDu() const;

  /* setter */
  void setDistForceW(double x, double y, double z);
  void setDistTorqueCOG(double x, double y, double z);
  void setParamVerbose(bool param_verbose);
  void setCtrlLoopDu(double ctrl_loop_du);

protected:
  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value);

private:
  double ctrl_loop_du_ = 0.01;
  bool param_verbose_ = false;

  geometry_msgs::Vector3 dist_force_w_;     // disturbance force in world frame
  geometry_msgs::Vector3 dist_torque_cog_;  // disturbance torque in cog frame
};

}  // namespace nmpc
}  // namespace aerial_robot_control

#endif  // WRENCH_EST_BASE_H
