// -*- mode: c++ -*-

#pragma once

#include <diagnostic_msgs/KeyValue.h>
#include "beetle/model/beetle_omni_robot_model.h"
#include "aerial_robot_control/nmpc_manager.h"

/* protocol */
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "spinal/DesireCoord.h"

/* TODO: action */
#include "actionlib/server/simple_action_server.h"
#include "aerial_robot_msgs/TrackTrajAction.h"
#include "aerial_robot_msgs/TrackTrajFeedback.h"
#include "aerial_robot_msgs/TrackTrajGoal.h"
#include "aerial_robot_msgs/TrackTrajResult.h"

namespace aerial_robot_navigation
{

class BeetleOmniNavigator : public NMPCManager
{
public:
  BeetleOmniNavigator() : NMPCManager() {};
  ~BeetleOmniNavigator() override = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator) override;

  void update() override;

protected:
  void setXrUrRef(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i, const tf::Vector3& ref_acc_i,
                  const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b, const tf::Vector3& ref_ang_acc_b,
                  const int& horizon_idx);
  virtual void allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                            const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                            const VectorXd& ref_wrench_b, vector<double>& x, vector<double>& u) const;

  void callbackSetRPY(const spinal::DesireCoordConstPtr& msg);
  void callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg);  // TODO: override?
  void callbackSetRefTraj(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);

private:

  /* subscribers */
  ros::Subscriber sub_set_rpy_;
  ros::Subscriber sub_set_ref_x_u_;
  ros::Subscriber sub_set_traj_;
};
};  // namespace aerial_robot_navigation