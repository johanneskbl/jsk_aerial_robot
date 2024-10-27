//
// Created by li-jinjie on 24-10-27.
//

#ifndef AERIAL_ROBOT_CONTROL_NMPC_MANAGER_H
#define AERIAL_ROBOT_CONTROL_NMPC_MANAGER_H

#include "flight_navigation.h"
#include "aerial_robot_msgs/PredXU.h"

namespace aerial_robot_navigation
{
class NMPCManager : public BaseNavigator
{
public:
  inline void init_nmpc_info(int NX, int NU, int NN, aerial_robot_msgs::PredXU& x_u_ref)
  {
    NX_ = NX;
    NU_ = NU;
    NN_ = NN;
    x_u_ref_ = x_u_ref;
  }

  inline void init_alloc_mat_pinv(const Eigen::MatrixXd& alloc_mat_pinv)
  {
    alloc_mat_pinv_ = alloc_mat_pinv;
  }

  inline aerial_robot_msgs::PredXU getRefXU() const
  {
    return x_u_ref_;
  }

  inline bool getTrajectoryMode() const
  {
    return trajectory_mode_;
  }

protected:
  /* var used in NMPC controller */
  int NX_;
  int NU_;
  int NN_;

  Eigen::MatrixXd alloc_mat_pinv_;

  aerial_robot_msgs::PredXU x_u_ref_;

  /* trajectory manager */
  bool trajectory_mode_ = false;
  ros::Time receive_time_;
};
}  // namespace aerial_robot_navigation

#endif  // AERIAL_ROBOT_CONTROL_NMPC_MANAGER_H
