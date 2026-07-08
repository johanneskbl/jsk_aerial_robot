//
// Created by li-jinjie on 23-11-25.
//

#ifndef BASE_MPC_CONTROLLER_H
#define BASE_MPC_CONTROLLER_H

#include "aerial_robot_control/control/base/base.h"
#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_msgs/PredXU.h"

namespace aerial_robot_control
{

namespace nmpc
{

/**
 * @brief The BaseMPC class to define the common interface of NMPC controllers, especially for the ROS communication.
 */
class BaseMPC : public ControlBase
{
public:
protected:
  // define MPC solver
  boost::shared_ptr<pluginlib::ClassLoader<aerial_robot_control::mpc_solver::BaseMPCSolver>> mpc_solver_loader_ptr_;
  boost::shared_ptr<aerial_robot_control::mpc_solver::BaseMPCSolver> mpc_solver_ptr_;
  inline static int NN_, NX_, NU_, NP_;

  /* initialize() */
  inline void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                         boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                         boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                         boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                         double ctrl_loop_du) override
  {
    ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

    /* init mpc solver plugin */
    mpc_solver_loader_ptr_ =
        boost::make_shared<pluginlib::ClassLoader<aerial_robot_control::mpc_solver::BaseMPCSolver>>(
            "aerial_robot_control", "aerial_robot_control::mpc_solver::BaseMPCSolver");
    
    try
    {
      // 1. read the plugin name from the parameter server
      std::string mpc_solver_name;
      if (!nh_.getParam("mpc_solver_name", mpc_solver_name))
      {
        ROS_ERROR(
            "mpc_solver_name for mpc_solver_plugin is not loaded. "
            "You must specify the mpc_solver_name in the launch file.");
        return;
      }

      // 2. load the plugin
      mpc_solver_ptr_ = mpc_solver_loader_ptr_->createInstance(mpc_solver_name);
      mpc_solver_ptr_->initialize();
      ROS_INFO("load mpc solver plugin: %s", mpc_solver_name.c_str());
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("mpc_solver_plugin: The plugin failed to load for some reason. Error: %s", ex.what());
    }

    /* Store solver parameters */
    NN_ = mpc_solver_ptr_->NN_;
    NX_ = mpc_solver_ptr_->NX_;
    NU_ = mpc_solver_ptr_->NU_;
    NP_ = mpc_solver_ptr_->NP_;
  }

  // define the ROS msg related to MPC
  inline static void initPredXU(aerial_robot_msgs::PredXU& x_u)
  {
    x_u.x.layout.dim.emplace_back();
    x_u.x.layout.dim.emplace_back();
    x_u.x.layout.dim[0].label = "horizon";
    x_u.x.layout.dim[0].size = NN_ + 1;
    x_u.x.layout.dim[0].stride = (NN_ + 1) * NX_;
    x_u.x.layout.dim[1].label = "state";
    x_u.x.layout.dim[1].size = NX_;
    x_u.x.layout.dim[1].stride = NX_;
    x_u.x.layout.data_offset = 0;
    x_u.x.data.resize((NN_ + 1) * NX_);
    std::fill(x_u.x.data.begin(), x_u.x.data.end(), 0.0);
    // quaternion
    for (int i = 6; i < (NN_ + 1) * NX_; i += NX_)
      x_u.x.data[i] = 1.0;

    x_u.u.layout.dim.emplace_back();
    x_u.u.layout.dim.emplace_back();
    x_u.u.layout.dim[0].label = "horizon";
    x_u.u.layout.dim[0].size = NN_;
    x_u.u.layout.dim[0].stride = NN_ * NU_;
    x_u.u.layout.dim[1].label = "input";
    x_u.u.layout.dim[1].size = NU_;
    x_u.u.layout.dim[1].stride = NU_;
    x_u.u.layout.data_offset = 0;
    x_u.u.data.resize(NN_ * NU_);
    std::fill(x_u.u.data.begin(), x_u.u.data.end(), 0.0);
  }

  /* update() */
  // 1. set reference for NMPC
  virtual void callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg) = 0;
  static void rosXU2VecXU(const aerial_robot_msgs::PredXU& x_u, std::vector<std::vector<double>>& x_vec,
                          std::vector<std::vector<double>>& u_vec)
  {
    if (x_u.x.layout.dim[1].stride != NX_ || x_u.u.layout.dim[1].stride != NU_)
      ROS_ERROR("The dimension of x_u: nx, nu is not correct!");

    if (x_u.x.layout.dim[0].size != NN_ + 1 || x_u.u.layout.dim[0].size != NN_)
      ROS_ERROR("The dimension of x_u: nn for x, nn for u is not correct!");

    // convert x_u to xr_ and ur_
    for (int i = 0; i < NN_; i++)
    {
      std::copy(x_u.x.data.begin() + i * NX_, x_u.x.data.begin() + (i + 1) * NX_, x_vec[i].begin());
      std::copy(x_u.u.data.begin() + i * NU_, x_u.u.data.begin() + (i + 1) * NU_, u_vec[i].begin());
    }
    std::copy(x_u.x.data.begin() + NN_ * NX_, x_u.x.data.begin() + (NN_ + 1) * NX_, x_vec[NN_].begin());
  }
  // 2. get the current state from the measurement
  virtual std::vector<double> meas2VecX() = 0;
  // 3. solve NMPC
  virtual void controlCore() = 0;
  // 4. send the command to the robot
  virtual void sendCmd() = 0;
  // 5. viz the result in RVIZ
  virtual void callbackViz(const ros::TimerEvent& event) = 0;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // BASE_MPC_CONTROLLER_H
