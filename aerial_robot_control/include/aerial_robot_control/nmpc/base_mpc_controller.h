//
// Created by li-jinjie on 23-11-25.
//

#ifndef BASE_MPC_CONTROLLER_H
#define BASE_MPC_CONTROLLER_H

#include "aerial_robot_control/control/base/base.h"
#include "aerial_robot_control/nmpc/base_mpc_solver.h"

/* Geometry */
#include <tf_conversions/tf_eigen.h>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <numeric>

/* ROS messages */
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseArray.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "aerial_robot_msgs/PredXU.h"
#include "aerial_robot_msgs/MPCState.h"
#include "spinal/SetControlMode.h"
#include "spinal/FlightConfigCmd.h"

namespace aerial_robot_control
{

namespace nmpc
{

/**
 * @brief Base class of all MPC controllers
 * It handles the MPC solver plugin, communication with spinal, solver failures, tracking frame and
 * common utilities for references, recording, and visualization.
 * Frames: World frame W (inertial), Body frame B (origin at the CoG), End-effector frame EE, and
 * Tracking frame T (T = B for cog_tracking, T = EE for ee_tracking).
 * NOTE: The generic methods assume that the first 13 states are [p, v, q, w]. Quaternions in format w,x,y,z.
 * NOTE: The MPC model and thus the solver state is always described in Body frame and the
 * MPC cost and thus the references are described in Tracking frame.
 */
class BaseMPC : public ControlBase
{
public:
  BaseMPC() = default;  // Note that the constructor should not have arguments by rule of rospluginlib
  ~BaseMPC() override = default;
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;
  void activate() override;
  bool update() override;
  void reset() override;

protected:
  ros::NodeHandle mpc_nh_;
  ros::Timer tmr_viz_;

  ros::Publisher pub_flight_config_cmd_spinal_;  // for flight config commands to spinal (e.g., force landing)
  ros::Publisher pub_solver_time_;               // for logging the solve time
  ros::Publisher pub_viz_ref_;                   // for visualization
  ros::Publisher pub_viz_pred_;                  // for visualization

  ros::ServiceClient srv_set_control_mode_;

  // Plugin loader for MPC solver
  boost::shared_ptr<pluginlib::ClassLoader<aerial_robot_control::mpc_solver::BaseMPCSolver>> mpc_solver_loader_ptr_;
  boost::shared_ptr<aerial_robot_control::mpc_solver::BaseMPCSolver> mpc_solver_ptr_;
  inline static int NN_, NX_, NU_, NP_;

  bool is_warmup_ = true;
  bool is_attitude_ctrl_;
  bool is_body_rate_ctrl_;
  bool is_print_phys_params_;
  bool is_debug_;
  bool has_triggered_force_landing_ = false;

  double mass_;
  std::vector<double> inertia_;
  double gravity_const_;

  double t_mpc_samp_;
  double t_mpc_step_;
  double t_mpc_horizon_;
  double t_mpc_dt_ = 0.0;  // measured closed-loop timestep between two solves
  double t_mpc_timestep_;
  double t_mpc_prev_timestep_;

  // Recording
  double record_rate_ = 10.0;  // Publishing rate of the recording in Hz
  double t_stamp_record_;
  double t_last_record_ = 0.0;

  // Frame used by the MPC cost, allocation and reference
  // NOTE: Python code generation reads the same parameter from the config file but needs to be
  // regenerated after changing it.
  enum class TrackingFrame
  {
    CoG,  // Body frame (origin at the CoG)
    EE    // End-effector frame
  };
  TrackingFrame tracking_frame_ = TrackingFrame::CoG;
  inline bool isTrackingEE() const { return tracking_frame_ == TrackingFrame::EE; }
  // Name of the robot frame described by the references, set as header.frame_id by their publishers
  inline std::string getTrackingFrameName() const { return isTrackingEE() ? "ee_contact" : "cog"; }

  // Pose of the Tracking frame in Body frame, updated once per control loop (identity for T = B)
  // NOTE: Guarded by a mutex, since the reference callbacks and the visualization read it from other threads.
  tf::Vector3 t_p_b_ = tf::Vector3(0.0, 0.0, 0.0);  // Origin of Tracking frame in Body frame
  tf::Quaternion q_bt_ = tf::Quaternion(0.0, 0.0, 0.0, 1.0);  // x, y, z, w
  mutable std::mutex t_pose_mutex_;

  /* Initialization methods */
  virtual void initGeneralParams();
  void setControlMode();
  static void initPredXU(aerial_robot_msgs::PredXU& x_u);
  virtual void initMPCCostW() = 0;
  virtual void initMPCConstraints() = 0;

  /* Activate methods */
  void updateInertialParams();

  /* Reset methods */
  virtual void resetSolver() = 0;

  /* State methods */
  virtual std::vector<double> meas2VecX() = 0;

  /* Update methods */
  virtual void controlCore() = 0;
  bool solve(const std::vector<double>& bx0);
  void handleSolverFailure();
  virtual double getCommand(int idx_u, double T_horizon = 0.0);
  virtual void sendCmd() = 0;
  virtual void publishRecording() {}
  void publishSolverTime() const;

  /* Tracking frame methods */
  void updateTrackingFramePose();
  void getTrackingFramePose(tf::Vector3& t_p_b, tf::Quaternion& q_bt) const;
  void convertFromBToT(const tf::Vector3& pos_w, const tf::Vector3& vel_w, const tf::Quaternion& q_wb,
                       const tf::Vector3& omega_b, tf::Vector3& t_pos_w, tf::Vector3& t_vel_w, tf::Quaternion& q_wt,
                       tf::Vector3& omega_t) const;
  void convertFromTToB(const tf::Vector3& t_pos_w, const tf::Vector3& t_vel_w, const tf::Quaternion& q_wt,
                       const tf::Vector3& omega_t, tf::Vector3& pos_w, tf::Vector3& vel_w, tf::Quaternion& q_wb,
                       tf::Vector3& omega_b) const;

  /* Reference methods */
  static void rosXU2VecXU(const aerial_robot_msgs::PredXU& x_u, std::vector<std::vector<double>>& x_vec,
                          std::vector<std::vector<double>>& u_vec);
  virtual void callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg) = 0;
  bool isRefInTrackingFrame(const std::string& frame_id) const;
  Eigen::VectorXd computeRefWrenchB(const tf::Vector3& ref_acc_w, const tf::Quaternion& ref_quat_wt,
                                    const tf::Vector3& ref_omega_t, const tf::Vector3& ref_ang_acc_t) const;

  /* Callback methods */
  virtual void callbackViz(const ros::TimerEvent& event);

  /* Util methods */
  static void ensureQuaternionContinuity(tf::Quaternion& quat, tf::Quaternion& quat_prev);
  static bool isMulDOFJointTrajPtEqual(const trajectory_msgs::MultiDOFJointTrajectoryPoint& a,
                                       const trajectory_msgs::MultiDOFJointTrajectoryPoint& b,
                                       bool if_compare_time = true, double epsilon = 1e-6);
  static inline bool isAlmostEqual(const double a, const double b, const double epsilon = 1e-6)
  {
    return std::fabs(a - b) < epsilon;
  }

  /* Debug methods */
  void printPhysicalParams() const;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // BASE_MPC_CONTROLLER_H
