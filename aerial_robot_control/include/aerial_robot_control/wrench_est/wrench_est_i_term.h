//
// Created by li-jinjie on 24-10-24.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_I_TERM_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_I_TERM_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"
#include "aerial_robot_control/wrench_est/utils.h"

/* dynamic reconfigure */
#include <dynamic_reconfigure/server.h>
#include "aerial_robot_msgs/DynamicReconfigureLevels.h"
#include "aerial_robot_control/ITermConfig.h"

using ITermDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::ITermConfig>;

namespace aerial_robot_control
{

class WrenchEstITerm : public WrenchEstBase
{
public:
  WrenchEstITerm() = default;

  void initWrenchPub() override
  {
    pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("dist_w_f_cog_tq/iterm", 1);
  }

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator, double ctrl_loop_du) override
  {
    WrenchEstBase::initialize(nh, robot_model, estimator, ctrl_loop_du);

    ros::NodeHandle i_term_nh(nh, "controller/i_term");

    double fx_limit, fy_limit, fz_limit, mx_limit, my_limit, mz_limit;
    getParam<double>(i_term_nh, "limit_fx", fx_limit, 5.0);
    getParam<double>(i_term_nh, "limit_fy", fy_limit, 5.0);
    getParam<double>(i_term_nh, "limit_fz", fz_limit, 5.0);
    getParam<double>(i_term_nh, "limit_mx", mx_limit, 1.0);
    getParam<double>(i_term_nh, "limit_my", my_limit, 1.0);
    getParam<double>(i_term_nh, "limit_mz", mz_limit, 1.0);

    double i_gain_x, i_gain_y, i_gain_z, i_gain_roll, i_gain_pitch, i_gain_yaw;
    getParam<double>(i_term_nh, "i_gain_x", i_gain_x, 1.0);
    getParam<double>(i_term_nh, "i_gain_y", i_gain_y, 1.0);
    getParam<double>(i_term_nh, "i_gain_z", i_gain_z, 1.0);
    getParam<double>(i_term_nh, "i_gain_roll", i_gain_roll, 0.5);
    getParam<double>(i_term_nh, "i_gain_pitch", i_gain_pitch, 0.5);
    getParam<double>(i_term_nh, "i_gain_yaw", i_gain_yaw, 0.5);

    // NOTE: ctrl_loop_du is the *rate* of the control loop, so freq is actually its *period*
    double freq = 1.0 / getCtrlLoopDu();
    i_term_[0].initialize(i_gain_x, fx_limit, freq);  // x
    i_term_[1].initialize(i_gain_y, fy_limit, freq);  // y
    i_term_[2].initialize(i_gain_z, fz_limit, freq);  // z

    i_term_[3].initialize(i_gain_roll, mx_limit, freq);   // roll
    i_term_[4].initialize(i_gain_pitch, my_limit, freq);  // pitch
    i_term_[5].initialize(i_gain_yaw, mz_limit, freq);    // yaw

    /* init dynamic reconfigure */
    reconf_servers_.push_back(boost::make_shared<ITermDynamicConfig>(i_term_nh));
    reconf_servers_.back()->setCallback(boost::bind(&WrenchEstITerm::cfgCallback, this, _1, _2));
  }

  /**
   * @brief Update the I-term with the residual between the measured and the predicted velocities
   * The velocity residual over one control period T is (a_real - a_model) * T, i.e., it is proportional to the model
   * error of the linear acceleration (force / mass) and the angular acceleration (torque / inertia). Therefore, the
   * I-term converges to the model error of the wrench with the time constant
   *   tau = mass / (i_gain * dt),   or   tau = inertia / (i_gain * dt)   for the torque,
   * where dt is the period of the control loop. Since the prediction follows the planned acceleration of the model, the
   * residual is not biased by maneuvers.
   * NOTE: In contrast to a position residual, the velocity can be interpolated linearly on the first shooting interval
   * without error for a constant acceleration, also backwards in time.
   * @param vel_pred_w Predicted linear velocity of the CoG at the time of the measurement, expressed in World frame [m/s]
   * @param omega_pred_b Predicted angular velocity at the time of the measurement, expressed in Body frame [rad/s]
   * @param vel_w Measured linear velocity of the CoG, expressed in World frame [m/s]
   * @param omega_b Measured angular velocity, expressed in Body frame [rad/s]
   */
  void update(const tf::Vector3& vel_pred_w, const tf::Vector3& omega_pred_b,
              const tf::Vector3& vel_w, const tf::Vector3& omega_b)
  {
    double fx_w_i_term = i_term_[0].update(vel_w.x() - vel_pred_w.x());
    double fy_w_i_term = i_term_[1].update(vel_w.y() - vel_pred_w.y());
    double fz_w_i_term = i_term_[2].update(vel_w.z() - vel_pred_w.z());
    double mx_cog_i_term = i_term_[3].update(omega_b.x() - omega_pred_b.x());
    double my_cog_i_term = i_term_[4].update(omega_b.y() - omega_pred_b.y());
    double mz_cog_i_term = i_term_[5].update(omega_b.z() - omega_pred_b.z());

    setRawDistForceW(fx_w_i_term, fy_w_i_term, fz_w_i_term);
    setRawDistTorqueCOG(mx_cog_i_term, my_cog_i_term, mz_cog_i_term);
  }

  void reset() override
  {
    WrenchEstBase::reset();

    for (auto& i_term : i_term_)
      i_term.reset();
  }

private:
  ITerm i_term_[6];  // x, y, z, roll, pitch, yaw

  std::vector<boost::shared_ptr<ITermDynamicConfig>> reconf_servers_;

  void cfgCallback(ITermConfig& config, uint32_t level)
  {
    using Levels = aerial_robot_msgs::DynamicReconfigureLevels;

    if (config.i_gain_flag)
    {
      try
      {
        switch (level)
        {
          case Levels::RECONFIGURE_I_GAIN_X: {
            i_term_[0].setIGain(config.i_gain_x);
            ROS_INFO_STREAM("change i_gain_x for '" << config.i_gain_x << "'");
            break;
          }
          case Levels::RECONFIGURE_I_GAIN_Y: {
            i_term_[1].setIGain(config.i_gain_y);
            ROS_INFO_STREAM("change i_gain_y for '" << config.i_gain_y << "'");
            break;
          }
          case Levels::RECONFIGURE_I_GAIN_Z: {
            i_term_[2].setIGain(config.i_gain_z);
            ROS_INFO_STREAM("change i_gain_z for '" << config.i_gain_z << "'");
            break;
          }
          case Levels::RECONFIGURE_I_GAIN_ROLL: {
            i_term_[3].setIGain(config.i_gain_roll);
            ROS_INFO_STREAM("change i_gain_roll for '" << config.i_gain_roll << "'");
            break;
          }
          case Levels::RECONFIGURE_I_GAIN_PITCH: {
            i_term_[4].setIGain(config.i_gain_pitch);
            ROS_INFO_STREAM("change i_gain_pitch for '" << config.i_gain_pitch << "'");
            break;
          }
          case Levels::RECONFIGURE_I_GAIN_YAW: {
            i_term_[5].setIGain(config.i_gain_yaw);
            ROS_INFO_STREAM("change i_gain_yaw for '" << config.i_gain_yaw << "'");
            break;
          }
          default:
            break;
        }
      }
      catch (std::invalid_argument& e)
      {
        ROS_ERROR_STREAM("I Term config failed: " << e.what());
      }
    }
  }
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_I_TERM_H
