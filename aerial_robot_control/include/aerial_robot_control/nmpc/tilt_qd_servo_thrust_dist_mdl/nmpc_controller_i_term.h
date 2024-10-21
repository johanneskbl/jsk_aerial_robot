//
// Created by li-jinjie on 24-9-28.
//

#ifndef TILT_QD_SERVO_THRUST_NMPC_W_ITERM_CONTROLLER_H
#define TILT_QD_SERVO_THRUST_NMPC_W_ITERM_CONTROLLER_H

#include "nmpc_controller.h"
#include "aerial_robot_control/nmpc/wrench_est/wrench_est_i_term.h"

#include "geometry_msgs/WrenchStamped.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoThrustNMPCwITerm : public TiltQdServoThrustDistNMPC
{
protected:
  WrenchEstITerm wrench_est_i_term_;

  void initParams() override;

  void calcDisturbWrench() override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_THRUST_NMPC_W_ITERM_CONTROLLER_H
