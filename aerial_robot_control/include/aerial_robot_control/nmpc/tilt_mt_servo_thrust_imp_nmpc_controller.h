//
// Created by li-jinjie on 24-11-21.
//

#ifndef TILT_MT_SERVO_THRUST_IMP_NMPC_CONTROLLER_H
#define TILT_MT_SERVO_THRUST_IMP_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_thrust_dist_nmpc_controller.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_thrust_dist_imp_mdl/nmpc_solver.h"

namespace aerial_robot_control
{

namespace nmpc
{

/**
 * @brief MPC controller with a servo angle model and thrust model based on impedance control
 * The position and orientation weights of the cost are set by the virtual mass, damping and stiffness
 * of the impedance model, and the virtual mass and inertia are passed as model parameters.
 */
class TiltMtServoThrustImpNMPC : public TiltMtServoThrustDistNMPC
{
protected:
  int idx_p_imp_end_ = 0;

  /* Initialization methods */
  void initMPCCostW() override;

  /* Activate methods */
  void initMPCPhysicalParams() override;
  void setImpParams();

  /* Callback methods */
  void callbackMPCcfg(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_THRUST_IMP_NMPC_CONTROLLER_H
