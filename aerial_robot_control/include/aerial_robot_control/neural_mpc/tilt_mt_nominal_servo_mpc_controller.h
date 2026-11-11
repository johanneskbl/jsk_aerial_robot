#ifndef TILT_MT_NOMINAL_SERVO_MPC_CONTROLLER_H
#define TILT_MT_NOMINAL_SERVO_MPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_nmpc_controller.h"

namespace aerial_robot_control
{

namespace nmpc
{

/**
 * @brief MPC controller using the nominal model of the neural MPC framework
 * The state, input and cost are the same as TiltMtServoNMPC.
 * It serves for dataset recording and as a baseline for the neural MPC.
 */
class TiltMtNominalServoMPC : public TiltMtServoNMPC
{
public:
  TiltMtNominalServoMPC() = default;
  ~TiltMtNominalServoMPC() override = default;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_NOMINAL_SERVO_MPC_CONTROLLER_H
