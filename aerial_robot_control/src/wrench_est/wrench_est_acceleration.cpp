//
// Created by li-jinjie on 24-10-25.
// References:
// https://github.com/sugihara-16/aerial_robot/blob/35b2a2c86792c8ba7b4f346339964d4cf8d6a280/robots/beetle/src/control/beetle_controller.cpp#L352
// T. Tomic, et al., T-RO, "External Wrench Estimation, Collision Detection,  and Reflex Reaction for Flying Robots"
//

#include "aerial_robot_control/wrench_est/wrench_est_acceleration.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::WrenchEstAcceleration, aerial_robot_control::WrenchEstBase)