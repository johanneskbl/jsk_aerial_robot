/*
 * servo_filtering.h
 *
 * Created on: 2026-11-22
 * Low-pass filtered servo angle and Savitzky-Golay derivative for the servo velocity:
 * An alpha-beta filter (similar to the one used for the ESC RPM) filters for a complete
 * read of the Dynamixel servos. Additionally, a Savitzky-Golay derivative of the last
 * 60 ms of angles estimates the velocity.
 */

#ifndef SERVO_FILTERING_H
#define SERVO_FILTERING_H

#include "dynamixel_serial.h"

namespace ServoFiltering {
  // After each complete read: filters present_position_ and calculates the
  // filtered velocity of the sent servos in place
  void update(std::array<ServoData, MAX_SERVO_NUM>& servo, unsigned int servo_num);
}

#endif /* SERVO_FILTERING_H */
