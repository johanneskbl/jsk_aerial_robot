#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Publish the target RPM of every spinal PWM test command (topic pwm_test, type spinal/PwmTest),
so that a recorded bag contains the commanded RPM next to the commanded PWM, e.g. for

  rostopic pub /pwm_test spinal/PwmTest "motor_index: [0]
  pwms: [0.6]"

The ESC runs closed-loop RPM control and decodes the PWM linearly:
    pwm in [0.5, 1.0]  <->  rpm in [min_rpm, max_rpm]
with min_rpm / max_rpm read from MotorInfoDShot.yaml.

The node follows spinal's PWM test state like AttitudeController::pwmTestCallback does and publishes
the target RPM of the DShot motors on pwm_test/target_rpm (std_msgs/Float32MultiArray, data[i] is
motor i, 0 means motor off). spinal holds a command until the next one, but `rostopic pub` sends it
only once, so the held command is republished at ~rate while spinal is in PWM test mode. Leaving
PWM test mode (empty message) publishes all zeros once.

Usage:
  roslaunch spinal bridge.launch
  rosrun beetle_omni pwm_test_target_rpm.py
  rosbag record /pwm_test /pwm_test/target_rpm /esc_telem
Optional args:
  _motor_info_file:=<path>  # default: beetle_omni/config/MotorInfoDShot.yaml
  _rate:=50.0               # [Hz] republish rate of the held command, 0 publishes on new commands only
"""

import os
import threading
from typing import List, Tuple

import rospkg
import rospy
import yaml
from std_msgs.msg import Float32MultiArray

try:
    from spinal.msg import PwmTest  # motor_index: uint8[], pwms: float32[]
except ImportError as e:
    raise ImportError("spinal/PwmTest not found, source a workspace whose spinal package provides PwmTest.msg") from e

IDLE_PWM = 0.5  # spinal IDLE_DUTY, decoded as min_rpm by the ESC
MAX_PWM = 1.0  # spinal PWM_TEST_MAX_DUTY, decoded as max_rpm by the ESC
DSHOT_MOTOR_NUMBER = 4  # spinal drives motors 0-3 by DShot, the other indices are plain PWM outputs


def load_rpm_window(motor_info_file: str) -> Tuple[float, float]:
    with open(motor_info_file) as f:
        motor_info = yaml.safe_load(f)["motor_info"]
    min_rpm, max_rpm = float(motor_info["min_rpm"]), float(motor_info["max_rpm"])
    if not min_rpm < max_rpm:
        raise ValueError(f"{motor_info_file}: min_rpm ({min_rpm}) must be smaller than max_rpm ({max_rpm})")
    return min_rpm, max_rpm


class PwmTestTargetRpm:
    """Mirrors spinal's PWM test state and publishes it as target RPMs."""

    def __init__(self, min_rpm: float, max_rpm: float, rate: float):
        self.min_rpm = min_rpm
        self.max_rpm = max_rpm
        self.lock = threading.Lock()  # the subscriber and the timer run in different threads
        self.test_mode = False
        self.target_rpms: List[float] = [0.0] * DSHOT_MOTOR_NUMBER

        self.rpm_pub = rospy.Publisher("pwm_test/target_rpm", Float32MultiArray, queue_size=10)
        self.pwm_sub = rospy.Subscriber("pwm_test", PwmTest, self.pwm_test_callback, queue_size=10)
        if rate > 0:
            self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.timer_callback)

    def pwm_to_rpm(self, pwm: float) -> float:
        # spinal switches the motor off for an out of range PWM; the negated form also catches NaN
        if not IDLE_PWM <= pwm <= MAX_PWM:
            return 0.0
        return self.min_rpm + (self.max_rpm - self.min_rpm) * (pwm - IDLE_PWM) / (MAX_PWM - IDLE_PWM)

    def pwm_test_callback(self, msg: PwmTest):
        with self.lock:
            # an empty message makes spinal leave PWM test mode, the motors stop
            if not msg.pwms:
                if self.test_mode:
                    self.test_mode = False
                    self.target_rpms = [0.0] * DSHOT_MOTOR_NUMBER
                    self.publish()
                return

            # spinal enters PWM test mode with all motors off
            if not self.test_mode:
                self.test_mode = True
                self.target_rpms = [0.0] * DSHOT_MOTOR_NUMBER

            if not msg.motor_index:
                # simultaneous test mode: every motor gets the first PWM
                commands = [(i, msg.pwms[0]) for i in range(DSHOT_MOTOR_NUMBER)]
            elif len(msg.motor_index) == len(msg.pwms):
                commands = zip(msg.motor_index, msg.pwms)  # uint8[] arrives as bytes, which iterates as ints
            else:
                rospy.logerr("The number of indices does not match the number of PWMs, spinal ignores the command")
                commands = []

            for i, pwm in commands:
                if i < DSHOT_MOTOR_NUMBER:
                    self.target_rpms[i] = self.pwm_to_rpm(pwm)
            self.publish()

    def timer_callback(self, event):
        with self.lock:
            if self.test_mode:
                self.publish()

    def publish(self):
        self.rpm_pub.publish(Float32MultiArray(data=list(self.target_rpms)))


def main():
    rospy.init_node("pwm_test_target_rpm")
    default_file = os.path.join(rospkg.RosPack().get_path("beetle_omni"), "config", "MotorInfoDShot.yaml")
    motor_info_file = rospy.get_param("~motor_info_file", default_file)
    rate = float(rospy.get_param("~rate", 50.0))
    min_rpm, max_rpm = load_rpm_window(motor_info_file)

    PwmTestTargetRpm(min_rpm, max_rpm, rate)
    rospy.loginfo(
        "PWM [%.1f, %.1f] -> target RPM [%.0f, %.0f] (from %s), publishing on %s",
        IDLE_PWM,
        MAX_PWM,
        min_rpm,
        max_rpm,
        motor_info_file,
        rospy.resolve_name("pwm_test/target_rpm"),
    )
    rospy.spin()


if __name__ == "__main__":
    main()
