#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point, Wrench
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from std_srvs.srv import Empty


class ConstantForceApplier:
    def __init__(self):
        rospy.init_node("constant_force_applier")

        # ===== Parameters =====
        # Note: body_name must be a link name, not a model name
        # The format is usually "model_name::link_name"
        self.body_name = rospy.get_param("~body_name", "beetle1::root")
        self.reference_frame = rospy.get_param("~reference_frame", "world")
        self.rate_hz = rospy.get_param("~rate", 50.0)

        # Application point of the wrench, expressed in the reference_frame
        self.point_x = rospy.get_param("~point_x", 0.0)
        self.point_y = rospy.get_param("~point_y", 0.0)
        self.point_z = rospy.get_param("~point_z", 0.0)

        # Target constant force (N)
        self.force_x = rospy.get_param("~force_x", 0.0)
        self.force_y = rospy.get_param("~force_y", 0.0)
        self.force_z = rospy.get_param("~force_z", 0.0)

        # Target constant torque (N*m)
        self.torque_x = rospy.get_param("~torque_x", 0.2)
        self.torque_y = rospy.get_param("~torque_y", 0.0)
        self.torque_z = rospy.get_param("~torque_z", 0.0)

        # Duration of each wrench application
        # It is recommended to set this slightly larger than 1 / rate_hz
        # to avoid gaps between consecutive applications
        self.duration = rospy.get_param("~duration", 0.05)

        # Ramp duration in seconds
        # The applied wrench increases gradually from zero to the target value
        self.ramp_duration = rospy.get_param("~ramp_duration", 2.0)

        rospy.loginfo("Waiting for /gazebo/apply_body_wrench service...")
        rospy.wait_for_service("/gazebo/apply_body_wrench")
        self.apply_wrench_srv = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)

        # Optional: clear any remaining wrench when shutting down
        self.clear_available = False
        try:
            rospy.wait_for_service("/gazebo/clear_body_wrenches", timeout=1.0)
            self.clear_wrenches_srv = rospy.ServiceProxy("/gazebo/clear_body_wrenches", Empty)
            self.clear_available = True
        except rospy.ROSException:
            rospy.logwarn("/gazebo/clear_body_wrenches not available.")

        self.start_time = rospy.Time.now()
        rospy.on_shutdown(self.on_shutdown)

    def get_ramp_scale(self):
        """
        Compute a scale factor in [0, 1] for gradual ramp-up.
        The scale increases linearly with time until it reaches 1.
        """
        if self.ramp_duration <= 0.0:
            return 1.0

        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        scale = max(0.0, min(1.0, elapsed / self.ramp_duration))
        return scale

    def build_request(self):
        scale = self.get_ramp_scale()

        req = ApplyBodyWrenchRequest()
        req.body_name = self.body_name
        req.reference_frame = self.reference_frame

        req.reference_point = Point(
            x=self.point_x,
            y=self.point_y,
            z=self.point_z,
        )

        req.wrench = Wrench()
        req.wrench.force.x = scale * self.force_x
        req.wrench.force.y = scale * self.force_y
        req.wrench.force.z = scale * self.force_z
        req.wrench.torque.x = scale * self.torque_x
        req.wrench.torque.y = scale * self.torque_y
        req.wrench.torque.z = scale * self.torque_z

        req.start_time = rospy.Time(0)  # Apply immediately
        req.duration = rospy.Duration(self.duration)
        return req

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo(
            "Applying ramped wrench to [%s] in frame [%s]",
            self.body_name,
            self.reference_frame,
        )

        while not rospy.is_shutdown():
            req = self.build_request()
            scale = self.get_ramp_scale()
            try:
                self.apply_wrench_srv(req)
                rospy.loginfo_throttle(
                    0.5,
                    "Ramp scale: %.3f | Force: [%.3f, %.3f, %.3f] | Torque: [%.3f, %.3f, %.3f]",
                    scale,
                    scale * self.force_x,
                    scale * self.force_y,
                    scale * self.force_z,
                    scale * self.torque_x,
                    scale * self.torque_y,
                    scale * self.torque_z,
                )
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call /gazebo/apply_body_wrench: %s", str(e))
            rate.sleep()

    def on_shutdown(self):
        rospy.loginfo("Shutting down constant_force_applier...")
        if self.clear_available:
            try:
                self.clear_wrenches_srv()
                rospy.loginfo("Cleared body wrenches.")
            except rospy.ServiceException as e:
                rospy.logwarn("Failed to clear body wrenches: %s", str(e))


if __name__ == "__main__":
    node = ConstantForceApplier()
    node.run()
