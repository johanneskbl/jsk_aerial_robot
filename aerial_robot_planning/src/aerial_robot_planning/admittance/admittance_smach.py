"""
 Created by jinjie on 2026/04/03.
"""

import rospy
import smach
from nav_msgs.msg import Odometry

from ..pub_mpc_joint_traj import MPCSinglePtPub
from ..util import topic_ready

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped


class AdmittanceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done_admittance"], input_keys=["robot_name"], output_keys=["robot_name"])
        self.rate = rospy.Rate(50)

        self.ee_odom_sub = None
        self.ee_odom_msg = Odometry()

        self.ext_wrench_body_frame_sub = None
        self.ext_wrench_body_frame_msg = WrenchStamped()

    def execute(self, userdata):
        rospy.loginfo("Executing state AdmittanceState")
        robot_name = userdata.robot_name

        # initialization
        try:
            ee_odom_topic = f"/{userdata.robot_name}/uav/ee_contact/odom"

            if not topic_ready(ee_odom_topic, Odometry, timeout=5):
                rospy.logerr(f"Topic {ee_odom_topic} not available. Exiting AdmittanceState.")
                return "done_admittance"

            self.ee_odom_sub = rospy.Subscriber(ee_odom_topic, Odometry, self._sub_ee_odom_callback)

            ext_wrench_body_frame_topic = f"/{userdata.robot_name}/ext_wrench_est/value"
            if not topic_ready(ext_wrench_body_frame_topic, WrenchStamped, timeout=5):
                rospy.logerr(f"Topic {ext_wrench_body_frame_topic} not available. Exiting AdmittanceState.")
                return "done_admittance"

            self.ext_wrench_body_frame_sub = rospy.Subscriber(
                ext_wrench_body_frame_topic, WrenchStamped, self._sub_ext_wrench_body_frame_callback
            )

        except Exception as e:
            rospy.logerr(f"Initialization error in AdmittanceState: {e}")
            return "done_admittance"

        while not rospy.is_shutdown():
            # Some contents here for the task execution.
            print("111")
            print(f"odom: {self.ee_odom_msg}")
            print(f"wrench: {self.ext_wrench_body_frame_msg}\n")

            self.rate.sleep()

        return "done_admittance"

    def _sub_ee_odom_callback(self, msg):
        self.ee_odom_msg = msg

    def _sub_ext_wrench_body_frame_callback(self, msg):
        self.ext_wrench_body_frame_msg = msg


def create_admittance_state_machine():
    sm_sub = smach.StateMachine(outcomes=["DONE_ADMITTANCE"], input_keys=["robot_name"])

    with sm_sub:
        smach.StateMachine.add("ADMITTANCE", AdmittanceState(), transitions={"done_admittance": "DONE_ADMITTANCE"})

    return sm_sub
