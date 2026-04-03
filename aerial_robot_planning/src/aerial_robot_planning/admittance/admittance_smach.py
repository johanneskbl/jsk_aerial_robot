"""
 Created by jinjie on 2026/04/03.
"""
import numpy as np
import rospy
import smach
from nav_msgs.msg import Odometry

from ..util import topic_ready

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped

from scipy.spatial.transform import Rotation as R


def skew(v):
    v = np.asarray(v).reshape(
        3,
    )
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


class AdmittanceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done_admittance"], input_keys=["robot_name"], output_keys=["robot_name"])
        self.rate = rospy.Rate(50)

        self.ee_odom_sub = None
        self.ee_odom_msg = Odometry()

        self.ext_wrench_body_frame_sub = None
        self.ext_wrench_body_frame_msg = WrenchStamped()

        # physical params
        # change flag is_print_phys_params to True to print these params in the console
        # TODO: make this part automatically update with different end-effectors
        self.p_ee_in_body = np.array([[0.0, 0.0029, 0.2644]]).T
        self.q_ee_in_body = np.array([[0, 0, 0, 1]]).T  # q xyzw, same with the order of tf

        self.rot_body_ee = R.from_quat(self.q_ee_in_body.flatten()).as_matrix()  # ^R_T Rot

        # ====== publisher ======
        self.tool_wrench_pub = None

    def execute(self, userdata):
        rospy.loginfo("Executing state AdmittanceState")
        robot_name = userdata.robot_name

        self.tool_wrench_pub = rospy.Publisher(
            f"/{robot_name}/admittance_tool_frame/f_w_tq_t", WrenchStamped, queue_size=10
        )

        # initialization
        try:
            ee_odom_topic = f"/{robot_name}/uav/ee_contact/odom"

            if not topic_ready(ee_odom_topic, Odometry, timeout=5):
                rospy.logerr(f"Topic {ee_odom_topic} not available. Exiting AdmittanceState.")
                return "done_admittance"

            self.ee_odom_sub = rospy.Subscriber(ee_odom_topic, Odometry, self._sub_ee_odom_callback)

            ext_wrench_body_frame_topic = f"/{robot_name}/ext_wrench_est/value"
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

            # print(f"odom: {self.ee_odom_msg}")
            # print(f"wrench: {self.ext_wrench_body_frame_msg}\n")

            tool_force_w, tool_torque_t = self.compute_tool_wrench()
            self.publish_tool_wrench(tool_force_w, tool_torque_t)

            self.rate.sleep()

        return "done_admittance"

    def _sub_ee_odom_callback(self, msg):
        self.ee_odom_msg = msg

    def _sub_ext_wrench_body_frame_callback(self, msg):
        self.ext_wrench_body_frame_msg = msg

    def compute_tool_wrench(self):
        ext_force_b = np.array(
            [
                self.ext_wrench_body_frame_msg.wrench.force.x,
                self.ext_wrench_body_frame_msg.wrench.force.y,
                self.ext_wrench_body_frame_msg.wrench.force.z,
            ]
        )

        ext_torque_b = np.array(
            [
                self.ext_wrench_body_frame_msg.wrench.torque.x,
                self.ext_wrench_body_frame_msg.wrench.torque.y,
                self.ext_wrench_body_frame_msg.wrench.torque.z,
            ]
        )

        rot_w_b = R.from_quat(
            [
                self.ee_odom_msg.pose.pose.orientation.x,
                self.ee_odom_msg.pose.pose.orientation.y,
                self.ee_odom_msg.pose.pose.orientation.z,
                self.ee_odom_msg.pose.pose.orientation.w,
            ]
        ).as_matrix()

        tool_force_w = rot_w_b @ ext_force_b

        tool_torque_t = self.rot_body_ee.T @ (-skew(self.p_ee_in_body) @ ext_force_b + ext_torque_b)

        return tool_force_w, tool_torque_t

    def publish_tool_wrench(self, tool_force_w, tool_torque_t):
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()

        wrench_msg.wrench.force.x = tool_force_w[0]
        wrench_msg.wrench.force.y = tool_force_w[1]
        wrench_msg.wrench.force.z = tool_force_w[2]

        wrench_msg.wrench.torque.x = tool_torque_t[0]
        wrench_msg.wrench.torque.y = tool_torque_t[1]
        wrench_msg.wrench.torque.z = tool_torque_t[2]

        self.tool_wrench_pub.publish(wrench_msg)


def create_admittance_state_machine():
    sm_sub = smach.StateMachine(outcomes=["DONE_ADMITTANCE"], input_keys=["robot_name"])

    with sm_sub:
        smach.StateMachine.add("ADMITTANCE", AdmittanceState(), transitions={"done_admittance": "DONE_ADMITTANCE"})

    return sm_sub
