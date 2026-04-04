"""
 Created by jinjie on 2026/04/03.
"""
import numpy as np
import rospy
import smach
from nav_msgs.msg import Odometry

from ..util import topic_ready

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from scipy.spatial.transform import Rotation as R


def skew(v):
    v = np.asarray(v).reshape(
        3,
    )
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def quat_multiply(qx1, qy1, qz1, qw1, qx2, qy2, qz2, qw2):
    """
    Multiply two quaternions.
    """
    return np.array(
        [
            qw1 * qx2 + qx1 * qw2 + qy1 * qz2 - qz1 * qy2,
            qw1 * qy2 - qx1 * qz2 + qy1 * qw2 + qz1 * qx2,
            qw1 * qz2 + qx1 * qy2 - qy1 * qx2 + qz1 * qw2,
            qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2,
        ]
    )


class AdmittanceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done_admittance"], input_keys=["robot_name"], output_keys=["robot_name"])

        self.dt = 0.02
        self.rate = rospy.Rate(1 / self.dt)

        self.ee_odom_sub = None
        self.ee_odom_msg = Odometry()

        self.ext_wrench_body_frame_sub = None
        self.ext_wrench_body_frame_msg = WrenchStamped()

        # ROS parameters
        self.N_nmpc = None

        self.start_time = None

        # physical params
        # change flag is_print_phys_params to True to print these params in the console
        # TODO: make this part automatically update with different end-effectors
        self.p_ee_in_body = np.array([[0.0, 0.0029, 0.2644]]).T
        self.q_ee_in_body = np.array([[0, 0, 0, 1]]).T  # q xyzw, same with the order of tf

        self.rot_body_ee = R.from_quat(self.q_ee_in_body.flatten()).as_matrix()  # ^R_T Rot

        # ====== M D K ======
        self.Mp = np.diag([3, 3, 3])  # [kg]
        self.Mp_rev = np.linalg.inv(self.Mp)
        self.Dp = np.diag([10, 10, 10])  # [N*s/m]
        self.Kp = np.diag([5, 5, 5])  # [N/m]

        self.Mq = np.diag([0.1, 0.1, 0.1])  # [kg*m^2]
        self.Mq_rev = np.linalg.inv(self.Mq)
        self.Dq = np.diag([0.5, 0.5, 0.5])  # [N*m*s/rad]
        self.Kq = np.diag([0.1, 0.1, 0.1])  # [N*m/rad]

        # ====== admittance state ========
        self.p_a = np.zeros(3)  # [m]
        self.v_a = np.zeros(3)  # [m/s]
        self.q_a = np.array([0, 0, 0, 1])  # xyzw, same with the order of tf
        self.w_a = np.zeros(3)  # [rad/s]

        self.p_r = np.zeros(3)
        self.q_r = np.array([0, 0, 0, 1])  # xyzw, same with the order of tf

        # ====== publisher ======
        self.tool_wrench_pub = None
        self.pub_ref_traj = None

    def execute(self, userdata):
        rospy.loginfo("Executing state AdmittanceState")
        robot_name = userdata.robot_name

        self.tool_wrench_pub = rospy.Publisher(
            f"/{robot_name}/admittance_tool_frame/f_w_tq_t", WrenchStamped, queue_size=10
        )
        self.pub_ref_traj = rospy.Publisher(f"/{robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=3)

        # get ROS parameters
        self.N_nmpc = rospy.get_param(f"{robot_name}/controller/nmpc/NN")

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

        self.start_time = rospy.Time.now().to_sec()

        # at the beginning, init p_a and q_a
        self.p_a = np.array(
            [
                self.ee_odom_msg.pose.pose.position.x,
                self.ee_odom_msg.pose.pose.position.y,
                self.ee_odom_msg.pose.pose.position.z,
            ]
        )
        self.q_a = np.array(
            [
                self.ee_odom_msg.pose.pose.orientation.x,
                self.ee_odom_msg.pose.pose.orientation.y,
                self.ee_odom_msg.pose.pose.orientation.z,
                self.ee_odom_msg.pose.pose.orientation.w,
            ]
        )

        self.p_r = self.p_a.copy()
        self.q_r = self.q_a.copy()

        while not rospy.is_shutdown():
            # Some contents here for the task execution.

            # print(f"odom: {self.ee_odom_msg}")
            # print(f"wrench: {self.ext_wrench_body_frame_msg}\n")

            tool_force_w, tool_torque_t = self.compute_tool_wrench()
            self.publish_tool_wrench(tool_force_w, tool_torque_t)

            # ===== admittance dynamics =====
            a_lin_a = self.Mp_rev @ (tool_force_w - self.Dp @ self.v_a - self.Kp @ (self.p_a - self.p_r))
            self.v_a = self.v_a + a_lin_a * self.dt
            self.p_a = self.p_a + self.v_a * self.dt

            q_error = quat_multiply(
                -self.q_r[0],
                -self.q_r[1],
                -self.q_r[2],
                self.q_r[3],
                self.q_a[0],
                self.q_a[1],
                self.q_a[2],
                self.q_a[3],
            )

            a_ang_a = self.Mq_rev @ (tool_torque_t - self.Dq @ self.w_a - self.Kq @ q_error[0:3])
            self.w_a = self.w_a + a_ang_a * self.dt
            # update q_a using the current angular velocity w_a
            q_a_dot = 0.5 * quat_multiply(
                self.q_a[0],
                self.q_a[1],
                self.q_a[2],
                self.q_a[3],
                self.w_a[0],
                self.w_a[1],
                self.w_a[2],
                0,
            )
            self.q_a = self.q_a + q_a_dot * self.dt
            self.q_a = self.q_a / np.linalg.norm(self.q_a)

            # ======= pub ==========
            multi_dof_joint_traj = MultiDOFJointTrajectory()
            multi_dof_joint_traj.header.frame_id = "world"
            multi_dof_joint_traj.header.stamp = rospy.Time.now()
            multi_dof_joint_traj.joint_names.append("ee")

            time_from_start = rospy.Time.now().to_sec() - self.start_time
            for i in range(self.N_nmpc + 1):
                traj_pt = MultiDOFJointTrajectoryPoint()
                traj_pt.transforms.append(
                    Transform(
                        translation=Vector3(self.p_a[0], self.p_a[1], self.p_a[2]),
                        rotation=Quaternion(self.q_a[0], self.q_a[1], self.q_a[2], self.q_a[3]),
                    )
                )
                traj_pt.velocities.append(
                    Twist(
                        linear=Vector3(self.v_a[0], self.v_a[1], self.v_a[2]),
                        angular=Vector3(self.w_a[0], self.w_a[1], self.w_a[2]),
                    )
                )
                traj_pt.accelerations.append(
                    Twist(
                        linear=Vector3(a_lin_a[0], a_lin_a[1], a_lin_a[2]),
                        angular=Vector3(a_ang_a[0], a_ang_a[1], a_ang_a[2]),
                    )
                )
                traj_pt.time_from_start = rospy.Duration.from_sec(time_from_start)

                multi_dof_joint_traj.points.append(traj_pt)

            self.pub_ref_traj.publish(multi_dof_joint_traj)

            # ===== sleep =====
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
