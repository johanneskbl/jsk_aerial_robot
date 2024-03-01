'''
 Created by li-jinjie on 24-2-29.
'''
from spinal.msg import Imu
from nav_msgs.msg import Odometry

import numpy as np
import casadi as ca
import rospy
import os
import rospkg
import yaml

# ========================
# read parameters from yaml
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("beetle"), "config", "BeetleNMPCFull.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)

nmpc_params = param_dict["controller"]["nmpc"]
nmpc_params["N_node"] = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])

physical_params = param_dict["controller"]["physical"]
mass = physical_params["mass"]
gravity = physical_params["gravity"]
Ixx = physical_params["inertia_diag"][0]
Iyy = physical_params["inertia_diag"][1]
Izz = physical_params["inertia_diag"][2]
dr1 = physical_params["dr1"]
p1_b = physical_params["p1"]
dr2 = physical_params["dr2"]
p2_b = physical_params["p2"]
dr3 = physical_params["dr3"]
p3_b = physical_params["p3"]
dr4 = physical_params["dr4"]
p4_b = physical_params["p4"]
kq_d_kt = physical_params["kq_d_kt"]


class DisturbEstNode:
    def __init__(self):
        self.node_name = "disturb_est_node"
        rospy.init_node(self.node_name, anonymous=True)
        self.namespace = rospy.get_namespace()

        self.disturb_wrench = np.zeros(6)

        self.func_wrench = gen_model_input_2_wrench()

        # subscribe
        self.sub_odom = rospy.Subscriber("uav/cog/odom", Odometry, self.callback_odom, queue_size=1)
        self.sub_imu = rospy.Subscriber("imu", Imu, self.callback_imu, queue_size=1)

        pass

    def estimate_disturb_wrench(self, q, ft, a):
        disturb_wrench_cog = self.func_wrench(q, ft, a)
        pass

    def callback_odom(self, msg: Odometry):
        pass

    def callback_imu(self, msg: Imu):
        acc_real = msg.acc_data
        pass


def gen_model_input_2_wrench():
    qw = ca.SX.sym("qw")
    qx = ca.SX.sym("qx")
    qy = ca.SX.sym("qy")
    qz = ca.SX.sym("qz")
    q = ca.vertcat(qw, qx, qy, qz)

    wx = ca.SX.sym("wx")
    wy = ca.SX.sym("wy")
    wz = ca.SX.sym("wz")
    w = ca.vertcat(wx, wy, wz)

    # control inputs
    ft1 = ca.SX.sym("ft1")
    ft2 = ca.SX.sym("ft2")
    ft3 = ca.SX.sym("ft3")
    ft4 = ca.SX.sym("ft4")
    ft = ca.vertcat(ft1, ft2, ft3, ft4)
    a1 = ca.SX.sym("a1")
    a2 = ca.SX.sym("a2")
    a3 = ca.SX.sym("a3")
    a4 = ca.SX.sym("a4")
    a = ca.vertcat(a1, a2, a3, a4)

    # transformation matrix
    row_1 = ca.horzcat(
        ca.SX(1 - 2 * qy ** 2 - 2 * qz ** 2), ca.SX(2 * qx * qy - 2 * qw * qz), ca.SX(2 * qx * qz + 2 * qw * qy)
    )
    row_2 = ca.horzcat(
        ca.SX(2 * qx * qy + 2 * qw * qz), ca.SX(1 - 2 * qx ** 2 - 2 * qz ** 2), ca.SX(2 * qy * qz - 2 * qw * qx)
    )
    row_3 = ca.horzcat(
        ca.SX(2 * qx * qz - 2 * qw * qy), ca.SX(2 * qy * qz + 2 * qw * qx), ca.SX(1 - 2 * qx ** 2 - 2 * qy ** 2)
    )
    rot_ib = ca.vertcat(row_1, row_2, row_3)
    rot_bi = ca.transpose(rot_ib)

    den = np.sqrt(p1_b[0] ** 2 + p1_b[1] ** 2)
    rot_be1 = np.array([[p1_b[0] / den, -p1_b[1] / den, 0], [p1_b[1] / den, p1_b[0] / den, 0], [0, 0, 1]])

    den = np.sqrt(p2_b[0] ** 2 + p2_b[1] ** 2)
    rot_be2 = np.array([[p2_b[0] / den, -p2_b[1] / den, 0], [p2_b[1] / den, p2_b[0] / den, 0], [0, 0, 1]])

    den = np.sqrt(p3_b[0] ** 2 + p3_b[1] ** 2)
    rot_be3 = np.array([[p3_b[0] / den, -p3_b[1] / den, 0], [p3_b[1] / den, p3_b[0] / den, 0], [0, 0, 1]])

    den = np.sqrt(p4_b[0] ** 2 + p4_b[1] ** 2)
    rot_be4 = np.array([[p4_b[0] / den, -p4_b[1] / den, 0], [p4_b[1] / den, p4_b[0] / den, 0], [0, 0, 1]])

    rot_e1r1 = ca.vertcat(
        ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a1), -ca.sin(a1)), ca.horzcat(0, ca.sin(a1), ca.cos(a1))
    )
    rot_e2r2 = ca.vertcat(
        ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a2), -ca.sin(a2)), ca.horzcat(0, ca.sin(a2), ca.cos(a2))
    )
    rot_e3r3 = ca.vertcat(
        ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a3), -ca.sin(a3)), ca.horzcat(0, ca.sin(a3), ca.cos(a3))
    )
    rot_e4r4 = ca.vertcat(
        ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a4), -ca.sin(a4)), ca.horzcat(0, ca.sin(a4), ca.cos(a4))
    )

    # inertial
    iv = ca.diag([Ixx, Iyy, Izz])
    inv_iv = ca.diag([1 / Ixx, 1 / Iyy, 1 / Izz])
    g_i = np.array([0, 0, -gravity])

    # wrench
    ft_r1 = ca.vertcat(0, 0, ft1)
    ft_r2 = ca.vertcat(0, 0, ft2)
    ft_r3 = ca.vertcat(0, 0, ft3)
    ft_r4 = ca.vertcat(0, 0, ft4)

    tau_r1 = ca.vertcat(0, 0, -dr1 * ft1 * kq_d_kt)
    tau_r2 = ca.vertcat(0, 0, -dr2 * ft2 * kq_d_kt)
    tau_r3 = ca.vertcat(0, 0, -dr3 * ft3 * kq_d_kt)
    tau_r4 = ca.vertcat(0, 0, -dr4 * ft4 * kq_d_kt)

    f_u_b = (
            ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1))
            + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2))
            + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3))
            + ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, ft_r4))
    )
    tau_u_b = (
            ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, tau_r1))
            + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, tau_r2))
            + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, tau_r3))
            + ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, tau_r4))
            + ca.cross(np.array(p1_b), ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1)))
            + ca.cross(np.array(p2_b), ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2)))
            + ca.cross(np.array(p3_b), ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3)))
            + ca.cross(np.array(p4_b), ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, ft_r4)))
    )

    f_b = f_u_b + ca.mtimes(rot_bi, g_i * mass)
    tau_b = -ca.cross(w, ca.mtimes(iv, w)) + tau_u_b

    wrench_b = ca.vertcat(f_b, tau_b)

    func = ca.Function("func", [q, ft, a], [wrench_b], ["quaternion", "thrust", "servo_angle"], ["wrench_b"])

    return func
