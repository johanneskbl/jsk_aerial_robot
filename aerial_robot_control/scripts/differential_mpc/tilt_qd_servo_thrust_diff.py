#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
import casadi as ca
from nmpc.nmpc_tilt_mt.tilt_qd.qd_nmpc_base import QDNMPCBase
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_jetson as phys_jetson

class NMPCTiltQdServoThrustDiff(QDNMPCBase):
    """
    Controller Name: Tiltable Quadrotor NMPC including Differential Servo and Thrust Model 
    The controller itself is constructed in base class. The control inputs are the rates of the tilt and thrust commands.
    This file is used to define the properties of the controller, specifically, the weights and cost function for the acados solver.
    The output of the controller is the thrust and servo angle velocities for each rotor.
    """

    def __init__(self, build: bool = True, phys=phys_jetson):
        # Model name
        self.model_name = "tilt_qd_servo_thrust_diff_mdl"
        self.phys = phys
        self.num_rotors = 4
        self.num_servos = 4

        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = True
        self.include_servo_second_order = False
        self.include_thrust_model = True
        self.include_thrust_derivative = True
        self.include_thrust_second_order = False
        self.include_differential_allocation = False
        self.include_nullspace_control = False
        self.include_cog_dist_model = False
        self.include_cog_dist_parameter = False  # TODO seperation between model and parameter necessary?)
        self.include_impedance = False

        # Read parameters from configuration file in the robot's package
        self.read_params(
            "controller", "nmpc", "beetle_omni", "BeetleNMPCFullServoThrustDiff.yaml"
        )

        # Create acados model & solver and generate c code
        super().__init__(method="differential_mpc", build=build)

    def get_cost_function(self, lin_acc_w=None, ang_acc_b=None, nullspace_proj=None, nullspace_proj_dot=None):
        # fmt: off
        # Cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp_cost.AcadosOcpCost for details
        # NONLINEAR_LS = error^T @ Q @ error; error = y - y_ref
        # qe = qr^* multiply q
        q_wt_w, q_wt_x, q_wt_y, q_wt_z = self._quaternion_multiply(self.qw, self.qx, self.qy, self.qz,
                                                                   self.ee_q[0], self.ee_q[1], self.ee_q[2], self.ee_q[3])

        qe_w, qe_x, qe_y, qe_z = self._quaternion_multiply(self.qwr, -self.qxr, -self.qyr, -self.qzr,
                                                           q_wt_w, q_wt_x, q_wt_y, q_wt_z)

        rot_wb = self._get_rot_wb_ca(self.qw, self.qx, self.qy, self.qz)
        skew_w = self._get_skew_symmetric_matrix(self.w)

        rot_bt = self._get_rot_wb_ca(self.ee_q[0], self.ee_q[1], self.ee_q[2], self.ee_q[3])
        rot_tb = rot_bt.T

        # === NO EE ===
        # qe_x =  self.qwr * self.qx - self.qw * self.qxr - self.qyr * self.qz + self.qy * self.qzr
        # qe_y =  self.qwr * self.qy - self.qw * self.qyr + self.qxr * self.qz - self.qx * self.qzr
        # qe_z = -self.qxr * self.qy + self.qx * self.qyr + self.qwr * self.qz - self.qw * self.qzr
        # =============

        state_y = ca.vertcat(
            self.p + rot_wb @ self.ee_p,
            self.v + rot_wb @ skew_w @ self.ee_p,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            rot_tb @ self.w,
            self.a_s,
            self.ft_s
        )

        if self.include_differential_allocation:
            state_y = ca.vertcat(
                state_y,
                self.fu_b_s,
                self.tau_b_s
            )

        state_y_e = state_y

        # - Nullspace secondary task
        if self.include_nullspace_control:
            # Secondary objective: pull the servo angles back to 0 (and optionally the thrusts
            # to 0) within the nullspace of the allocation Jacobian, i.e. without changing the
            # produced wrench. See NMPCTiltQdServoThrustDiffSecondOrder.get_cost_function for
            # details. In this first-order model the actuator velocities are the control
            # inputs themselves, so the residual lives in control_y (weights Rtd_c, Rad_c).
            thrust_task_gain = self.params["nullspace_thrust_gain"]  # [1/s]
            servo_task_gain = self.params["nullspace_servo_gain"]    # [1/s]
            thrust_target = 0.0
            servo_target = 0.0
            velocity_target = ca.vertcat(
                - thrust_task_gain * (self.ft_s - thrust_target),
                - servo_task_gain * (self.a_s - servo_target),
            )
            control_y = ca.simplify(
                ca.vertcat(self.ftd_c, self.ad_c) - ca.mtimes(nullspace_proj, velocity_target)
            )
        else:
            # Without nullspace control, just minimize actuator velocity
            control_y = ca.vertcat(
                self.ftd_c,
                self.ad_c,
            )

        return state_y, state_y_e, control_y
        # fmt: on

    def get_weights(self):
        return super().get_weights()

    def get_reference(
        self,
        target_xyz,
        target_qwxyz,
        ft_ref,
        a_ref,
        body_forces_ref = None,
        body_torques_ref = None,
    ):
        """
        Assemble reference trajectory from target pose and reference control values.
        Gets called from reference generator class.
        Note: The definition of the reference is closely linked to the definition of the cost function.
        Therefore, this is explicitly stated in each controller file to increase comprehensiveness.

        :param target_xyz: Target position
        :param target_qwxyz: Target quarternions
        :param body_forces_ref: Reference body forces
        :param body_torques_ref: Reference body torques
        :param ad_ref: Reference servo angle derivatives
        :param ftd_ref: Reference thrust derivatives
        :return xr: Reference for the state x
        :return ur: Reference for the input u
        """
        # Get dimensions
        ocp = self.get_ocp()
        nn = ocp.solver_options.N_horizon
        nx = ocp.dims.nx
        nu = ocp.dims.nu

        # Assemble state reference
        xr = np.zeros([nn + 1, nx])
        xr[:, 0] = target_xyz[0]  # x
        xr[:, 1] = target_xyz[1]  # y
        xr[:, 2] = target_xyz[2]  # z
        # No reference for vx, vy, vz (idx: 3, 4, 5)
        xr[:, 6] = target_qwxyz[0]  # qx
        xr[:, 7] = target_qwxyz[1]  # qx
        xr[:, 8] = target_qwxyz[2]  # qy
        xr[:, 9] = target_qwxyz[3]  # qz
        # No reference for wx, wy, wz (idx: 10, 11, 12)
        xr[:, self.servo_start_idx] = a_ref[0]  # TODO: The servo reference is ill defined and therefore should be avoided at all
        xr[:, self.servo_start_idx + 1] = a_ref[1]
        xr[:, self.servo_start_idx + 2] = a_ref[2]
        xr[:, self.servo_start_idx + 3] = a_ref[3]
        xr[:, self.thrust_start_idx] = ft_ref[0]
        xr[:, self.thrust_start_idx + 1] = ft_ref[1]
        xr[:, self.thrust_start_idx + 2] = ft_ref[2]
        xr[:, self.thrust_start_idx + 3] = ft_ref[3]
        if self.include_differential_allocation:
            xr[:, self.wrench_state_start_idx + 0] = body_forces_ref[0]
            xr[:, self.wrench_state_start_idx + 1] = body_forces_ref[1]
            xr[:, self.wrench_state_start_idx + 2] = body_forces_ref[2]
            xr[:, self.wrench_state_start_idx + 3] = body_torques_ref[0]
            xr[:, self.wrench_state_start_idx + 4] = body_torques_ref[1]
            xr[:, self.wrench_state_start_idx + 5] = body_torques_ref[2]

        # Assemble input reference
        # NOTE: Reference has to be zero if variable is included as state in cost function!
        ur = np.zeros([nn, nu])

        # DONT use reference if having a difference in the cost function, since the difference is already minimized by the cost function itself
        # if self.include_thrust_derivative:
        #     ur[:, 0] = ftd_ref[0]
        #     ur[:, 1] = ftd_ref[1]
        #     ur[:, 2] = ftd_ref[2]
        #     ur[:, 3] = ftd_ref[3]
        # if self.include_servo_derivative:
        #     ur[:, 4] = ad_ref[0]
        #     ur[:, 5] = ad_ref[1]
        #     ur[:, 6] = ad_ref[2]
        #     ur[:, 7] = ad_ref[3]

        return xr, ur
