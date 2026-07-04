#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
import casadi as ca
from nmpc.nmpc_tilt_mt.tilt_qd.qd_nmpc_base import QDNMPCBase
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni


class NMPCTiltQdServoThrustDiff(QDNMPCBase):
    """
    Controller Name: Tiltable Quadrotor NMPC including Differential Servo and Thrust Model 
    The controller itself is constructed in base class. The control inputs are the rates of the tilt and thrust commands.
    This file is used to define the properties of the controller, specifically, the weights and cost function for the acados solver.
    The output of the controller is the thrust and servo angle velocities for each rotor.
    """

    def __init__(self, build: bool = True, phys=phys_omni):
        # Model name
        self.model_name = "tilt_qd_servo_thrust_diff_mdl"
        self.phys = phys
        self.num_rotors = 4

        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = True
        self.include_thrust_model = True
        self.include_thrust_derivative = True
        self.include_cog_dist_model = False
        self.include_cog_dist_parameter = False  # TODO seperation between model and parameter necessary?)
        self.include_impedance = False

        self.include_differential_allocation = False

        # Read parameters from configuration file in the robot's package
        self.read_params(
            "controller", "nmpc", "beetle_omni", "BeetleNMPCFullServoThrustDiff.yaml"
        )

        # Create acados model & solver and generate c code
        super().__init__(method="differential_mpc", build=True)

    def get_cost_function(self, lin_acc_w=None, ang_acc_b=None, nullspace_proj=None):
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

        state_y = ca.vertcat(
            self.p + rot_wb @ self.ee_p,
            self.v + rot_wb @ skew_w @ self.ee_p,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            rot_tb @ self.w,
            self.a_s,
            self.ft_s,
        )

        if self.include_differential_allocation:
            state_y = ca.vertcat(
                state_y,
                self.fu_b_s,
                self.tau_u_b_s
            )

        state_y_e = state_y

        if not self.include_servo_derivative and not self.include_thrust_derivative:
            if self.include_servo_model and self.include_thrust_model:
                control_y = ca.vertcat(
                    self.ft_c - self.ft_s,
                    self.a_c - self.a_s,
                )
            else:
                raise ValueError("Currently, this controller requires both servo and thrust models to be included in the model.")
        else:
            if self.include_servo_derivative and self.include_thrust_derivative:
                control_y = ca.vertcat(
                    self.ftd_c,
                    self.ad_c,
                )
            else:
                raise ValueError("Currently, this controller requires both servo and thrust derivatives to be included in the model.")

        # - Nullspace projection
        if self.include_differential_allocation:
            # Target actuator states (bring prop speed to 0)
            target_gain = 0.6
            thrust_target = 0.0
            actuators_target = -ca.vertcat(self.ft_s - thrust_target, 0,0,0,0)
            time_constant_matrix = ca.diag(ca.vertcat([self.phys.t_rotor]*4, [self.phys.t_servo]*4))

            control_y = ca.simplify(target_gain * ca.mtimes(ca.mtimes(time_constant_matrix, nullspace_proj), actuators_target) - control_y)

        return state_y, state_y_e, control_y
        # fmt: on

    def get_weights(self):
        # Define Weights
        Q = np.diag(
            [
                self.params["Qp_xy"],
                self.params["Qp_xy"],
                self.params["Qp_z"],
                self.params["Qv_xy"],
                self.params["Qv_xy"],
                self.params["Qv_z"],
                0,
                self.params["Qq_xy"],
                self.params["Qq_xy"],
                self.params["Qq_z"],
                self.params["Qw_xy"],
                self.params["Qw_xy"],
                self.params["Qw_z"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qt"],
                self.params["Qt"],
                self.params["Qt"],
                self.params["Qt"]
            ]
        )
        if self.include_differential_allocation:
            Q = np.diag(
                [
                    self.params["Qp_xy"],
                    self.params["Qp_xy"],
                    self.params["Qp_z"],
                    self.params["Qv_xy"],
                    self.params["Qv_xy"],
                    self.params["Qv_z"],
                    0,
                    self.params["Qq_xy"],
                    self.params["Qq_xy"],
                    self.params["Qq_z"],
                    self.params["Qw_xy"],
                    self.params["Qw_xy"],
                    self.params["Qw_z"],
                    self.params["Qa"],
                    self.params["Qa"],
                    self.params["Qa"],
                    self.params["Qa"],
                    self.params["Qt"],
                    self.params["Qt"],
                    self.params["Qt"],
                    self.params["Qt"],
                    self.params["Qfu"],
                    self.params["Qfu"],
                    self.params["Qfu"],
                    self.params["Qtau"],
                    self.params["Qtau"],
                    self.params["Qtau"],
                ]
            )
        print("Q: \n", Q)

        if not self.include_servo_derivative and not self.include_thrust_derivative:
            R = np.diag(
                [
                    self.params["Rt_c"],
                    self.params["Rt_c"],
                    self.params["Rt_c"],
                    self.params["Rt_c"],
                    self.params["Ra_c"],
                    self.params["Ra_c"],
                    self.params["Ra_c"],
                    self.params["Ra_c"],
                ]
            )
        else:
            R = np.diag(
                [
                    self.params["Rtd_c"],
                    self.params["Rtd_c"],
                    self.params["Rtd_c"],
                    self.params["Rtd_c"],
                    self.params["Rad_c"],
                    self.params["Rad_c"],
                    self.params["Rad_c"],
                    self.params["Rad_c"],
                ]
            )
        print("R: \n", R)

        return Q, R

    def get_reference(
        self,
        target_xyz,
        target_qwxyz,
        ft_ref,
        a_ref,
        body_forces_ref,
        body_torques_ref,
        ad_ref,
        ftd_ref,
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
        xr[:, 13] = a_ref[0]
        xr[:, 14] = a_ref[1]
        xr[:, 15] = a_ref[2]
        xr[:, 16] = a_ref[3]
        xr[:, 17] = ft_ref[0]
        xr[:, 18] = ft_ref[1]
        xr[:, 19] = ft_ref[2]
        xr[:, 20] = ft_ref[3]
        if self.include_differential_allocation:
            xr[:, 21] = body_forces_ref[0]
            xr[:, 22] = body_forces_ref[1]
            xr[:, 23] = body_forces_ref[2]
            xr[:, 24] = body_torques_ref[0]
            xr[:, 25] = body_torques_ref[1]
            xr[:, 26] = body_torques_ref[2]

        # Assemble input reference
        # NOTE: Reference has to be zero if variable is included as state in cost function!
        ur = np.zeros([nn, nu])
        if self.include_thrust_derivative:
            ur[:, 0] = ftd_ref[0]
            ur[:, 1] = ftd_ref[1]
            ur[:, 2] = ftd_ref[2]
            ur[:, 3] = ftd_ref[3]
        if self.include_servo_derivative:
            ur[:, 4] = ad_ref[0]
            ur[:, 5] = ad_ref[1]
            ur[:, 6] = ad_ref[2]
            ur[:, 7] = ad_ref[3]

        return xr, ur
