#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
import casadi as ca
from nmpc.nmpc_tilt_mt.tilt_qd.qd_nmpc_base import QDNMPCBase
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni


class NMPCTiltQdServoThrustDiffSecondOrder(QDNMPCBase):
    """
    Controller Name: Tiltable Quadrotor NMPC including Differential Servo and Thrust Model 
    The controller itself is constructed in base class. The control inputs are the rates of the tilt and thrust commands.
    This file is used to define the properties of the controller, specifically, the weights and cost function for the acados solver.
    The output of the controller is the thrust and servo angle velocities for each rotor.
    """

    def __init__(self, build: bool = True, phys=phys_omni):
        # Model name
        self.model_name = "tilt_qd_servo_thrust_diff_second_order_mdl"
        self.phys = phys
        self.num_rotors = 4
        self.num_servos = 4

        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = True
        self.include_servo_second_order = True
        self.include_thrust_model = True
        self.include_thrust_derivative = True
        self.include_thrust_second_order = True
        self.include_differential_allocation = True
        self.include_nullspace_control = True
        self.include_cog_dist_model = False
        self.include_cog_dist_parameter = False  # TODO seperation between model and parameter necessary?)
        self.include_impedance = False

        # Read parameters from configuration file in the robot's package
        self.read_params(
            "controller", "nmpc", "beetle_omni", "BeetleNMPCFullServoThrustDiffSecondOrder.yaml"
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

        if self.include_nullspace_control:
            # Secondary objective: pull the servo angles back to 0 (and optionally the thrusts
            # to 0) with a P-term for a velocity target vd = - gain * (state - target).
            # Multiplying vd with the nullspace projector N of the wrench Jacobian keeps
            # only the wrench-invariant part of that motion, so the secondary task cannot
            # interfere with the pose/wrench tracking objective. This matters e.g. for
            # cartwheeling: an unloaded arm (zero thrust) is wrench-wise free, so N passes its
            # servo target through and the arm is re-centered for the next rotation instead of
            # being left at its previous angle.
            nullspace_thrust_gain = self.params["nullspace_thrust_gain"]  # [1/s]
            nullspace_servo_gain = self.params["nullspace_servo_gain"]    # [1/s]
            thrust_target = 0.0
            servo_target = 0.0
            velocity_target = ca.vertcat(
                - nullspace_thrust_gain * (self.ft_s - thrust_target),
                - nullspace_servo_gain * (self.a_s - servo_target),
            )
            # Residual = actuator velocity - achievable (nullspace) part of the target velocity.
            # NOTE!: the reference for the velocity entries in the cost must stay 0.
            velocity_residual = ca.simplify(
                ca.vertcat(self.ftd_s, self.ad_s) - ca.mtimes(nullspace_proj, velocity_target)
            )
            ftd_y = velocity_residual[0:4]
            ad_y = velocity_residual[4:8]
        else:
            # Without nullspace control, directly track a reference
            ftd_y = self.ftd_s
            ad_y = self.ad_s

        state_y = ca.vertcat(
            self.p + rot_wb @ self.ee_p,
            self.v + rot_wb @ skew_w @ self.ee_p,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            rot_tb @ self.w,
            self.a_s,
            ad_y,
            self.ft_s,
            ftd_y,
        )

        if self.include_differential_allocation:
            state_y = ca.vertcat(
                state_y,
                self.fu_b_s,
                self.tau_b_s
            )

        state_y_e = state_y


        ############## FROM EUGENIO ##############
        # # - Nullspace projection
        # if self.include_nullspace_control:
        #     # TODO Check if correct especially if control_y should be left or right of the minus
        #     control_y = ca.simplify(actuator_velocity_y - control_y)
        #     # control_y = ca.simplify(target_gain * ca.mtimes(ca.mtimes(time_constant_matrix, nullspace_proj), actuators_target) - ca.vertcat(self.ft_c - self.ft_s, self.a_c - self.a_s))

        #     # time_contant_matrix_inv = ca.diag(ca.vertcat([1/self.t_rotor]*self.num_rotors, [1/self.t_servo]*self.num_rotors))
        #     # actuators_target_jacobian = ca.jacobian(actuators_target, stacked_actuators)
        #     # control_y = ca.simplify(
        #     #     ca.mtimes(time_contant_matrix_inv, ca.vertcat(self.ftd_c - self.ftd_s, self.ad_c - self.ad_s))
        #     #     - ca.mtimes(nullspace_proj_dot, actuators_target)
        #     #     - ca.mtimes(ca.mtimes(nullspace_proj, actuators_target_jacobian), stacked_actuator_velocities))
        # else:
        #     # Without nullspace control, just minimize actuator acceleration
        #     # TODO check if correct!!!!!!!!
        #     # control_y = ca.vertcat(
        #     #     self.ftd_c,
        #     #     self.ad_c,
        #     # )
        ########################################

        # Damp the actuator accelerations for smooth commands. From the second-order dynamics
        # the acceleration is (velocity command - velocity state) / time constant, so this
        # residual is the acceleration scaled by the respective actuator time constant.
        # The nullspace secondary task is handled purely at velocity level in state_y above.
        control_y = ca.vertcat(
            self.ftd_c - self.ftd_s,
            self.ad_c - self.ad_s,
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
        body_torques_ref = None
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
        if not self.include_nullspace_control:
            xr[:, self.servo_start_idx + 0] = a_ref[0]  # TODO: The servo reference is ill defined and therefore should be avoided at all
            xr[:, self.servo_start_idx + 1] = a_ref[1]
            xr[:, self.servo_start_idx + 2] = a_ref[2]
            xr[:, self.servo_start_idx + 3] = a_ref[3]
            # No reference for servo velocity
            xr[:, self.thrust_start_idx + 0] = ft_ref[0]
            xr[:, self.thrust_start_idx + 1] = ft_ref[1]
            xr[:, self.thrust_start_idx + 2] = ft_ref[2]
            xr[:, self.thrust_start_idx + 3] = ft_ref[3]
            # No reference for thrust velocity
        else:
            if self.params["Qa"] > 0.0 or self.params["Qt"] > 0.0:
                raise ValueError(
                    "Qa and Qt must be 0 when including nullspace control, as the actuator states are not tracked to a reference but only to the nullspace target."
                )
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
