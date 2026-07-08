import sys, os
from copy import deepcopy
import time
import numpy as np
import argparse

from nmpc_tilt_mt.utils.nmpc_viz import Visualizer

# Quadrotor
import nmpc_tilt_mt.tilt_qd.phys_param_beetle_omni as phys_omni
import nmpc_tilt_mt.archive.phys_param_beetle_art as phys_art

# - Naive models
from nmpc_tilt_mt.archive.tilt_qd_no_servo_ac_cost import NMPCTiltQdNoServoAcCost
from nmpc_tilt_mt.tilt_qd.tilt_qd_no_servo import NMPCTiltQdNoServo

# - Consider the servo delay with its model
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo import NMPCTiltQdServo
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_dist import NMPCTiltQdServoDist
from nmpc_tilt_mt.archive.tilt_qd_servo_drag_w_dist import NMPCTiltQdServoDragDist
from nmpc_tilt_mt.archive.tilt_qd_servo_w_cog_end_dist import NMPCTiltQdServoWCogEndDist

# - Conside servo angle derivative as state
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_diff import NMPCTiltQdServoDiff

# - Consider second order servo angle and thrust models and optimize w.r.t. theit velocities
from ..differential_mpc.tilt_qd_servo_thrust_diff import NMPCTiltQdServoThrustDiff

# - Consider the absolute servo angle command in cost
from nmpc_tilt_mt.archive.tilt_qd_servo_old_cost import NMPCTiltQdServoOldCost

# - Consider the thrust delay with its model
from nmpc_tilt_mt.tilt_qd.tilt_qd_thrust import NMPCTiltQdThrust

# - Consider the servo & thrust delay with its models
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_thrust import NMPCTiltQdServoThrust
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist
from nmpc_tilt_mt.archive.tilt_qd_servo_thrust_drag import NMPCTiltQdServoThrustDrag

# Birotor
from nmpc_tilt_mt.tilt_bi.tilt_bi_servo import NMPCTiltBiServo
from nmpc_tilt_mt.tilt_bi.tilt_bi_2ord_servo import NMPCTiltBi2OrdServo

# Trirotor
from nmpc_tilt_mt.tilt_tri.tilt_tri_servo import NMPCTiltTriServo
from nmpc_tilt_mt.tilt_tri.tilt_tri_servo_dist import NMPCTiltTriServoDist


def main(args):
    # ========== Init ==========
    # ---------- Controller ----------
    if args.arch == "qd":
        if args.model == 0:
            nmpc = NMPCTiltQdNoServo(phys=phys_art)
        elif args.model == 1:
            nmpc = NMPCTiltQdServo(phys=phys_art)
        elif args.model == 2:
            nmpc = NMPCTiltQdThrust(phys=phys_art)
        elif args.model == 3:
            nmpc = NMPCTiltQdServoThrust(phys=phys_art)
        elif args.model == 4:
            nmpc = NMPCTiltQdServoThrustDiff(phys=phys_omni)
            a_c_integ = np.zeros(4)
            ft_c_integ = np.zeros(4)

        elif args.model == 21:
            nmpc = NMPCTiltQdServoDist(phys=phys_omni)
        elif args.model == 22:
            nmpc = NMPCTiltQdServoThrustDist(phys=phys_omni)

        # Archived methods
        elif args.model == 91:
            nmpc = NMPCTiltQdNoServoAcCost()
        elif args.model == 92:
            nmpc = NMPCTiltQdServoOldCost()
        elif args.model == 93:
            nmpc = NMPCTiltQdServoDiff()
            a_c_integ = np.zeros(4)
        elif args.model == 94:
            nmpc = NMPCTiltQdServoDragDist()
        elif args.model == 95:
            nmpc = NMPCTiltQdServoThrustDrag()
        elif args.model == 96:
            nmpc = NMPCTiltQdServoWCogEndDist()
        else:
            raise ValueError(f"Invalid control model {args.model}.")

    elif args.arch == "bi":
        if args.model == 0:
            nmpc = NMPCTiltBiServo()
        elif args.model == 1:
            nmpc = NMPCTiltBi2OrdServo()
        else:
            raise ValueError(f"Invalid model {args.model}.")

    elif args.arch == "tri":
        if args.model == 0:
            nmpc = NMPCTiltTriServo()
        elif args.model == 1:
            nmpc = NMPCTiltTriServoDist()
        else:
            raise ValueError(f"Invalid model {args.model}.")

    else:
        raise ValueError(f"Invalid robot architecture {args.arch}.")

    # Get time constants
    if nmpc.include_servo_model:
        t_servo_ctrl = nmpc.phys.t_servo
    else:
        t_servo_ctrl = 0.0
    ts_ctrl = nmpc.params["T_samp"]

    # OCP solver
    ocp_solver = nmpc.get_ocp_solver()
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    n_param = ocp_solver.acados_ocp.dims.np

    x_init = np.zeros(nx)
    x_init[6] = 1.0  # qw
    u_init = np.zeros(nu)

    for stage in range(ocp_solver.N + 1):
        ocp_solver.set(stage, "x", x_init)
    for stage in range(ocp_solver.N):
        ocp_solver.set(stage, "u", u_init)

    # ---------- Simulator ----------
    if args.arch == "qd":
        sim_phy = phys_omni if 20 < args.model < 30 else phys_art
        if args.sim_model == 0:
            sim_nmpc = NMPCTiltQdServoThrust(phys=sim_phy)  # Consider both the servo delay and the thrust delay
        elif args.sim_model == 1:
            sim_nmpc = NMPCTiltQdServoThrustDrag(phys=sim_phy)  # Also consider drag in wrench formulation
        elif args.sim_model == 2:
            sim_nmpc = NMPCTiltQdServoThrustDiff(phys=sim_phy)  # Consider differential servo and thrust models
        else:
            raise ValueError(f"Invalid sim model {args.sim_model}.")

    elif args.arch == "bi":
        if args.sim_model == 0:
            sim_nmpc = NMPCTiltBiServo()
        # elif args.sim_model == 1:
        #     sim_nmpc = NMPCTiltBi2OrdServo()   # This model is wrong
        else:
            raise ValueError(f"Invalid sim model {args.sim_model}.")

    elif args.arch == "tri":
        sim_nmpc = NMPCTiltTriServo()

    else:
        raise ValueError(f"Invalid robot architecture {args.arch}.")

    # Get time constants
    if sim_nmpc.include_servo_model:
        t_servo_sim = sim_nmpc.phys.t_servo
    else:
        t_servo_sim = 0.0
    if sim_nmpc.include_thrust_model:
        t_rotor_sim = sim_nmpc.phys.t_rotor
    else:
        t_rotor_sim = 0.0

    ts_sim = 0.005  # or 0.001

    t_total_sim = 15.0
    if args.plot_type == 1:
        t_total_sim = 4.0
    if args.plot_type == 2:
        t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # Sim solver
    sim_solver = sim_nmpc.create_acados_sim_solver(ts_sim, build=True)
    nx_sim = sim_solver.acados_sim.dims.nx

    # State Initialization
    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw

    # ---------- Reference ----------
    reference_generator = nmpc.get_reference_generator()

    # Disturbance simulation
    impulse_done = False

    # ---------- Visualization ----------
    viz = Visualizer(
        args.arch,
        N_sim,
        nx_sim,
        nu,
        x_init_sim,
        x_lower_constraints=dict(
            list(zip(ocp_solver.acados_ocp.constraints.idxbx, ocp_solver.acados_ocp.constraints.lbx))
        ),
        x_upper_constraints=dict(
            list(zip(ocp_solver.acados_ocp.constraints.idxbx, ocp_solver.acados_ocp.constraints.ubx))
        ),
        u_lower_constraints=(
            [nmpc.params["thrust_min"], nmpc.params["a_min"]] if nmpc.tilt else [nmpc.params["thrust_min"]]
        ),
        u_upper_constraints=(
            [nmpc.params["thrust_max"], nmpc.params["a_max"]] if nmpc.tilt else [nmpc.params["thrust_max"]]
        ),
        tilt=nmpc.tilt,
        include_servo_model=sim_nmpc.include_servo_model,
        include_thrust_model=sim_nmpc.include_thrust_model,
        include_cog_dist_model=sim_nmpc.include_cog_dist_model,
    )

    # Prepare containers to record simulation data (x and u) for future comparison
    x_history = []
    u_history = []

    is_sqp_change = False
    t_sqp_start = 2.5
    t_sqp_end = 3.0

    # ========== Run simulation ==========
    u_cmd = u_init
    t_ctl = 0.0
    x_now_sim = x_init_sim
    for i in range(N_sim):
        # --------- Update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim

        # --------- Update state estimation ---------
        # Assemble state from simulation and disturbance estimation
        x_now = np.zeros(nx)
        x_now[:12] = deepcopy(x_now_sim[:12])  # x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz
        if nmpc.include_servo_model:
            # sim_nmpc needs to also include servo model to simulate servo state
            x_now[nmpc.servo_start_idx:nmpc.servo_end_idx] = deepcopy(
                x_now_sim[sim_nmpc.servo_start_idx:sim_nmpc.servo_end_idx]
            )
        if nmpc.include_servo_second_order:
            # Estimate the current servo derivative for the second-order model
            a_s = x_now_sim[nmpc.servo_start_idx:nmpc.servo_end_idx]  # Current servo angle state
            a_c = u_cmd[4:8]  # Current servo angle command
            ad_s = (a_c - a_s) / t_servo_sim
            # - Add gaussian noise -
            ad_s += np.random.normal(0, 0.1, 4)
            # ----------------------
            x_now[nmpc.servo_vel_start_idx:nmpc.servo_vel_end_idx] = ad_s  # Current servo angle derivatives
        if nmpc.include_thrust_model:
            # sim_nmpc needs to also include thrust model to simulate thrust state
            x_now[nmpc.thrust_start_idx:nmpc.thrust_end_idx] = deepcopy(
                x_now_sim[sim_nmpc.thrust_start_idx:sim_nmpc.thrust_end_idx]
            )
        if nmpc.include_thrust_second_order:
            # Estimate the current thrust derivative for the second-order model
            ft_s = x_now_sim[nmpc.thrust_start_idx:nmpc.thrust_end_idx]  # Current thrust state
            ft_c = u_cmd[0:4]  # Current thrust command
            ftd_s = (ft_c - ft_s) / t_rotor_sim
            # - Add gaussian noise -
            ftd_s += np.random.normal(0, 1, 4)
            # ----------------------
            x_now[nmpc.thrust_vel_start_idx:nmpc.thrust_vel_end_idx] = ftd_s  # Current thrust derivatives
        if nmpc.include_differential_allocation:
            # Calculate the body wrench state
            tilt_angles = x_now[nmpc.servo_start_idx : nmpc.servo_end_idx]
            thrusts = x_now[nmpc.thrust_start_idx : nmpc.thrust_end_idx]
            current_body_wrench = reference_generator.compute_current_body_wrench(tilt_angles, thrusts)
            x_now[nmpc.wrench_state_start_idx : nmpc.wrench_state_end_idx] = current_body_wrench.flatten()
        if nmpc.include_cog_dist_model:
            # Disturbance state is zeroed out for this simulation
            x_now[nmpc.cog_dist_state_start_idx : nmpc.cog_dist_state_end_idx] = np.zeros(3)

        # --------- Update control target ---------
        target_xyz = np.array([[0.0, 0.0, 1.0]]).T
        target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if args.plot_type == 2:
            target_xyz = np.array([[0.0, 0.0, 0.0]]).T
            target_rpy = np.array([[0.5, 0.5, 0.5]]).T

        if t_total_sim > 2.0:
            if 2.0 <= t_now < 6:
                target_xyz = np.array([[0.3, 0.6, 1.0]]).T

                roll = 90.0 / 180.0 * np.pi
                pitch = 0.0 / 180.0 * np.pi
                yaw = 0.0 / 180.0 * np.pi
                target_rpy = np.array([[roll, pitch, yaw]]).T

            # if 3.0 <= t_now < 5.5:
            #     assert t_sqp_end <= 3.0
            #     target_xyz = np.array([[1.0, 1.0, 1.0]]).T
            #     target_rpy = np.array([[0.0, 0.0, 0.0]]).T
            # if t_now >= 5.5:
            #     target_xyz = np.array([[1.0, 1.0, 1.0]]).T

            #     roll = 30.0 / 180.0 * np.pi
            #     pitch = 0.0 / 180.0 * np.pi
            #     yaw = 0.0 / 180.0 * np.pi
            #     target_rpy = np.array([[roll, pitch, yaw]]).T

            if 6 <= t_now < 12:
                assert t_sqp_end <= 3.0
                target_xyz = np.array([[0.3, 0.6, 1.0]]).T
                roll = 0.0 / 180.0 * np.pi
                pitch = 90.0 / 180.0 * np.pi
                yaw = 0.0 / 180.0 * np.pi
                target_rpy = np.array([[roll, pitch, yaw]]).T

            if t_now >= 12:
                target_xyz = np.array([[0.3, 0.6, 1.0]]).T
                roll = 0.0 / 180.0 * np.pi
                pitch = 180.0 / 180.0 * np.pi
                yaw = 0.0 / 180.0 * np.pi
                target_rpy = np.array([[roll, pitch, yaw]]).T

        # Compute reference trajectory from target pose
        xr, ur = reference_generator.compute_trajectory(target_xyz, target_rpy=target_rpy)

        if args.plot_type == 2:
            if nx > 13:
                xr[:, 13:] = 0.0
            if args.arch == "bi":
                ur[:, 2:] = 0.0
            elif args.arch == "tri":
                ur[:, 3:] = 0.0
            elif args.arch == "qd":
                ur[:, 4:] = 0.0

        # --------- Set SQP mode ---------
        if is_sqp_change and t_sqp_start > t_sqp_end:
            if t_now >= t_sqp_start:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP"

            if t_now >= t_sqp_end:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP_RTI"

        # --------- Update solver ---------
        comp_time_start = time.time()
        if t_ctl >= ts_ctrl:
            t_ctl = 0.0

            # Set reference and parameters for nonlinear quaternion error
            # 0 ~ N-1
            for j in range(ocp_solver.N):
                yr = np.concatenate((xr[j, :], ur[j, :]))
                ocp_solver.set(j, "yref", yr)
                quaternion_r = xr[j, 6:10]
                nmpc.acados_parameters[0:4] = quaternion_r
                ocp_solver.set(j, "p", nmpc.acados_parameters)  # For nonlinear quaternion error
            # N
            yr = xr[ocp_solver.N, :]
            ocp_solver.set(ocp_solver.N, "yref", yr)  # Final state of x, no u
            quaternion_r = xr[ocp_solver.N, 6:10]
            nmpc.acados_parameters[0:4] = quaternion_r
            ocp_solver.set(ocp_solver.N, "p", nmpc.acados_parameters)  # For nonlinear quaternion error

            # Compute control feedback and take the first action
            try:
                u_cmd = ocp_solver.solve_for_x0(x_now)
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                break

        comp_time_end = time.time()
        viz.comp_time[i] = (comp_time_end - comp_time_start) * 1000.0  # in ms

        if args.arch == "qd":
            # Use previous servo angle as reference
            if type(nmpc) is NMPCTiltQdNoServoAcCost:
                nmpc.update_a_prev(u_cmd.item(4), u_cmd.item(5), u_cmd.item(6), u_cmd.item(7))

            # Integrate thrust velocity command translating to the thrust command
            if nmpc.include_thrust_derivative:
                ft_c_integ += u_cmd[0:4] * ts_ctrl
                u_cmd[0:4] = ft_c_integ

            # Integrate servo angle velocity command translating to the servo angle command
            if nmpc.include_servo_derivative:
                a_c_integ += u_cmd[4:8] * ts_ctrl
                u_cmd[4:8] = a_c_integ  # convert from delta input to real input

            # Nullspace control for differential allocation
            # TODO: Understand and verify this (especially if indexing is correct)
            # if nmpc.include_nullspace_control:
            #     current_servo_angle = x_now[nmpc.servo_start_idx : nmpc.servo_end_idx]
            #     current_thrust = x_now[nmpc.thrust_start_idx : nmpc.thrust_end_idx]
            #     differential_allocation_mat = (
            #         reference_generator.compute_differential_allocation_matrix(
            #             current_servo_angle, current_thrust
            #         )
            #     )
            #     try:
            #         # Compute nullspace projection matrix
            #         differential_allocation_mat_pinv = np.linalg.pinv(differential_allocation_mat)
            #         nullspace_projection = np.eye(8) - differential_allocation_mat_pinv @ differential_allocation_mat
            #         controls = np.concatenate((current_thrust, current_servo_angle))
            #         u_cmd -= nullspace_projection @ controls
            #     except np.linalg.LinAlgError:
            #         print("Singular allocation matrix encountered. Skipping nullspace optimization for this step.")

            # tilt_rate_limit = 4.0  # rad/s
            # u_cmd[4:8] = np.clip(
            #     u_cmd[4:8],
            #     -tilt_rate_limit * 0.0480 + current_servo_angle,
            #     tilt_rate_limit * 0.0480 + current_servo_angle,
            # )
            # thrust_rate_limit = 50.0  # N/s
            # u_cmd[0:4] = np.clip(
            #     u_cmd[0:4],
            #     -thrust_rate_limit * 0.0942 + current_thrust,
            #     thrust_rate_limit * 0.0942 + current_thrust,
            # )

        # --------- Update simulation ---------
        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", u_cmd)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # --------- Check constraints ---------
        # Boundary constraints
        for idx in ocp_solver.acados_ocp.constraints.idxbx:
            lbxi = np.where(ocp_solver.acados_ocp.constraints.idxbx == idx)[0][0]
            if (
                x_now_sim[idx] < ocp_solver.acados_ocp.constraints.lbx[lbxi]
                or x_now_sim[idx] > ocp_solver.acados_ocp.constraints.ubx[lbxi]
            ):
                print(
                    f"Warning: Constraint violation at index {idx} in simulation step {i}. "
                    f"Value: {x_now_sim[idx]:.14f}, "
                    f"Lower bound: {ocp_solver.acados_ocp.constraints.lbx[lbxi]}, "
                    f"Upper bound: {ocp_solver.acados_ocp.constraints.ubx[lbxi]}"
                )
        # Nonlinear unit quaternion constraint
        quat_norm = np.linalg.norm(x_now_sim[6:10])
        if quat_norm < 0.999 or quat_norm > 1.001:
            print(
                f"Warning: Constraint violation for unit_q in simulation step {i}. "
                f"Value: {quat_norm:.14f} != 1.0, "
                f"Quaternion: {x_now_sim[6:10]}"
            )

        # --------- Log simulation data ---------
        x_history.append(x_now_sim.copy())
        u_history.append(u_cmd.copy())

        # --------- Update visualizer ---------
        if not nmpc.include_servo_second_order or not nmpc.include_thrust_second_order:
            viz.update(i, x_now_sim, u_cmd)  # NOTE: The recording frequency of u_cmd is the same as ts_sim
        else:
            ad_s = x_now[nmpc.servo_vel_start_idx:nmpc.servo_vel_end_idx]
            ftd_s = x_now[nmpc.thrust_vel_start_idx:nmpc.thrust_vel_end_idx]
            actuator_vel = np.concatenate((ftd_s, ad_s))
            viz.update(i, x_now_sim, actuator_vel)

    # ========== Visualize ==========
    if not args.no_viz:
        if args.plot_type == 0:
            viz.visualize(
                ocp_solver.acados_ocp.model.name,
                sim_solver.model_name,
                ts_ctrl,
                ts_sim,
                t_total_sim,
                t_servo_ctrl=t_servo_ctrl,
                t_servo_sim=t_servo_sim,
            )
        elif args.plot_type == 1:
            viz.visualize_less(ts_sim, t_total_sim)
        elif args.plot_type == 2:
            viz.visualize_rpy(ocp_solver.acados_ocp.model.name, ts_sim, t_total_sim)

    if args.save_data:
        file_path = args.file_path

        np.savez(
            file_path + f"nmpc_{type(nmpc).__name__}_sim_{type(sim_nmpc).__name__}.npz",
            x=np.array(x_history),
            u=np.array(u_history),
        )

    return np.array(x_history), np.array(u_history)


if __name__ == "__main__":
    # fmt: off
    # Read command line arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models.")
    parser.add_argument(
        "-model",
        "--model",
        type=int,
        default=0,
        help="The NMPC model to be simulated. "
             "Options: 0 (basic model), 1 (servo), "
             "2 (thrust), 3(servo+thrust), "
             "21 (servo+dist), 22 (servo+thrust+dist), "
             "91(no_servo_new_cost), 92(servo_old_cost), "
             "93(servo_diff), 94(servo+drag+dist), "
             "95 (servo+thrust+drag), 96 (servo+drag_param+dist).",
    )

    parser.add_argument(
        "-sim",
        "--sim_model",
        type=int,
        default=0,
        help="The simulation model. "
             "Options: 0 (default: servo+thrust), "
             "1 (servo+thrust+drag).",
    )

    parser.add_argument(
        "-p",
        "--plot_type",
        type=int,
        default=0,
        help="The type of plot. "
             "Options: 0 (default: full), 1 (less), 2 (only rpy)."
    )

    parser.add_argument(
        "-a",
        "--arch",
        type=str,
        default='qd',
        help="The robot's architecture. Options: bi, tri, qd (default)."
    )

    parser.add_argument(
        "--no_viz",
        action="store_true",
        help="Disable visualization after simulation. Note that this is different from the plot_type option, "
        "because plot_type also decides the simulation parameters.",
    )

    parser.add_argument(
        "-s",
        "--save_data",
        action="store_true",
        help="Save simulation x and u data to file"
    )

    parser.add_argument(
        "--file_path",
        type=str,
        default=f"../../../../test/data/",
        help="Path to save the data file"
    )

    args = parser.parse_args()
    main(args)
    # fmt: on
