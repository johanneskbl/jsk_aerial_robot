import time
import numpy as np
import matplotlib.pyplot as plt

from sim_environment.sim_solver import create_acados_sim_solver
from utils.controller_utils import check_state_constraints, check_input_constraints
from utils.model_utils import set_linearization_params, set_linearization_params_sim, set_l4casadi_params, set_l4casadi_params_sim, set_delayed_states_as_params
from utils.reference_utils import sample_random_position_target, sample_random_orientation_target
from utils.geometry_utils import unit_quaternion, euclidean_dist, quaternion_dist
from visualize_comparison_icra_2026 import plot_comparison
from config.configurations import EnvConfig
from neural_controller import NeuralMPC


def main(model_options, solver_options, dataset_options, sim_options, run_options):
    """
    IDEA: Control with neural model, simulate with neural model (trained on real data), record data for training
    """
    np.random.seed(sim_options["seed"])  # Set seed for reproducibility

    # ------------------------
    # TODO set these somewhere else
    # Model options
    # model_options.update({
    #     "MODEL_ID": 0,
    #     "version": 1,
    #     "name": "test_model"
    # })
    T_sim = 0.005  # or 0.001
    T_prop_step = T_sim  # 0.001
    # ------------------------

    # --- Initialize simulator ---
    model_options["only_use_nominal"] = False
    model_options["neural_model_instance"] = "neuralmodel_185"
    sim_neural_mpc = NeuralMPC(
        model_options=model_options,
        solver_options=solver_options,
        sim_options=sim_options,
        run_options=run_options,
        use_as_simulator=True,
    )
    sim_model_neural = sim_neural_mpc.get_acados_model()
    sim_solver_neural = create_acados_sim_solver(sim_neural_mpc, sim_model_neural, T_sim)

    # --- Initialize controllers ---
    # Controller trained on simulated labels (i.e., nominal controller in neural simulator)
    model_options["only_use_nominal"] = False
    solver_options["include_floor_bounds"] = True
    neural_mpc = NeuralMPC(
        model_options=model_options, solver_options=solver_options, sim_options=sim_options, run_options=run_options
    )
    ocp_solver_neural = neural_mpc.get_ocp_solver()
    ocp_model_neural = neural_mpc.get_acados_model()
    reference_generator = neural_mpc.get_reference_generator()

    # Nominal controller
    model_options["only_use_nominal"] = True
    nominal_mpc = NeuralMPC(
        model_options=model_options, solver_options=solver_options, sim_options=sim_options, run_options=run_options
    )
    ocp_solver_nominal = nominal_mpc.get_ocp_solver()

    # Recover some necessary variables from the MPC object
    nx = ocp_model_neural.x.shape[0]
    nu = ocp_model_neural.u.shape[0]
    N = neural_mpc.N
    T_samp = neural_mpc.T_samp  # Time step for the control loop

    # --- Set initial state ---
    if run_options["initial_state"] is None:
        # state = [p, v, q, w, (a and/or t and/or ds)]
        state_curr_neural = np.zeros(nx)
        state_curr_neural[6] = 1.0  # Real part of quaternion
    else:
        state_curr_neural = run_options["initial_state"]
    state_curr_nominal = state_curr_neural.copy()
    state_curr_sim_neural = state_curr_neural.copy()
    state_curr_sim_nominal = state_curr_neural.copy()

    # --- Warm up solver ---
    x_l = np.zeros((0, nx))
    u_l = np.zeros((0, nu))
    for i in range(N+1):
        x_l = np.append(x_l, ocp_solver_neural.get(i, "x")[np.newaxis,:], axis=0)
        u_l = np.append(u_l, ocp_solver_neural.get(i, "u")[np.newaxis,:] if i < N else ocp_solver_neural.get(N-1, "u")[np.newaxis,:], axis=0)
    if model_options["use_l4casadi"]:
        for i in range(20):
            neural_mpc.learned_dyn_model.get_params(np.concatenate([x_l[:, neural_mpc.state_feats], u_l[:, neural_mpc.u_feats]], axis=1))
    for _ in range(20):
        u_temp = ocp_solver_neural.solve_for_x0(state_curr_neural)
        sim_solver_neural.simulate(x=state_curr_sim_neural, u=u_temp, p=sim_solver_neural.acados_sim.parameter_values)
    x_l = []

    # --- Set up running history for delayed neural networks ---
    if neural_mpc.use_mlp and "delay" in neural_mpc.mlp_metadata["NetworkConfig"]["model_name"]:
        delay = neural_mpc.mlp_metadata["NetworkConfig"]["delay_horizon"]  # Delay as number of time steps into the past
        history = np.tile(np.append(state_curr_neural, np.zeros((nu,))), (delay, 1))

    # --- Set target states ---
    if run_options["preset_targets"] is not None:
        targets = run_options["preset_targets"]
    else:
        # Takeoff
        targets = np.array([0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0])[np.newaxis, :]
    tracking_mode = "position"
    targets_reached = np.array([False for _ in targets])

    plot = run_options["plot_trajectory"]
    if plot:
        target_dim = targets.shape[1]
        state_dim = nx
        control_dim = nu
        rec_dict = {
            "timestamp": np.zeros((0, 1)),
            "dt": np.zeros((0, 1)),
            "comp_time_neural": np.zeros((0, 1)),
            "comp_time_nominal": np.zeros((0, 1)),
            "target": np.zeros((0, target_dim)),
            "state_ref": np.zeros((0, state_dim)),
            "state_in_neural": np.zeros((0, state_dim)),
            "state_in_nominal": np.zeros((0, state_dim)),
            "state_out_neural": np.zeros((0, state_dim)),
            "state_out_nominal": np.zeros((0, state_dim)),
            "control_neural": np.zeros((0, control_dim)),
            "control_nominal": np.zeros((0, control_dim)),
        }

    # --- Set up simulation ---
    u_cmd_neural = None
    u_cmd_nominal = None
    i = 0
    j = 0
    t_now = 0.0  # Total virtual time in seconds

    # ---------- Targets loop ----------
    print("Targets reached:")
    while False in targets_reached:
        # --- Target ---
        current_target_idx = np.where(targets_reached == False)[0][0]
        current_target = targets[current_target_idx]
        current_target_reached = False

        # --------- MPC loop ---------
        global_comp_time = time.time()
        while not current_target_reached:
            # --- Emergency recovery ---
            if i > 1000:
                print("===== Emergency recovery triggered!!! =====")
                print(f"Iteration: {i}")
                print(f"Euclidean dist: {(current_target[:3] - state_curr_sim_neural[:3]) ** 2}")
                print(f"Current state: {state_curr_sim_neural}")
                i = 0
                ocp_solver_neural.set(0, "x", state_ref[-1, :])
                ocp_solver_nominal.set(0, "x", state_ref[-1, :])
                sim_solver_neural.set("x", state_ref[-1, :])
                state_curr_neural = state_ref[-1, :]
                state_curr_nominal = state_ref[-1, :]

            # --- Reference ---
            # Compute reference for Input u with an allocation matrix - TODO still makes sense if we don't know model in the first place?
            # Alternative is setting the modular trajectory yref dynamically in control loop
            state_ref, control_ref = reference_generator.compute_trajectory(
                target_xyz=current_target[:3], target_rpy=current_target[6:9]
            )
            # Track reference in solver over horizon
            neural_mpc.track(ocp_solver_neural, state_ref, control_ref, u_cmd_neural)
            nominal_mpc.track(ocp_solver_nominal, state_ref, control_ref, u_cmd_nominal)

            # --- Get current state ---
            if u_cmd_neural is None:
                # If no command is available, use initial/last state
                sim_solver_neural.set("x", state_curr_neural)  # doesn't work
                u_cmd_neural = np.zeros((nu,))
            else:
                state_curr_neural = state_curr_sim_neural.copy()
                check_state_constraints(ocp_solver_neural, state_curr_neural, i)

            if u_cmd_nominal is None:
                u_cmd_nominal = np.zeros((nu,))
            else:
                state_curr_nominal = state_curr_sim_nominal.copy()
                check_state_constraints(ocp_solver_nominal, state_curr_nominal, i)

            if model_options["linearize_mlp"]:
                set_linearization_params(neural_mpc, ocp_solver_neural)

            elif model_options["use_l4casadi"]:
                set_l4casadi_params(neural_mpc, ocp_solver_neural)

            # --- Prepare delayed neural network input ---
            if "delay" in neural_mpc.mlp_metadata["NetworkConfig"]["model_name"]:
                set_delayed_states_as_params(neural_mpc, ocp_solver_neural, history, u_cmd_neural)

            # --- Set parameters in OCP solver ---
            for j in range(ocp_solver_neural.N + 1):
                ocp_solver_neural.set(j, "p", neural_mpc.acados_parameters[j, :])
            for j in range(ocp_solver_nominal.N + 1):
                ocp_solver_nominal.set(j, "p", nominal_mpc.acados_parameters[j, :])

            ############################################################################################
            # --- Optimize control input ---
            # Compute control feedback and take the first action
            comp_time_neural = time.time()
            u_cmd_neural = ocp_solver_neural.solve_for_x0(state_curr_neural)
            comp_time_neural = (time.time() - comp_time_neural) * 1000  # in ms

            comp_time_nominal = time.time()
            u_cmd_nominal = ocp_solver_nominal.solve_for_x0(state_curr_nominal)
            comp_time_nominal = (time.time() - comp_time_nominal) * 1000  # in ms

            # --- Sanity check constraints ---
            check_input_constraints(neural_mpc, u_cmd_neural, i)
            check_input_constraints(nominal_mpc, u_cmd_nominal, i)
            ############################################################################################

            # --- Running history for delayed neural networks ---
            if "delay" in neural_mpc.mlp_metadata["NetworkConfig"]["model_name"]:
                # Append current state and control to history for next iteration
                # Sorted from newest to oldest
                history = history[:-1, :]
                history = np.append(np.append(state_curr_neural, u_cmd_neural)[np.newaxis, :], history, axis=0)

            # --- Record time, reference, current state and last optimized input ---
            if plot:
                rec_dict["timestamp"] = np.append(rec_dict["timestamp"], t_now)
                rec_dict["dt"] = np.append(  # -2 since current timestamp is just added on index -1
                    rec_dict["dt"], t_now - rec_dict["timestamp"][-2] if len(rec_dict["timestamp"]) > 1 else T_samp
                )
                rec_dict["comp_time_neural"] = np.append(rec_dict["comp_time_neural"], comp_time_neural)
                rec_dict["comp_time_nominal"] = np.append(rec_dict["comp_time_nominal"], comp_time_nominal)
                rec_dict["target"] = np.append(rec_dict["target"], current_target[np.newaxis, :], axis=0)
                rec_dict["state_ref"] = np.append(rec_dict["state_ref"], state_ref[0:1, :], axis=0)  # Assuming constant ref
                rec_dict["state_in_neural"] = np.append(
                    rec_dict["state_in_neural"], state_curr_neural[np.newaxis, :], axis=0
                )
                rec_dict["state_in_nominal"] = np.append(
                    rec_dict["state_in_nominal"], state_curr_nominal[np.newaxis, :], axis=0
                )
                rec_dict["control_neural"] = np.append(
                    rec_dict["control_neural"], u_cmd_neural[np.newaxis, :], axis=0
                )
                rec_dict["control_nominal"] = np.append(
                    rec_dict["control_nominal"], u_cmd_nominal[np.newaxis, :], axis=0
                )
            # --- Prepare sim solver ---
            if sim_neural_mpc.use_mlp and model_options["linearize_mlp"]:
                set_linearization_params_sim(sim_neural_mpc, state_curr_sim_neural, u_cmd_neural, model_options["linearize_order"])

            elif sim_neural_mpc.use_mlp and model_options["use_l4casadi"]:
                set_l4casadi_params_sim(sim_neural_mpc, sim_solver_neural, u_cmd_neural)

            # --- Prepare delayed neural network input ---
            if sim_neural_mpc.use_mlp and "delay" in sim_neural_mpc.mlp_metadata["NetworkConfig"]["model_name"]:
                raise NotImplementedError("Implement.")
                set_delayed_states_as_params(sim_neural_mpc, sim_solver_neural, history, u_cmd_neural)

            # --- Set parameters in OCP solver ---
            sim_solver_neural.set("p", sim_neural_mpc.acados_parameters[0, :])

            # --- Simulate forward ---
            # Simulate with the optimized input until the next time step of the control period is reached
            # Note: Pretend to run simulation in parallel to the control loop
            # i.e., the control loop has no effect on the simulation loop execution times
            # Simply trigger new control optimization after simulating for T_samp seconds
            simulation_time = 0.0
            j = 0
            state_curr_sim_neural = state_curr_neural.copy()
            state_curr_sim_nominal = state_curr_nominal.copy()
            while simulation_time < T_samp:
                # Simulation runtime (inner loop)
                simulation_time += T_sim
                # --- Increment virtual time ---
                # ASSUMPTION: Simulation time is exactly equal to real time
                # i.e., the simulation has a zero runtime
                # This is somewhat realistic since in the real machine
                # the simulation (i.e. measurement + estimation) is run
                # in parallel to the real-time control loop.
                # Increment global time at every simulation step since the
                # control loop runs in parallel and is assumpted to be idle at some times
                t_now += T_sim

                # Simulate
                state_curr_sim_neural = sim_solver_neural.simulate(
                    x=state_curr_sim_neural, u=u_cmd_neural, p=sim_solver_neural.acados_sim.parameter_values
                )
                state_curr_sim_nominal = sim_solver_neural.simulate(
                    x=state_curr_sim_nominal, u=u_cmd_nominal, p=sim_solver_neural.acados_sim.parameter_values
                )

                # Ensure unit quaternion
                state_curr_sim_neural[6:10] = unit_quaternion(state_curr_sim_neural[6:10])
                state_curr_sim_nominal[6:10] = unit_quaternion(state_curr_sim_nominal[6:10])

                # Target check
                if tracking_mode == "position" and euclidean_dist(current_target[:3], state_curr_sim_neural[:3], thresh=0.01):
                    # Target reached!
                    current_target_reached = True
                    targets_reached[current_target_idx] = True

                    # NOTE: Break condition turned off to allow for complete simulation
                    # and therefore smooth trajectory
                    # Also, it makes no physical sense to jump to next control step immediately
                    # break
                elif tracking_mode == "orientation" and quaternion_dist(
                    state_ref[0, 6:10], state_curr_sim_neural[6:10], thresh=0.01
                ):
                    # Rotation target reached!
                    current_target_reached = True
                    targets_reached[current_target_idx] = True
                # --- Increment simulation step ---
                j += 1
            # --- Increment control step ---
            if current_target_reached:
                i = 0
                j = 0
                simulation_time = 0.0

                # Generate new target
                if run_options["preset_targets"] is None:
                    new_target = sample_random_position_target(
                        state_curr_sim_neural[:3],
                        sim_options["world_radius"],
                        aggressive=run_options["aggressive"],
                        low_flight=run_options["low_flight_targets"],
                    )

                    # if t_now < 5:
                    #     new_target = sample_random_position_target(
                    #         state_curr_sim[:3],
                    #         sim_options["world_radius"],
                    #         aggressive=run_options["aggressive"],
                    #         low_flight=run_options["low_flight_targets"],
                    #     )
                    # elif t_now < 10 and not euclidean_dist(np.array([0, 0, 1.0]), state_curr_sim[:3], thresh=0.025):
                    #     new_target = np.array([0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0])[np.newaxis, :]
                    # else:
                    #     tracking_mode = "orientation"
                    #     new_target = sample_random_orientation_target(
                    #         aggressive=True,  # run_options["aggressive"],
                    #         low_flight=False,  # run_options["low_flight_targets"],
                    #     )

                    targets = np.append(targets, new_target, axis=0)
                    targets_reached = np.append(targets_reached, False)
            else:
                i += 1

            # --- Record out data ---
            if plot:
                # State after simulation
                rec_dict["state_out_neural"] = np.append(
                    rec_dict["state_out_neural"], state_curr_sim_neural[np.newaxis, :], axis=0
                )
                rec_dict["state_out_nominal"] = np.append(
                    rec_dict["state_out_nominal"], state_curr_sim_nominal[np.newaxis, :], axis=0
                )

            # --- Break condition for the inner loop ---
            if t_now >= sim_options["max_sim_time"]:
                break

        # Current target was reached!

        # --- Break condition for the outer loop ---
        if t_now >= sim_options["max_sim_time"]:
            break

        print(f"Computation time for target {current_target_idx}: {time.time() - global_comp_time}")

    # End of simulation

    # --- Plot simple trajectory ---
    if plot:
        plot_comparison(rec_dict, neural_mpc)
        plt.show()

    halt = 1


if __name__ == "__main__":
    main(
        EnvConfig.model_options,
        EnvConfig.solver_options,
        EnvConfig.dataset_options,
        EnvConfig.sim_options,
        EnvConfig.run_options,
    )
