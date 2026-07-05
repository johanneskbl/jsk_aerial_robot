import os
import time
import numpy as np
import matplotlib.pyplot as plt

from sim_environment.sim_solver import create_acados_sim_solver, simulate_trajectory
from sim_environment.forward_prop import init_forward_prop, forward_prop
from sim_environment.disturbances import apply_cog_disturbance, apply_motor_noise, apply_extra_mass
from utils.controller_utils import check_state_constraints, check_input_constraints, get_rotor_positions
from utils.data_utils import get_recording_dict_and_file
from utils.model_utils import set_linearization_params, set_linearization_params_sim, set_l4casadi_params, set_l4casadi_params_sim, \
    set_delayed_states_as_params, set_delayed_states_as_params_sim, set_temporal_states_as_params, set_temporal_states_as_params_sim, \
    set_mlp_params
from utils.geometry_utils import unit_quaternion, euclidean_dist
from utils.visualization_utils import initialize_plotter, draw_robot, animate_robot, plot_trajectory, plot_trajectory_comparison, plot_disturbances
from config.configurations import EnvConfig
from neural_controller import NeuralMPC
from online_neural_controller import OnlineNeuralMPC
from online_data import OnlineDataset
from online_trainer import OnlineTrainer


def run_simulation(model_options, solver_options, dataset_options, sim_options, run_options,
                   useMLP=True, onlineMLP=False):
    """
    Run one complete MPC simulation and return the recorded data.

    Parameters
    ----------
    useMLP    : bool  — load and use the neural residual model.
    onlineMLP : bool  — train the NN online during the run (requires useMLP=True).

    Returns
    -------
    online_dataset : OnlineDataset  (._rec holds the full recording; .n_samples the buffer state)
    dist_dict      : dict           (populated when run_options["plot_trajectory"]=True, else {})
    neural_mpc     : OnlineNeuralMPC (useful for plotting: neural_model, state_feats, …)
    """
    np.random.seed(sim_options["seed"])
    # Dedicated RNG for trajectory selection, seeded independently so that
    # online training (which calls np.random for batch sampling) does not
    # shift the trajectory sequence away from the static-MLP run.
    traj_rng = np.random.RandomState(sim_options["seed"])

    # Apply flags to model_options before the MPC is constructed
    if not useMLP:
        model_options["only_use_nominal"]  = True
        model_options["online_neural_mpc"] = False
    elif onlineMLP:
        model_options["only_use_nominal"]  = False
        model_options["online_neural_mpc"] = True
    else:
        model_options["only_use_nominal"]  = False
        model_options["online_neural_mpc"] = False

    T_sim = sim_options["T_sim"]
    T_prop_step = T_sim
    # ------------------------

    # --- Initialize controller ---
    neural_mpc = OnlineNeuralMPC(
        model_options=model_options,
        solver_options=solver_options,
        sim_options=sim_options,
        run_options=run_options
    )

    ocp_solver = neural_mpc.get_ocp_solver()
    ocp_model = neural_mpc.get_acados_model()
    reference_generator = neural_mpc.get_reference_generator()

    # Recover some necessary variables from the MPC object
    nx = ocp_model.x.shape[0]
    nu = ocp_model.u.shape[0]
    N = neural_mpc.N
    T_horizon = neural_mpc.T_horizon
    T_samp = neural_mpc.T_samp  # Time step for the control loop
    T_step = neural_mpc.T_step  # Time step in MPC (= T_horizon / N)
    # reference_over_sampling = 1     # TODO what is this?
    # control_period = T_horizon / (N * reference_over_sampling)    # The time period between two control inputs

    # Sanity check: The optimization should be faster or equal than the duration of the optimization time step
    assert T_samp <= T_horizon / N
    assert T_samp >= T_sim

    # --- Initialize simulation environment ---
    if sim_options["use_real_world_simulator"]:
        # Use neural model trained on real world data as simulator
        sim_model_options = model_options.copy()
        sim_model_options["only_use_nominal"] = False
        sim_model_options["neural_model_instance"] = sim_options["sim_neural_model_instance"]
        sim_neural_mpc = NeuralMPC(
            model_options=sim_model_options,
            solver_options=solver_options,
            sim_options=sim_options,
            run_options=run_options,
            use_as_simulator=True,
        )
        sim_model = sim_neural_mpc.get_acados_model()
        sim_solver = create_acados_sim_solver(sim_neural_mpc, sim_model, T_sim)
    elif sim_options["use_nominal_simulator"]:
        # Use nominal model as simulator
        sim_model_options = model_options.copy()
        sim_model_options["only_use_nominal"] = True
        sim_neural_mpc = NeuralMPC(
            model_options=sim_model_options,
            solver_options=solver_options,
            sim_options=sim_options,
            run_options=run_options,
            use_as_simulator=True,
        )
        sim_model = sim_neural_mpc.get_acados_model()
        sim_solver = create_acados_sim_solver(sim_neural_mpc, sim_model, T_sim)
    else:
        # Create sim solver with same model as controller
        sim_neural_mpc = neural_mpc
        sim_model_options = model_options
        sim_solver = create_acados_sim_solver(sim_neural_mpc, ocp_model, T_sim)

    # Undisturbed model for creating labels to train on
    discretized_dynamics = init_forward_prop(neural_mpc, T_prop_step=T_prop_step, num_stages=4)

    # --- Set initial state ---
    if run_options["initial_state"] is None:
        # state = [p, v, q, w, (a and/or t and/or ds)]
        state_curr = np.zeros(nx)
        state_curr[6] = 1.0  # Real part of quaternion
    else:
        state_curr = run_options["initial_state"]
    state_curr_sim = state_curr.copy()

    # --- Warm up solver ---
    x_l = np.zeros((0, nx))
    u_l = np.zeros((0, nu))
    for i in range(N+1):
        x_l = np.append(x_l, ocp_solver.get(i, "x")[np.newaxis,:], axis=0)
        u_l = np.append(u_l, ocp_solver.get(i, "u")[np.newaxis,:] if i < N else ocp_solver.get(N-1, "u")[np.newaxis,:], axis=0)
    if model_options["use_l4casadi"]:
        for i in range(20):
            neural_mpc.learned_dyn_model.get_params(np.concatenate([x_l[:, neural_mpc.state_feats], u_l[:, neural_mpc.u_feats]], axis=1))
    for _ in range(20):
        u_temp = ocp_solver.solve_for_x0(state_curr)
        sim_solver.simulate(x=state_curr_sim, u=u_temp, p=sim_solver.acados_sim.parameter_values)
    x_l = []

    # --- Initial guess ---
    # TODO Provide a new initial guess when changing target
    u_init = np.zeros((nu,))
    u_init[:4] = 8.0  # Thrust in N for hovering
    for i in range(N):
        ocp_solver.set(i, "x", state_curr)
        ocp_solver.set(i, "u", u_init)
    ocp_solver.set(N, "x", state_curr)
    sim_solver.set("x", state_curr)
    sim_solver.set("u", u_init)

    # --- Reference trajectories ---
    traj_list = run_options["trajectories"]

    # --- Set up running history for delayed neural networks ---
    if neural_mpc.use_mlp and "delay" in neural_mpc.mlp_metadata["NetworkConfig"]["model_name"]:
        delay = neural_mpc.mlp_metadata["NetworkConfig"]["delay_horizon"]  # Delay as number of time steps into the past
        history = np.tile(np.append(state_curr, np.zeros((nu,))), (delay, 1))

    # --- Prepare recording ---
    recording = run_options["recording"]
    if recording:
        # Create an empty dict or get a pre-recorded dict and filepath to store
        # TODO actually able to use a pre-recorded dict? And if so make it overwriteable
        model_options["state_dim"] = nx
        model_options["control_dim"] = nu
        model_options["include_quaternion_constraint"] = neural_mpc.include_quaternion_constraint
        model_options["include_soft_constraints"] = neural_mpc.include_soft_constraints
        model_options["mpc_params"] = neural_mpc.params
        if neural_mpc.use_mlp:
            model_options["delay_horizon"] = neural_mpc.mlp_metadata["NetworkConfig"]["delay_horizon"]
        else:
            model_options["delay_horizon"] = 0
        ds_name = model_options["mpc_type"] + "_" + dataset_options["ds_name_suffix"]
        _, rec_file = get_recording_dict_and_file(ds_name, model_options, sim_options, solver_options, None)

        if run_options["real_time_plot"]:
            run_options["real_time_plot"] = False
            print("Turned off real time plot during recording mode.")

    # --- Real time plot ---
    # Generate necessary art pack for real time plot
    if run_options["real_time_plot"]:
        # TODO fix reference trajectory plotting
        art_pack = initialize_plotter(world_rad=sim_options["world_radius"], n_properties=N)
        trajectory_history = state_curr[np.newaxis, :]
        rotor_positions = get_rotor_positions(neural_mpc)

    plot = run_options["plot_trajectory"]
    dist_dict = {}
    if plot:
        dist_dict = {"timestamp": np.zeros((0,))}
        if sim_options["disturbances"]["cog_dist"]:
            dist_dict["z"] = np.zeros((0, 1))
            dist_dict["cog_dist"] = np.zeros((0, 6))
        if sim_options["disturbances"]["motor_noise"]:
            dist_dict["motor_noise"] = np.zeros((0, 8))
        # "drag": np.zeros((0, 0)),
        # "payload": np.zeros((0, 0)),


    # --- Set up simulation ---
    u_cmd = None
    i = 0
    j = 0
    t_now = 0.0  # Total virtual time in seconds

    # ---------- Online dataset and trainer (created once, persist across all trajectories) ----------
    online_dataset = OnlineDataset(
        nx=nx,
        nu=nu,
        T_step=T_step,
        T_samp=T_samp,
        buffer_size=dataset_options["buffer_size"],
        min_samples=dataset_options["min_samples"],
        batch_size=dataset_options["batch_size"],
        state_indices=list(range(nx)),
        window_size=dataset_options["window_size"],
        weighted_decay=dataset_options["weighted_decay"],
    )

    # --- Online trainer (only with neural model + online training flag) ---
    if useMLP and onlineMLP and neural_mpc.use_mlp:
        online_trainer = OnlineTrainer(
            model=neural_mpc.neural_model,
            nx=nx,
            nu=nu,
            state_feats=neural_mpc.state_feats,
            u_feats=neural_mpc.u_feats,
            y_reg_dims=neural_mpc.y_reg_dims,
            state_indices=list(range(nx)),
            device=neural_mpc.device,
            lr=dataset_options["lr"],
            window_size=dataset_options["window_size"],
            grad_clip_norm=dataset_options["grad_clip_norm"],
            n_frozen_layers=dataset_options["n_frozen_layers"],
            warmup_steps=dataset_options["warmup_steps"],
            train_every=dataset_options["train_every"],
            lambda_anchor=dataset_options["lambda_anchor"],
        )
        print(f"[OnlineTrainer] online training enabled "
              f"(window_size={dataset_options['window_size']}, input_dim={online_trainer.input_dim}, "
              f"lambda_anchor={dataset_options['lambda_anchor']})")
    else:
        online_trainer = None
        if useMLP:
            print("[OnlineTrainer] NN loaded in inference-only mode (onlineMLP=False)")
        else:
            print("[OnlineTrainer] nominal controller, no NN")

    # ---------- Trajectory loop ----------
    T_takeoff = sim_options["T_takeoff"]
    t_last = T_takeoff

    global_comp_time = time.time()
    print_takeoff = True
    # True once t_now first crosses T_takeoff: guards the one-shot buffer purge.
    _takeoff_done = False
    while t_now < sim_options["max_sim_time"]:
        # Get next reference
        traj = traj_rng.choice(traj_list)  # , p=[0.1, 0.3, 0.2, 0.1, 0.3])
        print("-----------------------------------")
        print(f"Tracking trajectory: {traj}")

        # Training buffer is NOT reset between trajectories so that learning
        # is continuous across the entire tracking phase.
        # (The buffer is purged once at the takeoff→tracking transition below.)

        reached_init = False
        finished = False
        print_init = True
        print_track = True
        k = 0
        # Defensive initialization: overwritten on the first valid pose_ref.
        # Guards against an UnboundLocalError if pose_ref is None at n=0.
        state_ref_const = np.zeros((1, nx))
        state_ref_const[0, 6] = 1.0  # unit quaternion real part
        control_ref_const = np.zeros((1, nu))

        while not finished:
            global_comp_time = time.time()

            # --- Get current state ---
            if u_cmd is None:
                # If no command is available, use initial/last state
                sim_solver.set("x", state_curr)  # doesn't work
                u_cmd = np.zeros((nu,))
            else:
                state_curr = state_curr_sim.copy()
                check_state_constraints(ocp_solver, state_curr, i)

            # --- Reference ---
            state_ref = np.zeros((N + 1, nx))
            state_ref[:, 6] = 1.0
            control_ref = np.zeros((N, nu))
            # TODO redo reference setting and making use of setting the reference dynamically for each mpc node and not using a constant!
            for n in range(N + 1):
                # Pose reference
                # First takeoff, then selected trajectory
                if t_now < T_takeoff:
                    if print_takeoff:
                        print("Taking off...")
                        print_takeoff = False
                    pose_ref = reference_generator.get_pose_from_trajectory("smooth_takeoff", t_now, T_takeoff)
                    last_ref = pose_ref
                else:
                    if not reached_init:
                        if print_init:
                            print("Going to init pose...")
                            print_init = False
                        # Get init pose to fly from current position
                        pose_init = reference_generator.get_pose_from_trajectory(
                            traj, 0, run_options["trajectory_length"]
                        )
                        # Smooth transition to init pose of next trajectory
                        pose_ref = np.array(last_ref) + (np.array(pose_init) - np.array(last_ref)) / (
                            1 + 100 * np.exp(-(t_now - t_last) * 4)
                        )

                        if euclidean_dist(state_curr[0:3], pose_init[0:3]) < 0.1 or k > 5000:
                            reached_init = True
                            t_init = t_now
                            k = 0
                        else:
                            k += 1
                    else:
                        if print_track:
                            print("Init pose reached. Now tracking trajectory...")
                            print_track = False
                        pose_ref = reference_generator.get_pose_from_trajectory(
                            traj, t_now - t_init + n * T_step, run_options["trajectory_length"]
                        )
                        if n == 0 and pose_ref is not None:
                            # Store last reference to smoothly go to init pose of next trajectory
                            last_ref = pose_ref

                if pose_ref is None:
                    finished = True
                    reached_init = False
                    t_last = t_now
                    print(f"Tracking finished for {traj}!")
                    state_ref[n, :] = state_ref_const[0, :]
                    if n < N:
                        control_ref[n, :] = control_ref_const[0, :]
                    continue

                # Compute reference for Input u with an allocation matrix - TODO still makes sense if we don't know model in the first place?
                # Alternative is setting the modular trajectory yref dynamically in control loop
                state_ref_const, control_ref_const = reference_generator.compute_trajectory(
                    target_xyz=pose_ref[0:3], target_rpy=pose_ref[3:6]
                )

                # Append reference
                state_ref[n, :] = state_ref_const[0, :]
                if n < N:
                    control_ref[n, :] = control_ref_const[0, :]
            # Track reference in solver over horizon
            neural_mpc.track(ocp_solver, state_ref, control_ref, u_cmd)

            # --- Prepare neural model linearization ---
            # Optimization cycle
            # neural_mpc without model:    0.31 ms
            # neural_mpc with their model: 0.53 ms (without linearization) -> + 71%
            # Ours without model:      1.05 ms
            # Ours with our model:     3.69 ms (without linearization) -> + 250% [min]
            # Ours with their model:   1.56 ms (without linearization) -> + 48%  [min]
            # Ours with our model:     27.1 ms (without linearization) [4x64+full in]
            # Ours with their model:   17.1 ms (without linearization) [4x64+full in&out]

            comp_time = time.time()
            if neural_mpc.use_mlp and model_options["linearize_mlp"]:
                set_linearization_params(neural_mpc, ocp_solver, model_options["linearize_order"])

            elif neural_mpc.use_mlp and model_options["use_l4casadi"]:
                set_l4casadi_params(neural_mpc, ocp_solver)

            # --- Prepare delayed neural network input ---
            if neural_mpc.use_mlp and "delay" in neural_mpc.mlp_metadata["NetworkConfig"]["model_name"]:
                set_delayed_states_as_params(neural_mpc, ocp_solver, history, u_cmd)

            # --- Set parameters in OCP solver ---
            for j in range(ocp_solver.N + 1):
                ocp_solver.set(j, "p", neural_mpc.acados_parameters[j, :])

            ############################################################################################
            # --- Optimize control input ---
            # Compute control feedback and take the first action
            # acados wrapper to solve the OCP and get first control command from sequence
            u_cmd = ocp_solver.solve_for_x0(state_curr)
            mpc_first_pred = ocp_solver.get(1, "x")  # predicted state at t + T_step
            comp_time = (time.time() - comp_time) * 1000  # in ms

            # --- Sanity check constraints ---
            check_input_constraints(neural_mpc, u_cmd, i)
            ############################################################################################

            # --- Running history for delayed neural networks ---
            if neural_mpc.use_mlp and "delay" in neural_mpc.mlp_metadata["NetworkConfig"]["model_name"]:
                # Append current state and control to history for next iteration
                # Sorted from newest to oldest
                history = history[:-1, :]
                history = np.append(np.append(state_curr, u_cmd)[np.newaxis, :], history, axis=0)

            # --- Plot realtime ---
            if run_options["real_time_plot"]:
                raise NotImplementedError("Rethink.")
                # Note: Simulation is without disturbance here !
                #########################################
                # TODO OVERTHINK THIS!!!
                state_traj = simulate_trajectory(ocp_solver, sim_solver, state_curr)
                #########################################
                draw_robot(
                    art_pack,
                    None,  # TODO also display reference trajectory
                    None,
                    state_curr,
                    state_traj,
                    trajectory_history,
                    rotor_positions,
                    follow_robot=False,
                    animation=run_options["save_animation"],
                )

            # --- Simulate forward ---
            # Save the timestamp at which the control command was issued.
            # t_now advances inside the simulation loop, so we must capture it NOW
            # before t_now is incremented.
            t_ctrl = t_now

            simulation_time = 0.0
            j = 0
            state_curr_sim = state_curr.copy()
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

                # --- Set disturbance forces as parameters ---
                # TODO only apply disturbance to simulation model for next time step
                if sim_options["disturbances"]["cog_dist"]:
                    apply_cog_disturbance(
                        sim_solver, neural_mpc, sim_options["disturbances"]["cog_dist_factor"], u_cmd, state_curr
                    )
                    if plot:
                        dist_dict["timestamp"] = np.append(dist_dict["timestamp"], t_now)
                        dist_dict["z"] = np.append(dist_dict["z"], state_curr_sim[2])
                        dist_dict["cog_dist"] = np.append(
                            dist_dict["cog_dist"],
                            sim_solver.acados_sim.parameter_values[
                                np.newaxis, neural_mpc.cog_dist_start_idx : neural_mpc.cog_dist_end_idx
                            ],
                            axis=0,
                        )
                if sim_options["disturbances"]["motor_noise"]:
                    apply_motor_noise(sim_solver, neural_mpc, u_cmd)
                    if plot:
                        if not sim_options["disturbances"]["cog_dist"]:
                            dist_dict["timestamp"] = np.append(dist_dict["timestamp"], t_now)
                        dist_dict["motor_noise"] = np.append(
                            dist_dict["motor_noise"],
                            sim_solver.acados_sim.parameter_values[
                                np.newaxis, neural_mpc.motor_noise_start_idx : neural_mpc.motor_noise_end_idx
                            ],
                            axis=0,
                        )
                # --- Prepare sim solver ---
                if sim_neural_mpc.use_mlp and sim_model_options["linearize_mlp"]:
                    set_linearization_params_sim(sim_neural_mpc, state_curr_sim, u_cmd, sim_model_options["linearize_order"])

                elif sim_neural_mpc.use_mlp and sim_model_options["use_l4casadi"]:
                    set_l4casadi_params_sim(sim_neural_mpc, sim_solver, u_cmd)

                # --- Prepare delayed neural network input ---
                if sim_neural_mpc.use_mlp and "delay" in sim_neural_mpc.mlp_metadata["NetworkConfig"]["model_name"]:
                    raise NotImplementedError("Implement.")
                    set_delayed_states_as_params(sim_neural_mpc, sim_solver, history, u_cmd)

                # --- Set base parameters in sim solver ---
                sim_solver.set("p", sim_neural_mpc.acados_parameters[0, :])

                # --- Overlay disturbances AFTER set("p", ...) ---
                # apply_extra_mass re-calls set("p", ...) with the disturbance force
                # added, so the MPC controller's parameter vector stays unmodified.
                if sim_options["disturbances"]["extra_mass"]:
                    apply_extra_mass(
                        sim_solver, neural_mpc,
                        sim_options["disturbances"]["extra_mass_kg"],
                    )

                # Simulate
                state_curr_sim = sim_solver.simulate(
                    x=state_curr_sim, u=u_cmd
                )

                # Ensure unit quaternion
                state_curr_sim[6:10] = unit_quaternion(state_curr_sim[6:10])

                # Ensure height constraint
                if state_curr_sim[2] < 0:
                    state_curr_sim[2] = 0
                    state_curr_sim[5] = 0

                # --- Increment simulation step ---
                j += 1
            # --- Increment control step ---
            i += 1

            # --- Sanity check constraints ---
            check_state_constraints(ocp_solver, state_curr_sim, i)

            # --- Compute next state prediction (nominal forward integration) ---
            # T_samp-ahead: stored in _rec["state_pred"] for reference / visualization
            state_prop = forward_prop(
                discretized_dynamics,
                state_curr[np.newaxis, :],
                u_cmd[np.newaxis, :],
                T_prop_horizon=T_samp,
                T_prop_step=T_prop_step,
            )[-1, :]

            # T_step-ahead nominal prediction (no NN correction): used as the
            # label baseline during online training so the residual
            #   label = actual(T+T_step) - nominal(T+T_step)
            # matches the offline training convention, avoiding the d/2 bias
            # that arises when mpc_first_pred (which includes the NN term) is
            # used instead.
            nominal_T_step = forward_prop(
                discretized_dynamics,
                state_curr[np.newaxis, :],
                u_cmd[np.newaxis, :],
                T_prop_horizon=T_step,
                T_prop_step=T_prop_step,
            )[-1, :]

            # --- Online data collection (unconditional: needed for training) ---
            online_dataset.get_data({
                "t_now":              t_ctrl,
                "comp_time":          comp_time,
                "state_ref":          state_ref[0, :],
                "state_curr":         state_curr,
                "u_cmd":              u_cmd,
                "mpc_first_pred":     mpc_first_pred,
                "nominal_first_pred": nominal_T_step,
                "state_out":          state_curr_sim,
                "state_pred":         state_prop,
            })

            # --- Online training step (tracking phase only) ---
            if t_now >= T_takeoff:
                if not _takeoff_done:
                    # One-shot purge of takeoff data so the model only trains
                    # on tracking dynamics.  _rec is NOT cleared (full recording
                    # is preserved for plots).
                    online_dataset.reset()
                    _takeoff_done = True
                    if online_trainer is not None:
                        print("[OnlineTrainer] takeoff ended — buffer purged, "
                              "waiting for tracking samples to mature.")
                if (online_dataset.ready_for_training()
                        and online_trainer is not None
                        and online_trainer.should_train()):
                    data = online_trainer.get_data(online_dataset, strategy="weighted")
                    loss = online_trainer.learn(data)
                    # Sync updated PyTorch weights → acados_parameters.
                    # The for-j loop at the top of the next MPC iteration
                    # forwards them to the running solver automatically.
                    set_mlp_params(neural_mpc)
                    if online_trainer.step_count % 100 == 0:
                        print(f"[OnlineTrainer] step {online_trainer.step_count} | loss {loss:.6f} "
                              f"| buffer {online_dataset.n_samples}/{online_dataset.buffer_size}")
        

            # --- Log trajectory for real-time plot ---
            if run_options["real_time_plot"]:
                trajectory_history = np.append(trajectory_history, state_curr_sim[np.newaxis, :], axis=0)
                if len(trajectory_history) > 300:
                    trajectory_history = np.delete(trajectory_history, obj=0, axis=0)

            # Current target was reached!
            # --- Save data ---
            if recording and (i % 100 == 0 or t_now >= sim_options["max_sim_time"] or finished):
                online_dataset.save(rec_file)

            # --- Break condition for the outer loop ---
            if t_now >= sim_options["max_sim_time"]:
                break

    # End of simulation
    print(f"Trajectory tracking finished in {(time.time() - global_comp_time):.2f} seconds!")
    if recording:
        print(f"Recording finished. Data saved to {rec_file}")

    # --- Save recording data ---
    if recording:
        online_dataset.save(rec_file)
        print(f"Recording saved to {rec_file}.npz")
    # --- Create video ---
    if run_options["save_animation"]:
        print("-------------- Saving animation as video --------------")
        dir_path = os.path.dirname(os.path.abspath(__file__))
        counter = 1
        while True:
            file_name = f"video/robot_animation_{str(counter).zfill(3)}.mp4"
            file_path = os.path.join(dir_path, file_name)
            if not os.path.exists(file_path):
                break
            counter += 1
        animate_robot(file_path)
        print(f"Saved in directory: {file_path}")

    return online_dataset, dist_dict, neural_mpc


def main(model_options, solver_options, dataset_options, sim_options, run_options):
    """
    Main function to run the MPC simulation loop and label recording.
    :param model_options: Options for the MPC model.
    :param dataset_options: Options for the recording.
    :param sim_options: Options for the simulation.
    :param run_options: Additional parameters for the simulation.
    """
    # ----------------------------------------------------------------
    # Neural model control flags
    #
    # useMLP   : use the neural residual model inside the controller.
    #            False  → nominal-only controller (no NN, fastest).
    #            True   → NN loaded and used for inference.
    # onlineMLP: train the NN online during simulation.
    #            Requires useMLP=True.
    #            False  → frozen weights (inference only).
    #            True   → gradient updates at every ready step.
    #
    # useMLP | onlineMLP | behaviour
    # -------+-----------+--------------------------------------------------
    # False  | False     | nominal controller, no NN
    # True   | False     | NN at fixed weights (pre-trained inference only)
    # True   | True      | NN trained online during flight
    # ----------------------------------------------------------------
    useMLP    = run_options["useMLP"]
    onlineMLP = run_options["onlineMLP"]

    dataset, dist_dict, neural_mpc = run_simulation(
        model_options, solver_options, dataset_options, sim_options, run_options,
        useMLP=useMLP, onlineMLP=onlineMLP,
    )

    plot      = run_options["plot_trajectory"]
    recording = run_options["recording"]
    if plot and not recording:
        plot_trajectory(
            model_options, sim_options, dataset._rec, neural_mpc,
            dist_dict=dist_dict, save=run_options["save_figures"]
        )
        if sim_options["disturbances"]["cog_dist"] or sim_options["disturbances"]["motor_noise"]:
            plot_disturbances(dist_dict, save=run_options["save_figures"])
        plt.show()
        print("Done.")
        print(f"online_dataset recorded steps  : {dataset.n_recorded}  (full _rec, never overwritten)")
        print(f"online_dataset training samples: {dataset.n_samples}  (circular buffer, capped at {dataset.buffer_size})")


if __name__ == "__main__":
    main(
        EnvConfig.model_options,
        EnvConfig.solver_options,
        EnvConfig.dataset_options,
        EnvConfig.sim_options,
        EnvConfig.run_options,
    )

