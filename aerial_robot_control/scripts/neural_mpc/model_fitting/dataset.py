import numpy as np
from torch.utils.data import Dataset
from config.configurations import ModelFitConfig, NetworkConfig
from utils.data_utils import undo_jsonify
from utils.geometry_utils import v_dot_q, quaternion_inverse
from utils.statistics_utils import prune_dataset
from utils.filter_utils import moving_average_filter, low_pass_filter
from utils.visualization_utils import plot_dataset

class TrajectoryDataset(Dataset):
    """
    Dataset for training neural networks on trajectory data.
    """

    def __init__(
        self,
        dataframe,
        state_feats,
        u_feats,
        y_reg_dims,
        neural_mpc,
        save_file_path=None,
        save_file_name=None,
        mode=None,
    ):
        self.df = dataframe
        self.state_feats = state_feats
        self.u_feats = u_feats
        self.y_reg_dims = y_reg_dims
        self.T_samp = neural_mpc.T_samp
        self.T_step = neural_mpc.T_step
        self.N = neural_mpc.N

        self.prepare_data()
        if ModelFitConfig.prune and (NetworkConfig.delay_horizon == 0 and not NetworkConfig.temporalize):
            # Don't prune when using delayed or temporal networks with history since pruning causes incontinuities
            self.prune()
        if NetworkConfig.delay_horizon > 0:
            self.append_history()
        self.calculate_statistics()
        if ModelFitConfig.plot_dataset:
            if not ModelFitConfig.save_plots:
                save_file_path = None
                save_file_name = None
            plot_dataset(
                self.x,
                self.y,
                self.dt,
                self.timestamp,
                self.state_curr,
                self.state_out,
                self.state_pred,
                self.state_ref,
                self.control,
                self.control_pred if NetworkConfig.temporalize else None,
                self.T_step,
                self.state_in_filtered if ModelFitConfig.use_moving_average_filter else None,
                self.control_filtered if ModelFitConfig.use_moving_average_filter else None,
                self.x_raw if ModelFitConfig.use_moving_average_filter else None,
                self.x.copy() if ModelFitConfig.use_moving_average_filter else None,
                self.y_raw if ModelFitConfig.use_moving_average_filter else None,
                self.y_filtered if ModelFitConfig.use_moving_average_filter else None,
                save_file_path,
                save_file_name,
                mode,
            )

    def __len__(self):
        return len(self.x)

    def __getitem__(self, idx):
        return self.x[idx], self.y[idx]

    def prepare_data(self):
        state = undo_jsonify(self.df["state"].to_numpy())
        control = undo_jsonify(self.df["control"].to_numpy())
        if NetworkConfig.temporalize:
            # Parse temporalized predictions and controls
            state_pred = np.zeros((state.shape[0], self.N, state.shape[1]))
            control_pred = np.zeros((state.shape[0], self.N - 1, control.shape[1]))
            for i in range(1, self.N + 1):
                state_pred[:, i-1, :] = undo_jsonify(self.df[f"state_pred_{i}"].to_numpy())
                if i < self.N:
                    control_pred[:, i-1, :] = undo_jsonify(self.df[f"control_pred_{i}"].to_numpy())
        else:
            state_pred = undo_jsonify(self.df["state_pred_1"].to_numpy())
        state_ref = undo_jsonify(self.df["state_ref"].to_numpy())
        dt = self.df["dt"].to_numpy()
        timestamp = self.df["timestamp"].to_numpy()
        recording_start_idx = undo_jsonify(self.df["recording_start_idx"].to_numpy(), to_float=False)[0, ...]

        # Prediction horizon of predicted state w.r.t. sampling rate
        if ModelFitConfig.prop_long_horizon:
            P = round(self.T_step / self.T_samp)  # T_step / T_samp = 0.1s / 0.01s = 10 steps
            P_step = self.T_step
        else:
            P = 1
            P_step = self.T_samp

        # Offset by prediction horizon P
        state_out = state[P:, :]
        if recording_start_idx.shape[0] > 1:
            # Multiple recordings have been concatenated, therefore we artificially continue state_out to avoid outliers in the label
            for i in range(1, recording_start_idx.shape[0]):
                start_idx = recording_start_idx[i]
                state_out[start_idx - P : start_idx, :] = np.tile(state_out[start_idx - (P + 1), :], (P, 1))
        # Adjust to size of state_out
        state_curr = state[:-P, :]
        state_pred = state_pred[:-P, ...]
        state_ref = state_ref[:-P, :]
        control = control[:-P, :]
        dt = dt[:-P]
        timestamp = timestamp[:-P]
        if NetworkConfig.temporalize:
            control_pred = control_pred[:-P, :, :]

        # Remove invalid entries (dt = 0)
        invalid = np.where(dt == 0)
        state_curr = np.delete(state_curr, invalid, axis=0)
        state_out = np.delete(state_out, invalid, axis=0)
        state_pred = np.delete(state_pred, invalid, axis=0)
        state_ref = np.delete(state_ref, invalid, axis=0)
        control = np.delete(control, invalid, axis=0)
        dt = np.delete(dt, invalid, axis=0)
        timestamp = np.delete(timestamp, invalid, axis=0)
        if NetworkConfig.temporalize:
            control_pred = np.delete(control_pred, invalid, axis=0)

        # Assemble a multi-step horizon label matrix
        if NetworkConfig.temporalize:
            # Create new state_out that is filled with the next N state_outs at each time step t
            idx_trunc = (self.N - 1) * P  # Indices to truncate from the end of state_out to ensure we have enough future steps to fill over the entire horizon
            state_out_over_horizon = np.zeros((state_out.shape[0] - idx_trunc, self.N, state_out.shape[1]))
            for t in range(state_out.shape[0] - idx_trunc):
                time_curr = timestamp[t]
                t_prev = t - 1  # Initialize t_prev to the previous index to check for duplicate timestamps
                for i in range(self.N):
                    time_next = time_curr + i * P_step  # Get the timestamp corresponding to the state_out at node i
                    t_next = (np.abs(timestamp - time_next)).argmin()  # Convert to corrsponding index
                    # t_next = t + i * P
                    if t_prev == t_next:
                        raise ValueError(f"WARNING: Duplicate timestamp found: t_prev == t_next -> {t_prev} == {t_next} at index {t}.")
                    state_out_over_horizon[t, i, :] = state_out[t_next, :]

                    t_prev = t_next
            state_out = state_out_over_horizon
            # Adjust to size of state_out
            state_curr = state_curr[:-idx_trunc, :]
            state_pred = state_pred[:-idx_trunc, :, :]
            state_ref = state_ref[:-idx_trunc, :]
            control = control[:-idx_trunc, :]
            control_pred = control_pred[:-idx_trunc, :, :]
            dt = dt[:-idx_trunc]
            timestamp = timestamp[:-idx_trunc]
        state_raw = state_curr.copy()

        # Sanity check
        if NetworkConfig.temporalize:
            if state_pred.shape[1] != self.N:
                raise ValueError(f"Expected {self.N} prediction steps in state_pred but got {state_pred.shape[1]}.")
            if control_pred.shape[1] != self.N - 1:
                raise ValueError(f"Expected {self.N-1} prediction steps in control_pred but got {control_pred.shape[1]}.")
            if state_curr.shape[0] != state_out.shape[0] \
            or state_curr.shape[0] != state_pred.shape[0] \
            or state_curr.shape[0] != state_ref.shape[0] \
            or state_curr.shape[0] != control.shape[0] \
            or state_curr.shape[0] != control_pred.shape[0]:
                raise ValueError("Inconsistent shapes in the dataset.")
        else:
            if (
                state_curr.shape != state_out.shape \
             or state_curr.shape != state_pred.shape \
             or state_curr.shape != state_ref.shape \
             or state_curr.shape[0] != control.shape[0]
            ):
                raise ValueError("Inconsistent shapes in the dataset.")

        # Transform velocity to body frame
        def velocity_mapping(state_sequence):
            p_traj = state_sequence[:, :3]
            v_w_traj = state_sequence[:, 3:6]
            q_traj = state_sequence[:, 6:10]
            other_traj = state_sequence[:, 10:]  # w, a_s, f_s, etc.

            v_b_traj = np.empty_like(v_w_traj)
            for t in range(len(v_w_traj)):
                v_b_traj[t, :] = v_dot_q(v_w_traj[t, :], quaternion_inverse(q_traj[t, :]))
            return np.concatenate((p_traj, v_b_traj, q_traj, other_traj), axis=1)

        if ModelFitConfig.input_transform:
            if NetworkConfig.temporalize and not ModelFitConfig.label_transform:
                raise ValueError("Here we transform curr state but for temporal networks we also use pred in input so it also needs to be transformed!")
            state_curr = velocity_mapping(state_curr)
            state_ref = velocity_mapping(state_ref)
        else:
            # Don't transform input but let network learn in world frame directly
            pass
        if ModelFitConfig.label_transform:
            if NetworkConfig.temporalize:
                if not ModelFitConfig.input_transform:
                    raise ValueError("Here we transform pred but for temporal networks we also use it in input so also curr state needs to be transformed!")
                for i in range(state_pred.shape[1]):
                    state_pred[:, i, :] = velocity_mapping(state_pred[:, i, :])
                    state_out[:, i, :] = velocity_mapping(state_out[:, i, :])
            else:
                state_pred = velocity_mapping(state_pred)
                state_out = velocity_mapping(state_out)
        else:
            # Don't transform labels but let network predict in world frame directly
            pass

        # =============================================================
        # Compute residual dynamics of actual state and predicted (or "propagated") state
        if ModelFitConfig.prop_long_horizon:
            y = (state_out - state_pred) / self.T_step
        else:
            y = (state_out - state_pred) / np.expand_dims(dt, 2 if NetworkConfig.temporalize else 1)

        # Nonlinear quaternion error computation
        # NOTE: The difference between two quaternions can be computed by q_diff = q2 quaternion-multiply inverse(q1)
        # where: inverse(q1) = conjugate(q1) / abs(q1)
        # and:   conjugate( quaternion(re, i, j, k) ) = quaternion(re, -i, -j, -k)
        # and:   abs(q1) = 1 for unit quaternions.
        # therefore: q_diff =   w1*w2 - x1*x2 - y1*y2 - z1*z2
        #                     i(w1*x2 + x1*w2 + y1*z2 - z1*y2)
        #                     j(w1*y2 - x1*z2 + y1*w2 + z1*x2)
        #                     k(w1*z2 + x1*y2 - y1*x2 + z1*w2)
        # NOTE: Here q1 is state_pred and q2 is state_out because we want to compute the error from propagated to actual
        # qw_prop = state_pred[:, 6]
        # qx_prop = state_pred[:, 7]
        # qy_prop = state_pred[:, 8]
        # qz_prop = state_pred[:, 9]
        # qw_out = state_out[:, 6]
        # qx_out = state_out[:, 7]
        # qy_out = state_out[:, 8]
        # qz_out = state_out[:, 9]

        # qe_w = qw_out * qw_prop - qx_out * qx_prop - qy_out * qy_prop - qz_out * qz_prop
        # qe_x = qw_out * qx_prop + qx_out * qw_prop + qy_out * qz_prop - qz_out * qy_prop
        # qe_y = qw_out * qy_prop - qx_out * qz_prop + qy_out * qw_prop + qz_out * qx_prop
        # qe_z = qw_out * qz_prop + qx_out * qy_prop - qy_out * qx_prop + qz_out * qw_prop
        # q_e = np.stack((qe_w, qe_x, qe_y, qe_z), axis=1)
        # y[:, 6:10] = q_e / np.expand_dims(dt, 1)
        # =============================================================

        # Data filtering
        # NOTE: Apply after computing residual dynamics to have more significant smoothing effect
        # If applied before, the effectiveness of the smoothing is drastically reduced
        if ModelFitConfig.use_moving_average_filter:
            if NetworkConfig.temporalize:
                for i in range(-1, self.N-1):
                    if i == -1:
                        self.x_raw = np.concatenate((state_curr[:, self.state_feats], control[:, self.u_feats]), axis=1, dtype=np.float32)
                    else:
                        self.x_raw = np.concatenate((self.x_raw, state_pred[:, i, self.state_feats], control_pred[:, i, self.u_feats]), axis=1, dtype=np.float32)
            else:
                self.x_raw = np.concatenate((state_curr[:, self.state_feats], control[:, self.u_feats]), axis=1, dtype=np.float32)
            self.y_raw = y[..., self.y_reg_dims].copy()
            
            print(f"[DATASET] Applying moving average filter with window size {ModelFitConfig.window_size} to network input and labels.")
            state_curr = moving_average_filter(state_curr, window_size=ModelFitConfig.window_size)
            if NetworkConfig.temporalize:
                for i in range(y.shape[1]):
                    state_pred[:, i, :] = moving_average_filter(state_pred[:, i, :], window_size=ModelFitConfig.window_size)
                    y[:, i, :] = moving_average_filter(y[:, i, :], window_size=ModelFitConfig.window_size)
            else:
                y = moving_average_filter(y, window_size=ModelFitConfig.window_size)
            if ModelFitConfig.control_filtering:
                control = moving_average_filter(control, window_size=ModelFitConfig.window_size)
                if NetworkConfig.temporalize:
                    for i in range(control_pred.shape[1]):
                        control_pred[:, i, :] = moving_average_filter(control_pred[:, i, :], window_size=ModelFitConfig.window_size)
            if ModelFitConfig.plot_dataset:
                self.state_in_filtered = state_curr.copy()
                self.control_filtered = control.copy()
                self.y_filtered = y[..., self.y_reg_dims].copy()
        elif ModelFitConfig.use_moving_average_filter_only_label:
            print(f"[DATASET] Applying moving average filter with window size {ModelFitConfig.window_size} to labels only.")
            if NetworkConfig.temporalize:
                for i in range(y.shape[1]):
                    y[:, i, :] = moving_average_filter(y[:, i, :], window_size=ModelFitConfig.window_size)
            else:
                y = moving_average_filter(y, window_size=ModelFitConfig.window_size)
        if ModelFitConfig.use_low_pass_filter:
            # Sampling frequency
            fs = 1.0 / np.mean(dt)
            # Cutoff frequencies
            # cutoff_pos = 0.8
            # cutoff_vel = 0.8
            # cutoff_quat =  0.8
            # cutoff_angular_vel = 0.8
            # cutoff_thrust = 1.0
            # cutoff_servo = 0.3
            cutoff_input = ModelFitConfig.low_pass_filter_cutoff_input
            cutoff_acc = ModelFitConfig.low_pass_filter_cutoff_label
            print(f"[DATASET] Applying low-pass filter with cutoff frequency {cutoff_input} to network input and "
                  f"labels with cutoff frequency {cutoff_acc}.")

            for dim in range(state_curr.shape[1]):
                state_curr[:, dim] = low_pass_filter(state_curr[:, dim], cutoff=cutoff_input, fs=fs)
            for dim in range(control.shape[1]):
                control[:, dim] = low_pass_filter(control[:, dim], cutoff=cutoff_input, fs=fs)
            if NetworkConfig.temporalize:
                for i in range(y.shape[1]):
                    for dim in self.y_reg_dims:
                        y[:, i, dim] = low_pass_filter(y[:, i, dim], cutoff=cutoff_acc, fs=fs)
                    for dim in range(control.shape[1]):
                        control_pred[:, i, dim] = low_pass_filter(control_pred[:, i, dim], cutoff=cutoff_input, fs=fs)
            else:
                for dim in self.y_reg_dims:
                    y[:, dim] = low_pass_filter(y[:, dim], cutoff=cutoff_acc, fs=fs)

        # Store data
        self.state_raw = state_raw
        self.state_curr = state_curr
        self.state_out = state_out if not NetworkConfig.temporalize else state_out_over_horizon
        self.state_pred = state_pred
        self.state_ref = state_ref
        self.control = control
        self.dt = dt
        self.timestamp = timestamp
        if NetworkConfig.temporalize:
            self.control_pred = control_pred

        # Assemble network input - WE USE CURRENT STATE AND LAST CONTROL COMMAND AS WELL AS THE PREDICTION HORIZON UNTIL N-1 OF LAST OPTIMIZATION
        if NetworkConfig.temporalize:
            for i in range(-1, self.N-1):
                if i == -1:
                    self.x = np.concatenate((state_curr[:, self.state_feats], control[:, self.u_feats]), axis=1, dtype=np.float32)
                else:
                    self.x = np.concatenate((self.x, state_pred[:, i, self.state_feats], control_pred[:, i, self.u_feats]), axis=1, dtype=np.float32)

        else:
            self.x = np.concatenate((state_curr[:, self.state_feats], control[:, self.u_feats]), axis=1, dtype=np.float32)
        # Store labels
        if NetworkConfig.temporalize:
            # Flatten by appending the labels for each stage after another
            # Reshape to [[y_t0_n0_d0, y_t0_n0_d1, ..., y_t0_n0_dk, y_t0_n1_d0, ..., y_t0_n1_dk, ..., y_t0_nN_d0, ..., y_t0_nN_dk],
            #             [y_t1_n0_d0, y_t1_n0_d1, ..., y_t1_n0_dk, y_t1_n1_d0, ..., y_t1_n1_dk, ..., y_t1_nN_d0, ..., y_t1_nN_dk], ...]
            # with t time steps, n nodes and k label dims
            self.y = y[:, :, self.y_reg_dims].reshape(y.shape[0], -1, order="C").astype(np.float32)
        else:
            self.y = y[:, self.y_reg_dims].astype(np.float32)

    def prune(self):
        """
        Prune the dataset to remove samples with outliers in velocity.
        """
        histogram_n_bins = ModelFitConfig.histogram_n_bins
        histogram_thresh = ModelFitConfig.histogram_thresh
        
        vel_idx = np.array([3, 4, 5])
        v_z_idx = np.array([5])
        if set(vel_idx).issubset(set(self.state_feats)) and set(vel_idx).issubset(set(self.y_reg_dims)):
            x_vel_idx_real = np.where(np.in1d(self.state_feats, vel_idx))[0]
            y_vel_idx_real = np.where(np.in1d(self.y_reg_dims, vel_idx))[0]
        elif set(vel_idx).issubset(set(self.state_feats)) and set(v_z_idx).issubset(set(self.y_reg_dims)):
            # Only vz in labels
            x_vel_idx_real = np.where(np.in1d(self.state_feats, vel_idx))[0]
            y_vel_idx_real = np.where(np.in1d(self.y_reg_dims, v_z_idx))[0]
        else:
            print("[PRUNING] Velocity features not part of input AND output, skipping pruning.")
            # Pruning only works right now if the velocity features are part of the input and output
            return

        # Prune noisy data
        if histogram_n_bins is not None and histogram_thresh is not None:
            labels = ["vx", "vy", "vz"]

            self.pruned_idx = prune_dataset(
                self.x[:, x_vel_idx_real],
                self.y[:, y_vel_idx_real],
                ModelFitConfig.vel_cap,
                histogram_n_bins,
                histogram_thresh,
                plot=ModelFitConfig.plot_dataset,
                labels=labels,
            )
            self.x = self.x[self.pruned_idx]
            self.y = self.y[self.pruned_idx]

    def append_history(self):
        """
        Append previous states and controls to the input features for temporal networks.
        Creates sliding windows of historical data with zero-padding for initial samples.

        :param delay: Number of historical time steps to include
        :param state_feats: List of state feature indices
        """
        n_samples = self.x.shape[0]
        n_state_feats = len(self.state_feats)
        n_control_feats = len(self.u_feats)
        delay = NetworkConfig.delay_horizon

        # Create new input array with history: current + delay previous steps
        state_history = np.zeros((n_samples, n_state_feats * (delay + 1)), dtype=np.float32)
        control_history = np.zeros((n_samples, n_control_feats * (delay + 1)), dtype=np.float32)

        for i in range(n_samples):
            # Current time step (most recent)
            state_history[i, :n_state_feats] = self.x[i, :n_state_feats]
            control_history[i, :n_control_feats] = self.x[i, n_state_feats:]

            # Historical time steps (delay previous steps)
            for j in range(1, delay + 1):
                state_start_idx = j * n_state_feats
                state_end_idx = (j + 1) * n_state_feats
                control_start_idx = j * n_control_feats
                control_end_idx = (j + 1) * n_control_feats

                if i - j >= 0:
                    # Use actual historical data
                    state_history[i, state_start_idx:state_end_idx] = self.x[i - j, :n_state_feats]
                    control_history[i, control_start_idx:control_end_idx] = self.x[i - j, n_state_feats:]
                else:
                    # Pad with initial value where history doesn't exist
                    state_history[i, state_start_idx:state_end_idx] = self.x[0, :n_state_feats]
                    control_history[i, control_start_idx:control_end_idx] = self.x[0, n_state_feats:]

        # Update the dataset with the new input features
        self.x = np.append(state_history, control_history, axis=1)

    def calculate_statistics(self):
        # Calculate mean and std for normalization
        self.x_mean = np.mean(self.x, axis=0)
        self.x_std = np.std(self.x, axis=0)
        self.y_mean = np.mean(self.y, axis=0)
        self.y_std = np.std(self.y, axis=0)
        # Sanity check since we divide by x_std in normalization
        # if np.any(self.x_std == 0):
        #     raise ValueError("Input features have zero standard deviation, cannot normalize.")
