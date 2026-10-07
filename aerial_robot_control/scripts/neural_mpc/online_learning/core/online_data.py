import numpy as np
from collections import deque


class OnlineDataset:
    """
    Collects flight data during simulation and generates time-aligned residual
    labels for online neural network training.

    Label derivation
    ----------------
    At each control step (time T):
      - The MPC solver provides its first predicted node: x_hat(T + T_step).
      - The actual state at T + T_step is observed T_step / T_samp steps later.
      - Once available, the residual label is computed as a *rate* (divided by
        T_step so it has acceleration units for velocity dims, matching the
        offline convention in model_fitting/dataset.py):
            Y = (actual_state(T + T_step)[state_indices]
                 - x_hat(T + T_step)[state_indices]) / T_step
      - The training input is the SLIDING WINDOW of the last window_size observations:
            X = [obs(T-(W-1)*T_samp), ..., obs(T-T_samp), obs(T)]
            with obs(t) = [state_curr(t), u_cmd(t)]  (full vectors)

    With T_samp = 0.01 s and T_step = 0.1 s, the pending deque holds ~10 entries
    at steady state before each prediction matures.

    Validation support
    ------------------
    The internal _rec dict mirrors the fields of rec_dict from the main loop.
    Call validate(rec_dict) after get_data() to assert that both are consistent.

    Parameters
    ----------
    nx : int          State dimension.
    nu : int          Control input dimension.
    T_step : float    MPC horizon step size in seconds (T_horizon / N).
    T_samp : float    Control loop sampling period in seconds. Must satisfy T_samp <= T_step.
    buffer_size : int Maximum number of (X, Y) training samples (circular, FIFO on overflow).
    min_samples : int Minimum matured samples before sample_batch() can be called.
    batch_size : int  Mini-batch size returned by sample_batch().
    state_indices : list[int] | None
                      State dimensions included in the residual label Y.
                      None defaults to all nx dimensions.
    window_size : int Number of consecutive observations concatenated as training input X.
                      1 = no temporal context (default, backward-compatible).
                      N > 1 = sliding window of N steps; X has shape (buffer_size, N*(nx+nu)).
    forget_tau : float
                      Forgetting time constant IN SECONDS for the "weighted"
                      sampling strategy: a sample of age dt is drawn with
                      probability proportional to exp(-dt / forget_tau).

                      This replaces the former `weighted_decay`, which weighted
                      by POSITION in the buffer (exp(decay * chrono_pos)) and was
                      therefore not a fixed horizon at all: with decay=1e-3 the
                      newest/oldest ratio was exp(0.26) ~ 1.3 (nearly uniform) at
                      256 stored samples but exp(10) ~ 2.2e4 (only the last ~10 s
                      matter) once the 10000-sample buffer had filled. The
                      effective forgetting horizon silently drifted during the
                      flight. Weighting by age fixes the horizon at forget_tau
                      seconds regardless of the buffer fill level.
                      Set <= 0 to fall back to uniform sampling.
    """

    def __init__(
        self,
        nx: int,
        nu: int,
        T_step: float,
        T_samp: float,
        buffer_size: int = 5000,
        min_samples: int = 200,
        batch_size: int = 64,
        state_indices: list = None,
        window_size: int = 1,
        forget_tau: float = 10.0,
    ):
        assert T_samp <= T_step, "T_samp must be <= T_step"
        assert window_size >= 1, "window_size must be >= 1"

        self.nx = nx
        self.nu = nu
        self.T_step = T_step
        self.T_samp = T_samp
        self.buffer_size = buffer_size
        self.min_samples = min_samples
        self.batch_size = batch_size
        self.state_indices = state_indices if state_indices is not None else list(range(nx))
        self.window_size = window_size
        self.forget_tau = forget_tau

        n_out = len(self.state_indices)
        x_dim = window_size * (nx + nu)

        # Circular training buffer (pre-allocated for speed)
        self._X = np.zeros((buffer_size, x_dim))    # windowed input
        self._Y = np.zeros((buffer_size, n_out))    # residual label
        self._T = np.zeros((buffer_size,))          # time the input window was observed [s]
        self._ptr = 0    # write pointer (wraps around)
        self._count = 0  # number of valid samples currently stored
        self._t_last = 0.0  # most recent t_now seen by get_data(); "age zero" reference

        # Rolling history of (window_size) most recent [state_curr; u_cmd] pairs.
        # Each entry is a 1-D array of length nx + nu.
        self._history: deque = deque(maxlen=window_size)

        # Pending queue: each entry waits T_step seconds for its ground-truth.
        # Tuple layout: (t_stored, X_window_flat, mpc_first_pred)
        self._pending: deque = deque()

        # ------------------------------------------------------------------
        # Full flight recording — grows at every get_data() call, NEVER
        # overwritten. Stores the complete flight history for offline analysis.
        # Only the training buffer (_X, _Y) is circular and size-limited.
        #
        # To compute the residual offline for step k (rate, /T_step):
        #   Y[k] = (state_curr[k + T_step/T_samp] - mpc_first_pred[k]) / T_step
        # ------------------------------------------------------------------
        self._rec = {
            "timestamp":          np.zeros((0,)),
            "dt":                 np.zeros((0,)),
            "comp_time":          np.zeros((0,)),
            "state_ref":          np.zeros((0, nx)),
            "state_curr":         np.zeros((0, nx)),
            "state_out":          np.zeros((0, nx)),   # actual next state (T_samp ahead)
            "state_pred":         np.zeros((0, nx)),   # nominal forward_prop prediction
            "control":            np.zeros((0, nu)),
            "mpc_first_pred":     np.zeros((0, nx)),   # MPC node-1 prediction (T_step ahead)
            "nominal_first_pred": np.zeros((0, nx)),   # nominal prediction (no NN, T_step ahead)
            # 1.0 while actually TRACKING a trajectory, 0.0 during take-off and
            # during the repositioning between two trajectory segments.
            #
            # In that repositioning the reference is a sigmoid sliding from the
            # last pose to the next segment's start, so it deliberately sits
            # ahead of the aircraft and the position error is large by
            # construction. Any "worst departure from the reference" computed
            # over the whole tracking phase measures that transit, not
            # disturbance rejection: nominal MPC scored the SMALLEST maximum of
            # every controller (71.7 cm vs 82-91 cm) purely because of it.
            "tracking":           np.zeros((0,)),
        }

    # ------------------------------------------------------------------
    # Core data collection  (call every control step)
    # ------------------------------------------------------------------

    def get_data(self, observation: dict) -> None:
        """
        Feed one control step into the dataset.

        Steps performed:
          1. Append all fields to the internal _rec dict (for validation).
          2. Append [state_curr, u_cmd] to the rolling history and build the
             current sliding-window vector X_window.
          3. Check whether any pending predictions have matured; if so, write
             the (X_window, residual) pair to the circular training buffer.
          4. Queue the current step for future labeling.

        Parameters
        ----------
        observation : dict with keys
            "t_now"          float        current virtual time (seconds)
            "state_curr"     ndarray(nx)  observed state BEFORE the simulation step
            "u_cmd"          ndarray(nu)  control command applied this step
            "mpc_first_pred" ndarray(nx)  ocp_solver.get(1, "x") — predicted state
                                           at t_now + T_step
            --- optional (used for _rec recording and validation) ---
            "comp_time"      float        MPC solve time in ms
            "state_ref"      ndarray(nx)  reference state (horizon step 0)
            "state_out"      ndarray(nx)  actual next state after simulation
            "state_pred"     ndarray(nx)  nominal-model prediction (forward_prop)
        """
        t_now          = observation["t_now"]
        state_curr     = observation["state_curr"]
        u_cmd          = observation["u_cmd"]
        mpc_first_pred = observation["mpc_first_pred"]

        # "Now" reference used to age the stored samples in sample_batch().
        self._t_last = t_now

        # ---- 1. Record to _rec ----
        dt_val = (t_now - self._rec["timestamp"][-1]
                  if len(self._rec["timestamp"]) > 0 else self.T_samp)
        self._rec["timestamp"] = np.append(self._rec["timestamp"], t_now)
        self._rec["dt"]        = np.append(self._rec["dt"],        dt_val)
        self._rec["tracking"] = np.append(
            self._rec["tracking"], float(observation.get("tracking", 1.0)))
        self._rec["comp_time"] = np.append(
            self._rec["comp_time"], observation.get("comp_time", 0.0)
        )
        state_ref = observation.get("state_ref", np.zeros(self.nx))
        self._rec["state_ref"]  = np.append(
            self._rec["state_ref"], state_ref[np.newaxis, :], axis=0
        )
        self._rec["state_curr"] = np.append(
            self._rec["state_curr"], state_curr[np.newaxis, :], axis=0
        )
        state_out = observation.get("state_out", np.zeros(self.nx))
        self._rec["state_out"]  = np.append(
            self._rec["state_out"], state_out[np.newaxis, :], axis=0
        )
        state_pred = observation.get("state_pred", np.zeros(self.nx))
        self._rec["state_pred"] = np.append(
            self._rec["state_pred"], state_pred[np.newaxis, :], axis=0
        )
        self._rec["control"] = np.append(
            self._rec["control"], u_cmd[np.newaxis, :], axis=0
        )
        self._rec["mpc_first_pred"] = np.append(
            self._rec["mpc_first_pred"], mpc_first_pred[np.newaxis, :], axis=0
        )
        nominal_pred = observation.get("nominal_first_pred")
        if nominal_pred is None:
            nominal_pred = mpc_first_pred
        self._rec["nominal_first_pred"] = np.append(
            self._rec["nominal_first_pred"], nominal_pred[np.newaxis, :], axis=0
        )

        # ---- 2. Update rolling history and build window vector ----
        self._history.append(np.concatenate([state_curr, u_cmd]))
        X_window = self._build_window()

        # ---- 3. Match matured pending predictions ----
        # A prediction stored at t_stored matures when t_now >= t_stored + T_step.
        # With T_samp=0.01s and T_step=0.1s this happens after exactly 10 steps.
        while self._pending:
            t_stored, X_win, pred = self._pending[0]
            if t_now >= t_stored + self.T_step - 1e-9:
                self._pending.popleft()
                # Divide by T_step to obtain the residual *rate* (acceleration for
                # velocity dims), matching the offline label convention in
                # model_fitting/dataset.py (prop_long_horizon):
                #     y = (state_out - state_pred) / T_step
                # The NN output is added to ds (the continuous dynamics), so it
                # must be an acceleration. Without this division the online label
                # would be a plain state difference over T_step (a factor of
                # T_step too small), and online training would drive the model
                # toward a ~10x-too-weak residual that fails to compensate the
                # disturbance.
                residual = (state_curr[self.state_indices] - pred[self.state_indices]) / self.T_step
                idx = self._ptr % self.buffer_size
                self._X[idx] = X_win
                self._Y[idx] = residual
                # Timestamp of the INPUT window (t_stored), not of the moment the
                # label matured: the sample describes the dynamics at t_stored, so
                # that is the time its age must be measured from.
                self._T[idx] = t_stored
                self._ptr += 1
                self._count = min(self._count + 1, self.buffer_size)
            else:
                break

        # ---- 4. Queue current step for future labeling ----
        # nominal_pred was extracted in section 1 (falling back to mpc_first_pred
        # when absent) so the label matches the offline convention:
        #   residual = actual(T+T_step) - nominal(T+T_step)
        self._pending.append((t_now, X_window, nominal_pred.copy()))

    def _build_window(self) -> np.ndarray:
        """Flatten the rolling history into a 1-D window vector (zero-padded at start)."""
        if not self._history:
            return np.zeros(self.window_size * (self.nx + self.nu))
        history_arr = np.array(self._history)           # (len_hist, nx + nu)
        n_pad = self.window_size - len(history_arr)
        if n_pad > 0:
            pad = np.zeros((n_pad, self.nx + self.nu))
            history_arr = np.concatenate([pad, history_arr], axis=0)
        return history_arr.flatten()

    # ------------------------------------------------------------------
    # Validation  (compare _rec against external rec_dict)
    # ------------------------------------------------------------------

    def validate(self, rec_dict: dict, step: int = None, verbose: bool = True,
                 full: bool = False) -> bool:
        """
        Compare _rec against rec_dict.

        Parameters
        ----------
        rec_dict : external recording dict from the main loop
        step     : current control step (for display only)
        verbose  : print a diff summary on mismatch
        full     : if False (default), compare only the last entry of each field.
                   if True, compare ALL entries that exist in both dicts
                   (uses the minimum shared length — safe when rec_dict is periodically reset).

        Checked fields: timestamp, state_curr, control, state_out, state_pred.
        Fields absent from rec_dict are skipped.

        Returns True if all checked fields match within tolerance.
        """
        n_self = len(self._rec["timestamp"])
        if n_self == 0 or len(rec_dict.get("timestamp", [])) == 0:
            return True

        fields = ["timestamp", "state_curr", "control", "state_out", "state_pred"]
        errors = []

        for name in fields:
            ref = rec_dict.get(name)
            if ref is None or len(ref) == 0:
                continue

            if full:
                # Compare the full shared history
                n_compare = min(n_self, len(ref))
                my_vals  = self._rec[name][-n_compare:]
                ref_vals = np.asarray(ref)[-n_compare:]
                if not np.allclose(my_vals, ref_vals, rtol=1e-5, atol=1e-8):
                    diffs = np.abs(np.asarray(my_vals) - np.asarray(ref_vals))
                    bad_rows = int(np.any(diffs > 1e-8, axis=-1).sum()) if diffs.ndim > 1 \
                               else int((diffs > 1e-8).sum())
                    errors.append(
                        f"  '{name}': {bad_rows}/{n_compare} rows differ, "
                        f"max_abs_diff={float(diffs.max()):.3e}"
                    )
            else:
                # Compare only the last entry (fast per-step check)
                my_val  = self._rec[name][-1]
                ref_val = ref[-1]
                if not np.allclose(my_val, ref_val, rtol=1e-5, atol=1e-8):
                    max_diff = float(np.max(np.abs(np.asarray(my_val) - np.asarray(ref_val))))
                    errors.append(f"  '{name}': max_abs_diff={max_diff:.3e}")

        if errors and verbose:
            mode  = "FULL" if full else "LAST"
            label = f" step={step}" if step is not None else ""
            print(f"[OnlineDataset.validate({mode})]{label} MISMATCH:\n" + "\n".join(errors))
            return False
        return True

    # ------------------------------------------------------------------
    # Buffer access
    # ------------------------------------------------------------------

    def get_buffer(self) -> tuple:
        """Return all valid (X, Y) pairs currently stored in the training buffer."""
        return self._X[: self._count], self._Y[: self._count]

    def _chrono_to_idx(self, chrono_pos: np.ndarray) -> np.ndarray:
        """
        Convert chronological positions (0 = oldest, n-1 = newest) to
        actual array indices in the circular buffer.

        Before wrap (count < buffer_size): array index == chrono position.
        After wrap: oldest entry is at ptr % buffer_size, so
            array_index = (oldest + chrono_pos) % buffer_size.
        """
        if self._count < self.buffer_size:
            return chrono_pos
        oldest = self._ptr % self.buffer_size
        return (oldest + chrono_pos) % self.buffer_size

    def sample_batch(self, strategy: str = "random") -> tuple:
        """
        Draw a mini-batch for one gradient step.

        All strategies work correctly even after the circular buffer has
        wrapped around (i.e., when buffer_full is True).

        Parameters
        ----------
        strategy : "random"   — uniform sampling (default)
                   "recent"   — most recent batch_size samples
                   "weighted" — recency-weighted by AGE IN SECONDS: a sample of
                                age dt is drawn with probability proportional to
                                exp(-dt / forget_tau). Unlike the previous
                                position-based weighting, the forgetting horizon
                                no longer depends on how full the buffer is.

        Returns
        -------
        X_batch : ndarray (k, window_size * (nx + nu))
        Y_batch : ndarray (k, len(state_indices))
                  with k = min(batch_size, n_samples)
        """
        n = self._count
        if n == 0:
            raise ValueError("sample_batch() called on an empty buffer.")
        k = min(self.batch_size, n)   # never ask for more samples than are stored

        if strategy == "random":
            # Uniform random — chrono position = array index before wrap, irrelevant after
            indices = np.random.choice(n, size=k, replace=False)
            return self._X[indices], self._Y[indices]

        if strategy == "recent":
            chrono_pos = np.arange(n - k, n)
            indices = self._chrono_to_idx(chrono_pos)
        elif strategy == "weighted":
            indices = self._weighted_indices(k)
        else:
            raise ValueError(f"Unknown sampling strategy: '{strategy}'")

        return self._X[indices], self._Y[indices]

    def _weighted_indices(self, k: int) -> np.ndarray:
        """
        Draw k distinct array indices with probability proportional to
        exp(-age / forget_tau), age = t_last - t_sample in SECONDS.

        Sampling uses the Gumbel-top-k trick: adding independent Gumbel(0,1)
        noise to the log-weights and taking the k largest keys yields exactly
        the same distribution as successive weighted sampling without
        replacement (which is what np.random.choice(replace=False, p=...)
        implements), but in O(n) instead of the O(n^2)-ish loop numpy runs — the
        gradient step happens inside the control loop, so this matters.

        Note: because indices 0..count-1 are exactly the valid slots both before
        and after the buffer wraps, no chronological remapping is needed here —
        the timestamps carry the ordering.
        """
        n = self._count
        if k >= n:
            return np.arange(n)
        if self.forget_tau is None or self.forget_tau <= 0:
            return np.random.choice(n, size=k, replace=False)

        age = np.maximum(self._t_last - self._T[:n], 0.0)     # seconds
        log_w = -age / self.forget_tau                        # <= 0, no overflow
        # Gumbel(0,1) = -log(-log(U)); clip U away from 0 and 1 to stay finite.
        u = np.clip(np.random.random(n), 1e-12, 1.0 - 1e-12)
        keys = log_w - np.log(-np.log(u))
        return np.argpartition(-keys, k - 1)[:k]



    def recent_batch(self, n_samples: int) -> tuple:
        """
        Return the n_samples most recent (X, Y) pairs, newest last.

        Used by the online trainer's supervisor to compare the adapted model
        against the frozen baseline on fresh data. Returns None when the buffer
        is empty.
        """
        n = self._count
        if n == 0:
            return None
        m = min(int(n_samples), n)
        indices = self._chrono_to_idx(np.arange(n - m, n))
        return self._X[indices], self._Y[indices]

    def ready_for_training(self) -> bool:
        """Returns True when enough matured samples are available."""
        return self._count >= self.min_samples

    # ------------------------------------------------------------------
    # Persistence and reset
    # ------------------------------------------------------------------

    def save(self, path: str) -> None:
        """
        Save both the full flight recording (_rec) and the training buffer (X, Y).

        The .npz file contains:
          - All _rec fields (complete flight history, n_recorded rows each)
          - X, Y  (circular training buffer, n_samples rows)
        """
        np.savez(
            path,
            # --- full flight recording ---
            timestamp=self._rec["timestamp"],
            dt=self._rec["dt"],
            comp_time=self._rec["comp_time"],
            state_ref=self._rec["state_ref"],
            state_curr=self._rec["state_curr"],
            state_out=self._rec["state_out"],
            state_pred=self._rec["state_pred"],
            control=self._rec["control"],
            mpc_first_pred=self._rec["mpc_first_pred"],
            nominal_first_pred=self._rec["nominal_first_pred"],
            # --- training buffer ---
            X=self._X[: self._count],
            Y=self._Y[: self._count],
        )
        print(f"OnlineDataset: saved {self.n_recorded} recorded steps "
              f"and {self._count} training samples to {path}")

    def reset(self) -> None:
        """
        Clear the circular training buffer, the pending queue, and the
        rolling history. Does NOT clear _rec — call reset_rec() for that.
        """
        self._X[:] = 0
        self._Y[:] = 0
        self._T[:] = 0
        self._ptr = 0
        self._count = 0
        self._pending.clear()
        self._history.clear()

    def reset_rec(self) -> None:
        """Clear the internal recording dict (frees memory after saving)."""
        self._rec = {
            "timestamp":          np.zeros((0,)),
            "dt":                 np.zeros((0,)),
            "comp_time":          np.zeros((0,)),
            "state_ref":          np.zeros((0, self.nx)),
            "state_curr":         np.zeros((0, self.nx)),
            "state_out":          np.zeros((0, self.nx)),
            "state_pred":         np.zeros((0, self.nx)),
            "control":            np.zeros((0, self.nu)),
            "mpc_first_pred":     np.zeros((0, self.nx)),
            "nominal_first_pred": np.zeros((0, self.nx)),
        }

    # ------------------------------------------------------------------
    # Inspection helpers
    # ------------------------------------------------------------------

    @property
    def n_samples(self) -> int:
        """Number of valid (X, Y) pairs in the circular training buffer (capped at buffer_size)."""
        return self._count

    @property
    def n_recorded(self) -> int:
        """Total number of control steps recorded in _rec (never overwritten, grows forever)."""
        return len(self._rec["timestamp"])

    @property
    def buffer_full(self) -> bool:
        """True when the circular buffer has wrapped around at least once."""
        return self._count >= self.buffer_size

    @property
    def n_pending(self) -> int:
        """Number of predictions still waiting for their ground-truth state."""
        return len(self._pending)

    @property
    def x_dim(self) -> int:
        """Input dimension: window_size * (nx + nu)."""
        return self.window_size * (self.nx + self.nu)
