import copy
import os
import numpy as np
import torch
import torch.nn as nn
from torch.optim.lr_scheduler import LambdaLR

from online_learning.core.online_data import OnlineDataset
from online_learning.core.online_guards import (WeightGuard, BaselineSupervisor, report_adaptation,
                           get_flat, set_flat)


class OnlineTrainer:
    """
    Performs online gradient updates on the neural residual model used by the MPC.

    The trainer operates on the same PyTorch model instance embedded in the MPC
    controller (neural_mpc.neural_model). Weight updates happen in-place.

    Sliding-window support
    ----------------------
    When window_size > 1 the OnlineDataset stores a flat vector of
    window_size consecutive [state_curr, u_cmd] pairs as X.
    For each time step in the window, this trainer extracts state_feats and
    u_feats and concatenates them, giving a total input of:
        window_size * (len(state_feats) + len(u_feats))
    The model is therefore expected to have a matching input layer when
    window_size > 1.  With window_size=1 (default) behavior is identical to
    using a single-step observation (backward-compatible).

    Safety layer
    ------------
    Online training runs inside the control loop of a flying robot, so a
    diverging update is not a failed experiment — it is a crash. Three
    independent guards bound what a gradient step can do, from the cheapest to
    the strongest:

      1. per-step rate limit   ||W_k - W_{k-1}|| <= max_step_rel * ||W_0||
         The MPC runs SQP_RTI, i.e. ONE QP iteration per control step. That
         relies on the model changing slowly between iterations; a large weight
         jump invalidates the warm start. The rate limit enforces that
         assumption explicitly instead of hoping the learning rate is small
         enough.
      2. hard trust region     ||W   - W_0||     <= trust_region_rel * ||W_0||
         A ball around the pre-trained weights. Unlike lambda_anchor (a soft
         penalty whose strength is rescaled by Adam's per-parameter step) this
         bound is unconditional: the online model can never wander arbitrarily
         far from a baseline that is known to fly.
      3. supervisor            see supervise()
         Rolling check that the adapted model actually predicts better than the
         frozen baseline; reverts to the baseline when it does not.

    A fourth guard lives in the controller, not here: the residual injected into
    the dynamics is smoothly saturated inside the CasADi graph
    (model_options["residual_sat"]), so even a corrupted network cannot command
    an unbounded acceleration.

    MPC coupling note
    -----------------
    When the MPC solver was built with linearize_mlp=False (direct ca_forward()
    embedding), the running solver is NOT updated by in-place weight changes —
    its CasADi graph baked the weights as constants at build time.
    With model_options["online_neural_mpc"]=True the weights are acados
    parameters instead, and set_mlp_params() pushes them to the running solver
    without a rebuild. Use transfer_model() to persist the updated weights.

    Parameters
    ----------
    model : torch.nn.Module
        Residual MLP to train (neural_mpc.neural_model). Modified in-place.
    nx : int
        Full state dimension.
    nu : int
        Full control dimension.
    state_feats : list[int]
        Indices of state_curr used as NN input (from mlp_metadata ModelFitConfig).
    u_feats : list[int]
        Indices of u_cmd used as NN input (from mlp_metadata ModelFitConfig).
    y_reg_dims : array-like of int
        State dimensions predicted by the NN (from mlp_metadata ModelFitConfig).
        Used to slice the residual label Y from the dataset.
    state_indices : list[int]
        state_indices used when OnlineDataset was created. Every element of
        y_reg_dims must be present here so that Y[:, mapped_col] is valid.
    device : torch.device
        Device for training (CPU or CUDA).
    lr : float
        Learning rate for the Adam optimizer.
    window_size : int
        Must match the window_size used in the paired OnlineDataset.
        Default 1 = single-step input (no temporal context).
    grad_clip_norm : float
        Maximum L2 norm for gradient clipping. Set to 0 to disable.
        Default 1.0 — prevents divergence from noisy online labels.
    n_frozen_layers : int
        Number of initial trainable layers (caLinear / caBatchNorm1D) to keep
        frozen during online training.  Frozen layers preserve the
        offline-learned feature representations; only the remaining layers
        adapt online.  Default 0 = all layers trainable.
    warmup_steps : int
        Number of gradient steps over which the learning rate rises linearly
        from 0 to its target value.  Avoids large initial updates that could
        corrupt the pre-trained baseline.  Default 0 = no warmup.
    train_every : int
        Run a gradient step only once every train_every calls to
        should_train().  Set > 1 to reduce per-control-iteration overhead
        while keeping the buffer well-populated between updates.  Default 1.
    lambda_anchor : float
        Rate of the DECOUPLED L2 pull toward the pre-trained baseline captured
        at construction time. After each optimizer step:
            W <- W - lr * lambda_anchor * (W - W_init)
        When no disturbance is present the MSE gradient vanishes and the anchor
        term drives the weights back toward the offline baseline, producing
        natural forgetting without a hard reset.

        NOTE: this is applied OUTSIDE the loss, AdamW-style. Adding
        lambda_anchor*||W-W_0||^2 to the loss (the previous implementation)
        routes the penalty through Adam's adaptive per-parameter scaling, so
        its effective strength differs from layer to layer and lambda_anchor
        has no interpretable unit. Decoupled, lambda_anchor is simply a pull
        rate: lr*lambda_anchor is the fraction of the distance to the baseline
        removed per step. Set to 0.0 (default) to disable.
    trust_region_rel : float
        Hard bound ||W - W_0|| <= trust_region_rel * ||W_0||, enforced by
        projection after every step. Set to 0 to disable.

        Default 3.0, calibrated on neuralmodel_209 rather than guessed:
          - LEGITIMATE adaptation to the configured disturbances needs
            ||W-W_0||/||W_0|| = 0.47 (lr 1e-3, 1.6 m/s² payload) up to 1.24
            (lr 1e-2, 4 m/s²). Note the demand grows with lr — a large learning
            rate reaches the same fit through a longer, less efficient path in
            weight space, so it is the first thing to lower if this bound binds.
          - an UNGUARDED divergence was measured at ~250x.
        3.0 therefore sits ~2.4x above the largest honest demand and ~80x below
        a runaway: it should never bind in flight, and report() tells you if it
        did.
    max_step_rel : float
        Hard bound ||W_k - W_{k-1}|| <= max_step_rel * ||W_0|| per gradient
        step. Set to 0 to disable.

        Default 0.1. Measured per-step motion on the same model is 0.0026
        (lr 1e-3), 0.022 (lr 1e-2) and 0.056 (lr 3e-2), so this leaves ~5x
        headroom at the configured learning rate and only catches a jump that
        would invalidate the SQP_RTI warm start.
    supervisor_every : int
        Run the baseline comparison every N gradient steps. 0 disables the
        supervisor entirely.
    supervisor_window : int
        Number of most recent matured samples used for the comparison.
    supervisor_tol : float
        Revert when mse_online > supervisor_tol * mse_baseline.
    supervisor_patience : int
        Number of consecutive failed checks before reverting.
    revert_lr_decay : float
        Learning rate multiplier applied on each revert (1.0 = keep lr). The
        default 0.5 turns a divergence into an automatic learning-rate backoff
        instead of a revert/diverge cycle.
    """

    def __init__(
        self,
        model: nn.Module,
        nx: int,
        nu: int,
        state_feats: list,
        u_feats: list,
        y_reg_dims,
        state_indices: list,
        device: torch.device,
        lr: float = 1e-4,
        window_size: int = 1,
        grad_clip_norm: float = 1.0,
        n_frozen_layers: int = 0,
        warmup_steps: int = 0,
        train_every: int = 1,
        lambda_anchor: float = 0.0,
        trust_region_rel: float = 3.0,
        max_step_rel: float = 0.1,
        supervisor_every: int = 50,
        supervisor_window: int = 256,
        supervisor_tol: float = 1.0,
        supervisor_patience: int = 3,
        revert_lr_decay: float = 0.5,
    ):
        assert window_size >= 1
        assert train_every >= 1

        self.model = model
        self.nx = nx
        self.nu = nu
        self.state_feats = list(state_feats)
        self.u_feats = list(u_feats)
        self.y_reg_dims = np.asarray(y_reg_dims, dtype=int)
        self.state_indices = list(state_indices)
        self.device = device
        self.window_size = window_size
        self.grad_clip_norm = grad_clip_norm
        self.n_frozen_layers = n_frozen_layers
        self.warmup_steps = warmup_steps
        self.train_every = train_every
        self.lambda_anchor = lambda_anchor
        self.supervisor_every = int(supervisor_every)
        self.supervisor_window = int(supervisor_window)
        self.supervisor_tol = float(supervisor_tol)
        self.supervisor_patience = int(supervisor_patience)
        self.revert_lr_decay = float(revert_lr_decay)

        # Map y_reg_dims to column positions in Y (which is indexed by state_indices)
        for dim in self.y_reg_dims:
            if dim not in self.state_indices:
                raise ValueError(
                    f"y_reg_dim {dim} not found in state_indices. "
                    "Create OnlineDataset with state_indices=list(range(nx))."
                )
        self._y_col = np.array([self.state_indices.index(d) for d in self.y_reg_dims])

        # Pre-compute the input column indices once. learn() runs inside the
        # control loop, so it must not rebuild index arrays on every call.
        # Layout mirrors the MPC model build: per window step w, the state
        # features then the control features, windows concatenated oldest-first.
        stride = self.nx + self.nu
        self._x_cols = np.concatenate([
            np.concatenate([
                w * stride + np.asarray(self.state_feats, dtype=int),
                w * stride + self.nx + np.asarray(self.u_feats, dtype=int),
            ])
            for w in range(self.window_size)
        ])

        # load_model() freezes all parameters (requires_grad=False) for inference.
        # Re-enable gradients so that loss.backward() + optimizer.step() actually
        # update the weights during online training.
        for param in self.model.parameters():
            param.requires_grad_(True)

        # Freeze selected initial layers to preserve offline-learned features
        self._apply_layer_freezing()

        # Trainable parameter list, fixed from here on: the optimizer, the
        # gradient clipping and every safety guard operate on exactly this set.
        self._trainable = [p for p in self.model.parameters() if p.requires_grad]
        if not self._trainable:
            raise ValueError(
                f"n_frozen_layers={n_frozen_layers} froze every parameter — "
                "nothing left to train."
            )

        # Capture pre-trained weights as the baseline for the anchor, the trust
        # region and the supervisor. Only trainable parameters are stored;
        # frozen layers never receive updates so they are identical by
        # construction.
        self._anchor_params = {
            name: p.clone().detach()
            for name, p in self.model.named_parameters()
            if p.requires_grad
        }
        self._anchor_list = [
            (p, self._anchor_params[name])
            for name, p in self.model.named_parameters()
            if p.requires_grad
        ]

        # Hard bounds (rate limit + trust region), shared implementation.
        self.guard = WeightGuard(self._trainable, trust_region_rel, max_step_rel)
        self.loss_fn = nn.MSELoss()

        # Divergence detector against a frozen copy of the pre-trained model.
        self.supervisor = BaselineSupervisor(
            self.model, self.loss_fn, every=self.supervisor_every,
            window=self.supervisor_window, tol=self.supervisor_tol,
            patience=self.supervisor_patience,
        )

        # BatchNorm running statistics are baked as CONSTANTS into the CasADi
        # graph at solver build time (only gamma/beta are acados parameters).
        # Letting them drift in train() mode would silently desynchronise the
        # MPC's model from the PyTorch model. _set_train_mode() keeps every
        # BatchNorm in eval mode; warn once so the trade-off is visible.
        self._has_batchnorm = any(
            isinstance(m, nn.modules.batchnorm._BatchNorm) for m in self.model.modules()
        )
        if self._has_batchnorm:
            print("[OnlineTrainer] BatchNorm detected: running statistics are held "
                  "FROZEN during online training because the acados graph bakes "
                  "them as constants (only gamma/beta are parameters).")

        # Build optimizer with only trainable (unfrozen) parameters
        self._base_lr = float(lr)
        self.optimizer = torch.optim.Adam(self._trainable, lr=self._base_lr)

        # LR warmup scheduler
        self.scheduler = self._make_scheduler()

        self._step_count = 0
        self._sim_steps = 0
        self._reset_counters()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _reset_counters(self) -> None:
        """Diagnostic counters — how often each guard actually fired."""
        self._n_skipped_nonfinite = 0   # updates dropped (NaN/Inf loss or gradient)
        self._n_reverts = 0
        self._last_loss = float("nan")  # pure data MSE of the last step
        self.guard.n_rate_limited = 0
        self.guard.n_trust_clipped = 0
        self.supervisor.n_checks = 0
        self.supervisor.n_fails = 0
        self.supervisor.reset()

    def _apply_layer_freezing(self) -> None:
        """
        Freeze the first n_frozen_layers trainable layers.

        Iterates over model.fully_connected_stack (if present) and freezes
        any layer that has parameters, up to n_frozen_layers.  Activation
        layers (ReLU, GELU, …) are skipped since they have no parameters.
        """
        if self.n_frozen_layers <= 0:
            return
        if not hasattr(self.model, "fully_connected_stack"):
            return
        frozen = 0
        for layer in self.model.fully_connected_stack:
            if frozen >= self.n_frozen_layers:
                break
            params = list(layer.parameters())
            if params:  # only layers with trainable params count
                for p in params:
                    p.requires_grad_(False)
                frozen += 1

    def _make_scheduler(self) -> LambdaLR:
        """Create a LambdaLR scheduler with optional linear warmup."""
        warmup = self.warmup_steps

        def lr_lambda(step: int) -> float:
            if warmup > 0 and step < warmup:
                return float(step + 1) / float(warmup)
            return 1.0

        return LambdaLR(self.optimizer, lr_lambda=lr_lambda)

    def _set_train_mode(self) -> None:
        """
        Put the model in training mode for the gradient step, but keep every
        BatchNorm layer in eval mode (see __init__ for why). Dropout stays
        active, matching the offline training regime.
        """
        self.model.train()
        for m in self.model.modules():
            if isinstance(m, nn.modules.batchnorm._BatchNorm):
                m.eval()

    def _get_flat(self) -> torch.Tensor:
        """Flatten all trainable parameters into a single detached 1-D tensor."""
        return get_flat(self._trainable)

    def _set_flat(self, vec: torch.Tensor) -> None:
        """Write a flat vector back into the trainable parameters."""
        set_flat(self._trainable, vec)

    def _to_tensors(self, data: tuple) -> tuple:
        """
        Turn a raw (X, Y) buffer batch into model-ready tensors.

        Feature extraction mirrors the MPC model build: for each window step w,
        state_feats are taken at offset w*(nx+nu) and u_feats at
        w*(nx+nu)+nx, then all windows are concatenated (see self._x_cols).

        No normalisation is applied here: self.model.forward() normalises its
        input (x_mean/x_std) and denormalises its output (y_mean/y_std)
        internally, exactly like ca_forward() at inference time, and the offline
        training loop feeds raw physical units too. X and Y therefore stay in
        raw physical units and the loss is an MSE in m/s² — the same quantity
        that was minimised offline.
        """
        X_raw, Y_raw = data
        X_feat = np.ascontiguousarray(X_raw[:, self._x_cols], dtype=np.float32)
        Y_feat = np.ascontiguousarray(Y_raw[:, self._y_col], dtype=np.float32)
        return (torch.from_numpy(X_feat).to(self.device),
                torch.from_numpy(Y_feat).to(self.device))

    # ------------------------------------------------------------------
    # Data retrieval
    # ------------------------------------------------------------------

    def get_data(self, dataset: OnlineDataset, strategy: str = "weighted") -> tuple:
        """
        Sample a mini-batch from the dataset.

        Parameters
        ----------
        dataset  : OnlineDataset to sample from
        strategy : passed to dataset.sample_batch()

        Returns
        -------
        (X_raw, Y_raw) numpy arrays to be passed to learn()
        """
        return dataset.sample_batch(strategy=strategy)

    # ------------------------------------------------------------------
    # Training frequency control
    # ------------------------------------------------------------------

    def should_train(self) -> bool:
        """
        Increment the simulation-step counter and return True every
        train_every calls.  Call once per control step when the buffer is
        ready; the caller skips the gradient step when this returns False.
        """
        self._sim_steps += 1
        return (self._sim_steps % self.train_every) == 0

    # ------------------------------------------------------------------
    # Training step
    # ------------------------------------------------------------------

    def learn(self, data: tuple) -> float:
        """
        Perform one guarded gradient update step.

        Sequence:
          1. forward + MSE on raw physical units
          2. drop the update if the loss is not finite
          3. backward, gradient-norm clipping; drop the update if the gradient
             is not finite
          4. Adam step
          5. decoupled anchor pull toward the pre-trained weights
          6. per-step rate limit, then projection onto the trust region

        Parameters
        ----------
        data : (X_raw, Y_raw) returned by get_data()
              X_raw shape: (batch, window_size * (nx + nu))
              Y_raw shape: (batch, len(state_indices))

        Returns
        -------
        loss : float
            The PURE data MSE for this step, never mixed with the anchor
            penalty — so it stays comparable across lambda_anchor settings and
            usable as a divergence signal. NaN when the update was dropped
            because the loss itself was not finite.
        """
        X_tensor, Y_tensor = self._to_tensors(data)

        self._set_train_mode()
        self.optimizer.zero_grad(set_to_none=True)
        Y_pred = self.model(X_tensor)
        loss = self.loss_fn(Y_pred, Y_tensor)

        # --- Guard 0: never let a non-finite loss reach the weights ---------
        # A single NaN label or an exploded activation would otherwise poison
        # every weight permanently through Adam's moments — on hardware that is
        # not a failed run, it is a fall.
        if not torch.isfinite(loss):
            self.optimizer.zero_grad(set_to_none=True)
            self._n_skipped_nonfinite += 1
            self.model.eval()
            self._last_loss = float("nan")
            return float("nan")

        data_loss = float(loss.detach())
        loss.backward()

        # Clip on the TRAINABLE parameters only, consistently with the optimizer.
        # max_norm=inf performs no clipping but still returns the total norm,
        # which is the cheapest way to detect a non-finite gradient.
        max_norm = self.grad_clip_norm if self.grad_clip_norm > 0 else float("inf")
        total_norm = torch.nn.utils.clip_grad_norm_(self._trainable, max_norm)
        if not torch.isfinite(total_norm):
            self.optimizer.zero_grad(set_to_none=True)
            self._n_skipped_nonfinite += 1
            self.model.eval()
            self._last_loss = data_loss
            return data_loss

        lr_now = self.optimizer.param_groups[0]["lr"]   # lr actually used by this step
        self.optimizer.step()
        self.scheduler.step()

        self._apply_guards(lr_now)

        self.model.eval()
        self._step_count += 1
        self._last_loss = data_loss
        return data_loss

    @torch.no_grad()
    def _apply_guards(self, lr_now: float) -> None:
        """Decoupled anchor pull, then the shared hard bounds."""
        # --- Decoupled anchor (AdamW-style): W <- W - lr*lambda*(W - W_0) ----
        if self.lambda_anchor > 0.0:
            coeff = min(lr_now * self.lambda_anchor, 1.0)   # coeff>1 would overshoot
            for p, p0 in self._anchor_list:
                p.add_(p0 - p, alpha=coeff)

        # --- Per-step rate limit + hard trust region (see online_guards.py) ---
        self.guard.project()

    # ------------------------------------------------------------------
    # Supervisor
    # ------------------------------------------------------------------

    def supervise(self, dataset: OnlineDataset) -> bool:
        """
        Compare the adapted model against the frozen pre-trained baseline on the
        most recent matured samples, and revert to the baseline when the adapted
        model has been worse `supervisor_patience` checks in a row.

        Call once per control step, right AFTER learn() and BEFORE pushing the
        weights to the solver, so a revert is picked up in the same iteration.
        Cheap: the comparison only runs every supervisor_every gradient steps.

        What this does and does not measure
        -----------------------------------
        Both models are evaluated on the same recent samples, which the online
        model has very likely already trained on. This is therefore NOT a
        generalisation estimate — it is a divergence detector: a model that
        fits the data it was just trained on WORSE than a model that never saw
        it is unambiguously broken.

        Returns
        -------
        True when a revert was performed this call.
        """
        if not self.supervisor.due(self._step_count):
            return False

        batch = dataset.recent_batch(self.supervisor.window)
        if batch is None:
            return False

        X, Y = self._to_tensors(batch)
        if self.supervisor.check(self.model, X, Y):
            self._revert()
            return True
        return False

    def _revert(self) -> None:
        """
        Restore the pre-trained weights, reset the optimizer/scheduler state and
        back off the learning rate.

        The Adam moment estimates describe the diverged trajectory, so they must
        go with the weights. Decaying the base learning rate turns a divergence
        into an automatic backoff rather than a revert/diverge cycle.
        """
        self.guard.restore_anchor()
        self._base_lr *= self.revert_lr_decay
        self.optimizer = torch.optim.Adam(self._trainable, lr=self._base_lr)
        self.scheduler = self._make_scheduler()   # warmup restarts, deliberately
        self.supervisor.reset()
        self._n_reverts += 1
        print(f"[OnlineTrainer] reverted (#{self._n_reverts}); lr is now {self._base_lr:.3g}")

    # ------------------------------------------------------------------
    # Per-trajectory reset
    # ------------------------------------------------------------------

    def reset_model(self, initial_state_dict: dict = None) -> None:
        """
        Reload the pre-trained weights and reset the optimizer, scheduler, safety
        state and all step counters for a fresh training run.

        Call at the start of each new trajectory so the online learner always
        starts from the same pre-trained baseline regardless of what happened in
        the previous trajectory. Note that the default pipeline in
        trajectory_tracking_and_record.py deliberately does NOT call this:
        learning is continuous across trajectories.

        Parameters
        ----------
        initial_state_dict : dict | None
            PyTorch state dict to restore. None (default) restores the anchor
            weights captured at construction, which is what the safety layer
            treats as the baseline — pass a different one only if you really
            intend the model to leave that baseline.
        """
        if initial_state_dict is None:
            self.guard.restore_anchor()
        else:
            self.model.load_state_dict(copy.deepcopy(initial_state_dict))
            for param in self.model.parameters():
                param.requires_grad_(True)
            self._apply_layer_freezing()
            # load_state_dict copies in place, so the parameter objects survive
            # and self._trainable / self._anchor_list stay valid. The anchor
            # itself is intentionally NOT moved: the safety layer keeps
            # measuring against the weights that were validated offline.

        self.optimizer = torch.optim.Adam(self._trainable, lr=self._base_lr)
        self.scheduler = self._make_scheduler()
        self.guard.resync()          # the reset is intentional, do not rate-limit it
        self._step_count = 0
        self._sim_steps = 0
        self._reset_counters()

    # ------------------------------------------------------------------
    # Weight export
    # ------------------------------------------------------------------

    def transfer_model(self, path: str = None) -> dict:
        """
        Export the current model weights.

        If path is provided the state dict is saved to disk as a .pt file.
        The state dict is always returned for in-memory use.

        Parameters
        ----------
        path : str | None   e.g. "updated_weights.pt"

        Returns
        -------
        state_dict : dict   PyTorch state dict of the updated model
        """
        state_dict = self.model.state_dict()
        if path is not None:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            torch.save(state_dict, path)
            print(f"OnlineTrainer: saved model weights to '{path}' "
                  f"(after {self._step_count} training steps)")
        return state_dict

    # ------------------------------------------------------------------
    # Inspection
    # ------------------------------------------------------------------

    def stats(self) -> dict:
        """How often each guard fired, plus the final distance to the baseline."""
        s = dict(
            scheme="sgd",
            steps=self._step_count,
            skipped_nonfinite=self._n_skipped_nonfinite,
            reverts=self._n_reverts,
            lr=self._base_lr,
            last_loss=self._last_loss,
        )
        s.update(self.guard.stats())
        s.update(self.supervisor.stats())
        return s

    def report(self) -> None:
        """
        Print the safety-layer summary.

        Read it: a guard that fires on a large fraction of the steps is telling
        you the learning rate is wrong, not that the guard is doing its job.
        """
        report_adaptation(self.stats(), "Online training summary (SGD)",
                          extra=[("final lr", f"{self.stats()['lr']:.3g}")])

    @property
    def step_count(self) -> int:
        """Total number of gradient steps performed so far."""
        return self._step_count

    @property
    def sim_steps(self) -> int:
        """Total number of simulation steps counted by should_train()."""
        return self._sim_steps

    @property
    def input_dim(self) -> int:
        """Expected NN input dimension: window_size * (len(state_feats) + len(u_feats))."""
        return self.window_size * (len(self.state_feats) + len(self.u_feats))
