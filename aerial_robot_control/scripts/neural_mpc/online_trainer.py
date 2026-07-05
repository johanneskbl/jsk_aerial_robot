import copy
import os
import numpy as np
import torch
import torch.nn as nn
from torch.optim.lr_scheduler import LambdaLR

from online_data import OnlineDataset


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

    MPC coupling note
    -----------------
    When the MPC solver was built with linearize_mlp=False (direct ca_forward()
    embedding), the running solver is NOT updated by in-place weight changes —
    its CasADi graph baked the weights as constants at build time.
    Use transfer_model() to persist the updated weights; solver rebuild with
    the new weights is handled in a later phase (Week 3).

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
        Weight of the L2 regularization term that pulls the online weights
        toward the pre-trained baseline captured at construction time.
            loss = MSE(pred, target) + lambda_anchor * ||W - W_init||²
        When no disturbance is present the MSE gradient vanishes and the
        anchor term drives the weights back toward the offline baseline,
        producing natural forgetting without a hard reset.
        Set to 0.0 (default) to disable.
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

        # Map y_reg_dims to column positions in Y (which is indexed by state_indices)
        for dim in self.y_reg_dims:
            if dim not in self.state_indices:
                raise ValueError(
                    f"y_reg_dim {dim} not found in state_indices. "
                    "Create OnlineDataset with state_indices=list(range(nx))."
                )
        self._y_col = np.array([self.state_indices.index(d) for d in self.y_reg_dims])

        # load_model() freezes all parameters (requires_grad=False) for inference.
        # Re-enable gradients so that loss.backward() + optimizer.step() actually
        # update the weights during online training.
        for param in self.model.parameters():
            param.requires_grad_(True)

        # Freeze selected initial layers to preserve offline-learned features
        self._apply_layer_freezing()

        # Capture pre-trained weights as the L2 anchor baseline.
        # Only trainable parameters are stored; frozen layers are excluded
        # because they never receive gradient updates.
        self._anchor_params = {
            name: p.clone().detach()
            for name, p in self.model.named_parameters()
            if p.requires_grad
        }

        # Build optimizer with only trainable (unfrozen) parameters
        trainable = [p for p in self.model.parameters() if p.requires_grad]
        self.optimizer = torch.optim.Adam(trainable, lr=lr)
        self.loss_fn = nn.MSELoss()

        # LR warmup scheduler
        self.scheduler = self._make_scheduler()

        self._step_count = 0
        self._sim_steps = 0

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

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
        Perform one gradient update step.

        Feature extraction (mirrors the MPC model build):
          For each time step w in [0, window_size):
            offset = w * (nx + nu)
            state_part = X[:, offset + state_feats]
            u_part     = X[:, offset + nx + u_feats]
          X_feat = concat of all (state_part, u_part) across the window
          Y_feat = Y[:, y_reg_dims]

        Input/output transforms:
          If the model stores x_mean/x_std (set during offline training),
          X_feat is normalised before the forward pass to match the CasADi
          evaluation path.  Similarly Y_feat is normalised if y_mean/y_std
          are present.  When both transforms are disabled (the default) the
          statistics are 0/1 and the normalisation is a no-op.

        Parameters
        ----------
        data : (X_raw, Y_raw) returned by get_data()
              X_raw shape: (batch, window_size * (nx + nu))
              Y_raw shape: (batch, len(state_indices))

        Returns
        -------
        loss : float  MSE loss value for this step
        """
        X_raw, Y_raw = data

        # --- Feature extraction with sliding-window support ---
        feat_parts = []
        for w in range(self.window_size):
            offset = w * (self.nx + self.nu)
            state_part = X_raw[:, offset + np.array(self.state_feats)]         # (batch, n_sf)
            u_part     = X_raw[:, offset + self.nx + np.array(self.u_feats)]   # (batch, n_uf)
            feat_parts.append(np.concatenate([state_part, u_part], axis=1))

        X_feat = np.concatenate(feat_parts, axis=1).astype(np.float32)         # (batch, W*(n_sf+n_uf))
        Y_feat = Y_raw[:, self._y_col].astype(np.float32)                      # (batch, n_y)

        # --- Input / output normalisation (mirrors the CasADi forward pass) ---
        if getattr(self.model, "x_mean", None) is not None:
            x_mean = np.tile(self.model.x_mean.detach().cpu().numpy().flatten(), self.window_size)
            x_std  = np.tile(self.model.x_std.detach().cpu().numpy().flatten(),  self.window_size)
            X_feat = (X_feat - x_mean) / (x_std + 1e-8)

        if getattr(self.model, "y_mean", None) is not None:
            y_mean = self.model.y_mean.detach().cpu().numpy().flatten()
            y_std  = self.model.y_std.detach().cpu().numpy().flatten()
            Y_feat = (Y_feat - y_mean) / (y_std + 1e-8)

        # --- Convert to tensors and update ---
        X_tensor = torch.tensor(X_feat, device=self.device)
        Y_tensor = torch.tensor(Y_feat, device=self.device)

        self.model.train()
        self.optimizer.zero_grad()
        Y_pred = self.model(X_tensor)
        loss = self.loss_fn(Y_pred, Y_tensor)

        if self.lambda_anchor > 0:
            anchor_loss = sum(
                ((p - self._anchor_params[name]) ** 2).sum()
                for name, p in self.model.named_parameters()
                if p.requires_grad and name in self._anchor_params
            )
            loss = loss + self.lambda_anchor * anchor_loss

        loss.backward()

        if self.grad_clip_norm > 0:
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), self.grad_clip_norm)

        self.optimizer.step()
        self.scheduler.step()

        self._step_count += 1
        return loss.item()

    # ------------------------------------------------------------------
    # Per-trajectory reset
    # ------------------------------------------------------------------

    def reset_model(self, initial_state_dict: dict) -> None:
        """
        Reload initial weights and reset the optimizer, scheduler, and all
        step counters for a fresh training run.

        Call at the start of each new trajectory so the online learner always
        starts from the same pre-trained baseline regardless of what happened
        in the previous trajectory.

        Parameters
        ----------
        initial_state_dict : dict
            PyTorch state dict captured before the first trajectory
            (e.g. copy.deepcopy(model.state_dict())).
        """
        self.model.load_state_dict(copy.deepcopy(initial_state_dict))
        for param in self.model.parameters():
            param.requires_grad_(True)
        self._apply_layer_freezing()
        lr = self.optimizer.defaults["lr"]
        trainable = [p for p in self.model.parameters() if p.requires_grad]
        self.optimizer = torch.optim.Adam(trainable, lr=lr)
        self.scheduler = self._make_scheduler()
        self._step_count = 0
        self._sim_steps = 0

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
