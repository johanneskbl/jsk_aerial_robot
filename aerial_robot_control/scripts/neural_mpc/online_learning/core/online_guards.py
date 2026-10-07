"""
Safety layer for online adaptation.

Kept in one place on purpose: the whole point of these guards is that they can
be trusted, and two drifting copies of a bound are worse than none. Both
OnlineTrainer holds a WeightGuard and a BaselineSupervisor
and get identical behaviour and identical reporting from them.

The guards operate on torch parameters, which are the substrate both schemes
share: the optimizer updates them, and the guards below bound the
solution into them. Everything downstream (set_mlp_params -> acados) then reads
the same source of truth.
"""

import copy
import numpy as np
import torch


def get_flat(params) -> torch.Tensor:
    """Flatten parameters into a single detached 1-D tensor."""
    return torch.cat([p.detach().reshape(-1) for p in params])


@torch.no_grad()
def set_flat(params, vec: torch.Tensor) -> None:
    """
    Write a flat vector back into parameters.

    Copies into the existing storage (rather than rebinding .data the way
    torch.nn.utils.vector_to_parameters does) so the parameters keep owning
    their memory and any optimizer state stays valid.
    """
    i = 0
    for p in params:
        n = p.numel()
        p.copy_(vec[i:i + n].view_as(p))
        i += n


class WeightGuard:
    """
    Two hard bounds on what one adaptation step may do to the weights.

      1. per-step rate limit   ||W_k - W_{k-1}|| <= max_step_rel * ||W_0||
         The MPC runs SQP_RTI, i.e. ONE QP iteration per control step, which
         assumes the model changes slowly between iterations; a large weight
         jump invalidates the warm start. This enforces that assumption
         explicitly instead of hoping the step size is small enough.

      2. hard trust region     ||W   - W_0||     <= trust_region_rel * ||W_0||
         A ball around the pre-trained weights. Unlike a soft penalty (whose
         strength an adaptive optimizer rescales per parameter) this bound is
         unconditional: the online model can never wander arbitrarily far from
         a baseline that is known to fly.

    Both are relative to ||W_0|| so the same setting transfers across network
    sizes. Defaults are calibrated by measurement on neuralmodel_209 — see
    config/configurations.py.
    """

    def __init__(self, params, trust_region_rel: float = 3.0, max_step_rel: float = 0.1):
        self.params = list(params)
        if not self.params:
            raise ValueError("WeightGuard needs at least one parameter to guard.")
        self.anchor_flat = get_flat(self.params).clone()
        self.anchor_norm = float(torch.linalg.vector_norm(self.anchor_flat))
        if self.anchor_norm == 0.0:
            raise ValueError(
                "Pre-trained weights have zero norm — cannot scale the safety bounds."
            )
        self.trust_radius = max(float(trust_region_rel), 0.0) * self.anchor_norm
        self.max_step = max(float(max_step_rel), 0.0) * self.anchor_norm
        self.prev_flat = self.anchor_flat.clone()
        self.n_rate_limited = 0
        self.n_trust_clipped = 0
        self.last_dist = 0.0

    @torch.no_grad()
    def project(self) -> None:
        """
        Apply both bounds to the current parameter values, in place.

        Order matters and is safe: the rate limit is relative to the previous
        accepted point, then the trust-region projection follows. Euclidean
        projection onto a ball is non-expansive and prev_flat is always inside
        that ball, so the second step can only SHORTEN the move — the rate limit
        still holds afterwards.
        """
        flat = get_flat(self.params)

        if self.max_step > 0.0:
            delta = flat - self.prev_flat
            n = float(torch.linalg.vector_norm(delta))
            if n > self.max_step:
                flat = self.prev_flat + delta * (self.max_step / n)
                self.n_rate_limited += 1

        if self.trust_radius > 0.0:
            dev = flat - self.anchor_flat
            n = float(torch.linalg.vector_norm(dev))
            if n > self.trust_radius:
                flat = self.anchor_flat + dev * (self.trust_radius / n)
                self.n_trust_clipped += 1

        set_flat(self.params, flat)
        self.prev_flat = flat.clone()
        self.last_dist = float(torch.linalg.vector_norm(flat - self.anchor_flat))

    @torch.no_grad()
    def restore_anchor(self) -> None:
        """Put the pre-trained weights back and forget the step history."""
        set_flat(self.params, self.anchor_flat)
        self.prev_flat = self.anchor_flat.clone()
        self.last_dist = 0.0

    @torch.no_grad()
    def resync(self) -> None:
        """
        Accept the current parameter values as the new 'previous' point without
        projecting. Use after an intentional external write (e.g. a reset), so
        the rate limit does not fire on a change it should not police.
        """
        self.prev_flat = get_flat(self.params).clone()
        self.last_dist = float(
            torch.linalg.vector_norm(self.prev_flat - self.anchor_flat)
        )

    def stats(self) -> dict:
        return dict(
            rate_limited=self.n_rate_limited,
            trust_clipped=self.n_trust_clipped,
            anchor_dist=self.last_dist,
            anchor_dist_rel=self.last_dist / self.anchor_norm,
            trust_radius_rel=(self.trust_radius / self.anchor_norm
                              if self.anchor_norm else float("nan")),
        )


class BaselineSupervisor:
    """
    Divergence detector: compares the adapted model against a frozen copy of the
    pre-trained model on the most recent matured samples, and asks for a revert
    when the adapted model has been worse `patience` checks in a row.

    What this does and does not measure
    -----------------------------------
    Both models are evaluated on the same recent samples, which the adapted
    model has very likely already been fitted on. This is therefore NOT a
    generalisation estimate — it is a divergence detector: a model that fits the
    data it was just trained on WORSE than a model that never saw it is
    unambiguously broken.
    """

    def __init__(self, model, loss_fn, every: int = 50, window: int = 256,
                 tol: float = 1.0, patience: int = 3):
        self.baseline_model = copy.deepcopy(model)
        self.baseline_model.eval()
        for p in self.baseline_model.parameters():
            p.requires_grad_(False)
        self.loss_fn = loss_fn
        self.every = int(every)
        self.window = int(window)
        self.tol = float(tol)
        self.patience = int(patience)
        self.n_checks = 0
        self.n_fails = 0
        self.strikes = 0
        self.last = None          # (mse_adapted, mse_baseline)

    def due(self, step_count: int) -> bool:
        return (self.every > 0 and step_count > 0
                and (step_count % self.every) == 0)

    def check(self, model, X: torch.Tensor, Y: torch.Tensor) -> bool:
        """
        Run one comparison. Returns True when a revert is warranted.

        The caller supplies already-featurised tensors so this class stays
        agnostic of how each scheme builds its inputs.
        """
        was_training = model.training
        model.eval()   # deterministic: no dropout on either side
        with torch.no_grad():
            mse_adapted = float(self.loss_fn(model(X), Y))
            mse_base = float(self.loss_fn(self.baseline_model(X), Y))
        if was_training:
            model.train()

        self.n_checks += 1
        self.last = (mse_adapted, mse_base)

        failed = (not np.isfinite(mse_adapted)) or (mse_adapted > self.tol * mse_base)
        if failed:
            self.n_fails += 1
            self.strikes += 1
        else:
            self.strikes = 0

        if self.strikes >= self.patience:
            print(f"[supervisor] adapted model worse than the frozen baseline "
                  f"{self.strikes}x in a row (mse {mse_adapted:.4g} vs "
                  f"{mse_base:.4g}) — reverting to the pre-trained weights.")
            return True
        return False

    def reset(self) -> None:
        self.strikes = 0

    def stats(self) -> dict:
        return dict(supervisor_checks=self.n_checks, supervisor_fails=self.n_fails)


def report_adaptation(s: dict, title: str, extra=None) -> None:
    """
    Print one adaptation scheme's end-of-run summary.

    One line per run, so configurations are comparable at a glance.
    """
    n = s.get("steps", 0)
    if n == 0:
        print(f"[{s.get('scheme', 'online')}] no adaptation step was performed.")
        return
    print("\n" + "-" * 66)
    print(title)
    print("-" * 66)
    print(f"  adaptation steps          {n}")
    for label, value in (extra or []):
        print(f"  {label:<25} {value}")
    print(f"  last data MSE             {s['last_loss']:.6g}")
    print(f"  ||W - W_0|| / ||W_0||     {s['anchor_dist_rel']:.3f}  "
          f"(trust region {s['trust_radius_rel']:.3f})")
    print(f"  updates dropped (NaN/Inf) {s['skipped_nonfinite']:>6}  "
          f"({100.0 * s['skipped_nonfinite'] / n:.1f}%)")
    print(f"  steps rate-limited        {s['rate_limited']:>6}  "
          f"({100.0 * s['rate_limited'] / n:.1f}%)")
    print(f"  steps trust-clipped       {s['trust_clipped']:>6}  "
          f"({100.0 * s['trust_clipped'] / n:.1f}%)")
    print(f"  supervisor checks/fails   {s['supervisor_checks']:>6} / {s['supervisor_fails']}")
    print(f"  reverts to baseline       {s['reverts']:>6}")
    print("-" * 66 + "\n")
