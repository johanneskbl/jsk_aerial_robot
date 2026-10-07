import os
from datetime import datetime
import numpy as np


# ----------------------------------------------------------------------
# Time-profile builders for TIME-VARYING disturbance magnitudes.
# Each returns a callable f(t) -> value, with t = simulation time [s].
# Use them (or any lambda) as a disturbance magnitude in sim_options,
# e.g.  "wind_x": sine(amp=2.0, period=20.0).
# A plain number still works and stays constant (backward-compatible).
# ----------------------------------------------------------------------
def const(value):
    """Constant value (same as passing the number directly)."""
    return lambda t: value


def step(t_on, value, before=0.0):
    """`before` until t_on, then `value` (e.g. a payload dropped at t_on)."""
    return lambda t: value if t >= t_on else before


def pulse(t_on, t_off, value, outside=0.0):
    """`value` while t_on <= t < t_off, else `outside` (a transient gust/load)."""
    return lambda t: value if t_on <= t < t_off else outside


def ramp(t0, t1, v0, v1):
    """Linear ramp from v0 (at t0) to v1 (at t1), clamped outside [t0, t1]."""
    def _f(t):
        if t <= t0:
            return v0
        if t >= t1:
            return v1
        return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return _f


def sine(amp, period, offset=0.0, phase=0.0):
    """offset + amp * sin(2*pi*t/period + phase) (e.g. an oscillating wind)."""
    w = 2.0 * np.pi / period
    return lambda t: offset + amp * np.sin(w * t + phase)


class DirectoryConfig:
    """
    Class for storing directories within the package.
    """

    _dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    SAVE_DIR = _dir_path + "/results/model_fitting"
    RESULTS_DIR = _dir_path + "/results"
    # Everything the ONLINE adaptation produces lives inside its own package, so
    # the package is self-contained and can be read (or handed over) on its own.
    # The trained networks stay in SAVE_DIR above: they are shared with the
    # offline neural MPC and are not an online-learning artefact.
    ONLINE_RESULTS_DIR = _dir_path + "/online_learning/results"
    SIMULATION_DIR = _dir_path + "/sim_plots/" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    CONFIG_DIR = _dir_path + "/config"
    DATA_DIR = _dir_path + "/data"


class EnvConfig:
    """
    Class for storing the model, solver, simulator and environment configurations.
    """

    # MPC options
    model_options = {
        "model_name": "",
        "arch_type": "tilt_qd",  # or "fix_qd" or "tilt_bi" or "tilt_tri"
        "mpc_type": "NMPCTiltQdServo",
        #    NMPCFixQdAngvelOut
        #    NMPCFixQdThrustOut
        #    NMPCTiltQdNoServo
        #    NMPCTiltQdServo
        #    NMPCTiltQdServoDist
        #    NMPCTiltQdServoImpedance
        #    NMPCTiltQdServoThrustDist
        #    NMPCTiltQdServoThrustImpedance
        #    NMPCTiltTriServo
        #    NMPCTiltBiServo
        #    NMPCTiltBi2OrdServo
    }

    # MLP options
    model_options.update(
        {
            "only_use_nominal": False,
            "neural_model_name": "residual_mlp",  # "residual_mlp" or "residual_vae" or "delayed_residual_mlp" or "temporal_residual_mlp"
            "neural_model_instance": "neuralmodel_209",  # 185, 161, 129, 120, 113, 90, 88, 87, 63, 58, 60, 29, 31, 35
            "online_neural_mpc": False,  # Whether to train the neural model online
            # "neural_model_name": "residual_vae",
            # "neural_model_instance": "neuralmodel_009",
            # ---- all before dont have standalone solver ----
            # 62: trained on residual_06 (first on standalone controller) (with 0.1 dist) (vx,vy,vz, no transform) -> good results
            # 63: trained on residual_neural_sim_nominal_control_03 -> WITH standalone SOLVER BUILDING
            # 64: small network trained on real hovering & ground effect data -> for CONTROLLER
            # 65: large network trained on real hovering & ground effect data -> for SIMULATOR
            # 66: really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 67: A LOT OF OVERFITTING really, really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 68: BN & L1 REGULARIZATION really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 71: (good) L1 REGULARIZATION really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 72: L2 REGULARIZATION really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 73: (VERY GOOD) L1 REGULARIZATION small network trained on simulated labels with 72 -> for CONTROLLER
            # 74: same as 73 but only 50 epochs to avoid oscillations
            # 75: same as 74 but without regularization
            # ---- all before have high oscillations ----
            # 76: (only 50 epochs) small network trained on real hovering & ground effect data -> for CONTROLLER
            # 79: (only 50 epochs) very large network trained on real hovering & ground effect data -> for SIMULATOR
            # 80: (only 25 epochs) large network trained on real hovering & ground effect data -> for SIMULATOR
            # 81: (only 75 epochs, lower lr) large network trained on real hovering & ground effect data -> for SIMULATOR
            # 82: (same as 81 but full state input and input & label transform) large network trained on real hovering & ground effect data -> for SIMULATOR
            # 83: small network trained on simulated labels with 82 -> for CONTROLLER
            # 84: same as 83 but one more layer
            # 85: same as 84
            # 86: same as 85 but without L1 regularization
            # 87: same as 86 but with angular velocities in input and 100 epochs (instead of 50) -> GOOD!
            # ---- all before have no moving average filter ----
            # 88: [GOOD!] WITH LABELS & INPUT DATA FILTERED, trained on GROUND_EFFECT_ONLY, middle size, normal settings -> for CONTROLLER
            # 89: (NO DIFFERENCE to 88) same as 88 but large size -> for SIMULATOR
            # 90: [GOOD!] same as 88 but large size and on TRAIN_FOR_PAPER -> for SIMULATOR
            # 92: Only z and cmd inputs (w/o transforms), on GROUND_EFFECT_ONLY -> for CONTROLLER
            # 93: Only z and cmd inputs (w/o transforms), on TRAIN_FOR_PAPER -> for CONTROLLER
            # ---- all before have no low pass filter ----
            # 96: [very good] Only z and avg cmd inputs (w/o transforms) & az as label, with LPF (1.0 ctf) on TRAIN_FOR_PAPER -> for CONTROLLER
            # 97: [a bit too good] Regular state in, avg input and large network (w/o transforms), with LPF (1.0 ctf) on TRAIN_FOR_PAPER -> for SIMULATOR
            # 98: Only z and avg cmd inputs (w/o transforms) but full labels, with LPF (0.1 ctf) on TRAIN_FOR_PAPER -> for CONTROLLER
            # [BAD LEARNING] 99: Same as 98 but without control averaging and with homogenous weight and weight decay (L2) -> for CONTROLLER
            # 100: Same as 99 but without weight decay -> for CONTROLLER
            # 101: Same as 97 but with grad penalty and consistency regularization -> for SIMULATOR
            # 102: Same as 101 but without LPF but with moving average
            # 103: Same as 96 but without extra losses and WITH L2 regularization
            # 104, 105, 106, 107: Same as 103 but slightly larger L2 factor
            # 108 (good), 109, 110 (VERY GOOD): Same as 107 but with consistency loss
            # 113 (BEST SO FAR), 114, 115 (Cant find solution when too high lambda): Same as 110 but with gradient penalty
            # 116 (bad): No mov avg (raw data), no grad pen, low consistency with L2
            # 117 (not as good as 113): Same as 116 but with mov avg
            # 118 (not as good as 113): Same as 113 but regular state
            # ---- all before not on FULL dataset ----
            # 119 (not good): Same as 113 but trained on FULL dataset
            # 120 (BEST SO FAR [similar to 113]): Same as 113 on fixed FULL dataset
            # 121: Same as 113 but on 13_RECORDINGS dataset
            # 124 (BETTER THAN 125): Same as 120 but with permutation symmetry loss (symmetry t1&t2, t3&t4, a1&a3, a2&a4)
            # 125 (worse than 124): Same as 120 but with permutation symmetry loss (symmetry t1&t3, t2&t4, a1&a3, a2&a4)
            # 127 (worse than 124): Same as 120 but with permutation symmetry loss (symmetry t1&t2, t3&t4, a1&a2, a3&a4)
            # 128 (worse than 124): Same as 120 but with permutation symmetry loss (symmetry t1&t3, t2&t4, a1&a2, a3&a4)
            # 129 (very simple functions due not much learning): Same as 120 but with permutation symmetry loss (and on TRAIN ds) (symmetry thrust change 50% which two to swap, a1&a3, a2&a4)
            # 133 (no learning): Same as 120 but with label transform ON TRAIN
            # ---- No symmetry loss from here ----
            # 134 (symmetry doesnt make sense! UNSTABLE EVEN IN SIM FOR CIRC TRAJ!): Same as 120 / with consistency loss and weight decay L2 on TRAIN (no symmetry loss, no grad loss) ON TRAIN
            # 135 (better learning behavior for normal L2 loss): Same as 134 but with lower consistency weight and way lower epsilon ON TRAIN
            # 136 (good learning but unstable flight EVEN IN SIM): Same as 135 but with FULL state input and no servo cmd (no transform) && only normal loss + zero-out regularization loss ON TRAIN
            # 137 (decent learning UNSTABLE IN SIM): Same as 136 (full state in) but with input and label transform & consistency + reg loss (& no moving average filter) ON FULL DATASET
            # 138 (decent learning): Same as 137 but with higher consist and reg losses
            # 140 (decent learning): Same as 138 but with higher consist epsilon
            # 141: Same as 140 but with grad loss and higher epsilon & zero-out lambda
            # 142: Same as 141 but without ang vel in input and servo only in cmd (not as state)
            # 143: Same as 142 but no reg loss and higher grad loss & consist
            # 144: Same as 143 but without vel in input (only label transform) and only with reg (no grad, no consist) and with seperate val dataset
            # 145: Same as 144 but with mov avg filter (33 window size) and on vx, vy, vz as labels(!!)
            # 146: Same as 145 but with revised propagation
            # 147: Same as 146 but average over propagation steps
            # 148: Same as 147 but with grad & consist loss
            # 149: Labels with MPC & T_step=0.01 (no revised out) (ReLU, 32 nodes)
            # 150 (val loss high): Labels with MPC & T_step=0.01
            # 151 (no learning): Same as 150 but with zero-out & consist & grad loss (bigger network)
            # 152 (decent learning): Same as 151 but only consist & zero-out loss & lower lambas (bigger network)
            # 153 (good learning): Same as 152 but with tiny network (16 nodes) and lower lr
            # 155 (best learning): Same as 153 but with MultiStepLR (experiment)
            # 159: On long prediction horizon propagation dataset (with mov avg on label)
            # 161 (decent learning, great closed-loop performance): Same as 159 but with larger num samples for consistency loss (& GELU)
            # 162 (overfitted but early learning looked good): Pure learning without additional losses on long pred labels
            # 163 (decent learning, decent cl results): Same as 162 but with less epochs (early stopping)
            # 164 (bad training but low gradient loss): Same setup as 120 but on long pred dataset (since it smoothed a lot)
            # 165 (same): Same as 164 but lower grad loss since bad learning
            # 166 (bad learning): Same as 165 but lower grad loss and with zero-out loss
            # 167 [GOOD] (decent learning, good out, good cl, good sim): Same as 166 but with lower extra losses
            # 168 (good learning, decent out, more noisy cl)
            # --- from here on complete recording dataset ---
            # 169 (Bad generalization): On new dataset with correct recording
            # 170 (decent generalization but almost no training 0.1->0.09): Same as 169 but with higher reg losses
            # 171 (divergent loss): Same as 169 but with tiny network (8 nodes, 1 layer) ON ONLY JOY
            # 172 (divergent loss): Same as 169 but with tiny network (8 nodes, 2 layers) ON ONLY JOY
            # --- all before don't have dropout ---
            # 175 (decent generalization!): Same as 169 but with dropout 0.5 & single layer ON ONLY JOY
            # 176 (const loss, high val): Same as 175 but only with zero out reg and on full train dataset
            # 177 (rising val loss): Same as 176 but with higher zero-out reg
            # 179 (GOOD CL!, const loss, low val): Dropout on Input
            # 180 (also good but slow learning): Same as 179 but with servo state as input and lower zero-out reg
            # 181 (slightly better learning and still const val): Same as 180 but without zero-out reg (only dropout for regularization) (and lambda LR)
            # 182 (same learning but worse val): Same as 181 but larger network (2 layers instead of 1))
            # 183 (significantly better training but slightly worse val): Same as 181 but more nodes, single layer (64 instead of 32x32))
            # 184 (exactly the same): Same as 183 but with 2 layers instead of 1
            # 185 (GOOD CL! good training, better val): Same as 183 but with zero out loss
            # 186 (GOOD CL! worse train, same val): Same as 185 but with 32 nodes instead of 64 nodes (single layer)
            # 187 (decent learning): FOR ROBOMECH PAPER Same as 185 but on dataset with simulated labels (nominal control, sim 185 model)
            # 188: Vanilla configuration without any additional losses (to observe difference when adding new energy reg loss)
            # 196 (low training loss, insane overfitting): Vanilla configuration with 32 nodes on 1 layer no extra losses or dropout
            # 197-202: Experiments if the energy regularization makes a difference. Lesson: Slightly improved training and val loss but not fixing overfitting.
            #                                                                               But sharper gradients in input output mapping.
            #                                                                               Dropout really negates overfitting but also negates learning - maybe 0.5 is overkill.
            # 203 (good learning and no overfitting): Same as 196 but with dropout 0.2
            # 204 (good learning and no overfitting): Same as 203 but with dropout 0.1 -> GOAL!
            # 205 (lower learning but strong overfitting): Same as 203 but with dropout 0.01
            # 206 (same: good learning and no overfitting): Same as 204 but with energy regularization of 1e3
            # 207 (Very high loss ~800): Same as 204 but with absolute (not relative) energy regularization penalty of  -> very high loss
            # 208 (decent learning, no overfitting but larger val loss; energy reg loss is const 0.008207!): Same as 207 but with energy reg 1e-5
            # 209 (same behavior just higher loss values as energy loss doesnt change!; energy reg loss is const 0.8207!): Same as 208 but with energy reg 1e-3
            # ---- all before with bad ref and old phys / from now on really good dataset (correct phys, correct ref, no failure, two full recordings for train and one full for val) ----
            # 211 (decent learning, no overfitting): Vanilla configuration with small network and dropout 0.1
            # 212 (no learning) : Same as 211 but with absolute energy regularization 1e-3
            # 213 (decent learning, no overfitting): Same as 211 but with absolute energy regularization 1e-5
            # 214 (decent learning, no overfitting): Same as 211 but with relative energy regularization 1e3
            
            # ---- VAE ----
            # 1 (bad learning & stepwise output): Use VAE! NOT LEARNING LOGVAR
            # 2 (no learning & very low KL Loss): Learning logvar
            # 3 (slightly better learning): Higher beta (1e4)
            # 6 (no learning): Beta = 0.1, Adam, ReLU
            # 8 (decent learning): Beta = 1e-3, Adam, ReLU
            # 9 (better learning, but same generalization, NOT VERY GOOD OUT PLOTS): Beta = 1e-4, AdamW, GELU
            # 10 (better learning, but same generalization): Beta = 1e-5
            # 11 (same learning but DOESNT MAKE SENSE): Beta = 0.0
            "linearize_mlp": False,  # Linearize MLP at each control step in the MPC using first or second order Taylor Expansion
            "linearize_order": 1,  # Order of Taylor Expansion (first or second)
            "use_l4casadi": False,  # Set order with "linearize_order"
            "use_gpu": False,  # Call neural model and its Jacobian & Hessian batched on GPU for MLP linearization (currently not set for L4casadi)
            # --- Safety: hard bound on the residual the NN injects into ds ---
            # Smoothly saturates the network output inside the CasADi graph:
            #   f(x) = x / (1 + (x/a)^6)^(1/6),  |f| <= a,  f ~= x for |x| << a.
            # Baked in at build time, so it applies to the nominal/static/online
            # controller alike and cannot be bypassed by a diverged update.
            # Sizing for this robot (m = 3.146 kg): the configured disturbances
            # need at most ~4 m/s2 of residual (payload 0.5 kg -> 1.56, ground
            # effect k=0.15 -> ~1.5, wind 3 N -> 0.95), while full thrust gives
            # ~38 m/s2. 10 m/s2 (~1 g) therefore leaves the useful range
            # untouched (0.07% shrink at 4 m/s2) and still caps a runaway well
            # below the control authority. Set to 0 to disable.
            "residual_sat": 10.0,  # [m/s^2]
            # --- Which MLP weights become acados parameters ---
            # "all"        : every layer. Required whenever the whole network
            #                adapts, which is the default.
            # "last_layer" : only the output layer — 99 parameters pushed to the
            #                solver every step instead of 1699, with the trunk
            #                baked into the CasADi graph as constants. Valid only
            #                when the trunk really is frozen, i.e. together with
            #                dataset_options["n_frozen_layers"] >= 1; otherwise
            #                the trunk would be trained and never flown. The
            #                check at the bottom of this file enforces that.
            "parametric_scope": "all",
        }
    )

    solver_options = {
        "cost_function_type": "NONLINEAR_LS",  # "NONLINEAR_LS" or "EXTERNAL"
        "solver_type": "PARTIAL_CONDENSING_HPIPM",  # TODO actually implement this
        "terminal_cost": True,  # TODO actually implement this
        "include_floor_bounds": False,
        "include_soft_constraints": True,
        "include_quaternion_constraint": False,
        "include_delta_u": False,
        "include_energy_cost": False,
        # --- Stage-0 thrust-rate penalty ---------------------------------
        # Adds  Rt_d * || ft_c(0) - ft_applied_previously ||^2  to the stage-0
        # cost only, through acados' native cost_y_expr_0 / W_0 / yref_0.
        #
        # Why: measured on a 45 s flight, online adaptation puts 46% of the
        # thrust command's energy above 2 Hz (nominal and static MLP: 0.4%),
        # with successive command increments anti-correlated at -0.51 — i.e. the
        # command flips direction almost every control step. The cause is that
        # the model is rewritten at 100 Hz while SQP_RTI performs a single QP
        # iteration per step, so the solution keeps jumping. This penalty acts
        # directly on the signal that chatters: the thrust actually applied.
        #
        # Only stage 0 is touched, so the planned trajectory over the horizon
        # keeps its usual cost. NONLINEAR_LS only (see OnlineNeuralMPC).
        "include_thrust_rate_cost": True,
        # The single knob, set by MEASUREMENT — a first guess of 100, sized to
        # make the rate term comparable to the existing thrust and servo-rate
        # terms, turned out to be 10x too large. Swept on a 45 s flight:
        #
        #   Rt_d      RMSE pos   chatter   corr lag1   rms dU
        #   (none)      0.2513     46.3%      -0.51     0.648
        #      10       0.2016      1.2%      +0.14     0.027   <-- best
        #     100       0.4337      0.9%      +0.51     0.029   over-damped
        #    1000       0.2709     16.3%      -0.27     0.133   worse again
        #   nominal     0.2312      0.4%      +0.11     0.016   (reference)
        #
        # At 10 the chatter is back to the nominal level, the tracking beats
        # both nominal and the static MLP, and adaptation is not slowed
        # (recovery after the payload step: 8.7 s vs 11.1 s without the term).
        # At 100 the command is over-damped and tracking collapses. Above that
        # the trend REVERSES — do not assume "more damping is safer".
        "Rt_d": 10.0,
    }



# ----------------------------------------------------------------------
# Online hyperparameters: where the values below come from.
#
# Run `search03` (results/tuning/search03/), 631 flights, 0 failures. 64
# candidates by Latin hypercube -> 16 -> 4, confirmed on 12 flights held out
# from the selection. Disturbances are drawn per flight and keyed by the seed,
# so a held-out flight is unseen in BOTH senses: trajectory and disturbance.
#
# The four finalists, on the 12 held-out flights (paired sign test vs nominal):
#
#   candidate           rmse     vs nominal   wins    rough   p99.9
#   #1 adopted        18.01 cm     -31.8%    12/12    2.3x    5.99 ms
#   #2                18.26 cm     -30.9%    12/12    2.2x    5.54 ms
#   #3                18.38 cm     -30.5%    12/12    2.3x    5.80 ms
#   #4                18.79 cm     -29.0%    12/12    2.3x    5.51 ms
#   previous config   19.12 cm     -27.6%    12/12    2.0x    5.60 ms
#   nominal           26.52 cm          -        -    1.0x    1.67 ms
#   frozen MLP        26.55 cm      +0.8%     6/12    1.1x    5.05 ms
#
# #1 beats the previous configuration on 12 flights out of 12 (p = 0.0002) but
# by only 5.9 %, and costs 15 % more command roughness (2.3x vs 2.0x nominal).
# That is a trade, not a free win — #2 is 1.5 % worse and slightly smoother. If
# smoothness matters more than the last centimetre, #2 is the defensible choice:
#   lr 1.017e-3, max_step_rel 0.03284, trust_region_rel 1.331,
#   lambda_anchor 3.020, train_every 1, forget_tau 6.30, buffer_size 630
#
# READ THE ATTRIBUTION TABLE BEFORE BUILDING A STORY ON THESE NUMBERS. Of the
# tracking error each disturbance costs nominal MPC, the adopted controller
# recovers 72 % of the wind's, 30 % of the ground effect's and 6 % of the drag's
# (3/4 flights, p = 0.31 — indistinguishable from chance). It is mostly learning
# a slowly varying force bias, not a state-dependent model.
# ----------------------------------------------------------------------


    dataset_options = {
        "ds_name_suffix": "dataset_neural_sim_nominal_control",
        # Online dataset parameters
        "buffer_size":     383,   # max (X, Y) pairs stored in the circular training buffer
                                  # = forget_tau / T_samp; the two must agree
        "min_samples":     256,    # minimum matured samples before training starts
        "batch_size":      64,     # mini-batch size for each gradient step
        "window_size":     1,      # sliding-window context length; 1 = single-step (no context)
        # Forgetting horizon of the recency-weighted sampling, IN SECONDS:
        # P(sample) ~ exp(-age / forget_tau).
        # Replaces the former "weighted_decay", which weighted by POSITION in the
        # buffer and therefore had no fixed horizon at all — it was nearly
        # uniform at 256 stored samples and concentrated on the last ~10 s once a
        # 10000-sample buffer had filled, so the effective horizon drifted during
        # the flight. NOTE: the buffer itself also forgets — at 100 Hz,
        # buffer_size=700 only holds 7 s, so forget_tau above that barely bites.
        "forget_tau":      3.828,   # [s]  (0 = uniform sampling)
        # ---------------- Adaptation: Adam on the residual network ----------
        # These values are NOT hand-tuned. They come from the hyperparameter
        # search in online_learning/tools/tune_online.py (run "search02", whose
        # full log is under results/tuning/search02/): 32 candidates drawn by
        # Latin hypercube over 6 dimensions, narrowed to 10, then to 3, and
        # confirmed on 8 flights that took no part in the selection.
        #
        # Measured on those 8 held-out flights, against the frozen offline model:
        #     tracking error   17.9 cm  vs  24.8 cm      (-28%)
        #     better on        8 flights out of 8        (p = 0.004)
        #     undisturbed      -22% — it does not degrade a calm flight
        #     command roughness 1.4x nominal MPC          (the previous default
        #                                                  was 8.9x, i.e. visibly
        #                                                  chattering)
        #     cost             4.6 ms/step of a 10 ms budget
        #
        # Three candidates survived confirmation and their tracking errors are
        # statistically INDISTINGUISHABLE (best pairwise test 6/8, p = 0.145), so
        # the choice was made on the two axes that do separate them: this one is
        # the smoothest and the cheapest. The other two are in the log if the
        # trade-off ever changes; do not re-derive them by hand.
        #
        # Re-run the search with:  python3 -m online_learning.tools.tune_online
        "lr":              2.416e-3,  # Adam learning rate
        "grad_clip_norm":  1,    # max gradient L2 norm (0 = disabled)
        "n_frozen_layers": 0,      # number of initial layers kept frozen during online training
        "warmup_steps":    40,      # linear LR warmup steps (0 = no warmup)
        "train_every":     2,     # gradient step every N control iterations
        # Decoupled (AdamW-style) pull toward the pre-trained weights, applied
        # AFTER the optimizer step:  W <- W - lr*lambda_anchor*(W - W_0).
        # lr*lambda_anchor is the fraction of the distance to the baseline
        # removed per step, so unlike the previous in-loss penalty (whose
        # strength Adam rescaled per parameter) this is a real forgetting rate.
        "lambda_anchor":   3.701,  # (0 = disabled)
        # ---------------- Safety layer (see OnlineTrainer) ----------------
        # Bounds on what online training may do to the weights. Sized on
        # neuralmodel_209 by measurement, not by guess: honest adaptation to the
        # configured disturbances moves the weights by 0.47-1.24 * ||W_0|| and
        # takes steps of at most 0.022 * ||W_0||, while an unguarded divergence
        # reaches ~250 * ||W_0||. Both defaults therefore sit far above what
        # learning needs and far below a runaway. Check the end-of-run summary:
        # a guard firing on many steps means the learning rate is wrong, not
        # that the guard is doing its job.
        "trust_region_rel":    1.895,  # ||W - W_0||     <= this * ||W_0||   (0 = off)
        "max_step_rel":        0.006362,  # ||W_k - W_k-1|| <= this * ||W_0|| (0 = off)
                                     # protects the SQP_RTI warm start, which
                                     # assumes the model changes slowly between
                                     # the single QP iterations it performs.
        "supervisor_every":     50,  # compare against the frozen baseline every N steps (0 = off)
        "supervisor_window":   256,  # newest matured samples used for that comparison
        "supervisor_tol":      1.0,  # fail when mse_online > tol * mse_baseline
        "supervisor_patience":   3,  # consecutive failures before reverting to W_0
        "revert_lr_decay":     0.5,  # lr multiplier on each revert (1.0 = keep lr)
    }
    sim_options = {
        # Choice of disturbances modeled in our Simplified Simulator
        "disturbances": {
            # --- NOT WIRED UP: enabling either of these changes nothing ---
            # apply_cog_disturbance() and apply_motor_noise() write into
            # sim_solver.acados_sim.parameter_values, which the compiled solver
            # never reads, and they run BEFORE the set("p", ...) that overwrites
            # the whole vector anyway. They are kept only so old configuration
            # files still load. Anything below this pair goes through
            # apply_cog_disturbances(), which does reach the integrator.
            "cog_dist": False,  # Disturbance forces and torques on CoG (INERT)
            "cog_dist_model": "mu = 1 / (abs(z)+1)**2 * cog_dist_factor * max_thrust * 4 | std = 0",
            "cog_dist_factor": 0.2,  # 0.1
            "motor_noise": False,  # Asymmetric rotor/servo noise (INERT)
            "payload": False,  # Payload force in the Z axis (superseded by extra_mass)
            # --- Step 3.1: fixed extra mass at CoG ---
            # Applies a constant downward force = extra_mass_kg * g (world z-up).
            # Cannot be combined with cog_dist (both write the same parameter slot).
            # CAN be combined with ground_effect (their CoG forces are summed).
            "extra_mass": True,  # enable constant payload mass disturbance
            "extra_mass_kg": step(t_on=30.0, value=0.5),  # payload appears at t = 30 s
            # --- ground effect: extra lift near the ground ---
            # Upward world-z force = T_vert * k / (1 + (z/z0)^2), where
            # T_vert = current total rotor thrust and z = height above ground.
            # Summed into the same CoG slot as extra_mass; exclusive with cog_dist.
            "ground_effect": True,     # enable ground-effect lift disturbance
            "ground_effect_k": 0.20,     # strength: extra lift fraction of thrust at z->0
            "ground_effect_z0": 0.5,    # characteristic height [m] (effect halves at z0)
            # --- wind: constant horizontal force (world frame), x and y only ---
            # F_x = wind_x, F_y = wind_y  [N]. No z component (kept simple).
            # Summed into the CoG slot; combines with extra_mass / ground_effect;
            # exclusive with cog_dist.
            "wind": True,     # enable wind force disturbance
            "wind_x": sine(amp=2.0, period=20.0),          # oscillating crosswind [N]
            "wind_y": ramp(t0=10.0, t1=40.0, v0=0.0, v1=3.0),  # wind builds up 10->40 s [N]
            # --- aerodynamic drag: 2nd-order polynomial in the velocity ---
            # F = -(k1 * v + k2 * ||v|| * v), v = world-frame velocity [m/s].
            # Each coefficient is a scalar (isotropic) or one value per axis;
            # both may also be callables f(t) like every magnitude above.
            #
            # Why it earns its place: extra_mass and wind are forces that depend
            # on TIME alone, so three estimated constants reproduce them exactly
            # and they cannot justify a network. Ground effect and drag are the
            # only STATE-dependent sources here — and drag is the one the MLP is
            # equipped to learn, since vx, vy, vz are among its 16 inputs
            # (state_feats below) while nothing angular is.
            #
            # Sizing is set by MEASUREMENT, and the first guess was wrong by a
            # factor of five. Three 120 s nominal flights (seeds 897/4242/31337)
            # give a ground speed of mean 0.29, p95 0.56, max 0.92 m/s — these
            # trajectories are slow. Textbook quadrotor coefficients (k1 ~ 0.3,
            # k2 ~ 0.1) then produce 0.06 m/s^2, which is 6 % of the wind and
            # would make drag invisible to the search: a disturbance nothing can
            # measure teaches nothing about the method.
            #
            # The values below give, on a 3.146 kg airframe:
            #     0.56 m/s (p95)  ->  1.35 N  =  0.43 m/s^2
            #     0.92 m/s (max)  ->  3.15 N  =  1.00 m/s^2
            # i.e. the same order as the wind (0.95 m/s^2) and the payload
            # (1.56 m/s^2) — a full participant, not the dominant term — and far
            # under the 10 m/s^2 residual saturation at any speed reachable here.
            # About two thirds of the force comes from the QUADRATIC term at p95,
            # which is the point: that part is nonlinear in the state and no
            # constant-force estimator can absorb it.
            #
            # Read them as a stand-in for the velocity-dependent aerodynamics a
            # rigid-body model omits — rotor drag, blade flapping, induced flow,
            # downwash recirculation — sized to matter at the speeds actually
            # flown, NOT as a calibrated fuselage drag model for this airframe.
            # z gets the larger coefficients because the rotor disc presents its
            # full area to vertical motion.
            "drag": True,                       # enable aerodynamic drag
            "drag_linear":    [0.8, 0.8, 1.2],  # k1 [N/(m/s)]
            "drag_quadratic": [3.0, 3.0, 4.5],  # k2 [N/(m/s)^2]
            # --- Time dependence (online learning shines on time-varying loads) ---
            # ANY magnitude above (extra_mass_kg, ground_effect_k/z0, wind_x/wind_y)
            # may be a constant OR a function f(t) -> value, t = sim time [s].
            # Use the Disturbance profile helpers, e.g.:
            #   "wind_x": sine(amp=2.0, period=20.0)      # oscillating crosswind
            #   "extra_mass_kg": step(t_on=30.0, value=0.5)   # payload appears at 30 s
            #   "wind_y": ramp(t0=10, t1=40, v0=0.0, v1=3.0)  # wind builds up
        },
        "use_nominal_simulator": True,  # Use nominal model as simulator
        "use_real_world_simulator": False,  # Use neural model trained on real world data as simulator
        "sim_neural_model_instance": "neuralmodel_185",  # 113, 90, 87, 58  # Used when use_real_world_simulator = True
        "max_sim_time": 120,
        "world_radius": 2,
        "seed": 897,
        "T_sim":     0.005,  # inner simulation step size (seconds)
        "T_takeoff": 5.0,    # duration of the takeoff phase (seconds)
    }

    # Run options
    run_options = {
        "real_machine": True,
    }

    # Trajectory tracking options
    run_options.update(
        {
            "trajectories": [
                # "line",
                # "circle",
                "helix",
                "lemniscate_I",
                "lemniscate_II",
                "roll",
                "pitch",
            ],  # "step", "hover", "takeoff", "smooth_takeoff", "circle", "helix", "lemniscate_I", "lemniscate_II", "roll", "pitch"
            "trajectory_length": 15.0,
        }
    )

    # Point tracking options
    run_options.update(
        {
            "preset_targets": None,
            "low_flight_targets": True,
            "initial_state": None,
            "initial_guess": None,
            "aggressive": False,
        }
    )

    # Recording options
    run_options.update(
        {
            "recording": False,
        }
    )

    # Visualization options
    run_options.update(
        {
            "plot_trajectory": True,
            "save_figures": False,
            "real_time_plot": False,
            "save_animation": False,
        }
    )

    # Neural model run options
    run_options.update(
        {
            "useMLP": True,     # Load and use the neural residual model
            "onlineMLP": True,  # Train the NN online during simulation (requires useMLP=True)
        }
    )

    if model_options["online_neural_mpc"] and model_options["only_use_nominal"]:
        raise ValueError("Conflict in options.")
    if model_options.get("parametric_scope", "all") == "last_layer":
        # Only the parametrised layers reach the solver, so any layer that is
        # BOTH trainable and unparametrised would be updated and then silently
        # ignored — the flown model would differ from the trained one. The
        # combination is legitimate exactly when the trunk is frozen, which is
        # also where it pays: 99 solver parameters instead of 1699.
        # This count was 2, commented "neuralmodel_209: one hidden + one
        # output". The network actually has THREE parameterised layers
        # (16->32->32->3), so the check demanded n_frozen_layers >= 1 where 2 is
        # needed: with 1, the second hidden layer stayed trainable while only the
        # output layer reached the solver — trained every step and never flown.
        # The real count is asserted against the loaded model in
        # OnlineNeuralMPC, which is the only place that knows it; this stays as
        # an early, cheap check.
        n_trainable_layers = 3      # neuralmodel_209: two hidden + one output
        if dataset_options.get("n_frozen_layers", 0) < n_trainable_layers - 1:
            raise ValueError(
                "parametric_scope='last_layer' requires the trunk to be frozen: "
                f"set dataset_options['n_frozen_layers'] >= {n_trainable_layers - 1}, "
                "or use parametric_scope='all'."
            )
    if model_options["linearize_mlp"] and model_options["use_l4casadi"]:
        raise ValueError("Conflict in options.")
    if (model_options["linearize_mlp"] or model_options["use_l4casadi"]) and model_options["linearize_order"] not in [1, 2]:
        raise ValueError("Only first and second order linearization supported.")
    if not (model_options["linearize_mlp"] or model_options["use_l4casadi"]) and model_options["use_gpu"]:
        raise ValueError("acados does not support GPU usage natively in optimization framework. GPU can only be used with linearization.")
    if sim_options["disturbances"]["extra_mass"] and sim_options["disturbances"]["cog_dist"]:
        raise ValueError("extra_mass and cog_dist both write the CoG parameter slot — enable only one at a time.")
    if sim_options["disturbances"].get("ground_effect", False) and sim_options["disturbances"]["cog_dist"]:
        raise ValueError("ground_effect and cog_dist both write the CoG parameter slot — enable only one at a time "
                         "(ground_effect may be combined with extra_mass, not with cog_dist).")
    if sim_options["disturbances"].get("wind", False) and sim_options["disturbances"]["cog_dist"]:
        raise ValueError("wind and cog_dist both write the CoG parameter slot — enable only one at a time "
                         "(wind may be combined with extra_mass / ground_effect, not with cog_dist).")
    if sim_options["use_real_world_simulator"] and sim_options["use_nominal_simulator"]:
        raise ValueError("Conflict in options.")
    if sim_options["use_real_world_simulator"]:
        for value in sim_options["disturbances"].values():
            if value == True:
                raise ValueError("Simulated disturbances not meaningful when using real world simulator.")
    if solver_options["cost_function_type"] == "NONLINEAR_LS" and solver_options["include_energy_cost"]:
        raise ValueError("NONLINEAR_LS cost function does not support energy cost as it is an additional term.")
    # if run_options["real_machine"]:
    #     for value in sim_options["disturbances"].values():
    #         if value == True:
    #             raise ValueError("No simulated disturbances allowed on real machine.")


class NetworkConfig:
    # ============================= MODEL SELECTION =============================
    # Choose between "MLP", "OMLP" or "VAE" for the neural network architecture
    model_type = "MLP"  # Options: "MLP", "VAE", "OMLP"
    
    # Define characteristics of the MLP model with its name
    if model_type == "MLP":
        model_name = "residual_mlp"
    elif model_type == "OMLP":
        model_name = "residual_omlp"
    elif model_type == "VAE":
        model_name = "residual_vae"
    else:
        raise ValueError(f"Unsupported model type: {model_type}")

    # Delay horizon for delayed input/output models (e.g., "delayed_residual_mlp")
    delay_horizon = 0  # Number of time steps into the past to consider (set to 0 to only use current state)
    if delay_horizon > 0:
        model_name = f"delay_{model_name}"

    # Predict entire horizon at once
    temporalize = True
    if temporalize:
        model_name = f"temporal_{model_name}"

    # Number of neurons in each hidden layer
    if model_type == "MLP" or model_type == "OMLP":
        # hidden_sizes = [8, 8]
        hidden_sizes = [32]
        # hidden_sizes = [64]
        # hidden_sizes = [32, 32]
        # hidden_sizes = [64, 64]
        # hidden_sizes = [128, 256, 128, 64]
    elif model_type == "VAE":
        encoder_hidden_sizes = [16, 8]
        latent_dim = 8  # Daumenregel: (input_dim + output_dim) / 2
        decoder_hidden_sizes = [8, 16]

        # KL divergence weight (beta in beta-VAE)
        # Higher beta encourages better disentanglement but may reduce reconstruction quality
        beta = 1e-4
        # Guidance:
            # beta = 0: No KL regularization (becomes autoencoder)
            # beta = 1: Standard VAE
            # beta > 1: Stronger disentanglement, may reduce reconstruction quality
            # beta < 1: Better reconstruction, less regularization

        # KL annealing: gradually increase beta during training
        use_kl_annealing = False
        kl_annealing_epochs = 10  # Number of epochs to anneal from 0 to full beta

    # Activation function
    activation = "GELU"  # Options: "ReLU", "LeakyReLU", "GELU", "Tanh", "Sigmoid"

    # Use batch normalization after each layer
    use_batch_norm = False

    # Use dropout after each layer
    dropout_p = 0.1  # To disable dropout, set to 0.0
    dropout_input = True

    # -----------------------------------------------------------------------------------------

    # Number of epochs
    num_epochs = 130

    # Batch size
    batch_size = 64

    # Loss weighting of different predicted dimensions (default ones-vector)
    # loss_weight = [1.0]
    loss_weight = [1.0, 1.0, 1.0]
    # loss_weight = [1.0, 1.0, 10.0]
    # Optimizer
    optimizer = "AdamW"  # Options: "Adam", "SGD", "RMSprop", "Adagrad", "AdamW"
    # Weight decay (L2 regularization)
    weight_decay = 1e-3  # Set to 0.0 to disable [1e-2 for AdamW, 1e-4 for Adam]
    # Zero-output regularization
    zero_out_lambda = 0.0 # 1e-2  # Set to 0.0 to disable
    # L1 regularization
    l1_lambda = 0.0  #1e-4  # Set to 0.0 to disable
    # Energy regularization
    energy_lambda = 0.0 #1e3  # for relative 1e3 / for absolute 1e-5  # Set to 0.0 to disable
    # Penalize gradients
    gradient_lambda = 0.0 #1e0  # Set to 0.0 to disable
    # Output consistency regularization epsilon
    consistency_lambda = 0.0 #1e-1 # 10.0  #1e4 # 5.0  # Set to 0.0 to disable
    consistency_epsilon = 0.3  # Relative noise to input; Set to 0.0 to disable
    consistency_num_samples = 1

    # Learning rate
    learning_rate = 1e-3  # for residual
    # learning_rate = 1e-2  # for residual
    # learning_rate = 1e-5  # for delayed
    # learning_rate = 1e-3  # for LR scheduling
    # lr_milestones = [50, 100] #[100, 150]
    lr_scheduler = "LambdaLR"  # "ReduceLROnPlateau", "LambdaLR", "MultiStepLR", "LRScheduler", None

    # Number of workers, i.e., number of threads for loading data
    num_workers = 0

    if weight_decay != 0.0 and l1_lambda != 0.0:
        raise ValueError("Don't use both L1 and L2 regularization at the same time.")
    if (zero_out_lambda != 0.0 and l1_lambda != 0.0) or \
       (zero_out_lambda != 0.0 and energy_lambda != 0.0) or \
       (energy_lambda != 0.0 and l1_lambda != 0.0):
        raise ValueError("Only one type of regularization should be used at a time.")


class ModelFitConfig:
    # ------- Propagation -------
    prop_long_horizon = True

    # ------- Low-Pass Filter -------
    use_low_pass_filter = False
    low_pass_filter_cutoff_input = 1.0
    low_pass_filter_cutoff_label = 0.1

    # ------- Moving Average Filter -------
    use_moving_average_filter = False
    use_moving_average_filter_only_label = False  # VERY IMPORTANT!
    control_filtering = False  # USE WAY SMALLER WINDOW SIZE IF TRUE!
    window_size = 5 #33  # Must be odd

    # ------- Coordinate Transform -------
    input_transform = False
    label_transform = False

    # ------- Pruning -------
    prune = False

    # Histogram pruning parameters
    histogram_n_bins = 10
    histogram_thresh = 0.001  # Remove bins where the total ratio of data is lower than threshold
    vel_cap = 16  # Remove datapoints where abs(velocity) > vel_cap

    # ------- Plotting -------
    plot_dataset = True
    save_plots = False

    # ------- Dataset loading -------
    train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_ENTIRE_HORIZON"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_ENTIRE_HORIZON_DEBUG"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_VAL_ENTIRE_HORIZON"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_ONLY_JOY"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_WITH_REF_ALL_PROP"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_FULL"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_FOR_PAPER"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_GROUND_EFFECT_ONLY"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "residual_dataset_neural_sim_nominal_control_03"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "residual_dataset_06"
    train_ds_instance = "dataset_001"  # "dataset_020"
    # real machine 01, dataset 007: Large dataset from many old flights with mode 0 and other discrepancies (200k datapoints)
    # real machine 01, dataset 007: Large dataset from mode 10 with focus on ground effect (66k datapoints)
    # real machine 01, dataset 013: same as 007 but with fixed prop and dt
    # residual 06, dataset 001 on neural standalone
    # residual neural sim nominal control 03, dataset 001: first with correct solver building
    # residual neural sim nominal control 05, dataset 001: on simulator as large network trained with L1 regularization
    # residual neural sim nominal control 07, dataset 001: on simulator as large network trained with full state and transforms and L1 regularization
    # real machine GROUND_EFFECT_ONLY: hovering and ground effect data only (48k datapoints)
    # val_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_VAL_FOR_PAPER"
    # val_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_VAL_WITH_REF_ALL_PROP"
    val_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_VAL"
    # val_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_VAL_ENTIRE_HORIZON"
    # val_ds_name = "NMPCTiltQdServo" + "_" + "residual_dataset_neural_sim_nominal_control_03"
    val_ds_instance = "dataset_001"
    # === FROM HERE WITH MOVING AVERAGE FILTER APPLIED ===
    # NMPCTiltQdServo_real_machine_dataset_GROUND_EFFECT_ONLY,  dataset_002
    # ------- Features used for the model -------
    # State features
    state_feats = [2]  # [z]
    state_feats.extend([3, 4, 5])  # [vx, vy, vz]
    state_feats.extend([6, 7, 8, 9])  # [qw, qx, qy, qz]
    # state_feats.extend([10, 11, 12])  # [roll_rate, pitch_rate, yaw_rate]
    state_feats.extend([13, 14, 15, 16])  # [servo_angle_1, servo_angle_2, servo_angle_3, servo_angle_4]
    # state_feats.extend([17, 18, 19, 20, 21, 22])  # [fds_1, fds_2, fds_3, tau_ds_1, tau_ds_2, tau_ds_3]
    # state_feats.extend([17, 18, 19, 20])  # [thrust_1, thrust_2, thrust_3, thrust_4]

    # Control input features
    u_feats = [0, 1, 2, 3]  # [thrust_cmd_1, thrust_cmd_2, thrust_cmd_3, thrust_cmd_4]
    # u_feats.extend([4, 5, 6, 7])  # [servo_angle_cmd_1, servo_angle_cmd_2, servo_angle_cmd_3, servo_angle_cmd_4]

    # Variables to be regressed
    y_reg_dims = []
    # y_reg_dims = [5]  # [az]
    # y_reg_dims.extend([0, 1, 2])  # [vx, vy, vz]
    y_reg_dims.extend([3, 4, 5])  # [ax, ay, az]
    # y_reg_dims.extend([6, 7, 8, 9])  # [qw_dot, qx_dot, qy_dot, qz_dot]
    # y_reg_dims.extend([10, 11, 12])  # [roll_acc, pitch_acc, yaw_acc]
