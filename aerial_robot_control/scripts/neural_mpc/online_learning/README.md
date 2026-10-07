# online_learning

Online adaptation of a residual model for a tiltrotor quadrotor NMPC.

A small MLP (16 → 32 → 32 → 3, 1699 weights) predicts the acceleration the nominal rigid-body model gets wrong.
That residual lives **inside the acados solver**, so the optimiser plans with it.
This package changes the network's weights **during the flight**, under a safety
layer, so the controller keeps correcting itself as the disturbances change.

Everything here runs against the same two baselines, selected by two flags to
`run_simulation(...)` in `../trajectory_tracking_and_record.py`:

| mode | flags | what corrects the dynamics |
|---|---|---|
| nominal | `useMLP=False` | nothing — rigid-body model only |
| frozen (offline) | `useMLP=True, onlineMLP=False` | the MLP, never updated |
| online | `useMLP=True, onlineMLP=True` | the MLP, adapted in flight |

---

## 1. The idea

A nominal MPC plans with a rigid-body model. Reality adds forces that model does
not contain — a payload, wind, ground effect, aerodynamic drag. The classical fix
is to estimate a constant force and subtract it. That works only for
disturbances that depend on time alone.

Instead, a network `f(x, u) -> (ax, ay, az)` is added to the model **inside the
solver**, so the optimiser plans against corrected dynamics rather than
correcting a plan afterwards. Training it offline is not enough: the offline
model is fixed at whatever the training flights contained. Online adaptation
keeps fitting it to the aircraft *now*.

The hard part is not the learning. It is doing it to a model that a real-time
optimiser is using, at 100 Hz, without destabilising the aircraft. Most of this
package is about that constraint.

---

## 2. How the weights change during flight

One pass of the control loop, in order. The code is in
`../trajectory_tracking_and_record.py` around the `online_trainer` block; each
step below names the class that owns it.

### 2.1 Collect one sample — `OnlineDataset.get_data()`

At control step `T` the solver has already produced its first predicted node,
`x̂(T + T_step)`. That prediction is stored in a **pending queue**, not used yet:
its ground truth does not exist for another `T_step` seconds.

`T_step / T_samp = 0.1 / 0.01 = 10` control steps later, the actual state arrives
and the sample matures:

```
Y = ( x_actual(T + T_step) − x̂(T + T_step) ) / T_step        # the label
X = [ state(T), u_cmd(T) ]                                    # the input
```

Two details that are easy to get wrong:

* **The division by `T_step` is not cosmetic.** It makes `Y` a rate, so it has
  acceleration units, which is the convention the offline training used. Drop it
  and the online model learns a quantity ten times larger than the one the
  solver expects.
* **`Y` is the error of the model the solver actually used**, so it is exactly
  what the residual has to supply. This is why the label needs the MPC's own
  prediction and not a separate integration.

Samples land in a circular buffer of `buffer_size` entries. At 100 Hz,
`buffer_size × T_samp` is the buffer's horizon in seconds — keep it consistent
with `forget_tau`, or the recency weighting below does nothing.

The buffer is **purged once** when take-off ends, so the model never trains on
take-off dynamics.

### 2.2 Decide whether to train — `OnlineTrainer.should_train()`

A gradient step runs only every `train_every` control steps, and only once the
buffer holds `min_samples` matured samples.

### 2.3 Draw a batch — `OnlineDataset.sample_batch(strategy="weighted")`

Sampling is **recency-weighted**: a sample of age `dt` is drawn with probability
proportional to `exp(−dt / forget_tau)`, with `forget_tau` in **seconds**. Age,
not position in the buffer — an earlier version weighted by index, so the
effective forgetting horizon silently drifted as the buffer filled.

### 2.4 One Adam step — `OnlineTrainer.learn()`

Plain MSE on the residual, one Adam step, plus three things layered on it:

* **warm-up** — the learning rate ramps linearly over `warmup_steps`, so the
  first steps after take-off cannot lurch.
* **frozen layers** — the first `n_frozen_layers` layers can be held fixed.
* **decoupled anchor** — after the optimiser step,
  `W ← W − lr·lambda_anchor·(W − W₀)`. `lr × lambda_anchor` is the fraction of
  the distance to the pre-trained weights removed per step, so it has a readable
  unit: a half-life. Decoupled (AdamW-style) rather than an `L2` term in the
  loss, because Adam's per-parameter scaling would make an in-loss penalty mean
  something different in every layer.

A non-finite loss or gradient never reaches the weights.

### 2.5 Bound the step — `WeightGuard.project()`

Two **hard** bounds, both relative to `‖W₀‖` so they transfer across network
sizes:

1. **Rate limit** — `‖W_k − W_{k−1}‖ ≤ max_step_rel · ‖W₀‖`.
   The MPC is SQP_RTI: **one QP iteration per control step**, which assumes the
   model changes slowly between iterations. A large weight jump invalidates the
   warm start. This enforces that assumption instead of hoping for it.
2. **Trust region** — `‖W − W₀‖ ≤ trust_region_rel · ‖W₀‖`.
   A ball around weights known to fly. Unconditional, unlike a soft penalty.

Both rescale the whole update rather than clipping element-wise. **This makes
`lr` and `max_step_rel` interact**: above some `lr` the guard rescales every
step, so the step *size* stops depending on `lr` at all and only its direction
still does. Sweeping `lr` alone therefore explores the wrong set — see §5.

### 2.6 Check it is still sane — `BaselineSupervisor.check()`

The guards of §2.5 bound *how far* and *how fast* the weights move. They cannot
tell whether the model is getting better or worse: a model can drift slowly,
legally, and still end up useless. The supervisor is the check on quality.

**What it compares.** It holds a frozen `deepcopy` of the pre-trained network,
made once at construction with `requires_grad_(False)`. Every
`supervisor_every` gradient steps it pulls the newest `supervisor_window`
matured samples — `OnlineDataset.recent_batch()`, the *most recent* ones, not a
weighted draw — and evaluates both networks on them:

```
mse_adapted  = loss(model(X), Y)
mse_baseline = loss(frozen_W0(X), Y)
failed       = mse_adapted > supervisor_tol * mse_baseline      # tol = 1.0
```

Both are run under `model.eval()`, so **dropout is off on both sides**. With
dropout active the comparison would be noise against a deterministic opponent,
and the adapted model would lose at random. The training mode is restored
afterwards.

**What this is, and what it is not.** The adapted model has almost certainly
already been trained on these very samples. So this is **not** a generalisation
estimate — it would be a badly biased one. It is a *divergence detector*, and
the bias is what makes it sharp: a model that fits the data it was just trained
on **worse** than a model that never saw that data is unambiguously broken.
There is no benign reading of that outcome. A non-finite loss counts as a
failure too.

**Why patience.** One bad check is not evidence: the window slides, the aircraft
enters a new manoeuvre, one comparison can go the wrong way. The supervisor
counts *consecutive* failures in `strikes`, and a single passing check resets
the counter to zero. Only `supervisor_patience` failures in a row trigger a
revert. Tuned at `every=50`, `patience=3`, that means roughly 150 gradient
steps of sustained degradation.

**What a revert does** — `OnlineTrainer._revert()`, four actions, and each of
them is necessary:

1. `guard.restore_anchor()` — the weights go back to `W₀`.
2. **The Adam optimiser is rebuilt from scratch.** Its moment estimates describe
   the trajectory that just diverged; carrying them over would push the restored
   weights straight back down the same path. They must go with the weights.
3. `base_lr *= revert_lr_decay` (0.5). This is what turns a revert into an
   automatic backoff rather than a revert/diverge/revert cycle: each failure
   halves how fast the next attempt can move.
4. The warm-up scheduler restarts, deliberately — the model is at `W₀` again and
   is in the same situation it was in at take-off.

**Ordering matters.** `supervise()` is called *after* `learn()` and *before*
`set_mlp_params()`, so a revert reaches the solver in the same control step that
detected it. Between the two calls the solver is still flying the previous
weights, so a diverged model is never pushed to it.

**Reading the end-of-run report.** `supervisor_checks` and `supervisor_fails`
are printed at the end of every flight. Occasional failures are normal.
Repeated *reverts* mean the learning rate is wrong — not that the supervisor is
doing its job well.

### 2.7 Practically: how weights change in a solver already compiled to C

This is the part that sounds impossible. acados generates C from a CasADi
expression graph, compiles it into a shared library, and that library is fixed
for the whole flight. Rebuilding it takes ~20 s, so it cannot happen at 100 Hz.
Yet the network's weights change every other control step.

**The trick: the weights are not in the C code. They are inputs to it.**

An acados model has a parameter vector `p`, meant for things that change between
solves without changing the problem's structure — a reference, a mass, a
measured disturbance. Nothing says `p` has to be small, or that it cannot be a
neural network's weights.

So when `model_options["online_neural_mpc"] = True`, the model is built with the
weights as **CasADi symbols** instead of numbers
(`online_neural_controller.py`, the parametric branch of `create_acados_model`):

```python
W_s = ca.MX.sym(f"W_{l_idx}", n_out, n_in)     # a symbol, not a value
b_s = ca.MX.sym(f"b_{l_idx}", n_out, 1)
parameters = ca.vertcat(parameters, W_s.reshape((-1, 1)), b_s)
...
h = ca.mtimes(W_s, h) + b_s                     # forward pass, symbolically
```

The generated C therefore contains the network's **structure** — the matrix
products, the GELU activations, the input and output normalisation, and all the
derivatives acados needs — with the weights left as slots in `p`. The compiled
library never changes. What is written into those slots does.

Three consequences worth stating plainly:

* **The residual is exactly affine in the last layer's weights** and polynomial
  in the others, so acados' own derivatives stay valid: it differentiates the
  symbolic graph once, at build time, with respect to states and controls.
  Changing `p` changes the numbers those derivatives evaluate to, not their
  form.
* **Normalisation constants and BatchNorm running statistics are baked in** as
  `ca.DM` constants, because they do not adapt. Only what adapts is symbolic.
* **The scope is a choice.** `parametric_scope="all"` exposes all three
  parameterised layers — 1699 values. `"last_layer"` exposes only the output
  layer — 99 values — and bakes the frozen trunk in as constants. It is valid
  only if the trunk really is frozen (§4.6), and it saves no computation at all
  (§6).

**The update path, per gradient step:**

```
PyTorch weights  --set_mlp_params()-->  neural_mpc.acados_parameters
                                                 |
    next MPC iteration:  for j in range(N+1):    |
                             ocp_solver.set(j, "p", acados_parameters[j, :])
                                                 v
                                        the running C solver
```

`set_mlp_params()` (in `../utils/model_utils.py`) flattens the current PyTorch
weights in the same order the symbols were concatenated, and writes them into
the slice `[mlp_weight_start_idx : mlp_weight_end_idx]` of `acados_parameters`.
It raises if the vector length does not match the slice the solver reserved —
which catches a scope mismatch between the build and the update.

`acados_parameters` has one row **per horizon node** (`N + 1 = 21`), and the
loop at the top of the next MPC iteration pushes every row. So all 21 nodes plan
with the same, newly updated network: the optimiser is not correcting the first
step with an old model and the rest with a newer one.

**Two things this design forces, both of which bite if ignored:**

* **acados keys its generated C by model name**, and the parameter vector's
  *length* is part of the compiled interface. Two builds with different lengths
  and the same name collide, and on a mismatch acados prints and calls
  `exit()` — it does not raise. That is why the model name carries a
  `_lastlayer` suffix, and why turning disturbances off (which removes 6 CoG
  parameters) cannot share a process with a disturbed run.
* **PyTorch is the source of truth, the solver is a copy.** Nothing keeps them
  in step except the `set_mlp_params()` call. `tests/test_casadi_sync.py`
  asserts that the CasADi model equals the PyTorch model before *and* after a
  training step — this is the core invariant of the whole scheme.

For the record, the alternative the codebase also supports:
`model_options["linearize_mlp"]` pushes only a Taylor expansion `(x₀, y₀, J₀)`
of the network instead of its weights. Far fewer parameters, and the solver
carries an affine function instead of an MLP. Untested here, and the most
promising lever for an embedded target (§6).

---

## 3. Layout

### core/ — what runs DURING the flight

**`online_data.py` → `OnlineDataset`** (534 lines)
The memory. Holds the pending queue that waits `T_step` for each prediction's
ground truth, builds the residual label when it matures, and stores `(X, Y)` in
a circular buffer. Also keeps `_rec`, a full recording of the flight used by the
figures and by `validate()`.
Read first: `get_data()` (one sample in), `sample_batch()` (a recency-weighted
batch out), `recent_batch()` (the newest N, for the supervisor).

**`online_trainer.py` → `OnlineTrainer`** (671 lines)
The adaptation law. Owns the Adam optimiser, the warm-up schedule, the frozen
layers, the decoupled anchor, and both guards.
Read first: `should_train()` (is a step due), `learn()` (the step itself, and
where the guards are applied), `supervise()` (§2.6), `_revert()`.
`stats()` and `report()` produce the end-of-flight summary — the fastest way to
tell a healthy run from a fighting one.

**`online_guards.py` → `WeightGuard`, `BaselineSupervisor`** (241 lines)
The safety layer, kept in one file on purpose: these bounds only work if they
can be trusted, and two drifting copies of a bound are worse than none.
`WeightGuard.project()` enforces the rate limit and the trust region;
`restore_anchor()` is the revert. `BaselineSupervisor.check()` is the divergence
detector. `get_flat`/`set_flat` move parameters to and from a flat vector while
keeping the optimiser state valid.

**`online_neural_controller.py` → `OnlineNeuralMPC`** (1418 lines)
The MPC that carries a *parametric* copy of the network inside its acados
solver — the piece that makes in-flight adaptation possible at all (§2.7).
Read first: the parametric branch of `create_acados_model()`, where the weights
become `ca.MX.sym` symbols. The rest is the standard NMPC formulation (cost,
constraints, reference generator) and is shared with the offline path.
**Careful:** `run_simulation` builds this class for all three modes, so a change
here moves the baselines too.

### tools/ — what runs ON THE GROUND

**`harness.py`** (744 lines) — *not run directly*
Everything about flying and measuring, shared by the three tools below: the
randomised disturbance scenarios (`apply_scenario`, `_source_rng`), the metrics
(`regime_metrics`, `deviation_metrics`, `chatter_metrics`), one flight in its own
process (`simulate`, `run_one`), and many in parallel with a resumable
checkpoint (`run_batch`). Also the worker entry point the parent spawns.

**`tune_online.py`** (739 lines) — searches the hyperparameters (§5).
The search space (`SPACE`), the Latin hypercube sampler, the stage plan
(`STAGES`), the constrained decision rule (`score`), and the report tables.

**`evaluate.py`** (233 lines) — **start here.** Measures configurations you
already have against nominal MPC, with paired sign tests. Edit `CONFIGS` at the
top; everything else is reporting.

**`figures.py`** (462 lines) — the presentation outputs: position and thrust
over time, bar charts, the parameter table, and a 3D replay GIF.

### tests/ — 5 suites, no acados, no simulation, seconds to run

See `tests/README.md` for what each one pins down. `test_casadi_sync.py` holds
the invariant everything depends on: the CasADi model in the solver equals the
PyTorch model, before and after training.

### results/ — everything this package produced (§7)

Outside the package, and deliberately so:

* `../neural_controller.py` — `NeuralMPC`, the **nominal and offline** path,
  imported by ~14 files. Not ours. Do not move or rename.
* `../trajectory_tracking_and_record.py` — flies all three modes, so the
  baselines and the adaptive runs stay comparable. A change here moves the
  baselines too.
* `../results/model_fitting/` — the trained networks, shared with the offline
  neural MPC.

---

## 4. Tutorial: using this package

Everything runs **as a module, from `neural_mpc/`**. Elsewhere the imports of
`config/`, `utils/` and `nmpc/` fail.

```bash
cd .../scripts/neural_mpc
```

### 4.1 Fly once and look at it

The simplest thing: change nothing, and measure the configuration currently in
`config/configurations.py` against the two baselines.

```bash
python3 -m online_learning.tools.evaluate            # ~10 min, 36 flights
python3 -m online_learning.tools.evaluate --report   # tables only, no flying
python3 -m online_learning.tools.evaluate --timing   # + per-step cost
```

It prints tracking error, worst departure, command roughness, and a **paired
sign test** against nominal MPC over 12 flights that no search ever used.

### 4.2 Try a variant

Edit `CONFIGS` at the top of `tools/evaluate.py`. Give only the keys that
differ — everything else stays at the configured value, so the difference is
attributable:

```python
CONFIGS = [
    ("nominal", dict(useMLP=False, onlineMLP=False), {}, {}),
    ("online",  dict(useMLP=True,  onlineMLP=True),  {}, {}),
    ("faster",  dict(useMLP=True,  onlineMLP=True),  dict(lr=5e-3), {}),
]
```

Then rerun. Results are checkpointed, so only the new rows are flown.

### 4.3 Search the hyperparameters

```bash
python3 -m online_learning.tools.tune_online              # ~3 h, 631 flights
python3 -m online_learning.tools.tune_online --report     # re-report only
python3 -m online_learning.tools.tune_online --clean-arenas
```

Protocol in §5. It resumes from its checkpoint if interrupted.

### 4.4 Draw the figures

```bash
python3 -m online_learning.tools.figures
```

Writes `results/figures/` (position, thrust, bar charts, parameter table) and a
GIF in `results/animations/`.

### 4.5 Run the tests

```bash
for t in online_learning/tests/test_*.py; do python3 "$t"; done
```

No acados, no simulation, seconds to run.

### 4.6 The parameters you will actually change

All in `config/configurations.py`, in `EnvConfig.dataset_options` unless noted.

**Adaptation speed and reach**

| parameter | what it does | current |
|---|---|---|
| `lr` | Adam learning rate | 2.416e-3 |
| `train_every` | gradient step every N control steps | 2 |
| `buffer_size` | samples kept; `× 0.01 s` = horizon | 383 |
| `forget_tau` | recency horizon **in seconds** | 3.828 |
| `min_samples` | matured samples before training starts | 256 |
| `batch_size` | mini-batch | 64 |

**Safety layer** — read §2.5 before touching these.

| parameter | what it does | current |
|---|---|---|
| `max_step_rel` | per-step bound, × `‖W₀‖` | 0.006362 |
| `trust_region_rel` | distance bound from `W₀`, × `‖W₀‖` | 1.895 |
| `lambda_anchor` | pull rate toward `W₀` (`lr × this` per step) | 3.701 |
| `supervisor_every` / `_window` / `_tol` / `_patience` | divergence check | 50 / 256 / 1.0 / 3 |
| `revert_lr_decay` | lr multiplier on each revert | 0.5 |

**Model** — in `EnvConfig.model_options`.

| parameter | what it does | current |
|---|---|---|
| `parametric_scope` | `"all"` (1699 weights) or `"last_layer"` (99, needs `n_frozen_layers >= 2`) | `"all"` |
| `residual_sat` | clips the residual, m/s² | 10.0 |
| `n_frozen_layers` | *(in `dataset_options`)* parameterised layers held fixed, counted from the input; the network has 3 | 0 |

**Disturbances** — in `EnvConfig.sim_options["disturbances"]`. See §8.

---

## 5. How results are produced

`tools/tune_online.py` searches six hyperparameters — `lr`, `max_step_rel`,
`trust_region_rel`, `lambda_anchor`, `train_every`, `data_horizon_s` — by
**Latin hypercube** (192 candidates; a grid on the same budget would afford
about two values per dimension), then narrows **192 → 40 → 8** by successive
halving, confirms the finalists on **24 flights held out from the selection**,
attributes their behaviour to individual disturbances, and finally measures the
per-step cost sequentially. About 2650 flights, ~11 h on 8 pinned workers.

Each candidate is flown on three scenarios at every selecting stage: `all` (the
randomised disturbance mixture, which is what the ranking minimises), `none`
(every injected disturbance off) and `drag_real` (nothing injected, but drag at
textbook magnitude — the physically honest "undisturbed" flight, since `none`
also removes aerodynamics a real aircraft always has). Both clean scenarios are
constraints; the worse of the two binds.

**Stability is the primary criterion, precision the secondary one.** That is not
a sort by roughness — the smoothest controller is the one that barely adapts, so
sorting would elect something close to nominal. Stability is a bound
(`CHATTER_TOL_X_NOMINAL`, 1.5x nominal on the same flights) and error only ranks
whoever clears it. `print_pareto()` lists the whole trade and marks the
Pareto-optimal candidates, so a rejected-but-far-more-accurate configuration
stays visible instead of disappearing behind a threshold.

Three things the protocol exists to prevent:

* **Sweeping one knob at a time explores the wrong set.** `WeightGuard` rescales
  the update once it exceeds `max_step_rel · ‖W₀‖`, so above some `lr` the step
  size stops depending on `lr`. `lr` and `max_step_rel` must move together.
* **The best of N candidates is optimistically biased** on the flights that
  chose it. `SEARCH_SEEDS` and `HOLDOUT_SEEDS` are disjoint and a test enforces
  it. Since a seed also fixes the disturbance realisation, a held-out flight is
  unseen in **both** senses.
* **A weighted sum lets a candidate buy accuracy with instability**, which is
  the failure mode being hunted. The decision rule is a **constrained
  minimisation**: minimise tracking error subject to never diverging, staying
  under a roughness bound, and fitting the real-time budget.

**Metrics, and the mask each one needs:**

| metric | definition | mask |
|---|---|---|
| tracking error | RMSE of position vs reference | `t ≥ T_takeoff` |
| worst departure | max distance to reference | tracking **only** — excludes the repositioning between trajectory segments |
| command roughness | rms per-step thrust-command increment, ÷ nominal | `t ≥ T_takeoff` |
| computation | solver ms and gradient ms, separately | measured solo |

Both masks were found the hard way. Including take-off compresses the roughness
ratio (1.1× with it, 1.5× without). Including the segment transits inverts the
excursion result entirely — nominal MPC, which does no adaptation, posts the
*best* maximum, because during a transit the reference deliberately leads the
aircraft.

**Claims use a paired sign test**, never mean ± std: the spread between flights
is trajectory difficulty, which pairing cancels. With 12 flights, 12/12 →
p = 0.0002.

---

## 6. What the measurements say

Numbers below are from `results/tuning/` and **age**. Re-check them before
quoting.

**Online adaptation works, and clearly** (12 held-out flights, `search03` +
`retarget`):

| | tracking | worst (tracking) | p99 | roughness |
|---|---|---|---|---|
| nominal | 26.52 cm | 57.7 cm | 46.2 cm | 1.00× |
| frozen MLP | 26.55 cm | — | — | 1.1× |
| **online (adopted)** | **20.33 cm** | **53.2 cm** | **39.4 cm** | 1.66× |

−23 % tracking error, 12/12 flights, p = 0.0002.

**Three findings that constrain how you may describe it:**

1. **The frozen MLP is not a useful baseline in this simulator.** 6/12 against
   nominal, p = 0.61. `use_nominal_simulator=True` means the simulator *is* the
   nominal rigid-body model plus the injected disturbances, so a network trained
   on real flights has no unmodelled dynamics to correct and costs a
   near-constant ~5.4 cm whatever the aerodynamics (`results/tuning/diag_drag`).

2. **The network mostly learns a slowly varying force bias.** Of the error each
   disturbance costs nominal MPC, the adopted controller recovers 72 % of the
   wind's, 30 % of the ground effect's, and 6 % of the drag's (3/4 flights,
   p = 0.31). Wind is a force of time alone — three estimated constants would
   reproduce it. Raising drag 3× lifts its recovery to 18 %, so part of the 6 %
   was a signal floor, but it stays far below the wind's 72 %.

3. **The cost is the network inside the solver, not the adaptation.** nominal
   1.42 ms → frozen MLP 4.26 ms → online 4.31 ms per control step. The gradient
   step is 0.17 ms. `parametric_scope="last_layer"` pushes 99 weights instead of
   1699 and saves **nothing** (4.17 vs 4.15 ms): the solver's cost is evaluating
   the network, not transferring its weights. For an embedded target the lever
   is the ~2.8 ms the MLP costs the solver — a smaller network, or
   `model_options["linearize_mlp"]`, which is untested here.

   *That run also reported 34.68 vs 26.51 cm of tracking error for
   `last_layer`, but it was flown with `n_frozen_layers=1` while the network has
   three parameterised layers, so the middle layer was trained and never flown.
   The accuracy number is confounded and should not be quoted as the cost of
   freezing the trunk. The timing conclusion is unaffected — it does not depend
   on which layers train.*

**The anchor buys command stability.** Under a roughness bound of 2× nominal,
13 of 16 candidates were rejected, and every survivor had a high
`lambda_anchor` (10–22) while every reject had 0–6. Removing the anchor
entirely improves tracking by 1–7 % and **doubles** roughness on the realistic
disturbance mixture.

---

## 7. results/

Self-contained; the trained networks stay outside, in `../results/model_fitting/`.

| folder | what it is |
|---|---|
| `tuning/search03/` | the 631-flight hyperparameter search |
| `tuning/retarget/` | re-selection under the deployment objective (§6) |
| `tuning/deploy/` | excursion with the correct mask + per-step cost split |
| `tuning/diag_drag/` | why drag recovery is small — the three hypotheses |
| `tuning/search02/` | superseded; provenance of the previous configuration |
| `figures/`, `animations/`, `presentation/` | generated outputs |

Each `tuning/<run>/runs.jsonl` is one JSON object per flight and is resumable:
rerunning a tool skips what is already there.

---

## 8. Disturbances

Four sources, all summed into one world-frame CoG force by
`../sim_environment/disturbances.py`:

| source | law | depends on |
|---|---|---|
| payload | `F_z = −m(t)·g` | time |
| wind | `F_x, F_y = f(t)` | time |
| ground effect | `F_z = T·k/(1+(z/z₀)²)` | **state** (z) |
| drag | `F = −(k₁·v + k₂·‖v‖·v)` | **state** (v) |

Only the last two are state-dependent, and they are what justifies a network at
all — say that rather than "the network beats nominal MPC", which is a weak
claim.

In the tools, the disturbances are **drawn per flight** from a stream keyed by
the seed alone (`harness._source_rng`). Two consequences, both load-bearing:
every configuration flying seed 897 meets identical disturbances, so comparisons
stay paired; and a source flown alone is the same realisation it had inside the
mixture, so the isolating families are an attribution.

Adding a source means adding it to `COG_SOURCES` in
`../sim_environment/disturbances.py`. The simulation loop and both controller
classes derive their gates from that tuple.

`cog_dist` and `motor_noise` are **inert**: they write
`sim_solver.acados_sim.parameter_values`, which the compiled solver never reads.
Enabling them changes nothing, silently.

---

## 9. Traps that have already cost hours

**acados keys generated C code by MODEL NAME**, and compiles in
`include/aerial_robot_control/neural_mpc/<model_name>/`. Two processes sharing a
name corrupt each other's build. Anything changing the parameter-vector size
must change the name — hence the `_lastlayer` and `_dthrust` suffixes. For
parallel runs `NEURAL_MPC_BUILD_ARENA=<id>` gives each worker its own directory,
and **both** controller classes honour it.

**On a parameter mismatch acados calls `exit()`**, it does not raise. Anything
that must count failures has to isolate each flight in a subprocess — which is
what `harness.run_one()` does.

**Thread oversubscription.** numpy/torch size their pools for the machine — 80
threads per worker here. Eight workers pinned to one thread each are ~11× faster
than three unpinned. Set `OMP_NUM_THREADS=1` (and `OPENBLAS_`, `MKL_`,
`NUMEXPR_`) for any batch run.

**Timing runs must be flown solo**, and `solo` is part of the checkpoint key.
Without it the timing stage was served results measured under contention, and
reported a 145 ms worst step against a 10 ms budget.

**Reproducibility.** `run_simulation` seeds numpy **and torch**. Torch matters
because dropout stays active during the online gradient step: unseeded, four
identical flights spread over 16–19 cm of tracking error.

**Safety-layer bounds are relative to `‖W₀‖`**, and `W₀` is not the same object
under different parametrisations, so the numbers do not transfer.

---

## 10. Working style that fits this code

Measure before claiming. Several confident-looking conclusions here turned out
to be artefacts of the measurement rather than findings: a chatter threshold
that rejected nominal MPC itself; a real-time verdict caused by the measuring
process's own threads; a roughness discrepancy that was a take-off window; an
excursion result that inverted once the segment transits were masked out; and a
predicted 17× speed-up from `last_layer` that measured 1.00×.

When a number is surprising, check **how it was computed** before explaining why
it is true.
