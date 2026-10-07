# online_learning — context

**Read `README.md` in this directory first.** It explains the method, the
layout, the tutorial and the measured results. This file holds only what an
agent needs on top of it: the traps, and the working style that fits this code.

## Run everything from `neural_mpc/`, as modules

```bash
cd .../scripts/neural_mpc
python3 -m online_learning.tools.evaluate      # measure the current config
python3 -m online_learning.tools.tune_online   # the hyperparameter search
python3 -m online_learning.tools.figures       # the presentation figures
for t in online_learning/tests/test_*.py; do python3 "$t"; done
```

Elsewhere the imports of `config/`, `utils/` and `nmpc/` fail. The tools spawn
their workers as `-m` from that directory for the same reason.

## Boundaries — what is NOT ours

* `../neural_controller.py` → `NeuralMPC` is the **nominal/offline** path,
  imported by ~14 files. Do not move or rename.
* `../trajectory_tracking_and_record.py` flies all three modes, so the
  baselines and the adaptive runs stay comparable. A change here moves the
  baselines too, which silently invalidates any comparison.
* `../results/model_fitting/` holds the trained networks and is shared. This
  package's own outputs live in `online_learning/results/`.
* Everything else at the `neural_mpc/` root belongs to the offline neural MPC.

RLS and the disturbance-observer baseline were deliberately removed; only Adam
remains. Do not reintroduce them unasked.

## Numbers

| | |
|---|---|
| state / control | 17 / 8 (4 thrusts + 4 servo angles) |
| control period `T_samp` | 0.01 s, read from `robots/beetle_omni/config/BeetleOmniNMPCNominalServo.yaml` |
| MPC step / horizon | 0.1 s / 2.0 s, N = 20, SQP_RTI — **one QP iteration per step** |
| residual MLP | `neuralmodel_209`, 16 → 32 → 32 → 3 (ax, ay, az), GELU, dropout 0.1; 1699 weights, three parameterised layers |
| mass, g | 3.146 kg, 9.798 m/s² |
| real-time budget | 10 ms/step; nominal 1.4, frozen 4.3, online 4.3 |

## Traps that have already cost hours

**acados keys generated C code by MODEL NAME**, and `rh_base._mkdir()` chdirs
into `include/aerial_robot_control/neural_mpc/<model_name>/` to compile there.
Two processes sharing a name corrupt each other's build. Anything that changes
the size of the parameter vector must change the name — hence the `_lastlayer`
and `_dthrust` suffixes. For parallel runs, `NEURAL_MPC_BUILD_ARENA=<id>` gives
each worker its own directory; **both** controller classes honour it (missing it
in the simulator's class once caused 21 failures in 31 runs). Clean up with
`python3 -m online_learning.tools.tune_online --clean-arenas`.

**Two parameter-vector sizes cannot share one process.** Turning disturbances
off removes the 6 CoG parameters, so a "disturbed" and an "undisturbed" flight
have different layouts under the same model name. On a mismatch acados calls
`exit()` — it does not raise. `harness.run_one()` isolates every flight in a
subprocess for exactly this reason.

**Thread oversubscription.** numpy/torch open a pool sized for the machine — 80
threads per worker here, so 3 workers put 240 threads on 32 cores. Eight workers
pinned to one thread each are ~11x faster. Set `OMP_NUM_THREADS=1` (and
`OPENBLAS_`, `MKL_`, `NUMEXPR_`) for any batch run.

**Timing runs must be keyed `solo`.** The timing stage re-flies specs the
confirmation already flew, so before `solo` entered `KEY_FIELDS` every one of
its runs was served from the checkpoint — the "uncontended" table was reporting
measurements taken with eight simulations sharing the CPU (145.9 ms worst step,
a false OVER). `rows_for(..., solo=)` keeps the two populations apart. Nothing
else in the results changes with contention: the simulation has no wall-clock
pacing.

**The real-time verdict is p99.9, not the max.** The first online step allocates
the optimiser state and lands 15–25x the median (~90 ms). Judging on the max
rejects every adapting controller.

**Reproducibility.** `run_simulation` seeds numpy **and torch**. Torch matters
because dropout stays active during the online gradient step: unseeded, four
identical flights spread over 16–19 cm of tracking error. Never remove
`torch.manual_seed`.

**Safety-layer bounds are relative to `||W_0||`**, and `W_0` is not the same
object under different parametrisations, so the numbers do not transfer.

**`parametric_scope="last_layer"`** exposes only the output layer, so every
OTHER parameterised layer must be frozen — otherwise it is trained every step
and never flown, and the trained model silently stops being the flown model.
The network has THREE parameterised layers (16->32->32->3), so
`n_frozen_layers >= 2`. `configurations.py` declared 2 layers and therefore
demanded only 1, which let the middle layer fall through the gap;
`test_casadi_sync.py` now asserts that constant against the real network.

Measured, the scope saves **no** computation (4.17 vs 4.15 ms): the solver's
cost is evaluating the network, not transferring its weights. The accuracy
figure from that same run (34.68 vs 26.51 cm) was taken with
`n_frozen_layers=1` and is therefore CONFOUNDED by the bug above — it is not a
clean measurement of what freezing the trunk costs.

**Two metric masks, both found the hard way.** Roughness excludes take-off (the
same flight reads 1.1x with it and 1.5x without). The worst-departure metric
also excludes the repositioning between trajectory segments, where the reference
deliberately leads the aircraft — with the transits included, nominal MPC posts
the best maximum of every controller, which inverts the conclusion.

**Disturbance draws must not depend on `hash()`.** Python randomises str hashing
per process and the workers are subprocesses; `harness._source_rng` uses SHA-256
so every candidate meets identical disturbances on a given seed. A test spawns a
subprocess with a different `PYTHONHASHSEED` to check it.

## Working style that fits this code

Measure before claiming. Several confident-looking conclusions here turned out
to be artefacts of the measurement, not findings: a chatter threshold that
rejected nominal MPC itself; a real-time verdict caused by the measuring
process's own threads; a 1.1x/1.4x roughness discrepancy that was a take-off
window; an excursion result that inverted once segment transits were masked;
and a predicted 17x speed-up from `last_layer` that measured 1.00x.

When a number is surprising, check **how it was computed** before explaining why
it is true.
