"""
Hyperparameter search for the online residual adaptation, with a confirmation
protocol designed so the winner is not a lucky draw.

evaluate.py MEASURES configurations you already have. This one SEARCHES the
joint hyperparameter space and then proves its answer. Both run their flights
through harness.py.

Three reasons it is not a grid:

 1. The knobs interact, so sweeping them one at a time explores the wrong set.
    WeightGuard rescales the whole update once it exceeds max_step_rel*||W_0||
    (online_guards.py), so above some lr the STEP SIZE stops depending on lr
    entirely — only its direction still does, and Adam has already normalised
    that. A 31-point lr sweep at fixed max_step_rel can be 31 runs of nearly the
    same controller. lr and max_step_rel have to move together.

 2. Most of the budget must not be spent on candidates that are already losing.
    Successive halving: many candidates on few flights, then the survivors on
    more flights, then the finalists on many. Same total cost, far more of it
    spent where the decision is close.

 3. Picking the best of N candidates BIASES its score upward — the winner is
    partly the luckiest, and the more candidates the worse it gets. The only
    fix is to re-fly the finalists on flights that took no part in the search.
    SEARCH_SEEDS and HOLDOUT_SEEDS are disjoint and the final table uses only
    the held-out ones.

The decision rule is a CONSTRAINED minimisation, not a weighted sum: minimise
disturbed tracking error subject to never diverging, not degrading undisturbed
flight, not chattering, and fitting the real-time budget. A weighted sum lets a
candidate buy tracking error with instability, which is the exact failure mode
being hunted here.

RANDOMISED DISTURBANCES
-----------------------
Earlier searches flew ONE disturbance profile: the payload always appeared at
t = 30 s, the crosswind was always a 2 N sine of period 20 s. Only the
trajectory changed between flights. Whatever those searches selected was
therefore fitted to that single scenario, and nothing in the protocol could tell
a genuinely robust setting from one tuned to a payload at t = 30 s.

Now every flight draws its own realisation — payload mass and instant, wind
amplitude, period, phase and ramp, ground-effect strength and drag scales. The
draw is keyed by the SEED alone, so every candidate flying seed 897 meets
identical disturbances and the comparison stays paired. See harness.py, which
holds the scenarios, the metrics and the run machinery; this file is only the
search protocol on top of them.

Usage
-----
    python3 -m online_learning.tools.tune_online            # run (or resume)
    python3 -m online_learning.tools.tune_online --report   # re-report only
    python3 -m online_learning.tools.tune_online --clean-arenas

Results stream to results/tuning/<run-id>/runs.jsonl as they complete, so an
interrupted search resumes where it stopped instead of starting over.
"""
import argparse
import json
import math
import os
from datetime import datetime

import numpy as np

from config.configurations import DirectoryConfig, EnvConfig

# Everything about RUNNING and MEASURING a flight lives in harness.py; this file
# is only the search protocol on top of it. The names are re-exported here
# because the tests and the sibling tools address them through this module.
from online_learning.tools.harness import (          # noqa: F401
    FAMILIES, HOLDOUT_SEEDS, PKG_ROOT, REFERENCES, SEARCH_SEEDS,
    apply_scenario, chatter_metrics, describe_scenario, deviation_metrics,
    load_checkpoint, nominal_roughness, payload_onset, regime_metrics,
    rows_for, run_batch, simulate, spec_key, _failed,
    _is_infra, _jsonable, _mean, sign_test_p,
)

# ======================================================================
# SEARCH SPACE  (the "core 6")
# ----------------------------------------------------------------------
# Every dimension here governs HOW FAST or HOW FAR the model is allowed to
# move, which is what decides both tracking quality and the instability that
# wind provokes. Ranges are centred on the values measured while calibrating
# the safety layer, and extended by about a decade each way.
#
# data_horizon_s replaces the (buffer_size, forget_tau) pair. They cannot be
# searched independently: samples arrive at 1/T_samp = 100 Hz, so a 700-sample
# buffer only holds 7 s and any forget_tau above that is inert — the
# configuration file says so itself. One horizon drives both, which removes a
# whole plane of degenerate combinations from the search.
# ======================================================================
SPACE = {
    "lr":               ("logu",   1e-4, 3e-2),
    "max_step_rel":     ("logu",   1e-3, 5e-2),
    "trust_region_rel": ("logu",   0.5,  5.0),
    # A quarter of the samples get exactly 0 (feature off); the rest are
    # log-uniform. lambda_anchor is a decoupled pull rate — lr*lambda_anchor is
    # the fraction of the distance to the baseline removed per step — so the
    # useful range is O(1..100) and 0 is a genuinely different regime, not the
    # limit of small values.
    "lambda_anchor":    ("logu0", 1.0,  100.0),
    "train_every":      ("choice", [1, 2, 5, 10]),
    "data_horizon_s":   ("u",      3.0,  30.0),
}


# One training sample is stored per control step, so the buffer's horizon in
# seconds is buffer_size * T_samp. T_samp lives in the robot's controller YAML
# (robots/beetle_omni/config/BeetleOmniNMPCNominalServo.yaml: T_samp: 0.01), not
# in EnvConfig — read at import so a change to the control rate cannot silently
# turn data_horizon_s into the wrong number of samples.
# neural_mpc/, the directory the tools are meant to be run from: it holds
# config/, utils/, nmpc/ and the packages below it. Everything path-related is
# expressed from here rather than from __file__, so moving this file inside the
# package cannot silently redirect it.


def _control_period():
    # neural_mpc/ -> scripts/ -> aerial_robot_control/ -> jsk_aerial_robot/
    yml = os.path.abspath(os.path.join(
        PKG_ROOT, "../../..", "robots/beetle_omni/config/BeetleOmniNMPCNominalServo.yaml"))
    try:
        with open(yml) as f:
            for line in f:
                if "T_samp:" in line:
                    return float(line.split("T_samp:")[1].split("#")[0].strip())
    except (OSError, ValueError, IndexError):
        pass
    print(f"[tune] WARNING: could not read T_samp from {yml}, assuming 100 Hz. "
          "data_horizon_s -> buffer_size will be wrong if the control rate is not "
          "100 Hz.")
    return 0.01


T_SAMP = _control_period()


def _expand(theta):
    """Search vector -> dataset_options overrides."""
    h = float(theta["data_horizon_s"])
    over = {k: theta[k] for k in
            ("lr", "max_step_rel", "trust_region_rel", "lambda_anchor", "train_every")}
    over["train_every"] = int(over["train_every"])
    over["forget_tau"] = h
    over["buffer_size"] = int(round(h / T_SAMP))
    return over


def label_of(theta):
    return (f"lr{theta['lr']:.2e}_step{theta['max_step_rel']:.3f}"
            f"_tr{theta['trust_region_rel']:.2f}_anc{theta['lambda_anchor']:.0f}"
            f"_ev{int(theta['train_every'])}_h{theta['data_horizon_s']:.0f}")

# ======================================================================
# STAGES
# ----------------------------------------------------------------------
# n_cand   : candidates entering the stage (stage 1 draws them)
# n_flights: flights per candidate
# sim_time : flight duration [s]
# scenarios: which disturbance settings to fly
# workers  : parallel processes (1 = sequential)
#
# Stage 1 flies 60 s rather than 120 s because every disturbance EVENT is over
# by t=40: the payload steps at 30, the wind ramps 10->40, the crosswind sine
# has a 20 s period. Past 40 s the environment is stationary, so 60 s already
# contains the whole transient and the first settled stretch. Halving the
# flight halves the cost while costing little information — unlike 20 s, which
# would never see the payload at all.
#
# Stage 3 runs SEQUENTIALLY on purpose: it is the stage whose numbers get
# quoted, and per-step computation time measured while three simulations share
# the CPU is meaningless.
# ======================================================================
#
# The simulation contains no wall-clock pacing (no sleep, nothing reads the
# real time to decide anything), so running three at once cannot change a
# single number in the results — only the per-step COMPUTATION TIME, which is
# therefore measured in a dedicated sequential stage at the end rather than
# constraining the search with figures taken under CPU contention.
#
# THREADS: numpy, torch and the BLAS underneath them each open a thread pool
# sized for the whole machine. Measured here: one worker process ran 80 threads,
# so three workers put 240 threads on 32 cores and the load average sat at 47 —
# the workers were fighting each other, not progressing (82 s of wall clock per
# completed 60 s flight, against 25 s predicted). The residual MLP is 16->32->3;
# there is nothing in it worth threading. So each worker is pinned to one thread
# and the worker count is raised instead, which is real parallelism rather than
# oversubscription.
#
# The timing stage is the exception: it runs alone and must reproduce what a
# deployed controller would actually do, so it keeps the default threading.
# clean_top: an optimisation, in the stage that can afford it least.
#   The 'none' scenario only ever enters the decision as a CONSTRAINT — it can
#   disqualify a candidate, never promote one. A candidate ranked far down on
#   disturbed error therefore cannot reach the next stage whatever its
#   undisturbed flight looks like, and flying it is wasted time. So 'all' is
#   flown for everyone, and 'none' only for the top clean_top by disturbed
#   error. clean_top is set to several times the number of survivors, so the
#   constraint can still eliminate the leaders and let those below through.
WORKERS = 8            # single-threaded workers on 32 cores, headroom left over
STAGES = [
    # refs=True is not optional. Without the baselines this stage cannot
    # compute roughness (it is a ratio against nominal on the same flights), so
    # it ranked on precision alone — and precision and stability are strongly
    # anti-correlated here. Measured on the first attempt at this search: the
    # 40 candidates it promoted were the 40 roughest, every one of them was
    # rejected at the next stage, and the stable candidates had been discarded
    # 2 hours earlier. The 3 extra reference flights cost nothing.
    #
    # One caveat on this stage's roughness column: rows_for() does not filter on
    # flight duration, so the nominal yardstick here can be averaged over 60 s
    # and 120 s reference flights. That does NOT affect what advances —
    # pareto_select() works on (roughness, error) and a common divisor leaves a
    # Pareto front unchanged — but read the REJECT verdicts printed at this
    # stage as indicative. The binding decisions are all taken later, where
    # every flight is 120 s.
    dict(name="screen",  n_cand=192, n_flights=4, sim_time=60,
         scenarios=("all",), pool="search",
         workers=WORKERS, refs=True, rt=False, threads=1),
    dict(name="rank",    n_cand=40, n_flights=8, sim_time=120,
         scenarios=("all", "none", "drag_real"), pool="search", clean_top=20,
         workers=WORKERS, refs=True,  rt=False, threads=1),
    dict(name="confirm", n_cand=8,  n_flights=24, sim_time=120,
         scenarios=("all", "none", "drag_real"), pool="holdout",
         workers=WORKERS, refs=True,  rt=False, threads=1),
    # Attribution, on the finalists only: each disturbance flown ALONE, with the
    # same realisation it had inside 'all' on that seed (see _source_rng). Any
    # instability can then be pinned on a source instead of guessed at.
    dict(name="isolate", n_cand=8,  n_flights=6, sim_time=120,
         scenarios=("wind", "drag", "ground"), pool="holdout",
         workers=WORKERS, refs=True, rt=False, threads=1),
    dict(name="timing",  n_cand=8,  n_flights=2, sim_time=120,
         scenarios=("all",), pool="holdout",
         workers=1, refs=True,  rt=True,
         # Pinned, like every other stage. An earlier version left the library
         # defaults here, reasoning that a deployed controller would use them —
         # wrong: the default pool is 80 threads on this machine, and thrashing
         # them over a 16->32->32->3 MLP is not what any flight computer does. It
         # inflated the measurement 4x and produced a false "OVER" verdict.
         threads=1),
]

# Flights used to SEARCH and flights used to CONFIRM must not overlap: the
# selected candidate's score on the flights that selected it is optimistic.
#
# A seed now fixes BOTH the trajectory sequence and the disturbance realisation
# (payload mass and instant, wind shape, ground-effect strength, drag scale), so
# a held-out flight is held out in both senses: the winner has never seen that
# trajectory OR that disturbance. Twelve of them, because the confirmation is

# ---- constraints (a candidate violating any of these is not a candidate) ----
# An undisturbed flight may not be worse than NOMINAL MPC by more than this.
# The reference used to be the frozen MLP; it was changed because in this
# simulator the truth IS the nominal model plus the injected disturbances, so
# the frozen network has nothing to correct and costs a near-constant penalty
# (~5.4 cm, measured in results/tuning/diag_drag). Measuring "did adaptation
# hurt the clean flight" against a handicapped reference flatters everything.
# Applied to BOTH clean scenarios, `none` and `drag_real`; the worse one binds.
# 15 %, not 5 %. The anchor is what buys command stability, and it pulls the
# model toward W_0 — the frozen MLP, which is itself ~35 % worse than nominal on
# an undisturbed flight (results/tuning/diag_drag). Demanding 5 % of nominal
# therefore forbids any meaningfully anchored model by construction, and the
# first attempt at this search proved it: roughness fell monotonically with
# lambda_anchor (0 -> 5.7-9.4x, 22 -> 1.50x) while clean degradation rose with
# it (+1 % -> +15 %), and NOTHING satisfied both bounds. Stability is the
# primary criterion, so the clean flight is the one that gives.
CLEAN_TOL_PCT = 15.0

# Command roughness, as a MULTIPLE of what nominal MPC does on the same flights.
#
# The absolute hf_frac threshold this replaced (5% of thrust-command energy
# above 2 Hz) was calibrated on 45 s flights and did not survive contact with
# 120 s ones: measured here, nominal MPC — which has no adaptation and cannot
# chatter from learning — scores 16.9%, and the frozen MLP 16.6%. An absolute
# 5% bar therefore rejected every candidate AND both baselines, which is the
# signature of a broken constraint rather than of bad candidates.
#
# rms_du (rms of the per-step thrust-command increment) separates cleanly on the
# same data: nominal 0.0093, static 0.0103, the configuration then in
# configurations.py 0.0618 (6.7x nominal, visibly chattering), the search winner
# 0.0182 (2.0x). Relative to nominal, on the same flights, is also the honest
# comparison — it cancels whatever the trajectory itself contributes.
#
# TIGHTENED FROM 3.0 TO 1.5: command stability is now the PRIMARY criterion and
# tracking error only ranks whoever clears this bar (see score()). 3.0x never
# bound — the finalists of the previous search sat at 1.3-1.7x while the
# rejected-but-accurate candidates sat at 2.3-2.9x, so 1.5x is exactly where the
# two groups separate. Expect roughly half the field to be rejected here; that
# is the constraint doing its job, not a broken threshold.
CHATTER_TOL_X_NOMINAL = 1.5
RT_REQUIRED = True      # worst control step must fit one control period

RUN_TIMEOUT_S = 3600
RESULT_MARKER = "__RESULT__"
SEARCH_SEED = 20260730  # reproducible candidate draw

# ======================================================================
# Sampling
# ======================================================================
def latin_hypercube(n, d, rng):
    """
    n points in [0,1)^d, stratified: every dimension is split into n equal bins
    and each bin is used exactly once.

    Preferred over i.i.d. uniform because with 32 points in 6 dimensions plain
    random sampling routinely leaves a whole decade of lr unvisited, and over a
    grid because only a few of the six dimensions matter and a grid spends its
    budget resolving the ones that do not.
    """
    out = np.empty((n, d))
    for j in range(d):
        out[:, j] = (rng.permutation(n) + rng.random(n)) / n
    return out


def sample_candidates(n, rng):
    keys = list(SPACE)
    u = latin_hypercube(n, len(keys), rng)
    cands = []
    for i in range(n):
        theta = {}
        for j, k in enumerate(keys):
            spec, x = SPACE[k], u[i, j]
            if spec[0] == "logu":
                lo, hi = spec[1], spec[2]
                theta[k] = float(np.exp(np.log(lo) + x * (np.log(hi) - np.log(lo))))
            elif spec[0] == "logu0":
                lo, hi = spec[1], spec[2]
                if x < 0.25:
                    theta[k] = 0.0
                else:
                    y = (x - 0.25) / 0.75
                    theta[k] = float(np.exp(np.log(lo) + y * (np.log(hi) - np.log(lo))))
            elif spec[0] == "u":
                theta[k] = float(spec[1] + x * (spec[2] - spec[1]))
            elif spec[0] == "choice":
                opts = spec[1]
                theta[k] = opts[min(int(x * len(opts)), len(opts) - 1)]
            else:
                raise ValueError(f"unknown spec {spec[0]} for {k}")
        cands.append(theta)
    return cands


# ======================================================================
# Scoring
# ======================================================================


def clean_reference(done, seeds):
    """Nominal MPC's error on each undisturbed scenario — the clean yardstick."""
    out = {}
    for sc in ("none", "drag_real"):
        v = _mean([r.get("rmse_track") for r in rows_for(done, "nominal", sc, seeds)
                   if not r.get("failed")])
        if np.isfinite(v) and v > 0:
            out[sc] = v
    return out


def score(done, label, seeds, scenarios, clean_ref=None, nom_du=None):
    """
    (feasible, disturbed RMSE, diagnostics) for one candidate.

    STABILITY FIRST, THEN PRECISION.
    ---------------------------------
    Command stability is the primary criterion and tracking error the secondary
    one, which is NOT the same as sorting by roughness: the smoothest controller
    is the one that barely adapts, so a lexicographic sort would elect something
    close to nominal MPC and call it a win. Stability is expressed instead as an
    admissibility bound — roughness <= CHATTER_TOL_X_NOMINAL times nominal on the
    same flights — and error only ranks whoever clears it. `print_pareto()` shows
    the whole trade so the threshold is never the hidden decision.

    Constraints rather than penalty terms: a candidate that diverges once, or
    chatters, or degrades an undisturbed flight, is not a slightly worse
    candidate — it is not deployable, and no amount of tracking error saved on
    the disturbed runs changes that. A weighted sum would let it buy accuracy
    with instability, which is the exact failure mode being hunted.

    THREE SCENARIOS, ALL BINDING.
      all        the randomised disturbance mixture — what `rmse` ranks on.
      none       every injected disturbance off. Kept because "does adaptation
                 hurt when there is nothing to adapt to?" has to be answered,
                 but note it also removes drag and ground effect, which a real
                 aircraft always has: it is a physically impossible flight.
      drag_real  no injected disturbance, but drag at textbook magnitude — the
                 honest "undisturbed" flight. Held to the same bound as `none`.

    The clean reference is NOMINAL MPC, not the frozen MLP. In this simulator
    the truth IS the nominal model plus the injected disturbances, so the frozen
    network has nothing to correct and costs a near-constant penalty; measuring
    "did we degrade the clean flight" against it flatters every candidate.

    The real-time budget is deliberately NOT a constraint here: it can only be
    measured sequentially, so it is checked once per finalist in the 'timing'
    stage.
    """
    dist_rows = rows_for(done, label, "all", seeds)
    # A toolchain failure is not evidence about the hyperparameters. It is
    # counted and reported separately so it stays visible, but it does not
    # disqualify — only a flight that actually blew up does.
    n_fail = sum(1 for r in dist_rows if r.get("failed") and not r.get("infra"))
    n_infra = sum(1 for r in dist_rows if r.get("failed") and r.get("infra"))
    ok = [r for r in dist_rows if not r.get("failed")]
    rmse = _mean([r.get("rmse_track") for r in ok])
    hf = _mean([r.get("hf_frac") for r in ok])
    du = _mean([r.get("rms_du") for r in ok])
    dev = _mean([r.get("trk_max") for r in ok])
    du_x = du / nom_du if (nom_du and np.isfinite(du)) else np.nan

    # Undisturbed regression, one entry per clean scenario that was flown.
    clean = {}
    for sc in ("none", "drag_real"):
        ref = (clean_ref or {}).get(sc)
        if sc not in scenarios or ref in (None, 0, np.inf) or not np.isfinite(ref):
            continue
        v = _mean([r.get("rmse_track") for r in rows_for(done, label, sc, seeds)
                   if not r.get("failed")])
        if np.isfinite(v):
            clean[sc] = 100.0 * (v / ref - 1.0)
    clean_pct = max(clean.values()) if clean else np.nan   # the worst of them

    reasons = []
    if n_fail:
        reasons.append(f"{n_fail} diverged")
    if np.isfinite(du_x) and du_x > CHATTER_TOL_X_NOMINAL:
        reasons.append(f"rough {du_x:.2f}x")
    for sc, pct in clean.items():
        if pct > CLEAN_TOL_PCT:
            reasons.append(f"{sc} +{pct:.0f}%")
    if not np.isfinite(rmse):
        # Every flight lost to the toolchain: no evidence either way, so it is
        # not a candidate, but say why rather than call it a divergence.
        reasons.append("no usable flight")

    return dict(label=label, rmse=rmse, hf=hf, du=du, du_x=du_x, dev=dev,
                clean_pct=clean_pct, clean=clean, n_fail=n_fail, n_infra=n_infra,
                feasible=not reasons, reasons=reasons,
                per_seed={s: next((r.get("rmse_track", np.inf)
                                   for r in dist_rows if r.get("seed") == s), np.inf)
                          for s in seeds})


def pareto_select(scores, n):
    """
    Advance `n` candidates by NON-DOMINATED SORTING on (roughness, error).

    Ranking a screening stage by error alone selects against the very objective
    the search is built on: the most precise candidates are systematically the
    roughest, so the stable half of the field is discarded before anything has
    measured its stability. Taking successive Pareto fronts keeps BOTH ends of
    the trade, and lets the later stages — which fly longer, on more seeds, with
    the clean scenarios — decide where on that front the answer lies.

    Ordering within a front is by error, so the report still reads best-first.
    Candidates with no usable flight sort last.
    """
    live = [s for s in scores if np.isfinite(s["rmse"]) and np.isfinite(s["du_x"])]
    rest = [s for s in scores if s not in live]
    out = []
    while live and len(out) < n:
        ids = {id(a) for a in live}
        front = [a for a in live if not any(
            b["du_x"] <= a["du_x"] and b["rmse"] <= a["rmse"] and id(b) != id(a)
            and (b["du_x"] < a["du_x"] or b["rmse"] < a["rmse"])
            for b in live)]
        if not front:
            break
        front.sort(key=lambda s: s["rmse"])
        out += front
        front_ids = {id(a) for a in front}
        live = [a for a in live if id(a) not in front_ids]
    rest.sort(key=lambda s: s["rmse"])
    return (out + rest)[:n]


def rank(scores):
    """Feasible candidates first, then by disturbed RMSE."""
    return sorted(scores, key=lambda s: (not s["feasible"], s["rmse"]))


# ======================================================================
# Reporting
# ======================================================================
def print_stage(stage, scores, seeds):
    print("\n" + "=" * 104)
    print(f"STAGE '{stage['name']}'  —  {len(scores)} candidates, "
          f"{len(seeds)} flights of {stage['sim_time']} s, "
          f"scenarios {', '.join(stage['scenarios'])}")
    print("=" * 104)
    hdr = (f"{'#':>3}  {'candidate':<46}{'rmse':>9}{'rough':>8}"
           f"{'clean':>9}{'div':>5}{'infra':>6}  verdict")
    print(hdr)
    print("-" * len(hdr))
    for i, s in enumerate(rank(scores), 1):
        rmse = f"{s['rmse']:.4f}" if np.isfinite(s["rmse"]) else "--"
        hf = f"{s['du_x']:.1f}x" if np.isfinite(s.get("du_x", np.nan)) else "--"
        cl = f"{s['clean_pct']:+.1f}%" if np.isfinite(s["clean_pct"]) else "--"
        verdict = "ok" if s["feasible"] else "REJECT: " + ", ".join(s["reasons"])
        print(f"{i:>3}  {s['label']:<46}{rmse:>9}{hf:>8}{cl:>9}"
              f"{s['n_fail']:>5}{s.get('n_infra', 0):>6}  {verdict}")
    print("=" * 104)
    n_infra = sum(s.get("n_infra", 0) for s in scores)
    if n_infra:
        print(f"'infra' = flights lost to the acados build toolchain after "
              f"{MAX_ATTEMPTS} attempts ({n_infra} here).\nThey are NOT counted "
              "as divergences: they say nothing about the hyperparameters.\n"
              "Full stderr for each is under the run directory's failures/.")
        print("=" * 104)


def print_pareto(scores, seeds):
    """
    The stability/precision trade, in full, so the threshold is never the hidden
    decision.

    Stability is the primary criterion, expressed as an admissibility bound
    rather than as a sort key: sorting by roughness would elect whatever adapts
    least. But a bound is still a line drawn somewhere, and a candidate at 1.52x
    is not meaningfully worse than one at 1.48x. This lists every candidate by
    roughness with its error alongside, and marks the ones no other candidate
    beats on BOTH axes — the Pareto front. If the adopted configuration sits
    just inside the bound while a slightly rougher one is much more accurate,
    that is a judgement for a human, and it should be visible.
    """
    ok = [s for s in scores if np.isfinite(s["rmse"]) and np.isfinite(s["du_x"])]
    if not ok:
        return
    front = [a for a in ok if not any(
        b["du_x"] <= a["du_x"] and b["rmse"] <= a["rmse"] and b is not a
        and (b["du_x"] < a["du_x"] or b["rmse"] < a["rmse"]) for b in ok)]
    front_labels = {id(f) for f in front}

    print("\n" + "=" * 104)
    print("STABILITY / PRECISION TRADE   —   sorted by roughness, * = Pareto-optimal")
    print("=" * 104)
    hdr = (f"{'':>2} {'candidate':<46}{'rough':>8}{'rmse':>9}{'worst':>9}"
           f"{'clean':>9}  verdict")
    print(hdr)
    print("-" * len(hdr))
    for a in sorted(ok, key=lambda x: x["du_x"]):
        mark = "*" if id(a) in front_labels else " "
        bar = "  <-- roughness bound" if abs(a["du_x"] - CHATTER_TOL_X_NOMINAL) < 1e-9 else ""
        dev = f"{a['dev'] * 100:.1f}" if np.isfinite(a.get("dev", np.nan)) else "--"
        cl = f"{a['clean_pct']:+.1f}%" if np.isfinite(a["clean_pct"]) else "--"
        verdict = "ok" if a["feasible"] else "REJECT: " + ", ".join(a["reasons"])
        print(f"{mark:>2} {a['label']:<46}{a['du_x']:>7.2f}x{a['rmse'] * 100:>9.2f}"
              f"{dev:>9}{cl:>9}  {verdict}{bar}")
    print("=" * 104)
    print(f"The bound is {CHATTER_TOL_X_NOMINAL}x nominal roughness. Candidates "
          "above it are rejected however\naccurate they are — stability is the "
          "primary criterion. Read the front before\nadopting: a rejected "
          "candidate that is far more accurate is worth knowing about,\neven if "
          "the rule says no.")
    print("=" * 104)


def print_confirmation(done, finalists, seeds):
    """The only table whose numbers are quotable: held-out flights, paired."""
    print("\n" + "=" * 104)
    print(f"CONFIRMATION on {len(seeds)} HELD-OUT flights "
          "(never used to select anything)")
    print("=" * 104)

    ref = "static"
    base = {s: next((r.get("rmse_track", np.inf)
                     for r in rows_for(done, ref, "all", seeds) if r.get("seed") == s),
                    np.inf) for s in seeds}

    print(f"Paired against '{ref}', disturbed flights. "
          "'wins' = flights with lower error.\n")
    hdr = (f"{'candidate':<46}{'rmse':>9}{'vs ref':>9}{'wins':>8}"
           f"{'p(sign)':>10}{'rough':>8}{'clean':>9}")
    print(hdr)
    print("-" * len(hdr))

    clean_ref = clean_reference(done, seeds)
    nom_du = nominal_roughness(done, seeds)
    for label in finalists:
        s = score(done, label, seeds, ("all", "none", "drag_real"), clean_ref, nom_du)
        d = [(s["per_seed"][k], base[k]) for k in seeds
             if np.isfinite(s["per_seed"][k]) and np.isfinite(base[k]) and base[k] > 0]
        wins = sum(1 for a, b in d if a < b)
        pct = np.mean([100.0 * (a / b - 1.0) for a, b in d]) if d else np.nan
        p = sign_test_p(wins, len(d))
        rmse = f"{s['rmse']:.4f}" if np.isfinite(s["rmse"]) else "--"
        cl = f"{s['clean_pct']:+.1f}%" if np.isfinite(s["clean_pct"]) else "--"
        hf = f"{s['du_x']:.1f}x" if np.isfinite(s.get("du_x", np.nan)) else "--"
        flag = "" if s["feasible"] else "   REJECT: " + ", ".join(s["reasons"])
        print(f"{label:<46}{rmse:>9}{pct:>+8.1f}%{wins:>5}/{len(d):<2}"
              f"{p:>10.3f}{hf:>8}{cl:>9}{flag}")
    print("=" * 104)
    print("p(sign) is the probability of winning that many flights by chance alone.\n"
          "With 8 paired flights: 8/8 -> p=0.004, 7/8 -> p=0.035, 6/8 -> p=0.145.\n"
          "A candidate that wins on error but is REJECTed fails a deployability\n"
          "constraint and must not be adopted whatever its p-value.")
    print("=" * 104)


def print_timing(done, labels, seeds):
    """
    Per-step computation cost, measured with nothing else running.

    A configuration that does not fit the control period is not deployable
    however well it tracks, and this is the only stage whose timings mean
    anything — everywhere else three simulations share the CPU.
    """
    print("\n" + "=" * 104)
    print("COMPUTATION COST per control step (sequential, uncontended)")
    print("=" * 104)
    hdr = (f"{'candidate':<40}{'mean':>9}{'p99':>9}{'p99.9':>9}{'max':>10}"
           f"{'over budget':>14}{'fits?':>8}")
    print(hdr + "     [ms]")
    print("-" * (len(hdr) + 10))
    for label in labels:
        # solo=True: the runs flown alone by this stage, never the contended
        # ones the confirmation left in the checkpoint under the same label.
        rows = [r for r in rows_for(done, label, "all", seeds, solo=True)
                if not r.get("failed") and "rt_max" in r]
        if not rows:
            print(f"{label:<40}{'--':>9}{'--':>9}{'--':>9}{'--':>10}"
                  f"{'--':>14}{'--':>8}")
            continue
        budget = rows[0]["rt_budget"]
        mx = max(r["rt_max"] for r in rows)
        mean = float(np.mean([r["rt_mean"] for r in rows]))
        p99 = float(np.max([r.get("rt_p99", np.nan) for r in rows]))
        p999 = float(np.max([r.get("rt_p999", np.nan) for r in rows]))
        over = sum(int(r.get("rt_over", 0)) for r in rows)
        n = sum(int(r.get("rt_n", 0)) for r in rows)
        # The verdict is p99.9, not the max. One warm-up step in 12000 is not a
        # real-time violation; a p99.9 above budget is.
        ok = "OK" if (np.isfinite(p999) and p999 <= budget) else "OVER"
        print(f"{label:<40}{mean:>9.2f}{p99:>9.2f}{p999:>9.2f}{mx:>10.2f}"
              f"{f'{over}/{n}':>14}{ok:>8}")
    print("=" * 104)
    print(f"Budget {rows[0]['rt_budget'] if rows else 10.0:.1f} ms. Measured with "
          "threads pinned, one run at a time (never reused from a\n"
          "contended stage — see KEY_FIELDS). The verdict is p99.9, NOT the max: "
          "the first online\nstep allocates the optimiser state and warms up torch, "
          "and lands 15-25x the median.\n"
          "Check the 'over budget' count — a handful out of 12000 is warm-up, a "
          "steady stream is not.")
    print("=" * 104)


def print_isolation(done, labels, seeds, scenarios):
    """
    Each disturbance flown ALONE, against the mixture, on the same flights.

    Inside 'all' the sources are superimposed, so nothing in the other tables
    can say which one a candidate is struggling with. Here each family enables
    exactly one, with the same realisation it had inside 'all' on that seed
    (_source_rng is keyed by seed and source, not by family), so the columns are
    an attribution rather than a separate experiment.
    """
    fams = [s for s in scenarios if s != "none"]
    print("\n" + "=" * 104)
    print("ATTRIBUTION  (one disturbance at a time, same realisation as inside 'all')")
    print("=" * 104)
    hdr = f"{'candidate':<40}" + "".join(f"{f:>12}" for f in fams) + f"{'all':>12}"
    print(hdr + "        <- position RMSE [m]")
    print("-" * (len(hdr) + 24))

    def cell(label, sc, key, fmt="{:.4f}"):
        v = _mean([r.get(key) for r in rows_for(done, label, sc, seeds)
                   if not r.get("failed")])
        return fmt.format(v) if np.isfinite(v) else "--"

    for label in labels:
        print(f"{label:<40}" + "".join(f"{cell(label, f, 'rmse_track'):>12}"
                                       for f in fams)
              + f"{cell(label, 'all', 'rmse_track'):>12}")
    print()
    print(f"{'candidate':<40}" + "".join(f"{f:>12}" for f in fams) + f"{'all':>12}"
          + "        <- command roughness rms_du")
    print("-" * (len(hdr) + 24))
    for label in labels:
        print(f"{label:<40}" + "".join(f"{cell(label, f, 'rms_du', '{:.4f}'):>12}"
                                       for f in fams)
              + f"{cell(label, 'all', 'rms_du', '{:.4f}'):>12}")
    print("=" * 104)
    print("Read the columns against the 'nominal' and 'static' rows, not against\n"
          "each other: the families differ in how hard they are, so only the gap\n"
          "to the baselines on the SAME column means anything.\n"
          "\n"
          "'drag' is the column that justifies a network. Payload and wind are\n"
          "forces that depend on time alone, so three estimated constants would\n"
          "reproduce them exactly; drag and ground effect depend on the state.\n"
          "A candidate that beats 'static' on payload and wind but not on drag\n"
          "has learnt a bias, not a model.")
    print("=" * 104)


# ======================================================================
# Driver
# ======================================================================
def specs_for(label, flags, ds_over, stage, seeds, scenarios=None):
    # solo=True marks a run that must be flown alone, so it can never be served
    # from a result produced under CPU contention. See KEY_FIELDS.
    return [dict(label=label, flags=flags, ds_over=ds_over, stage=stage["name"],
                 seed=s, scenario=sc, sim_time=stage["sim_time"],
                 solo=bool(stage.get("rt")))
            for s in seeds
            for sc in (stage["scenarios"] if scenarios is None else scenarios)]




def main(run_id=None, report_only=False):
    out_dir = os.path.join(DirectoryConfig.ONLINE_RESULTS_DIR, "tuning",
                           run_id or datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    os.makedirs(out_dir, exist_ok=True)
    ckpt = os.path.join(out_dir, "runs.jsonl")
    done = load_checkpoint(ckpt)

    rng = np.random.default_rng(SEARCH_SEED)
    cands = sample_candidates(STAGES[0]["n_cand"], rng)
    themap = {label_of(t): t for t in cands}

    print("=" * 104)
    print(f"ONLINE HYPERPARAMETER SEARCH   ->  {out_dir}")
    print("=" * 104)
    print(f"space      : {', '.join(SPACE)}")
    print(f"search     : seeds {SEARCH_SEEDS}")
    print(f"held out   : seeds {HOLDOUT_SEEDS}  (confirmation only)")
    total_wall, total_runs = 0.0, 0
    for st in STAGES:
        n_ref = len(REFERENCES) if st["refs"] else 0
        n_sc = len(st["scenarios"])
        if st.get("clean_top"):
            # 'all' for everyone, 'none' only for the top clean_top plus refs.
            n = ((st["n_cand"] + n_ref) * (n_sc - 1)
                 + min(st["clean_top"], st["n_cand"]) + n_ref) * st["n_flights"]
        else:
            n = (st["n_cand"] + n_ref) * st["n_flights"] * n_sc
        # ~0.9 s of wall clock per simulated second, plus ~20 s of acados
        # code generation and compilation per run (measured on this machine).
        wall = n * (0.9 * st["sim_time"] + 20.0) / st["workers"]
        total_wall += wall
        total_runs += n
        print(f"stage {st['name']:<8}: ({st['n_cand']}+{n_ref}) x {st['n_flights']} "
              f"flights x {n_sc} scenario(s) x {st['sim_time']}s = {n:>4} runs, "
              f"{st['workers']} worker(s)  ~{wall / 3600:.1f} h")
    print(f"{'':<15}estimated total {total_runs} runs, ~{total_wall / 3600:.1f} h "
          "(rough; the per-run eta printed below is measured)")

    # The disturbances are now drawn per flight, so the protocol is only honest
    # if the draw is visible. Every candidate meets these exact realisations.
    print("-" * 104)
    print("Disturbance realisation per HELD-OUT flight (identical for every "
          "candidate — the comparison is paired):")
    for s in HOLDOUT_SEEDS[:max(st["n_flights"] for st in STAGES
                                if st["pool"] == "holdout")]:
        print(f"  seed {s:<6} {describe_scenario('all', s)}")
    if done:
        print(f"resuming   : {len(done)} runs already in the checkpoint")
    print("=" * 104)

    survivors, ordered = list(themap), []
    for si, stage in enumerate(STAGES):
        # Selecting stages stay on the search seeds; the confirmation and the
        # diagnostics fly seeds no selection ever touched.
        pool = SEARCH_SEEDS if stage["pool"] == "search" else HOLDOUT_SEEDS
        seeds = pool[:stage["n_flights"]]
        survivors = survivors[:stage["n_cand"]]
        fail_dir = os.path.join(out_dir, "failures")

        def build(labels, scenarios):
            out = []
            for label in labels:
                if label in themap:
                    out += specs_for(label, dict(useMLP=True, onlineMLP=True),
                                     _expand(themap[label]), stage, seeds, scenarios)
                else:
                    flags, over = next((f, o) for l, f, o in REFERENCES if l == label)
                    out += specs_for(label, flags, over if over is not None else {},
                                     stage, seeds, scenarios)
            return out

        refs = [r[0] for r in REFERENCES] if stage["refs"] else []
        print(f"\n{'#' * 104}\n# STAGE {si + 1}/{len(STAGES)}: {stage['name']}"
              f"\n{'#' * 104}")

        # The 'all' family (or, in the isolating stage, every family) is flown
        # for everyone; 'none' is deferred so clean_top can skip candidates that
        # the disturbed flights have already ruled out.
        deferred = ("none",) if ("none" in stage["scenarios"]
                                 and stage.get("clean_top")) else ()
        primary = tuple(sc for sc in stage["scenarios"] if sc not in deferred)

        specs = build(survivors + refs, primary)
        print(f"  scenarios {', '.join(primary)}: {len(specs)} runs")
        if not report_only:
            run_batch(specs, stage["workers"], ckpt, done, fail_dir=fail_dir,
                      threads=stage.get("threads", 1))

        if deferred:
            by_err = sorted(
                survivors,
                key=lambda l: score(done, l, seeds, primary)["rmse"])
            keep = by_err[:stage["clean_top"]]
            specs2 = build(keep + refs, deferred)
            print(f"  scenarios {', '.join(deferred)}: {len(specs2)} runs "
                  f"(top {len(keep)} of {len(survivors)} by disturbed error, "
                  f"plus the references)")
            if not report_only:
                run_batch(specs2, stage["workers"], ckpt, done, fail_dir=fail_dir,
                          threads=stage.get("threads", 1))

        if stage["rt"]:
            print_timing(done, survivors + refs, seeds)
            break

        clean_ref = clean_reference(done, seeds)
        nom_du = nominal_roughness(done, seeds)

        if stage["name"] == "isolate":
            print_isolation(done, survivors + refs, seeds, stage["scenarios"])
            continue

        scores = [score(done, l, seeds, stage["scenarios"], clean_ref, nom_du)
                  for l in survivors]
        print_stage(stage, scores, seeds)

        ordered = rank(scores)
        if stage["name"] == "confirm":
            print_pareto(scores, seeds)
            print_confirmation(done, [s["label"] for s in ordered] + refs, seeds)
            _print_winner(ordered, themap)
        n_next = STAGES[si + 1]["n_cand"]
        # The screening stage advances by Pareto front, not by error: see
        # pareto_select(). Later stages have measured both clean scenarios and
        # can apply the constrained rule directly.
        chosen = (pareto_select(scores, n_next) if stage["name"] == "screen"
                  else ordered[:n_next])
        survivors = [s["label"] for s in chosen]
        if stage["name"] != "confirm":
            print(f"-> {len(survivors)} candidate(s) advance, "
                  f"{len(ordered) - len(survivors)} dropped")

    with open(os.path.join(out_dir, "candidates.json"), "w") as f:
        json.dump(themap, f, indent=2)
    print(f"\nCheckpoint: {ckpt}\nCandidates: {out_dir}/candidates.json")
    return done


def _print_winner(ordered, themap):
    best = next((s for s in ordered if s["feasible"] and s["label"] in themap), None)
    print("\n" + "=" * 104)
    if best is None:
        print("NO CANDIDATE SATISFIED THE CONSTRAINTS.")
        print("Keep the configuration currently in configurations.py. The search\n"
              "space or the constraints need revisiting — see the REJECT reasons.")
        print("=" * 104)
        return
    print(f"WINNER: {best['label']}")
    print("=" * 104)
    print("Paste into EnvConfig.dataset_options:\n")
    for k, v in _expand(themap[best["label"]]).items():
        print(f"    {k!r}: {v!r},")
    print("\nAdopt it only if the confirmation table above shows it beating the\n"
          "references on the held-out flights with a small p(sign) AND no REJECT.")
    print("=" * 104)


def clean_arenas():
    """Delete the per-worker build directories created by parallel runs."""
    import glob
    import shutil
    root = os.path.abspath(os.path.join(
        PKG_ROOT, "../../include/aerial_robot_control/neural_mpc"))
    hits = [p for p in glob.glob(os.path.join(root, "*_w[0-9]*"))
            if os.path.isdir(p)]
    for p in hits:
        shutil.rmtree(p, ignore_errors=True)
        print(f"removed {p}")
    print(f"{len(hits)} arena directory(ies) removed.")




if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-id", default=None,
                    help="resume this run id under results/tuning/")
    ap.add_argument("--report", action="store_true",
                    help="re-report from the checkpoint without simulating")
    ap.add_argument("--clean-arenas", action="store_true",
                    help="delete the per-worker acados build directories")
    args = ap.parse_args()
    if args.clean_arenas:
        clean_arenas()
    else:
        main(run_id=args.run_id, report_only=args.report)
