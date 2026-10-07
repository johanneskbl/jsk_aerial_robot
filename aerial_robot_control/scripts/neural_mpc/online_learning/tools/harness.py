"""
Run flights and measure them: the shared machinery under every tool here.

tune_online.py searches hyperparameters, evaluate.py measures named
configurations, figures.py draws them. All three need the same four things, and
each of the four hides a trap that cost real time to find:

  SCENARIOS   the disturbances a flight meets, drawn per flight from a stream
              keyed by the SEED alone (_source_rng). Every configuration flying
              seed 897 therefore meets identical disturbances, which is what
              keeps every comparison PAIRED and every sign test valid.

  METRICS     tracking error, excursion, command roughness, computation time.
              Two of them are masked deliberately: roughness and error skip
              take-off, and the excursion also skips the repositioning between
              trajectory segments, where the reference deliberately leads the
              aircraft.

  ONE FLIGHT  simulate(), run in its OWN PROCESS. acados calls exit() on a
              parameter mismatch instead of raising, and it keys its generated C
              code by model name, so flights must not share an interpreter or a
              build directory.

  MANY        run_batch(): a pool of single-threaded workers, one build arena
              each, appending to a resumable checkpoint and telling a build
              failure apart from a divergence.

Not meant to be run directly, except as the worker a parent spawns:

    python3 -m online_learning.tools.harness --worker '{...}'
"""
import copy
import hashlib
import itertools
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time

import numpy as np

from config.configurations import EnvConfig, ramp, sine, step

# The drag coefficients the randomised scenarios scale around. Read from the
# configuration rather than duplicated, so re-sizing drag there re-sizes the
# search's disturbances with it.
BASE_DRAG_K1 = EnvConfig.sim_options["disturbances"]["drag_linear"]
BASE_DRAG_K2 = EnvConfig.sim_options["disturbances"]["drag_quadratic"]

# neural_mpc/, the directory every tool must run from: it holds config/, utils/,
# nmpc/ and the packages below. Everything path-related is expressed from here
# rather than from __file__, so moving a file inside the package cannot silently
# redirect it.
PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

RUN_TIMEOUT_S = 3600
RESULT_MARKER = "__RESULT__"

# Flights used to SEARCH and flights used to CONFIRM must never overlap: a
# candidate's score on the flights that selected it is optimistically biased.
# A seed fixes BOTH the trajectory sequence and the disturbance realisation, so
# a held-out flight is unseen in both senses. Twelve held-out flights, because
# the sign test is the whole argument: 12/12 -> p = 0.0002, and 10/12 -> p =
# 0.019 still clears 5 %, where 6/8 did not.
SEARCH_SEEDS = [897, 4242, 77, 2024, 5255, 3141, 1729, 6180,
                2718, 1123]
HOLDOUT_SEEDS = [31337, 8080, 606, 1234, 999, 24680, 13579, 4711,
                 1618, 8675, 4096, 5772, 7331, 2357, 8191, 6271,
                 9001, 4649, 3607, 8837, 5849, 7919, 2029, 6469]

# References carried into the final stage. "current" is the configuration
# already in configurations.py — if the search cannot beat it on held-out
# flights, the honest answer is to keep it, and that has to be measurable.
REFERENCES = [
    ("nominal", dict(useMLP=False, onlineMLP=False), None),
    ("static",  dict(useMLP=True,  onlineMLP=False), None),
    ("current", dict(useMLP=True,  onlineMLP=True),  {}),
]


# ======================================================================
# Scenarios
# ======================================================================
# Which CoG sources each family switches on. 'none' enables nothing and is the
# regression check; the single-source families are attribution diagnostics.
FAMILIES = {
    "all":    ("payload", "ground", "wind", "drag"),
    "none":   (),
    "wind":   ("wind",),
    "drag":   ("drag",),
    "ground": ("ground",),
    # Diagnostic variants of the drag magnitude. They exist because the
    # coefficients in configurations.py were sized to be VISIBLE next to the
    # wind, not to be physically right: they are 10-45x textbook quadrotor
    # values. The offline network was trained on REAL flights, where drag and
    # ground effect are present at their true magnitude, so an inflated drag
    # asks it for something that contradicts its training data — and a 'drag'
    # column showing little improvement then says more about the coefficients
    # than about the network.
    #
    #   drag_real : textbook magnitude (x0.1 of configured). What the airframe
    #               actually experiences, and what the offline net should
    #               already know. If 'static' does not beat 'nominal' here, the
    #               offline model has a problem that inflated drag did not cause.
    #   drag_hard : x3 of configured. If the recovery fraction does not rise
    #               with the damage, 6 % was not a signal-strength floor.
    "drag_real": ("drag",),
    "drag_hard": ("drag",),
}

# Multiplier applied to the drawn drag coefficients, per family.
DRAG_FAMILY_SCALE = {"drag_real": 0.1, "drag_hard": 3.0}


def _source_rng(seed, source):
    """
    The random stream for ONE disturbance source on ONE flight.

    Keyed by (seed, source) and by nothing else. Two consequences, both
    deliberate:

     1. Every candidate flying seed 897 meets exactly the same payload, the same
        wind and the same drag. Without this the comparison would no longer be
        paired: a candidate could win by drawing an easy flight, which is the
        precise failure the held-out protocol exists to prevent.

     2. The wind in the 'wind' family is the SAME wind as inside 'all' for that
        seed, because the stream does not depend on which family asked. That is
        what makes the isolating families an attribution rather than a separate
        experiment.

    SHA-256 rather than hash(): the workers are subprocesses, and Python
    randomises str hashing per process unless PYTHONHASHSEED is pinned. A
    process-dependent draw would silently unpair every comparison.
    """
    h = hashlib.sha256(f"dist|{int(seed)}|{source}".encode()).digest()
    return np.random.default_rng(int.from_bytes(h[:8], "little"))


# Ranges for the randomised realisation. Centred on the values that were fixed
# in configurations.py, widened enough that the tuned hyperparameters cannot be
# fitted to one particular disturbance shape — the limitation the previous
# search carried: the payload always appeared at t=30 s, so nothing tested
# whether the winner still adapted to a payload at t=18 s or t=44 s.
DIST_RANGES = dict(
    payload_t_on=(12.0, 45.0), payload_kg=(0.30, 0.80),
    ground_k=(0.10, 0.30), ground_z0=(0.30, 0.80),
    wind_amp=(1.0, 3.0), wind_period=(10.0, 30.0),
    wind_ramp_t0=(5.0, 20.0), wind_ramp_len=(15.0, 40.0), wind_ramp_v1=(1.5, 4.0),
    drag_scale=(0.5, 1.5),
)


def apply_scenario(sim_options, scenario, seed):
    """
    Replace the configured disturbances with a randomised realisation of
    `scenario` for this flight.

    Every source is switched off first, so nothing configured in EnvConfig
    leaks into a family that did not ask for it.
    """
    if scenario not in FAMILIES:
        raise ValueError(f"unknown scenario {scenario!r}, expected one of "
                         f"{sorted(FAMILIES)}")
    d = sim_options["disturbances"]
    for k, v in list(d.items()):
        if isinstance(v, bool):
            d[k] = False

    want = FAMILIES[scenario]
    R = DIST_RANGES

    if "payload" in want:
        r = _source_rng(seed, "payload")
        d["extra_mass"] = True
        d["extra_mass_kg"] = step(t_on=float(r.uniform(*R["payload_t_on"])),
                                  value=float(r.uniform(*R["payload_kg"])))
    if "ground" in want:
        r = _source_rng(seed, "ground")
        d["ground_effect"] = True
        d["ground_effect_k"] = float(r.uniform(*R["ground_k"]))
        d["ground_effect_z0"] = float(r.uniform(*R["ground_z0"]))
    if "wind" in want:
        r = _source_rng(seed, "wind")
        d["wind"] = True
        d["wind_x"] = sine(amp=float(r.uniform(*R["wind_amp"])),
                           period=float(r.uniform(*R["wind_period"])),
                           phase=float(r.uniform(0.0, 2.0 * math.pi)))
        t0 = float(r.uniform(*R["wind_ramp_t0"]))
        d["wind_y"] = ramp(t0=t0, t1=t0 + float(r.uniform(*R["wind_ramp_len"])),
                           v0=0.0, v1=float(r.uniform(*R["wind_ramp_v1"])))
    if "drag" in want:
        r = _source_rng(seed, "drag")
        d["drag"] = True
        # The per-family multiplier is applied on top of the per-flight draw, so
        # drag_real and drag_hard remain the SAME realisation as 'drag' on that
        # seed, only scaled — the attribution stays paired across variants.
        fam_scale = DRAG_FAMILY_SCALE.get(scenario, 1.0)
        # One scale per term, so the linear/quadratic balance varies too and the
        # winner is not tuned to one particular shape of velocity dependence.
        d["drag_linear"] = (np.asarray(BASE_DRAG_K1) * fam_scale
                            * float(r.uniform(*R["drag_scale"]))).tolist()
        d["drag_quadratic"] = (np.asarray(BASE_DRAG_K2) * fam_scale
                               * float(r.uniform(*R["drag_scale"]))).tolist()
    return sim_options


def payload_onset(sim_options, default=0.0):
    """
    When the payload actually steps on this flight, read back from the profile.

    Read back rather than returned by apply_scenario(), so it stays right even
    if the draw changes: whatever the callable does is what this reports.
    """
    d = sim_options["disturbances"]
    if not d.get("extra_mass", False):
        return default
    f = d["extra_mass_kg"]
    if not callable(f):
        return default                       # constant mass: no onset to find
    grid = np.arange(0.0, float(sim_options.get("max_sim_time", 120.0)), 0.05)
    on = [x for x in grid if f(x) > 1e-9]
    return float(on[0]) if on else default


def describe_scenario(scenario, seed):
    """The realisation as plain text — for the run log and for the tests."""
    sim = copy.deepcopy(EnvConfig.sim_options)
    d = apply_scenario(sim, scenario, seed)["disturbances"]
    if not any(d.get(k, False) for k in ("extra_mass", "ground_effect", "wind", "drag")):
        return "undisturbed"
    out = []
    if d.get("extra_mass"):
        on = next(t for t in np.arange(0, 120, 0.1) if d["extra_mass_kg"](t) > 0)
        out.append(f"payload {d['extra_mass_kg'](119.0):.2f} kg @ {on:.0f}s")
    if d.get("ground_effect"):
        out.append(f"ground k={d['ground_effect_k']:.2f} z0={d['ground_effect_z0']:.2f}")
    if d.get("wind"):
        out.append(f"wind |x|<={max(abs(d['wind_x'](t)) for t in np.arange(0, 60, 0.5)):.1f}N "
                   f"y->{d['wind_y'](119.0):.1f}N")
    if d.get("drag"):
        out.append(f"drag k1={d['drag_linear'][0]:.2f} k2={d['drag_quadratic'][0]:.2f}")
    return ", ".join(out)



# ======================================================================
# Metrics
# ======================================================================
# Where the payload lands, when the flight did not draw its own instant.
# tune_online passes the drawn value; anything flying the fixed configuration
# profile uses this.
PAYLOAD_ONSET = 30.0
# "Recovered" once the error is back under this multiple of the pre-disturbance
# error and stays there.
RECOVERY_FACTOR = 1.25
REGIMES = [("pre", None, PAYLOAD_ONSET),
           ("transient", PAYLOAD_ONSET, PAYLOAD_ONSET + 10.0),
           ("converged", PAYLOAD_ONSET + 10.0, None)]


def _pos_err(rec):
    t = np.asarray(rec["timestamp"])
    e = np.asarray(rec["state_curr"])[:, :3] - np.asarray(rec["state_ref"])[:, :3]
    return t, np.linalg.norm(e, axis=1)


def regime_metrics(rec, t_takeoff, onset=PAYLOAD_ONSET):
    """
    Position RMSE per regime, plus the adaptation time after the onset.

    `onset` is when the payload actually appears on THIS flight. It used to be
    the module constant, which was correct only while every flight stepped the
    payload at t = 30 s. tune_online.py now draws the instant per flight and
    passes what it drew; leaving the default in place would report the transient
    of a payload that arrived at 14 s as if it had arrived at 30 s, and
    `t_adapt` would be measured from the wrong moment.
    """
    t, e = _pos_err(rec)
    out = {}
    if t.size == 0:
        return {f"rmse_{name}": np.inf for name, _, _ in REGIMES} | {"t_adapt": np.inf}

    regimes = [("pre", None, onset),
               ("transient", onset, onset + 10.0),
               ("converged", onset + 10.0, None)]
    for name, lo, hi in regimes:
        lo = t_takeoff if lo is None else max(lo, t_takeoff)
        m = (t >= lo) & ((t < hi) if hi is not None else True)
        out[f"rmse_{name}"] = float(np.sqrt(np.mean(e[m] ** 2))) if np.any(m) else np.nan

    # Whole tracking phase, for a single headline number.
    m_all = t >= t_takeoff
    out["rmse_track"] = float(np.sqrt(np.mean(e[m_all] ** 2))) if np.any(m_all) else np.inf

    # Adaptation time: first instant after the onset from which the error stays
    # below RECOVERY_FACTOR * pre-disturbance RMSE for the rest of the flight.
    base = out.get("rmse_pre")
    out["t_adapt"] = np.nan
    if base is not None and np.isfinite(base) and base > 0:
        thr = RECOVERY_FACTOR * base
        post = t >= onset
        if np.any(post):
            e_post, t_post = e[post], t[post]
            bad = np.where(e_post > thr)[0]
            if bad.size == 0:
                out["t_adapt"] = 0.0                    # never left the band
            elif bad[-1] < len(t_post) - 1:
                out["t_adapt"] = float(t_post[bad[-1] + 1] - onset)
            else:
                out["t_adapt"] = np.inf                 # never recovered
    return out

def deviation_metrics(rec, t_takeoff):
    """
    How far the aircraft ever gets from the reference, not just on average.

    RMSE hides excursions: a controller that tracks well but overshoots 40 cm
    once when the payload lands scores the same as one that never leaves 15 cm.
    For anything that has to fly near an obstacle it is the excursion that
    decides, so max and p99 are reported alongside it.

    p99 as well as max because one sample out of 12000 is a single integrator
    step, not a manoeuvre — the same reason the real-time verdict reads p99.9
    rather than the worst step.
    """
    t = np.asarray(rec["timestamp"])
    x = np.asarray(rec["state_curr"])
    r = np.asarray(rec["state_ref"])
    if t.size == 0:
        return dict(dev_max=np.inf, dev_p99=np.inf, dev_p95=np.inf)
    m = t >= t_takeoff
    if not np.any(m):
        return dict(dev_max=np.inf, dev_p99=np.inf, dev_p95=np.inf)
    e = np.linalg.norm(x[m, :3] - r[m, :3], axis=1)
    out = dict(dev_max=float(np.max(e)),
               dev_p99=float(np.percentile(e, 99)),
               dev_p95=float(np.percentile(e, 95)))

    # The number that answers "how far did it ever get from where it should be":
    # repositioning between segments excluded. Without that mask the maximum is
    # the transit, and nominal MPC — which does no adaptation at all — posts the
    # best score, which is the giveaway that the metric is measuring the
    # reference generator rather than the controller.
    trk = np.asarray(rec.get("tracking", []))
    if trk.size == e.size + int(np.sum(~m)):    # recorded for the whole flight
        trk = trk[m]
    if trk.size == e.size and np.any(trk > 0.5):
        et = e[trk > 0.5]
        out.update(trk_max=float(np.max(et)),
                   trk_p99=float(np.percentile(et, 99)),
                   trk_rmse=float(np.sqrt(np.mean(et ** 2))),
                   trk_frac=float(np.mean(trk > 0.5)))
    return out


def chatter_metrics(rec, t_takeoff, f_split=2.0):
    """
    Is the thrust command oscillating rather than tracking?

    hf_frac  : share of the command's spectral energy above f_split, computed on
               the DETRENDED signal so the manoeuvre itself does not count as
               chatter. Measured previously: 0.4% for nominal MPC, 46% for
               unguarded online adaptation.
    rms_du   : rms of the per-step command increment [N]
    corr_du  : lag-1 autocorrelation of that increment. Negative means each step
               partly undoes the previous one, which is the signature of
               alternation rather than of a trend.
    """
    t = np.asarray(rec["timestamp"])
    u = np.asarray(rec["control"])
    if t.size < 16 or u.ndim != 2 or u.shape[1] < 4:
        return dict(hf_frac=np.nan, rms_du=np.nan, corr_du=np.nan)
    m = t >= t_takeoff
    ft = u[m, :4]                       # the four rotor thrust commands
    if ft.shape[0] < 16:
        return dict(hf_frac=np.nan, rms_du=np.nan, corr_du=np.nan)

    dt = float(np.median(np.diff(t[m]))) or 0.01
    x = ft - ft.mean(axis=0, keepdims=True)
    # Detrend each rotor with a moving average over ~1 s: what is left is what
    # the aircraft cannot be manoeuvring for.
    w = max(3, int(round(1.0 / dt)) | 1)
    ker = np.ones(w) / w
    trend = np.apply_along_axis(lambda c: np.convolve(c, ker, mode="same"), 0, x)
    resid = x - trend

    spec = np.abs(np.fft.rfft(resid, axis=0)) ** 2
    freq = np.fft.rfftfreq(resid.shape[0], d=dt)
    tot = spec[1:].sum()
    hf = spec[1:][freq[1:] > f_split].sum()
    du = np.diff(ft, axis=0)
    a, b = du[:-1].ravel(), du[1:].ravel()
    denom = float(np.sqrt((a ** 2).sum() * (b ** 2).sum()))
    return dict(
        hf_frac=float(100.0 * hf / tot) if tot > 0 else np.nan,
        rms_du=float(np.sqrt(np.mean(du ** 2))),
        corr_du=float((a * b).sum() / denom) if denom > 0 else np.nan,
    )



# ======================================================================
# Worker: one simulation, in its own process
# ======================================================================
def simulate(spec):
    import matplotlib
    matplotlib.use("Agg")
    from trajectory_tracking_and_record import run_simulation

    mo = copy.deepcopy(EnvConfig.model_options)
    so = copy.deepcopy(EnvConfig.solver_options)
    ds = copy.deepcopy(EnvConfig.dataset_options)
    ds.update(spec["ds_over"])
    # model_options overrides, for knobs that live there rather than in
    # dataset_options — parametric_scope above all, which decides whether 1699
    # or 99 weights are pushed to the solver every control step. The controller
    # appends "_lastlayer" to the model name in that case, so the two builds
    # cannot collide in the acados cache.
    mo.update(spec.get("mo_over") or {})
    if mo.get("parametric_scope") == "last_layer" and ds.get("n_frozen_layers", 0) < 1:
        # configurations.py enforces this pairing at import; an override made
        # here bypasses that check, and the failure it prevents is silent — the
        # trunk would be trained and never flown.
        raise ValueError("parametric_scope='last_layer' needs n_frozen_layers >= 1")
    sim = copy.deepcopy(EnvConfig.sim_options)
    sim["seed"] = spec["seed"]
    sim["max_sim_time"] = spec["sim_time"]
    apply_scenario(sim, spec["scenario"], spec["seed"])
    run = dict(EnvConfig.run_options)
    run.update(recording=False, plot_trajectory=False, real_time_plot=False,
               save_animation=False)

    dataset, dist, _mpc = run_simulation(mo, so, ds, sim, run, **spec["flags"])

    # The payload instant is drawn per flight, so the regime split has to be
    # told when it actually happened rather than assume the old fixed t = 30 s.
    m = regime_metrics(dataset._rec, sim["T_takeoff"],
                       onset=payload_onset(sim, default=sim["T_takeoff"]))
    m.update(deviation_metrics(dataset._rec, sim["T_takeoff"]))
    m.update(chatter_metrics(dataset._rec, sim["T_takeoff"]))
    m.update(label=spec["label"], seed=spec["seed"], scenario=spec["scenario"],
             stage=spec["stage"], solo=bool(spec.get("solo")), failed=False)

    online = dist.get("online") or {}
    m["reverts"] = online.get("reverts", 0)
    m["guard_hits"] = online.get("rate_limited", 0) + online.get("trust_clipped", 0)
    m["adapt_steps"] = online.get("steps", 0)

    ti = dist.get("timing") or {}
    if ti and np.asarray(ti.get("mpc_ms", [])).size:
        tot = np.asarray(ti["mpc_ms"]) + np.asarray(ti["train_ms"])
        budget = ti["T_samp"] * 1000.0
        m["rt_max"] = float(np.max(tot))
        m["rt_mean"] = float(np.mean(tot))
        # Percentiles and a count, because the max alone is not a verdict: the
        # first online step allocates the optimiser state and warms up torch,
        # and it lands 15-25x the median. Judging deployability on one such step
        # would reject every adapting controller — the same shape of error as
        # the chatter threshold that once rejected nominal MPC itself.
        m["rt_p99"] = float(np.percentile(tot, 99))
        m["rt_p999"] = float(np.percentile(tot, 99.9))
        m["rt_over"] = int(np.sum(tot > budget))
        m["rt_n"] = int(tot.size)
        m["rt_budget"] = budget
        # Kept apart, because they are cut by different levers: mpc_ms is the
        # solver carrying the network (parametric_scope decides how many weights
        # are pushed every step), train_ms is the gradient step (train_every and
        # batch_size decide it). Only the sum was stored before, which cannot
        # say which one to attack for an embedded target.
        m["mpc_mean"] = float(np.mean(ti["mpc_ms"]))
        m["mpc_p99"] = float(np.percentile(ti["mpc_ms"], 99))
        m["train_mean"] = float(np.mean(ti["train_ms"]))
        m["train_p99"] = float(np.percentile(ti["train_ms"], 99))
    return m



# ======================================================================
# Parent: scheduling, checkpointing
# ======================================================================
# Fields that identify one simulation. Reusing a result across stages is the
# point — the references fly the same specs in several stages — but 'solo' MUST
# be part of the identity.
#
# It was not, and the bug was invisible until a full run was watched: the timing
# stage flies the same (label, seed, scenario, sim_time) as the confirmation, so
# every one of its runs was served from the checkpoint, and the "uncontended"
# timings were in fact measured with eight simulations sharing the CPU. That is
# the same class of artefact as the 80-thread verdict recorded in the stage
# comments — a measurement reporting its own contention. Observed here: 145.9 ms
# worst step reused from a contended run, against a 10 ms budget.
KEY_FIELDS = ("label", "seed", "scenario", "sim_time", "flags", "ds_over",
              "mo_over", "solo")


def spec_key(spec):
    """Stable identity of one simulation, for resuming a checkpoint."""
    payload = json.dumps({k: spec.get(k) for k in KEY_FIELDS},
                         sort_keys=True, default=str)
    return hashlib.sha1(payload.encode()).hexdigest()[:16]


def _jsonable(m):
    return {k: (None if isinstance(v, float) and not np.isfinite(v) else v)
            for k, v in m.items()}


# Failures that come from the BUILD TOOLCHAIN rather than from the flight.
# acados generates C code, renders templates and compiles a shared library on
# every run; that machinery fails transiently (measured at roughly 3% of runs
# even with one build arena per worker). Such a failure says nothing about the
# hyperparameters, so it must never be counted as a divergence — over a few
# hundred runs that would silently disqualify good candidates.
INFRA_MARKERS = (
    "JSONDecodeError", "t_renderer", "OSError", "cannot open shared object",
    "No such file or directory", "Errno", "make:", "Permission denied",
    "SharedMemoryError", "MemoryError",
)
MAX_ATTEMPTS = 3


def _is_infra(why):
    return any(mark in why for mark in INFRA_MARKERS)


def _failed(spec, why, infra):
    return dict(label=spec["label"], seed=spec["seed"], scenario=spec["scenario"],
                stage=spec["stage"], solo=bool(spec.get("solo")),
                failed=True, infra=infra, why=why,
                rmse_track=np.inf, t_adapt=np.inf, hf_frac=np.inf)


def _thread_env(arena, threads):
    """
    Child environment: private build arena, and a thread budget.

    The limits have to be set in the CHILD's environment rather than by calling
    a torch/numpy API, because the pools are sized at import time — by the time
    any Python in the worker runs, it is already too late.
    """
    env = dict(os.environ, NEURAL_MPC_BUILD_ARENA=arena)
    if threads and threads > 0:
        for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                    "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS",
                    "TORCH_NUM_THREADS"):
            env[var] = str(threads)
    return env


def run_one(spec, arenas, fail_dir=None, threads=1):
    """
    Run one simulation in a child process, inside a private build arena.

    A toolchain failure is retried, in a freshly acquired arena — the arena is
    released between attempts, so a retry usually lands somewhere else and a
    corrupted build directory does not doom the run. A failure that survives
    MAX_ATTEMPTS is recorded as infrastructure, NOT as a divergence.
    """
    tag = f"{spec['label'][:38]}|{spec['scenario']}|seed={spec['seed']}"
    why, infra = "unknown", False

    for attempt in range(1, MAX_ATTEMPTS + 1):
        arena = arenas.get()
        # Stagger the launches. Each run spends its first seconds generating and
        # compiling C code, and simultaneous builds are what trips the toolchain:
        # measured with 8 workers, 5 runs in 8 needed a second attempt. A few
        # seconds of jitter against a 60-120 s flight is under 5% overhead and
        # decorrelates the build phases. On a retry it also avoids marching back
        # in lockstep with whatever the run collided with the first time.
        time.sleep(np.random.uniform(0.0, 4.0) * attempt)
        try:
            proc = subprocess.run(
                # Run as a MODULE from neural_mpc/, not as a loose file: the
                # worker needs config/, utils/ and the package itself on its
                # import path, and only that working directory provides them.
                [sys.executable, "-m", "online_learning.tools.harness",
                 "--worker", json.dumps(spec)],
                capture_output=True, text=True, timeout=RUN_TIMEOUT_S,
                env=_thread_env(arena, threads), cwd=PKG_ROOT)
        except subprocess.TimeoutExpired:
            why, infra, proc = "timeout", True, None
        finally:
            arenas.put(arena)

        if proc is not None:
            line = next((l for l in reversed(proc.stdout.splitlines())
                         if l.startswith(RESULT_MARKER)), None)
            if line is not None:
                m = json.loads(line[len(RESULT_MARKER):])
                m = {k: (np.inf if v is None and (k.startswith("rmse") or k in
                                                  ("t_adapt", "hf_frac")) else v)
                     for k, v in m.items()}
                m["infra"] = False
                print(f"  [{tag}] track={m['rmse_track']:.4f}  "
                      f"hf={m.get('hf_frac', np.nan):.1f}%  "
                      f"t_adapt={m.get('t_adapt', np.nan):.1f}s"
                      + (f"  (attempt {attempt})" if attempt > 1 else ""))
                return m

            stderr = proc.stderr.strip()
            why = (stderr.splitlines() or ["no output"])[-1]
            infra = _is_infra(stderr)
            if fail_dir:
                os.makedirs(fail_dir, exist_ok=True)
                with open(os.path.join(fail_dir, f"{spec_key(spec)}.log"), "w") as f:
                    f.write(f"# attempt {attempt}, arena {arena}, exit "
                            f"{proc.returncode}\n# {tag}\n\n{stderr}\n")

        kind = "toolchain" if infra else "SIMULATION"
        if infra and attempt < MAX_ATTEMPTS:
            print(f"  [{tag}] {kind} failure (attempt {attempt}/{MAX_ATTEMPTS}), "
                  f"retrying: {why[:80]}")
            continue
        print(f"  [{tag}] FAILED [{kind}]: {why[:100]}")
        break

    return _failed(spec, why[:300], infra)


def run_batch(specs, n_workers, ckpt_path, done, fail_dir=None, threads=1):
    """
    Run `specs`, skipping any already in `done`, appending each result to the
    checkpoint as it lands.

    Threads, not processes: each thread only waits on a subprocess, so the GIL
    is irrelevant, and a shared queue of build arenas is trivial this way. The
    arena count equals the worker count, so no two live simulations ever
    generate C code into the same directory.
    """
    todo = [s for s in specs if spec_key(s) not in done]
    if not todo:
        print(f"  (all {len(specs)} runs already in the checkpoint)")
        return
    print(f"  {len(todo)} runs to go ({len(specs) - len(todo)} already done), "
          f"{n_workers} worker(s)")

    arenas = queue.Queue()
    for i in range(n_workers):
        arenas.put(f"w{i}")

    lock = threading.Lock()
    t0 = time.time()
    counter = itertools.count(1)

    def work(spec):
        m = run_one(spec, arenas, fail_dir, threads)
        with lock:
            k = next(counter)
            done[spec_key(spec)] = m
            with open(ckpt_path, "a") as f:
                f.write(json.dumps({"key": spec_key(spec), **_jsonable(m)}) + "\n")
            el = time.time() - t0
            eta = el / k * (len(todo) - k)
            print(f"    [{k}/{len(todo)}] elapsed {el / 60:.1f} min, "
                  f"eta {eta / 60:.1f} min")

    if n_workers <= 1:
        for s in todo:
            work(s)
    else:
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=n_workers) as ex:
            list(ex.map(work, todo))



def rows_for(done, label, scenario, seeds, solo=False):
    """
    Every recorded run matching (label, scenario, seed).

    `solo` selects which measurement regime: the timing stage re-flies specs the
    other stages already flew, alone, and the two must never be mixed. Scoring
    wants the contended ones (they are the bulk and the flight is identical —
    the simulation contains no wall-clock pacing); print_timing wants the solo
    ones, because per-step cost is the one number contention changes.
    """
    out = []
    for m in done.values():
        if m.get("label") == label and m.get("scenario") == scenario \
                and m.get("seed") in seeds and bool(m.get("solo")) == solo:
            out.append(m)
    return out


def _mean(vals):
    v = [x for x in vals if x is not None and np.isfinite(x)]
    return float(np.mean(v)) if v else np.inf


def nominal_roughness(done, seeds):
    """rms_du of nominal MPC on the same flights — the yardstick for chatter."""
    v = _mean([r.get("rms_du") for r in rows_for(done, "nominal", "all", seeds)
               if not r.get("failed")])
    return v if np.isfinite(v) else None




def sign_test_p(k, n):
    """
    One-sided sign test: probability of k or more wins out of n under a fair
    coin. This is what "not luck" means with a handful of paired flights, and
    it needs no assumption about the shape of the error distribution — which
    matters, because the per-flight errors are visibly not normal.
    """
    if n == 0:
        return 1.0
    tail = sum(math.comb(n, i) for i in range(k, n + 1))
    return tail / (2 ** n)



def load_checkpoint(path):
    done = {}
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    continue          # a torn last line from a hard kill
                done[row.pop("key")] = {
                    k: (np.inf if v is None and (k.startswith("rmse") or k in
                                                 ("t_adapt", "hf_frac")) else v)
                    for k, v in row.items()}
    return done


# ======================================================================
# Worker entry point
# ======================================================================
def _worker():
    spec = json.loads(sys.argv[sys.argv.index("--worker") + 1])
    print(RESULT_MARKER + json.dumps(_jsonable(simulate(spec))), flush=True)


if __name__ == "__main__":
    if "--worker" in sys.argv:
        _worker()
    else:
        print(__doc__)
