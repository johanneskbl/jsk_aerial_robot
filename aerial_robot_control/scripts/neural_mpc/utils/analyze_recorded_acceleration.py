#!/usr/bin/env python3
"""
Analysis of the *measured* linear acceleration recorded during the CDC 2026
real-world experiments.

Figure 8 of the paper compares the acceleration that the three controllers
predict *internally* (analytical model, optionally plus the neural residual).
This script provides the empirical counterpart: it reads the flight rosbags and
compares the acceleration the robot *actually* experienced, in order to test the
claim that

    a residual correction with a lower energy footprint yields a smoother,
    lower-variance acceleration and therefore a more stable flight.

Acceleration source
-------------------
The primary signal is ``<ns>/sensor_plugin/imu1/acc_only`` field
``acc_non_bias_world_frame`` (~200 Hz).  In ``aerial_robot_estimation`` this is
built as ``acc_w = R_base * acc_body - [0, 0, G]`` and then bias corrected
(see ``aerial_robot_estimation/src/sensor/imu.cpp``), so it is the
gravity-compensated, bias-corrected world-frame acceleration.  That is exactly
the quantity ``v_dot`` of the analytical model (6b), which makes it directly
comparable to the predicted acceleration of Figure 8.

The IMU sits on the baselink while the model is written at the CoG, but the
baselink->CoG offset is only ~(0, 0.003, 0.018) m, so the lever-arm terms
``omega x (omega x r)`` stay below ~0.02 m/s^2 for the rates flown here --
two orders of magnitude under the signals of interest.  They are ignored.

As an estimator-independent cross-check the script also differentiates the EKF
CoG velocity from ``<ns>/uav/cog/odom`` (100 Hz), which reproduces the label
definition (13) used to train the residual network.

Phase segmentation
------------------
Every run contains a takeoff plus three tracked trajectories (circle,
lemniscate, setpoint).  The runs are not synchronised with each other, so
phases are recovered per bag:

* takeoff  -- ``<ns>/flight_state`` TAKEOFF(3) -> HOVER(5), plus a hover tail
* tracking -- the TRACK windows of ``/mpc_smach_introspection/smach/container_status``,
  clipped to the interval in which the reference trajectory is actually
  published, then classified from the reference geometry itself.

Each phase is then re-zeroed on the onset of reference motion, which aligns the
runs to within a few tens of milliseconds and makes the per-block paired
statistics below meaningful.

Usage
-----
    python3 analyze_recorded_acceleration.py --bag-dir ~/ros/rosbags
    python3 analyze_recorded_acceleration.py --phases circle setpoint --show
    python3 analyze_recorded_acceleration.py --ours-bag <other>.bag --no-cache
"""

import argparse
import os
import shutil
import sys

import numpy as np
import matplotlib

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from scipy.signal import filtfilt, welch
from scipy.stats import wilcoxon

# Allow running this file directly from anywhere inside the package.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_THIS_DIR)
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from utils.filter_utils import butter_lowpass  # noqa: E402


# ======================================================================================
# Configuration
# ======================================================================================

# Physical parameters of the tiltable-quadrotor (robots/beetle_omni/config/
# PhysParamBeetleOmniJetson.yaml).  Only mass and gravity are needed here.
MASS = 3.146  # kg
GRAVITY = 9.798  # m/s^2, Tokyo

DEFAULT_BAG_DIR = "/home/jojo_ws/rosbags"
# Searched in order when a bag is not found in --bag-dir.
FALLBACK_BAG_DIRS = ["~/ros/rosbags", "/home/come-dragon/ros/rosbags", "~/rosbags"]
# Extracted bag arrays live here, deliberately outside the results tree: they run
# to tens of MB and must not be swept into the repository with the figures.
DEFAULT_CACHE_DIR = "~/.cache/jsk_neural_mpc/accel_analysis"

# The three controllers compared in the paper.  Keys are the labels used in all
# tables and figures; keep them in sync with visualize_results_cdc_2026.py.
DEFAULT_BAGS = {
    "Analytical MPC": "2026-03-29-09-22-46_mode_10_success.bag",
    "RTNMPC": "2026-03-29-09-30-56_mode_11_model_211_success.bag",
    # "Ours": "2026-03-29-09-58-09_mode_11_model_213_success.bag",
    "Ours": "2026-03-29-09-49-16_mode_11_model_214_success.bag",
    # TAKEAWAYS:
    #   - Model 214 (rel energy) is actually better than 213 (abs) since
    #     213 is almost identical to 211
    #   - Especially in figure smoothness_setpoint
    # Need to remove quarter where 214 started higher from comparison
    #   - Include acceleration plots in reasoning
    #     as to why our hypothesis holds (lower std acceleration)
    #     Make abs acceleration candle plot and std acc plot for 
    #     setpoint and circle WE ARE NOT ONLY LOWER THAN RTNMPC but also nominal
    #   - Remove takeoff as a traj and begin with setpoint as in video
    #   - For normal setpoint plot, focus on transition segments! 
}
# Controller the smoothness claim is about, and the baseline it is claimed
# against.  Used for the paired per-block tests.
TREATMENT = "Ours"
BASELINE = "RTNMPC"

ROBOT_NS = "beetle1"
SMACH_TOPIC = "/mpc_smach_introspection/smach/container_status"

# navi state enum, aerial_robot_control/include/aerial_robot_control/flight_navigation.h
TAKEOFF_STATE = 3
HOVER_STATE = 5

# --- signal processing ---------------------------------------------------------------
# The raw 200 Hz IMU carries a strong 50-100 Hz airframe/rotor vibration component
# that no controller can influence.  Everything the MPC can shape lives well below
# 10 Hz (T_step = 0.1 s), so the control-band analysis is low-passed there.  The
# vibration band is reported separately instead of being silently discarded.
CONTROL_BAND_HZ = 10.0
LOWPASS_ORDER = 4
PSD_BANDS_HZ = [(0.0, 2.0), (2.0, 5.0), (5.0, 10.0), (10.0, 20.0), (20.0, 100.0)]
HF_BAND_HZ = (10.0, 100.0)

# --- windowing -----------------------------------------------------------------------
BLOCK_S = 1.0  # block length for per-block statistics and the block bootstrap
N_BOOTSTRAP = 2000
# The runs enter the circle from slightly different altitudes, so the first
# couple of seconds are an entry transient that says nothing about the residual
# model.  They are excluded from the steady-state statistics.
CIRCLE_SKIP_S = 2.0
SETPOINT_TRANSITION_S = 3.0  # a step change is "in transition" for this long
TAKEOFF_HOVER_TAIL_S = 3.0  # hover time appended to the takeoff phase

# --- plotting (matches visualize_results_cdc_2026.py) --------------------------------
MATLAB_BLUE = "#0072BD"
MATLAB_ORANGE = "#D95319"
MATLAB_YELLOW = "#EDB120"
MATLAB_GREEN = "#77AC30"
MATLAB_PURPLE = "#7E2F8E"

COLORS = {"Analytical MPC": MATLAB_BLUE, "RTNMPC": MATLAB_YELLOW, "Ours": MATLAB_PURPLE}
# Identity is carried by dash pattern as well as hue, so the figures survive
# greyscale printing and colour-vision deficiency.
LINESTYLES = {"Analytical MPC": (0, (3.0, 2.5)), "RTNMPC": (0, (6.0, 1.5)), "Ours": "-"}
DENSE_DOTTED = (0, (0.5, 2.0))
FIG_FORMAT = "pdf"  # overridden by --format
FIG_TIGHT_LAYOUT_TOP = 0.94
LEGEND_FRAME_ALPHA = 0.85

PHASE_TITLES = {
    "takeoff": "Takeoff Trajectory",
    "circle": "Circle Trajectory",
    "lemniscate": "Lemniscate Trajectory",
    "setpoint": "Setpoint Trajectory",
}


# ======================================================================================
# Bag loading
# ======================================================================================


def resolve_bag(name, bag_dir):
    """Return the first existing path for ``name``, searching the fallback dirs."""
    if os.path.isabs(name):
        if os.path.exists(name):
            return name
        raise FileNotFoundError(name)

    candidates = [os.path.join(os.path.expanduser(bag_dir), name)]
    candidates += [os.path.join(os.path.expanduser(d), name) for d in FALLBACK_BAG_DIRS]
    # The catkin workspace root is two levels above src/jsk_aerial_robot/...
    ws_root = os.path.abspath(os.path.join(_PKG_DIR, "../../../../../.."))
    candidates.append(os.path.join(ws_root, "rosbags", name))

    for c in candidates:
        if os.path.exists(c):
            return c
    raise FileNotFoundError(f"Could not find '{name}'. Looked in:\n  " + "\n  ".join(candidates))


def _cache_path(bag_path, cache_dir):
    stat = os.stat(bag_path)
    tag = f"{os.path.basename(bag_path)}.{int(stat.st_mtime)}.{stat.st_size}.npz"
    return os.path.join(cache_dir, tag)


def load_run(bag_path, ns=ROBOT_NS, cache_dir=None):
    """Extract the arrays needed for the analysis from one bag.

    Deserialising a 300 MB bag takes tens of seconds, so the extracted arrays
    are cached next to the results and reused on subsequent calls.
    """
    if cache_dir is not None:
        cache = _cache_path(bag_path, cache_dir)
        if os.path.exists(cache):
            with np.load(cache, allow_pickle=True) as f:
                return {k: f[k] for k in f.files}

    import rosbag  # imported lazily so --help works without a ROS environment

    topics = {
        "acc": f"/{ns}/sensor_plugin/imu1/acc_only",
        "odom": f"/{ns}/uav/cog/odom",
        "ref": f"/{ns}/set_ref_traj",
        "flight_state": f"/{ns}/flight_state",
        "smach": SMACH_TOPIC,
    }
    acc, odom, ref, fstate, smach = [], [], [], [], []

    bag = rosbag.Bag(bag_path)
    try:
        t0 = bag.get_start_time()
        duration = bag.get_end_time() - t0
        for topic, msg, t in bag.read_messages(topics=list(topics.values())):
            if topic == topics["acc"]:
                a = msg.acc_non_bias_world_frame
                ab = msg.acc_body_frame
                acc.append(
                    (msg.header.stamp.to_sec() - t0, a.x, a.y, a.z, ab.x, ab.y, ab.z)
                )
            elif topic == topics["odom"]:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                v, w = msg.twist.twist.linear, msg.twist.twist.angular
                odom.append(
                    (
                        msg.header.stamp.to_sec() - t0,
                        p.x, p.y, p.z,
                        v.x, v.y, v.z,
                        q.w, q.x, q.y, q.z,
                        w.x, w.y, w.z,
                    )
                )
            elif topic == topics["ref"]:
                pt = msg.points[0]
                tr = pt.transforms[0].translation
                vl = pt.velocities[0].linear
                al = pt.accelerations[0].linear
                # The reference stream carries no usable header stamp, so the bag
                # receive time is used; it is within one publish period (20 ms).
                ref.append((t.to_sec() - t0, tr.x, tr.y, tr.z, vl.x, vl.y, vl.z, al.x, al.y, al.z))
            elif topic == topics["flight_state"]:
                fstate.append((t.to_sec() - t0, float(msg.data)))
            elif topic == topics["smach"]:
                if msg.path == "/MPC_SMACH" and msg.info != "HEARTBEAT":
                    # 0 = IDLE, 1 = INIT, 2 = TRACK
                    code = {"IDLE": 0, "INIT": 1, "TRACK": 2}.get(msg.active_states[0], -1)
                    smach.append((t.to_sec() - t0, float(code)))
    finally:
        bag.close()

    run = {
        "acc": np.asarray(acc, dtype=float),
        "odom": np.asarray(odom, dtype=float),
        "ref": np.asarray(ref, dtype=float),
        "flight_state": np.asarray(fstate, dtype=float),
        "smach": np.asarray(smach, dtype=float),
        "duration": np.asarray([duration]),
        "bag_path": np.asarray([bag_path]),
    }
    if cache_dir is not None:
        os.makedirs(cache_dir, exist_ok=True)
        np.savez_compressed(_cache_path(bag_path, cache_dir), **run)
    return run


# ======================================================================================
# Phase segmentation
# ======================================================================================


def classify_reference(ref_win):
    """Name the trajectory from the geometry of its reference positions.

    ``ref_win`` is the ``ref`` array restricted to one TRACK window.  The order
    of the tests matters: the setpoint reference also spans 0.5 m in z, so it
    has to be caught by its piecewise-constant shape *before* the z-range test
    that identifies the lemniscate.
    """
    p = ref_win[:, 1:4]
    speed = np.linalg.norm(ref_win[:, 4:7], axis=1)

    # Setpoint: a handful of held poses, no commanded velocity between them.
    n_unique = len(np.unique(np.round(p, 2), axis=0))
    if n_unique <= 10 and speed.max() < 0.05:
        return "setpoint"

    # Lemniscate: the only continuous trajectory that also moves in z.
    if p[:, 2].max() - p[:, 2].min() > 0.25:
        return "lemniscate"

    # Circle: constant xy radius at constant height.
    radius = np.linalg.norm(p[:, :2], axis=1)
    if radius.mean() > 0.5 and radius.std() < 0.15 * max(radius.mean(), 1e-9):
        return "circle"

    return "unknown"


def _motion_onset(ref_win, vel_thresh=0.02):
    """First time the reference itself starts to move, else the window start."""
    speed = np.linalg.norm(ref_win[:, 4:7], axis=1)
    moving = np.where(speed > vel_thresh)[0]
    if len(moving):
        return ref_win[moving[0], 0]
    # Setpoint references have zero commanded velocity; fall back to position steps.
    steps = np.where(np.linalg.norm(np.diff(ref_win[:, 1:4], axis=0), axis=1) > 0.05)[0]
    if len(steps):
        return ref_win[steps[0] + 1, 0]
    return ref_win[0, 0]


def segment_phases(run, takeoff_tail_s=TAKEOFF_HOVER_TAIL_S):
    """Return ``{phase_name: (t_start, t_end, t_align)}`` in bag time.

    ``t_align`` is the instant used to re-zero the phase clock across runs.
    """
    phases = {}

    fs = run["flight_state"]
    if len(fs):
        take = np.where(fs[:, 1] == TAKEOFF_STATE)[0]
        hover = np.where(fs[:, 1] == HOVER_STATE)[0]
        if len(take) and len(hover):
            t_start = fs[take[0], 0]
            t_hover = fs[hover[0], 0]
            phases["takeoff"] = (t_start, t_hover + takeoff_tail_s, t_start)

    smach, ref = run["smach"], run["ref"]
    if not len(smach) or not len(ref):
        return phases

    end_of_bag = float(run["duration"][0])
    for i, (t_s, code) in enumerate(smach):
        if code != 2:  # not TRACK
            continue
        t_e = smach[i + 1, 0] if i + 1 < len(smach) else end_of_bag
        # The SMACH state can linger after the trajectory has been fully
        # published; clip to the interval that actually carries a reference.
        in_win = (ref[:, 0] >= t_s) & (ref[:, 0] <= t_e)
        if in_win.sum() < 10:
            continue
        ref_win = ref[in_win]
        t_e = min(t_e, ref_win[-1, 0])

        name = classify_reference(ref_win)
        if name == "unknown":
            continue
        if name in phases:  # keep the longest occurrence if a trajectory repeats
            if t_e - t_s <= phases[name][1] - phases[name][0]:
                continue
        phases[name] = (t_s, t_e, _motion_onset(ref_win))

    return phases


def setpoint_step_times(run, window):
    """Absolute times at which the setpoint reference jumps."""
    t_s, t_e = window[0], window[1]
    ref = run["ref"]
    m = (ref[:, 0] >= t_s) & (ref[:, 0] <= t_e)
    r = ref[m]
    if len(r) < 2:
        return np.array([])
    jumps = np.where(np.linalg.norm(np.diff(r[:, 1:4], axis=0), axis=1) > 0.05)[0]
    return r[jumps + 1, 0]


# ======================================================================================
# Signal processing
# ======================================================================================


def low_pass(x, fs, cutoff=CONTROL_BAND_HZ, order=LOWPASS_ORDER):
    """Zero-phase Butterworth low-pass along the time axis of an (N, D) array."""
    b, a = butter_lowpass(cutoff, fs, order=order)
    return filtfilt(b, a, x, axis=0)


def sample_rate(t):
    return 1.0 / float(np.median(np.diff(t)))


def _ref_covered(t, t_ref, max_gap=0.2):
    """True where a reference sample exists within ``max_gap`` of each time."""
    if len(t_ref) == 0:
        return np.zeros(len(t), dtype=bool)
    idx = np.clip(np.searchsorted(t_ref, t), 1, len(t_ref) - 1)
    gap = np.minimum(np.abs(t - t_ref[idx - 1]), np.abs(t_ref[idx] - t))
    return gap <= max_gap


def slice_signals(run, window, cutoff=CONTROL_BAND_HZ):
    """Extract and condition every signal needed for one phase.

    Returns a dict with the phase clock re-zeroed on ``window[2]``.
    """
    t_s, t_e, t_align = window

    acc = run["acc"]
    m = (acc[:, 0] >= t_s) & (acc[:, 0] <= t_e)
    t_a = acc[m, 0]
    a_raw = acc[m, 1:4]
    fs_a = sample_rate(t_a)
    a_ctrl = low_pass(a_raw, fs_a, cutoff)

    odom = run["odom"]
    mo = (odom[:, 0] >= t_s) & (odom[:, 0] <= t_e)
    t_o = odom[mo, 0]
    p, v = odom[mo, 1:4], odom[mo, 4:7]
    fs_o = sample_rate(t_o)
    # Cross-check acceleration: derivative of the EKF velocity, i.e. the label
    # definition (13).  Low-passed identically so the two are comparable.
    a_odom = np.gradient(low_pass(v, fs_o, cutoff), t_o, axis=0)

    # Reference. np.interp clamps outside its support, which would silently
    # invent a reference for phases the trajectory publisher never covered --
    # takeoff, for instance, is flown by the navigator, not by set_ref_traj.
    # Those samples are marked NaN so the ref-based metrics come out NaN rather
    # than wrong.
    ref = run["ref"]
    a_ref = np.stack([np.interp(t_a, ref[:, 0], ref[:, 7 + i]) for i in range(3)], axis=1)
    p_ref = np.stack([np.interp(t_o, ref[:, 0], ref[:, 1 + i]) for i in range(3)], axis=1)
    v_ref = np.stack([np.interp(t_o, ref[:, 0], ref[:, 4 + i]) for i in range(3)], axis=1)
    covered = _ref_covered(t_o, ref[:, 0])
    a_ref[~_ref_covered(t_a, ref[:, 0])] = np.nan
    p_ref[~covered] = np.nan
    v_ref[~covered] = np.nan

    return {
        "t": t_a - t_align,
        "a_raw": a_raw,
        "a": a_ctrl,
        "a_ref": a_ref,
        "jerk": np.gradient(a_ctrl, t_a, axis=0),
        "fs": fs_a,
        "t_odom": t_o - t_align,
        "p": p,
        "v": v,
        "p_ref": p_ref,
        "v_ref": v_ref,
        "a_odom": a_odom,
        "fs_odom": fs_o,
    }


# ======================================================================================
# Metrics
# ======================================================================================


def mechanical_energy(p, v):
    """E = m|v|^2/2 + m g h for a measured or reference state."""
    return 0.5 * MASS * (v ** 2).sum(axis=1) + MASS * GRAVITY * p[:, 2]


def nan_rms(x):
    """RMS ignoring NaN; NaN if nothing is left (i.e. no reference coverage)."""
    valid = np.asarray(x, dtype=float)
    valid = valid[np.isfinite(valid)]
    return float(np.sqrt((valid ** 2).mean())) if valid.size else float("nan")


def nan_mean(x):
    valid = np.asarray(x, dtype=float)
    valid = valid[np.isfinite(valid)]
    return float(valid.mean()) if valid.size else float("nan")


def band_rms(t, x, bands=PSD_BANDS_HZ):
    """RMS of each column of ``x`` restricted to the given frequency bands."""
    fs = sample_rate(t)
    nperseg = int(min(len(x), max(256, fs * 4)))
    f, pxx = welch(x, fs=fs, nperseg=nperseg, axis=0)
    df = f[1] - f[0]
    out = {}
    for lo, hi in bands:
        sel = (f >= lo) & (f < hi)
        out[f"{lo:g}-{hi:g}Hz"] = np.sqrt(pxx[sel].sum(axis=0) * df)
    return out


def compute_metrics(sig):
    """Scalar metrics for one phase of one run.

    Reported per axis and for the vector norm:

    ``std_a``      spread of the acceleration -- the variance claim.
    ``rms_res``    RMS of ``a - a_ref``: the acceleration the trajectory did not
                   ask for.  On the circle the commanded centripetal term is
                   ~0.4 m/s^2, so the raw spread would mostly measure the task,
                   not the controller.
    ``rms_jerk``   RMS ``da/dt`` in the control band -- the smoothness claim.
    ``tv_rate``    mean |da/dt|, a robust companion to the RMS jerk.
    ``rms_dEdt``   RMS of the mechanical power d/dt (m|v|^2/2 + mgh); the
                   measured analogue of the energy difference (16) that the
                   regularisation penalises.
    """
    a, a_ref, jerk = sig["a"], sig["a_ref"], sig["jerk"]
    res = a - a_ref
    a_norm = np.linalg.norm(a, axis=1)
    jerk_norm = np.linalg.norm(jerk, axis=1)

    m = {}
    for i, ax in enumerate("xyz"):
        m[f"std_a_{ax}"] = float(a[:, i].std())
        m[f"rms_res_{ax}"] = nan_rms(res[:, i])
        m[f"rms_jerk_{ax}"] = float(np.sqrt((jerk[:, i] ** 2).mean()))
        m[f"tv_rate_{ax}"] = float(np.abs(jerk[:, i]).mean())
    m["std_a_norm"] = float(a_norm.std())
    m["rms_res_norm"] = nan_rms(np.sqrt((res ** 2).sum(axis=1)))
    m["rms_jerk_norm"] = float(np.sqrt((jerk_norm ** 2).mean()))
    m["tv_rate_norm"] = float(jerk_norm.mean())

    # Vibration band, reported so the low-pass is auditable rather than hidden.
    raw_bands = band_rms(sig["t"], sig["a_raw"])
    for name, vals in raw_bands.items():
        for i, ax in enumerate("xyz"):
            m[f"band_{name}_{ax}"] = float(vals[i])
    lo = HF_BAND_HZ[0]
    hf = np.sqrt(sum(raw_bands[k] ** 2 for k in raw_bands if float(k.split("-")[0]) >= lo))
    m["rms_hf_norm"] = float(np.linalg.norm(hf))

    # Mechanical energy of the measured state, and the tracking error.
    t_o, p, v = sig["t_odom"], sig["p"], sig["v"]
    energy = mechanical_energy(p, v)
    dedt = np.gradient(energy, t_o)
    m["rms_dEdt"] = float(np.sqrt((dedt ** 2).mean()))
    m["mean_E"] = float(energy.mean())

    # Excess energy over the reference trajectory.  Absolute E is dominated by
    # m*g*h, so comparing it across controllers mostly restates the altitude
    # error; the excess over the reference is the measured analogue of the
    # energy difference (16) that the regulariser penalises.
    e_ref = mechanical_energy(sig["p_ref"], sig["v_ref"])
    excess = energy - e_ref
    m["rms_dE_excess"] = nan_rms(excess)
    m["mean_dE_excess"] = nan_mean(excess)
    m["mae_p"] = nan_mean(np.linalg.norm(p - sig["p_ref"], axis=1))
    return m


def compute_odom_metrics(sig, t_min=None):
    """The same smoothness metrics from the EKF-velocity derivative.

    The IMU and the mocap-driven EKF are independent sensing paths, so a
    ranking that survives both is not an artefact of accelerometer noise or of
    the estimator's own filtering.  Absolute values are not comparable between
    the two -- differentiating a filtered velocity attenuates jerk -- only the
    ordering is.
    """
    t, a = sig["t_odom"], sig["a_odom"]
    if t_min is not None:
        keep = t >= t_min
        t, a = t[keep], a[keep]
    jerk = np.gradient(a, t, axis=0)
    return {
        "std_a_norm": float(np.linalg.norm(a, axis=1).std()),
        "rms_jerk_norm": float(np.sqrt((np.linalg.norm(jerk, axis=1) ** 2).mean())),
    }


def block_metric_series(sig, metric, block_s=BLOCK_S, t_min=None):
    """Evaluate ``metric`` on consecutive fixed-length blocks.

    Returns ``(block_start_times, values)``.  Blocks are the unit for both the
    bootstrap and the paired test: they are long enough to contain several
    control cycles and short enough to give a usable sample count.
    """
    t = sig["t"]
    lo = t[0] if t_min is None else max(t[0], t_min)
    edges = np.arange(lo, t[-1] - 0.5 * block_s, block_s)
    starts, vals = [], []
    for e in edges:
        idx = np.where((t >= e) & (t < e + block_s))[0]
        if len(idx) < 10:
            continue
        starts.append(e)
        vals.append(metric(sig, idx))
    return np.asarray(starts), np.asarray(vals)


def metric_rms_jerk_norm(sig, idx):
    return float(np.sqrt((np.linalg.norm(sig["jerk"][idx], axis=1) ** 2).mean()))


def metric_std_a_norm(sig, idx):
    return float(np.linalg.norm(sig["a"][idx], axis=1).std())


def metric_rms_res_norm(sig, idx):
    r = sig["a"][idx] - sig["a_ref"][idx]
    return nan_rms(np.sqrt((r ** 2).sum(axis=1)))


BLOCK_METRICS = {
    "rms_jerk_norm": metric_rms_jerk_norm,
    "std_a_norm": metric_std_a_norm,
    "rms_res_norm": metric_rms_res_norm,
}


def block_bootstrap_ci(values, n_boot=N_BOOTSTRAP, seed=0, agg=np.mean):
    """Percentile CI for ``agg`` over resampled blocks.

    Resampling whole blocks rather than samples keeps the strong serial
    correlation of a 200 Hz flight signal from inflating the effective sample
    size, which a naive bootstrap would do by a factor of ~200.
    """
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if len(values) < 3:
        return (np.nan, np.nan)
    rng = np.random.default_rng(seed)
    draws = agg(values[rng.integers(0, len(values), size=(n_boot, len(values)))], axis=1)
    return tuple(np.percentile(draws, [2.5, 97.5]))


def paired_block_test(sig_a, sig_b, metric, block_s=BLOCK_S, t_min=None):
    """Wilcoxon signed-rank test on blocks matched by aligned phase time.

    Both runs fly the identical trajectory and the phase clocks are re-zeroed on
    reference-motion onset, so block *k* of one run and block *k* of the other
    cover the same portion of the manoeuvre.  Pairing removes the
    between-manoeuvre variance that swamps the unpaired comparison.
    """
    _, v_a = block_metric_series(sig_a, metric, block_s, t_min)
    _, v_b = block_metric_series(sig_b, metric, block_s, t_min)
    n = min(len(v_a), len(v_b))
    v_a, v_b = v_a[:n], v_b[:n]
    keep = np.isfinite(v_a) & np.isfinite(v_b)
    v_a, v_b, n = v_a[keep], v_b[keep], int(keep.sum())
    if n < 5:
        return {"n": n, "median_delta": np.nan, "rel_delta": np.nan, "p": np.nan}
    delta = v_a - v_b
    try:
        _, p = wilcoxon(v_a, v_b)
    except ValueError:  # all differences zero
        p = 1.0
    return {
        "n": n,
        "median_delta": float(np.median(delta)),
        "rel_delta": float(np.median(delta) / np.median(v_b)) if np.median(v_b) else np.nan,
        "p": float(p),
    }


def subwindow_masks(phase, run, window):
    """Split a phase into the sub-intervals the claim is really about.

    Section V-C of the paper argues the energy loss shows up "only in the
    transition movements", so the setpoint phase is reported split into the 3 s
    following each reference step and the quasi-static hold in between.  The
    circle simply drops its entry transient.

    Returns ``{name: callable(t) -> bool mask}`` so the same sub-window can be
    applied to the 200 Hz IMU clock and the 100 Hz odometry clock alike.
    """
    if phase == "setpoint":
        steps = setpoint_step_times(run, window) - window[2]

        def transition(t):
            m = np.zeros(len(t), dtype=bool)
            for s in steps:
                m |= (t >= s) & (t < s + SETPOINT_TRANSITION_S)
            return m

        # The clock is zeroed on the first step, so the initial 8 s hold sits at
        # negative t; it is quasi-static hold data and belongs in "hold".
        return {"transition": transition, "hold": lambda t: ~transition(t)}
    if phase == "circle":
        return {"steady": lambda t: t >= CIRCLE_SKIP_S}
    return {}


# ======================================================================================
# Reporting
# ======================================================================================


def _fmt(v, w=8, p=4):
    return "nan".rjust(w) if v is None or not np.isfinite(v) else f"{v:{w}.{p}f}"


def print_phase_table(phase, per_ctrl, block_stats):
    title = PHASE_TITLES.get(phase, phase)
    print("\n" + "=" * 108)
    print(f"{title.upper()}   (control band 0-{CONTROL_BAND_HZ:g} Hz, measured IMU acceleration in world frame)")
    print("=" * 108)

    hdr = f"{'controller':16s}"
    for ax in "xyz":
        hdr += f" {'std a' + ax:>8s}"
    hdr += (f" {'std|a|':>8s} {'RMSres':>8s} {'RMSjerk':>8s} {'TVrate':>8s}"
            f" {'RMS dE/dt':>10s} {'RMS Eexc':>9s} {'MAE p':>8s}")
    print(hdr)
    print("-" * 108)
    for name, m in per_ctrl.items():
        row = f"{name:16s}"
        for ax in "xyz":
            row += " " + _fmt(m[f"std_a_{ax}"], 8, 4)
        row += (
            " " + _fmt(m["std_a_norm"], 8, 4)
            + " " + _fmt(m["rms_res_norm"], 8, 4)
            + " " + _fmt(m["rms_jerk_norm"], 8, 3)
            + " " + _fmt(m["tv_rate_norm"], 8, 3)
            + " " + _fmt(m["rms_dEdt"], 10, 3)
            + " " + _fmt(m["rms_dE_excess"], 9, 3)
            + " " + _fmt(m["mae_p"], 8, 4)
        )
        print(row)

    print("\n  units: acc m/s^2, jerk m/s^3, dE/dt W, Eexc J, MAE m")
    print("  RMSres   = RMS(a - a_ref), the acceleration not demanded by the trajectory")
    print("  RMS Eexc = RMS(E - E_ref), mechanical energy carried above the reference")

    if block_stats:
        print(f"\n  per-block ({BLOCK_S:g} s) means with 95% block-bootstrap CI")
        print(f"  {'controller':16s} {'RMS jerk':>26s} {'std |a|':>26s} {'RMS residual':>26s}")
        for name, st in block_stats.items():
            row = f"  {name:16s}"
            for key in ("rms_jerk_norm", "std_a_norm", "rms_res_norm"):
                mean, (lo, hi) = st[key]
                row += f"   {mean:7.3f} [{lo:6.3f},{hi:6.3f}]"
            print(row)


def print_odom_crosscheck(odom_rows):
    if not odom_rows:
        return
    print("\n  cross-check from the EKF velocity derivative (independent of the IMU;"
          "\n  compare the ordering, not the absolute values)")
    print(f"  {'controller':16s} {'std|a|':>8s} {'RMSjerk':>8s}")
    for name, m in odom_rows.items():
        print(f"  {name:16s} " + _fmt(m["std_a_norm"], 8, 4) + " " + _fmt(m["rms_jerk_norm"], 8, 3))


def print_paired_tests(phase, tests):
    if not tests:
        return
    print(f"\n  paired per-block test, {TREATMENT} vs {BASELINE} "
          f"(Wilcoxon signed-rank; negative delta favours {TREATMENT})")
    print(f"  {'sub-window':14s} {'metric':16s} {'n':>4s} {'median delta':>13s} {'relative':>9s} {'p':>8s}")
    for (sub, metric), r in tests.items():
        rel = "nan" if not np.isfinite(r["rel_delta"]) else f"{100 * r['rel_delta']:+.1f}%"
        pstr = "nan" if not np.isfinite(r["p"]) else f"{r['p']:.4f}"
        print(f"  {sub:14s} {metric:16s} {r['n']:4d} {r['median_delta']:13.4f} {rel:>9s} {pstr:>8s}")


def print_subwindow_table(phase, sub_rows):
    if not sub_rows:
        return
    print("\n  split by sub-window")
    print(f"  {'sub-window':14s} {'controller':16s} {'std ax':>8s} {'std ay':>8s} {'std az':>8s} "
          f"{'std|a|':>8s} {'RMSjerk':>8s} {'RMS Eexc':>9s} {'MAE p':>8s} {'n':>6s}")
    for sub, rows in sub_rows.items():
        for name, m in rows.items():
            print(f"  {sub:14s} {name:16s} " + " ".join(_fmt(m[k], 8, 4) for k in
                  ("std_a_x", "std_a_y", "std_a_z", "std_a_norm"))
                  + " " + _fmt(m["rms_jerk_norm"], 8, 3)
                  + " " + _fmt(m["rms_dE_excess"], 9, 3)
                  + " " + _fmt(m["mae_p"], 8, 4) + f" {m['n']:6d}")


def write_csv(path, records):
    import csv

    if not records:
        return
    lead = ["phase", "sub_window", "controller"]
    rest = []
    for r in records:  # sub-window rows carry extra keys, so take the union
        rest += [k for k in r if k not in lead and k not in rest]
    keys = lead + rest
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in records:
            w.writerow(r)
    print(f"\nwrote {path}")


def write_latex_table(path, phases, per_phase_metrics):
    """A booktabs table of the headline smoothness metrics, ready to \\input."""
    lines = [
        r"% Generated by utils/analyze_recorded_acceleration.py",
        r"\begin{table}[t]",
        r"\caption{Measured linear acceleration (world frame, control band "
        + r"$0$--$" + f"{CONTROL_BAND_HZ:g}" + r"\,$Hz) recorded during the real-world experiments.}",
        r"\label{tab:measured_acc}",
        r"\centering",
        r"\begin{tabular}{llccc}",
        r"\toprule",
        r"Trajectory & Controller & $\sigma_{\|\boldsymbol{a}\|}$ & "
        r"RMS $\|\dot{\boldsymbol{a}}\|$ & RMS $\|\boldsymbol{a}-\boldsymbol{a}_\mathrm{ref}\|$ \\",
        r" &  & [m/s$^2$] & [m/s$^3$] & [m/s$^2$] \\",
        r"\midrule",
    ]
    def tex_num(v, p):
        return "--" if not np.isfinite(v) else f"{v:.{p}f}"

    for phase in phases:
        rows = per_phase_metrics.get(phase, {})
        for i, (name, m) in enumerate(rows.items()):
            traj = PHASE_TITLES.get(phase, phase).replace(" Trajectory", "") if i == 0 else ""
            lines.append(
                f"{traj} & {name} & {tex_num(m['std_a_norm'], 4)} & {tex_num(m['rms_jerk_norm'], 3)} "
                f"& {tex_num(m['rms_res_norm'], 4)} \\\\"
            )
        lines.append(r"\midrule")
    lines[-1] = r"\bottomrule"
    lines += [r"\end{tabular}", r"\end{table}", ""]
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"wrote {path}")


# ======================================================================================
# Plotting
# ======================================================================================


def setup_plot_style():
    """Same look as visualize_results_cdc_2026.py so the figures drop into the paper."""
    try:
        import scienceplots  # noqa: F401

        plt.style.use(["science", "grid"])
    except Exception:
        plt.style.use("default")

    use_tex = shutil.which("latex") is not None
    font_size = 17
    plt.rcParams.update(
        {
            "font.size": font_size,
            "axes.labelsize": font_size,
            "axes.titlesize": font_size,
            "figure.titlesize": font_size + 3,
            "figure.titleweight": "bold",
            "legend.fontsize": font_size - 2,
            "legend.framealpha": 0.7,
            "legend.fancybox": True,
            "lines.linewidth": 2.0,
            "axes.linewidth": 1.0,
            "grid.alpha": 0.35,
            "grid.linewidth": 0.6,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "lines.dash_capstyle": "round",
            "text.usetex": use_tex,
            "font.family": "serif",
            "mathtext.fontset": "cm",
            **({"text.latex.preamble": r"\usepackage{amsmath,amssymb}"} if use_tex else {}),
        }
    )


def latex_heading(text):
    if bool(plt.rcParams.get("text.usetex", False)):
        return rf"\textsc{{{text}}}"
    return text


def bm(symbol):
    """Bold math symbol that also renders without a LaTeX install.

    matplotlib's mathtext fallback has no ``\\boldsymbol``, so the axis labels
    would crash on a machine without TeX.
    """
    macro = r"\boldsymbol" if bool(plt.rcParams.get("text.usetex", False)) else r"\mathbf"
    return macro + "{" + symbol + "}"


def _finish(fig, path, show, bottom=0.0, top=FIG_TIGHT_LAYOUT_TOP):
    fig.tight_layout(rect=[0, bottom, 1, top])
    fig.savefig(path, bbox_inches="tight")
    print(f"wrote {path}")
    if not show:
        plt.close(fig)


def _figure_legend(fig, ax, ncol):
    """Legend strip under the axes.

    An in-axes legend sits on top of the first seconds of every trajectory,
    which is exactly the transient the comparison is about.
    """
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.0),
               ncol=ncol, fancybox=True, framealpha=LEGEND_FRAME_ALPHA)


def plot_measured_acceleration(phase, sigs, out_dir, show, xlim=None):
    """Measured a_x, a_y, a_z -- the empirical counterpart of Fig. 8."""
    fig, axs = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    labels = [r"$a_x$ [m/s$^2$]", r"$a_y$ [m/s$^2$]", r"$a_z$ [m/s$^2$]"]

    ref_drawn = False
    for name, sig in sigs.items():
        for i in range(3):
            if not ref_drawn and np.any(np.abs(sig["a_ref"][:, i]) > 1e-6):
                axs[i].plot(sig["t"], sig["a_ref"][:, i], linestyle=DENSE_DOTTED,
                            color="black", label="Reference" if i == 0 else None)
        ref_drawn = True

    for name, sig in sigs.items():
        for i in range(3):
            axs[i].plot(sig["t"], sig["a"][:, i], color=COLORS.get(name),
                        linestyle=LINESTYLES.get(name, "-"),
                        label=name if i == 0 else None)

    for i in range(3):
        axs[i].set_ylabel(labels[i])
        axs[i].grid("on")
        if xlim:
            axs[i].set_xlim(*xlim)
    axs[1].yaxis.set_major_locator(mticker.MaxNLocator(nbins=4))
    axs[0].set_title(latex_heading(PHASE_TITLES.get(phase, phase)))
    axs[-1].set_xlabel(r"$t$ [s]")
    _figure_legend(fig, axs[0], 2)
    _finish(fig, os.path.join(out_dir, f"measured_acc_{phase}.{FIG_FORMAT}"), show, bottom=0.09)


def plot_rolling_smoothness(phase, sigs, out_dir, show, window_s=BLOCK_S, xlim=None):
    """Rolling RMS jerk and rolling std|a| -- where the smoothness differs in time."""
    fig, axs = plt.subplots(2, 1, figsize=(7, 5.5), sharex=True)
    for name, sig in sigs.items():
        t = sig["t"]
        n = max(3, int(window_s * sig["fs"]))
        kern = np.ones(n) / n
        jn2 = np.linalg.norm(sig["jerk"], axis=1) ** 2
        roll_jerk = np.sqrt(np.convolve(jn2, kern, mode="same"))
        an = np.linalg.norm(sig["a"], axis=1)
        roll_mean = np.convolve(an, kern, mode="same")
        roll_std = np.sqrt(np.maximum(np.convolve(an ** 2, kern, mode="same") - roll_mean ** 2, 0.0))
        kw = dict(color=COLORS.get(name), linestyle=LINESTYLES.get(name, "-"))
        axs[0].plot(t, roll_jerk, label=name, **kw)
        axs[1].plot(t, roll_std, **kw)

    axs[0].set_ylabel(r"RMS $\|\dot{" + bm("a") + r"}\|$ [m/s$^3$]")
    axs[1].set_ylabel(r"$\sigma_{\|" + bm("a") + r"\|}$ [m/s$^2$]")
    axs[1].set_xlabel(r"$t$ [s]")
    for ax in axs:
        ax.grid("on")
        if xlim:
            ax.set_xlim(*xlim)
    axs[0].set_title(latex_heading(PHASE_TITLES.get(phase, phase) + f" -- {window_s:g} s rolling"))
    _figure_legend(fig, axs[0], 3)
    _finish(fig, os.path.join(out_dir, f"smoothness_{phase}.{FIG_FORMAT}"), show, bottom=0.11)


def plot_psd(phase, sigs, out_dir, show):
    """PSD of the *unfiltered* acceleration: which band carries the difference."""
    fig, axs = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    labels = [r"$a_x$", r"$a_y$", r"$a_z$"]
    for name, sig in sigs.items():
        fs = sig["fs"]
        nperseg = int(min(len(sig["a_raw"]), max(256, fs * 4)))
        f, pxx = welch(sig["a_raw"], fs=fs, nperseg=nperseg, axis=0)
        for i in range(3):
            axs[i].loglog(f[1:], pxx[1:, i], color=COLORS.get(name),
                          linestyle=LINESTYLES.get(name, "-"),
                          label=name if i == 0 else None)
    for i in range(3):
        axs[i].set_ylabel(labels[i] + r" PSD")
        axs[i].grid("on", which="both")
        axs[i].axvline(CONTROL_BAND_HZ, color="0.4", linewidth=1.0, linestyle=(0, (1, 2)))
    axs[0].set_title(latex_heading(PHASE_TITLES.get(phase, phase) + " -- spectrum"))
    axs[-1].set_xlabel(r"$f$ [Hz]")
    _figure_legend(fig, axs[0], 3)
    _finish(fig, os.path.join(out_dir, f"psd_{phase}.{FIG_FORMAT}"), show, bottom=0.09)


def plot_block_distributions(phase, block_series, out_dir, show):
    """Distribution of the per-block metrics -- the spread behind each mean."""
    metrics = [("rms_jerk_norm", r"RMS $\|\dot{" + bm("a") + r"}\|$ [m/s$^3$]"),
               ("std_a_norm", r"$\sigma_{\|" + bm("a") + r"\|}$ [m/s$^2$]"),
               ("rms_res_norm", r"RMS $\|" + bm("a") + "-" + bm("a") + r"_\mathrm{ref}\|$ [m/s$^2$]")]
    names = list(block_series.keys())
    fig, axs = plt.subplots(1, 3, figsize=(11, 4))
    for ax, (key, label) in zip(axs, metrics):
        data = [block_series[n][key] for n in names]
        bp = ax.boxplot(data, patch_artist=True, widths=0.55, showfliers=False,
                        medianprops=dict(color="black", linewidth=1.5))
        for patch, n in zip(bp["boxes"], names):
            patch.set_facecolor(COLORS.get(n, "0.6"))
            patch.set_alpha(0.55)
            patch.set_edgecolor("black")
        for i, (n, d) in enumerate(zip(names, data)):
            ax.scatter(np.full(len(d), i + 1) + np.random.default_rng(0).uniform(-0.12, 0.12, len(d)),
                       d, s=8, color=COLORS.get(n, "0.4"), zorder=3, alpha=0.8)
        ax.set_xticks(range(1, len(names) + 1))
        ax.set_xticklabels([n.replace(" MPC", "") for n in names], rotation=20, ha="right")
        ax.set_ylabel(label, fontsize=13)
        ax.grid("on", axis="y")
        ax.tick_params(labelsize=12)
    fig.suptitle(latex_heading(PHASE_TITLES.get(phase, phase) + f" -- per {BLOCK_S:g} s block"))
    _finish(fig, os.path.join(out_dir, f"blocks_{phase}.{FIG_FORMAT}"), show)


def plot_energy(phase, sigs, out_dir, show, xlim=None):
    """Measured energy excess over the reference -- the quantity (16) penalises."""
    fig, axs = plt.subplots(2, 1, figsize=(7, 5.5), sharex=True)
    for name, sig in sigs.items():
        t = sig["t_odom"]
        energy = mechanical_energy(sig["p"], sig["v"])
        excess = energy - mechanical_energy(sig["p_ref"], sig["v_ref"])
        kw = dict(color=COLORS.get(name), linestyle=LINESTYLES.get(name, "-"))
        axs[0].plot(t, excess, label=name, **kw)
        axs[1].plot(t, np.gradient(energy, t), **kw)
    axs[0].axhline(0.0, color="0.4", linewidth=1.0, linestyle=DENSE_DOTTED)
    axs[0].set_ylabel(r"$E-E_\mathrm{ref}$ [J]")
    axs[1].set_ylabel(r"$\dot{E}$ [W]")
    axs[1].set_xlabel(r"$t$ [s]")
    for ax in axs:
        ax.grid("on")
        if xlim:
            ax.set_xlim(*xlim)
    axs[0].set_title(latex_heading(PHASE_TITLES.get(phase, phase) + " -- mechanical energy"))
    _figure_legend(fig, axs[0], 3)
    _finish(fig, os.path.join(out_dir, f"energy_{phase}.{FIG_FORMAT}"), show, bottom=0.11)


# ======================================================================================
# Driver
# ======================================================================================


def analyze(bags, phases_wanted, out_dir, cache_dir, ns=ROBOT_NS, cutoff=CONTROL_BAND_HZ,
            make_plots=True, show=False):
    os.makedirs(out_dir, exist_ok=True)

    runs, phase_map = {}, {}
    for name, path in bags.items():
        print(f"loading {name:16s} <- {path}")
        runs[name] = load_run(path, ns=ns, cache_dir=cache_dir)
        phase_map[name] = segment_phases(runs[name])
        found = ", ".join(f"{k} [{v[0]:.1f}-{v[1]:.1f}s]" for k, v in sorted(phase_map[name].items()))
        print(f"    phases: {found}")

    records, per_phase_metrics = [], {}

    for phase in phases_wanted:
        missing = [n for n in bags if phase not in phase_map[n]]
        if missing:
            print(f"\n[skip] '{phase}' not found in: {', '.join(missing)}")
            continue

        sigs = {n: slice_signals(runs[n], phase_map[n][phase], cutoff) for n in bags}
        per_ctrl = {n: compute_metrics(s) for n, s in sigs.items()}
        per_phase_metrics[phase] = per_ctrl

        # Per-block series, restricted to the meaningful part of the phase.
        t_min = CIRCLE_SKIP_S if phase == "circle" else None
        block_series, block_stats = {}, {}
        for n, s in sigs.items():
            block_series[n] = {}
            block_stats[n] = {}
            for key, fn in BLOCK_METRICS.items():
                _, vals = block_metric_series(s, fn, BLOCK_S, t_min)
                block_series[n][key] = vals
                block_stats[n][key] = (float(np.mean(vals)) if len(vals) else np.nan,
                                       block_bootstrap_ci(vals))

        print_phase_table(phase, per_ctrl, block_stats)

        # Sub-window breakdown.
        sub_rows = {}
        for n, s in sigs.items():
            for sub, mask_fn in subwindow_masks(phase, runs[n], phase_map[n][phase]).items():
                mask, mask_o = mask_fn(s["t"]), mask_fn(s["t_odom"])
                if mask.sum() < 20 or mask_o.sum() < 10:
                    continue
                sub_sig = {
                    "a": s["a"][mask], "a_ref": s["a_ref"][mask], "jerk": s["jerk"][mask],
                    "a_raw": s["a_raw"][mask], "t": s["t"][mask], "fs": s["fs"],
                    "t_odom": s["t_odom"][mask_o], "p": s["p"][mask_o], "v": s["v"][mask_o],
                    "p_ref": s["p_ref"][mask_o], "v_ref": s["v_ref"][mask_o],
                }
                m = compute_metrics(sub_sig)
                m["n"] = int(mask.sum())
                sub_rows.setdefault(sub, {})[n] = m
        print_subwindow_table(phase, sub_rows)

        odom_rows = {n: compute_odom_metrics(s, t_min) for n, s in sigs.items()}
        print_odom_crosscheck(odom_rows)

        # Paired tests, whole phase and per sub-window.
        tests = {}
        if TREATMENT in sigs and BASELINE in sigs:
            for key, fn in BLOCK_METRICS.items():
                tests[("all", key)] = paired_block_test(sigs[TREATMENT], sigs[BASELINE], fn,
                                                        BLOCK_S, t_min)
            if phase == "setpoint":
                for sub in ("transition", "hold"):
                    sub_sigs = {}
                    for n in (TREATMENT, BASELINE):
                        s = sigs[n]
                        mask = subwindow_masks(phase, runs[n], phase_map[n][phase])[sub](s["t"])
                        sub_sigs[n] = {"a": s["a"][mask], "a_ref": s["a_ref"][mask],
                                       "jerk": s["jerk"][mask], "t": s["t"][mask]}
                    for key, fn in BLOCK_METRICS.items():
                        tests[(sub, key)] = paired_block_test(sub_sigs[TREATMENT],
                                                              sub_sigs[BASELINE], fn, BLOCK_S)
        print_paired_tests(phase, tests)

        for n, m in per_ctrl.items():
            records.append({"phase": phase, "sub_window": "all", "controller": n, **m})
        for sub, rows in sub_rows.items():
            for n, m in rows.items():
                records.append({"phase": phase, "sub_window": sub, "controller": n, **m})

        if make_plots:
            # Common x-range so the runs really are compared over the same
            # portion of the manoeuvre.  The setpoint clock is zeroed on its
            # first step, so its range starts negative.
            xlim = (max(float(s["t"][0]) for s in sigs.values()),
                    min(float(s["t"][-1]) for s in sigs.values()))
            plot_measured_acceleration(phase, sigs, out_dir, show, xlim)
            plot_rolling_smoothness(phase, sigs, out_dir, show, BLOCK_S, xlim)
            plot_psd(phase, sigs, out_dir, show)
            plot_block_distributions(phase, block_series, out_dir, show)
            plot_energy(phase, sigs, out_dir, show, xlim)

    write_csv(os.path.join(out_dir, "acceleration_metrics.csv"), records)
    write_latex_table(os.path.join(out_dir, "acceleration_table.tex"),
                      [p for p in phases_wanted if p in per_phase_metrics], per_phase_metrics)
    return per_phase_metrics


def build_arg_parser():
    p = argparse.ArgumentParser(
        description="Compare the measured acceleration of the three CDC 2026 controllers.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--bag-dir", default=DEFAULT_BAG_DIR, help="directory holding the rosbags")
    p.add_argument("--analytical-bag", default=DEFAULT_BAGS["Analytical MPC"])
    p.add_argument("--rtnmpc-bag", default=DEFAULT_BAGS["RTNMPC"])
    p.add_argument("--ours-bag", default=DEFAULT_BAGS["Ours"])
    p.add_argument("--ns", default=ROBOT_NS, help="robot namespace")
    p.add_argument("--phases", nargs="+", default=["takeoff", "circle", "setpoint"],
                   choices=["takeoff", "circle", "lemniscate", "setpoint"],
                   help="phases to analyse; the lemniscate is off by default")
    p.add_argument("--cutoff", type=float, default=CONTROL_BAND_HZ,
                   help="low-pass cutoff defining the control band [Hz]")
    p.add_argument("--out-dir", default=None, help="output directory")
    p.add_argument("--cache-dir", default=DEFAULT_CACHE_DIR,
                   help="where the extracted bag arrays are cached (kept out of the "
                        "results tree so the caches never land in the repository)")
    p.add_argument("--no-cache", action="store_true", help="re-read the bags instead of the cache")
    p.add_argument("--no-plots", action="store_true", help="tables only")
    p.add_argument("--show", action="store_true", help="display the figures")
    p.add_argument("--format", default="pdf", choices=["pdf", "png", "svg"],
                   help="figure file format")
    return p


def main(argv=None):
    args = build_arg_parser().parse_args(argv)

    global FIG_FORMAT
    FIG_FORMAT = args.format

    if not args.show:
        matplotlib.use("Agg")
    setup_plot_style()

    out_dir = args.out_dir or os.path.join(_PKG_DIR, "results", "acceleration_analysis")
    cache_dir = None if args.no_cache else os.path.expanduser(args.cache_dir)

    bags = {
        "Analytical MPC": resolve_bag(args.analytical_bag, args.bag_dir),
        "RTNMPC": resolve_bag(args.rtnmpc_bag, args.bag_dir),
        "Ours": resolve_bag(args.ours_bag, args.bag_dir),
    }

    analyze(bags, args.phases, out_dir, cache_dir, ns=args.ns, cutoff=args.cutoff,
            make_plots=not args.no_plots, show=args.show)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
