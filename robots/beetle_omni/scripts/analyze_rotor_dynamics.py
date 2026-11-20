#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Rotor dynamics analysis for the rotor bench
Identification of the ESC telemetry noise and of the closed-loop as a first-order and second-order system.

Seen from the onboard PC, the ESC (RPM mode) and the rotor form a single-input single-output system
    input   u(t)   target RPM of the ESC, commanded through spinal's PWM test mode (/pwm_test)
    output  y(t)   rotor speed (RPM) and rotor acceleration (RPM rate) in the ESC telemetry (/esc_telem)

Analysis steps, each one derived in the docstring of the function or class named in brackets:
  1. Signal chain     exact target RPM of the ESC, telemetry resolution and time base   [esc_target_rpm, read_bag]
  2. Noise            RPM and RPM rate noise while the target is held, noise spectra,   [find_holds, measure_hold_noise,
                      delay and gain of the RPM rate against the differentiated RPM      noise_spectra, check_rpm_rate]
  3. Models           closed speed loop derived from the rotor and the ESC speed PID,   [RotorModel and subclasses]
                      reduced to first- and second-order models with dead time
  4. Fitting          output-error least squares on the training data                   [simulate, fit_model]
  5. Validation       fit quality on training and held-out data, step responses, Bode   [fit_metrics, measured_steps,
                      plot against the empirical frequency response                      estimate_frequency_response]
  6. Operating point  refits on each step split show how the dynamics change with RPM   [fit_operating_points]

Data: bags recorded during
  `rosrun beetle_omni rotor_bench_test.py`   segments labelled on /rotor_bench_test/segment
  `roslaunch motor_test test.launch`         no labels, one run per PWM step
each with /pwm_test and /esc_telem. Training data are the bags given as positional arguments without their held-out
segments (heldout/*, which rotor_bench_test.py cuts out of the step splits on purpose). Validation data are these
held-out segments and every bag given with --validation.

Usage:
  `rosrun beetle_omni analyze_rotor_dynamics.py STEPS.bag CHIRP.bag --validation MOTOR_TEST.bag --save-dir DIR`
Optional args (see --help), e.g.:
  --motor-index 2                          # spinal 0-based motor index, default: the motor the bags command most
  --time-source bag                        # telemetry time = host receive time instead of the spinal stamp
  --models first_order second_order_zero   # subset of the models
  --no-show                                # only save the report and the figures

Output: a text report (stdout, and DIR/report.txt) and the figures noise, step_response, bode, fit_zoom and
fit_bag<i> for every bag (shown, and saved as PNG to DIR).
"""

import argparse
import collections
import itertools
import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import rosbag
import rospkg
import yaml
from scipy import optimize, signal

from utils import matlab_blue, matlab_green, matlab_orange, matlab_purple, matlab_yellow

try:
    import scienceplots  # noqa: F401 - registers the "science" style

    # The Computer Modern font (cmr10) of the science style has no "_" and ">" glyphs, which bag and segment names need
    PLOT_STYLE = ["science", "grid", "no-latex", {"font.serif": ["STIXGeneral"], "mathtext.fontset": "stix"}]
except ImportError:
    PLOT_STYLE = ["default"]

COMMAND_TOPIC = "/pwm_test"
TELEMETRY_TOPIC = "/esc_telem"
SEGMENT_TOPIC = "/rotor_bench_test/segment"
UNLABELED = "all"  # segment name of every sample of a bag without segment labels

# Signal chain, see esc_target_rpm() and read_bag()
IDLE_PWM = 0.5  # spinal IDLE_DUTY, decoded as rpm_min by the ESC
MAX_PWM = 1.0  # spinal PWM_TEST_MAX_DUTY, decoded as rpm_max by the ESC
DSHOT_MIN_THROTTLE = 48  # spinal dshot.h and AM32 drive_by_rpm: 48 -> rpm_min, 2047 -> rpm_max
DSHOT_RANGE = 1999  # 2047 - 48
POLE_PAIRS = 7  # motor_poles / 2 in the ESC settings and in spinal
RPM_STEP = 10.0 / POLE_PAIRS  # [RPM] resolution of the telemetry RPM (eRPM / 10 field)
RPM_RATE_STEP = 100.0 / POLE_PAIRS  # [RPM/s] resolution of the telemetry RPM rate (eRPM/s / 100 field)
ESC_PID_PERIOD = 1.05e-3  # [s] period T_s of the AM32 speed PID, see RotorModel

# Analysis
MIN_RUN_DURATION = 0.5  # [s] shorter closed-loop runs are skipped
# [s] hold and step windows end this long before the next target change: the telemetry stamps can lead the command
# times (negative apparent dead time, see read_bag), so their last samples could already show the next change
TARGET_CHANGE_MARGIN = 0.05
FIT_MIN_STD = 50.0  # [RPM] the fit % of a segment whose RPM varies less would only measure noise
STEP_PRE = 0.2  # [s] a measured step needs a constant target this long before the step
STEP_POST = 0.5  # [s] and this long after it
SETTLING_BAND = 0.05  # settling time band of the step responses, 5 % of the step

BAG_COLORS = [matlab_blue, matlab_orange, matlab_green, matlab_purple, matlab_yellow]


# ---------------------------------------------------------------------------
# Rotor models
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Parameter:
    """A model parameter: SI value inside the fit, `value / scale` in `unit` in the report."""

    name: str
    unit: str
    scale: float
    lower: float  # bounds of the fit (SI)
    upper: float
    guesses: Tuple[float, ...]  # initial values of the multi-start fit (SI)
    fmt: str = ".3f"


GAIN = Parameter("K", "-", 1.0, 0.5, 2.0, (1.0,), ".4f")
DELAY = Parameter("theta", "ms", 1e-3, -0.05, 0.1, (-0.005, 0.01), ".1f")
NATURAL_FREQUENCY = Parameter("wn", "rad/s", 1.0, 5.0, 2000.0, (50.0, 150.0), ".1f")
DAMPING = Parameter("zeta", "-", 1.0, 0.05, 10.0, (0.7, 1.5), ".3f")


class RotorModel:
    """Base class of the LTI models of the closed speed loop, from the target RPM u to the telemetry RPM y.

    Theory: rotor and ESC speed loop
      Rotor   J dΩ/dt = k_t i - k_q Ω²: motor torque against the aerodynamic drag torque of the propeller, with the
              winding current i = (d V_b - k_e Ω) / R for the throttle (duty) d at the battery voltage V_b. The
              electrical time constant L / R (< 0.1 ms) lies far below the mechanical one and is neglected.
              Linearized at the operating speed Ω₀:
                  dδΩ/dt = -a δΩ + b δd,     a = (k_t k_e / R + 2 k_q Ω₀) / J,     b = k_t V_b / (R J)
                  P(s) = δΩ(s) / δd(s) = b / (s + a)
              The plant pole a grows with Ω₀: the bare rotor responds faster at a higher RPM.
      ESC     AM32 drive_by_rpm (main.c) runs the speed PID every T_s = 1.05 ms on the error e_k = u_k - y_k and
              accumulates its output into the throttle:
                  d_k = d_{k-1} + (K_p e_k + K_d (e_k - e_{k-1})) / 10⁴
              Summed up, d_k = (K_p / 10⁴) Σ_j e_j + (K_d / 10⁴) e_k. So the firmware's "P" gain acts as integral and
              its "D" gain as proportional gain of a PI controller on the throttle (in DShot counts 0..2047):
                  C(s) = k_P + k_I / s = k_P (s + ω_z) / s,
                  k_P = K_d / 10⁴,     k_I = K_p / (10⁴ T_s),     ω_z = k_I / k_P = K_p / (K_d T_s)
              e.g. P65 D3000: k_P = 0.3 counts/RPM, k_I = 6.2 counts/(RPM s), ω_z = 20.6 rad/s (3.3 Hz).
      Loop    The telemetry reports the filtered speed the PID controls, so with P and C:
                  T(s) = C P / (1 + C P) = b k_P (s + ω_z) / (s² + (a + b k_P) s + b k_P ω_z)
              a second-order system with the zero -ω_z of the PI controller and the static gain T(0) = 1: the
              integral action removes every steady-state error. Not in T(s) are the 15 Hz alpha-beta filter of the
              speed inside the loop, the zero-crossing averaging and the command and telemetry latencies. They add
              phase lag and a steep roll-off above ~15 Hz, which the reduced models below can only approximate
              (by θ and, in G₁ and G₂, by a slower pole). Their bandwidth is therefore an upper bound.

    Reduced models (subclasses), each with a free static gain K (≈ 1 expected) and dead time θ:
      FirstOrderModel        G₁(s) = K / (τ s + 1) · e^(-θs)
      SecondOrderModel       G₂(s) = K ω_n² / (s² + 2ζω_n s + ω_n²) · e^(-θs)
      SecondOrderZeroModel   G₃(s) = K ω_n² (s / ω_z + 1) / (s² + 2ζω_n s + ω_n²) · e^(-θs)
    G₃ has the structure of T(s). G₁ is T(s) when the PI zero cancels the plant pole (ω_z = a, then
    T(s) = 1 / (s / (b k_P) + 1)). G₂ is the classical second-order model that keeps the two poles but drops the zero.

    A subclass defines its parameters and the delay-free part G₀(s) = N(s) / D(s) of G(s) = G₀(s) e^(-θs).
    """

    key = ""  # command line name
    name = ""  # display name
    parameters: Tuple[Parameter, ...] = ()

    def transfer_function(self, p: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        """Numerator N and denominator D of G₀(s), coefficients of the highest power of s first."""
        raise NotImplementedError

    def initial_guesses(self) -> List[np.ndarray]:
        """Every combination of the parameter guesses, the starts of the multi-start fit."""
        return [np.array(x) for x in itertools.product(*(p.guesses for p in self.parameters))]


class FirstOrderModel(RotorModel):
    """First-order lag with dead time (FOPDT), G₁(s) = K / (τ s + 1) · e^(-θs).

    Time domain:  τ dy/dt + y(t) = K u(t - θ)
    Step response to a target step Δu at t = 0:  y(t) = y(0) + K Δu (1 - e^(-(t - θ) / τ)) for t ≥ θ
    Parameters:
        K   static gain [-], 1 if the rotor tracks the target without steady-state error
        τ   time constant [s]: 63 % of a step is reached τ after θ, the -3 dB bandwidth is 1 / (2π τ)
        θ   apparent dead time [s] from the command on the host to the telemetry stamp (see read_bag)
    """

    key = "first_order"
    name = "1st order"
    parameters = (GAIN, Parameter("tau", "ms", 1e-3, 1e-3, 1.0, (0.01, 0.03), ".1f"), DELAY)

    def transfer_function(self, p):
        return np.array([p["K"]]), np.array([p["tau"], 1.0])


class SecondOrderModel(RotorModel):
    """Second-order lag with dead time (SOPDT), G₂(s) = K ω_n² / (s² + 2ζω_n s + ω_n²) · e^(-θs).

    Time domain:  d²y/dt² + 2ζω_n dy/dt + ω_n² y(t) = K ω_n² u(t - θ)
    Poles s = -ω_n (ζ ± sqrt(ζ² - 1)): two real poles with the time constants 1 / |s| for ζ ≥ 1, an oscillating pair
    with the step overshoot exp(-π ζ / sqrt(1 - ζ²)) for ζ < 1.
    Parameters:
        K     static gain [-]
        ω_n   natural frequency [rad/s]
        ζ     damping ratio [-], 1 / √2 gives the flattest magnitude without a resonance peak
        θ     apparent dead time [s]
    """

    key = "second_order"
    name = "2nd order"
    parameters = (GAIN, NATURAL_FREQUENCY, DAMPING, DELAY)

    def transfer_function(self, p):
        wn, zeta = p["wn"], p["zeta"]
        return np.array([p["K"] * wn**2]), np.array([1.0, 2.0 * zeta * wn, wn**2])


class SecondOrderZeroModel(RotorModel):
    """Second-order lag with a zero and dead time, G₃(s) = K ω_n² (s / ω_z + 1) / (s² + 2ζω_n s + ω_n²) · e^(-θs).

    The structure of the closed speed loop T(s) (see RotorModel) with
        ω_n² = b k_P ω_z,     2ζω_n = a + b k_P,     ω_z = k_I / k_P
    Time domain:  d²y/dt² + 2ζω_n dy/dt + ω_n² y(t) = K ω_n² (u(t - θ) + du/dt(t - θ) / ω_z)
    The zero passes a target step on directly (the proportional path of the PI controller), so the rotor rises faster
    than its poles alone allow. If ω_z lies close to the slower pole, the two form a doublet: a quick rise to near the
    target, followed by a slow creep in which the integral action removes the rest of the error (time constant
    ~1 / ω_z).
    Parameters:
        K     static gain [-]
        ω_n   natural frequency of the poles [rad/s]
        ζ     damping ratio of the poles [-]
        ω_z   zero [rad/s], expected at the PI zero K_p / (K_d T_s) of the ESC speed PID
        θ     apparent dead time [s]
    """

    key = "second_order_zero"
    name = "2nd order + zero"
    parameters = (GAIN, NATURAL_FREQUENCY, DAMPING, Parameter("wz", "rad/s", 1.0, 1.0, 2000.0, (20.0,), ".1f"), DELAY)

    def transfer_function(self, p):
        wn, zeta, wz = p["wn"], p["zeta"], p["wz"]
        return np.array([p["K"] * wn**2 / wz, p["K"] * wn**2]), np.array([1.0, 2.0 * zeta * wn, wn**2])


MODELS: Dict[str, RotorModel] = {m.key: m for m in (FirstOrderModel(), SecondOrderModel(), SecondOrderZeroModel())}
MODEL_COLORS = {"first_order": matlab_blue, "second_order": matlab_orange, "second_order_zero": matlab_green}


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
@dataclass
class AnalysisConfig:
    """Settings of the analysis, the most relevant ones are command line options (see parse_args)."""

    # Data
    motor_index: Optional[int] = None  # spinal 0-based motor index, None: the motor the bags command most often
    rpm_min: float = 900.0  # [RPM] ESC RPM window, from MotorInfoDShot.yaml by default
    rpm_max: float = 13300.0  # [RPM]
    time_source: str = "stamp"  # time of the telemetry: "stamp" (spinal header stamp) or "bag" (host receive time)
    startup_skip: float = 1.0  # [s] cut after every switch-on, covers the open-loop start-up of the ESC
    heldout_prefix: str = "heldout"  # segments of the training bags with this prefix are validation data

    # Noise
    hold_settle: float = 0.5  # [s] cut after every target change before the noise is measured
    min_hold: float = 1.0  # [s] shortest hold window (after hold_settle) that is measured
    noise_band: float = 2000.0  # [RPM] width of the RPM bands of the noise table and spectra
    noise_window: float = 1.0  # [s] Welch segment of the noise spectra
    rate_check_window: float = 0.06  # [s] Savitzky-Golay window of the differentiated RPM
    rate_check_max_lag: float = 0.1  # [s] largest lag searched between RPM rate and differentiated RPM

    # Identification
    models: List[str] = field(default_factory=lambda: list(MODELS))
    sim_dt: float = 0.5e-3  # [s] simulation grid
    split_prefixes: List[str] = field(default_factory=lambda: ["steps/", "heldout/"])  # operating-point refits
    esc_gains: Optional[Tuple[float, float, float]] = None  # AM32 speed PID (K_p, K_i, K_d), default: from bag name

    # Validation
    frf_windows: Dict[str, float] = field(default_factory=lambda: {"chirp": 4.0, "steps": 2.0})  # prefix: Welch [s]
    min_coherence: float = 0.8  # empirical frequency response points below are drawn faint
    min_step: float = 1000.0  # [RPM] smallest target step in the step response comparison
    # (segment prefix, start [s] (< 0: before the end of the segment), duration [s]) of the zoomed fit plots
    zoom_windows: List[Tuple[str, float, float]] = field(
        default_factory=lambda: [
            ("steps/middle", 0.0, 4.0),
            ("heldout/below_hover", 0.0, 4.0),
            ("sine", 0.0, 2.0),
            ("chirp", -2.0, 2.0),
        ]
    )


def load_rpm_window(motor_info_file: str) -> Tuple[float, float]:
    """min_rpm / max_rpm of a MotorInfoDShot.yaml, the RPM window the ESC decodes the PWM onto."""
    with open(motor_info_file) as f:
        motor_info = yaml.safe_load(f)["motor_info"]
    return float(motor_info["min_rpm"]), float(motor_info["max_rpm"])


def default_motor_info_file() -> str:
    try:
        package_dir = rospkg.RosPack().get_path("beetle_omni")
    except rospkg.ResourceNotFound:
        package_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    return os.path.join(package_dir, "config", "MotorInfoDShot.yaml")


def parse_esc_gains(bag_paths: Sequence[str]) -> Optional[Tuple[float, float, float]]:
    """AM32 speed PID gains (K_p, K_i, K_d) from a bag name like ..._P65_I0_D3000_..., None if no name has them."""
    for path in bag_paths:
        match = re.search(r"_P(\d+)_I(\d+)_D(\d+)", os.path.basename(path))
        if match:
            return tuple(float(g) for g in match.groups())
    return None


# ---------------------------------------------------------------------------
# Data: signal chain
# ---------------------------------------------------------------------------
@dataclass(eq=False)
class BagRecording:
    """Signals of one motor in one bag, times in seconds since the bag start."""

    path: str
    label: str  # "[i]", short name in tables and plots
    validation: bool  # the whole bag is validation data
    t: np.ndarray  # [s] telemetry time (header stamp or receive time, see AnalysisConfig.time_source)
    rpm: np.ndarray  # [RPM] rotor speed y
    rpm_rate: np.ndarray  # [RPM/s] ESC estimate of dy/dt
    t_cmd: np.ndarray  # [s] command time (receive time of /pwm_test, i.e. its publish time on the host)
    u_cmd: np.ndarray  # [RPM] target RPM u of the ESC, NaN while the motor is off
    segments: List[Tuple[float, str]]  # (start time [s], name) from /rotor_bench_test/segment
    uplink_latency: np.ndarray  # [s] receive time - header stamp of every telemetry message
    dropped_frames: int  # telemetry messages without a valid frame of this motor

    @property
    def name(self) -> str:
        return os.path.splitext(os.path.basename(self.path))[0]


def esc_target_rpm(pwm: np.ndarray, rpm_min: float, rpm_max: float) -> np.ndarray:
    """Target RPM u that the ESC tracks for a PWM test command, NaN where the command switches the motor off.

    The command passes two integer conversions. spinal (AttitudeController, DShot output) rounds the PWM to a DShot
    throttle, and the ESC (AM32 setInput, drive_by_rpm) maps the throttle onto its RPM window by rounded division:
        d = floor(48 + (pwm - 0.5) / 0.5 · 1999 + 0.5)                     DShot throttle in [48, 2047]
        u = rpm_min + floor(((d - 48) (rpm_max - rpm_min) + 999) / 1999)   integer RPM, steps of ~6.2 RPM
    spinal switches the motor off for a PWM outside [0.5, 1.0].
    """
    pwm = np.asarray(pwm, dtype=float)
    with np.errstate(invalid="ignore"):
        on = (pwm >= IDLE_PWM) & (pwm <= MAX_PWM)  # NaN compares False
    frac = (np.where(on, pwm, IDLE_PWM) - IDLE_PWM) / (MAX_PWM - IDLE_PWM)
    throttle = np.floor(DSHOT_MIN_THROTTLE + frac * DSHOT_RANGE + 0.5)
    target = rpm_min + np.floor(
        ((throttle - DSHOT_MIN_THROTTLE) * (rpm_max - rpm_min) + DSHOT_RANGE // 2) / DSHOT_RANGE
    )
    return np.where(on, target, np.nan)


def command_pwm(msg, motor_index: int) -> Optional[float]:
    """PWM that a spinal/PwmTest message commands to `motor_index`, as AttitudeController::pwmTestCallback reads it.

    An empty message leaves PWM test mode (motor off, NaN). An empty motor_index list commands pwms[0] to every motor.
    Otherwise only the listed motors change, so a message without this motor returns None (no change).
    """
    if not msg.pwms:
        return math.nan
    if not msg.motor_index:
        return msg.pwms[0]
    indices = list(msg.motor_index)  # uint8[] is deserialized as bytes
    if motor_index in indices and len(indices) == len(msg.pwms):
        return msg.pwms[indices.index(motor_index)]
    return None


def detect_motor_index(bag_paths: Sequence[str]) -> int:
    """The motor index that the /pwm_test messages of the bags list most often."""
    counts = collections.Counter()
    for path in bag_paths:
        with rosbag.Bag(path) as bag:
            for _, msg, _ in bag.read_messages(topics=[COMMAND_TOPIC]):
                counts.update(list(msg.motor_index))
    if not counts:
        raise ValueError(f"No motor index on {COMMAND_TOPIC} in the bags, set --motor-index")
    return counts.most_common(1)[0][0]


def read_bag(path: str, label: str, validation: bool, motor_index: int, cfg: AnalysisConfig) -> BagRecording:
    """Read the commands, the telemetry of one motor and the segment labels of a bag.

    Input: every command is converted to the exact target RPM of the ESC, see esc_target_rpm().
    Output: spinal decodes the KISS telemetry frame of the ESC (esc_telem.cpp) into
        rpm      = round(10 · eRPM field / p)     [RPM],     resolution q_y = 10 / p ≈ 1.43 RPM
        rpm_rate = 100 · eRPM-rate field / p      [RPM/s],   resolution q_r = 100 / p ≈ 14.3 RPM/s
    with p = 7 pole pairs. Messages with crc_error != 0 repeat the last valid values and are dropped.

    Time base: a command is taken at its receive time in the bag. rotor_bench_test.py and motor_test publish on the
    recording host, so this is their publish time within ~1 ms. The telemetry is taken at its header stamp (default),
    which spinal sets when it publishes the frame, in its clock that rosserial synchronizes to the host. The stamps are
    regular (8 ± 1 ms). But rosserial assumes a symmetric serial latency, so the spinal clock can be offset against the
    host clock by some ms. A constant offset only shifts the apparent dead time θ of the models (it can even become
    negative). The dynamics (τ, ω_n, ζ, ω_z) do not depend on it. With time_source = "bag" the telemetry is taken at
    its receive time on the host instead: θ then contains the whole uplink latency that host software sees, at the
    cost of the receive jitter.
    All times count from the bag start: absolute UNIX times (~1.8e9 s) resolve only 2.4e-7 s in float64, which is
    coarser than the finite-difference steps that the fit takes in θ.
    """
    telemetry_field = f"esc_telemetry_{motor_index + 1}"
    t_stamp, t_recv, rpm, rpm_rate = [], [], [], []
    t_cmd, pwm_cmd, segments = [], [], []
    dropped = 0
    with rosbag.Bag(path) as bag:
        t_ref = bag.get_start_time()
        for topic, msg, t_bag in bag.read_messages(topics=[COMMAND_TOPIC, TELEMETRY_TOPIC, SEGMENT_TOPIC]):
            t_bag = t_bag.to_sec() - t_ref
            if topic == TELEMETRY_TOPIC:
                esc = getattr(msg, telemetry_field)
                if esc.crc_error != 0:
                    dropped += 1
                    continue
                t_stamp.append(msg.stamp.to_sec() - t_ref)
                t_recv.append(t_bag)
                rpm.append(esc.rpm)
                rpm_rate.append(esc.rpm_rate)
            elif topic == COMMAND_TOPIC:
                pwm = command_pwm(msg, motor_index)
                if pwm is not None:
                    t_cmd.append(t_bag)
                    pwm_cmd.append(pwm)
            else:
                segments.append((t_bag, msg.data))
    if not rpm or not t_cmd:
        raise ValueError(f"{path}: no {TELEMETRY_TOPIC} or {COMMAND_TOPIC} messages for motor {motor_index}")

    t_stamp, t_recv = np.array(t_stamp), np.array(t_recv)
    t = t_stamp if cfg.time_source == "stamp" else t_recv
    order = np.argsort(t, kind="stable")
    return BagRecording(
        path=path,
        label=label,
        validation=validation,
        t=t[order],
        rpm=np.array(rpm, dtype=float)[order],
        rpm_rate=np.array(rpm_rate, dtype=float)[order],
        t_cmd=np.array(t_cmd),
        u_cmd=esc_target_rpm(np.array(pwm_cmd), cfg.rpm_min, cfg.rpm_max),
        segments=segments,
        uplink_latency=t_recv - t_stamp,
        dropped_frames=dropped,
    )


# ---------------------------------------------------------------------------
# Data: closed-loop runs
# ---------------------------------------------------------------------------
@dataclass(eq=False)
class Run:
    """One closed-loop interval of a bag: the motor is commanded without a break, the ESC start-up is cut off."""

    bag: BagRecording
    t: np.ndarray  # [s] telemetry samples inside the run window
    rpm: np.ndarray  # [RPM]
    rpm_rate: np.ndarray  # [RPM/s]
    segment: np.ndarray  # segment name of every sample
    validation: np.ndarray  # True for the validation samples
    t_cmd: np.ndarray  # [s] commands from switch-on to the last one before switch-off
    u_cmd: np.ndarray  # [RPM]

    def target_at(self, t: np.ndarray) -> np.ndarray:
        """Target RPM at the times t: the ESC holds every command until the next one (zero-order hold)."""
        idx = np.searchsorted(self.t_cmd, t, side="right") - 1
        return self.u_cmd[np.clip(idx, 0, None)]

    def in_segment(self, prefix: str) -> np.ndarray:
        return np.char.startswith(self.segment, prefix)


def contiguous_stretches(mask: np.ndarray) -> List[Tuple[int, int]]:
    """Index ranges [i0, i1) of the stretches of True in `mask`."""
    edges = np.flatnonzero(np.diff(np.concatenate([[False], mask, [False]]).astype(int)))
    return list(zip(edges[::2], edges[1::2]))


def label_segments(t: np.ndarray, segments: List[Tuple[float, str]]) -> np.ndarray:
    """Name of the segment that each time t lies in, from the (start, name) labels of rotor_bench_test.py."""
    if not segments:
        return np.full(len(t), UNLABELED)
    starts = np.array([s for s, _ in segments])
    names = np.array([n for _, n in segments] + ["start"])  # index -1: before the first label
    return names[np.searchsorted(starts, t, side="right") - 1]


def split_runs(rec: BagRecording, cfg: AnalysisConfig) -> List[Run]:
    """Cut a recording into its closed-loop runs.

    A run lasts from a switch-on (first valid command) to the next switch-off (empty command) or the end of the bag.
    Its first startup_skip seconds are cut: the ESC starts the motor open-loop and only closes the speed loop after
    ~30 zero crossings, and the RPM rate estimate starts after 100.
    """
    runs = []
    on = ~np.isnan(rec.u_cmd)
    for i0, i1 in contiguous_stretches(on):
        t_start = rec.t_cmd[i0] + cfg.startup_skip
        t_end = rec.t_cmd[i1] if i1 < len(rec.t_cmd) else rec.t[-1] + 1.0
        m = (rec.t >= t_start) & (rec.t < t_end)
        if not m.any() or rec.t[m][-1] - rec.t[m][0] < MIN_RUN_DURATION:
            continue
        segment = label_segments(rec.t[m], rec.segments)
        validation = np.full(m.sum(), rec.validation) | np.char.startswith(segment, cfg.heldout_prefix)
        runs.append(
            Run(rec, rec.t[m], rec.rpm[m], rec.rpm_rate[m], segment, validation, rec.t_cmd[i0:i1], rec.u_cmd[i0:i1])
        )
    return runs


def telemetry_rate(runs: List[Run]) -> float:
    """[Hz] median telemetry rate of the runs, the uniform resampling rate of the spectral analyses."""
    return 1.0 / np.median(np.concatenate([np.diff(run.t) for run in runs]))


# ---------------------------------------------------------------------------
# Noise
# ---------------------------------------------------------------------------
@dataclass
class HoldNoise:
    """Noise of the RPM and the RPM rate in one hold window."""

    run: Run
    t_start: float  # [s]
    t_end: float  # [s]
    target: float  # [RPM]
    rpm_mean: float  # [RPM]
    rpm_std: float  # [RPM]
    rpm_p2p: float  # [RPM]
    rate_mean: float  # [RPM/s] bias b
    rate_std: float  # [RPM/s]
    rate_p2p: float  # [RPM/s]
    n: int


def find_holds(run: Run, settle: float, min_hold: float) -> List[Tuple[float, float, float]]:
    """Windows (start, end, target) of a run in which the target is constant and the rotor has settled.

    After a target change the closed loop settles within a few time constants of its slowest pole (≲ 50 ms, see the
    fitted models), so after `settle` seconds the transient has decayed below the noise and
        y(t_k) = ȳ + v_k,          rpm_rate(t_k) = b + w_k
    with the constant speed ȳ, the RPM noise v, and the noise w and bias b of the RPM rate (its true value is zero).
    Each window ends TARGET_CHANGE_MARGIN before the next change. Ramps, sines and chirps have no holds, only the step
    sequences, rests and the motor_test steps do.
    """
    holds = []
    change = np.flatnonzero(np.diff(run.u_cmd) != 0) + 1
    starts = np.concatenate([[0], change])
    ends = np.concatenate([change, [len(run.u_cmd)]])
    for i0, i1 in zip(starts, ends):
        t0 = max(run.t_cmd[i0] + settle, run.t[0])
        t1 = run.t_cmd[i1] - TARGET_CHANGE_MARGIN if i1 < len(run.t_cmd) else run.t[-1]
        if t1 - t0 >= min_hold:
            holds.append((t0, t1, run.u_cmd[i0]))
    return holds


def measure_hold_noise(run: Run, t0: float, t1: float, target: float) -> HoldNoise:
    """Noise statistics of the RPM and the RPM rate in the hold window [t0, t1).

    The RPM noise v_k is the residual of a straight-line fit y_k ≈ c₀ + c₁ t_k, which also removes a slow drift that is
    no noise of the measurement. The RPM rate keeps its mean as the bias b, the rate noise is w_k = rpm_rate_k - b.
    Reported per hold:
        σ   = sqrt(mean(v²))         standard deviation (RMS) of the noise
        p2p = max(v) - min(v)        peak-to-peak noise
    Lower bound: the rounding to the telemetry resolution q alone, with an error uniform in ±q/2, gives σ_q = q / √12.
    """
    m = (run.t >= t0) & (run.t < t1)
    t, rpm, rate = run.t[m], run.rpm[m], run.rpm_rate[m]
    tc = t - t.mean()
    v = rpm - np.polyval(np.polyfit(tc, rpm, 1), tc)
    w = rate - rate.mean()
    return HoldNoise(
        run=run,
        t_start=t0,
        t_end=t1,
        target=target,
        rpm_mean=rpm.mean(),
        rpm_std=v.std(),
        rpm_p2p=np.ptp(v),
        rate_mean=rate.mean(),
        rate_std=w.std(),
        rate_p2p=np.ptp(w),
        n=int(m.sum()),
    )


def rpm_band(rpm: float, band: float) -> Tuple[float, float]:
    lo = math.floor(rpm / band) * band
    return lo, lo + band


@dataclass
class NoiseSpectrum:
    band: Tuple[float, float]  # [RPM]
    f: np.ndarray  # [Hz]
    rpm_asd: np.ndarray  # [RPM/√Hz]
    rate_asd: np.ndarray  # [RPM/s/√Hz]
    n_windows: int


def noise_spectra(holds: List[HoldNoise], fs: float, window: float, band: float) -> List[NoiseSpectrum]:
    """Amplitude spectral density (ASD) of the RPM and RPM rate noise, per RPM band.

    Each hold is resampled to the uniform telemetry rate fs, detrended (linear for the RPM, mean for the rate, as in
    measure_hold_noise), and split into Welch segments of `window` seconds (Hann window, 50 % overlap). The one-sided
    power spectral densities S(f) of all segments of a band are averaged and shown as ASD = sqrt(S). By Parseval,
    σ² = ∫ S(f) df over [0, fs/2], so the ASD shows at which frequencies the noise power sits: white noise is flat,
    filtered noise falls off above the filter frequency (the 15 Hz alpha-beta filter of the telemetry RPM, the 60 ms
    Savitzky-Golay window of the RPM rate), and narrow peaks are periodic disturbances, e.g. the beat of the
    commutation against the motor PWM at some RPMs.
    """
    nperseg = int(round(window * fs))
    sums: Dict[Tuple[float, float], List] = {}
    for hold in holds:
        t = np.arange(hold.t_start, hold.t_end, 1.0 / fs)
        if len(t) < nperseg:
            continue
        rpm = signal.detrend(np.interp(t, hold.run.t, hold.run.rpm), type="linear")
        rate = signal.detrend(np.interp(t, hold.run.t, hold.run.rpm_rate), type="constant")
        f, s_rpm = signal.welch(rpm, fs, nperseg=nperseg)
        _, s_rate = signal.welch(rate, fs, nperseg=nperseg)
        n = (len(t) - nperseg) // (nperseg // 2) + 1  # Welch segments of the hold
        acc = sums.setdefault(rpm_band(hold.rpm_mean, band), [f, 0.0, 0.0, 0])
        acc[1] += n * s_rpm
        acc[2] += n * s_rate
        acc[3] += n
    return [
        NoiseSpectrum(key, f, np.sqrt(s_rpm / n), np.sqrt(s_rate / n), n)
        for key, (f, s_rpm, s_rate, n) in sorted(sums.items())
    ]


@dataclass
class RateCheck:
    """Delay and gain of the telemetry RPM rate against the differentiated telemetry RPM."""

    lag: float  # [s] the RPM rate is behind the derivative by this
    gain: float  # [-]
    rms_difference: float  # [RPM/s] RMS of rpm_rate - gain · dy/dt(t - lag)
    rms_derivative: float  # [RPM/s] RMS of dy/dt, the scale of the signal
    lags: np.ndarray  # [s] searched lags
    correlation: np.ndarray  # normalized cross-correlation at the lags
    derivative: np.ndarray  # [RPM/s] dy/dt(t - lag) of all runs
    rate: np.ndarray  # [RPM/s] rpm_rate of all runs


def check_rpm_rate(runs: List[Run], fs: float, window: float, max_lag: float) -> Optional[RateCheck]:
    """Delay λ and gain g of the telemetry RPM rate against the time derivative of the telemetry RPM.

    The ESC estimates dy/dt from the zero-crossing times by a 60 ms Savitzky-Golay fit, which describes the acceleration
    at the window centre, i.e. about 30 ms in the past (rpm_rate.c). Its reference here is the derivative of the
    measured RPM from a centred (zero-phase) Savitzky-Golay filter of `window` seconds, which has no delay:
        r(t) = d/dt SG[y](t)
    λ is the lag that maximizes the normalized cross-correlation of all runs (sub-sample by a parabola through the peak)
        ρ(λ) = Σ_k rpm_rate(t_k) r(t_k - λ) / sqrt(Σ_k rpm_rate(t_k)² · Σ_k r(t_k - λ)²)
    and g is the least-squares slope of rpm_rate(t) over r(t - λ). The RMS of rpm_rate - g r(t - λ) bounds the error of
    the RPM rate in motion from above: it also contains the noise of r and the different smoothing of both estimates.
    None if no run is long enough.
    """
    n_window = max(5, int(round(window * fs)) // 2 * 2 + 1)
    n_lag = int(round(max_lag * fs))
    lags = np.arange(-n_lag, n_lag + 1)
    pairs = []
    for run in runs:
        t = np.arange(run.t[0], run.t[-1], 1.0 / fs)
        if len(t) <= n_window + 2 * n_lag:
            continue
        derivative = signal.savgol_filter(np.interp(t, run.t, run.rpm), n_window, 2, deriv=1, delta=1.0 / fs)
        pairs.append((t, derivative, np.interp(t, run.t, run.rpm_rate)))
    if not pairs:
        return None

    def shifted(lag):  # rate(t_k) and derivative(t_k - lag), the edges of each run cut by n_lag samples
        k = slice(n_lag, -n_lag)
        return (
            np.concatenate([rate[k] for _, _, rate in pairs]),
            np.concatenate([np.interp(t[k] - lag, t, d) for t, d, _ in pairs]),
        )

    correlation = []
    for lag in lags / fs:
        rate, derivative = shifted(lag)
        correlation.append(np.dot(rate, derivative) / np.sqrt(np.dot(rate, rate) * np.dot(derivative, derivative)))
    correlation = np.array(correlation)

    i = int(np.clip(np.argmax(correlation), 1, len(lags) - 2))
    c_left, c_peak, c_right = correlation[i - 1 : i + 2]
    curvature = c_left - 2.0 * c_peak + c_right
    offset = 0.5 * (c_left - c_right) / curvature if curvature < 0 else 0.0
    lag = (lags[i] + offset) / fs

    rate, derivative = shifted(lag)
    gain = np.dot(rate, derivative) / np.dot(derivative, derivative)
    return RateCheck(
        lag=lag,
        gain=gain,
        rms_difference=np.sqrt(np.mean((rate - gain * derivative) ** 2)),
        rms_derivative=np.sqrt(np.mean(derivative**2)),
        lags=lags / fs,
        correlation=correlation,
        derivative=derivative,
        rate=rate,
    )


# ---------------------------------------------------------------------------
# Simulation and fitting
# ---------------------------------------------------------------------------
@dataclass
class SimulationGrid:
    """Uniform fine time grid of one run with the target on it, built once per run and reused by every simulation."""

    dt: float
    t: np.ndarray  # [s]
    u: np.ndarray  # [RPM] target, zero-order hold of the commands
    y0: float  # [RPM] measured RPM at the start of the run, the initial steady state


def make_grid(run: Run, dt: float) -> SimulationGrid:
    # The margins let the simulation be read at t - θ for every θ within the bounds of DELAY
    t = np.arange(run.t[0] - DELAY.upper - dt, run.t[-1] - DELAY.lower + 2 * dt, dt)
    return SimulationGrid(dt, t, run.target_at(t), float(np.median(run.rpm[:5])))


def simulate(num: np.ndarray, den: np.ndarray, delay: float, grid: SimulationGrid, t_out: np.ndarray) -> np.ndarray:
    """Simulated telemetry RPM ŷ of the model G(s) = N(s) / D(s) · e^(-θs) at the times t_out.

    Input: the ESC holds every target until the next command, so u(t) is piecewise constant (zero-order hold), and the
    delay-free part G₀ = N / D is discretized exactly on the grid Δ by the zero-order-hold transformation
        x_{k+1} = A_d x_k + B_d u_k,     A_d = e^(A Δ),     B_d = ∫₀^Δ e^(A σ) dσ B
    of a state-space realization (A, B, C, D) of G₀ (scipy cont2discrete), run as a difference equation (lfilter).
    Only the command times are rounded to the grid, by at most Δ/2.
    Dead time: y(t) = y₀(t - θ) for the delay-free response y₀, read from the grid by linear interpolation. This keeps
    the cost of the fit continuous in θ, while shifting the input instead would quantize θ to whole grid steps.
    Initial state: steady state at the measured RPM of the run start, i.e. y₀(t₀) = y(t₀) with all derivatives zero.
    rotor_bench_test.py rests before every segment group, so its runs start in a hold where this is exact. Otherwise
    (motor_test ramps up right after the start-up) the error of the initial state decays with the poles of the model.
    """
    num_d, den_d, _ = signal.cont2discrete((num, den), grid.dt, method="zoh")
    num_d = np.squeeze(num_d)
    dc_gain = num[-1] / den[-1]
    zi = signal.lfilter_zi(num_d, den_d) * grid.y0 / dc_gain
    y, _ = signal.lfilter(num_d, den_d, grid.u, zi=zi)
    return np.interp(t_out - delay, grid.t, y)


@dataclass
class FittedModel:
    model: RotorModel
    params: Dict[str, float]  # SI values

    @property
    def num_den(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.model.transfer_function(self.params)

    @property
    def delay(self) -> float:
        return self.params["theta"]

    def simulate(self, grid: SimulationGrid, t: np.ndarray) -> np.ndarray:
        num, den = self.num_den
        return simulate(num, den, self.delay, grid, t)

    def frequency_response(self, f: np.ndarray) -> np.ndarray:
        """G(j 2π f) including the dead time."""
        num, den = self.num_den
        w = 2.0 * np.pi * f
        return signal.freqs(num, den, worN=w)[1] * np.exp(-1j * w * self.delay)

    def step_response(self, t: np.ndarray) -> np.ndarray:
        """Response to a unit target step at t = 0, i.e. the step response of G₀ delayed by θ."""
        num, den = self.num_den
        t0 = np.arange(0.0, t[-1] - self.delay + 1e-4, 1e-4)
        _, y0 = signal.step((num, den), T=t0)
        return np.interp(t - self.delay, t0, y0, left=0.0)

    def format_params(self) -> List[str]:
        return [f"{self.params[p.name] / p.scale:{p.fmt}}" for p in self.model.parameters]


def fit_model(
    model: RotorModel,
    runs: List[Run],
    grids: List[SimulationGrid],
    masks: List[np.ndarray],
    starts: Optional[List[np.ndarray]] = None,
) -> FittedModel:
    """Output-error (simulation-error) least-squares fit of a model to the samples selected by `masks`.

    Estimate:  p̂ = argmin_p Σ_k (y_k - ŷ_k(p))²
    where ŷ_k(p) is simulated from the commands alone (simulate), never from measured outputs. For
    y_k = (G u)(t_k) + v_k with white Gaussian noise v, p̂ is the maximum-likelihood estimate. With coloured noise, as
    here, it stays consistent as long as v is uncorrelated with u, unlike an equation-error (ARX) fit. And the minimized
    error is exactly the error the model makes when it predicts the rotor speed from the command, e.g. in a controller.
    Standard errors from the Jacobian (σ² (JᵀJ)⁻¹) assume white residuals and would be far too optimistic here, so the
    spread of the operating-point refits (fit_operating_points) is reported instead.
    Each run is simulated from its start, so the state at a selected sample carries the history of the whole run.
    Solver: trust-region reflective least squares (scipy least_squares) within the parameter bounds, with Jacobian
    scaling, from every initial guess in `starts` (default: model.initial_guesses()). The lowest cost wins.
    """
    names = [p.name for p in model.parameters]
    lower = [p.lower for p in model.parameters]
    upper = [p.upper for p in model.parameters]
    data = [(run, grid, mask) for run, grid, mask in zip(runs, grids, masks) if mask.any()]

    def residuals(x):
        p = dict(zip(names, x))
        num, den = model.transfer_function(p)
        return np.concatenate([simulate(num, den, p["theta"], g, r.t[m]) - r.rpm[m] for r, g, m in data])

    best = None
    for x0 in starts if starts is not None else model.initial_guesses():
        sol = optimize.least_squares(residuals, np.clip(x0, lower, upper), bounds=(lower, upper), x_scale="jac")
        if best is None or sol.cost < best.cost:
            best = sol
    return FittedModel(model, dict(zip(names, best.x)))


@dataclass
class OperatingPointFit:
    split: str  # segment name
    rpm_mean: float  # [RPM] mean RPM of the split
    fits: Dict[str, FittedModel]  # model key: refit
    rmse: Dict[str, float]  # model key: RMSE of the refit on the split [RPM]


def fit_operating_points(
    fits: Dict[str, FittedModel], runs: List[Run], grids: List[SimulationGrid], prefixes: List[str]
) -> List[OperatingPointFit]:
    """Refit every model on each operating-point split (the step sequences of rotor_bench_test.py) on its own.

    The linearization in RotorModel depends on the operating speed Ω₀ (the plant pole a), so the closed-loop dynamics
    change with the RPM. A single global model is a compromise: the spread of the parameters over the splits shows how
    large it is, and serves as an empirical uncertainty of the global parameters. Each refit starts from the global
    fit and simulates the whole run, but only the samples of the split enter the cost.
    """
    splits = dict.fromkeys(s for run in runs for s in run.segment if any(s.startswith(p) for p in prefixes))
    results = []
    for split in splits:
        masks = [run.segment == split for run in runs]
        rpm_mean = np.concatenate([run.rpm[m] for run, m in zip(runs, masks)]).mean()
        refits, rmse = {}, {}
        for key, fit in fits.items():
            x0 = np.array([fit.params[p.name] for p in fit.model.parameters])
            refits[key] = fit_model(fit.model, runs, grids, masks, starts=[x0])
            e = np.concatenate([refits[key].simulate(g, r.t[m]) - r.rpm[m] for r, g, m in zip(runs, grids, masks)])
            rmse[key] = np.sqrt(np.mean(e**2))
        results.append(OperatingPointFit(split, rpm_mean, refits, rmse))
    return results


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
@dataclass
class FitMetrics:
    n: int
    rmse: float  # [RPM]
    fit: float  # [%], NaN for a flat segment
    max_error: float  # [RPM]


def fit_metrics(y: np.ndarray, y_hat: np.ndarray) -> FitMetrics:
    """Fit quality of the simulated RPM ŷ against the measured RPM y, with the error e = y - ŷ:
    RMSE    = sqrt(mean(e²))                     [RPM], at least the RPM noise σ even for a perfect model
    fit     = 100 (1 - ‖e‖ / ‖y - mean(y)‖)      [%], normalized RMSE fit (as in MATLAB compare): 100 % is a
                                                 perfect simulation, 0 % no better than the constant mean. Only
                                                 for segments whose RPM varies (std(y) ≥ FIT_MIN_STD), in a flat
                                                 segment only the noise would be left
    max |e|                                      [RPM], the worst sample
    """
    e = y - y_hat
    spread = np.linalg.norm(y - y.mean())
    fit = 100.0 * (1.0 - np.linalg.norm(e) / spread) if y.std() >= FIT_MIN_STD else math.nan
    return FitMetrics(len(y), np.sqrt(np.mean(e**2)), fit, np.abs(e).max())


@dataclass
class QualityRow:
    bag: str
    segment: str
    validation: bool
    metrics: Dict[str, FitMetrics]  # model key: metrics


def quality_table(runs: List[Run], predictions: Dict[str, List[np.ndarray]]) -> List[QualityRow]:
    """Fit metrics of every model per bag and segment, then over all training and all validation samples."""
    y = np.concatenate([run.rpm for run in runs])
    bag = np.concatenate([np.full(len(run.t), run.bag.label) for run in runs])
    segment = np.concatenate([run.segment for run in runs])
    validation = np.concatenate([run.validation for run in runs])
    y_hat = {key: np.concatenate(pred) for key, pred in predictions.items()}

    def row(bag_label, segment_name, mask):
        metrics = {key: fit_metrics(y[mask], y_hat[key][mask]) for key in y_hat}
        return QualityRow(bag_label, segment_name, bool(validation[mask].all()), metrics)

    rows = []
    for bag_label in dict.fromkeys(bag):
        for segment_name in dict.fromkeys(segment[bag == bag_label]):
            rows.append(row(bag_label, segment_name, (bag == bag_label) & (segment == segment_name)))
    for name, mask in (("training", ~validation), ("validation", validation)):
        if mask.any():
            rows.append(row("total", name, mask))
    return rows


@dataclass
class FrequencyResponseEstimate:
    source: str  # segment prefix
    f: np.ndarray  # [Hz]
    h: np.ndarray  # complex frequency response u -> y
    coherence: np.ndarray  # γ²
    n_windows: int


def estimate_frequency_response(
    runs: List[Run], prefix: str, window: float, fs: float
) -> Optional[FrequencyResponseEstimate]:
    """Non-parametric frequency response Ĥ(f) of u -> y from the samples of the segments that start with `prefix`.

    H1 estimator, for y = G u + v with output noise v uncorrelated with u:
        Ĥ(f) = Ŝ_uy(f) / Ŝ_uu(f),        γ²(f) = |Ŝ_uy(f)|² / (Ŝ_uu(f) Ŝ_yy(f)) ∈ [0, 1]
    with the Welch estimates of the cross and auto spectra (Hann window of `window` seconds, 50 % overlap), summed over
    all segments, weighted by their number of windows. Ĥ is not biased by v. The coherence γ² is the share of the output
    power that the input explains linearly: γ² ≈ 1 where the excitation dominates, lower where noise or nonlinearity
    do. The relative random error of |Ĥ| is ≈ sqrt(1 - γ²) / (|γ| sqrt(2 n)) for n windows.
    Prewhitening: the spectra are taken of the first differences Δu, Δy. A filter applied to both u and y cancels in Ĥ
    and γ², but this one flattens the ~1/f spectra of steps and ramps, which reduces the leakage of the windows.
    Signals: u is the target (zero-order hold) and y the telemetry RPM (linear interpolation) on the uniform telemetry
    rate fs. Ĥ contains the same apparent dead time as the models (see read_bag).
    """
    nperseg = int(round(window * fs))
    s_uu = s_uy = s_yy = 0.0
    n_total = 0
    f = None
    for run in runs:
        for i0, i1 in contiguous_stretches(run.in_segment(prefix)):
            t = np.arange(run.t[i0], run.t[i1 - 1], 1.0 / fs)
            if len(t) <= nperseg:
                continue
            du = np.diff(run.target_at(t))
            dy = np.diff(np.interp(t, run.t, run.rpm))
            n = (len(du) - nperseg) // (nperseg // 2) + 1
            f, p_uu = signal.welch(du, fs, nperseg=nperseg)
            _, p_uy = signal.csd(du, dy, fs, nperseg=nperseg)  # conj(U) Y
            _, p_yy = signal.welch(dy, fs, nperseg=nperseg)
            s_uu, s_uy, s_yy = s_uu + n * p_uu, s_uy + n * p_uy, s_yy + n * p_yy
            n_total += n
    if n_total == 0:
        return None
    keep = f > 0
    with np.errstate(divide="ignore", invalid="ignore"):
        h = s_uy[keep] / s_uu[keep]
        coherence = np.abs(s_uy[keep]) ** 2 / (s_uu[keep] * s_yy[keep])
    return FrequencyResponseEstimate(prefix, f[keep], h, coherence, n_total)


@dataclass
class MeasuredStep:
    run: Run
    t_step: float  # [s] command time of the step
    u_before: float  # [RPM]
    u_after: float  # [RPM]
    t: np.ndarray  # [s] time since the step
    y: np.ndarray  # normalized response, 0 before and 1 after the step


def measured_steps(runs: List[Run], min_step: float) -> List[MeasuredStep]:
    """Measured step responses: target steps of at least min_step RPM, held STEP_PRE before and STEP_POST after.

    Each response is normalized as (y - y_before) / (y_after - y_before), with y_before the mean RPM over 0.1 s before
    the step and y_after the mean over the last 0.2 s of the hold (which ends TARGET_CHANGE_MARGIN before the next
    change). For an LTI model, every normalized response equals the unit step response G(s) / (K s), so the spread of
    the measured curves shows how far the rotor is from linear time-invariant.
    """
    steps = []
    for run in runs:
        change = np.flatnonzero(np.diff(run.u_cmd) != 0) + 1
        t_change = np.concatenate([[run.t_cmd[0]], run.t_cmd[change], [run.t[-1]]])
        for j, i in enumerate(change, 1):
            t_step, t_prev, t_next = t_change[j], t_change[j - 1], t_change[j + 1]
            u_before, u_after = run.u_cmd[i - 1], run.u_cmd[i]
            if abs(u_after - u_before) < min_step or t_step - t_prev < STEP_PRE or t_next - t_step < STEP_POST:
                continue
            m = (run.t >= t_step - STEP_PRE) & (run.t < t_next - TARGET_CHANGE_MARGIN)
            t, y = run.t[m] - t_step, run.rpm[m]
            if t[0] > -0.1:  # the run window starts too late
                continue
            y_before, y_after = y[(t >= -0.1) & (t < 0.0)].mean(), y[t >= t[-1] - 0.2].mean()
            steps.append(MeasuredStep(run, t_step, u_before, u_after, t, (y - y_before) / (y_after - y_before)))
    return steps


def step_characteristics(t: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    """Rise time (10 -> 90 %), overshoot [-] and settling time (SETTLING_BAND) of a step response normalized from 0 to
    1, times counted from the command at t = 0."""

    def first_crossing(level):
        i = int(np.argmax(y >= level))
        if y[i] < level:
            return math.nan
        if i == 0:
            return t[0]
        return t[i - 1] + (level - y[i - 1]) * (t[i] - t[i - 1]) / (y[i] - y[i - 1])

    outside = np.flatnonzero(np.abs(y - 1.0) > SETTLING_BAND)
    settling = t[min(outside[-1] + 1, len(t) - 1)] if len(outside) else t[0]
    return first_crossing(0.9) - first_crossing(0.1), max(0.0, y.max() - 1.0), settling


def bandwidth(fit: FittedModel) -> float:
    """[Hz] first frequency at which |G| falls 3 dB below the static gain K."""
    f = np.logspace(-2, 3, 5000)
    mag = np.abs(fit.frequency_response(f)) / fit.params["K"]
    below = np.flatnonzero(mag < 1.0 / np.sqrt(2.0))
    if len(below) == 0 or below[0] == 0:
        return math.nan
    i = below[0]
    x = (mag[i - 1] - 1.0 / np.sqrt(2.0)) / (mag[i - 1] - mag[i])
    return 10 ** (np.log10(f[i - 1]) + x * (np.log10(f[i]) - np.log10(f[i - 1])))


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
def format_table(header: List[str], rows: List[List[str]], n_left: int = 1) -> str:
    """Plain text table, the first n_left columns aligned left, the others right."""
    widths = [max(len(str(c)) for c in col) for col in zip(header, *rows)]

    def line(cells):
        return "  ".join(
            f"{str(c):<{w}}" if i < n_left else f"{str(c):>{w}}" for i, (c, w) in enumerate(zip(cells, widths))
        )

    return "\n".join([line(header), line(["-" * w for w in widths])] + [line(r) for r in rows])


def format_value(value: float, spec: str = ".1f") -> str:
    return "-" if value is None or (isinstance(value, float) and math.isnan(value)) else f"{value:{spec}}"


def format_poles(values: np.ndarray) -> str:
    def one(z):
        return f"{z.real:.1f}" if abs(z.imag) < 1e-6 * max(1.0, abs(z)) else f"{z.real:.1f}{z.imag:+.1f}j"

    return ", ".join(one(z) for z in sorted(values, key=lambda z: (-z.real, z.imag)) if z.imag >= -1e-9)


def report_data(recordings: List[BagRecording], runs: List[Run], cfg: AnalysisConfig, motor_index: int) -> str:
    lines = [
        f"Motor index {motor_index} (esc_telemetry_{motor_index + 1}), ESC RPM window [{cfg.rpm_min:.0f}, "
        f"{cfg.rpm_max:.0f}], telemetry time: {cfg.time_source}",
    ]
    rows = []
    for rec in recordings:
        bag_runs = [run for run in runs if run.bag is rec]
        n = sum(len(run.t) for run in bag_runs)
        n_val = sum(int(run.validation.sum()) for run in bag_runs)
        dt = np.diff(rec.t)
        rows.append(
            [
                rec.label,
                rec.name,
                "validation" if rec.validation else "training",
                str(len(bag_runs)),
                f"{sum(run.t[-1] - run.t[0] for run in bag_runs):.1f}",
                f"{n - n_val} / {n_val}",
                f"{1.0 / np.median(dt):.1f}",
                str(rec.dropped_frames),
                f"{1e3 * np.median(rec.uplink_latency):.1f} / {1e3 * np.percentile(rec.uplink_latency, 95):.1f}",
            ]
        )
    header = ["bag", "file", "role", "runs", "time [s]", "train/val samples", "rate [Hz]", "dropped", "latency [ms]"]
    lines.append(format_table(header, rows, n_left=3))
    lines.append("latency: median / 95 % of telemetry receive time - spinal stamp")
    return "\n".join(lines)


def report_noise(holds: List[HoldNoise], rate_check: Optional[RateCheck], cfg: AnalysisConfig) -> str:
    if not holds:
        return "No holds: the bags have no constant targets of at least hold_settle + min_hold seconds"
    lines = [
        f"{len(holds)} holds (target constant, {cfg.hold_settle:.1f} s settling cut, >= {cfg.min_hold:.1f} s), "
        f"noise after linear detrend.",
        f"Quantization floor: RPM σ_q = {RPM_STEP / np.sqrt(12):.2f} RPM, "
        f"rate σ_q = {RPM_RATE_STEP / np.sqrt(12):.1f} RPM/s",
    ]
    rows = []
    groups: Dict[Tuple[float, float], List[HoldNoise]] = {}
    for hold in holds:
        groups.setdefault(rpm_band(hold.rpm_mean, cfg.noise_band), []).append(hold)
    for key, group in sorted(groups.items()) + [(None, holds)]:
        n = np.array([h.n for h in group])
        rows.append(
            [
                "all" if key is None else f"{key[0]:.0f}-{key[1]:.0f}",
                str(len(group)),
                f"{sum(h.t_end - h.t_start for h in group):.1f}",
                f"{np.sqrt(np.sum(n * np.array([h.rpm_std for h in group]) ** 2) / n.sum()):.2f}",
                f"{max(h.rpm_std for h in group):.2f}",
                f"{max(h.rpm_p2p for h in group):.0f}",
                f"{np.sqrt(np.sum(n * np.array([h.rate_std for h in group]) ** 2) / n.sum()):.0f}",
                f"{max(h.rate_std for h in group):.0f}",
                f"{max(h.rate_p2p for h in group):.0f}",
                f"{np.sum(n * np.array([h.rate_mean for h in group])) / n.sum():+.1f}",
            ]
        )
    header = ["RPM band", "holds", "time [s]", "RPM σ", "σ max", "p2p max", "rate σ", "σ max", "p2p max", "rate bias"]
    lines.append(format_table(header, rows))
    lines.append("RPM in [RPM], rate in [RPM/s], σ pooled over the holds of the band")
    worst = sorted(holds, key=lambda h: h.rpm_std, reverse=True)[:5]
    lines.append(
        "Noisiest holds (target: RPM σ / rate σ): "
        + ", ".join(f"{h.target:.0f}: {h.rpm_std:.1f} / {h.rate_std:.0f}" for h in worst)
    )
    if rate_check is None:
        return "\n".join(lines)
    lines.append(
        f"RPM rate vs. differentiated RPM ({cfg.rate_check_window * 1e3:.0f} ms Savitzky-Golay, all runs): "
        f"delay {rate_check.lag * 1e3:.1f} ms, gain {rate_check.gain:.3f}, RMS difference "
        f"{rate_check.rms_difference:.0f} RPM/s (RMS of dy/dt {rate_check.rms_derivative:.0f} RPM/s)"
    )
    return "\n".join(lines)


def report_models(
    fits: Dict[str, FittedModel], steps: List[MeasuredStep], esc_gains: Optional[Tuple[float, float, float]]
) -> str:
    lines = ["Fitted on all training samples (output-error least squares):"]
    for fit in fits.values():
        header = [p.name + f" [{p.unit}]" for p in fit.model.parameters]
        lines.append(f"\n  {fit.model.name}")
        lines.append("    " + format_table(header, [fit.format_params()]).replace("\n", "\n    "))
        num, den = fit.num_den
        lines.append(f"    poles [rad/s]: {format_poles(np.roots(den))}")
        if len(num) > 1:
            lines.append(f"    zeros [rad/s]: {format_poles(np.roots(num))}")
    if esc_gains is not None:
        kp, ki, kd = esc_gains
        if ki == 0 and kd > 0:
            lines.append(
                f"\nESC speed PID P{kp:.0f} I{ki:.0f} D{kd:.0f}: PI zero expected at "
                f"K_p / (K_d T_s) = {kp / (kd * ESC_PID_PERIOD):.1f} rad/s (see RotorModel)"
            )

    lines.append("\nStep response characteristics (times from the command, rise 10-90 %, settling 5 %):")
    t = np.arange(-0.05, 1.5, 1e-4)
    rows = []
    for fit in fits.values():
        rise, overshoot, settling = step_characteristics(t, fit.step_response(t) / fit.params["K"])
        rows.append(
            [
                fit.model.name,
                f"{rise * 1e3:.1f}",
                f"{overshoot * 100:.1f}",
                f"{settling * 1e3:.0f}",
                format_value(bandwidth(fit)),
            ]
        )
    for direction, up in (("up", True), ("down", False)):
        chars = np.array([step_characteristics(s.t, s.y) for s in steps if (s.u_after > s.u_before) == up])
        if len(chars) == 0:
            continue
        q25, q50, q75 = np.nanpercentile(chars, [25, 50, 75], axis=0)
        cells = [
            f"{q50[i] * scale:{spec}} ({q25[i] * scale:{spec}}-{q75[i] * scale:{spec}})"
            for i, scale, spec in ((0, 1e3, ".1f"), (1, 1e2, ".1f"), (2, 1e3, ".0f"))
        ]
        rows.append([f"measured {direction} steps (n={len(chars)})"] + cells + ["-"])
    header = ["", "rise [ms]", "overshoot [%]", "settling [ms]", "-3 dB [Hz]"]
    lines.append(format_table(header, rows))
    lines.append("measured: median (interquartile range)")
    return "\n".join(lines)


def report_quality(rows: List[QualityRow], fits: Dict[str, FittedModel]) -> str:
    header = ["bag", "segment", "role", "n"]
    for fit in fits.values():
        header += [f"{fit.model.name}: RMSE", "fit %", "max |e|"]
    table = []
    for row in rows:
        cells = [row.bag, row.segment, "val" if row.validation else "train", str(next(iter(row.metrics.values())).n)]
        for key in fits:
            m = row.metrics[key]
            cells += [f"{m.rmse:.1f}", format_value(m.fit), f"{m.max_error:.0f}"]
        table.append(cells)
    return format_table(header, table, n_left=3) + "\nRMSE and max |error| in [RPM], fit % = NRMSE fit"


def report_operating_points(op_fits: List[OperatingPointFit], fits: Dict[str, FittedModel]) -> str:
    lines = []
    for key, fit in fits.items():
        header = ["split", "mean RPM"] + [f"{p.name} [{p.unit}]" for p in fit.model.parameters] + ["RMSE"]
        rows = [
            [op.split, f"{op.rpm_mean:.0f}"] + op.fits[key].format_params() + [f"{op.rmse[key]:.1f}"] for op in op_fits
        ]
        rows.append(["global (training)", "-"] + fit.format_params() + ["-"])
        lines.append(f"  {fit.model.name}")
        lines.append("    " + format_table(header, rows).replace("\n", "\n    "))
    return "\n".join(lines)


def section(title: str, body: str) -> str:
    return f"{title}\n{'=' * len(title)}\n{body}\n"


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
def plot_noise(
    holds: List[HoldNoise], spectra: List[NoiseSpectrum], rate_check: Optional[RateCheck], cfg: AnalysisConfig
):
    fig, axes = plt.subplots(3, 2, figsize=(11, 11))
    bags = list(dict.fromkeys(h.run.bag for h in holds))
    for ax, attr, q, unit in (
        (axes[0, 0], "rpm_std", RPM_STEP, "RPM"),
        (axes[0, 1], "rate_std", RPM_RATE_STEP, "RPM/s"),
    ):
        for i, bag in enumerate(bags):
            sel = [h for h in holds if h.run.bag is bag]
            ax.plot(
                [h.rpm_mean for h in sel],
                [getattr(h, attr) for h in sel],
                "os^D"[i % 4],
                color=BAG_COLORS[i % len(BAG_COLORS)],
                ms=4,
                ls="none",
                label=f"{bag.label} {bag.name[:19]}",
            )
        ax.axhline(q / np.sqrt(12), color="k", ls=":", lw=1, label="quantization $q/\\sqrt{12}$")
        ax.set_xlabel("RPM")
        ax.set_ylabel(f"noise $\\sigma$ [{unit}]")
        ax.set_ylim(bottom=0)
        ax.legend(fontsize=7)
    axes[0, 0].set_title("RPM noise per hold")
    axes[0, 1].set_title("RPM rate noise per hold")

    cmap = plt.get_cmap("viridis")
    for spec in spectra:
        color = cmap((spec.band[0] + 0.5 * cfg.noise_band) / cfg.rpm_max)
        label = f"{spec.band[0]:.0f}-{spec.band[1]:.0f} RPM ({spec.n_windows})"
        axes[1, 0].semilogy(spec.f, spec.rpm_asd, color=color, lw=1, label=label)
        axes[1, 1].semilogy(spec.f, spec.rate_asd, color=color, lw=1, label=label)
    axes[1, 0].set_ylabel("ASD [RPM/$\\sqrt{Hz}$]")
    axes[1, 1].set_ylabel("ASD [RPM/s/$\\sqrt{Hz}$]")
    for ax, title in ((axes[1, 0], "RPM noise spectrum"), (axes[1, 1], "RPM rate noise spectrum")):
        ax.set_title(title)
        ax.set_xlabel("frequency [Hz]")
        ax.legend(fontsize=7, title="band (Welch windows)", title_fontsize=7)

    if rate_check is None:
        for ax in axes[2]:
            ax.set_visible(False)
    else:
        plot_rate_check(axes[2, 0], axes[2, 1], rate_check)
    fig.suptitle("Telemetry noise")
    fig.tight_layout()
    return fig


def plot_rate_check(ax_correlation, ax_scatter, rate_check: RateCheck):
    ax = ax_correlation
    ax.plot(rate_check.lags * 1e3, rate_check.correlation, color="k", lw=1)
    ax.axvline(rate_check.lag * 1e3, color=matlab_orange, ls="--", label=f"delay {rate_check.lag * 1e3:.1f} ms")
    ax.set_xlabel("lag of rpm_rate behind d/dt rpm [ms]")
    ax.set_ylabel("normalized cross-correlation")
    ax.set_title("RPM rate vs. differentiated RPM")
    ax.legend(fontsize=8)

    ax = ax_scatter
    lim = np.percentile(np.abs(rate_check.derivative), 99.9)
    ax.plot(rate_check.derivative, rate_check.rate, ".", color="0.4", ms=1, alpha=0.3)
    ax.plot(
        [-lim, lim],
        [-lim * rate_check.gain, lim * rate_check.gain],
        color=matlab_orange,
        label=f"gain {rate_check.gain:.3f}",
    )
    ax.plot([-lim, lim], [-lim, lim], color="k", ls=":", lw=1, label="1:1")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim * 1.3, lim * 1.3)
    ax.set_xlabel(f"d/dt rpm at t - {rate_check.lag * 1e3:.0f} ms [RPM/s]")
    ax.set_ylabel("rpm_rate [RPM/s]")
    ax.set_title(f"RPM rate vs. delayed d/dt rpm, RMS diff. {rate_check.rms_difference:.0f} RPM/s")
    ax.legend(fontsize=8)


def plot_step_responses(steps: List[MeasuredStep], fits: Dict[str, FittedModel], cfg: AnalysisConfig):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
    t = np.arange(-0.05, 0.4, 1e-4)
    cmap = plt.get_cmap("viridis")
    norm = plt.Normalize(cfg.rpm_min, cfg.rpm_max)
    for ax, up, title in zip(axes, (True, False), ("up steps", "down steps")):
        sel = [s for s in steps if (s.u_after > s.u_before) == up]
        for s in sel:
            ax.plot(s.t, s.y, color=cmap(norm(s.u_after)), lw=0.7, alpha=0.7)
        for key, fit in fits.items():
            ax.plot(t, fit.step_response(t) / fit.params["K"], color=MODEL_COLORS[key], lw=2, label=fit.model.name)
        ax.axhline(1.0, color="k", lw=0.5)
        ax.axvline(0.0, color="k", lw=0.5)
        ax.set_xlim(t[0], t[-1])
        ax.set_xlabel("time since the target step [s]")
        ax.set_title(f"{len(sel)} measured {title} (>= {cfg.min_step:.0f} RPM)")
        ax.legend(fontsize=8, loc="lower right")
    axes[0].set_ylabel("normalized RPM")
    with plt.rc_context({"axes.grid": False}):
        fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=axes, label="target after the step [RPM]")
    fig.suptitle("Step response: models (unit step / K) and measured steps (normalized)")
    return fig


def plot_bode(fits: Dict[str, FittedModel], frfs: List[FrequencyResponseEstimate], fs: float, cfg: AnalysisConfig):
    fig, (ax_mag, ax_phase, ax_coh) = plt.subplots(
        3, 1, figsize=(8, 9), sharex=True, gridspec_kw={"height_ratios": [3, 3, 1.5]}
    )
    f = np.logspace(np.log10(0.05), np.log10(fs / 2), 500)
    for key, fit in fits.items():
        h = fit.frequency_response(f)
        ax_mag.semilogx(f, 20 * np.log10(np.abs(h)), color=MODEL_COLORS[key], lw=1.5, label=fit.model.name)
        ax_phase.semilogx(f, np.degrees(np.unwrap(np.angle(h))), color=MODEL_COLORS[key], lw=1.5)
    for frf, marker, color in zip(frfs, "osD^", ("k", "0.45", matlab_purple, matlab_yellow)):
        good = frf.coherence >= cfg.min_coherence
        for sel, alpha, label in ((good, 1.0, f"measured: {frf.source} ({frf.n_windows} windows)"), (~good, 0.2, None)):
            style = dict(marker=marker, color=color, ms=3, ls="none", alpha=alpha)
            ax_mag.semilogx(frf.f[sel], 20 * np.log10(np.abs(frf.h[sel])), label=label, **style)
            ax_phase.semilogx(frf.f[sel], np.degrees(np.angle(frf.h[sel])), **style)
        ax_coh.semilogx(frf.f, frf.coherence, marker=marker, color=color, ms=2, ls="none")
    ax_mag.axhline(-3.0, color="k", ls=":", lw=0.8)
    ax_mag.set_ylim(-30, 5)
    ax_mag.set_ylabel("magnitude [dB]")
    ax_mag.legend(fontsize=8, loc="lower left")
    ax_phase.set_ylim(-200, 45)
    ax_phase.set_ylabel("phase [deg]")
    ax_coh.axhline(cfg.min_coherence, color="k", ls=":", lw=0.8)
    ax_coh.set_ylim(0, 1.05)
    ax_coh.set_ylabel("coherence $\\gamma^2$")
    ax_coh.set_xlabel("frequency [Hz]")
    ax_coh.set_xlim(f[0], f[-1])
    fig.suptitle(f"Frequency response u -> y (faint: $\\gamma^2$ < {cfg.min_coherence})")
    fig.tight_layout()
    return fig


def shade_validation(ax, run: Run):
    for i0, i1 in contiguous_stretches(run.validation & ~run.bag.validation):
        ax.axvspan(run.t[i0], run.t[i1 - 1], color=matlab_yellow, alpha=0.15, lw=0)


def plot_fit_bag(rec: BagRecording, runs: List[Run], predictions: Dict[str, List[np.ndarray]], fits, all_runs):
    fig, (ax, ax_res) = plt.subplots(2, 1, figsize=(13, 6.5), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    errors = []
    for k, run in enumerate(runs):
        i = all_runs.index(run)
        first = k == 0
        ax.plot(run.t, run.rpm, color="0.55", lw=1.0, label="measured" if first else None)
        ax.step(run.t_cmd, run.u_cmd, where="post", color="k", lw=0.6, ls=":", label="target" if first else None)
        for key, fit in fits.items():
            pred = predictions[key][i]
            ax.plot(run.t, pred, color=MODEL_COLORS[key], lw=0.8, label=fit.model.name if first else None)
            ax_res.plot(run.t, run.rpm - pred, color=MODEL_COLORS[key], lw=0.6)
            errors.append(np.abs(run.rpm - pred))
        for a in (ax, ax_res):
            shade_validation(a, run)
    for t_seg, name in rec.segments:
        if runs[0].t[0] <= t_seg <= runs[-1].t[-1] and name != "transition":
            ax.axvline(t_seg, color="k", lw=0.3)
            ax.text(t_seg, 1.0, f" {name}", transform=ax.get_xaxis_transform(), rotation=90, fontsize=6, va="top")
    lim = 1.2 * np.percentile(np.concatenate(errors), 99.5)
    ax_res.set_ylim(-lim, lim)
    ax.set_ylabel("RPM")
    ax.legend(fontsize=7, loc="upper right", ncol=2)
    ax_res.set_ylabel("measured - model [RPM]")
    ax_res.set_xlabel("time since bag start [s]")
    role = "validation" if rec.validation else "training (held-out segments shaded = validation)"
    fig.suptitle(f"{rec.label} {rec.name}: {role}", fontsize=9)
    fig.tight_layout()
    return fig


def plot_fit_zoom(runs: List[Run], predictions: Dict[str, List[np.ndarray]], fits, cfg: AnalysisConfig):
    windows = []
    for prefix, start, duration in cfg.zoom_windows:
        for i, run in enumerate(runs):
            stretches = contiguous_stretches(run.in_segment(prefix))
            if stretches:
                i0, i1 = stretches[0]
                t0 = (run.t[i0] if start >= 0 else run.t[i1 - 1]) + start
                windows.append((i, prefix, t0, t0 + duration))
                break
    if not windows:
        return None
    n_cols = 2 if len(windows) > 1 else 1
    n_rows = (len(windows) + n_cols - 1) // n_cols
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(11, 3.6 * n_rows), squeeze=False)
    for ax, (i, prefix, t0, t1) in zip(axes.flat, windows):
        run = runs[i]
        m = (run.t >= t0) & (run.t <= t1)
        mc = (run.t_cmd >= t0 - 0.1) & (run.t_cmd <= t1)
        ax.plot(run.t[m], run.rpm[m], ".-", color="0.55", ms=3, lw=0.6, label="measured")
        ax.step(run.t_cmd[mc], run.u_cmd[mc], where="post", color="k", lw=0.8, ls=":", label="target")
        for key, fit in fits.items():
            ax.plot(run.t[m], predictions[key][i][m], color=MODEL_COLORS[key], lw=1.2, label=fit.model.name)
        ax.set_xlim(t0, t1)
        ax.set_title(f"{run.bag.label} {prefix}{' (validation)' if run.validation[m].all() else ''}", fontsize=9)
        ax.set_xlabel("time since bag start [s]")
        ax.set_ylabel("RPM")
    for ax in axes.flat[len(windows) :]:
        ax.set_visible(False)
    axes.flat[0].legend(fontsize=7)
    fig.suptitle("Simulated vs. measured RPM (zoom)")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def parse_args() -> Tuple[AnalysisConfig, argparse.Namespace]:
    parser = argparse.ArgumentParser(
        description="Telemetry noise and first- / second-order identification of the closed rotor speed loop.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    defaults = AnalysisConfig()
    parser.add_argument("bags", nargs="+", help="training bags (their heldout/* segments are validation data)")
    parser.add_argument("--validation", nargs="*", default=[], help="bags used for validation only")
    parser.add_argument("--motor-index", type=int, default=None, help="spinal 0-based motor index (default: auto)")
    parser.add_argument(
        "--motor-info", default=default_motor_info_file(), help="MotorInfoDShot.yaml with the RPM window"
    )
    parser.add_argument("--time-source", choices=("stamp", "bag"), default=defaults.time_source, help="telemetry time")
    parser.add_argument("--startup-skip", type=float, default=defaults.startup_skip, help="[s] cut after switch-on")
    parser.add_argument("--hold-settle", type=float, default=defaults.hold_settle, help="[s] cut after target changes")
    parser.add_argument("--min-hold", type=float, default=defaults.min_hold, help="[s] shortest measured hold")
    parser.add_argument("--models", nargs="+", choices=list(MODELS), default=defaults.models, help="models to fit")
    parser.add_argument("--sim-dt", type=float, default=defaults.sim_dt, help="[s] simulation grid")
    parser.add_argument("--esc-gains", type=float, nargs=3, metavar=("KP", "KI", "KD"), help="AM32 speed PID gains")
    parser.add_argument("--min-step", type=float, default=defaults.min_step, help="[RPM] smallest compared step")
    parser.add_argument(
        "--zoom", nargs="*", metavar="SEGMENT:START:DURATION", help="zoom windows, START < 0 counts from the end"
    )
    parser.add_argument("--save-dir", help="save the report and the figures (PNG) to this directory")
    parser.add_argument("--no-show", action="store_true", help="do not show the figures")
    args = parser.parse_args()

    cfg = AnalysisConfig(
        motor_index=args.motor_index,
        time_source=args.time_source,
        startup_skip=args.startup_skip,
        hold_settle=args.hold_settle,
        min_hold=args.min_hold,
        models=args.models,
        sim_dt=args.sim_dt,
        esc_gains=tuple(args.esc_gains) if args.esc_gains else parse_esc_gains(args.bags + args.validation),
        min_step=args.min_step,
    )
    cfg.rpm_min, cfg.rpm_max = load_rpm_window(args.motor_info)
    if args.zoom is not None:
        cfg.zoom_windows = [(s, float(a), float(d)) for s, a, d in (z.rsplit(":", 2) for z in args.zoom)]
    return cfg, args


def main():
    cfg, args = parse_args()
    bag_paths = args.bags + args.validation
    motor_index = cfg.motor_index if cfg.motor_index is not None else detect_motor_index(bag_paths)

    # Data
    recordings = [
        read_bag(path, f"[{i}]", path in args.validation, motor_index, cfg) for i, path in enumerate(bag_paths, 1)
    ]
    runs = [run for rec in recordings for run in split_runs(rec, cfg)]
    if not runs:
        raise ValueError("No closed-loop runs in the bags")
    fs = telemetry_rate(runs)

    # Noise
    holds = [measure_hold_noise(run, *hold) for run in runs for hold in find_holds(run, cfg.hold_settle, cfg.min_hold)]
    spectra = noise_spectra(holds, fs, cfg.noise_window, cfg.noise_band)
    rate_check = check_rpm_rate(runs, fs, cfg.rate_check_window, cfg.rate_check_max_lag)

    # Identification
    grids = [make_grid(run, cfg.sim_dt) for run in runs]
    training = [~run.validation for run in runs]
    fits = {key: fit_model(MODELS[key], runs, grids, training) for key in cfg.models}
    predictions = {key: [fit.simulate(grid, run.t) for run, grid in zip(runs, grids)] for key, fit in fits.items()}
    op_fits = fit_operating_points(fits, runs, grids, cfg.split_prefixes)

    # Validation
    quality = quality_table(runs, predictions)
    frfs = [estimate_frequency_response(runs, prefix, window, fs) for prefix, window in cfg.frf_windows.items()]
    frfs = [frf for frf in frfs if frf is not None]
    steps = measured_steps(runs, cfg.min_step)

    report = "\n".join(
        [
            section("Data", report_data(recordings, runs, cfg, motor_index)),
            section("Noise in holds", report_noise(holds, rate_check, cfg)),
            section("Models", report_models(fits, steps, cfg.esc_gains)),
            section("Fit quality", report_quality(quality, fits)),
            section("Operating points (refits per split)", report_operating_points(op_fits, fits)),
        ]
    )
    print(report)

    plt.style.use(PLOT_STYLE)
    figures = {
        "noise": plot_noise(holds, spectra, rate_check, cfg),
        "step_response": plot_step_responses(steps, fits, cfg),
        "bode": plot_bode(fits, frfs, fs, cfg),
        "fit_zoom": plot_fit_zoom(runs, predictions, fits, cfg),
    }
    for i, rec in enumerate(recordings, 1):
        bag_runs = [run for run in runs if run.bag is rec]
        if bag_runs:
            figures[f"fit_bag{i}"] = plot_fit_bag(rec, bag_runs, predictions, fits, runs)

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        with open(os.path.join(args.save_dir, "report.txt"), "w") as f:
            f.write(report)
        for name, fig in figures.items():
            if fig is not None:
                fig.savefig(os.path.join(args.save_dir, f"{name}.png"), dpi=200)
        print(f"Saved the report and {len(figures)} figures to {args.save_dir}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
