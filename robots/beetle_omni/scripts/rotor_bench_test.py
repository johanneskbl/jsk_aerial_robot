#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Rotor bench test with propellers: publish RPM excitation profiles to the ESC(s)
through spinal's PWM test mode (topic /pwm_test, type spinal/PwmTest).

The ESC runs closed-loop RPM control and decodes the PWM linearly:
    pwm in [0.5, 1.0]  <->  rpm in [rpm_min, rpm_max]

Test plan, each group starts after a short rest at rpm_min:
  steps     N random step setpoints in each third of the RPM range (low / middle / high),
            with the two held-out bands cut out of these splits
  heldout   N random step setpoints in each held-out band, one just below and one just
            above the hover RPM. The hover RPM itself lies in the gap between them:

            |       low        |        middle         |       high        |
            |------------------|---[below]-+-[above]---|-------------------|
         rpm_min                         hover                          rpm_max

  linear    linear rise rpm_min -> rpm_max and fall back
  logistic  logistic (S-curve) rise rpm_min -> rpm_max and fall back
  sine      sine wave around osc_center_rpm
  chirp     sine with exponentially increasing frequency, excites the rotor dynamics
            over a whole frequency band (e.g. for a Bode plot)

The segment name is published on ~segment and the commanded RPM on ~target_rpm, so a
recorded bag can be split into training and held-out data afterwards.

Usage:
  roslaunch spinal bridge.launch
  rosrun beetle_omni rotor_bench_test.py _motor_indices:=1
Optional args (every field of BenchTestConfig), e.g.:
  _segments:="steps,heldout"   # subset of steps,heldout,linear,logistic,sine,chirp
  _n_steps:=10 _step_hold:=2.0 _seed:=0
  _dry_run:=true               # only print the test plan

When the plan ends or on Ctrl-C, the motors are set to idle and spinal leaves PWM test mode.
"""

import math
import random
import time
from dataclasses import dataclass, field, fields
from typing import Callable, Dict, List, Tuple

import rospy
from std_msgs.msg import Float32, String

try:
    from spinal.msg import PwmTest  # motor_index: uint8[], pwms: float32[]
except ImportError as e:
    raise ImportError("spinal/PwmTest not found, source a workspace whose spinal package provides PwmTest.msg") from e

IDLE_PWM = 0.5  # spinal IDLE_DUTY, decoded as rpm_min by the ESC
MAX_PWM = 1.0  # spinal MAX_PWM, decoded as rpm_max by the ESC

SEGMENT_GROUPS = ("steps", "heldout", "linear", "logistic", "sine", "chirp")


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
@dataclass
class BenchTestConfig:
    """Every field can be overridden as a private ROS param, e.g. _n_steps:=5."""

    # Interface
    topic: str = "/pwm_test"
    motor_indices: List[int] = field(default_factory=lambda: [1])
    rate: float = 50.0  # [Hz] publish rate
    seed: int = 0  # seed of the random step setpoints
    segments: List[str] = field(default_factory=lambda: list(SEGMENT_GROUPS))
    dry_run: bool = False  # only print the test plan, publish nothing
    start_delay: float = 3.0  # [s] warning period before the motors spin up

    # ESC RPM window, must match RPM_MIN / RPM_MAX of the ESC firmware
    rpm_min: float = 785.0
    rpm_max: float = 13300.0

    # Held-out bands: [hover - gap - width, hover - gap] and [hover + gap, hover + gap + width]
    rpm_hover: float = 8000.0
    heldout_gap: float = 200.0  # [RPM] distance from hover to the inner edge of each band
    heldout_width: float = 600.0  # [RPM]

    # Step sequences
    n_steps: int = 10  # steps per low / middle / high split
    n_steps_heldout: int = 5  # steps per held-out band
    step_hold: float = 2.0  # [s] hold time of each setpoint

    # Rest at rpm_min before each segment group
    rest_duration: float = 3.0  # [s]

    # Continuous profiles
    ramp_duration: float = 10.0  # [s] per rise / fall, linear and logistic
    logistic_steepness: float = 10.0  # larger -> sharper S-curve
    osc_center_rpm: float = 8000.0  # center of sine and chirp
    osc_amplitude_rpm: float = 2000.0
    sine_freq: float = 0.5  # [Hz]
    sine_periods: int = 5
    chirp_freq_start: float = 0.1  # [Hz]
    chirp_freq_end: float = 5.0  # [Hz]
    chirp_duration: float = 30.0  # [s]


def as_list(value) -> list:
    """Accept "1,2", 1 or [1, 2] for list parameters."""
    if isinstance(value, str):
        return [v.strip() for v in value.split(",") if v.strip()]
    if isinstance(value, (list, tuple)):
        return list(value)
    return [value]


def load_config() -> BenchTestConfig:
    """Read every BenchTestConfig field from the private ROS namespace (~<field>)."""
    cfg = BenchTestConfig()
    for f in fields(cfg):
        setattr(cfg, f.name, rospy.get_param("~" + f.name, getattr(cfg, f.name)))

    cfg.motor_indices = [int(i) for i in as_list(cfg.motor_indices)]
    cfg.segments = [str(s) for s in as_list(cfg.segments)]
    validate_config(cfg)
    return cfg


def validate_config(cfg: BenchTestConfig):
    def require(ok, msg):
        if not ok:
            raise ValueError(msg)

    unknown = set(cfg.segments) - set(SEGMENT_GROUPS)
    require(not unknown, f"Unknown segments {sorted(unknown)}, choose from {SEGMENT_GROUPS}")
    require(cfg.motor_indices, "motor_indices must not be empty")
    require(cfg.rpm_min < cfg.rpm_max, "rpm_min must be smaller than rpm_max")
    require(cfg.heldout_gap > 0, "heldout_gap must be > 0 so that the held-out bands exclude the hover RPM")
    require(
        cfg.rpm_min <= cfg.rpm_hover - cfg.heldout_gap - cfg.heldout_width
        and cfg.rpm_hover + cfg.heldout_gap + cfg.heldout_width <= cfg.rpm_max,
        "Held-out bands must lie inside [rpm_min, rpm_max]",
    )
    require(
        cfg.rpm_min <= cfg.osc_center_rpm - cfg.osc_amplitude_rpm
        and cfg.osc_center_rpm + cfg.osc_amplitude_rpm <= cfg.rpm_max,
        "Sine / chirp must stay inside [rpm_min, rpm_max]",
    )
    require(0 < cfg.chirp_freq_start < cfg.chirp_freq_end, "Need 0 < chirp_freq_start < chirp_freq_end")


# ---------------------------------------------------------------------------
# RPM splits
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class RpmBand:
    lo: float
    hi: float

    @property
    def width(self) -> float:
        return self.hi - self.lo

    def __str__(self):
        return f"[{self.lo:.0f}, {self.hi:.0f}]"


def subtract_bands(band: RpmBand, excluded: List[RpmBand]) -> List[RpmBand]:
    """Return the parts of `band` that are not covered by any of the `excluded` bands."""
    pieces = [band]
    for ex in excluded:
        remaining = []
        for p in pieces:
            if p.lo < ex.lo:
                remaining.append(RpmBand(p.lo, min(p.hi, ex.lo)))
            if p.hi > ex.hi:
                remaining.append(RpmBand(max(p.lo, ex.hi), p.hi))
        pieces = remaining
    return pieces


def sample_rpm(bands: List[RpmBand], rng: random.Random) -> float:
    """Draw an RPM uniformly from the union of disjoint bands."""
    band = rng.choices(bands, weights=[b.width for b in bands])[0]
    return rng.uniform(band.lo, band.hi)


def build_rpm_splits(cfg: BenchTestConfig) -> Tuple[Dict[str, List[RpmBand]], Dict[str, List[RpmBand]]]:
    """Split [rpm_min, rpm_max] into equal low / middle / high thirds with the held-out bands cut out.

    Returns (train_splits, heldout_splits), both mapping a split name to its list of bands.
    """
    heldout_splits = {
        "below_hover": [RpmBand(cfg.rpm_hover - cfg.heldout_gap - cfg.heldout_width, cfg.rpm_hover - cfg.heldout_gap)],
        "above_hover": [RpmBand(cfg.rpm_hover + cfg.heldout_gap, cfg.rpm_hover + cfg.heldout_gap + cfg.heldout_width)],
    }
    excluded = [band for bands in heldout_splits.values() for band in bands]

    edges = [cfg.rpm_min + i * (cfg.rpm_max - cfg.rpm_min) / 3 for i in range(4)]
    train_splits = {
        name: subtract_bands(RpmBand(lo, hi), excluded)
        for name, lo, hi in zip(("low", "middle", "high"), edges[:-1], edges[1:])
    }
    return train_splits, heldout_splits


# ---------------------------------------------------------------------------
# Profile segments
# ---------------------------------------------------------------------------
@dataclass
class Segment:
    """One piece of the test plan: commanded RPM as a function of the segment-local time."""

    name: str
    duration: float  # [s]
    rpm_at: Callable[[float], float]  # t in [0, duration) -> RPM
    info: str = ""


def hold(name: str, rpm: float, duration: float) -> Segment:
    return Segment(name, duration, lambda t: rpm, f"hold {rpm:.0f} RPM")


def step_sequence(name: str, bands: List[RpmBand], n_steps: int, step_hold: float, rng: random.Random) -> Segment:
    """Piecewise-constant sequence of `n_steps` random setpoints drawn from `bands`."""
    setpoints = [sample_rpm(bands, rng) for _ in range(n_steps)]

    def rpm_at(t):
        return setpoints[min(int(t / step_hold), n_steps - 1)]

    info = f"{n_steps} steps in {' + '.join(map(str, bands))}: " + ", ".join(f"{s:.0f}" for s in setpoints)
    return Segment(name, n_steps * step_hold, rpm_at, info)


def linear_ramp(name: str, rpm_start: float, rpm_end: float, duration: float) -> Segment:
    def rpm_at(t):
        return rpm_start + (rpm_end - rpm_start) * t / duration

    return Segment(name, duration, rpm_at, f"linear {rpm_start:.0f} -> {rpm_end:.0f} RPM")


def logistic_ramp(name: str, rpm_start: float, rpm_end: float, duration: float, steepness: float) -> Segment:
    """S-curve from rpm_start to rpm_end, normalized so that both end points are hit exactly."""

    def sigmoid(t):
        return 1.0 / (1.0 + math.exp(-steepness * (t / duration - 0.5)))

    s_start, s_end = sigmoid(0.0), sigmoid(duration)

    def rpm_at(t):
        return rpm_start + (rpm_end - rpm_start) * (sigmoid(t) - s_start) / (s_end - s_start)

    return Segment(name, duration, rpm_at, f"logistic {rpm_start:.0f} -> {rpm_end:.0f} RPM, steepness {steepness}")


def sine_wave(name: str, rpm_center: float, rpm_amplitude: float, freq: float, n_periods: int) -> Segment:
    def rpm_at(t):
        return rpm_center + rpm_amplitude * math.sin(2 * math.pi * freq * t)

    return Segment(name, n_periods / freq, rpm_at, f"{rpm_center:.0f} +- {rpm_amplitude:.0f} RPM at {freq} Hz")


def exponential_chirp(
    name: str, rpm_center: float, rpm_amplitude: float, freq_start: float, freq_end: float, duration: float
) -> Segment:
    """Sine whose frequency rises exponentially from freq_start to freq_end.

    It spends the same time in every octave, i.e. the excitation is spread evenly
    over a logarithmic frequency axis.
    """
    k = math.log(freq_end / freq_start) / duration  # [1/s] f(t) = freq_start * exp(k * t)

    def rpm_at(t):
        phase = 2 * math.pi * freq_start * (math.exp(k * t) - 1.0) / k
        return rpm_center + rpm_amplitude * math.sin(phase)

    info = f"{rpm_center:.0f} +- {rpm_amplitude:.0f} RPM, {freq_start} -> {freq_end} Hz"
    return Segment(name, duration, rpm_at, info)


# ---------------------------------------------------------------------------
# Test plan
# ---------------------------------------------------------------------------
def build_test_plan(cfg: BenchTestConfig) -> List[Segment]:
    train_splits, heldout_splits = build_rpm_splits(cfg)

    def steps(group, splits, n_steps):
        segments = []
        for name, bands in splits.items():
            # One RNG per split, so its setpoints only depend on the seed and not on the selected segments
            rng = random.Random(f"{cfg.seed}/{group}/{name}")
            segments.append(step_sequence(f"{group}/{name}", bands, n_steps, cfg.step_hold, rng))
        return segments

    lo, hi = cfg.rpm_min, cfg.rpm_max
    osc_center, osc_amplitude = cfg.osc_center_rpm, cfg.osc_amplitude_rpm
    groups = {
        "steps": steps("steps", train_splits, cfg.n_steps),
        "heldout": steps("heldout", heldout_splits, cfg.n_steps_heldout),
        "linear": [
            linear_ramp("linear/rise", lo, hi, cfg.ramp_duration),
            linear_ramp("linear/fall", hi, lo, cfg.ramp_duration),
        ],
        "logistic": [
            logistic_ramp("logistic/rise", lo, hi, cfg.ramp_duration, cfg.logistic_steepness),
            logistic_ramp("logistic/fall", hi, lo, cfg.ramp_duration, cfg.logistic_steepness),
        ],
        "sine": [sine_wave("sine", osc_center, osc_amplitude, cfg.sine_freq, cfg.sine_periods)],
        "chirp": [
            exponential_chirp(
                "chirp", osc_center, osc_amplitude, cfg.chirp_freq_start, cfg.chirp_freq_end, cfg.chirp_duration
            )
        ],
    }

    plan = []
    for group in cfg.segments:
        if cfg.rest_duration > 0:
            plan.append(hold("rest", cfg.rpm_min, cfg.rest_duration))
        plan.extend(groups[group])
    return plan


def log_plan(cfg: BenchTestConfig, plan: List[Segment]):
    total = sum(s.duration for s in plan)
    rospy.loginfo(
        "Rotor bench test: motors %s, seed %d, %d segments, %.1f s in total",
        cfg.motor_indices,
        cfg.seed,
        len(plan),
        total,
    )
    for i, segment in enumerate(plan, 1):
        rospy.loginfo("  %2d. %-20s %6.1f s  %s", i, segment.name, segment.duration, segment.info)


# ---------------------------------------------------------------------------
# ROS interface
# ---------------------------------------------------------------------------
class RotorCommandPublisher:
    """Converts RPM setpoints into spinal PWM test commands and publishes the segment labels."""

    def __init__(self, cfg: BenchTestConfig):
        self.cfg = cfg
        self.pwm_pub = rospy.Publisher(cfg.topic, PwmTest, queue_size=10)
        self.rpm_pub = rospy.Publisher("~target_rpm", Float32, queue_size=10)
        self.segment_pub = rospy.Publisher("~segment", String, queue_size=10, latch=True)

    def rpm_to_pwm(self, rpm: float) -> float:
        frac = (rpm - self.cfg.rpm_min) / (self.cfg.rpm_max - self.cfg.rpm_min)
        return IDLE_PWM + (MAX_PWM - IDLE_PWM) * frac

    def wait_for_subscriber(self, timeout: float = 5.0):
        deadline = time.monotonic() + timeout
        while self.pwm_pub.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(0.1)
        if self.pwm_pub.get_num_connections() == 0:
            rospy.logwarn("No subscriber on %s, is 'roslaunch spinal bridge.launch' running?", self.cfg.topic)

    def send_rpm(self, rpm: float):
        rpm = min(max(rpm, self.cfg.rpm_min), self.cfg.rpm_max)
        pwms = [self.rpm_to_pwm(rpm)] * len(self.cfg.motor_indices)
        self.pwm_pub.publish(PwmTest(motor_index=self.cfg.motor_indices, pwms=pwms))
        self.rpm_pub.publish(Float32(rpm))

    def set_segment(self, name: str):
        self.segment_pub.publish(String(name))

    def stop(self):
        """Command idle, then send an empty command, which makes spinal leave PWM test mode."""
        self.set_segment("stop")
        self.send_rpm(self.cfg.rpm_min)
        time.sleep(0.2)
        self.pwm_pub.publish(PwmTest())
        time.sleep(0.2)  # let the publisher thread flush before the node shuts down


def run_plan(plan: List[Segment], publisher: RotorCommandPublisher, rate_hz: float):
    rate = rospy.Rate(rate_hz)
    for i, segment in enumerate(plan, 1):
        rospy.loginfo("[%d/%d] %s (%.1f s): %s", i, len(plan), segment.name, segment.duration, segment.info)
        publisher.set_segment(segment.name)

        t_start = time.monotonic()
        while not rospy.is_shutdown():
            t = time.monotonic() - t_start
            if t >= segment.duration:
                break
            publisher.send_rpm(segment.rpm_at(t))
            rate.sleep()


def main():
    # Handle Ctrl-C ourselves so the motors can still be stopped while the node is alive
    rospy.init_node("rotor_bench_test", disable_signals=True)
    cfg = load_config()
    plan = build_test_plan(cfg)
    log_plan(cfg, plan)
    if cfg.dry_run:
        return

    publisher = RotorCommandPublisher(cfg)
    try:
        publisher.wait_for_subscriber()
        rospy.logwarn(
            "Motors %s spin up in %.0f s, keep clear of the propellers! (Ctrl-C to abort)",
            cfg.motor_indices,
            cfg.start_delay,
        )
        time.sleep(cfg.start_delay)
        run_plan(plan, publisher, cfg.rate)
        rospy.loginfo("Test plan finished")
    except KeyboardInterrupt:
        rospy.logwarn("Aborted by user")
    finally:
        publisher.stop()
        rospy.loginfo("Motors stopped, left PWM test mode")
        rospy.signal_shutdown("rotor bench test done")


if __name__ == "__main__":
    main()
