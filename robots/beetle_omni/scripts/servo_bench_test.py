#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Servo bench test: publish angle excitation profiles to all servos at once through spinal
(topic /servo/target_states, type spinal/ServoControlCmd).

The servos run their own position control. spinal forwards the raw Dynamixel goal position, which
maps linearly onto the joint angle with the gimbals parameters of Servo.yaml (as in the servo bridge):
    raw = angle_sgn * angle / angle_scale + zero_point_offset

Test plan, each group starts after a short rest at angle_neutral:
  steps        N random step setpoints in each third of the angle range (low / middle / high)
  small_steps  N setpoints in each of n_small_splits equal splits of the angle range, starting at the
               center of the split. Each setpoint changes by small_step_min to small_step_max in a random
               direction, which is reversed at the edges of the split. So the small-signal step responses
               cover the whole angle range in both directions
  linear       linear rise angle_min -> angle_max and fall back
  logistic     logistic (S-curve) rise angle_min -> angle_max and fall back
  sine         sine wave around osc_center_angle
  chirp        sine with exponentially increasing frequency, excites the servo dynamics
               over a whole frequency band (e.g. for a Bode plot)

Servo protection: the commanded angle never changes faster than max_angle_rate, except for the setpoint
changes inside the step sequences, which are true steps of at most max_step_angle:
  steps        every setpoint change jumps within one command. A step can be as large as the split
               it is drawn from, which is checked against max_step_angle when the config is validated.
               The small steps jump the same way, by at most small_step_max
  transition   before each segment, the angle ramps at max_angle_rate from the last command to the
               start angle of the segment. So the first setpoint of a step sequence is reached by a
               ramp and then held, and n_steps setpoints give n_steps - 1 step responses.
               The first transition starts from the present angle of each servo
  continuous   linear, logistic, sine and chirp are slew-limited to max_angle_rate
  stop         final ramp back to angle_neutral at max_angle_rate

The segment name ('transition' during the ramps between segments) is published on ~segment, so a
recorded bag of /servo/target_states and /servo/states can be split into the segments afterwards.

Usage:
  roslaunch spinal bridge.launch
  rosrun beetle_omni servo_bench_test.py
Optional args (every field of BenchTestConfig), e.g.:
  _servo_indices:="0,1"
  _segments:="steps,chirp"     # subset of steps,small_steps,linear,logistic,sine,chirp
  _n_steps:=10 _step_hold:=1.0 _seed:=0
  _dry_run:=true               # only print the test plan

When the plan ends or on Ctrl-C, the servos ramp back to angle_neutral at max_angle_rate and hold it.
Press Ctrl-C a second time during the ramp to stop the servos at their last command immediately.
"""

import math
import os
import random
import time
from dataclasses import dataclass, field, fields
from typing import Callable, List, Tuple

import rospkg
import rospy
import yaml
from spinal.msg import ServoControlCmd, ServoStates  # index: uint8[], angles: int16[] (raw)
from std_msgs.msg import String

SEGMENT_GROUPS = ("steps", "small_steps", "linear", "logistic", "sine", "chirp")


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
@dataclass
class BenchTestConfig:
    """Every field can be overridden as a private ROS param, e.g. _n_steps:=5."""

    # Interface
    topic: str = "/servo/target_states"
    state_topic: str = "/servo/states"  # present angles, from which the first transition starts
    servo_indices: List[int] = field(default_factory=lambda: [0, 1, 2, 3])
    rate: float = 100.0  # [Hz] publish rate, as T_samp = 0.01 s of the MPC
    seed: int = 0  # seed of the random step setpoints
    segments: List[str] = field(default_factory=lambda: list(SEGMENT_GROUPS))
    dry_run: bool = False  # only print the test plan, publish nothing
    start_delay: float = 3.0  # [s] warning period before the servos move

    # Dynamixel angle mapping, must match the gimbals of Servo.yaml
    angle_sgn: int = 1
    angle_scale: float = 0.001533981  # [rad] per raw unit
    zero_point_offset: int = 2048  # raw position at 0 rad

    # Angle window
    angle_min: float = -math.pi  # [rad]
    angle_max: float = math.pi  # [rad]
    angle_neutral: float = 0.0  # [rad] rest angle before each segment group and after the plan
    # Servo protection
    max_angle_rate: float = 5.0  # [rad/s] slew limit of every angle change except the steps of the step sequences
    max_step_angle: float = 2.1  # [rad] largest step of a step sequence, a split spans (angle_max - angle_min) / 3

    # Step sequences
    n_steps: int = 10  # setpoints per split of steps (low / middle / high) and of small_steps
    step_hold: float = 1.0  # [s] hold time of each setpoint
    n_small_splits: int = 9  # splits of small_steps, 9 centers them every 40 deg including 0 rad
    small_step_min: float = math.radians(0.5)  # [rad]
    small_step_max: float = math.radians(1.0)  # [rad]

    # Rest at angle_neutral before each segment group
    rest_duration: float = 3.0  # [s]

    # Continuous profiles
    ramp_duration: float = 5.0  # [s] per rise / fall, linear and logistic
    logistic_steepness: float = 10.0  # larger -> sharper S-curve
    osc_center_angle: float = 0.0  # [rad] center of sine and chirp
    osc_amplitude_angle: float = 0.15  # [rad]
    sine_freq: float = 1.0  # [Hz]
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


def load_angle_mapping(servo_file: str) -> Tuple[int, float, int]:
    """angle_sgn / angle_scale / zero_point_offset of the gimbals of a Servo.yaml, the mapping of the angle onto
    the raw position."""
    with open(servo_file) as f:
        gimbals = yaml.safe_load(f)["servo_controller"]["gimbals"]
    return int(gimbals["angle_sgn"]), float(gimbals["angle_scale"]), int(gimbals["zero_point_offset"])


def load_config() -> BenchTestConfig:
    """Read every BenchTestConfig field from the private ROS namespace (~<field>).

    The angle mapping defaults to the gimbals of servo_file, explicit _angle_sgn / _angle_scale / _zero_point_offset
    override it.
    """
    cfg = BenchTestConfig()
    servo_file = os.path.join(rospkg.RosPack().get_path("beetle_omni"), "config", "Servo.yaml")
    cfg.angle_sgn, cfg.angle_scale, cfg.zero_point_offset = load_angle_mapping(servo_file)
    for f in fields(cfg):
        setattr(cfg, f.name, rospy.get_param("~" + f.name, getattr(cfg, f.name)))

    cfg.servo_indices = [int(i) for i in as_list(cfg.servo_indices)]
    cfg.segments = [str(s) for s in as_list(cfg.segments)]
    validate_config(cfg)
    rospy.loginfo(
        "Angle mapping: sgn %d, scale %.9f rad, zero point %d (servo info: %s)",
        cfg.angle_sgn,
        cfg.angle_scale,
        cfg.zero_point_offset,
        servo_file,
    )
    return cfg


def validate_config(cfg: BenchTestConfig):
    def require(ok, msg):
        if not ok:
            raise ValueError(msg)

    unknown = set(cfg.segments) - set(SEGMENT_GROUPS)
    require(not unknown, f"Unknown segments {sorted(unknown)}, choose from {SEGMENT_GROUPS}")
    require(cfg.servo_indices, "servo_indices must not be empty")
    require(cfg.angle_min < cfg.angle_max, "angle_min must be smaller than angle_max")
    require(cfg.angle_min <= cfg.angle_neutral <= cfg.angle_max, "angle_neutral must lie inside [angle_min, angle_max]")
    require(cfg.rate > 0, "rate must be > 0")
    require(cfg.max_angle_rate > 0, "max_angle_rate must be > 0")
    require(
        (cfg.angle_max - cfg.angle_min) / 3 <= cfg.max_step_angle,
        "A split spans (angle_max - angle_min) / 3, so its steps can exceed max_step_angle",
    )
    require(0 < cfg.small_step_min <= cfg.small_step_max, "Need 0 < small_step_min <= small_step_max")
    require(
        cfg.n_small_splits > 0 and (cfg.angle_max - cfg.angle_min) / cfg.n_small_splits >= 2 * cfg.small_step_max,
        "Each of the n_small_splits > 0 splits must span 2 * small_step_max, so that a small step can always reverse",
    )
    require(
        cfg.angle_min <= cfg.osc_center_angle - cfg.osc_amplitude_angle
        and cfg.osc_center_angle + cfg.osc_amplitude_angle <= cfg.angle_max,
        "Sine / chirp must stay inside [angle_min, angle_max]",
    )
    require(0 < cfg.chirp_freq_start < cfg.chirp_freq_end, "Need 0 < chirp_freq_start < chirp_freq_end")


# ---------------------------------------------------------------------------
# Servo protection
# ---------------------------------------------------------------------------
def clamp_angle(cfg: BenchTestConfig, angle: float) -> float:
    return min(max(angle, cfg.angle_min), cfg.angle_max)


class AngleLimiter:
    """Limits every change of the commanded angle of one servo. It is called once per published command.

    A ramp command moves toward its target by at most max_delta = max_angle_rate / rate per call. If the loop
    runs slower than `rate`, the ramps get slower, which errs on the safe side. A step command jumps to its
    target at once if the jump is at most max_step_angle, and ramps like any other command otherwise.
    """

    def __init__(self, cfg: BenchTestConfig, angle: float):
        self.max_delta = cfg.max_angle_rate / cfg.rate
        self.max_jump = cfg.max_step_angle
        self.value = angle  # last commanded angle, starts at the present angle of the servo

    def in_reach(self, target: float) -> bool:
        """True if a single ramp command reaches `target`."""
        return abs(target - self.value) <= self.max_delta

    def __call__(self, target: float, step: bool = False) -> float:
        delta = target - self.value
        if self.in_reach(target) or (step and abs(delta) <= self.max_jump):
            self.value = target
        else:
            self.value += math.copysign(self.max_delta, delta)
        return self.value


def simulate_plan(cfg: BenchTestConfig, plan: List["Segment"]) -> Tuple[List[float], List[float], float]:
    """Run the plan through the angle limiter at the nominal publish rate, the same way run_plan does.

    The servos are assumed to start at angle_neutral. Returns per segment the duration of the transition ramp
    into it and the fraction of the segment during which the slew limit holds the command off the profile,
    and the commanded angle at the end of the plan.
    """
    limiter = AngleLimiter(cfg, cfg.angle_neutral)
    transitions, limited_fractions = [], []
    for segment in plan:
        start_angle = clamp_angle(cfg, segment.angle_at(0.0))
        n_ramp = 0
        if not limiter.in_reach(start_angle):
            while limiter.value != start_angle:
                limiter(start_angle)
                n_ramp += 1
        transitions.append(n_ramp / cfg.rate)

        n = max(1, math.ceil(segment.duration * cfg.rate))
        limited = 0
        for k in range(n):
            target = clamp_angle(cfg, segment.angle_at(k / cfg.rate))
            limited += limiter(target, step=segment.stepped) != target
        limited_fractions.append(limited / n)
    return transitions, limited_fractions, limiter.value


# ---------------------------------------------------------------------------
# Profile segments
# ---------------------------------------------------------------------------
@dataclass
class Segment:
    """One piece of the test plan: commanded angle as a function of the segment-local time."""

    name: str
    duration: float  # [s]
    angle_at: Callable[[float], float]  # t in [0, duration) -> angle [rad]
    info: str = ""
    stepped: bool = False  # piecewise constant profile whose setpoint changes are commanded as steps, not ramped


def hold(name: str, angle: float, duration: float) -> Segment:
    return Segment(name, duration, lambda t: angle, f"hold {angle:.2f} rad")


def step_sequence(name: str, lo: float, hi: float, n_steps: int, step_hold: float, seed: int) -> Segment:
    """Piecewise-constant sequence of `n_steps` random setpoints drawn uniformly from [lo, hi]."""
    # One RNG per sequence, so its setpoints only depend on the seed and not on the selected segments
    rng = random.Random(f"{seed}/{name}")
    setpoints = [rng.uniform(lo, hi) for _ in range(n_steps)]

    def angle_at(t):
        return setpoints[min(int(t / step_hold), n_steps - 1)]

    info = f"{n_steps} steps in [{lo:.2f}, {hi:.2f}] rad: " + ", ".join(f"{s:.2f}" for s in setpoints)
    return Segment(name, n_steps * step_hold, angle_at, info, stepped=True)


def small_step_sequence(
    name: str, lo: float, hi: float, step_min: float, step_max: float, n_steps: int, step_hold: float, seed: int
) -> Segment:
    """Piecewise-constant random walk of `n_steps` setpoints from the center of [lo, hi]. Each setpoint changes by
    step_min to step_max in a random direction, which is reversed where the walk would leave [lo, hi]."""
    rng = random.Random(f"{seed}/{name}")
    setpoints = [(lo + hi) / 2]
    for _ in range(n_steps - 1):
        step = rng.choice((-1, 1)) * rng.uniform(step_min, step_max)
        if not lo <= setpoints[-1] + step <= hi:
            step = -step
        setpoints.append(setpoints[-1] + step)

    def angle_at(t):
        return setpoints[min(int(t / step_hold), n_steps - 1)]

    info = (
        f"{n_steps} steps of {math.degrees(step_min):.1f} to {math.degrees(step_max):.1f} deg "
        f"in [{lo:.2f}, {hi:.2f}] rad: " + ", ".join(f"{s:.3f}" for s in setpoints)
    )
    return Segment(name, n_steps * step_hold, angle_at, info, stepped=True)


def linear_ramp(name: str, angle_start: float, angle_end: float, duration: float) -> Segment:
    def angle_at(t):
        return angle_start + (angle_end - angle_start) * t / duration

    return Segment(name, duration, angle_at, f"linear {angle_start:.2f} -> {angle_end:.2f} rad")


def logistic_ramp(name: str, angle_start: float, angle_end: float, duration: float, steepness: float) -> Segment:
    """S-curve from angle_start to angle_end, normalized so that both end points are hit exactly."""

    def sigmoid(t):
        return 1.0 / (1.0 + math.exp(-steepness * (t / duration - 0.5)))

    s_start, s_end = sigmoid(0.0), sigmoid(duration)

    def angle_at(t):
        return angle_start + (angle_end - angle_start) * (sigmoid(t) - s_start) / (s_end - s_start)

    return Segment(
        name, duration, angle_at, f"logistic {angle_start:.2f} -> {angle_end:.2f} rad, steepness {steepness}"
    )


def sine_wave(name: str, angle_center: float, angle_amplitude: float, freq: float, n_periods: int) -> Segment:
    def angle_at(t):
        return angle_center + angle_amplitude * math.sin(2 * math.pi * freq * t)

    return Segment(name, n_periods / freq, angle_at, f"{angle_center:.2f} +- {angle_amplitude:.2f} rad at {freq} Hz")


def exponential_chirp(
    name: str, angle_center: float, angle_amplitude: float, freq_start: float, freq_end: float, duration: float
) -> Segment:
    """Sine whose frequency rises exponentially from freq_start to freq_end.

    It spends the same time in every octave, i.e. the excitation is spread evenly
    over a logarithmic frequency axis.
    """
    k = math.log(freq_end / freq_start) / duration  # [1/s] f(t) = freq_start * exp(k * t)

    def angle_at(t):
        phase = 2 * math.pi * freq_start * (math.exp(k * t) - 1.0) / k
        return angle_center + angle_amplitude * math.sin(phase)

    info = f"{angle_center:.2f} +- {angle_amplitude:.2f} rad, {freq_start} -> {freq_end} Hz"
    return Segment(name, duration, angle_at, info)


# ---------------------------------------------------------------------------
# Test plan
# ---------------------------------------------------------------------------
def build_test_plan(cfg: BenchTestConfig) -> List[Segment]:
    lo, hi = cfg.angle_min, cfg.angle_max
    osc_center, osc_amplitude = cfg.osc_center_angle, cfg.osc_amplitude_angle
    edges = [lo + i * (hi - lo) / 3 for i in range(4)]
    small_edges = [lo + i * (hi - lo) / cfg.n_small_splits for i in range(cfg.n_small_splits + 1)]
    groups = {
        "steps": [
            step_sequence(f"steps/{name}", split_lo, split_hi, cfg.n_steps, cfg.step_hold, cfg.seed)
            for name, split_lo, split_hi in zip(("low", "middle", "high"), edges[:-1], edges[1:])
        ],
        "small_steps": [
            small_step_sequence(
                f"small_steps/{i}",
                split_lo,
                split_hi,
                cfg.small_step_min,
                cfg.small_step_max,
                cfg.n_steps,
                cfg.step_hold,
                cfg.seed,
            )
            for i, (split_lo, split_hi) in enumerate(zip(small_edges[:-1], small_edges[1:]))
        ],
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
            plan.append(hold("rest", cfg.angle_neutral, cfg.rest_duration))
        plan.extend(groups[group])
    return plan


def log_plan(cfg: BenchTestConfig, plan: List[Segment]):
    transitions, limited_fractions, final_angle = simulate_plan(cfg, plan)
    ramp_back = abs(final_angle - cfg.angle_neutral) / cfg.max_angle_rate
    total = sum(s.duration for s in plan) + sum(transitions) + ramp_back
    rospy.loginfo(
        "Servo bench test: servos %s, seed %d, %d segments, %.1f s in total, "
        "angle slew limit %.2f rad/s, max step %.2f rad",
        cfg.servo_indices,
        cfg.seed,
        len(plan),
        total,
        cfg.max_angle_rate,
        cfg.max_step_angle,
    )
    rospy.loginfo("  'limited' = share of the segment during which the slew limit holds the command off the profile")
    for i, (segment, transition, limited) in enumerate(zip(plan, transitions, limited_fractions), 1):
        if transition > 0:
            rospy.loginfo(
                "      %-20s %6.1f s  ramp to %.2f rad",
                "transition",
                transition,
                clamp_angle(cfg, segment.angle_at(0.0)),
            )
        rospy.loginfo(
            "  %2d. %-20s %6.1f s  limited %3.0f%%  %s",
            i,
            segment.name,
            segment.duration,
            100 * limited,
            segment.info,
        )
    rospy.loginfo(
        "  %2d. %-20s %6.1f s  ramp %.2f -> %.2f rad",
        len(plan) + 1,
        "stop",
        ramp_back,
        final_angle,
        cfg.angle_neutral,
    )


# ---------------------------------------------------------------------------
# ROS interface
# ---------------------------------------------------------------------------
class ServoCommandPublisher:
    """Converts angle setpoints into raw goal positions of all servos and publishes the segment labels.

    Every command passes through one angle limiter per servo, so no caller can make an angle jump by more than
    max_step_angle or, outside of steps, change faster than max_angle_rate.
    """

    def __init__(self, cfg: BenchTestConfig):
        self.cfg = cfg
        self.servos_commanded = False  # True once a goal position was sent, i.e. the servos may move
        self.cmd_pub = rospy.Publisher(cfg.topic, ServoControlCmd, queue_size=10)
        self.segment_pub = rospy.Publisher("~segment", String, queue_size=10, latch=True)

        # Start each limiter at the present angle of its servo, so that the first command does not jump
        states = rospy.wait_for_message(cfg.state_topic, ServoStates, timeout=5.0).servos
        # First entry per index, since spinal leaves the entries of servos that send no data at index 0
        present = {s.index: s.angle for s in reversed(states)}
        self.limiters = [AngleLimiter(cfg, self.raw_to_angle(present[i])) for i in cfg.servo_indices]

    def angle_to_raw(self, angle: float) -> int:
        return round(self.cfg.angle_sgn * angle / self.cfg.angle_scale + self.cfg.zero_point_offset)

    def raw_to_angle(self, raw: int) -> float:
        return self.cfg.angle_sgn * (raw - self.cfg.zero_point_offset) * self.cfg.angle_scale

    def in_reach(self, angle: float) -> bool:
        """True if a single ramp command brings every servo to `angle`."""
        return all(limiter.in_reach(angle) for limiter in self.limiters)

    def send_angle(self, angle: float, step: bool = False):
        """Command `angle` to all servos. A step jumps there at once (up to max_step_angle), otherwise each
        servo moves at most max_angle_rate / rate from its last command."""
        angle = clamp_angle(self.cfg, angle)
        raws = [self.angle_to_raw(limiter(angle, step=step)) for limiter in self.limiters]
        self.cmd_pub.publish(ServoControlCmd(index=self.cfg.servo_indices, angles=raws))
        self.servos_commanded = True

    def set_segment(self, name: str):
        self.segment_pub.publish(String(name))

    def ramp_to(self, angle: float):
        """Ramp all servos to `angle` at max_angle_rate.

        Plain sleeps instead of rospy.Rate, so the ramp also runs while rospy is shutting down.
        """
        angle = clamp_angle(self.cfg, angle)
        period = 1.0 / self.cfg.rate
        while any(limiter.value != angle for limiter in self.limiters):
            self.send_angle(angle)
            time.sleep(period)

    def stop(self):
        """Ramp all servos back to angle_neutral at max_angle_rate, where they hold."""
        self.set_segment("stop")
        if self.servos_commanded:
            rospy.loginfo("Ramping back to %.2f rad (Ctrl-C again to stop immediately)", self.cfg.angle_neutral)
            try:
                self.ramp_to(self.cfg.angle_neutral)
                rospy.loginfo("Servos hold %.2f rad", self.cfg.angle_neutral)
            except KeyboardInterrupt:
                rospy.logwarn("Ramp-back aborted by user, the servos hold their last command")
        time.sleep(0.2)  # let the publisher thread flush before the node shuts down


def run_plan(plan: List[Segment], publisher: ServoCommandPublisher, rate_hz: float):
    rate = rospy.Rate(rate_hz)
    for i, segment in enumerate(plan, 1):
        if rospy.is_shutdown():
            break

        # Ramp to the start of the segment first, so the segment itself runs exactly as planned
        start_angle = clamp_angle(publisher.cfg, segment.angle_at(0.0))
        if not publisher.in_reach(start_angle):
            rospy.loginfo("Transition to %.2f rad", start_angle)
            publisher.set_segment("transition")
            publisher.ramp_to(start_angle)

        rospy.loginfo("[%d/%d] %s (%.1f s): %s", i, len(plan), segment.name, segment.duration, segment.info)
        publisher.set_segment(segment.name)

        t_start = time.monotonic()
        while not rospy.is_shutdown():
            t = time.monotonic() - t_start
            if t >= segment.duration:
                break
            publisher.send_angle(segment.angle_at(t), step=segment.stepped)
            rate.sleep()


def main():
    # Handle Ctrl-C ourselves so the servos can still be ramped back while the node is alive
    rospy.init_node("servo_bench_test", disable_signals=True)
    cfg = load_config()
    plan = build_test_plan(cfg)
    log_plan(cfg, plan)
    if cfg.dry_run:
        return

    publisher = ServoCommandPublisher(cfg)
    try:
        rospy.logwarn(
            "Servos %s move in %.0f s, keep clear of the rotor arms! (Ctrl-C to abort)",
            cfg.servo_indices,
            cfg.start_delay,
        )
        time.sleep(cfg.start_delay)
        run_plan(plan, publisher, cfg.rate)
        rospy.loginfo("Test plan finished")
    except KeyboardInterrupt:
        rospy.logwarn("Aborted by user")
    finally:
        publisher.stop()
        rospy.signal_shutdown("servo bench test done")


if __name__ == "__main__":
    main()
