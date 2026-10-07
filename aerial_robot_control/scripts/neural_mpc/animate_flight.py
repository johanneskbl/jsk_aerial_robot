"""
Offline 3D replay of a recorded flight.

Reads a recording (the `_rec` dict an OnlineDataset carries, or the .npz it
saves) and replays it as a 3D animation. Nothing here touches the simulation:
you can animate runs you already did, replay them at any speed, and iterate on
the rendering without paying for a solver rebuild.

This is deliberately separate from the live view in utils/visualization_utils.py
(initialize_plotter / draw_robot), which is a debugging aid that runs inside the
control loop, slows it down, and is switched off automatically in recording
mode. That view also draws the airframe as a flat cross, which hides the one
thing that makes this robot interesting.

What is drawn
-------------
* The TILTING rotors. Each rotor disc is oriented by the servo angle actually
  recorded in the state (a_s = state[13:17]), using the same geometry the MPC
  model uses:

      thrust axis of rotor i, body frame:
          u_i = ( sin(th_i) sin(a_i),  -cos(th_i) sin(a_i),  cos(a_i) )
      with th_i = atan2(p_i_b[1], p_i_b[0]) the arm bearing.

  That expression is not invented for the drawing — it is exactly what
  create_acados_model() builds from rot_be @ rot_er, so what you see is the
  thrust direction the controller was really commanding. On this airframe the
  arms sit at +-45/+-135 deg, so a positive servo angle tilts every rotor
  tangentially: that is where the lateral and yaw authority comes from, and it
  is invisible on a conventional quadrotor plot.
* Thrust vectors, length proportional to the commanded thrust, with the
  hover value marked so over/under-thrust is readable at a glance.
* The reference trail (dashed) against the flown trail (solid), so the tracking
  error is visible as the gap between them.
* A telemetry column whose cursor is locked to the 3D view.
* The disturbances that are ACTIVE at that instant, evaluated from the same
  profile functions the simulator used (they are functions of time, so the
  animation can show the cause and the effect simultaneously).
"""

import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.geometry_utils import v_dot_q
from config.configurations import EnvConfig, DirectoryConfig
import nmpc.nmpc_tilt_mt.tilt_qd.phys_param_beetle_jetson as phys


# ======================================================================
# WHAT TO ANIMATE
# ----------------------------------------------------------------------
# REC_PATH = None  -> run a fresh simulation with the current EnvConfig and
#                     animate it (slow: it builds the acados solver).
# REC_PATH = "..." -> animate a .npz written by OnlineDataset.save(), i.e. a
#                     run made with run_options["recording"] = True.
# ======================================================================
REC_PATH = None

# Which controller to fly when REC_PATH is None.
RUN_FLAGS = dict(useMLP=True, onlineMLP=True)
MAX_SIM_TIME = 60          # shorter than the config default, for a quick look

# --- Playback ---
FPS = 30                   # frames per second of the produced animation
SPEED = 1.0                # 1.0 = real time, 2.0 = twice as fast
T_START = None             # seconds of simulation time; None = from the start
T_END = None               # None = to the end
TRAIL_SECONDS = 4.0        # how much history the trails keep

# --- Content (all four were requested; turn any off to declutter) ---
SHOW_TILT_ROTORS = True    # rotor discs oriented by the servo angles + thrust
SHOW_REFERENCE = True      # reference trail and current setpoint
SHOW_TELEMETRY = True      # right-hand column with a synchronised cursor
SHOW_DISTURBANCES = True   # wind arrow, payload, ground-effect ring

# --- View ---
FOLLOW_ROBOT = False       # False = fixed box around the whole flight
VIEW_MARGIN = 0.6          # [m] padding around the trajectory extent
ELEV, AZIM = 22, -55       # initial camera angles

# --- Output ---
SAVE = None                # None = interactive window only.
                           # "mp4" or "gif" = also write a file.
OUT_DIR = os.path.join(DirectoryConfig.RESULTS_DIR, "animations")
DPI = 110

# --- Drawing scale ---
# The airframe is TINY next to the flight envelope: 0.275 m arms inside a
# trajectory several metres across, so drawn to scale the robot is a dot and
# the rotor tilt — the whole point of this view — is invisible. The airframe is
# therefore drawn exaggerated by DRONE_SCALE. Positions, trails and reference
# are NOT scaled: only the robot's own size is, exactly like a map symbol.
DRONE_SCALE = 2.0          # 1.0 = true to scale
ROTOR_RADIUS = 0.10        # [m] visual radius of a rotor disc, before scaling
# Thrust arrows are expressed in ARM LENGTHS PER HOVER THRUST rather than in
# metres per newton: at hover every arrow is exactly THRUST_HOVER_LEN arms long,
# so "longer than usual" reads as "pushing harder than hover" at a glance, and
# the proportions stay right whatever DRONE_SCALE is.
THRUST_HOVER_LEN = 0.7
WIND_SCALE = 0.25          # [m per N] length of the wind arrow
ARM_LW, DISC_LW = 2.6, 1.8

C_BODY = "#2b2b2b"
C_ROTOR = "#1f77b4"
C_ROTOR1 = "#d62728"       # rotor 1 highlighted, so attitude is unambiguous
C_THRUST = "#ff7f0e"
C_TRAIL = "#1f77b4"
C_REF = "#444444"
C_WIND = "#17becf"
C_GE = "#9467bd"


# ======================================================================
# Recording input
# ======================================================================
def load_recording(path):
    """Load a .npz written by OnlineDataset.save() into a _rec-like dict."""
    z = np.load(path, allow_pickle=True)
    keys = ["timestamp", "dt", "state_ref", "state_curr", "state_out",
            "state_pred", "control"]
    rec = {k: z[k] for k in keys if k in z}
    missing = [k for k in ("timestamp", "state_curr", "control") if k not in rec]
    if missing:
        raise KeyError(f"{path} is missing {missing} — is it an OnlineDataset recording?")
    return rec


def simulate_now():
    """Fly the current EnvConfig once and return its recording."""
    import copy
    from trajectory_tracking_and_record import run_simulation

    sim = copy.deepcopy(EnvConfig.sim_options)
    sim["max_sim_time"] = MAX_SIM_TIME
    run = dict(EnvConfig.run_options)
    run.update(recording=False, plot_trajectory=False, real_time_plot=False,
               save_animation=False)
    dataset, _dist, _mpc = run_simulation(
        copy.deepcopy(EnvConfig.model_options), EnvConfig.solver_options,
        copy.deepcopy(EnvConfig.dataset_options), sim, run, **RUN_FLAGS)
    return dataset._rec, sim


# ======================================================================
# Airframe geometry
# ======================================================================
ROTOR_P_B = np.array([phys.p1_b, phys.p2_b, phys.p3_b, phys.p4_b], dtype=float)
ARM_BEARING = np.arctan2(ROTOR_P_B[:, 1], ROTOR_P_B[:, 0])
ARM_LENGTH = float(np.mean(np.linalg.norm(ROTOR_P_B[:, :2], axis=1)))
HOVER_THRUST = phys.mass * phys.gravity / 4.0


def rotor_axes(servo_angles):
    """
    Thrust axis and disc basis of each rotor, in the BODY frame.

    Mirrors create_acados_model()'s rot_be @ rot_er exactly:
        R_i = Rz(theta_i) @ Rx(a_i)
        u_i     = R_i @ (0,0,1)   thrust axis
        e1_i    = R_i @ (1,0,0)   disc in-plane basis
        e2_i    = R_i @ (0,1,0)
    """
    a = np.asarray(servo_angles, dtype=float)
    th = ARM_BEARING
    sa, ca = np.sin(a), np.cos(a)
    st, ct = np.sin(th), np.cos(th)
    u = np.stack([st * sa, -ct * sa, ca], axis=1)
    e1 = np.stack([ct, st, np.zeros_like(ct)], axis=1)
    e2 = np.stack([-st * ca, ct * ca, sa], axis=1)
    return u, e1, e2


_DISC_PHI = np.linspace(0, 2 * np.pi, 33)


def airframe_world(pos, quat, servo_angles, thrusts):
    """
    Build the world-frame polylines of the airframe for one instant.

    Returns (arms, discs, thrust_lines, disc1) — each an (N, 3) array using NaN
    rows as separators so a whole group is one matplotlib artist.
    """
    u_b, e1_b, e2_b = rotor_axes(servo_angles)
    s = DRONE_SCALE
    p_w = np.array([v_dot_q(s * p, quat) for p in ROTOR_P_B]) + pos

    nan = np.full((1, 3), np.nan)

    arms = []
    discs = []
    thrust = []
    for i in range(4):
        arms.append(np.vstack([pos, p_w[i], nan]))

        e1_w = v_dot_q(e1_b[i], quat)
        e2_w = v_dot_q(e2_b[i], quat)
        circle = (p_w[i]
                  + s * ROTOR_RADIUS * (np.cos(_DISC_PHI)[:, None] * e1_w
                                        + np.sin(_DISC_PHI)[:, None] * e2_w))
        discs.append(np.vstack([circle, nan]))

        u_w = v_dot_q(u_b[i], quat)
        length = (s * ARM_LENGTH * THRUST_HOVER_LEN
                  * float(thrusts[i]) / HOVER_THRUST)
        thrust.append(np.vstack([p_w[i], p_w[i] + u_w * length, nan]))

    return (np.vstack(arms), np.vstack(discs[1:]), np.vstack(thrust),
            discs[0][:-1])


# ======================================================================
# Disturbance profiles
# ======================================================================
def _val(v, t):
    """A disturbance magnitude is either a constant or a function of time."""
    return float(v(t)) if callable(v) else float(v)


def disturbance_state(sim_options, t, z):
    """Active disturbances at time t and height z, as plain numbers."""
    d = sim_options["disturbances"]
    out = dict(payload_kg=0.0, wind=np.zeros(2), ge_frac=0.0)
    if d.get("extra_mass", False):
        out["payload_kg"] = _val(d.get("extra_mass_kg", 0.0), t)
    if d.get("wind", False):
        out["wind"] = np.array([_val(d.get("wind_x", 0.0), t),
                                _val(d.get("wind_y", 0.0), t)])
    if d.get("ground_effect", False):
        k = _val(d.get("ground_effect_k", 0.0), t)
        z0 = _val(d.get("ground_effect_z0", 1.0), t)
        out["ge_frac"] = k / (1.0 + (max(z, 0.0) / z0) ** 2)
    return out


# ======================================================================
# Animator
# ======================================================================
# Palette for multi-run comparisons. Each run gets ONE colour and every element
# belonging to it (airframe, trail, reference, telemetry curve) uses that
# colour, so the eye groups by aircraft rather than by element type.
RUN_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]


class FlightAnimator:
    """
    Replay one or several recordings in a single synchronised 3D scene.

    Runs are given as dicts: {"label": str, "rec": dict, "color": str|None}.

    Time alignment
    --------------
    Runs are sampled at the same SIMULATION TIME, not at the same trajectory
    phase. Two controllers reach the "go to init pose" waypoint at different
    instants, so at a given t they are generally at different points of their
    respective references — that is a real difference between them, not an
    artefact to hide. Simulation time is what the two runs genuinely share: the
    disturbance profiles are functions of absolute t, so at every frame both
    aircraft are experiencing the same payload, the same wind and the same
    elapsed adaptation time. Each run is therefore drawn against ITS OWN
    reference, exactly like the time-series figures in compare_mlp_methods.py.
    """

    def __init__(self, runs, sim_options, title=None):
        if isinstance(runs, dict):          # a bare _rec, single-run shorthand
            runs = [{"label": "flight", "rec": runs}]
        self.sim = sim_options
        self.title = title
        self.runs = [self._prep(r, k) for k, r in enumerate(runs)]
        self._pick_frames()
        self._build_figure()

    # -- data ------------------------------------------------------------
    def _prep(self, r, k):
        rec = r["rec"]
        d = dict(label=r.get("label", f"run {k}"),
                 color=r.get("color") or RUN_COLORS[k % len(RUN_COLORS)],
                 rec=rec)
        d["t"] = np.asarray(rec["timestamp"], dtype=float)
        d["x"] = np.asarray(rec["state_curr"], dtype=float)
        d["u"] = np.asarray(rec["control"], dtype=float)
        d["ref"] = np.asarray(rec.get("state_ref", d["x"]), dtype=float)
        d["has_servo"] = d["x"].shape[1] >= 17
        d["err"] = np.linalg.norm(d["x"][:, :3] - d["ref"][:, :3], axis=1)
        d["thrust_tot"] = d["u"][:, :4].sum(axis=1)
        d["resid"] = self._true_residual(rec)
        return d

    @staticmethod
    def _true_residual(rec):
        """Measured vertical residual acceleration, if the recording has it."""
        if not all(k in rec for k in ("state_out", "state_pred", "dt")):
            return None
        dt = np.asarray(rec["dt"], dtype=float)
        dt = np.where(dt > 0, dt, np.nan)
        return (np.asarray(rec["state_out"])[:, 5]
                - np.asarray(rec["state_pred"])[:, 5]) / dt

    def _pick_frames(self):
        # Intersection of the runs' time spans: never extrapolate past the end
        # of the shorter recording.
        t0 = max(r["t"][0] for r in self.runs)
        t1 = min(r["t"][-1] for r in self.runs)
        if T_START is not None:
            t0 = max(t0, T_START)
        if T_END is not None:
            t1 = min(t1, T_END)
        step = SPEED / FPS
        n = max(int((t1 - t0) / step), 1)
        self.sample_t = t0 + np.arange(n) * step
        for r in self.runs:
            r["idx"] = np.clip(np.searchsorted(r["t"], self.sample_t),
                               0, len(r["t"]) - 1)
            r["trail_n"] = max(int(TRAIL_SECONDS / np.median(np.diff(r["t"]))), 2)
        print(f"[animate] {n} frames @ {FPS} fps (x{SPEED} speed) "
              f"covering t = {t0:.1f} .. {t1:.1f} s "
              f"for {len(self.runs)} run(s): "
              + ", ".join(r["label"] for r in self.runs))

    # -- figure ----------------------------------------------------------
    def _build_figure(self):
        ncol = 3 if SHOW_TELEMETRY else 1
        self.fig = plt.figure(figsize=(15, 8.5) if SHOW_TELEMETRY else (9, 8.5),
                              dpi=DPI)
        gs = gridspec.GridSpec(3, ncol, figure=self.fig,
                               width_ratios=[2.1, 2.1, 1.25][:ncol],
                               wspace=0.28, hspace=0.42)
        self.ax = self.fig.add_subplot(gs[:, :2] if SHOW_TELEMETRY else gs[:, 0],
                                       projection="3d")
        self._setup_3d()
        self._make_artists()
        if SHOW_TELEMETRY:
            self._setup_telemetry(gs)

    def _setup_3d(self):
        ax = self.ax
        p = np.vstack([r["x"][r["idx"]][:, :3] for r in self.runs])
        lo, hi = p.min(axis=0) - VIEW_MARGIN, p.max(axis=0) + VIEW_MARGIN
        # The floor of the box IS the ground: ground effect is defined against
        # z = 0, so a box that dips below it would put the ground-effect ring
        # underground and misread the drone's clearance.
        lo[2] = 0.0
        span = np.maximum(hi - lo, 0.8)
        self._span = span
        ax.set_xlim(lo[0], lo[0] + span[0])
        ax.set_ylim(lo[1], lo[1] + span[1])
        ax.set_zlim(lo[2], lo[2] + span[2])
        ax.set_box_aspect(span)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
        ax.view_init(elev=ELEV, azim=AZIM)
        ax.grid(True, alpha=0.25)

    def _make_artists(self):
        ax = self.ax
        e = lambda **kw: ax.plot([], [], [], **kw)[0]
        for r in self.runs:
            c = r["color"]
            r["a_trail"] = e(color=c, lw=2.4, alpha=0.95, label=r["label"])
            r["a_ref"] = e(color=c, lw=1.2, ls="--", alpha=0.45)
            r["a_refpt"] = e(color=c, marker="o", ms=6, ls="None", mfc="none",
                             alpha=0.6)
            r["a_arms"] = e(color=c, lw=ARM_LW, solid_capstyle="round")
            r["a_discs"] = e(color=c, lw=DISC_LW, alpha=0.9)
            # Rotor 1 is drawn as a filled hub marker rather than in a second
            # colour: with several aircraft on screen, colour must mean "which
            # run", never "which rotor".
            r["a_hub1"] = e(color=c, marker="o", ms=7, ls="None")
            r["a_thrust"] = e(color=c, lw=1.8, alpha=0.55)
            r["a_shadow"] = e(color=c, lw=1.0, alpha=0.15)
            r["a_payload"] = e(color=c, marker="s", ms=8, ls="-", lw=1.1,
                               alpha=0.8)
            r["a_ge"] = e(color=C_GE, lw=1.8, alpha=0.0)
        self.a_wind = ax.plot([], [], [], color=C_WIND, lw=2.4, alpha=0.9)[0]
        self.txt = ax.text2D(0.02, 0.97, "", transform=ax.transAxes,
                             fontsize=9, family="monospace", va="top")
        ax.legend(loc="upper right", fontsize=8, framealpha=0.85)

    def _setup_telemetry(self, gs):
        specs = [("position error  ||e||  [m]", "err"),
                 (r"total thrust  $\Sigma f_t$  [N]", "thrust_tot")]
        if any(r["resid"] is not None for r in self.runs):
            specs.append(("measured residual  $a_z$  [m/s$^2$]", "resid"))
        self.tel = []
        for k, (title, key) in enumerate(specs):
            a = self.fig.add_subplot(gs[k, 2])
            dots = []
            for r in self.runs:
                y = r[key]
                if y is None:
                    dots.append(None); continue
                a.plot(r["t"], y, color=r["color"], lw=0.9, alpha=0.6)
                dots.append(a.plot([], [], "o", color=r["color"], ms=5)[0])
            cursor = a.axvline(self.sample_t[0], color="k", lw=1.0, alpha=0.7)
            a.set_title(title, fontsize=9, pad=8)
            a.tick_params(labelsize=8)
            # Without this, a nearly-constant signal makes matplotlib print an
            # offset like "1e-13+1.2806e-1" right where the title sits.
            a.ticklabel_format(axis="y", useOffset=False, style="plain")
            a.grid(True, alpha=0.25)
            a.spines["top"].set_visible(False); a.spines["right"].set_visible(False)
            if k == len(specs) - 1:
                a.set_xlabel("t [s]", fontsize=9)
            d = self.sim["disturbances"]
            if d.get("extra_mass", False) and callable(d.get("extra_mass_kg")):
                on = self._step_time(d["extra_mass_kg"])
                if on is not None:
                    a.axvline(on, color="#8c564b", ls=":", lw=1.2, alpha=0.8)
            self.tel.append((a, key, cursor, dots))

    def _step_time(self, f):
        """Locate the instant a step-like profile switches on, for the marker."""
        ts = np.linspace(self.sample_t[0], self.sample_t[-1], 400)
        v = np.array([_val(f, s) for s in ts])
        jump = np.where(np.abs(np.diff(v)) > 1e-9)[0]
        return float(ts[jump[0] + 1]) if jump.size else None

    # -- per-frame update -------------------------------------------------
    def update(self, k):
        t_now = self.sample_t[k]
        for r in self.runs:
            i = r["idx"][k]
            pos = r["x"][i, :3]
            q = r["x"][i, 6:10]
            q = q / max(np.linalg.norm(q), 1e-9)
            lo = max(i - r["trail_n"], 0)

            tr = r["x"][lo:i + 1, :3]
            r["a_trail"].set_data_3d(tr[:, 0], tr[:, 1], tr[:, 2])
            r["a_shadow"].set_data_3d(tr[:, 0], tr[:, 1],
                                      np.full(len(tr), self.ax.get_zlim()[0]))

            if SHOW_REFERENCE:
                rf = r["ref"][lo:i + 1, :3]
                r["a_ref"].set_data_3d(rf[:, 0], rf[:, 1], rf[:, 2])
                r["a_refpt"].set_data_3d([r["ref"][i, 0]], [r["ref"][i, 1]],
                                         [r["ref"][i, 2]])

            if SHOW_TILT_ROTORS:
                servo = r["x"][i, 13:17] if r["has_servo"] else np.zeros(4)
                arms, discs, thrust, disc1 = airframe_world(
                    pos, q, servo, r["u"][i, :4])
                r["a_arms"].set_data_3d(arms[:, 0], arms[:, 1], arms[:, 2])
                alld = np.vstack([discs, disc1])
                r["a_discs"].set_data_3d(alld[:, 0], alld[:, 1], alld[:, 2])
                hub = disc1[:-1].mean(axis=0)
                r["a_hub1"].set_data_3d([hub[0]], [hub[1]], [hub[2]])
                r["a_thrust"].set_data_3d(thrust[:, 0], thrust[:, 1], thrust[:, 2])

            if SHOW_DISTURBANCES:
                self._draw_local_disturbances(r, t_now, pos)

        if SHOW_DISTURBANCES:
            self._draw_wind(t_now)

        if SHOW_TELEMETRY:
            for a, key, cursor, dots in self.tel:
                cursor.set_xdata([t_now, t_now])
                for r, dot in zip(self.runs, dots):
                    if dot is None or r[key] is None:
                        continue
                    i = r["idx"][k]
                    dot.set_data([r["t"][i]], [r[key][i]])

        if FOLLOW_ROBOT:
            pos = self.runs[0]["x"][self.runs[0]["idx"][k], :3]
            h = self._span / 2.0
            self.ax.set_xlim(pos[0] - h[0], pos[0] + h[0])
            self.ax.set_ylim(pos[1] - h[1], pos[1] + h[1])
            self.ax.set_zlim(max(pos[2] - h[2], 0.0), pos[2] + h[2])

        self._update_text(k, t_now)
        return []

    def _draw_wind(self, t):
        """Wind is uniform, so it is drawn once, anchored to the scene."""
        d = disturbance_state(self.sim, t, 1.0)
        w = d["wind"]
        if np.linalg.norm(w) < 1e-6:
            self.a_wind.set_data_3d([], [], [])
            return
        xl, yl, zl = self.ax.get_xlim(), self.ax.get_ylim(), self.ax.get_zlim()
        anchor = np.array([xl[0] + 0.12 * (xl[1] - xl[0]),
                           yl[0] + 0.12 * (yl[1] - yl[0]),
                           zl[1] - 0.12 * (zl[1] - zl[0])])
        tip = anchor + np.array([w[0], w[1], 0.0]) * WIND_SCALE
        v = tip - anchor
        n = np.array([-v[1], v[0], 0.0])
        nn = np.linalg.norm(n)
        n = n / nn * 0.10 if nn > 1e-9 else np.zeros(3)
        b = tip - 0.25 * v
        pts = np.vstack([anchor, tip, b + n, tip, b - n])
        self.a_wind.set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])

    def _draw_local_disturbances(self, r, t, pos):
        """Payload and ground effect follow each aircraft's own height."""
        d = disturbance_state(self.sim, t, pos[2])
        if d["payload_kg"] > 1e-6:
            drop = np.vstack([pos, pos - np.array([0, 0, 0.22])])
            r["a_payload"].set_data_3d(drop[:, 0], drop[:, 1], drop[:, 2])
            r["a_payload"].set_markersize(6 + 8 * min(d["payload_kg"], 1.0))
        else:
            r["a_payload"].set_data_3d([], [], [])

        if d["ge_frac"] > 1e-4:
            z0 = self.ax.get_zlim()[0]
            rad = 0.35 + 1.6 * d["ge_frac"]
            c = np.stack([pos[0] + rad * np.cos(_DISC_PHI),
                          pos[1] + rad * np.sin(_DISC_PHI),
                          np.full_like(_DISC_PHI, z0)], axis=1)
            r["a_ge"].set_data_3d(c[:, 0], c[:, 1], c[:, 2])
            r["a_ge"].set_alpha(min(0.15 + 5.0 * d["ge_frac"], 0.85))
        else:
            r["a_ge"].set_alpha(0.0)
            r["a_ge"].set_data_3d([], [], [])

    def _update_text(self, k, t_now):
        lines = [f"t {t_now:7.2f} s"]
        for r in self.runs:
            i = r["idx"][k]
            lines.append(
                f"{r['label'][:18]:<18} |e| {r['err'][i]:5.3f} m   "
                f"thrust {r['thrust_tot'][i] / (4 * HOVER_THRUST):4.2f}x hover")
        d = disturbance_state(self.sim, t_now, self.runs[0]["x"][self.runs[0]["idx"][k], 2])
        act = []
        if d["payload_kg"] > 1e-6:
            act.append(f"payload {d['payload_kg']:.2f} kg")
        if np.linalg.norm(d["wind"]) > 1e-6:
            act.append(f"wind ({d['wind'][0]:+.1f},{d['wind'][1]:+.1f}) N")
        if d["ge_frac"] > 1e-4:
            act.append(f"ground effect {100 * d['ge_frac']:.0f}%")
        lines.append("dist " + (", ".join(act) if act else "none"))
        self.txt.set_text("\n".join(lines))
        self.fig.suptitle(self.title or
                          f"Tiltrotor quadrotor — flight replay (x{SPEED:g} speed)",
                          fontsize=12, fontweight="bold")

    # -- run ---------------------------------------------------------------
    @staticmethod
    def _writer(fmt):
        """
        Pick a writer, degrading gracefully instead of dying on save.

        matplotlib can only produce mp4 through an external ffmpeg binary, and
        this machine does not have one — so asking for mp4 would fail after the
        whole animation had been rendered. Fall back and say how to fix it.
        """
        import matplotlib.animation as manim
        avail = manim.writers.list()
        if fmt == "mp4":
            if "ffmpeg" in avail:
                return "mp4", FFMpegWriter(fps=FPS, bitrate=4000)
            print("[animate] mp4 requested but no ffmpeg writer is available "
                  f"(matplotlib sees: {avail}).\n"
                  "          Install one with  sudo apt install ffmpeg  or  "
                  "pip install imageio-ffmpeg\n"
                  "          Falling back to gif for now.")
            fmt = "gif"
        if fmt == "gif":
            return "gif", PillowWriter(fps=FPS)
        if fmt == "html":
            # Self-contained HTML player, no external binary needed and much
            # lighter than a gif for long flights.
            return "html", manim.HTMLWriter(fps=FPS, embed_frames=True)
        raise ValueError(f"SAVE must be None, 'mp4', 'gif' or 'html' (got {fmt!r})")

    def run(self, save=None, stem="flight", show=True):
        anim = FuncAnimation(self.fig, self.update, frames=len(self.sample_t),
                             interval=1000 / FPS, blit=False, repeat=True)
        save = SAVE if save is None else save
        if save:
            ext, writer = self._writer(save)
            os.makedirs(OUT_DIR, exist_ok=True)
            n = 1
            while os.path.exists(os.path.join(OUT_DIR, f"{stem}_{n:03d}.{ext}")):
                n += 1
            path = os.path.join(OUT_DIR, f"{stem}_{n:03d}.{ext}")
            print(f"[animate] writing {path} … ({len(self.sample_t)} frames, "
                  f"this takes a while)")
            anim.save(path, writer=writer, dpi=DPI)
            print(f"[animate] saved {path}")
        if show:
            plt.show()
        return anim


def animate_runs(runs, sim_options, title=None, save=None, stem="flight",
                 show=True):
    """
    Convenience entry point used by compare_mlp_methods.py.

    `runs` is a list of {"label", "rec", "color"} dicts — exactly the structure
    the comparison script already builds for its time-series figures.
    """
    return FlightAnimator(runs, sim_options, title=title).run(
        save=save, stem=stem, show=show)


# ======================================================================
def main():
    if REC_PATH:
        rec = load_recording(REC_PATH)
        sim_options = EnvConfig.sim_options   # profiles are not stored in the npz
        print(f"[animate] loaded {REC_PATH} ({len(rec['timestamp'])} steps)")
        print("[animate] NOTE: disturbance overlays are evaluated from the "
              "CURRENT EnvConfig, not from the recording — they only match if "
              "the config is unchanged since that flight.")
    else:
        print("[animate] no REC_PATH set: flying a fresh simulation first "
              "(this builds the acados solver)")
        rec, sim_options = simulate_now()

    return FlightAnimator(rec, sim_options).run()


if __name__ == "__main__":
    main()
