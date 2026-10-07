"""
Three figures, no options to choose from.

    python3 -m online_learning.tools.figures

    thrust.png     rotor command over time    -> stability
    pose.png       x, y, z over time          -> accuracy
    bars.png       all controllers, two histograms: accuracy and stability
    parameters.png the tuned parameters, one column per retained option
    flight.gif     3D replay at x2            -> what it looks like

The first two and the GIF come from ONE flight, which is what you can look at.
bars.png and parameters.png come from the 8 held-out flights of the search —
the aggregate that single flight is a sample of. Show them together: one
illustrates, the other is the evidence.

Everything is flown on ONE held-out flight — one the hyperparameter search
never used to select anything — so the pictures and the reported numbers come
from the same place. Runs are reproducible: run_simulation seeds both numpy and
torch, so re-running this gives byte-identical curves.

The three configurations shown are the winner, the frozen offline model and
nominal MPC. That is the smallest set that says anything: the winner alone is a
curve without a scale.

--cached reuses the last flights (seconds instead of minutes) when you only
want to change a window, a colour or the animation speed.
"""

import argparse
import copy
import json
import os

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from config.configurations import EnvConfig, DirectoryConfig
from trajectory_tracking_and_record import run_simulation

OUT_DIR = os.path.join(DirectoryConfig.ONLINE_RESULTS_DIR, "figures")
CACHE = os.path.join(OUT_DIR, "flights.npz")

# The winner of the search. Left empty -> whatever is in configurations.py,
# which is where the winner belongs once you have adopted it.
WINNER = {}

RUNS = [
    ("Nominal MPC",          dict(useMLP=False, onlineMLP=False), None,   "tab:red"),
    ("Offline MLP (frozen)", dict(useMLP=True,  onlineMLP=False), None,   "tab:orange"),
    ("Online (adaptive)",    dict(useMLP=True,  onlineMLP=True),  WINNER, "tab:blue"),
]

SEED = 31337        # held out from the search
SIM_TIME = 120

# The payload appears at t=30 s and the wind ramp ends at t=40 s: that is where
# the controllers differ, and it is the only stretch worth showing at full size.
WINDOW = (25.0, 50.0)

# Position curves stop before the runs drift apart in trajectory PHASE — each
# reaches its "go to init pose" waypoint at a slightly different instant, and
# past that the curves look disordered for a reason that has nothing to do with
# tracking quality.
POSE_WINDOW = (0.0, 50.0)

# Frame count is  span / speed * fps , and a GIF frame costs about 65 kB here.
# Speed therefore buys window: x2 shows twice the flight for the same file.
#   x1, 15 s window -> 300 frames, ~20 MB
#   x2, 30 s window -> 300 frames, ~20 MB   <- this one
#   x4, 60 s window -> 300 frames, ~20 MB
# x2 is slow enough to follow the aircraft by eye while covering the payload
# step at t=30 s and the recovery that follows it. Raise to x3 or x4 if you
# want more of the flight in the same file.
ANIM_SPEED = 2.0
ANIM_FPS = 20
ANIM_WINDOW = (25.0, 55.0)
ANIM_AGAINST = "Offline MLP (frozen)"
ANIM_SUBJECT = "Online (adaptive)"

DPI = 150
KEYS = ("timestamp", "state_curr", "state_ref", "control")


def _t(rec):
    return np.asarray(rec["timestamp"])


def fly():
    run = dict(EnvConfig.run_options)
    run.update(recording=False, plot_trajectory=False, real_time_plot=False,
               save_animation=False)
    sim = copy.deepcopy(EnvConfig.sim_options)
    sim["seed"], sim["max_sim_time"] = SEED, SIM_TIME

    out = []
    for i, (label, flags, over, color) in enumerate(RUNS, 1):
        ds = copy.deepcopy(EnvConfig.dataset_options)
        if over:
            ds.update(over)
        print(f"[{i}/{len(RUNS)}] flying {label} ...")
        dataset, _d, _m = run_simulation(
            copy.deepcopy(EnvConfig.model_options),
            copy.deepcopy(EnvConfig.solver_options),
            ds, copy.deepcopy(sim), run, **flags)
        out.append(dict(label=label, color=color, rec=dataset._rec))

    os.makedirs(OUT_DIR, exist_ok=True)
    blob = {}
    for i, r in enumerate(out):
        for k in KEYS:
            blob[f"{i}_{k}"] = np.asarray(r["rec"][k])
    np.savez_compressed(CACHE, **blob)
    return out, sim


def load():
    if not os.path.exists(CACHE):
        return None, None
    z = np.load(CACHE, allow_pickle=False)
    if f"{len(RUNS) - 1}_timestamp" not in z:
        return None, None
    runs = [dict(label=lbl, color=col, rec={k: z[f"{i}_{k}"] for k in KEYS})
            for i, (lbl, _f, _o, col) in enumerate(RUNS)]
    sim = copy.deepcopy(EnvConfig.sim_options)
    sim["seed"], sim["max_sim_time"] = SEED, SIM_TIME
    print(f"  reusing {len(runs)} cached flights")
    return runs, sim


def _save(fig, name):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=DPI, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  wrote {path}")


def fig_thrust(runs):
    """
    Rotor 1's command. The figure that shows stability.

    The number next to each label is the rms of the per-step command increment,
    as a multiple of what nominal MPC does on the same flight — how much the
    controller moves the motors from one cycle to the next. Tracking error says
    where the aircraft ended up; this says how hard it had to work to get there.

    Computed over the TRACKING PHASE only, t >= T_takeoff, and over all four
    rotors. Excluding the take-off matters: it contains large transients that
    every controller shares, and averaging them in compresses the differences —
    on this flight the adopted configuration reads 1.1x with take-off included
    and 1.5x without. The search reports the tracking phase, so this figure
    does too, or the two would disagree for no reason a reader could guess.
    """
    t0 = EnvConfig.sim_options["T_takeoff"]

    def roughness(rec):
        t = _t(rec)
        u = np.asarray(rec["control"])[t >= t0, :4]
        return float(np.sqrt(np.mean(np.diff(u, axis=0) ** 2)))

    ref_du = roughness(runs[0]["rec"])
    fig, ax = plt.subplots(figsize=(11, 4.8))
    for r in runs:
        u = np.asarray(r["rec"]["control"])
        ax.plot(_t(r["rec"]), u[:, 0], color=r["color"], lw=1.3, alpha=0.9,
                label=f"{r['label']}   ({roughness(r['rec']) / ref_du:.1f}x nominal)")
    ax.set_xlim(*WINDOW)
    ax.axvline(30.0, color="k", ls=":", lw=1)
    ax.annotate("payload", (30.2, ax.get_ylim()[1]), va="top", fontsize=9)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("rotor 1 thrust command [N]")
    ax.set_title("Commanded thrust — a smooth line is a stable controller",
                 fontweight="bold")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    _save(fig, "thrust.png")


def fig_pose(runs):
    """x, y, z against their references. The figure that shows accuracy."""
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for k, name in enumerate("xyz"):
        ax = axes[k]
        for r in runs:
            sc = np.asarray(r["rec"]["state_curr"])
            sr = np.asarray(r["rec"]["state_ref"])
            ax.plot(_t(r["rec"]), sr[:, k], color=r["color"], ls="--", lw=1.0,
                    alpha=0.3)
            ax.plot(_t(r["rec"]), sc[:, k], color=r["color"], lw=1.6, alpha=0.9,
                    label=r["label"])
        ax.axvline(30.0, color="k", ls=":", lw=1)
        ax.set_xlim(*POSE_WINDOW)
        ax.set_ylabel(f"{name} [m]")
        ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=9, ncol=len(runs), loc="upper center")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("Position vs time — solid = flown, dashed = reference, "
                 "dotted line = payload appears", fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    _save(fig, "pose.png")


# ======================================================================
# Cross-flight views, read from the search checkpoint
# ----------------------------------------------------------------------
# The curves above are one flight, which is what you can actually look at.
# These two are the aggregate the flight is a sample of: 8 flights that the
# search never used to select anything. Keep them together — one shows, the
# other proves.
# ======================================================================
SEARCH_RUN = "search02"
CKPT_LABEL = {"nominal": "Nominal MPC", "static": "Offline MLP (frozen)"}


def _checkpoint():
    path = os.path.join(DirectoryConfig.ONLINE_RESULTS_DIR, "tuning", SEARCH_RUN,
                        "runs.jsonl")
    if not os.path.exists(path):
        print(f"  [skip] no search checkpoint at {path}")
        return None, None, None
    rows = [json.loads(l) for l in open(path)]
    hold = sorted({r["seed"] for r in rows if r.get("stage") == "confirm"})
    if not hold:
        print("  [skip] the checkpoint has no confirmation flights yet")
        return None, None, None
    # Whatever reached the confirmation stage and is not a reference is a
    # finalist; the search decides how many, this figure does not.
    finalists = [l for l in {r["label"] for r in rows if r.get("stage") == "confirm"}
                 if l not in CKPT_LABEL and l != "current"]
    return rows, hold, sorted(finalists)


def _agg(rows, hold, label, scenario, key):
    v = [r[key] for r in rows if r["label"] == label and r["scenario"] == scenario
         and r["seed"] in hold and not r.get("failed") and r.get(key) is not None]
    return (float(np.mean(v)), float(np.std(v)), len(v)) if v else (np.nan, np.nan, 0)


def _is_adopted(params):
    """
    Is this candidate the one currently in configurations.py?

    Compared with a RELATIVE tolerance: the values written into the config are
    rounded for readability (0.010447368 -> 0.01045), so an absolute epsilon
    would never match and every column would read as an alternative.
    """
    if not params:
        return False
    d = EnvConfig.dataset_options
    return (int(params["train_every"]) == int(d["train_every"])
            and np.isclose(params["lr"], d["lr"], rtol=1e-2)
            and np.isclose(params["max_step_rel"], d["max_step_rel"], rtol=1e-2))


def _candidates():
    path = os.path.join(DirectoryConfig.ONLINE_RESULTS_DIR, "tuning", SEARCH_RUN,
                        "candidates.json")
    return json.load(open(path)) if os.path.exists(path) else None


def _display(label, finalists, cand=None):
    """
    Name a column by WHAT IT IS, not by its position in a sorted list.

    Numbering finalists 1..n invites the reader to assume 1 is best; here their
    tracking errors are statistically indistinguishable, so the only honest
    distinction is which one is actually in configurations.py.
    """
    if label in CKPT_LABEL:
        return CKPT_LABEL[label]
    if label == "current":
        return "Online (previous)"
    if cand and _is_adopted(cand[label]):
        return "Online (adopted)"
    if len(finalists) == 1:
        return "Online (tuned)"
    alts = [f for f in finalists if not (cand and _is_adopted(cand[f]))]
    return f"Online (alt. {alts.index(label) + 1})" if label in alts else "Online"


def fig_bars():
    """
    Two histograms, one figure: accuracy on the left, stability on the right.

    Deliberately not one axis with two scales. They answer different questions,
    and a controller has to answer both — the point is precisely that a
    configuration can win the left panel and lose the right one.
    """
    rows, hold, finalists = _checkpoint()
    if rows is None:
        return
    cand = _candidates()
    nom_du = _agg(rows, hold, "nominal", "all", "rms_du")[0]
    if cand:
        # Same order as the parameter table: adopted first, then smoothest.
        finalists = [f for f in finalists if f in cand]
        finalists.sort(key=lambda f: (not _is_adopted(cand[f]),
                                      _agg(rows, hold, f, "all", "rms_du")[0] / nom_du))
    order = ["nominal", "static"] + finalists
    order = [l for l in order if _agg(rows, hold, l, "all", "rmse_track")[2]]
    if not order:
        print("  [skip] nothing to compare")
        return

    err = [_agg(rows, hold, l, "all", "rmse_track")[0] * 100 for l in order]
    sd = [_agg(rows, hold, l, "all", "rmse_track")[1] * 100 for l in order]
    rough = [_agg(rows, hold, l, "all", "rms_du")[0] / nom_du for l in order]
    names = [_display(l, finalists, cand) for l in order]
    def _col(l):
        if l == "nominal":
            return "tab:red"
        if l == "static":
            return "tab:orange"
        if l == "current":
            return "tab:grey"
        # The adopted configuration is drawn solid; the alternatives that the
        # search could not separate from it are drawn faded, so the figure says
        # "these three are equivalent, this is the one in the config".
        return "tab:blue" if (cand and _is_adopted(cand.get(l, {}))) else "#a8c8e8"
    cols = [_col(l) for l in order]

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5))
    a1.bar(range(len(order)), err, yerr=sd, capsize=4, color=cols)
    a1.set_ylabel("tracking error [cm]")
    a1.set_title(f"Accuracy   (mean +/- std over {len(hold)} held-out flights)")
    a2.bar(range(len(order)), rough, color=cols)
    a2.axhline(1.0, color="k", ls=":", lw=1)
    a2.annotate("nominal MPC", (len(order) - 0.5, 1.03), ha="right", fontsize=8)
    a2.set_ylabel("command roughness  (x nominal MPC)")
    a2.set_title("Stability   (lower = smoother motors)")
    for a in (a1, a2):
        a.set_xticks(range(len(order)))
        a.set_xticklabels(names, rotation=20, ha="right")
        a.grid(True, axis="y", alpha=0.3)
    fig.suptitle("All controllers compared", fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    _save(fig, "bars.png")


def fig_parameters():
    """
    The tuned parameters, one column per finalist.

    Row order goes from what you would tune first to what you would tune last.
    The last two rows are results, not settings: they are what separates
    configurations whose tracking error is indistinguishable.
    """
    rows, hold, finalists = _checkpoint()
    if rows is None:
        return
    cand_path = os.path.join(DirectoryConfig.ONLINE_RESULTS_DIR, "tuning", SEARCH_RUN,
                             "candidates.json")
    if not os.path.exists(cand_path):
        print("  [skip] candidates.json not written yet")
        return
    cand = json.load(open(cand_path))
    finalists = [f for f in finalists if f in cand]
    if not finalists:
        print("  [skip] no finalist parameters to show")
        return
    # Adopted first, then by command roughness — the axis that actually decided,
    # since the tracking errors are indistinguishable.
    nom = _agg(rows, hold, "nominal", "all", "rms_du")[0]
    finalists.sort(key=lambda f: (not _is_adopted(cand[f]),
                                  _agg(rows, hold, f, "all", "rms_du")[0] / nom))

    T = EnvConfig.sim_options.get("T_samp", 0.01)
    spec = [
        ("learning rate", lambda p: f"{p['lr']:.2e}"),
        ("max step per cycle", lambda p: f"{p['max_step_rel']:.4f}"),
        ("drift allowed from offline model", lambda p: f"{p['trust_region_rel']:.2f}"),
        ("pull back to offline model", lambda p: f"{p['lambda_anchor']:.2f}"),
        ("update every N cycles", lambda p: f"{int(p['train_every'])}"),
        ("memory [s]", lambda p: f"{p['data_horizon_s']:.1f}"),
    ]
    body = [[name] + [fn(cand[f]) for f in finalists] for name, fn in spec]
    nom_du = _agg(rows, hold, "nominal", "all", "rms_du")[0]
    body.append(["--> tracking error [cm]"] +
                [f"{_agg(rows, hold, f, 'all', 'rmse_track')[0] * 100:.1f}"
                 for f in finalists])
    body.append(["--> roughness (x nominal)"] +
                [f"{_agg(rows, hold, f, 'all', 'rms_du')[0] / nom_du:.1f}x"
                 for f in finalists])
    body.append(["--> cost [ms/step]"] +
                [f"{_agg(rows, hold, f, 'all', 'rt_mean')[0]:.1f}"
                 for f in finalists])

    cols = [_display(f, finalists, cand) for f in finalists]
    fig, ax = plt.subplots(figsize=(5.5 + 2.6 * len(cols), 1.2 + 0.5 * len(body)))
    ax.axis("off")
    t = ax.table(cellText=body, colLabels=["parameter"] + cols,
                 cellLoc="center", loc="center")
    t.auto_set_font_size(False)
    t.set_fontsize(10)
    t.scale(1, 1.7)
    for j in range(len(cols) + 1):
        t[0, j].set_facecolor("#dddddd")
        t[0, j].set_text_props(fontweight="bold")
    for i in range(len(body) - 2, len(body) + 1):      # the three result rows
        for j in range(len(cols) + 1):
            t[i, j].set_facecolor("#eef4fb")
    ax.set_title(f"Online adaptation — parameters retained\n"
                 f"last three rows measured on {len(hold)} held-out flights",
                 fontweight="bold", pad=16)
    _save(fig, "parameters.png")


def animation(runs, sim):
    """3D replay at real speed, the adaptive run against the frozen model."""
    import animate_flight as anim
    # animate_flight lives at the neural_mpc root because it is a general flight
    # renderer, and its default output is the shared results/. Point it at this
    # package's own results instead, so everything the online work produces
    # stays inside the package.
    anim.OUT_DIR = os.path.join(DirectoryConfig.ONLINE_RESULTS_DIR, "animations")
    anim.FPS, anim.SPEED = ANIM_FPS, ANIM_SPEED
    anim.T_START, anim.T_END = ANIM_WINDOW
    by = {r["label"]: r for r in runs}
    if ANIM_SUBJECT not in by or ANIM_AGAINST not in by:
        print("  [skip] animation needs both runs")
        return None
    pair = [dict(label=by[ANIM_AGAINST]["label"], rec=by[ANIM_AGAINST]["rec"],
                 color=by[ANIM_AGAINST]["color"]),
            dict(label=by[ANIM_SUBJECT]["label"], rec=by[ANIM_SUBJECT]["rec"],
                 color=by[ANIM_SUBJECT]["color"])]
    span = ANIM_WINDOW[1] - ANIM_WINDOW[0]
    print(f"  rendering {span:.0f} s of flight at x{ANIM_SPEED:g} "
          f"({int(span / ANIM_SPEED * ANIM_FPS)} frames) — this takes a while")
    return anim.animate_runs(
        pair, sim, title=f"{ANIM_SUBJECT}  vs  {ANIM_AGAINST}   (x{ANIM_SPEED:g})",
        save="gif", stem="flight", show=False)


def main(cached=False, no_anim=False):
    runs, sim = load() if cached else (None, None)
    if runs is None:
        runs, sim = fly()
    print("\nfigures:")
    fig_thrust(runs)
    fig_pose(runs)
    fig_bars()
    fig_parameters()
    a = None
    if not no_anim:
        print("\nanimation:")
        a = animation(runs, sim)
    print(f"\nDone -> {OUT_DIR}  (the GIF lands in results/animations/)")
    return runs, a


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cached", action="store_true")
    ap.add_argument("--no-anim", action="store_true")
    a = ap.parse_args()
    main(cached=a.cached, no_anim=a.no_anim)
