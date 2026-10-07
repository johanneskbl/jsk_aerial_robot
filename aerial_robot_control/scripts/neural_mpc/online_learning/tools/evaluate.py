"""
Measure named configurations against nominal MPC, on flights none of them chose.

tune_online.py SEARCHES the hyperparameter space. This measures a handful of
configurations you already have — the one in configurations.py, a variant you
want to try, the baselines — and reports whether the difference is real.

WHAT IT REPORTS, AND WHY EACH COLUMN EXISTS

  rmse            position error over the tracking phase. The headline number,
                  and the one that hides the most.
  worst (TRACK)   the largest departure from the reference, EXCLUDING the
                  repositioning between trajectory segments. That mask is not
                  cosmetic: with the transits included, nominal MPC — which does
                  no adaptation at all — posts the best maximum of every
                  controller, because the reference deliberately leads the
                  aircraft during a transit. Both columns are printed so the
                  artefact stays visible.
  p99 (TRACK)     the same, at the 99th percentile. A single worst sample is one
                  integrator step; the percentile is what a distribution says.
  rough           rms of the per-step thrust-command increment, as a multiple of
                  nominal on the SAME flights. Command stability.
  ms/step         solver and gradient step, separately, measured solo.

THE REFERENCE IS NOMINAL MPC, not the frozen MLP. In this simulator
`use_nominal_simulator=True` means the truth IS the nominal rigid-body model
plus the injected disturbances, so a network trained on real flights has no
unmodelled dynamics to correct and costs a near-constant penalty. Pairing
against it flatters everything.

CLAIMS USE A PAIRED SIGN TEST over the held-out flights, never mean +/- std.
Every configuration meets the identical trajectory and the identical
disturbances on a given seed (harness._source_rng), so pairing cancels flight
difficulty — which is most of the spread. With n = 24, 24/24 -> p = 6e-8,
and 20/24 -> p = 0.0008.

    python3 -m online_learning.tools.evaluate              # run (or resume)
    python3 -m online_learning.tools.evaluate --report     # tables only
    python3 -m online_learning.tools.evaluate --timing     # + cost per step
"""
import argparse
import os

import numpy as np

from config.configurations import DirectoryConfig
from online_learning.tools import harness as H

# ======================================================================
# WHAT TO MEASURE  — edit this list, nothing else
# ----------------------------------------------------------------------
# (label, run flags, dataset_options overrides, model_options overrides)
#
# {} means "whatever configurations.py currently says". To try a variant, give
# only the keys that differ; everything else stays at the configured value, so
# any difference in the table is attributable to those keys alone.
# ======================================================================
CONFIGS = [
    ("nominal", dict(useMLP=False, onlineMLP=False), {}, {}),
    ("frozen",  dict(useMLP=True,  onlineMLP=False), {}, {}),
    ("online",  dict(useMLP=True,  onlineMLP=True),  {}, {}),
    # Example of a variant: the anchor switched off. Uncomment to measure it.
    # ("online_no_anchor", dict(useMLP=True, onlineMLP=True),
    #  dict(lambda_anchor=0.0), {}),
]

SEEDS = H.HOLDOUT_SEEDS          # 24 flights, none used by any search
SCENARIO = "all"                 # "none", "wind", "drag", "ground" also exist
SIM_TIME = 120
WORKERS = 8
RUN_ID = "evaluate"

# Timing is measured on fewer flights and SEQUENTIALLY: per-step cost measured
# while eight simulations share the CPU reports its own contention. Nothing else
# in the results changes with contention — the simulation has no wall-clock
# pacing — so only this stage pays for being serial.
TIMING_SEEDS = H.HOLDOUT_SEEDS[:2]


def specs(timing=False):
    seeds = TIMING_SEEDS if timing else SEEDS
    return [dict(label=label, flags=flags, ds_over=ds, mo_over=mo,
                 stage="timing" if timing else "evaluate",
                 seed=s, scenario=SCENARIO, sim_time=SIM_TIME, solo=timing)
            for label, flags, ds, mo in CONFIGS for s in seeds]


def agg(done, label, seeds, key, solo=False):
    return H._mean([r.get(key) for r in H.rows_for(done, label, SCENARIO, seeds, solo)
                    if not r.get("failed")])


def per_seed(done, label, seeds, key, solo=False):
    return {r["seed"]: r.get(key)
            for r in H.rows_for(done, label, SCENARIO, seeds, solo)
            if not r.get("failed") and r.get(key) is not None}


def paired(done, label, ref, seeds, key):
    """(wins, n, p) for `label` beating `ref` flight by flight."""
    a, b = per_seed(done, label, seeds, key), per_seed(done, ref, seeds, key)
    s = [x for x in seeds if x in a and x in b]
    w = sum(1 for x in s if a[x] < b[x])
    return w, len(s), H.sign_test_p(w, len(s))


def report(done):
    labels = [c[0] for c in CONFIGS]
    ref = "nominal"
    nom_du = agg(done, ref, SEEDS, "rms_du")

    print("\n" + "=" * 104)
    print(f"TRACKING   —   {len(SEEDS)} flights of {SIM_TIME} s, scenario "
          f"'{SCENARIO}', all configurations on identical disturbances")
    print("=" * 104)
    hdr = (f"{'configuration':<24}{'rmse':>9}{'worst':>9}{'p99':>9}"
           f"{'worst':>10}{'p99':>9}{'rough':>8}{'fails':>7}")
    print(hdr)
    print(f"{'':<24}{'[cm]':>9}{'(all)':>9}{'(all)':>9}{'(TRACK)':>10}{'(TRACK)':>9}")
    print("-" * len(hdr))
    for l in labels:
        f = lambda k: agg(done, l, SEEDS, k) * 100
        rx = agg(done, l, SEEDS, "rms_du") / nom_du if nom_du else np.nan
        nf = sum(1 for r in H.rows_for(done, l, SCENARIO, SEEDS)
                 if r.get("failed") and not r.get("infra"))
        cell = lambda v: f"{v:.2f}" if np.isfinite(v) else "--"
        print(f"{l:<24}{cell(f('rmse_track')):>9}{cell(f('dev_max')):>9}"
              f"{cell(f('dev_p99')):>9}{cell(f('trk_max')):>10}"
              f"{cell(f('trk_p99')):>9}{rx:>7.2f}x{nf:>7}")
    print("=" * 104)
    print("'worst (all)' includes the repositioning between trajectory segments,\n"
          "where the reference deliberately leads the aircraft. It is NOT a\n"
          "controller property — read 'worst (TRACK)'.")

    print("\n" + "=" * 104)
    print(f"PAIRED SIGN TESTS against '{ref}', same flights")
    print("=" * 104)
    hdr = f"{'configuration':<24}{'rmse':>18}{'worst (TRACK)':>20}{'roughness':>18}"
    print(hdr)
    print("-" * len(hdr))
    for l in labels:
        if l == ref:
            continue
        cells = []
        for key in ("rmse_track", "trk_max", "rms_du"):
            w, n, p = paired(done, l, ref, SEEDS, key)
            cells.append(f"{w}/{n} p={p:.3f}" if n else "--")
        print(f"{l:<24}{cells[0]:>18}{cells[1]:>20}{cells[2]:>18}")
    print("=" * 104)
    print("p is the probability of winning that many flights by chance alone.\n"
          f"With {len(SEEDS)} paired flights: {len(SEEDS)}/{len(SEEDS)} -> "
          f"p={H.sign_test_p(len(SEEDS), len(SEEDS)):.4f}, "
          f"{len(SEEDS)-2}/{len(SEEDS)} -> p={H.sign_test_p(len(SEEDS)-2, len(SEEDS)):.3f}.\n"
          "A configuration that wins on error while losing on roughness has\n"
          "bought accuracy with instability. Both columns decide, not one.")
    print("=" * 104)


def report_timing(done):
    print("\n" + "=" * 104)
    print("COMPUTATION COST per control step   (solo, sequential, threads pinned)")
    print("=" * 104)
    hdr = (f"{'configuration':<24}{'solver':>9}{'train':>9}{'total':>9}"
           f"{'p99.9':>9}{'max':>9}{'over budget':>14}")
    print(hdr)
    print(f"{'':<24}{'[ms]':>9}{'[ms]':>9}{'[ms]':>9}{'[ms]':>9}{'[ms]':>9}")
    print("-" * len(hdr))
    for l in [c[0] for c in CONFIGS]:
        g = lambda k: agg(done, l, TIMING_SEEDS, k, solo=True)
        if not np.isfinite(g("mpc_mean")):
            print(f"{l:<24}" + "".join(f"{'--':>9}" for _ in range(5)) + f"{'--':>14}")
            continue
        rows = [r for r in H.rows_for(done, l, SCENARIO, TIMING_SEEDS, solo=True)
                if not r.get("failed")]
        over = sum(int(r.get("rt_over", 0)) for r in rows)
        n = sum(int(r.get("rt_n", 0)) for r in rows)
        print(f"{l:<24}{g('mpc_mean'):>9.2f}{g('train_mean'):>9.2f}"
              f"{g('mpc_mean') + g('train_mean'):>9.2f}{g('rt_p999'):>9.2f}"
              f"{g('rt_max'):>9.2f}{f'{over}/{n}':>14}")
    budget = agg(done, CONFIGS[0][0], TIMING_SEEDS, "rt_budget", solo=True)
    print("=" * 104)
    print(f"Budget {budget:.1f} ms (one control period). The verdict is p99.9, "
          "NOT the max:\nthe first online step allocates the optimiser state and "
          "lands 15-25x the median.\n"
          "'solver' carries the network inside the MPC; 'train' is the gradient "
          "step. Measured\nhere: the solver is ~20x the gradient step, so the "
          "network's presence costs far more\nthan its adaptation.")
    print("=" * 104)


def main(report_only=False, timing=False):
    out = os.path.join(DirectoryConfig.ONLINE_RESULTS_DIR, "tuning", RUN_ID)
    os.makedirs(out, exist_ok=True)
    ckpt = os.path.join(out, "runs.jsonl")
    done = H.load_checkpoint(ckpt)
    fail = os.path.join(out, "failures")

    sp = specs()
    print("=" * 104)
    print(f"EVALUATE  ->  {out}")
    print("=" * 104)
    for label, flags, ds, mo in CONFIGS:
        extra = ", ".join(f"{k}={v}" for k, v in {**ds, **mo}.items())
        print(f"  {label:<24} {flags}" + (f"   [{extra}]" if extra else ""))
    print(f"\n{len(sp)} runs, {WORKERS} workers  "
          f"~{len(sp) * (0.9 * SIM_TIME + 20) / WORKERS / 60:.0f} min")
    print(f"seeds {SEEDS}")
    print("Disturbance realisation per flight (identical for every configuration):")
    for s in SEEDS[:4]:
        print(f"  seed {s:<6} {H.describe_scenario(SCENARIO, s)}")
    if len(SEEDS) > 4:
        print(f"  ... and {len(SEEDS) - 4} more")
    print("=" * 104)

    if not report_only:
        H.run_batch(sp, WORKERS, ckpt, done, fail_dir=fail, threads=1)
    report(done)

    if timing:
        if not report_only:
            H.run_batch(specs(timing=True), 1, ckpt, done, fail_dir=fail, threads=1)
        report_timing(done)
    return done


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--report", action="store_true",
                    help="re-report from the checkpoint without simulating")
    ap.add_argument("--timing", action="store_true",
                    help="also measure the per-step computation cost (sequential)")
    a = ap.parse_args()
    main(report_only=a.report, timing=a.timing)
