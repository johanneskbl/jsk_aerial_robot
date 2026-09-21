#!/usr/bin/env python3
"""Create the segmented CDC 2026 result datasets directly from the flight rosbags.

The published path to a dataset is

    bag -> PlotJuggler CSV export, cut by hand to one trajectory
        -> create_dataset_from_csv.py -> data/<ds_name>/dataset_001.csv

This script automates the two manual steps.  It cuts every trajectory out of a
bag with the phase segmentation of ``utils/analyze_recorded_acceleration.py``
(``flight_state`` for the takeoff; the SMACH TRACK windows, classified by the
geometry of the published reference, for the rest) and writes one
PlotJuggler-shaped CSV per trajectory.  Those CSVs are then fed to the
*unmodified* ``get_synched_data_from_rosbag`` of ``create_dataset_from_csv.py``,
so the datasets are identical in schema and in semantics -- same thrust-command
time base, same quaternion sign continuity, same temporal filter -- to the ones
that were exported by hand.

Every run flies takeoff, circle, lemniscate and setpoint, in that order.

Usage
-----
    python3 create_dataset_from_bag.py                      # the three paper runs
    python3 create_dataset_from_bag.py --phases setpoint --overwrite
    python3 create_dataset_from_bag.py --bags <other>.bag --no-write-datasets
"""

import argparse
import json
import os
import re
import time

import numpy as np
import pandas as pd

from config.configurations import DirectoryConfig
from utils.data_utils import jsonify, safe_mkfile_recursive
from utils.analyze_recorded_acceleration import load_run, segment_phases


DEFAULT_BAG_DIR = "/home/jojo_ws/rosbags"
# The three runs compared in the paper; the 213 bag exists as well and can be
# added on the command line.
DEFAULT_BAGS = [
    "2026-03-29-09-22-46_mode_10_success.bag",
    "2026-03-29-09-30-56_mode_11_model_211_success.bag",
    "2026-03-29-09-49-16_mode_11_model_214_success.bag",
]
DEFAULT_CSV_DIR = os.path.join(DEFAULT_BAG_DIR, "csv")
DEFAULT_CACHE_DIR = "~/.cache/jsk_neural_mpc/accel_analysis"

ROBOT_NS = "beetle1"
PHASES = ["takeoff", "circle", "lemniscate", "setpoint"]

# Single-step prediction dataset, as in create_dataset_from_csv.py.
N_PRED = 1
APPLY_TEMPORAL_FILTER = True

# A phase starts at most this long before its reference begins to move.  The
# SMACH TRACK state is entered well before the first setpoint step (~8 s), and
# that lead-in says nothing about the controller; the circle and the lemniscate
# start moving ~0.3 s after TRACK, so this leaves them untouched.
LEAD_IN_S = 3.0

# Phases whose own first seconds are an entry transient rather than tracking:
# the circle is entered from a slightly different altitude in every run.  These
# start this long *after* their reference begins to move, instead of before it.
PHASE_SKIP_S = {"circle": 3.0}


def dataset_name(bag_path: str, phase: str) -> str:
    """``..._mode_11_model_211_success.bag`` -> ``..._MODE_11_MODEL_211_CIRCLE``."""
    stem = os.path.splitext(os.path.basename(bag_path))[0]
    m = re.search(r"mode_(\d+)(?:_model_(\d+))?", stem)
    if m is None:
        raise ValueError(f"Cannot read the control mode from '{stem}'")
    tag = f"MODE_{m.group(1)}" + (f"_MODEL_{m.group(2)}" if m.group(2) else "")
    return f"RESULTS_CDC2026_real_machine_{tag}_{phase.upper()}"


# ======================================================================================
# Bag -> the wide frame PlotJuggler exports
# ======================================================================================


def _state_columns(prefix: str, i: int) -> list:
    s = f"{prefix}/states[{i}]"
    return (
        [f"{s}/position/{a}" for a in "xyz"]
        + [f"{s}/linear_velocity/{a}" for a in "xyz"]
        + [f"{s}/orientation/{a}" for a in "wxyz"]
        + [f"{s}/angular_velocity/{a}" for a in "xyz"]
        + [f"{s}/servo_angles[{j}]" for j in range(4)]
    )


def _state_values(st) -> list:
    return [
        st.position.x, st.position.y, st.position.z,
        st.linear_velocity.x, st.linear_velocity.y, st.linear_velocity.z,
        st.orientation.w, st.orientation.x, st.orientation.y, st.orientation.z,
        st.angular_velocity.x, st.angular_velocity.y, st.angular_velocity.z,
    ] + list(st.servo_angles[:4])


def _control_columns(prefix: str, i: int) -> list:
    c = f"{prefix}/controls[{i}]"
    return [f"{c}/thrust_commands[{j}]" for j in range(4)] + [
        f"{c}/servo_angle_commands[{j}]" for j in range(4)
    ]


def _control_values(ct) -> list:
    return list(ct.thrust_commands[:4]) + list(ct.servo_angle_commands[:4])


def extract_wide_frame(bag_path: str, ns: str = ROBOT_NS, n_pred: int = N_PRED) -> pd.DataFrame:
    """One pass over the bag -> the sparse, one-column-per-field frame of a CSV export.

    Each message fills only its own topic's columns and leaves the rest NaN,
    exactly like a PlotJuggler export; ``create_dataset_from_csv`` drops the NaN
    per group and interpolates everything onto the thrust-command time base.

    ``__time`` is seconds since the start of the bag, taken from the message
    header (the per-state headers inside MPCTrajectory are not filled in).
    """
    import rosbag  # imported lazily so --help works without a ROS environment

    pred = f"/{ns}/nmpc/record_pred"
    ref = f"/{ns}/nmpc/record_ref"
    acc = f"/{ns}/sensor_plugin/imu1/acc_only"

    columns = {
        pred: sum([_state_columns(pred, i) for i in range(n_pred + 1)], [])
        + _control_columns(pred, 0),
        ref: _state_columns(ref, 0) + _control_columns(ref, 0),
        acc: [f"{acc}/acc_body_frame/{a}" for a in "xyz"]
        + [f"{acc}/acc_world_frame/{a}" for a in "xyz"],
    }
    rows = {topic: [] for topic in columns}

    bag = rosbag.Bag(bag_path)
    try:
        t0 = bag.get_start_time()
        for topic, msg, _ in bag.read_messages(topics=list(columns)):
            t = msg.header.stamp.to_sec() - t0
            if topic == acc:
                b, w = msg.acc_body_frame, msg.acc_world_frame
                rows[topic].append([t, b.x, b.y, b.z, w.x, w.y, w.z])
            else:
                n_states = n_pred + 1 if topic == pred else 1
                rows[topic].append(
                    [t]
                    + sum([_state_values(msg.states[i]) for i in range(n_states)], [])
                    + _control_values(msg.controls[0])
                )
    finally:
        bag.close()

    frames = []
    for topic, cols in columns.items():
        if not rows[topic]:
            raise ValueError(f"'{topic}' is not in {os.path.basename(bag_path)}")
        # A repeated header stamp would give dt = 0 on the thrust-command time
        # base, which create_dataset_from_csv rejects outright.
        frames.append(
            pd.DataFrame(rows[topic], columns=["__time"] + cols).drop_duplicates(
                subset="__time", keep="first"
            )
        )
    return pd.concat(frames, ignore_index=True).sort_values("__time", kind="stable")


# ======================================================================================
# Synched data -> dataset CSV (same layout as create_dataset_from_csv.py)
# ======================================================================================


def write_dataset(data: dict, ds_name: str, source_csv: str, t_samp: float, overwrite: bool) -> str:
    """Write one ``dataset_XXX.csv`` and register it in ``data/metadata.json``."""
    ds_dir = os.path.join(DirectoryConfig.DATA_DIR, ds_name)

    timestamp = data["timestamp"].squeeze()
    recording_start_idx = np.tile([0], (len(timestamp), 1))

    state = np.hstack(
        (
            data["position"],
            data["velocity"],
            data["quaternion"],
            data["angular_velocity"],
            data["servo_angle_state"],
        )
    )
    control = np.hstack((data["thrust_cmd"], data["servo_angle_cmd"]))
    state_ref = np.hstack(
        (
            data["position_ref"],
            data["velocity_ref"],
            data["quaternion_ref"],
            data["angular_velocity_ref"],
            data["servo_angle_state_ref"],
        )
    )
    control_ref = np.hstack((data["thrust_cmd_ref"], data["servo_angle_cmd_ref"]))

    dataset_dict = {
        "timestamp": timestamp,
        "recording_start_idx": recording_start_idx,
        "dt": data["dt"],
        "state": state,
        "control": control,
    }
    for i in range(1, N_PRED + 1):
        dataset_dict[f"state_pred_{i}"] = np.hstack(
            (
                data[f"position_pred_{i}"],
                data[f"velocity_pred_{i}"],
                data[f"quaternion_pred_{i}"],
                data[f"angular_velocity_pred_{i}"],
                data[f"servo_angle_state_pred_{i}"],
            )
        )
    dataset_dict.update({"state_ref": state_ref, "control_ref": control_ref})
    if "linear_acc_body" in data and "linear_acc_world" in data:
        dataset_dict["acc_body"] = data["linear_acc_body"]
        dataset_dict["acc_world"] = data["linear_acc_world"]

    # Dataset instance: keep the counter of create_dataset_from_csv.py, but allow
    # overwriting dataset_001 -- the figures read that name specifically.
    ds_instance = "dataset_001"
    if os.path.exists(ds_dir) and not overwrite:
        existing = [f for f in os.listdir(ds_dir) if f.endswith(".csv")]
        if existing:
            last = max(int(os.path.splitext(f)[0].split("_")[1]) for f in existing)
            ds_instance = "dataset_" + str(last + 1).zfill(3)
    if not safe_mkfile_recursive(ds_dir, ds_instance + ".csv", overwrite=overwrite):
        raise FileExistsError(f"{os.path.join(ds_dir, ds_instance)}.csv exists; pass --overwrite")

    ############## Metadata ##############
    outer_fields = {
        "date": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
        "real_machine": True,
        "rosbag_file": [source_csv],
        "duration": data["duration"],
        "temporal_filtering": APPLY_TEMPORAL_FILTER,
        "mpc_type": "NMPCTiltQdServo",
        "T_samp": t_samp,
    }
    inner_fields = {
        "disturbances": {
            "cog_dist": False,
            "cog_dist_model": "",
            "cog_dist_factor": 0.0,
            "motor_noise": False,
            "drag": False,
            "payload": False,
        },
    }
    json_file_name = os.path.join(DirectoryConfig.DATA_DIR, "metadata.json")
    metadata = {}
    if os.path.exists(json_file_name):
        with open(json_file_name, "r") as json_file:
            metadata = json.load(json_file)
    metadata.setdefault(ds_name, {}).update(outer_fields)
    metadata[ds_name][ds_instance] = inner_fields
    with open(json_file_name, "w") as json_file:
        json.dump(metadata, json_file, indent=4)

    ############## Save dataset ##############
    path = os.path.join(ds_dir, ds_instance + ".csv")
    pd.DataFrame({k: jsonify(v) for k, v in dataset_dict.items()}).to_csv(
        path, index=False, header=True
    )
    return path


# ======================================================================================
# Driver
# ======================================================================================


def process_bag(bag_path, phases, csv_dir, cache_dir, overwrite, write_datasets, sync_fn,
                t_samp, lead_in=LEAD_IN_S):
    """Cut every requested trajectory out of one bag and write its dataset."""
    # The segmentation reads a different set of topics than the dataset does, and
    # caches them, so this pass is cheap on a re-run.
    run = load_run(bag_path, cache_dir=cache_dir)
    found = segment_phases(run)

    print(f"\n=== {os.path.basename(bag_path)}")
    wide = extract_wide_frame(bag_path)
    stem = os.path.splitext(os.path.basename(bag_path))[0]

    for phase in phases:
        if phase not in found:
            print(f"  [skip] '{phase}' not found in this bag")
            continue

        t_s, t_e, t_align = found[phase]
        if phase in PHASE_SKIP_S:
            t_s = t_align + PHASE_SKIP_S[phase]
        else:
            t_s = max(t_s, t_align - lead_in)
        seg = wide[(wide["__time"] >= t_s) & (wide["__time"] <= t_e)]
        csv_path = os.path.join(csv_dir, f"{stem}_{phase}.csv")
        seg.to_csv(csv_path, index=False)
        print(
            f"  {phase:11s} {t_s:7.2f} -> {t_e:7.2f} s  ({t_e - t_s:5.2f} s, {len(seg)} rows, "
            f"reference moves at {t_align - t_s:5.2f} s)"
        )

        if not write_datasets:
            continue
        data = sync_fn(csv_path, APPLY_TEMPORAL_FILTER, N_PRED)
        ds_path = write_dataset(data, dataset_name(bag_path, phase), csv_path, t_samp, overwrite)

        p_err = np.linalg.norm(data["position"] - data["position_ref"], axis=1)
        print(
            f"    -> {os.path.relpath(ds_path, DirectoryConfig.DATA_DIR)}: "
            f"{len(data['timestamp'])} steps, {data['duration']:.2f} s, "
            f"dt {np.mean(data['dt']) * 1e3:.2f} +- {np.std(data['dt']) * 1e3:.2f} ms, "
            f"MAE p {np.mean(p_err):.4f} m"
        )


def build_arg_parser():
    p = argparse.ArgumentParser(
        description="Cut the flight rosbags into one dataset per trajectory.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--bags", nargs="+", default=DEFAULT_BAGS, help="bag files or paths")
    p.add_argument("--bag-dir", default=DEFAULT_BAG_DIR, help="directory holding the rosbags")
    p.add_argument("--phases", nargs="+", default=PHASES, choices=PHASES)
    p.add_argument("--csv-dir", default=DEFAULT_CSV_DIR, help="where the per-trajectory CSV cuts go")
    p.add_argument("--cache-dir", default=DEFAULT_CACHE_DIR, help="cache of the segmentation topics")
    p.add_argument("--lead-in", type=float, default=LEAD_IN_S,
                   help="seconds of phase kept before the reference starts moving [s]")
    p.add_argument("--overwrite", action="store_true", help="overwrite dataset_001 instead of counting up")
    p.add_argument("--no-write-datasets", action="store_true", help="only write the CSV cuts")
    return p


def main(argv=None):
    args = build_arg_parser().parse_args(argv)

    # Imported here because the module builds an MPC (and regenerates the acados
    # solver) at import time.  Its synchronisation is reused verbatim.
    from create_dataset_from_csv import get_synched_data_from_rosbag, T_samp

    os.makedirs(os.path.expanduser(args.csv_dir), exist_ok=True)
    for bag in args.bags:
        bag_path = bag if os.path.isabs(bag) else os.path.join(os.path.expanduser(args.bag_dir), bag)
        if not os.path.exists(bag_path):
            raise FileNotFoundError(bag_path)
        process_bag(
            bag_path,
            args.phases,
            os.path.expanduser(args.csv_dir),
            os.path.expanduser(args.cache_dir),
            args.overwrite,
            not args.no_write_datasets,
            get_synched_data_from_rosbag,
            T_samp,
            args.lead_in,
        )


if __name__ == "__main__":
    main()
