import os
import sys
import shutil

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

import torch

from config.configurations import DirectoryConfig, EnvConfig
from utils.data_utils import undo_jsonify
from utils.geometry_utils import v_dot_q, quaternion_inverse
from sim_environment.forward_prop import init_forward_prop
from utils.model_utils import load_model

# Allow importing sibling package `nmpc/` (used by the ICRA scripts as well)
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_jetson as phys_jetson


matlab_blue = "#0072BD"
matlab_orange = "#D95319"
matlab_yellow = "#EDB120"
matlab_green = "#77AC30"
matlab_purple = "#7E2F8E"

# Matplotlib's default ':' can look too sparse depending on DPI/export.
# Use a custom short on/off dash pattern for a denser dotted look.
DENSE_DOTTED = (0, (0.5, 2.0))
LOOSELY_DASHED = (0, (3.0, 2.5))
VERY_LOOSELY_DASHED = (0, (3.0, 4.0))
VERY_VERY_LOOSELY_DASHED = (0, (2.0, 7.0))

# Figure title placement: keep titles close to the axes without overlap.
# NOTE: `tight_layout(rect=...)` does not automatically reserve space for
# `suptitle`, so we explicitly reserve a small top margin.
FIG_TITLE_Y = 0.985
FIG_TIGHT_LAYOUT_TOP = 0.94

# Legend box transparency (explicit calls below override rcParams).
LEGEND_FRAME_ALPHA = 0.85


def setup_plot_style():
    """Match the ICRA scripts' look (scienceplots + LaTeX-like fonts).

    Uses LaTeX if available; otherwise falls back to matplotlib mathtext.
    """
    try:
        import scienceplots  # noqa: F401

        plt.style.use(["science", "grid"])
    except Exception:
        # Fallback: still decent defaults
        plt.style.use("default")

    # Prefer LaTeX rendering (Computer Modern) if a TeX install exists.
    # This matches the ICRA plotting script, but we keep a fallback for machines
    # without LaTeX to avoid runtime crashes.
    has_latex = shutil.which("latex") is not None
    use_tex = bool(has_latex)

    font_size = 17
    plt.rcParams.update(
        {
            "font.size": font_size,
            "axes.labelsize": font_size,
            "axes.titlesize": font_size,
            "figure.titlesize": font_size + 3,
            "figure.titleweight": "bold",
            "legend.fontsize": font_size -2 ,
            "legend.framealpha": 0.7,
            "legend.fancybox": True,
            "lines.linewidth": 2.0,
            "axes.linewidth": 1.0,
            "grid.alpha": 0.35,
            "grid.linewidth": 0.6,
            "xtick.direction": "in",
            "ytick.direction": "in",
            # Make custom dotted patterns look like actual dots.
            "lines.dash_capstyle": "round",
            "text.usetex": use_tex,
            # When LaTeX isn't available, these settings still give a LaTeX-like look.
            "font.family": "serif",
            "mathtext.fontset": "cm",
            **(
                {
                    # NOTE: rcParams expects valid LaTeX here; a double backslash
                    # becomes a LaTeX linebreak (\\) and breaks compilation.
                    "text.latex.preamble": r"\usepackage{amsmath,amssymb}",
                }
                if use_tex
                else {}
            ),
        }
    )


def _escape_latex_text(text: str) -> str:
    """Escape a subset of LaTeX special characters for use in text mode."""

    # Keep this minimal; titles should remain human-readable.
    replacements = {
        "\\": r"\textbackslash{}",
        "{": r"\{",
        "}": r"\}",
        "#": r"\#",
        "$": r"\$",
        "%": r"\%",
        "&": r"\&",
        "_": r"\_",
        "^": r"\textasciicircum{}",
        "~": r"\textasciitilde{}",
    }

    escaped = text
    for k, v in replacements.items():
        escaped = escaped.replace(k, v)
    return escaped


def latex_heading(text: str) -> str:
    """Return a LaTeX-heading-like title string.

    - Uses upright Roman serif (non-italic) to match typical LaTeX headings.
    - Does NOT force capitalization; preserves the provided casing.
    """

    if bool(plt.rcParams.get("text.usetex", False)):
        safe = _escape_latex_text(text)
        # Force upright roman serif in text mode.
        return rf"\textsc{{{safe}}}"
    return text


def load_data(file_subpath):
    filepath = os.path.join(DirectoryConfig.DATA_DIR, file_subpath)
    df = pd.read_csv(filepath)

    timestamp = df["timestamp"].to_numpy()
    state = undo_jsonify(df["state"].to_numpy())
    state_ref = undo_jsonify(df["state_ref"].to_numpy())

    control = undo_jsonify(df["control"].to_numpy()) if "control" in df.columns else None
    dt = df["dt"].to_numpy() if "dt" in df.columns else None

    return {
        "timestamp": timestamp,
        "state": state,
        "state_ref": state_ref,
        "control": control,
        "dt": dt,
        "filepath": filepath,
    }

def compute_errors(state, state_ref):
    # Position tracking error norm
    p_err = np.linalg.norm(state[:, 0:3] - state_ref[:, 0:3], axis=1)
    
    # Attitude tracking error norm (quaternion difference)
    q_err = np.linalg.norm(state[:, 6:10] - state_ref[:, 6:10], axis=1)
    
    return p_err, q_err


def velocity_mapping(state_sequence: np.ndarray) -> np.ndarray:
    """Map world-frame linear velocity into body frame as done in the ICRA scripts."""
    p_traj = state_sequence[:, :3]
    v_w_traj = state_sequence[:, 3:6]
    q_traj = state_sequence[:, 6:10]
    other_traj = state_sequence[:, 10:]

    v_b_traj = np.empty_like(v_w_traj)
    for t in range(len(v_w_traj)):
        v_b_traj[t, :] = v_dot_q(v_w_traj[t, :], quaternion_inverse(q_traj[t, :]))
    return np.concatenate((p_traj, v_b_traj, q_traj, other_traj), axis=1)


def build_analytical_dynamics():
    class struct:
        def __init__(self):
            pass

    mpc = struct()
    mpc.tilt = True
    mpc.include_servo_model = True
    mpc.include_thrust_model = False
    mpc.include_servo_derivative = False
    mpc.include_cog_dist_parameter = True
    mpc.phys = phys_jetson

    return init_forward_prop(mpc, return_continuous=True)


def compute_analytical_linear_acc(state: np.ndarray, control: np.ndarray) -> np.ndarray:
    """Compute analytical-model linear acceleration a = \dot{v} in world frame."""
    if control is None:
        raise ValueError("control is required to compute analytical-model acceleration")

    dynamics = build_analytical_dynamics()
    x_dot = np.empty_like(state)
    for t in range(state.shape[0]):
        x_dot[t, :] = np.array(dynamics(x=state[t, :], u=control[t, :])["x_dot"]).squeeze()
    return x_dot[:, 3:6]


def compute_neural_compensation(
    model_instance_id: int,
    state: np.ndarray,
    control: np.ndarray,
    device: torch.device,
) -> np.ndarray:
    """Compute neural compensation term in world frame (shape: [T, 3])."""
    model_options = dict(EnvConfig.model_options)
    model_options["neural_model_instance"] = f"neuralmodel_{model_instance_id}"

    neural_model, mlp_metadata = load_model(model_options, sim_options=None, run_options=None, device=device)
    neural_model.eval()

    input_transform = bool(mlp_metadata["ModelFitConfig"]["input_transform"])
    label_transform = bool(mlp_metadata["ModelFitConfig"]["label_transform"])

    if input_transform:
        state_in_mlp = velocity_mapping(state)
    else:
        state_in_mlp = state.copy()

    state_feats = np.array(eval(mlp_metadata["ModelFitConfig"]["state_feats"]))
    u_feats = np.array(eval(mlp_metadata["ModelFitConfig"]["u_feats"]))

    state_tensor = torch.from_numpy(state_in_mlp).to(dtype=torch.float32, device=device)
    control_tensor = torch.from_numpy(control).to(dtype=torch.float32, device=device)
    mlp_in = torch.cat((state_tensor[:, state_feats], control_tensor[:, u_feats]), dim=1)

    with torch.no_grad():
        mlp_out = neural_model(mlp_in).detach().cpu().numpy()

    # If needed, map prediction back into world frame using current quaternion.
    if label_transform:
        for t in range(state.shape[0]):
            mlp_out[t, :] = v_dot_q(mlp_out[t, :], state[t, 6:10])

    if mlp_out.shape[1] != 3:
        raise ValueError(
            f"Expected neural model output dim 3 for acceleration compensation, got {mlp_out.shape[1]}"
        )
    return mlp_out

def visualize_results():
    setup_plot_style()

    # Load CSVs
    # analytical_result_path = "RESULTS_CDC2026_real_machine_MODE_10/dataset_001.csv"
    # neural_result_1_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_211/dataset_001.csv"
    # neural_result_2_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_213/dataset_001.csv"
    # neural_result_3_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_214/dataset_001.csv"


    # analytical_result_path = "RESULTS_CDC2026_real_machine_MODE_10_TAKEOFF/dataset_001.csv"
    # neural_result_1_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_211_TAKEOFF/dataset_001.csv"
    # # neural_result_2_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_213_TAKEOFF/dataset_001.csv"
    # neural_result_3_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_214_TAKEOFF/dataset_001.csv"


    # analytical_result_path = "RESULTS_CDC2026_real_machine_MODE_10_CIRCLE/dataset_001.csv"
    # neural_result_1_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_211_CIRCLE/dataset_001.csv"
    # # neural_result_2_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_213_CIRCLE/dataset_001.csv"
    # neural_result_3_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_214_CIRCLE/dataset_001.csv"


    # analytical_result_path = "RESULTS_CDC2026_real_machine_MODE_10_LEMNISCATE/dataset_001.csv"
    # neural_result_1_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_211_LEMNISCATE/dataset_001.csv"
    # # neural_result_2_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_213_LEMNISCATE/dataset_001.csv"
    # neural_result_3_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_214_LEMNISCATE/dataset_001.csv"


    analytical_result_path = "RESULTS_CDC2026_real_machine_MODE_10_SETPOINT/dataset_001.csv"
    neural_result_1_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_211_SETPOINT/dataset_001.csv"
    # neural_result_2_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_213_SETPOINT/dataset_001.csv"
    neural_result_3_path = "RESULTS_CDC2026_real_machine_MODE_11_MODEL_214_SETPOINT/dataset_001.csv"

    # Read data
    paths = [
        ("Analytical MPC", analytical_result_path, None),
        ("RTNMPC", neural_result_1_path, 211),
        ("Ours", neural_result_3_path, 214),
    ]

    data = {}
    for name, path, model_id in paths:
        d = load_data(path)
        ts = d["timestamp"] - d["timestamp"][0]
        data[name] = {
            "ts": ts,
            "state": d["state"],
            "state_ref": d["state_ref"],
            "control": d["control"],
            "dt": d["dt"],
            "model_id": model_id,
            "filepath": d["filepath"],
        }

    figsize = (7, 7)
    single_figsize = (7, 4)

    if "TAKEOFF" in analytical_result_path:
        title = "Takeoff Trajectory"
    elif "CIRCLE" in analytical_result_path:
        title = "Circle Trajectory"
    elif "LEMNISCATE" in analytical_result_path:
        title = "Lemniscate Trajectory"
    elif "SETPOINT" in analytical_result_path:
        title = "Setpoint Trajectory"
    else:
        title = ""

    # Crop to the shortest trajectory (prettier, aligned comparisons)
    min_dur = min(float(np.max(d["ts"])) for d in data.values())
    if "LEMNISCATE" in analytical_result_path:
        # Truncate the last 2 seconds (legacy setting)
        min_dur = 18.0
    for name in data:
        idx = np.where(data[name]["ts"] <= min_dur)[0]
        data[name]["ts"] = data[name]["ts"][idx]
        data[name]["state"] = data[name]["state"][idx]
        data[name]["state_ref"] = data[name]["state_ref"][idx]
        if data[name]["control"] is not None:
            data[name]["control"] = data[name]["control"][idx]
        if data[name]["dt"] is not None:
            data[name]["dt"] = data[name]["dt"][idx]


    colors = {
        "Analytical MPC": matlab_blue,
        "RTNMPC": matlab_yellow,
        "Ours": matlab_purple,
    }

    # --- Position
    fig_p, axs_p = plt.subplots(3, 1, figsize=figsize, sharex=True)

    labels_p = [r"$x$ [m]", r"$y$ [m]", r"$z$ [m]"]

    # Plot reference once (from analytical run)
    analytical_key = "Analytical MPC"
    ts_nom = data[analytical_key]["ts"]
    ref_nom = data[analytical_key]["state_ref"]
    for i in range(3):
        axs_p[i].plot(
            ts_nom,
            ref_nom[:, i],
            linestyle=DENSE_DOTTED,
            color="black",
            alpha=1.0,
            label="Reference",
        )
        if i == 0:
            axs_p[i].set_title(
                latex_heading(title)
            )

    for name, d in data.items():
        ts, state = d["ts"], d["state"]
        c = colors.get(name, None)
        if d["model_id"] is None:
            linestyle = LOOSELY_DASHED
            alpha = 0.95
        else:
            linestyle = "-"
            alpha = 1.0
        for i in range(3):
            axs_p[i].plot(ts, state[:, i], color=c, label=name, linestyle=linestyle, alpha=alpha)
            axs_p[i].set_ylabel(labels_p[i])
            axs_p[i].grid("on")
            axs_p[i].set_xlim(0.0, min_dur)

    # Fewer y tick labels on the y-position subplot (second axis)
    axs_p[1].yaxis.set_major_locator(mticker.MaxNLocator(nbins=4))
    axs_p[-1].set_xlabel(r"$t$ [s]")
    axs_p[0].legend(loc="upper right", ncol=1, fancybox=True, framealpha=LEGEND_FRAME_ALPHA)
    fig_p.tight_layout(rect=[0, 0, 1, FIG_TIGHT_LAYOUT_TOP])

    # --- Quaternion
    fig_q, axs_q = plt.subplots(4, 1, figsize=figsize, sharex=True)
    labels_q = [r"$q_w$", r"$q_x$", r"$q_y$", r"$q_z$"]

    ref_nom_q = data[analytical_key]["state_ref"]
    for i in range(4):
        axs_q[i].plot(
            ts_nom,
            ref_nom_q[:, 6 + i],
            linestyle=DENSE_DOTTED,
            color="black",
            alpha=1.0,
            label="Reference",
        )
        if i == 0:
            axs_q[i].set_title(
                latex_heading(title)
            )


    for name, d in data.items():
        ts, state = d["ts"], d["state"]
        c = colors.get(name, None)
        if d["model_id"] is None:
            linestyle = LOOSELY_DASHED
            alpha = 0.95
        else:
            linestyle = "-"
            alpha = 1.0
        for i in range(4):
            axs_q[i].plot(ts, state[:, 6 + i], color=c, label=name, linestyle=linestyle, alpha=alpha)
            axs_q[i].set_ylabel(labels_q[i])
            axs_q[i].grid("on")
            axs_q[i].set_xlim(0.0, min_dur)
    axs_q[-1].set_xlabel(r"$t$ [s]")
    axs_q[0].legend(loc="upper right", ncol=1, fancybox=True, framealpha=LEGEND_FRAME_ALPHA)
    fig_q.tight_layout(rect=[0, 0, 1, FIG_TIGHT_LAYOUT_TOP])

    # --- Tracking errors
    fig_err = plt.figure(figsize=single_figsize)
    plt.title(
        latex_heading(title)
    )
    plt.ylabel(r"$\|\boldsymbol{p} - \boldsymbol{p}_\mathrm{ref}\|$ [m]")
    plt.xlabel(r"$t$ [s]")
    plt.grid("on")
    print("--- Average Tracking Errors ---")
    
    for name, d in data.items():
        p_err, q_err = compute_errors(d["state"], d["state_ref"])
        
        # Plot Position Error
        if d["model_id"] is None:
            linestyle = VERY_LOOSELY_DASHED
            alpha = 0.95
        else:
            linestyle = "-"
            alpha = 1.0
        plt.plot(d["ts"], p_err, label=name, color=colors.get(name, None), linestyle=linestyle, alpha=alpha)
        
        # Print average error
        avg_p_err = np.mean(p_err)  # MAE for position error (in meters)
        avg_q_err = np.mean(q_err)  # MAE for attitude error (quaternion error norm squared)
        print(f"{name}:")
        print(f"  Position Error: {avg_p_err:.4f} m")
        print(f"  Attitude Error: {avg_q_err:.4f}")

    plt.xlim(0.0, min_dur)
    plt.legend(loc="upper right", ncol=1, fancybox=True, framealpha=LEGEND_FRAME_ALPHA)
    fig_err.tight_layout(rect=[0, 0, 1, FIG_TIGHT_LAYOUT_TOP])
    # fig_err, axs_err = plt.subplots(1, 2, figsize=(14, 6))
    # fig_err.suptitle("Tracking Errors Comparison")
    
    # axs_err[0].set_title("Position Tracking Error")
    # axs_err[0].set_ylabel("Error Norm [m]")
    # axs_err[0].set_xlabel("Time [s]")
    # axs_err[0].grid(True)
    
    # axs_err[1].set_title("Attitude Tracking Error")
    # axs_err[1].set_ylabel("Quat Error Norm")
    # axs_err[1].set_xlabel("Time [s]")
    # axs_err[1].grid(True)
    
    # print("--- Average Tracking Errors ---")
    
    # for name, d in data.items():
    #     p_err, q_err = compute_errors(d["state"], d["state_ref"])
        
    #     is_analytical = (name == "Nominal MPC")
    #     lw = 2.5 if is_analytical else 1.5
    #     ls = '-' if is_analytical else '--'
    #     zorder = 5 if is_analytical else 2
        
    #     # Plot Position Error
    #     axs_err[0].plot(d["ts"], p_err, label=name, linewidth=lw, linestyle=ls, zorder=zorder)
        
    #     # Plot Attitude Error
    #     axs_err[1].plot(d["ts"], q_err, label=name, linewidth=lw, linestyle=ls, zorder=zorder)
        
    #     # Print average error
    #     avg_p_err = np.mean(p_err)  # MAE for position error (in meters)
    #     avg_q_err = np.mean(q_err)  # MAE for attitude error (quaternion error norm squared)
    #     print(f"{name}:")
    #     print(f"  Position Error: {avg_p_err:.4f} m")
    #     print(f"  Attitude Error: {avg_q_err:.4f}")

    # axs_err[0].legend()
    # axs_err[1].legend()

    # --- Acceleration comparison (analytical vs analytical + neural compensation)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    fig_a = plt.figure(figsize=figsize)
    axs_a = [plt.subplot(3, 1, i + 1) for i in range(3)]
    a_labels = [r"$a_x$ [m/s$^2$]", r"$a_y$ [m/s$^2$]", r"$a_z$ [m/s$^2$]"]

    for name, d in data.items():
        ts = d["ts"]
        state = d["state"]
        control = d["control"]
        if control is None:
            continue

        lin_acc = compute_analytical_linear_acc(state, control)

        if d["model_id"] is None:
            # Nominal acceleration from the analytical model
            for i in range(3):
                axs_a[i].plot(
                    ts,
                    lin_acc[:, i],
                    color=colors.get(name, None),
                    linestyle=VERY_LOOSELY_DASHED,
                    alpha=0.95,
                    label=f"{name}" if i == 0 else None,
                )

        # Neural compensated acceleration (only for neural runs)
        if d["model_id"] is not None:
            comp = compute_neural_compensation(d["model_id"], state, control, device=device)
            acc_comp = lin_acc + comp
            for i in range(3):
                axs_a[i].plot(
                    ts,
                    acc_comp[:, i],
                    color=colors.get(name, None),
                    linestyle="-",
                    alpha=1.0,
                    label=f"{name}" if i == 0 else None,
                )

    for i in range(3):
        axs_a[i].set_ylabel(a_labels[i])
        axs_a[i].grid("on")
        axs_a[i].set_xlim(0.0, min_dur)
        if i < 2:
            axs_a[i].axes.xaxis.set_ticklabels([])

        if i == 0:
            axs_a[i].set_title(
                latex_heading(title)
            )

    axs_a[-1].set_xlabel(r"$t$ [s]")
    axs_a[0].legend(loc="upper right", ncol=1, fancybox=True, framealpha=LEGEND_FRAME_ALPHA)

    plt.tight_layout(rect=[0, 0, 1, FIG_TIGHT_LAYOUT_TOP])
    plt.show()

if __name__ == "__main__":
    visualize_results()