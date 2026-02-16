import os, sys
import time
import json
import numpy as np
import pandas as pd
from config.configurations import DirectoryConfig, ModelFitConfig
from utils.data_utils import safe_mkdir_recursive, jsonify, safe_mkfile_recursive
from sim_environment.forward_prop import init_forward_prop, forward_prop
from utils.filter_utils import moving_average_filter, low_pass_filter

# Only needed to get T_samp
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from neural_controller import NeuralMPC
from config.configurations import EnvConfig
model_options = EnvConfig.model_options
model_options["only_use_nominal"] = True
neural_mpc = NeuralMPC(
    model_options,
    EnvConfig.solver_options,
    EnvConfig.sim_options,
    EnvConfig.run_options,
)
T_samp = neural_mpc.params["T_samp"]

def get_synched_data_from_rosbag(file_path: str, apply_temporal_filter: bool) -> dict:
    """
    Load and read rosbags from csv file.
    Remove NaN values and synchronize topics based on timestamp. The guiding time series is the thrust command.
    Filter out inconsistent timesteps if apply_temporal_filter is set to True. Steps too close together are removed.
    Steps too far apart are artifically shortened to the ideal time step.
    ASSUMPTION: the temporal recording data is inconsistent due to process lag on the real-machine but the measurements
    are in fact accurately time consistent.
    Returns a dictionary with keys as the topic names and messages as values.
    """

    ############## Read csv file ##############
    df = pd.read_csv(file_path)

    # Position
    data_xyz = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[0]/position/x",
            "/beetle1/nmpc/record_pred/states[0]/position/y",
            "/beetle1/nmpc/record_pred/states[0]/position/z",
        ]
    ]
    data_xyz = data_xyz.dropna()

    # Velocity
    data_vel = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[0]/linear_velocity/x",
            "/beetle1/nmpc/record_pred/states[0]/linear_velocity/y",
            "/beetle1/nmpc/record_pred/states[0]/linear_velocity/z",
        ]
    ]
    data_vel = data_vel.dropna()

    # Quaternion
    data_qwxyz = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[0]/orientation/w",
            "/beetle1/nmpc/record_pred/states[0]/orientation/x",
            "/beetle1/nmpc/record_pred/states[0]/orientation/y",
            "/beetle1/nmpc/record_pred/states[0]/orientation/z",
        ]
    ]
    data_qwxyz = data_qwxyz.dropna()

    # Avoid sign flip of the quaternion
    # This is quite important to maintain continuity of the quaternion signal
    qw = data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/w"].to_numpy()
    qx = data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/x"].to_numpy()
    qy = data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/y"].to_numpy()
    qz = data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/z"].to_numpy()
    qw_prev = 1.0
    qx_prev = 0.0
    qy_prev = 0.0
    qz_prev = 0.0
    for i in range(len(qw)):
        qe_c_w = qw[i] * qw_prev + qx[i] * qx_prev + qy[i] * qy_prev + qz[i] * qz_prev
        if qe_c_w < 0:
            # Negate sign of quaternion at i
            qw[i] = -qw[i]
            qx[i] = -qx[i]
            qy[i] = -qy[i]
            qz[i] = -qz[i]

        qw_prev = qw[i]
        qx_prev = qx[i]
        qy_prev = qy[i]
        qz_prev = qz[i]
    data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/w"] = qw
    data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/x"] = qx
    data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/y"] = qy
    data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/z"] = qz

    # Angular velocity
    data_ang_vel = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[0]/angular_velocity/x",
            "/beetle1/nmpc/record_pred/states[0]/angular_velocity/y",
            "/beetle1/nmpc/record_pred/states[0]/angular_velocity/z",
        ]
    ]
    data_ang_vel = data_ang_vel.dropna()

    # Servo angle state
    data_alpha_s = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[0]/servo_angles[0]",
            "/beetle1/nmpc/record_pred/states[0]/servo_angles[1]",
            "/beetle1/nmpc/record_pred/states[0]/servo_angles[2]",
            "/beetle1/nmpc/record_pred/states[0]/servo_angles[3]",
        ]
    ]
    data_alpha_s = data_alpha_s.dropna()

    # Linear acceleration in Body (from IMU)
    data_lin_acc_b = df[
        [
            "__time",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/x",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/y",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/z",
        ]
    ]
    data_lin_acc_b = data_lin_acc_b.dropna()

    # Linear acceleration in World (from IMU)
    data_lin_acc_w = df[
        [
            "__time",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/x",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/y",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/z",
        ]
    ]
    data_lin_acc_w = data_lin_acc_w.dropna()

    # Thrust command
    data_thrust_cmd = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/controls[0]/thrust_commands[0]",
            "/beetle1/nmpc/record_pred/controls[0]/thrust_commands[1]",
            "/beetle1/nmpc/record_pred/controls[0]/thrust_commands[2]",
            "/beetle1/nmpc/record_pred/controls[0]/thrust_commands[3]",
        ]
    ]
    data_thrust_cmd = data_thrust_cmd.dropna()

    # Servo angle command
    data_servo_angle_cmd = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[0]",
            "/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[1]",
            "/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[2]",
            "/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[3]",
        ]
    ]
    data_servo_angle_cmd = data_servo_angle_cmd.dropna()

    # Predicted position
    data_xyz_pred = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[1]/position/x",
            "/beetle1/nmpc/record_pred/states[1]/position/y",
            "/beetle1/nmpc/record_pred/states[1]/position/z",
        ]
    ]
    data_xyz_pred = data_xyz_pred.dropna()

    # Predicted velocity
    data_vel_pred = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[1]/linear_velocity/x",
            "/beetle1/nmpc/record_pred/states[1]/linear_velocity/y",
            "/beetle1/nmpc/record_pred/states[1]/linear_velocity/z",
        ]
    ]
    data_vel_pred = data_vel_pred.dropna()

    # Predicted quaternion
    data_qwxyz_pred = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[1]/orientation/w",
            "/beetle1/nmpc/record_pred/states[1]/orientation/x",
            "/beetle1/nmpc/record_pred/states[1]/orientation/y",
            "/beetle1/nmpc/record_pred/states[1]/orientation/z",
        ]
    ]
    data_qwxyz_pred = data_qwxyz_pred.dropna()

    # Avoid sign flip of the quaternion
    # This is quite important to maintain continuity of the quaternion signal
    qw = data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/w"].to_numpy()
    qx = data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/x"].to_numpy()
    qy = data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/y"].to_numpy()
    qz = data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/z"].to_numpy()
    qw_prev = 1.0
    qx_prev = 0.0
    qy_prev = 0.0
    qz_prev = 0.0
    for i in range(len(qw)):
        qe_c_w = qw[i] * qw_prev + qx[i] * qx_prev + qy[i] * qy_prev + qz[i] * qz_prev
        if qe_c_w < 0:
            # Negate sign of quaternion at i
            qw[i] = -qw[i]
            qx[i] = -qx[i]
            qy[i] = -qy[i]
            qz[i] = -qz[i]

        qw_prev = qw[i]
        qx_prev = qx[i]
        qy_prev = qy[i]
        qz_prev = qz[i]
    data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/w"] = qw
    data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/x"] = qx
    data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/y"] = qy
    data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/z"] = qz

    # Predicted angular velocity
    data_ang_vel_pred = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[1]/angular_velocity/x",
            "/beetle1/nmpc/record_pred/states[1]/angular_velocity/y",
            "/beetle1/nmpc/record_pred/states[1]/angular_velocity/z",
        ]
    ]
    data_ang_vel_pred = data_ang_vel_pred.dropna()

    # Predicted servo angle state
    data_alpha_s_pred = df[
        [
            "__time",
            "/beetle1/nmpc/record_pred/states[1]/servo_angles[0]",
            "/beetle1/nmpc/record_pred/states[1]/servo_angles[1]",
            "/beetle1/nmpc/record_pred/states[1]/servo_angles[2]",
            "/beetle1/nmpc/record_pred/states[1]/servo_angles[3]",
        ]
    ]
    data_alpha_s_pred = data_alpha_s_pred.dropna()

    # Reference position
    data_xyz_ref = df[
        [
            "__time",
            "/beetle1/nmpc/record_ref/states[0]/position/x",
            "/beetle1/nmpc/record_ref/states[0]/position/y",
            "/beetle1/nmpc/record_ref/states[0]/position/z",
        ]
    ]
    data_xyz_ref = data_xyz_ref.dropna()

    # Predicted velocity
    data_vel_ref = df[
        [
            "__time",
            "/beetle1/nmpc/record_ref/states[0]/linear_velocity/x",
            "/beetle1/nmpc/record_ref/states[0]/linear_velocity/y",
            "/beetle1/nmpc/record_ref/states[0]/linear_velocity/z",
        ]
    ]
    data_vel_ref = data_vel_ref.dropna()

    # Reference quaternion
    data_qwxyz_ref = df[
        [
            "__time",
            "/beetle1/nmpc/record_ref/states[0]/orientation/w",
            "/beetle1/nmpc/record_ref/states[0]/orientation/x",
            "/beetle1/nmpc/record_ref/states[0]/orientation/y",
            "/beetle1/nmpc/record_ref/states[0]/orientation/z",
        ]
    ]
    data_qwxyz_ref = data_qwxyz_ref.dropna()

    # Avoid sign flip of the quaternion
    qwr = data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/w"].to_numpy()
    qxr = data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/x"].to_numpy()
    qyr = data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/y"].to_numpy()
    qzr = data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/z"].to_numpy()
    qwr_prev = 1.0
    qxr_prev = 0.0
    qyr_prev = 0.0
    qzr_prev = 0.0
    for i in range(len(qwr)):
        qe_c_w = qwr[i] * qwr_prev + qxr[i] * qxr_prev + qyr[i] * qyr_prev + qzr[i] * qzr_prev
        if qe_c_w < 0:
            # Negate sign of quaternion at i
            qwr[i] = -qwr[i]
            qxr[i] = -qxr[i]
            qyr[i] = -qyr[i]
            qzr[i] = -qzr[i]

        qwr_prev = qwr[i]
        qxr_prev = qxr[i]
        qyr_prev = qyr[i]
        qzr_prev = qzr[i]
    data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/w"] = qwr
    data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/x"] = qxr
    data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/y"] = qyr
    data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/z"] = qzr

    # Reference angular velocity
    data_ang_vel_ref = df[
        [
            "__time",
            "/beetle1/nmpc/record_ref/states[0]/angular_velocity/x",
            "/beetle1/nmpc/record_ref/states[0]/angular_velocity/y",
            "/beetle1/nmpc/record_ref/states[0]/angular_velocity/z",
        ]
    ]
    data_ang_vel_ref = data_ang_vel_ref.dropna()

    # Reference servo angle state
    data_alpha_s_ref = df[
        [
            "__time",
            "/beetle1/nmpc/record_ref/states[0]/servo_angles[0]",
            "/beetle1/nmpc/record_ref/states[0]/servo_angles[1]",
            "/beetle1/nmpc/record_ref/states[0]/servo_angles[2]",
            "/beetle1/nmpc/record_ref/states[0]/servo_angles[3]",
        ]
    ]
    data_alpha_s_ref = data_alpha_s_ref.dropna()

    # Reference thrust command
    data_thrust_cmd_ref = df[
        [
            "__time",
            "/beetle1/nmpc/record_ref/controls[0]/thrust_commands[0]",
            "/beetle1/nmpc/record_ref/controls[0]/thrust_commands[1]",
            "/beetle1/nmpc/record_ref/controls[0]/thrust_commands[2]",
            "/beetle1/nmpc/record_ref/controls[0]/thrust_commands[3]",
        ]
    ]
    data_thrust_cmd_ref = data_thrust_cmd_ref.dropna()

    # Reference servo angle command
    data_servo_angle_cmd_ref = df[
        [
            "__time",
            "/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[0]",
            "/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[1]",
            "/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[2]",
            "/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[3]",
        ]
    ]
    data_servo_angle_cmd_ref = data_servo_angle_cmd_ref.dropna()

    ############## Synchronize ##############
    # Interpolate all data to synchronize w.r.t. the time series of the thrust command using np.interp() [from numpy documentation]:
    # "
    # - One-dimensional linear interpolation for monotonically increasing sample points.
    # - Returns the one-dimensional piecewise linear interpolant to a function with given discrete data points (xp, fp), evaluated at x.
    # "

    # Reference timestamps
    t_ref = np.array(data_thrust_cmd["__time"])

    # Position
    t = np.array(data_xyz["__time"])
    data_xyz_interp = pd.DataFrame()
    data_xyz_interp["__time"] = t_ref
    data_xyz_interp["/beetle1/nmpc/record_pred/states[0]/position/x"] = np.interp(
        t_ref, t, data_xyz["/beetle1/nmpc/record_pred/states[0]/position/x"]
    )
    data_xyz_interp["/beetle1/nmpc/record_pred/states[0]/position/y"] = np.interp(
        t_ref, t, data_xyz["/beetle1/nmpc/record_pred/states[0]/position/y"]
    )
    data_xyz_interp["/beetle1/nmpc/record_pred/states[0]/position/z"] = np.interp(
        t_ref, t, data_xyz["/beetle1/nmpc/record_pred/states[0]/position/z"]
    )

    # Velocity
    t = np.array(data_vel["__time"])
    data_vel_interp = pd.DataFrame()
    data_vel_interp["__time"] = t_ref
    data_vel_interp["/beetle1/nmpc/record_pred/states[0]/linear_velocity/x"] = np.interp(
        t_ref, t, data_vel["/beetle1/nmpc/record_pred/states[0]/linear_velocity/x"]
    )
    data_vel_interp["/beetle1/nmpc/record_pred/states[0]/linear_velocity/y"] = np.interp(
        t_ref, t, data_vel["/beetle1/nmpc/record_pred/states[0]/linear_velocity/y"]
    )
    data_vel_interp["/beetle1/nmpc/record_pred/states[0]/linear_velocity/z"] = np.interp(
        t_ref, t, data_vel["/beetle1/nmpc/record_pred/states[0]/linear_velocity/z"]
    )

    # Quaternion
    t = np.array(data_qwxyz["__time"])
    data_qwxyz_interp = pd.DataFrame()
    data_qwxyz_interp["__time"] = t_ref
    data_qwxyz_interp["/beetle1/nmpc/record_pred/states[0]/orientation/w"] = np.interp(
        t_ref, t, data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/w"]
    )
    data_qwxyz_interp["/beetle1/nmpc/record_pred/states[0]/orientation/x"] = np.interp(
        t_ref, t, data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/x"]
    )
    data_qwxyz_interp["/beetle1/nmpc/record_pred/states[0]/orientation/y"] = np.interp(
        t_ref, t, data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/y"]
    )
    data_qwxyz_interp["/beetle1/nmpc/record_pred/states[0]/orientation/z"] = np.interp(
        t_ref, t, data_qwxyz["/beetle1/nmpc/record_pred/states[0]/orientation/z"]
    )

    # Angular velocity
    t = np.array(data_ang_vel["__time"])
    data_ang_vel_interp = pd.DataFrame()
    data_ang_vel_interp["__time"] = t_ref
    data_ang_vel_interp["/beetle1/nmpc/record_pred/states[0]/angular_velocity/x"] = np.interp(
        t_ref, t, data_ang_vel["/beetle1/nmpc/record_pred/states[0]/angular_velocity/x"]
    )
    data_ang_vel_interp["/beetle1/nmpc/record_pred/states[0]/angular_velocity/y"] = np.interp(
        t_ref, t, data_ang_vel["/beetle1/nmpc/record_pred/states[0]/angular_velocity/y"]
    )
    data_ang_vel_interp["/beetle1/nmpc/record_pred/states[0]/angular_velocity/z"] = np.interp(
        t_ref, t, data_ang_vel["/beetle1/nmpc/record_pred/states[0]/angular_velocity/z"]
    )

    # Servo angle state
    t = np.array(data_alpha_s["__time"])
    data_alpha_s_interp = pd.DataFrame()
    data_alpha_s_interp["__time"] = t_ref
    data_alpha_s_interp["/beetle1/nmpc/record_pred/states[0]/servo_angles[0]"] = np.interp(
        t_ref, t, data_alpha_s["/beetle1/nmpc/record_pred/states[0]/servo_angles[0]"]
    )
    data_alpha_s_interp["/beetle1/nmpc/record_pred/states[0]/servo_angles[1]"] = np.interp(
        t_ref, t, data_alpha_s["/beetle1/nmpc/record_pred/states[0]/servo_angles[1]"]
    )
    data_alpha_s_interp["/beetle1/nmpc/record_pred/states[0]/servo_angles[2]"] = np.interp(
        t_ref, t, data_alpha_s["/beetle1/nmpc/record_pred/states[0]/servo_angles[2]"]
    )
    data_alpha_s_interp["/beetle1/nmpc/record_pred/states[0]/servo_angles[3]"] = np.interp(
        t_ref, t, data_alpha_s["/beetle1/nmpc/record_pred/states[0]/servo_angles[3]"]
    )

    # Linear acceleration in Body
    t = np.array(data_lin_acc_b["__time"])
    data_lin_acc_b_interp = pd.DataFrame()
    data_lin_acc_b_interp["__time"] = t_ref
    data_lin_acc_b_interp["/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/x"] = np.interp(
        t_ref, t, data_lin_acc_b["/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/x"]
    )
    data_lin_acc_b_interp["/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/y"] = np.interp(
        t_ref, t, data_lin_acc_b["/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/y"]
    )
    data_lin_acc_b_interp["/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/z"] = np.interp(
        t_ref, t, data_lin_acc_b["/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/z"]
    )

    # Linear acceleration in World
    t = np.array(data_lin_acc_w["__time"])
    data_lin_acc_w_interp = pd.DataFrame()
    data_lin_acc_w_interp["__time"] = t_ref
    data_lin_acc_w_interp["/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/x"] = np.interp(
        t_ref, t, data_lin_acc_w["/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/x"]
    )
    data_lin_acc_w_interp["/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/y"] = np.interp(
        t_ref, t, data_lin_acc_w["/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/y"]
    )
    data_lin_acc_w_interp["/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/z"] = np.interp(
        t_ref, t, data_lin_acc_w["/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/z"]
    )

    # Thrust command
    # NOTE: No need to interpolate since this is the reference time series

    # Servo angle command
    t = np.array(data_servo_angle_cmd["__time"])
    data_servo_angle_cmd_interp = pd.DataFrame()
    data_servo_angle_cmd_interp["__time"] = t_ref
    data_servo_angle_cmd_interp["/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[0]"] = np.interp(
        t_ref, t, data_servo_angle_cmd["/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[0]"]
    )
    data_servo_angle_cmd_interp["/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[1]"] = np.interp(
        t_ref, t, data_servo_angle_cmd["/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[1]"]
    )
    data_servo_angle_cmd_interp["/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[2]"] = np.interp(
        t_ref, t, data_servo_angle_cmd["/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[2]"]
    )
    data_servo_angle_cmd_interp["/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[3]"] = np.interp(
        t_ref, t, data_servo_angle_cmd["/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[3]"]
    )

    # Predicted position
    t = np.array(data_xyz_pred["__time"])
    data_xyz_pred_interp = pd.DataFrame()
    data_xyz_pred_interp["__time"] = t_ref
    data_xyz_pred_interp["/beetle1/nmpc/record_pred/states[1]/position/x"] = np.interp(
        t_ref, t, data_xyz_pred["/beetle1/nmpc/record_pred/states[1]/position/x"]
    )
    data_xyz_pred_interp["/beetle1/nmpc/record_pred/states[1]/position/y"] = np.interp(
        t_ref, t, data_xyz_pred["/beetle1/nmpc/record_pred/states[1]/position/y"]
    )
    data_xyz_pred_interp["/beetle1/nmpc/record_pred/states[1]/position/z"] = np.interp(
        t_ref, t, data_xyz_pred["/beetle1/nmpc/record_pred/states[1]/position/z"]
    )

    # Predicted velocity
    t = np.array(data_vel_pred["__time"])
    data_vel_pred_interp = pd.DataFrame()
    data_vel_pred_interp["__time"] = t_ref
    data_vel_pred_interp["/beetle1/nmpc/record_pred/states[1]/linear_velocity/x"] = np.interp(
        t_ref, t, data_vel_pred["/beetle1/nmpc/record_pred/states[1]/linear_velocity/x"]
    )
    data_vel_pred_interp["/beetle1/nmpc/record_pred/states[1]/linear_velocity/y"] = np.interp(
        t_ref, t, data_vel_pred["/beetle1/nmpc/record_pred/states[1]/linear_velocity/y"]
    )
    data_vel_pred_interp["/beetle1/nmpc/record_pred/states[1]/linear_velocity/z"] = np.interp(
        t_ref, t, data_vel_pred["/beetle1/nmpc/record_pred/states[1]/linear_velocity/z"]
    )

    # Predicted quaternion
    t = np.array(data_qwxyz_pred["__time"])
    data_qwxyz_pred_interp = pd.DataFrame()
    data_qwxyz_pred_interp["__time"] = t_ref
    data_qwxyz_pred_interp["/beetle1/nmpc/record_pred/states[1]/orientation/w"] = np.interp(
        t_ref, t, data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/w"]
    )
    data_qwxyz_pred_interp["/beetle1/nmpc/record_pred/states[1]/orientation/x"] = np.interp(
        t_ref, t, data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/x"]
    )
    data_qwxyz_pred_interp["/beetle1/nmpc/record_pred/states[1]/orientation/y"] = np.interp(
        t_ref, t, data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/y"]
    )
    data_qwxyz_pred_interp["/beetle1/nmpc/record_pred/states[1]/orientation/z"] = np.interp(
        t_ref, t, data_qwxyz_pred["/beetle1/nmpc/record_pred/states[1]/orientation/z"]
    )

    # Predicted angular velocity
    t = np.array(data_ang_vel_pred["__time"])
    data_ang_vel_pred_interp = pd.DataFrame()
    data_ang_vel_pred_interp["__time"] = t_ref
    data_ang_vel_pred_interp["/beetle1/nmpc/record_pred/states[1]/angular_velocity/x"] = np.interp(
        t_ref, t, data_ang_vel_pred["/beetle1/nmpc/record_pred/states[1]/angular_velocity/x"]
    )
    data_ang_vel_pred_interp["/beetle1/nmpc/record_pred/states[1]/angular_velocity/y"] = np.interp(
        t_ref, t, data_ang_vel_pred["/beetle1/nmpc/record_pred/states[1]/angular_velocity/y"]
    )
    data_ang_vel_pred_interp["/beetle1/nmpc/record_pred/states[1]/angular_velocity/z"] = np.interp(
        t_ref, t, data_ang_vel_pred["/beetle1/nmpc/record_pred/states[1]/angular_velocity/z"]
    )

    # Predicted servo angle state
    t = np.array(data_alpha_s_pred["__time"])
    data_alpha_s_pred_interp = pd.DataFrame()
    data_alpha_s_pred_interp["__time"] = t_ref
    data_alpha_s_pred_interp["/beetle1/nmpc/record_pred/states[1]/servo_angles[0]"] = np.interp(
        t_ref, t, data_alpha_s_pred["/beetle1/nmpc/record_pred/states[1]/servo_angles[0]"]
    )
    data_alpha_s_pred_interp["/beetle1/nmpc/record_pred/states[1]/servo_angles[1]"] = np.interp(
        t_ref, t, data_alpha_s_pred["/beetle1/nmpc/record_pred/states[1]/servo_angles[1]"]
    )
    data_alpha_s_pred_interp["/beetle1/nmpc/record_pred/states[1]/servo_angles[2]"] = np.interp(
        t_ref, t, data_alpha_s_pred["/beetle1/nmpc/record_pred/states[1]/servo_angles[2]"]
    )
    data_alpha_s_pred_interp["/beetle1/nmpc/record_pred/states[1]/servo_angles[3]"] = np.interp(
        t_ref, t, data_alpha_s_pred["/beetle1/nmpc/record_pred/states[1]/servo_angles[3]"]
    )

    # Reference position
    t = np.array(data_xyz_ref["__time"])
    data_xyz_ref_interp = pd.DataFrame()
    data_xyz_ref_interp["__time"] = t_ref
    data_xyz_ref_interp["/beetle1/nmpc/record_ref/states[0]/position/x"] = np.interp(
        t_ref, t, data_xyz_ref["/beetle1/nmpc/record_ref/states[0]/position/x"]
    )
    data_xyz_ref_interp["/beetle1/nmpc/record_ref/states[0]/position/y"] = np.interp(
        t_ref, t, data_xyz_ref["/beetle1/nmpc/record_ref/states[0]/position/y"]
    )
    data_xyz_ref_interp["/beetle1/nmpc/record_ref/states[0]/position/z"] = np.interp(
        t_ref, t, data_xyz_ref["/beetle1/nmpc/record_ref/states[0]/position/z"]
    )

    # Reference velocity
    t = np.array(data_vel_ref["__time"])
    data_vel_ref_interp = pd.DataFrame()
    data_vel_ref_interp["__time"] = t_ref
    data_vel_ref_interp["/beetle1/nmpc/record_ref/states[0]/linear_velocity/x"] = np.interp(
        t_ref, t, data_vel_ref["/beetle1/nmpc/record_ref/states[0]/linear_velocity/x"]
    )
    data_vel_ref_interp["/beetle1/nmpc/record_ref/states[0]/linear_velocity/y"] = np.interp(
        t_ref, t, data_vel_ref["/beetle1/nmpc/record_ref/states[0]/linear_velocity/y"]
    )
    data_vel_ref_interp["/beetle1/nmpc/record_ref/states[0]/linear_velocity/z"] = np.interp(
        t_ref, t, data_vel_ref["/beetle1/nmpc/record_ref/states[0]/linear_velocity/z"]
    )

    # Reference quaternion
    t = np.array(data_qwxyz_ref["__time"])
    data_qwxyz_ref_interp = pd.DataFrame()
    data_qwxyz_ref_interp["__time"] = t_ref
    data_qwxyz_ref_interp["/beetle1/nmpc/record_ref/states[0]/orientation/w"] = np.interp(
        t_ref, t, data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/w"]
    )
    data_qwxyz_ref_interp["/beetle1/nmpc/record_ref/states[0]/orientation/x"] = np.interp(
        t_ref, t, data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/x"]
    )
    data_qwxyz_ref_interp["/beetle1/nmpc/record_ref/states[0]/orientation/y"] = np.interp(
        t_ref, t, data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/y"]
    )
    data_qwxyz_ref_interp["/beetle1/nmpc/record_ref/states[0]/orientation/z"] = np.interp(
        t_ref, t, data_qwxyz_ref["/beetle1/nmpc/record_ref/states[0]/orientation/z"]
    )

    # Reference angular velocity
    t = np.array(data_ang_vel_ref["__time"])
    data_ang_vel_ref_interp = pd.DataFrame()
    data_ang_vel_ref_interp["__time"] = t_ref
    data_ang_vel_ref_interp["/beetle1/nmpc/record_ref/states[0]/angular_velocity/x"] = np.interp(
        t_ref, t, data_ang_vel_ref["/beetle1/nmpc/record_ref/states[0]/angular_velocity/x"]
    )
    data_ang_vel_ref_interp["/beetle1/nmpc/record_ref/states[0]/angular_velocity/y"] = np.interp(
        t_ref, t, data_ang_vel_ref["/beetle1/nmpc/record_ref/states[0]/angular_velocity/y"]
    )
    data_ang_vel_ref_interp["/beetle1/nmpc/record_ref/states[0]/angular_velocity/z"] = np.interp(
        t_ref, t, data_ang_vel_ref["/beetle1/nmpc/record_ref/states[0]/angular_velocity/z"]
    )

    # Reference servo angle state
    t = np.array(data_alpha_s_ref["__time"])
    data_alpha_s_ref_interp = pd.DataFrame()
    data_alpha_s_ref_interp["__time"] = t_ref
    data_alpha_s_ref_interp["/beetle1/nmpc/record_ref/states[0]/servo_angles[0]"] = np.interp(
        t_ref, t, data_alpha_s_ref["/beetle1/nmpc/record_ref/states[0]/servo_angles[0]"]
    )
    data_alpha_s_ref_interp["/beetle1/nmpc/record_ref/states[0]/servo_angles[1]"] = np.interp(
        t_ref, t, data_alpha_s_ref["/beetle1/nmpc/record_ref/states[0]/servo_angles[1]"]
    )
    data_alpha_s_ref_interp["/beetle1/nmpc/record_ref/states[0]/servo_angles[2]"] = np.interp(
        t_ref, t, data_alpha_s_ref["/beetle1/nmpc/record_ref/states[0]/servo_angles[2]"]
    )
    data_alpha_s_ref_interp["/beetle1/nmpc/record_ref/states[0]/servo_angles[3]"] = np.interp(
        t_ref, t, data_alpha_s_ref["/beetle1/nmpc/record_ref/states[0]/servo_angles[3]"]
    )

    # Reference thrust command
    t = np.array(data_thrust_cmd_ref["__time"])
    data_thrust_cmd_ref_interp = pd.DataFrame()
    data_thrust_cmd_ref_interp["__time"] = t_ref
    data_thrust_cmd_ref_interp["/beetle1/nmpc/record_ref/controls[0]/thrust_commands[0]"] = np.interp(
        t_ref, t, data_thrust_cmd_ref["/beetle1/nmpc/record_ref/controls[0]/thrust_commands[0]"]
    )
    data_thrust_cmd_ref_interp["/beetle1/nmpc/record_ref/controls[0]/thrust_commands[1]"] = np.interp(
        t_ref, t, data_thrust_cmd_ref["/beetle1/nmpc/record_ref/controls[0]/thrust_commands[1]"]
    )
    data_thrust_cmd_ref_interp["/beetle1/nmpc/record_ref/controls[0]/thrust_commands[2]"] = np.interp(
        t_ref, t, data_thrust_cmd_ref["/beetle1/nmpc/record_ref/controls[0]/thrust_commands[2]"]
    )
    data_thrust_cmd_ref_interp["/beetle1/nmpc/record_ref/controls[0]/thrust_commands[3]"] = np.interp(
        t_ref, t, data_thrust_cmd_ref["/beetle1/nmpc/record_ref/controls[0]/thrust_commands[3]"]
    )

    # Reference servo angle command
    t = np.array(data_servo_angle_cmd_ref["__time"])
    data_servo_angle_cmd_ref_interp = pd.DataFrame()
    data_servo_angle_cmd_ref_interp["__time"] = t_ref
    data_servo_angle_cmd_ref_interp["/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[0]"] = np.interp(
        t_ref, t, data_servo_angle_cmd_ref["/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[0]"]
    )
    data_servo_angle_cmd_ref_interp["/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[1]"] = np.interp(
        t_ref, t, data_servo_angle_cmd_ref["/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[1]"]
    )
    data_servo_angle_cmd_ref_interp["/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[2]"] = np.interp(
        t_ref, t, data_servo_angle_cmd_ref["/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[2]"]
    )
    data_servo_angle_cmd_ref_interp["/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[3]"] = np.interp(
        t_ref, t, data_servo_angle_cmd_ref["/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[3]"]
    )

    ############## Export data to dictionary ##############
    data = dict()
    data["timestamp"] = t_ref[:, np.newaxis]
    data["dt"] = np.diff(data["timestamp"], axis=0).squeeze()
    if np.any(data["dt"] == 0.0):
        raise ValueError("There are duplicate timestamps in the thrust command data.")

    data["duration"] = float(data["timestamp"][-1] - data["timestamp"][0])

    data["position"] = data_xyz_interp[
        [
            "/beetle1/nmpc/record_pred/states[0]/position/x",
            "/beetle1/nmpc/record_pred/states[0]/position/y",
            "/beetle1/nmpc/record_pred/states[0]/position/z",
        ]
    ].to_numpy()
    data["velocity"] = data_vel_interp[
        [
            "/beetle1/nmpc/record_pred/states[0]/linear_velocity/x",
            "/beetle1/nmpc/record_pred/states[0]/linear_velocity/y",
            "/beetle1/nmpc/record_pred/states[0]/linear_velocity/z",
        ]
    ].to_numpy()
    data["quaternion"] = data_qwxyz_interp[
        [
            "/beetle1/nmpc/record_pred/states[0]/orientation/w",
            "/beetle1/nmpc/record_pred/states[0]/orientation/x",
            "/beetle1/nmpc/record_pred/states[0]/orientation/y",
            "/beetle1/nmpc/record_pred/states[0]/orientation/z",
        ]
    ].to_numpy()
    data["angular_velocity"] = data_ang_vel_interp[
        [
            "/beetle1/nmpc/record_pred/states[0]/angular_velocity/x",
            "/beetle1/nmpc/record_pred/states[0]/angular_velocity/y",
            "/beetle1/nmpc/record_pred/states[0]/angular_velocity/z",
        ]
    ].to_numpy()
    data["servo_angle_state"] = data_alpha_s_interp[
        [
            "/beetle1/nmpc/record_pred/states[0]/servo_angles[0]",
            "/beetle1/nmpc/record_pred/states[0]/servo_angles[1]",
            "/beetle1/nmpc/record_pred/states[0]/servo_angles[2]",
            "/beetle1/nmpc/record_pred/states[0]/servo_angles[3]",
        ]
    ].to_numpy()
    data["linear_acc_body"] = data_lin_acc_b_interp[
        [
            "/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/x",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/y",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_body_frame/z",
        ]
    ].to_numpy()
    data["linear_acc_world"] = data_lin_acc_w_interp[
        [
            "/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/x",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/y",
            "/beetle1/sensor_plugin/imu1/acc_only/acc_world_frame/z",
        ]
    ].to_numpy()

    data["thrust_cmd"] = data_thrust_cmd[
        [
            "/beetle1/nmpc/record_pred/controls[0]/thrust_commands[0]",
            "/beetle1/nmpc/record_pred/controls[0]/thrust_commands[1]",
            "/beetle1/nmpc/record_pred/controls[0]/thrust_commands[2]",
            "/beetle1/nmpc/record_pred/controls[0]/thrust_commands[3]",
        ]
    ].to_numpy()
    data["servo_angle_cmd"] = data_servo_angle_cmd_interp[
        [
            "/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[0]",
            "/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[1]",
            "/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[2]",
            "/beetle1/nmpc/record_pred/controls[0]/servo_angle_commands[3]",
        ]
    ].to_numpy()

    data["position_pred"] = data_xyz_pred_interp[
        [
            "/beetle1/nmpc/record_pred/states[1]/position/x",
            "/beetle1/nmpc/record_pred/states[1]/position/y",
            "/beetle1/nmpc/record_pred/states[1]/position/z",
        ]
    ].to_numpy()
    data["velocity_pred"] = data_vel_pred_interp[
        [
            "/beetle1/nmpc/record_pred/states[1]/linear_velocity/x",
            "/beetle1/nmpc/record_pred/states[1]/linear_velocity/y",
            "/beetle1/nmpc/record_pred/states[1]/linear_velocity/z",
        ]
    ].to_numpy()
    data["quaternion_pred"] = data_qwxyz_pred_interp[
        [
            "/beetle1/nmpc/record_pred/states[1]/orientation/w",
            "/beetle1/nmpc/record_pred/states[1]/orientation/x",
            "/beetle1/nmpc/record_pred/states[1]/orientation/y",
            "/beetle1/nmpc/record_pred/states[1]/orientation/z",
        ]
    ].to_numpy()
    data["angular_velocity_pred"] = data_ang_vel_pred_interp[
        [
            "/beetle1/nmpc/record_pred/states[1]/angular_velocity/x",
            "/beetle1/nmpc/record_pred/states[1]/angular_velocity/y",
            "/beetle1/nmpc/record_pred/states[1]/angular_velocity/z",
        ]
    ].to_numpy()
    data["servo_angle_state_pred"] = data_alpha_s_pred_interp[
        [
            "/beetle1/nmpc/record_pred/states[1]/servo_angles[0]",
            "/beetle1/nmpc/record_pred/states[1]/servo_angles[1]",
            "/beetle1/nmpc/record_pred/states[1]/servo_angles[2]",
            "/beetle1/nmpc/record_pred/states[1]/servo_angles[3]",
        ]
    ].to_numpy()

    data["position_ref"] = data_xyz_ref_interp[
        [
            "/beetle1/nmpc/record_ref/states[0]/position/x",
            "/beetle1/nmpc/record_ref/states[0]/position/y",
            "/beetle1/nmpc/record_ref/states[0]/position/z",
        ]
    ].to_numpy()
    data["velocity_ref"] = data_vel_ref_interp[
        [
            "/beetle1/nmpc/record_ref/states[0]/linear_velocity/x",
            "/beetle1/nmpc/record_ref/states[0]/linear_velocity/y",
            "/beetle1/nmpc/record_ref/states[0]/linear_velocity/z",
        ]
    ].to_numpy()
    data["quaternion_ref"] = data_qwxyz_ref_interp[
        [
            "/beetle1/nmpc/record_ref/states[0]/orientation/w",
            "/beetle1/nmpc/record_ref/states[0]/orientation/x",
            "/beetle1/nmpc/record_ref/states[0]/orientation/y",
            "/beetle1/nmpc/record_ref/states[0]/orientation/z",
        ]
    ].to_numpy()
    data["angular_velocity_ref"] = data_ang_vel_ref_interp[
        [
            "/beetle1/nmpc/record_ref/states[0]/angular_velocity/x",
            "/beetle1/nmpc/record_ref/states[0]/angular_velocity/y",
            "/beetle1/nmpc/record_ref/states[0]/angular_velocity/z",
        ]
    ].to_numpy()
    data["servo_angle_state_ref"] = data_alpha_s_ref_interp[
        [
            "/beetle1/nmpc/record_ref/states[0]/servo_angles[0]",
            "/beetle1/nmpc/record_ref/states[0]/servo_angles[1]",
            "/beetle1/nmpc/record_ref/states[0]/servo_angles[2]",
            "/beetle1/nmpc/record_ref/states[0]/servo_angles[3]",
        ]
    ].to_numpy()
    data["thrust_cmd_ref"] = data_thrust_cmd_ref_interp[
        [
            "/beetle1/nmpc/record_ref/controls[0]/thrust_commands[0]",
            "/beetle1/nmpc/record_ref/controls[0]/thrust_commands[1]",
            "/beetle1/nmpc/record_ref/controls[0]/thrust_commands[2]",
            "/beetle1/nmpc/record_ref/controls[0]/thrust_commands[3]",
        ]
    ].to_numpy()
    data["servo_angle_cmd_ref"] = data_servo_angle_cmd_ref_interp[
        [
            "/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[0]",
            "/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[1]",
            "/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[2]",
            "/beetle1/nmpc/record_ref/controls[0]/servo_angle_commands[3]",
        ]
    ].to_numpy()

    ############## Temporal filtering ##############
    # Filter for inconsistent timesteps in dt
    # This is very important for training neural networks since we learn to predict the residual dynamics from temporal 
    # dependencies in the measurements and outliers with large or small time steps influence the training significantly.
    # Enforce 0.009s < dt < 0.014s, for T_samp = 0.01s
    # NOTE: dt is the distance between timestamp[i] and timestamp[i+1] stored at index i
    if apply_temporal_filter:
        mean_dt = np.mean(data["dt"])
        std_dt = np.std(data["dt"])
        print(
            f"[INFO] Applying temporal filter to data with mean dt = {mean_dt:.2f}s and std dt = {std_dt:.2f}s. ",
            f"Removing all dt under {mean_dt - std_dt * 3:.2f}s and over {mean_dt + std_dt * 3:.2f}s.",
        )

        # 1. Filter out too large timesteps by shortening them to mean_dt
        # NOTE: This only helps if a time step is delayed and the next time step comes right after it.
        # For now only do this once, if the time gaps are persistent they would be propogated until the end.

        # TODO check if makes sense! (also only for too large dt and not both)
        # invalid_idx = np.append(np.where(data["dt"] > mean_dt + std_dt*3)[0], np.where(data["dt"] < mean_dt - std_dt*3)[0])
        # data["timestamp"][invalid_idx + 1] = (data["timestamp"][invalid_idx + 2] + data["timestamp"][invalid_idx]) / 2

        # # Recompute dt
        # data["dt"] = np.diff(data["timestamp"], axis=0).squeeze()
        # print(f"[INFO] Corrected {len(invalid_idx)} too long timesteps from data.")

        # Debug plot:
            # state_idx = 4
            # plt.figure()
            # plt.plot(timestamps_comp, state_in[:,state_idx], marker="x")
            # plt.plot(timestamps[..., np.where(dt>cutoff_dt)], state_in[np.where(dt>cutoff_dt),state_idx], marker="x", color="red")
    

        # 2. Filter out too small timesteps by removing entries
        while True:
            valid_indices = np.where((data["dt"] >= (mean_dt - std_dt*4)))[0]

            if len(data['dt']) - len(valid_indices) == 0:
                break

            for key in data.keys():
                if key not in ["dt", "duration"]:
                    data[key] = data[key][valid_indices + 1, :]
                    data[key] = data[key][:-1, :]  # Truncate last entry to match size of dt since we recompute it later

            # Recompute dt
            print(f"[INFO] Filtered out {len(data['dt']) - len(valid_indices)} invalid timesteps from data.")
            data["dt"] = np.diff(data["timestamp"], axis=0).squeeze()

    # Truncate last entry to match size of dt (since dt is by definition one entry shorter) NOTE: We do this after filtering to avoid indexing out of bounds
    for key in data.keys():
        if key not in ["dt", "duration"]:
            data[key] = data[key][:-1, :]

    return data


def combine_dicts(data_dicts: dict, T_samp: float):
    """
    Combine multiple csv files into one by appending the data.
    Assumes that all files have the same columns.
    """
    combined_data = dict()

    # Number of recordings
    combined_data["num_recordings"] = len(data_dicts)

    # List durations
    combined_data["duration"] = [data["duration"] for data in data_dicts.values()]

    # Start time of each recording
    combined_data["recording_start_idx"] = []

    # Append timestamps
    first = True
    for data in data_dicts.values():
        if first:
            # Mark liftoff
            combined_data["recording_start_idx"].append(0)
            combined_data["timestamp"] = data["timestamp"]
            combined_data["dt"] = data["dt"]
            last_time = data["timestamp"][-1]
            first = False
        else:
            combined_data["recording_start_idx"].append(combined_data["timestamp"].shape[0])
            combined_data["timestamp"] = np.append(
                combined_data["timestamp"],
                # Connect new timestamps to last timestamp + T_samp
                last_time + T_samp + (data["timestamp"] - data["timestamp"][0]),
            )
            last_time = combined_data["timestamp"][-1]
            combined_data["dt"][-1] = T_samp  # Overwrite last dt
            combined_data["dt"] = np.concatenate((combined_data["dt"], data["dt"]))

    # Append other fields
    for key in data_dicts[0].keys():
        if key in ["timestamp", "duration", "dt"]:
            continue
        combined_data[key] = np.concatenate([data[key] for data in data_dicts.values()], axis=0)

    return combined_data


if __name__ == "__main__":
    """
    Create dataset from rosbag
    """
    ############## Configuration ##############
    # Name of the dataset to be created
    ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_ONLY_JOY"
    # ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_VAL"
    ds_dir = os.path.join(DirectoryConfig.DATA_DIR, ds_name)

    apply_temporal_filter = True

    # Select which recordings to process
    rosbag_dir = "~/ros/rosbag_files/csv"
    csv_files = [
        # "2024-09-30-15-58-10_hover.csv",
        # "2024-10-01-15-19-19_new_motor_coeff_hover.csv",
        # "2024-10-02-21-56-59_new_new_motor_coeff_hover_setp_0.3_0.6.csv",
        # "2024-10-09-21-14-53_hand_fly_traj_pitch_roll.csv",
        # "2024-10-11-16-57-32_hand_fly_success.csv",
        # "2024-10-11-17-34-15_success_omni_pitch_rolling.csv",
        # "2024-10-12-10-45-20_success_omni_roll_rolling.csv",
        # "2024-10-15-18-32-17_-pitch_rolling_success.csv",
        # "2024-11-01-21-30-17_momentum_est_only.csv",
        # "2024-11-01-21-40-17_momentum_est_ctrl.csv",
        # "2024-11-07-16-24-27_ITerm_w_hand_dist.csv",
        # "2024-11-07-16-29-49_mom_w_hand_dist.csv",
        # "2024-11-07-17-08-20_mom_acc_est_only.csv",
        # "2024-11-07-17-16-15_mom_acc_est_ctrl_w_hand_dist.csv",
        # "2024-11-07-17-33-18_acceleration_est_only.csv",
        # "2024-11-12-20-13-50_acc_est_only_w_hand_dist.csv",
        # "2024-11-12-20-24-47_acc_ctrl_w_hand_dist.csv",
        # "2024-11-15-16-02-26_acc_ctrl_pitch=1.0_w_hand_dist.csv",
        # "2024-11-15-17-09-35_acc_ctrl_pitch=1.0_w_hand_dist_correct_branch.csv",
        # "2025-03-21-21-40-15_Roll90degYawRotate_mode_0.csv",
        # "2025-05-02-16-02-46_roll_pitch_mode_0.csv",
        # "2025-09-03-14-31-58_mode_0.csv",
        # "2025-09-03-14-38-30_jojo_ws_hovering_success_mode_0.csv",
        # "2025-09-07-17-54-58_jinjie_ws_hovering_success_mode_0.csv",
        # "2025-09-08-23-06-18_nominal_hovering_mode_0_success.csv",
        # "2025-09-08-23-12-12_hovering_mode_3_success.csv",
        # "2025-09-08-23-20-40_hovering_mode_10_success.csv",
        # "2025-09-10-16-44-59_long_flight_ground_effect_targets_mode_10_solver_error_for_aggressive_target_success.csv",
        ### TRAINING ###
        # "2025-09-10-17-09-30_long_flight_ground_effect_targets_mode_10_success_TRAIN_WITH_REF.csv",
        # "2025-09-10-18-52-13_multiple_smach_trajs_focus_on_rotation_mode_10_TRAIN_WITH_REF_FULL.csv",
        "2026-02-11-14-56-18_DATASET_RECORDING_SUCCESS_mode_10_all_record_hover_and_joystick_mov_incl_rotation_long_FULL.csv",
        # "2026-02-11-15-36-47_DATASET_RECORDING_SUCCESS_mode_10_all_record_mutilple_trajs_incl_rotation_with_mirrored_versions_FULL.csv",
        ### VALIDATION ###
        # "2025-09-10-16-44-59_long_flight_ground_effect_targets_mode_10_solver_error_for_aggressive_target_success_VAL_WITH_REF.csv",
        # "2026-02-11-13-48-15_DATASET_RECORDING_SUCCESS_mode_10_all_record_hover_and_joystick_mov_incl_rotation_FULL.csv",
        ### HOVERING & GROUND EFFECT TRAIN ###
        # NOT THIS SINCE TOO AGGRESSIVE: "2025-09-10-16-44-59_long_flight_ground_effect_targets_mode_10_solver_error_for_aggressive_target_success_GROUND_EFFECT_ONLY.csv",
        # "2025-09-10-18-52-13_multiple_smach_trajs_focus_on_rotation_mode_10_GROUND_EFFECT_ONLY.csv",
        # "2025-09-10-17-09-30_long_flight_ground_effect_targets_mode_10_success_GROUND_EFFECT_ONLY.csv",  # TRUNCATED FOR LESS AGGRESSIVE
        ### FULL DATASET ###
        # "2025-09-10-17-09-30_long_flight_ground_effect_targets_mode_10_success_FULL.csv",
        # "2025-09-10-18-52-13_multiple_smach_trajs_focus_on_rotation_mode_10_FULL.csv",
        # "2025-09-08-23-12-12_hovering_mode_3_success_FULL.csv",
        # "2025-09-08-23-20-40_hovering_mode_10_success_FULL.csv",
        ### Debug Recording
        # "2026-01-23-11-37-54_mode_11_circle_crash_instability_from_quick_movement.csv",
    ]
    csv_files = [os.path.join(rosbag_dir, file) for file in csv_files]

    data_dicts = dict()
    print(f"Started loading {len(csv_files)} csvs:")
    for i, csv_file in enumerate(csv_files):
        print(f"Loading {csv_file}...")
        data_dicts[i] = get_synched_data_from_rosbag(csv_file, apply_temporal_filter)

    print(f"Finished loading all csvs!")
    if len(csv_files) > 1:
        # TODO not meaningful for temporal neural networks!!
        print(f"Combining dicts..."),
        data = combine_dicts(data_dicts, T_samp)
        print(f"{len(csv_files)} dictionaries successfully combined!")
    else:
        data = data_dicts[0]
        data["recording_start_idx"] = [0]

    ############## Prepare data ##############
    # Time step
    timestamp = data["timestamp"].squeeze()
    dt = data["dt"]

    # State in
    state = np.hstack(
        (
            data["position"],
            data["velocity"],
            data["quaternion"],
            data["angular_velocity"],
            data["servo_angle_state"],
        )
    )

    # State predicted by the nominal model inside the MPC
    state_pred = np.hstack(
        (
            data["position_pred"],
            data["velocity_pred"],
            data["quaternion_pred"],
            data["angular_velocity_pred"],
            data["servo_angle_state_pred"],
        )
    )

    # Control input
    control = np.hstack(
        (
            data["thrust_cmd"],
            data["servo_angle_cmd"],
        )
    )

    # Acceleration measurements
    acc_body = data["linear_acc_body"]
    acc_world = data["linear_acc_world"]

    # Reference state
    state_ref = np.hstack(
        (
            data["position_ref"],
            data["velocity_ref"],
            data["quaternion_ref"],
            data["angular_velocity_ref"],
            data["servo_angle_state_ref"],
        )
    )

    # Reference control input
    control_ref = np.hstack(
        (
            data["thrust_cmd_ref"],
            data["servo_angle_cmd_ref"],
        )
    )

    ############## Metadata ##############
    # TODO move file naming and creation into data_utils and generalize
    # TODO make smarter!
    outer_fields = {
        "date": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
        "real_machine": True,
        "rosbag_file": csv_files,
        "duration": data["duration"],
        "temporal_filtering": apply_temporal_filter,
        "mpc_type": "NMPCTiltQdServo",
        "T_samp": T_samp,
    }
    inner_fields = {
        "disturbances": {
            "cog_dist": False,  # Disturbance forces and torques on CoG
            "cog_dist_model": "",
            "cog_dist_factor": 0.0,
            "motor_noise": False,  # Asymmetric noise in the rotor thrust and servo angles
            "drag": False,  # 2nd order polynomial aerodynamic drag effect
            "payload": False,  # Payload force in the Z axis
        },
    }

    if os.path.exists(ds_dir):
        ds_instances = []
        for _, _, file_names in os.walk(ds_dir):
            ds_instances.extend([os.path.splitext(file)[0] for file in file_names if not file.startswith(".")])

        # Increment counter for dataset file name
        if ds_instances:
            existing_instances = [int(instance.split("_")[1]) for instance in ds_instances]
            max_instance_number = max(existing_instances)
            ds_instance = "dataset_" + str(max_instance_number + 1).zfill(3)
        else:
            ds_instance = "dataset_001"
    else:
        safe_mkdir_recursive(ds_dir)
        ds_instance = "dataset_001"

    is_blank = safe_mkfile_recursive(ds_dir, ds_instance + ".csv")
    if not is_blank:
        raise FileExistsError(
            "Recording file already exists. Please change the dataset instance name or set overwrite to True."
        )

    # Update metadata json file
    json_file_name = os.path.join(DirectoryConfig.DATA_DIR, "metadata.json")
    if os.path.exists(json_file_name):
        with open(json_file_name, "r") as json_file:
            metadata = json.load(json_file)
        if ds_name not in metadata:
            metadata[ds_name] = outer_fields
        metadata[ds_name][ds_instance] = inner_fields

        # Write updated metadata to file
        with open(json_file_name, "w") as json_file:
            json.dump(metadata, json_file, indent=4)
    else:
        # Metadata file does not exist yet
        with open(json_file_name, "w") as json_file:
            ds_instance_name = "dataset_001"
            metadata = {ds_name: {**outer_fields, ds_instance_name: inner_fields}}
            json.dump(metadata, json_file, indent=4)

    ############## Save dataset ##############
    dataset_dict = {
        "timestamp": timestamp,
        "dt": dt,
        "state": state,
        "state_pred": state_pred,
        "control": control,
        "acc_body": acc_body,
        "acc_world": acc_world,
        "state_ref": state_ref,
        "control_ref": control_ref,        
    }

    # Generate new CSV to store data in
    rec_json = dict()
    for key in dataset_dict.keys():
        rec_json[key] = jsonify(dataset_dict[key])

    df = pd.DataFrame(rec_json)
    df.to_csv(os.path.join(ds_dir, ds_instance + ".csv"), index=False, header=True)
    print(f"Saved {ds_instance} in {ds_dir}.")
