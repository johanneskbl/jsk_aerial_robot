"""
 Created by li-jinjie on 2026/3/1.
"""

import time
import yaml
import os
import rospkg
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
import cvxpy as cp
import matplotlib.pyplot as plt
import scienceplots

from analyze_allocation import get_alloc_mtx_tilt_qd, get_alloc_mtx_tilt_tri, get_cmd_w_inv_mat
from analyze_allocation import mass, gravity

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze hovering efficiency under different attitudes.")
    parser.add_argument(
        "--arch",
        "-a",
        choices=["qd", "tri"],
        required=True,
        help="Choose the architecture to analyze: 'qd' for tiltable quadrotor, 'tri' for tiltable trirotor.",
    )
    parser.add_argument(
        "--resolution",
        "-r",
        type=float,
        default=15,
        help="Angular resolution in degrees for the attitude grid. Covering 0 to 180 degrees in roll and yaw.",
    )
    parser.add_argument(
        "--show_thrust_or_ratio",
        "-s",
        choices=["thrust", "ratio"],
        default="thrust",
        help="Choose whether to show absolute thrust values ('thrust') or thrust-to-weight ratio ('ratio') in the plot.",
    )

    args = parser.parse_args()

    qd_alloc_mat = get_alloc_mtx_tilt_qd()
    qd_alloc_pinv = np.linalg.pinv(qd_alloc_mat)
    tri_alloc_mat = get_alloc_mtx_tilt_tri()  # shutdown rotor 3 (1,2,4 active)
    tri_alloc_pinv = np.linalg.pinv(tri_alloc_mat)

    if args.arch == "qd":
        alloc_pinv = qd_alloc_pinv
    else:
        alloc_pinv = tri_alloc_pinv

    fg_w = np.array([0, 0, mass * gravity])

    # Create grids of yaw ∈ [0°, +180°], roll ∈ [0°, 180°]
    YAW_LOW = -180
    YAW_HIGH = 180
    ROLL_LOW = 0
    ROLL_HIGH = 180
    yaw_grid = np.arange(YAW_LOW, YAW_HIGH + 1, args.resolution)
    roll_grid = np.arange(ROLL_LOW, ROLL_HIGH + 1, args.resolution)

    result_hover_thrust_map = np.zeros((len(roll_grid), len(yaw_grid)))

    for i, roll in enumerate(roll_grid):
        for j, yaw in enumerate(yaw_grid):
            # Compute the rotation matrix for the given roll and yaw (assuming zero pitch)
            r = R.from_euler("zyx", [yaw, 0, roll], degrees=True)
            rot_mat = r.as_matrix()

            # Rotate the gravity force into the body frame
            fg_b = rot_mat.T @ fg_w

            tgt_wrench = np.array([[fg_b.item(0), fg_b.item(1), fg_b.item(2), 0, 0, 0]]).T

            # Compute reference values for thrust
            ft_ref, a_ref = get_cmd_w_inv_mat(alloc_pinv, tgt_wrench)

            # Compute the total thrust
            total_thrust = sum(ft_ref)

            # Store the total thrust in the result map
            result_hover_thrust_map[i, j] = total_thrust

    result_hover_ratio = result_hover_thrust_map / (mass * gravity)

    # Plotting thrust using color map
    plt.figure(figsize=(8, 6))
    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 11})
    label_size = 14

    if args.show_thrust_or_ratio == "ratio":
        plt.imshow(result_hover_ratio, extent=(YAW_LOW, YAW_HIGH, ROLL_LOW, ROLL_HIGH), origin="lower", cmap="viridis")
        cbar = plt.colorbar()
        cbar.set_label("Thrust-to-Weight Ratio", fontsize=label_size)
        # plt.title('Thrust-to-Weight Ratio for Hovering at Different Attitudes')
    else:
        plt.imshow(
            result_hover_thrust_map, extent=(YAW_LOW, YAW_HIGH, ROLL_LOW, ROLL_HIGH), origin="lower", cmap="viridis"
        )
        cbar = plt.colorbar()
        cbar.set_label("Total Thrust [N]", fontsize=label_size)
        # plt.title('Total Thrust Required for Hovering at Different Attitudes')

    plt.xlabel("Yaw $\\psi$ [$^{\circ}$]", fontsize=label_size)
    plt.ylabel("Roll $\\alpha$ [$^{\circ}$]", fontsize=label_size)
    plt.grid(False)
    plt.show()
