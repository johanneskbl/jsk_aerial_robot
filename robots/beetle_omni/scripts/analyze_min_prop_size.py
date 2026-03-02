"""
 Created by li-jinjie on 2026/3/2.
"""
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

# N is 3,4,5,6,..., a int list
N_START = 3
N_END = 12
N_STEP = 1

N = np.linspace(N_START, N_END, int((N_END - N_START) / N_STEP) + 1, dtype=int)

theta = 2 * np.pi / N

# Compute the minimum propeller size (distance from center to rotor) for each N
prop_to_frame_ratio_no_ae = (1 - np.cos(theta)) / 2
prop_to_frame_ratio_no_collision = np.cos((np.pi - theta) / 2)

# Set up the plot style
plt.style.use(["science", "grid"])
plt.rcParams.update({"font.size": 12})
label_size = 13

# plot prop_to_frame_ratio to N
plt.figure(figsize=(8, 3))

# Fill regions with colors
# Light red above prop_to_frame_ratio_no_collision
plt.fill_between(N, prop_to_frame_ratio_no_collision, 1.0, alpha=0.3, color="lightcoral", label="Collision Risk")

# Light blue between the two curves
plt.fill_between(
    N,
    prop_to_frame_ratio_no_ae,
    prop_to_frame_ratio_no_collision,
    alpha=0.3,
    color="lightblue",
    label="Aero-Interference Risk",
)

# Light green below prop_to_frame_ratio_no_ae
plt.fill_between(N, 0, prop_to_frame_ratio_no_ae, alpha=0.3, color="lightgreen", label="Safe Zone")

# Plot the curves
plt.plot(N, prop_to_frame_ratio_no_collision, marker="o", linewidth=2, label="Maximum No Collision")
plt.plot(N, prop_to_frame_ratio_no_ae, marker="o", linewidth=2, label="Maximum No Aero-Interference")

plt.legend()
# plt.title("Minimum Propeller Size to Frame Size Ratio vs Number of Rotors")
plt.xlabel("Number of Rotors $N$", fontsize=label_size)
plt.ylabel("Propeller Size / Frame Size $R/l$", fontsize=label_size)

plt.legend(fontsize=label_size - 2)

# # set legend transparency
# legend = plt.gca().get_legend()
# legend.get_frame().set_alpha(0.9)

plt.xticks(N)
plt.grid(alpha=0.5)
plt.xlim(N[0], N[-1])
plt.ylim(0, 1)
plt.show()
