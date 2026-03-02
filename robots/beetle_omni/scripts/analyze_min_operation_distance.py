import numpy as np
import matplotlib.pyplot as plt

l = 1.0
R = 0.5 * l

a = l / np.sqrt(2)
P = np.array(
    [
        [a, a, 0.0],
        [a, -a, 0.0],
        [-a, a, 0.0],
        [-a, -a, 0.0],
    ]
)

# 0..180 deg
thetas_deg = np.linspace(0.0, 180.0, 721)
thetas = np.deg2rad(thetas_deg)


def dist_point_to_segment(point, A, B):
    AB = B - A
    AP = point - A
    denom = np.dot(AB, AB)
    if denom < 1e-12:
        return np.linalg.norm(point - A)
    t = np.dot(AP, AB) / denom
    t = np.clip(t, 0.0, 1.0)
    closest = A + t * AB
    return np.linalg.norm(point - closest)


h_min = np.zeros_like(thetas)
r_max = np.zeros_like(thetas)

for k, th in enumerate(thetas):
    n = np.array([np.cos(th), 0.0, np.sin(th)])  # unit

    # minimum safe plane offset along +n
    h = np.max(P @ n) + R
    h_min[k] = h

    O = np.zeros(3)
    E = h * n  # can point "backward" when theta>90 deg

    dmins = []
    for p in P:
        d_seg = dist_point_to_segment(p, O, E)
        dmins.append(d_seg - R)

    r_max[k] = max(0.0, min(dmins))

print(f"r_max(0°)  = {r_max[0]:.6f} (expect {l/np.sqrt(2)-R:.6f})")
print(f"r_max(180°)= {r_max[-1]:.6f} (should match 0° by symmetry if z=0 motors)")

fig, ax1 = plt.subplots()
ax1.plot(thetas_deg, h_min)
ax1.set_xlabel("Tilt angle θ (deg), normal from +X to +Z to -X")
ax1.set_ylabel("Minimum operating distance h_min")
ax1.grid(True)

ax2 = ax1.twinx()
ax2.plot(thetas_deg, r_max, color="orange")
ax2.set_ylabel("Max support cylinder radius r_max")

plt.title("X-config: h_min and allowable support radius vs tilt angle (0–180°, R/l=0.5)")
plt.show()
