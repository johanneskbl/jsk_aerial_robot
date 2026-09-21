"""Camera geometry for projecting mocap-frame references into a video recording.

The world frame `W` is the OptiTrack/mocap frame used by the flight stack
(`parent_frame_id: world` in aerial_robot_base/launch/external_module/mocap.launch):
z is up, the origin is wherever the Motive ground-plane calibration put it.

A calibration is the tuple (K, dist, rvec, tvec) such that

    p_pixel = project(K, dist, R(rvec) @ P_world + tvec)

which is exactly OpenCV's `cv2.projectPoints` convention.

Created for the energy-regularized neural-MPC paper video.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, asdict

import cv2
import numpy as np


# --------------------------------------------------------------------------------------
# Calibration container
# --------------------------------------------------------------------------------------
@dataclass
class Calibration:
    """Full pinhole + radial/tangential distortion model with world->camera pose."""

    K: np.ndarray                       # (3, 3) intrinsics
    dist: np.ndarray                    # (5,) OpenCV distortion coefficients k1 k2 p1 p2 k3
    rvec: np.ndarray                    # (3,) Rodrigues, world -> camera
    tvec: np.ndarray                    # (3,) world -> camera
    image_size: tuple = (0, 0)          # (width, height)
    notes: dict = field(default_factory=dict)

    # -- derived ------------------------------------------------------------------
    @property
    def R(self) -> np.ndarray:
        return cv2.Rodrigues(np.asarray(self.rvec, float).reshape(3, 1))[0]

    @property
    def camera_position_world(self) -> np.ndarray:
        """Camera centre expressed in the world (mocap) frame."""
        return (-self.R.T @ np.asarray(self.tvec, float).reshape(3, 1)).ravel()

    @property
    def focal_px(self) -> float:
        return float(0.5 * (self.K[0, 0] + self.K[1, 1]))

    def horizontal_fov_deg(self) -> float:
        w = self.image_size[0] or (2.0 * self.K[0, 2])
        return float(np.degrees(2.0 * np.arctan(0.5 * w / self.K[0, 0])))

    # -- projection ---------------------------------------------------------------
    def project(self, points_world: np.ndarray) -> np.ndarray:
        """Project (N, 3) world points to (N, 2) pixels."""
        pts = np.asarray(points_world, float).reshape(-1, 1, 3)
        uv, _ = cv2.projectPoints(pts, np.asarray(self.rvec, float).reshape(3, 1),
                                  np.asarray(self.tvec, float).reshape(3, 1),
                                  self.K, self.dist)
        return uv.reshape(-1, 2)

    def depths(self, points_world: np.ndarray) -> np.ndarray:
        """Camera-frame z (metres in front of the camera) for (N, 3) world points."""
        P = np.asarray(points_world, float).reshape(-1, 3)
        return (P @ self.R.T + np.asarray(self.tvec, float).reshape(1, 3))[:, 2]

    # -- io -----------------------------------------------------------------------
    def to_json(self, path: str) -> None:
        d = asdict(self)
        for k in ("K", "dist", "rvec", "tvec"):
            d[k] = np.asarray(d[k], float).ravel().tolist()
        d["K"] = np.asarray(self.K, float).reshape(3, 3).tolist()
        d["image_size"] = list(self.image_size)
        d["_derived"] = {
            "camera_position_world": self.camera_position_world.tolist(),
            "focal_px": self.focal_px,
            "horizontal_fov_deg": self.horizontal_fov_deg(),
        }
        with open(path, "w") as f:
            json.dump(d, f, indent=2)

    @staticmethod
    def from_json(path: str) -> "Calibration":
        with open(path) as f:
            d = json.load(f)
        return Calibration(
            K=np.array(d["K"], float).reshape(3, 3),
            dist=np.array(d["dist"], float).ravel(),
            rvec=np.array(d["rvec"], float).ravel(),
            tvec=np.array(d["tvec"], float).ravel(),
            image_size=tuple(d.get("image_size", (0, 0))),
            notes=d.get("notes", {}),
        )


# --------------------------------------------------------------------------------------
# Focal length from a single ground-plane homography
# --------------------------------------------------------------------------------------
def focal_from_homography(H: np.ndarray, pp: tuple, image_scale: float | None = None,
                          return_details: bool = False):
    """Estimate focal length (px) from one plane->image homography.

    A single homography places two constraints on the image of the absolute conic
    (Zhang 2000).  For a square-pixel, zero-skew camera with a *known* principal
    point the unknown conic is w = diag(u, u, v) once the principal point is
    translated to the origin, and f^2 = v / u:

        h1' w h2 = 0            ->  (a1a2 + b1b2) u + (c1c2) v          = 0
        h1' w h1 = h2' w h2     ->  (a1^2+b1^2-a2^2-b2^2) u + (c1^2-c2^2) v = 0

    Both are solved together as a *homogeneous* 2x2 system via SVD.  That matters:
    when the camera has no roll, c1 is exactly zero and the first constraint
    collapses to 0 = 0.  Solving it explicitly (dividing by c1*c2) or rescaling it
    to unit weight promotes round-off to a full-strength equation and produces a
    badly wrong focal length; in the homogeneous form a vanishing row simply stops
    constraining, and the well-posed row decides the answer.

    Pixel coordinates are pre-scaled to O(1) for conditioning.

    Raises ValueError if the recovered f^2 is not positive, which happens when the
    plane is close to fronto-parallel -- the focal length is then genuinely
    unobservable and must be supplied from elsewhere.
    """
    H = np.asarray(H, float).reshape(3, 3)
    s = float(image_scale) if image_scale else max(abs(pp[0]), abs(pp[1]), 1.0) * 2.0

    N = np.array([[1.0 / s, 0.0, -pp[0] / s],
                  [0.0, 1.0 / s, -pp[1] / s],
                  [0.0, 0.0, 1.0]])
    Hn = N @ H
    Hn = Hn / max(np.linalg.norm(Hn), 1e-300)
    (a1, b1, c1) = Hn[:, 0]
    (a2, b2, c2) = Hn[:, 1]

    # rows of [u, v]; do NOT rescale them -- a small row is an uninformative row.
    A = np.array([[a1 * a2 + b1 * b2, c1 * c2],
                  [a1 * a1 + b1 * b1 - a2 * a2 - b2 * b2, c1 * c1 - c2 * c2]])

    U_, sv, Vt = np.linalg.svd(A)
    if sv[0] < 1e-14:
        raise ValueError("Focal length is not observable: both conic constraints vanish. "
                         "Pass --focal-px / --hfov-deg explicitly.")
    u, v = Vt[-1]
    if u < 0:
        u, v = -u, -v
    if not (u > 1e-14 and v > 0.0):
        raise ValueError(
            "Focal length is not observable from this homography (the ground plane is "
            "close to fronto-parallel, or the correspondences are degenerate). "
            "Pass --focal-px / --hfov-deg explicitly, or calibrate from a rosbag."
        )
    f = s * np.sqrt(v / u)
    if not np.isfinite(f) or f < 0.02 * s or f > 200.0 * s:
        raise ValueError(f"Implausible focal length {f:.1f} px recovered from the homography; "
                         "pass --focal-px / --hfov-deg explicitly.")

    if not return_details:
        return float(f)

    details = {
        "focal_px": float(f),
        # how strongly each constraint participates (relative row magnitude)
        "row_weight_orthogonality": float(np.linalg.norm(A[0])),
        "row_weight_equal_norm": float(np.linalg.norm(A[1])),
        "singular_values": [float(x) for x in sv],
        # >1 means the two constraints disagree / the system is nearly rank-deficient
        "conditioning": float(sv[0] / sv[1]) if sv[1] > 1e-300 else float("inf"),
    }
    for i, name in enumerate(("orthogonality", "equal_norm")):
        if np.linalg.norm(A[i]) > 1e-9 * max(1.0, np.linalg.norm(A)) and abs(A[i, 0]) > 1e-14:
            ratio = -A[i, 1] / A[i, 0]
            details[name] = float(s * np.sqrt(ratio)) if ratio > 0 else None
        else:
            details[name] = None
    return float(f), details


def pose_from_plane_homography(H: np.ndarray, K: np.ndarray, plane_z: float = 0.0,
                               check_world_points: np.ndarray | None = None):
    """Decompose a (z = plane_z) plane->image homography into (rvec, tvec).

    `H` maps homogeneous world plane coordinates (X, Y, 1) -- the actual 3D point
    being (X, Y, plane_z) -- to homogeneous pixels.

    Because H encodes K [r1 r2 (plane_z*r3 + t)], the plane offset is subtracted
    from the recovered translation column to give the true world->camera tvec.

    The overall sign of the decomposition is fixed by requiring the observed
    points to lie in front of the camera (using `check_world_points` when given,
    otherwise the plane origin).
    """
    H = np.asarray(H, float).reshape(3, 3)
    Kinv = np.linalg.inv(np.asarray(K, float).reshape(3, 3))
    M0 = Kinv @ H
    lam = 2.0 / (np.linalg.norm(M0[:, 0]) + np.linalg.norm(M0[:, 1]))
    M0 = M0 * lam

    best = None
    for sign in (1.0, -1.0):
        M = sign * M0
        r1, r2 = M[:, 0], M[:, 1]
        R = np.column_stack([r1, r2, np.cross(r1, r2)])
        U, _, Vt = np.linalg.svd(R)
        R = U @ np.diag([1.0, 1.0, float(np.linalg.det(U @ Vt))]) @ Vt
        tvec = M[:, 2] - plane_z * R[:, 2]

        if check_world_points is not None and len(check_world_points):
            P = np.asarray(check_world_points, float).reshape(-1, 3)
            score = float(np.mean((P @ R.T + tvec.reshape(1, 3))[:, 2] > 0))
        else:
            score = 1.0 if (R @ np.array([0.0, 0.0, plane_z]) + tvec)[2] > 0 else 0.0
        if best is None or score > best[0]:
            best = (score, R, tvec)

    _, R, tvec = best
    return cv2.Rodrigues(R)[0].ravel(), tvec


def calibrate_from_ground_points(
    image_points: np.ndarray,
    world_points: np.ndarray,
    image_size: tuple,
    focal_px: float | None = None,
    principal_point: tuple | None = None,
    dist: np.ndarray | None = None,
    refine: bool = True,
    extra_image_points: np.ndarray | None = None,
    extra_world_points: np.ndarray | None = None,
) -> Calibration:
    """Calibrate from >=4 correspondences that lie on one horizontal plane.

    Parameters
    ----------
    image_points : (N, 2) pixel coordinates of points on a horizontal plane.
    world_points : (N, 3) their world coordinates.  All must share the same z.
    focal_px     : if None it is estimated from the homography.
    dist         : known distortion coefficients; the image points are undistorted
                   with them before the homography is fitted.
    extra_*      : optional off-plane correspondences (e.g. the ball centre of the
                   parked robot) folded into the final non-linear refinement.
    """
    image_points = np.asarray(image_points, float).reshape(-1, 2)
    world_points = np.asarray(world_points, float).reshape(-1, 3)
    if len(image_points) != len(world_points):
        raise ValueError("image_points and world_points must have the same length")
    if len(image_points) < 4:
        raise ValueError("need at least 4 ground correspondences for a homography")

    zs = world_points[:, 2]
    if float(np.ptp(zs)) > 1e-6:
        raise ValueError(f"ground correspondences must be coplanar in z, got spread {np.ptp(zs):.4f} m")
    plane_z = float(zs[0])

    dist = np.zeros(5) if dist is None else np.asarray(dist, float).ravel()
    if pp_default := (principal_point is None):
        principal_point = (image_size[0] / 2.0, image_size[1] / 2.0)
    _ = pp_default

    # Work on distortion-free image coordinates so the homography is exact.
    if np.any(dist != 0.0):
        if focal_px is None:
            raise ValueError("a focal length is required to undistort; pass --focal-px/--hfov-deg")
        K_tmp = intrinsics(focal_px, principal_point)
        und = cv2.undistortPoints(image_points.reshape(-1, 1, 2), K_tmp, dist, P=K_tmp)
        img_lin = und.reshape(-1, 2)
    else:
        img_lin = image_points

    H, mask = cv2.findHomography(world_points[:, :2], img_lin,
                                 cv2.RANSAC if len(img_lin) >= 6 else 0, 3.0)
    if H is None:
        raise ValueError("homography estimation failed -- check the correspondences")

    focal_details = None
    focal_was_estimated = focal_px is None
    if focal_was_estimated:
        focal_px, focal_details = focal_from_homography(
            H, principal_point, image_scale=max(image_size), return_details=True)

    K = intrinsics(focal_px, principal_point)
    rvec, tvec = pose_from_plane_homography(H, K, plane_z=plane_z,
                                            check_world_points=world_points)

    all_img = image_points
    all_world = world_points
    if extra_image_points is not None and len(extra_image_points):
        all_img = np.vstack([all_img, np.asarray(extra_image_points, float).reshape(-1, 2)])
        all_world = np.vstack([all_world, np.asarray(extra_world_points, float).reshape(-1, 3)])

    if refine:
        rvec, tvec = cv2.solvePnPRefineLM(
            all_world.reshape(-1, 1, 3), all_img.reshape(-1, 1, 2), K, dist,
            np.asarray(rvec, float).reshape(3, 1), np.asarray(tvec, float).reshape(3, 1))
        rvec, tvec = np.ravel(rvec), np.ravel(tvec)

    calib = Calibration(K=K, dist=dist, rvec=rvec, tvec=tvec, image_size=tuple(image_size))
    calib.notes["method"] = "ground-plane homography"
    calib.notes["n_ground_points"] = int(len(world_points))
    calib.notes["plane_z"] = plane_z
    calib.notes["focal_estimated"] = bool(focal_was_estimated)
    if focal_details is not None:
        calib.notes["focal_estimate_detail"] = focal_details
    calib.notes["reprojection_rmse_px"] = float(
        reprojection_rmse(calib, all_world, all_img))
    return calib


def intrinsics(focal_px: float, principal_point) -> np.ndarray:
    return np.array([[focal_px, 0.0, principal_point[0]],
                     [0.0, focal_px, principal_point[1]],
                     [0.0, 0.0, 1.0]], float)


def focal_from_hfov(hfov_deg: float, image_width: int) -> float:
    return float(0.5 * image_width / np.tan(0.5 * np.radians(hfov_deg)))


def reprojection_rmse(calib: Calibration, world_points, image_points) -> float:
    pred = calib.project(np.asarray(world_points, float).reshape(-1, 3))
    err = pred - np.asarray(image_points, float).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))


def calibrate_from_correspondences(
    image_points: np.ndarray,
    world_points: np.ndarray,
    image_size: tuple,
    focal_px: float | None = None,
    principal_point: tuple | None = None,
    dist: np.ndarray | None = None,
    optimize_focal: bool = False,
    optimize_k1: bool = False,
) -> Calibration:
    """Calibrate from general (non-coplanar) 3D<->2D correspondences via PnP.

    This is the path used when the pose is recovered from a rosbag: the tracked
    ball centre sweeps a large 3D volume, so f (and optionally k1) can be
    estimated jointly with the pose instead of being assumed.
    """
    from scipy.optimize import least_squares

    image_points = np.asarray(image_points, float).reshape(-1, 2)
    world_points = np.asarray(world_points, float).reshape(-1, 3)
    if principal_point is None:
        principal_point = (image_size[0] / 2.0, image_size[1] / 2.0)
    if focal_px is None:
        focal_px = 0.9 * image_size[0]          # ~58 deg HFOV, a neutral starting guess
    dist = np.zeros(5) if dist is None else np.asarray(dist, float).ravel().copy()

    K = intrinsics(focal_px, principal_point)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        world_points.reshape(-1, 1, 3), image_points.reshape(-1, 1, 2), K, dist,
        flags=cv2.SOLVEPNP_EPNP, reprojectionError=8.0, iterationsCount=2000)
    if not ok:
        raise ValueError("solvePnPRansac failed -- check the 2D/3D correspondences")
    keep = np.arange(len(world_points)) if inliers is None else inliers.ravel()

    Wk, Ik = world_points[keep], image_points[keep]

    def unpack(p):
        rv, tv = p[0:3], p[3:6]
        i = 6
        f = p[i] if optimize_focal else focal_px
        i += int(optimize_focal)
        d = dist.copy()
        if optimize_k1:
            d[0] = p[i]
        return rv, tv, f, d

    def residual(p):
        rv, tv, f, d = unpack(p)
        uv, _ = cv2.projectPoints(Wk.reshape(-1, 1, 3), rv.reshape(3, 1), tv.reshape(3, 1),
                                  intrinsics(f, principal_point), d)
        return (uv.reshape(-1, 2) - Ik).ravel()

    p0 = np.concatenate([np.ravel(rvec), np.ravel(tvec)])
    if optimize_focal:
        p0 = np.append(p0, focal_px)
    if optimize_k1:
        p0 = np.append(p0, dist[0])

    sol = least_squares(residual, p0, method="lm", xtol=1e-12, ftol=1e-12)
    rv, tv, f, d = unpack(sol.x)

    calib = Calibration(K=intrinsics(f, principal_point), dist=d, rvec=rv, tvec=tv,
                        image_size=tuple(image_size))
    calib.notes["method"] = "PnP on 3D correspondences"
    calib.notes["n_points"] = int(len(world_points))
    calib.notes["n_inliers"] = int(len(keep))
    calib.notes["reprojection_rmse_px"] = float(reprojection_rmse(calib, Wk, Ik))
    return calib


# --------------------------------------------------------------------------------------
# The reference trajectory itself
# --------------------------------------------------------------------------------------
def circle_reference(radius: float = 1.0, z: float = 0.73, n: int = 720,
                     t0_at_angle: float = 0.0) -> np.ndarray:
    """Sample aerial_robot_planning/scripts/trajs.py::CircleTraj as (n, 3) world points.

    CircleTraj.get_3d_pt(t) = (r cos(w t), r sin(w t), z), i.e. a circle of radius
    `r` about the world z axis, traversed counter-clockwise seen from above,
    starting at (+r, 0).  With the defaults in trajs.py:
        r = 1.0 m,  z = 0.2 + 0.27 + 0.26 = 0.73 m
    and z is the commanded height of the *ball end-effector* (`ee_contact`),
    because pub_mpc_base.py tracks /<robot>/uav/ee_contact/odom when it exists.
    """
    th = t0_at_angle + np.linspace(0.0, 2.0 * np.pi, int(n), endpoint=True)
    return np.column_stack([radius * np.cos(th), radius * np.sin(th), np.full_like(th, z)])


# --------------------------------------------------------------------------------------
# Full bundle refinement
# --------------------------------------------------------------------------------------
FREE_ALL = ("rvec", "tvec", "f", "pp", "k1", "k2", "tangential")


def refine_full(image_points, world_points, image_size, calib: Calibration | None = None,
                focal_px: float | None = None, principal_point=None, dist=None,
                free=("rvec", "tvec", "f", "k1"), loss: str = "huber",
                f_scale: float = 6.0, max_nfev: int = 400, verbose: bool = False):
    """Bundle-refine intrinsics and pose against 3D<->2D correspondences.

    `free` selects which parameter groups are optimised; everything else is held.
    A robust loss is used by default because real tracks always retain a few
    mis-detections, and a plain least squares lets them drag the focal length and
    the distortion coefficients around by tens of percent.

    Returns a Calibration.  `notes` carries the robust and plain error statistics
    so the two can be compared.
    """
    from scipy.optimize import least_squares

    U = np.asarray(image_points, float).reshape(-1, 2)
    W = np.asarray(world_points, float).reshape(-1, 3)
    if len(U) != len(W) or len(U) < 6:
        raise ValueError(f"need >=6 matched points, got {len(U)} / {len(W)}")

    if calib is not None:
        K0, d0 = np.array(calib.K, float), np.asarray(calib.dist, float).ravel().copy()
        rv0, tv0 = np.asarray(calib.rvec, float).ravel(), np.asarray(calib.tvec, float).ravel()
    else:
        if principal_point is None:
            principal_point = (image_size[0] / 2.0, image_size[1] / 2.0)
        f0 = focal_px if focal_px else 0.9 * image_size[0]
        K0 = intrinsics(f0, principal_point)
        d0 = np.zeros(5) if dist is None else np.asarray(dist, float).ravel().copy()
        ok, rv0, tv0, _ = cv2.solvePnPRansac(W.reshape(-1, 1, 3), U.reshape(-1, 1, 2), K0, d0,
                                             flags=cv2.SOLVEPNP_EPNP, reprojectionError=12.0,
                                             iterationsCount=3000)
        if not ok:
            raise ValueError("solvePnPRansac failed to initialise the refinement")
        rv0, tv0 = np.ravel(rv0), np.ravel(tv0)

    free = set(free)
    base = dict(rvec=rv0.copy(), tvec=tv0.copy(), f=float(0.5 * (K0[0, 0] + K0[1, 1])),
                pp=np.array([K0[0, 2], K0[1, 2]]), k1=float(d0[0]), k2=float(d0[1]),
                tangential=np.array([d0[2], d0[3]]))
    order = [k for k in FREE_ALL if k in free]
    sizes = dict(rvec=3, tvec=3, f=1, pp=2, k1=1, k2=1, tangential=2)

    def pack():
        return np.concatenate([np.atleast_1d(base[k]).astype(float).ravel() for k in order])

    def unpack(p):
        vals, i = dict(base), 0
        for k in order:
            n = sizes[k]
            vals[k] = p[i] if n == 1 else p[i:i + n]
            i += n
        K = intrinsics(vals["f"], vals["pp"])
        d = np.array([vals["k1"], vals["k2"], vals["tangential"][0], vals["tangential"][1], 0.0])
        return np.atleast_1d(vals["rvec"]).ravel(), np.atleast_1d(vals["tvec"]).ravel(), K, d

    def residual(p):
        rv, tv, K, d = unpack(p)
        uv, _ = cv2.projectPoints(W.reshape(-1, 1, 3), rv.reshape(3, 1), tv.reshape(3, 1), K, d)
        return (uv.reshape(-1, 2) - U).ravel()

    sol = least_squares(residual, pack(), loss=loss, f_scale=f_scale, max_nfev=max_nfev,
                        xtol=1e-12, ftol=1e-12, verbose=2 if verbose else 0)
    rv, tv, K, d = unpack(sol.x)
    calib_out = Calibration(K=K, dist=d, rvec=rv, tvec=tv, image_size=tuple(image_size))
    err = np.linalg.norm(calib_out.project(W) - U, axis=1)
    calib_out.notes.update({
        "method": "robust bundle refinement",
        "free": sorted(free),
        "loss": loss,
        "f_scale_px": f_scale,
        "n_points": int(len(U)),
        "reprojection_rmse_px": float(np.sqrt(np.mean(err ** 2))),
        "reprojection_median_px": float(np.median(err)),
        "reprojection_p90_px": float(np.percentile(err, 90)),
        "inlier_frac_below_f_scale": float((err < f_scale).mean()),
    })
    return calib_out, err


# --------------------------------------------------------------------------------------
# Robot model -- for drawing a commanded *pose* rather than a path
# --------------------------------------------------------------------------------------
# Fallbacks, in the CoG frame, from robots/beetle_omni/config/PhysParamBeetleOmniJetson.yaml.
# These are validated against the real footage: projected through the recovered camera they
# land on the motor pods of the parked robot in PXL_20260329_094920628.mp4.
_ROTORS_FALLBACK = np.array([[+0.194824, +0.194652, -0.00368224],
                             [-0.194837, +0.194652, -0.00368224],
                             [-0.194837, -0.195008, -0.00368224],
                             [+0.194824, -0.195008, -0.00368224]])
_BALL_OFFSET_FALLBACK = np.array([0.0, 0.0, 0.264])
# Not in any config file: recovered from the tracker + mocap depth during calibration.
BALL_RADIUS_M = 0.041
# 9 inch propellers, as specified by the user.
PROP_DIAMETER_M = 9.0 * 0.0254          # 0.2286 m -> radius 0.1143 m

PHYS_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "../../../../robots/beetle_omni/config/PhysParamBeetleOmniJetson.yaml")


def load_robot_geometry(yaml_path: str | None = None) -> dict:
    """Rotor positions and the ball offset, in the CoG frame, in metres.

    Read from PhysParamBeetleOmniJetson.yaml when it is reachable so the numbers are
    never duplicated; falls back to the values recorded above otherwise. `p1..p4` are
    CoG-frame because that is the frame the MPC's dynamics (and this yaml) work in.
    """
    path = yaml_path or PHYS_YAML
    try:
        import yaml
        with open(path) as f:
            ph = yaml.safe_load(f)["physical"]
        return {"rotors": np.array([ph["p1"], ph["p2"], ph["p3"], ph["p4"]], float),
                "ball_offset": np.array(ph["ball_effector_p"], float),
                "ball_radius": BALL_RADIUS_M,
                "prop_radius": 0.5 * PROP_DIAMETER_M,
                "source": os.path.abspath(path)}
    except Exception:
        return {"rotors": _ROTORS_FALLBACK.copy(),
                "ball_offset": _BALL_OFFSET_FALLBACK.copy(),
                "ball_radius": BALL_RADIUS_M,
                "prop_radius": 0.5 * PROP_DIAMETER_M,
                "source": "built-in fallback"}


def quat_to_rot(q_wxyz) -> np.ndarray:
    """Body->world rotation matrix from a (w, x, y, z) quaternion."""
    w, x, y, z = np.asarray(q_wxyz, float).ravel()
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("zero-norm quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_to_rpy(R: np.ndarray):
    """Fixed-axis roll/pitch/yaw [rad] from a rotation matrix (tf's sxyz convention)."""
    R = np.asarray(R, float).reshape(3, 3)
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    if abs(R[2, 0]) < 0.99999:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:                                   # gimbal lock
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return float(roll), float(pitch), float(yaw)


def _circle_3d(centre, u, v, radius, n=180):
    th = np.linspace(0.0, 2.0 * np.pi, int(n), endpoint=True)
    return (np.asarray(centre, float).reshape(1, 3)
            + radius * (np.cos(th)[:, None] * np.asarray(u, float).reshape(1, 3)
                        + np.sin(th)[:, None] * np.asarray(v, float).reshape(1, 3)))


def drone_wireframe(pos, quat_wxyz, geom: dict | None = None, camera_position=None,
                    arms: bool = True, props: bool = True, mast: bool = True,
                    ball: bool = True, square: bool = False,
                    prop_radius: float | None = None,
                    n_circle: int = 180, n_prop: int = 160):
    """The robot drawn at one commanded pose, as a list of (points_world, closed).

    `pos`/`quat_wxyz` are the **CoG** pose, because that is what this stack's
    `set_ref_traj` commands and what the controller tracked on this recording.

    What gets drawn, and where each number comes from:

    * **arms** -- a single line from the frame centre out to each rotor, with the
      endpoints `p1..p4` from PhysParamBeetleOmniJetson.yaml.
    * **propellers** -- a circle of `prop_radius` at each rotor, in the rotor plane.
      9 inch diameter. Drawn untilted: the reference commands a body pose, not
      servo angles.
    * **mast and ball** -- `ball_effector_p` from the same yaml, and the ball's
      0.041 m radius measured during calibration. The ball is drawn as its true
      silhouette (a circle in the plane facing the camera) when `camera_position`
      is given, and in the body XY plane otherwise.

    Everything is built in body coordinates and then rigidly transformed, so the
    figure cannot be distorted by the pose. `square` (the rotor-plane outline) is
    off by default but still available.
    """
    geom = geom or load_robot_geometry()
    p = np.asarray(pos, float).reshape(3)
    R = quat_to_rot(quat_wxyz)
    rotors = np.asarray(geom["rotors"], float).reshape(4, 3)
    rp = float(geom.get("prop_radius", 0.5 * PROP_DIAMETER_M)
               if prop_radius is None else prop_radius)

    hub = np.array([0.0, 0.0, float(rotors[:, 2].mean())])   # frame centre, rotor plane
    body = []                                                # (points_body, closed)

    if arms:
        for i in range(4):
            body.append((np.vstack([hub, rotors[i]]), False))
    if props:
        for i in range(4):
            body.append((_circle_3d(rotors[i], np.array([1.0, 0.0, 0.0]),
                                    np.array([0.0, 1.0, 0.0]), rp, n_prop), True))
    if square:
        body.append((np.vstack([rotors, rotors[0]]), True))

    ball_b = np.asarray(geom["ball_offset"], float).reshape(3)
    if mast:
        body.append((np.vstack([hub, ball_b]), False))

    out = [(p[None, :] + pts @ R.T, closed) for pts, closed in body]

    if ball:
        ball_w = p + R @ ball_b
        r = float(geom["ball_radius"])
        if camera_position is not None:
            d = ball_w - np.asarray(camera_position, float).reshape(3)
            nd = np.linalg.norm(d)
            if nd < 1e-9:
                u, v = R[:, 0], R[:, 1]
            else:
                d = d / nd
                helper = np.array([0.0, 0.0, 1.0])
                if abs(float(d @ helper)) > 0.95:
                    helper = np.array([1.0, 0.0, 0.0])
                u = np.cross(d, helper); u /= np.linalg.norm(u)
                v = np.cross(d, u)
        else:
            u, v = R[:, 0], R[:, 1]
        out.append((_circle_3d(ball_w, u, v, r, n_circle), True))
    return out
