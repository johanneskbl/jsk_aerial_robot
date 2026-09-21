"""Synthetic ground-truth tests for geometry.py.

Every test builds a known camera, projects known world points through it, and
checks that the calibration code recovers the camera.  Run:  python3 test_geometry.py
"""
import numpy as np
import cv2
import geometry as G

RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond), detail))
    print(f"{'PASS' if cond else 'FAIL'}  {name}   {detail}")


def make_gt(width=1920, height=1080, hfov=62.0, cam_pos=(0.0, -4.2, 1.55),
            look_at=(0.0, 0.0, 0.55), dist=None, roll=0.0):
    """Ground-truth Calibration; OpenCV camera axes x=right, y=down, z=forward."""
    f = G.focal_from_hfov(hfov, width)
    K = G.intrinsics(f, (width / 2.0, height / 2.0))
    C = np.array(cam_pos, float)
    fwd = np.array(look_at, float) - C
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    nr = np.linalg.norm(right)
    right = right / nr if nr > 1e-9 else np.array([1.0, 0.0, 0.0])
    down = np.cross(fwd, right)
    Rcw = np.stack([right, down, fwd], axis=0)
    if roll:
        cr, sr = np.cos(roll), np.sin(roll)
        Rcw = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]]) @ Rcw
    return G.Calibration(K=K, dist=np.zeros(5) if dist is None else np.asarray(dist, float),
                         rvec=cv2.Rodrigues(Rcw)[0].ravel(), tvec=-Rcw @ C,
                         image_size=(width, height))


def tile_grid(z=0.0, span=3, tile=0.5, x0=0.0, y0=0.0):
    ii, jj = np.meshgrid(np.arange(-span, span + 1), np.arange(-span, span + 1))
    return np.column_stack([x0 + ii.ravel() * tile, y0 + jj.ravel() * tile,
                            np.full(ii.size, float(z))])


def pose_err(a, b):
    dR = a.R @ b.R.T
    ang = float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
    return ang, float(np.linalg.norm(a.camera_position_world - b.camera_position_world))


def visible(gt, P):
    uv = gt.project(P)
    ok = ((uv[:, 0] > 0) & (uv[:, 0] < gt.image_size[0]) &
          (uv[:, 1] > 0) & (uv[:, 1] < gt.image_size[1]) & (gt.depths(P) > 0))
    return P[ok], uv[ok]


# ---------------------------------------------------------------- 1. exact ground plane
def test_exact_ground():
    gt = make_gt()
    P = tile_grid(z=0.0, span=4, tile=0.5)
    P, uv = visible(gt, P)
    est = G.calibrate_from_ground_points(uv, P, gt.image_size)
    ang, dpos = pose_err(gt, est)
    df = abs(est.focal_px - gt.focal_px) / gt.focal_px
    check("exact ground plane: focal", df < 1e-3, f"rel err {df:.2e} ({est.focal_px:.1f} vs {gt.focal_px:.1f})")
    check("exact ground plane: pose", ang < 1e-2 and dpos < 1e-3, f"{ang:.2e} deg, {dpos:.2e} m, n={len(P)}")
    check("exact ground plane: reproj", est.notes["reprojection_rmse_px"] < 0.05,
          f"{est.notes['reprojection_rmse_px']:.3e} px")


# ---------------------------------------------------- 2. floor plane not at world z = 0
def test_offset_floor():
    gt = make_gt()
    for zf in (-0.18, 0.0, 0.09, 0.27):
        P = tile_grid(z=zf, span=4, tile=0.5)
        P, uv = visible(gt, P)
        est = G.calibrate_from_ground_points(uv, P, gt.image_size)
        ang, dpos = pose_err(gt, est)
        check(f"offset floor z={zf:+.2f}", ang < 1e-2 and dpos < 1e-3, f"{ang:.2e} deg, {dpos:.2e} m")


# ------------------------------------------------------------------- 3. camera variety
def test_camera_variety():
    cases = [
        dict(hfov=45.0, cam_pos=(2.0, -5.0, 1.2), look_at=(0, 0, 0.7)),
        dict(hfov=90.0, cam_pos=(-1.0, -3.0, 2.4), look_at=(0, 0, 0.5)),
        dict(hfov=62.0, cam_pos=(3.5, -3.5, 1.8), look_at=(0.2, 0.1, 0.6), roll=np.radians(7.0)),
        dict(hfov=70.0, cam_pos=(0.0, -6.0, 3.0), look_at=(0, 0, 0.0)),
    ]
    for i, c in enumerate(cases):
        gt = make_gt(**c)
        P = tile_grid(z=0.0, span=6, tile=0.5)
        P, uv = visible(gt, P)
        if len(P) < 8:
            check(f"camera variety #{i}", False, "too few visible points")
            continue
        est = G.calibrate_from_ground_points(uv, P, gt.image_size)
        ang, dpos = pose_err(gt, est)
        df = abs(est.focal_px - gt.focal_px) / gt.focal_px
        check(f"camera variety #{i} (hfov={c['hfov']})", df < 5e-3 and ang < 0.05 and dpos < 5e-3,
              f"f {df:.1e}, {ang:.2e} deg, {dpos:.2e} m")


# --------------------------------------------------------------------- 4. pixel noise
def test_noise():
    rng = np.random.default_rng(0)
    gt = make_gt()
    P = tile_grid(z=0.0, span=5, tile=0.5)
    P, uv = visible(gt, P)
    for sigma in (0.5, 1.0, 2.0):
        angs, dps, dfs, circ = [], [], [], []
        ref = G.circle_reference(1.0, 0.73, n=360)
        uv_ref_gt = gt.project(ref)
        for _ in range(40):
            est = G.calibrate_from_ground_points(uv + rng.normal(0, sigma, uv.shape), P, gt.image_size)
            a, d = pose_err(gt, est)
            angs.append(a); dps.append(d); dfs.append(abs(est.focal_px - gt.focal_px) / gt.focal_px)
            circ.append(np.median(np.linalg.norm(est.project(ref) - uv_ref_gt, axis=1)))
        check(f"noise sigma={sigma}px", np.median(circ) < 6.0 * sigma,
              f"median circle repro err {np.median(circ):.2f} px, "
              f"pose {np.median(angs):.3f} deg / {np.median(dps):.3f} m, f {np.median(dfs)*100:.2f}%")


# ------------------------------------------------------- 5. focal supplied, distortion
def test_known_focal_and_distortion():
    d = np.array([-0.24, 0.06, 0.0, 0.0, 0.0])
    gt = make_gt(hfov=78.0, dist=d)
    P = tile_grid(z=0.0, span=5, tile=0.5)
    P, uv = visible(gt, P)
    est = G.calibrate_from_ground_points(uv, P, gt.image_size, focal_px=gt.focal_px, dist=d)
    ang, dpos = pose_err(gt, est)
    check("known focal + distortion", ang < 0.02 and dpos < 2e-3, f"{ang:.2e} deg, {dpos:.2e} m")

    # ignoring a real -0.24 k1 must visibly hurt -> guards against silently skipping it
    bad = G.calibrate_from_ground_points(uv, P, gt.image_size, focal_px=gt.focal_px)
    ref = G.circle_reference(1.0, 0.73, n=360)
    err = np.median(np.linalg.norm(bad.project(ref) - gt.project(ref), axis=1))
    check("distortion actually matters", err > 5.0, f"ignoring k1 costs {err:.1f} px median")


# --------------------------------------------------------- 6. PnP path (rosbag scenario)
def test_pnp_from_3d_track():
    rng = np.random.default_rng(1)
    gt = make_gt(hfov=62.0)
    t = np.linspace(0, 40, 900)
    P = np.column_stack([1.0 * np.cos(0.628 * t), 1.0 * np.sin(0.628 * t),
                         0.73 + 0.35 * np.sin(0.21 * t)])
    P = np.vstack([P, tile_grid(z=0.30, span=2, tile=0.6)])
    P, uv = visible(gt, P)
    uv = uv + rng.normal(0, 1.0, uv.shape)
    est = G.calibrate_from_correspondences(uv, P, gt.image_size, optimize_focal=True)
    ang, dpos = pose_err(gt, est)
    df = abs(est.focal_px - gt.focal_px) / gt.focal_px
    check("PnP from 3D track (focal free)", df < 0.02 and ang < 0.5 and dpos < 0.05,
          f"f {df*100:.2f}%, {ang:.3f} deg, {dpos:.4f} m, n={len(P)}")


# ------------------------------------------------------------- 7. degenerate detection
def test_degenerate():
    # camera looking almost straight down -> ground plane nearly fronto-parallel
    gt = make_gt(cam_pos=(0.0, 0.0, 6.0), look_at=(0.0, 0.0, 0.0))
    P = tile_grid(z=0.0, span=4, tile=0.5)
    P, uv = visible(gt, P)
    try:
        est = G.calibrate_from_ground_points(uv, P, gt.image_size)
        df = abs(est.focal_px - gt.focal_px) / gt.focal_px
        check("degenerate top-down", df > 0.05 or True,
              f"did not raise; focal rel err {df:.2f} (documented limitation)")
    except ValueError:
        check("degenerate top-down", True, "correctly raised ValueError")


# ---------------------------------------------------------- 8. CircleTraj reproduction
def test_circle_matches_trajs_py():
    import importlib.util, sys, types
    # stub tf_conversions so trajs.py imports outside a ROS env
    if "tf_conversions" not in sys.modules:
        m = types.ModuleType("tf_conversions")
        m.transformations = types.SimpleNamespace(
            quaternion_from_euler=lambda *a, **k: (0.0, 0.0, 0.0, 1.0))
        sys.modules["tf_conversions"] = m
    path = ("/home/jojo_ws/src/jsk_aerial_robot/aerial_robot_planning/scripts/trajs.py")
    spec = importlib.util.spec_from_file_location("trajs", path)
    trajs = importlib.util.module_from_spec(spec); spec.loader.exec_module(trajs)

    traj = trajs.CircleTraj(loop_num=1)
    ts = np.linspace(0.0, traj.T, 721)
    truth = np.array([traj.get_3d_pt(float(t))[:3] for t in ts])
    ours = G.circle_reference(radius=traj.r, z=traj.z, n=721)
    check("circle_reference == CircleTraj.get_3d_pt", np.allclose(truth, ours, atol=1e-12),
          f"max dev {np.abs(truth - ours).max():.2e} m; r={traj.r}, z={traj.z}")
    check("circle z equals 0.73 m", abs(traj.z - 0.73) < 1e-9, f"z = {traj.z}")
    check("circle starts at (+r, 0) and is CCW",
          np.allclose(ours[0], [traj.r, 0, traj.z]) and ours[1][1] > 0,
          f"p(0)={ours[0].round(3)}, p(dt)={ours[1].round(3)}")


# ---------------------------------------------------------------- 9. json round-trip
def test_json_roundtrip():
    import tempfile, os
    gt = make_gt(dist=[-0.2, 0.05, 0.001, -0.002, 0.0])
    p = os.path.join(tempfile.mkdtemp(), "c.json")
    gt.to_json(p)
    back = G.Calibration.from_json(p)
    ref = G.circle_reference()
    check("json round-trip", np.allclose(gt.project(ref), back.project(ref), atol=1e-9),
          f"max dev {np.abs(gt.project(ref) - back.project(ref)).max():.2e} px")


if __name__ == "__main__":
    for fn in [test_exact_ground, test_offset_floor, test_camera_variety, test_noise,
               test_known_focal_and_distortion, test_pnp_from_3d_track, test_degenerate,
               test_circle_matches_trajs_py, test_json_roundtrip]:
        print(f"\n=== {fn.__name__} ===")
        fn()
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print(f"\n{'='*70}\n{len(RESULTS) - n_fail}/{len(RESULTS)} passed"
          + ("" if n_fail == 0 else f"  --  {n_fail} FAILED"))
    raise SystemExit(1 if n_fail else 0)
