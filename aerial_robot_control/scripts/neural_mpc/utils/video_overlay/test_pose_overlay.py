"""Tests for the commanded-pose wireframe overlay.  Run: python3 test_pose_overlay.py

Needs ROS on the path (`source /opt/ros/*/setup.bash`) for the synthetic-bag test;
everything else runs without it.
"""
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geometry as G
import render as R
from test_geometry import make_gt

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond), detail))
    print(f"{'PASS' if cond else 'FAIL'}  {name}   {detail}")


SIZE = (1920, 1080)
GT = make_gt(width=SIZE[0], height=SIZE[1], hfov=75.0,
             cam_pos=(-0.17, -1.9, 1.2), look_at=(0.0, 0.0, 0.9))


def rpy_to_quat(r, p, y):
    """Fixed-axis rpy -> (w, x, y, z), matching tf.transformations.quaternion_from_euler."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


# ------------------------------------------------------------------ 1. rotation maths
def test_rotation_conventions():
    # the two attitudes SetPointTraj actually commands, plus edge cases
    for rpy in [(0.0, 0.0, 0.0), (0.5, 0.0, 0.3), (0.5, 0.5, -0.3),
                (-0.9, 0.4, 2.9), (0.1, -1.2, -0.7)]:
        q = rpy_to_quat(*rpy)
        back = G.rot_to_rpy(G.quat_to_rot(q))
        check(f"rpy round-trip {tuple(round(v,2) for v in rpy)}",
              np.allclose(rpy, back, atol=1e-9),
              f"recovered {tuple(round(v,6) for v in back)}")
    Rm = G.quat_to_rot(rpy_to_quat(0.5, 0.5, -0.3))
    check("rotation matrix is orthonormal with det +1",
          np.allclose(Rm @ Rm.T, np.eye(3), atol=1e-12)
          and abs(np.linalg.det(Rm) - 1.0) < 1e-12,
          f"det {np.linalg.det(Rm):.12f}")
    q = rpy_to_quat(0.5, 0.5, -0.3)
    check("q and -q give the same rotation (double cover)",
          np.allclose(G.quat_to_rot(q), G.quat_to_rot(-q), atol=1e-14))
    check("unnormalised quaternion is normalised internally",
          np.allclose(G.quat_to_rot(q), G.quat_to_rot(3.7 * q), atol=1e-12))


# ------------------------------------------------------------------ 2. robot geometry
def test_robot_geometry():
    g = G.load_robot_geometry()
    rot = g["rotors"]
    adj = np.linalg.norm(rot[0] - rot[1])
    diag = np.linalg.norm(rot[0] - rot[2])
    check("rotor spacing matches PhysParamBeetleOmniJetson.yaml",
          abs(adj - 0.3897) < 5e-4 and abs(diag - 0.5511) < 5e-4,
          f"adjacent {adj:.4f} m, diagonal {diag:.4f} m, from {os.path.basename(g['source'])}")
    check("ball offset is the documented 0.264 m along body z",
          np.allclose(g["ball_offset"], [0, 0, 0.264], atol=1e-9),
          f"{g['ball_offset']}")
    # the fallback must agree with the yaml, or one of them has silently drifted
    fb = G.load_robot_geometry("/nonexistent/path.yaml")
    check("built-in fallback agrees with the yaml",
          np.allclose(fb["rotors"], g["rotors"], atol=1e-6)
          and np.allclose(fb["ball_offset"], g["ball_offset"], atol=1e-9),
          f"fallback source: {fb['source']}")


# --------------------------------------------------------------- 3. wireframe is rigid
def test_wireframe_is_a_rigid_transform():
    g = G.load_robot_geometry()
    pos, q = np.array([0.3, 0.2, 1.2]), rpy_to_quat(0.5, 0.0, 0.3)
    wf = G.drone_wireframe(pos, q, geom=g, camera_position=GT.camera_position_world)
    kinds = [len(p) for p, _ in wf]
    # 4 arms, 4 propeller circles, mast, ball
    check("wireframe is 4 arms + 4 props + mast + ball",
          len(wf) == 10 and kinds[:4] == [2] * 4 and kinds[4:8] == [160] * 4
          and kinds[8] == 2 and kinds[9] == 180, f"piece sizes {kinds}")
    check("no rotor square by default", not any(len(p) == 5 for p, _ in wf))
    check("square is available but opt-in",
          any(len(p) == 5 for p, _ in G.drone_wireframe(pos, q, geom=g, square=True)))

    R = G.quat_to_rot(q)
    hub = pos + R @ np.array([0.0, 0.0, g["rotors"][:, 2].mean()])
    rot_w = pos[None, :] + g["rotors"] @ R.T
    arms = [p for p, _ in wf[:4]]
    check("every arm is one line from the frame hub to its rotor",
          all(np.allclose(a[0], hub, atol=1e-12) for a in arms)
          and all(np.allclose(arms[i][1], rot_w[i], atol=1e-12) for i in range(4)),
          "4 single-segment arms")
    arm_len = np.array([np.linalg.norm(a[1] - a[0]) for a in arms])
    body_len = np.linalg.norm(g["rotors"] - [0, 0, g["rotors"][:, 2].mean()], axis=1)
    check("arm lengths survive the rotation unchanged",
          np.allclose(np.sort(arm_len), np.sort(body_len), atol=1e-12),
          f"{np.round(arm_len,4)} vs {np.round(body_len,4)}")

    props = [p for p, _ in wf[4:8]]
    for i, pr in enumerate(props):
        # drop the duplicated closing point before averaging: it is deliberate (the
        # closed-loop dash tiling keys off seg[0] == seg[-1]) but it biases a centroid
        # by r/n, which is 0.7 mm here and looks exactly like a real offset.
        c = pr[:-1].mean(axis=0)
        rad = np.linalg.norm(pr - c, axis=1)
        check(f"propeller {i+1} is a 9 in circle on its rotor",
              np.allclose(rad, g["prop_radius"], atol=1e-12)
              and np.linalg.norm(c - rot_w[i]) < 1e-9,
              f"r {rad.mean():.4f} m (9 in = {9*0.0254/2:.4f})")
    n = np.cross(props[0][1] - props[0][0], props[0][2] - props[0][0])
    n /= np.linalg.norm(n)
    check("propeller discs lie in the rotor plane",
          abs(abs(float(n @ R[:, 2])) - 1.0) < 1e-9,
          f"|n . body z| = {abs(float(n @ R[:,2])):.12f}")

    mast = wf[8][0]
    check("mast runs from the hub to the ball",
          np.allclose(mast[0], hub, atol=1e-12)
          and np.allclose(mast[1], pos + R @ g["ball_offset"], atol=1e-12),
          f"length {np.linalg.norm(mast[1]-mast[0]):.6f} m")
    check("mast points along the rotated body z axis",
          np.allclose((mast[1] - mast[0]) / np.linalg.norm(mast[1] - mast[0]),
                      R[:, 2], atol=1e-12))


def test_ball_silhouette_faces_the_camera():
    g = G.load_robot_geometry()
    pos, q = np.array([-0.3, 0.0, 1.0]), rpy_to_quat(0.5, 0.5, -0.3)
    cam = GT.camera_position_world
    wf = G.drone_wireframe(pos, q, geom=g, camera_position=cam)
    ball = wf[-1][0]
    centre = pos + G.quat_to_rot(q) @ g["ball_offset"]
    radii = np.linalg.norm(ball - centre, axis=1)
    check("ball circle has the measured 0.041 m radius",
          np.allclose(radii, g["ball_radius"], atol=1e-12),
          f"radius {radii.mean():.6f} +/- {radii.std():.2e} m")
    n = np.cross(ball[1] - ball[0], ball[2] - ball[0])
    n /= np.linalg.norm(n)
    view = centre - cam; view /= np.linalg.norm(view)
    check("ball circle lies in the plane facing the camera (a true silhouette)",
          abs(abs(float(n @ view)) - 1.0) < 1e-9,
          f"|circle normal . view| = {abs(float(n @ view)):.12f}")
    # projected, it should come out very nearly circular
    uv = GT.project(ball)
    c2 = uv.mean(axis=0); r2 = np.linalg.norm(uv - c2, axis=1)
    check("and projects to a near-circle", (r2.max() - r2.min()) / r2.mean() < 0.05,
          f"radius {r2.mean():.1f} px, spread {(r2.max()-r2.min())/r2.mean()*100:.2f}%")


# ------------------------------------------------------------------ 4. pose grouping
def _stairs():
    """Synthetic staircase: hover, poseA, poseB, hover -- as SetPointTraj produces."""
    rows, t = [], 0.0
    plan = [((0, 0, 0.7), (0, 0, 0), 3.0), ((0.3, 0.2, 1.2), (0.5, 0, 0.3), 8.0),
            ((-0.3, 0, 1.0), (0.5, 0.5, -0.3), 8.0), ((0, 0, 0.7), (0, 0, 0), 1.2)]
    for pos, rpy, dur in plan:
        q = rpy_to_quat(*rpy)
        for _ in range(int(dur * 50)):
            rows.append([t, *pos, *q]); t += 0.02
    return np.array(rows)


def test_segment_and_dedup():
    import bag_calib as B
    poses = _stairs()
    seg = B.segment_poses(poses)
    check("staircase splits into its four plateaus", len(seg) == 4,
          f"{len(seg)} plateaus, durations "
          f"{[round(s['t1']-s['t0'],2) for s in seg]}")

    # quaternion double cover must not look like a new pose
    flipped = poses.copy()
    half = len(flipped) // 2
    mask = (np.arange(len(flipped)) > half) & (np.arange(len(flipped)) < half + 40)
    flipped[mask, 4:8] *= -1.0
    check("q -> -q mid-plateau is not mistaken for a pose change",
          len(B.segment_poses(flipped)) == 4, f"{len(B.segment_poses(flipped))} plateaus")

    check("short plateaus are dropped by min_duration",
          len(B.segment_poses(poses, min_duration=2.0)) == 3,
          "the 1.2 s tail is filtered out")

    import importlib.util
    spec = importlib.util.spec_from_file_location("pr", os.path.join(HERE,
                                                                    "project_reference.py"))
    pr = importlib.util.module_from_spec(spec); spec.loader.exec_module(pr)
    distinct = pr._distinct_poses(seg)
    check("the hover pose visited twice collapses to one distinct pose",
          len(distinct) == 3 and distinct[0]["visits"] == 2,
          f"{len(distinct)} distinct, first visited {distinct[0]['visits']}x")
    check("hold is the longest single visit, not the sum",
          abs(distinct[0]["hold"] - 3.0) < 0.05,
          f"hold {distinct[0]['hold']:.2f} s (visits of 3.0 s and 1.2 s)")
    kept = [d for d in distinct if d["hold"] >= 4.0]
    check("min-hold 4 s keeps exactly the two commanded setpoints", len(kept) == 2,
          f"kept {[tuple(np.round(k['pos'],2)) for k in kept]}")


# ------------------------------------------------------------------ 5. rendering
def test_pose_png_properties():
    g = G.load_robot_geometry()
    wf = G.drone_wireframe([0.3, 0.2, 1.2], rpy_to_quat(0.5, 0, 0.3), geom=g,
                           camera_position=GT.camera_position_world)
    rgba = R.render_reference_rgba(SIZE, GT, polylines=wf, color="#2FBA00",
                                   thickness=8, dash=(24, 15))
    col = R.hex_to_bgr("#2FBA00")
    check("pose PNG is BGRA with a transparent background",
          rgba.shape == (SIZE[1], SIZE[0], 4) and (rgba[..., 3] == 0).mean() > 0.95,
          f"{(rgba[...,3]==0).mean()*100:.2f}% clear")
    check("pose PNG has no colour fringing",
          np.all(rgba[..., :3] == np.array(col, np.uint8)),
          f"{int((rgba[...,:3] != np.array(col,np.uint8)).any(axis=2).sum())} deviating pixels")
    solid = R.render_reference_rgba(SIZE, GT, polylines=wf, thickness=8, dash=None)
    check("the wireframe is actually dashed",
          (rgba[..., 3] > 0).sum() < 0.85 * (solid[..., 3] > 0).sum(),
          f"{(rgba[...,3]>0).sum()} vs {(solid[...,3]>0).sum()} covered px")

    # two different poses must not render identically
    wf2 = G.drone_wireframe([-0.3, 0.0, 1.0], rpy_to_quat(0.5, 0.5, -0.3), geom=g,
                            camera_position=GT.camera_position_world)
    rgba2 = R.render_reference_rgba(SIZE, GT, polylines=wf2, thickness=8, dash=(24, 15))
    check("distinct poses render distinctly",
          not np.array_equal(rgba[..., 3], rgba2[..., 3]),
          f"alpha overlap {(np.minimum(rgba[...,3],rgba2[...,3])>0).sum()} px")


# ------------------------------------------------------- 6. CLI on a synthetic rosbag
def test_cli_pose_overlay():
    try:
        import rosbag, rospy
        from trajectory_msgs.msg import (MultiDOFJointTrajectory,
                                         MultiDOFJointTrajectoryPoint)
        from geometry_msgs.msg import Transform, Vector3, Quaternion
    except Exception as e:
        check("CLI pose-overlay (needs ROS on the path)", True, f"SKIPPED: {e}")
        return

    tmp = tempfile.mkdtemp()
    bag_path = os.path.join(tmp, "synth.bag")
    poses = _stairs()
    t0 = 1000.0
    with rosbag.Bag(bag_path, "w") as bag:
        for row in poses:
            msg = MultiDOFJointTrajectory()
            pt = MultiDOFJointTrajectoryPoint()
            pt.transforms.append(Transform(
                translation=Vector3(*row[1:4]),
                rotation=Quaternion(x=row[5], y=row[6], z=row[7], w=row[4])))
            msg.points.append(pt)
            bag.write("/beetle1/set_ref_traj", msg, rospy.Time.from_sec(t0 + row[0]))

    cal = os.path.join(tmp, "c.json")
    GT.notes["time_offset_video_to_bag_s"] = 2.40
    GT.to_json(cal)
    out = os.path.join(tmp, "sp.png")
    r = subprocess.run([sys.executable, os.path.join(HERE, "project_reference.py"),
                        "pose-overlay", "--calib", cal, "--bag", bag_path,
                        "--video-window", "-2.4", "18.0", "--min-hold", "4.0",
                        "--out", out], capture_output=True, text=True)
    files = [os.path.join(tmp, f"sp_pose{i}.png") for i in (1, 2)]
    ok = r.returncode == 0 and all(os.path.exists(f) for f in files)
    check("CLI writes one PNG per held pose", ok,
          (r.stdout + r.stderr)[-300:] if not ok else "sp_pose1.png, sp_pose2.png")
    if ok:
        imgs = [cv2.imread(f, cv2.IMREAD_UNCHANGED) for f in files]
        check("both CLI PNGs are transparent BGRA",
              all(i is not None and i.shape[2] == 4 and (i[..., 3] == 0).mean() > 0.95
                  for i in imgs),
              ", ".join(f"{(i[...,3]==0).mean()*100:.1f}% clear" for i in imgs))
        check("the two CLI PNGs differ (different poses)",
              not np.array_equal(imgs[0][..., 3], imgs[1][..., 3]))
        check("--video-window resolved via the calibration's stored offset",
              "offset +2.40" in r.stdout, r.stdout.splitlines()[0] if r.stdout else "")

    r2 = subprocess.run([sys.executable, os.path.join(HERE, "project_reference.py"),
                         "pose-overlay", "--calib", cal, "--bag", bag_path,
                         "--video-window", "-2.4", "18.0", "--out", out,
                         "--min-hold", "4.0", "--pose-index", "2"],
                        capture_output=True, text=True)
    check("--pose-index selects a single pose and writes exactly --out",
          r2.returncode == 0 and os.path.exists(out),
          (r2.stdout + r2.stderr)[-200:] if r2.returncode else os.path.basename(out))


if __name__ == "__main__":
    for fn in [test_rotation_conventions, test_robot_geometry,
               test_wireframe_is_a_rigid_transform, test_ball_silhouette_faces_the_camera,
               test_segment_and_dedup, test_pose_png_properties, test_cli_pose_overlay]:
        print(f"\n=== {fn.__name__} ===")
        fn()
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print(f"\n{'='*70}\n{len(RESULTS)-n_fail}/{len(RESULTS)} passed"
          + ("" if n_fail == 0 else f"  --  {n_fail} FAILED"))
    raise SystemExit(1 if n_fail else 0)
