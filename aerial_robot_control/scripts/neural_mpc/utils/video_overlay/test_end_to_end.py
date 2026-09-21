"""End-to-end validation on a synthetic lab scene.

Renders a video of a red ball flying the CircleTraj over a 0.5 m checkerboard
floor from a known camera, then runs the real pipeline over it:

  1. floor-point calibration (simulating clicks on tile corners)
  2. ball-tracking + mocap PnP calibration, including time-offset recovery
  3. rendering, checking that the drawn circle actually lands on the ball

Run: python3 test_end_to_end.py
"""
import os
import subprocess
import sys
import tempfile

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geometry as G
import render as R
import bag_calib
from test_geometry import make_gt, tile_grid

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond), detail))
    print(f"{'PASS' if cond else 'FAIL'}  {name}   {detail}")


# ------------------------------------------------------------------ synthetic renderer
def topdown_checkerboard(ppm=100, extent=6.0, tile=0.5):
    n = int(2 * extent * ppm)
    ii = (np.arange(n) / (ppm * tile)).astype(int)
    board = ((ii[None, :] + ii[:, None]) % 2).astype(np.uint8)
    img = np.where(board[..., None] == 0, np.array([58, 58, 60], np.uint8),
                   np.array([104, 104, 108], np.uint8)).astype(np.uint8)
    noise = np.random.default_rng(3).normal(0, 6, img.shape)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    # world (X, Y) -> topdown pixel:  px = (X + extent) * ppm,  py = (Y + extent) * ppm
    T = np.array([[ppm, 0.0, extent * ppm], [0.0, ppm, extent * ppm], [0.0, 0.0, 1.0]])
    return img, T


def render_scene(calib, ball_world, board, T_world_to_board, ball_r=0.055):
    """One frame: warped checkerboard floor + a red ball at `ball_world`."""
    K, R_, t = calib.K, calib.R, np.asarray(calib.tvec, float).reshape(3, 1)
    H_world_img = K @ np.column_stack([R_[:, 0], R_[:, 1], t.ravel()])
    H = H_world_img @ np.linalg.inv(T_world_to_board)
    w, h = calib.image_size
    img = cv2.warpPerspective(board, H, (w, h), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=(30, 32, 36))
    # a plain "wall" above the horizon so the frame is not black
    horizon = img.sum(axis=2) < 20
    img[horizon] = (150, 132, 110)

    P = np.asarray(ball_world, float).reshape(1, 3)
    d = calib.depths(P)[0]
    if d > 0.2:
        uv = calib.project(P)[0]
        rad = max(2, int(round(calib.focal_px * ball_r / d)))
        if np.isfinite(uv).all():
            cv2.circle(img, tuple(np.round(uv).astype(int)), rad, (38, 34, 208), -1, cv2.LINE_AA)
            cv2.circle(img, tuple(np.round(uv).astype(int)), rad, (30, 26, 150), 2, cv2.LINE_AA)
    return img


def flight_path(t):
    """Takeoff, 3 circle laps at z = 0.73, landing -- gives the time-align a signal."""
    T, r, z = 10.0, 1.0, 0.73
    out = np.zeros((len(t), 3))
    for i, ti in enumerate(t):
        if ti < 6.0:                                   # rise on the spot
            out[i] = (0.0, 0.0, 0.30 + (z - 0.30) * min(1.0, ti / 5.0))
        elif ti < 8.0:                                 # slide out to (r, 0)
            a = (ti - 6.0) / 2.0
            out[i] = (r * a, 0.0, z)
        elif ti < 8.0 + 3 * T:                         # laps
            th = 2 * np.pi * (ti - 8.0) / T
            out[i] = (r * np.cos(th), r * np.sin(th), z)
        else:                                          # return and land
            a = min(1.0, (ti - 8.0 - 3 * T) / 4.0)
            out[i] = (r * (1 - a), 0.0, z - (z - 0.30) * a)
    return out


# ---------------------------------------------------------------------------- the test
def main():
    tmp = tempfile.mkdtemp(prefix="refproj_")
    print(f"workdir: {tmp}\n")

    gt = make_gt(width=1280, height=720, hfov=64.0,
                 cam_pos=(0.35, -4.30, 1.62), look_at=(0.0, 0.0, 0.55), roll=np.radians(-2.0))
    board, T = topdown_checkerboard()

    fps, dur = 30.0, 45.0
    ts = np.arange(0.0, dur, 1.0 / fps)
    path = flight_path(ts)

    video = os.path.join(tmp, "flight.mp4")
    vw = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), fps, gt.image_size)
    for p in path:
        vw.write(render_scene(gt, p, board, T))
    vw.release()
    check("synthetic video written", os.path.getsize(video) > 10000,
          f"{len(ts)} frames, {os.path.getsize(video)/1e6:.1f} MB")

    # ---------------------------------------------------------- 1. CLI: extract a frame
    frame_png = os.path.join(tmp, "frame0.png")
    run = lambda *a: subprocess.run([sys.executable, os.path.join(HERE, "project_reference.py")]
                                    + list(a), capture_output=True, text=True)
    r = run("frame", "--video", video, "--out", frame_png, "--index", "0")
    check("CLI frame", os.path.exists(frame_png) and r.returncode == 0, r.stdout.strip())

    # ------------------------------------- 2. calibrate from simulated clicks on the floor
    grid = tile_grid(z=0.0, span=5, tile=0.5)
    uv_all = gt.project(grid)
    vis = ((uv_all[:, 0] > 40) & (uv_all[:, 0] < gt.image_size[0] - 40) &
           (uv_all[:, 1] > 40) & (uv_all[:, 1] < gt.image_size[1] - 40) & (gt.depths(grid) > 0))
    grid, uv_all = grid[vis], uv_all[vis]
    rng = np.random.default_rng(7)
    sel = rng.choice(len(grid), size=min(12, len(grid)), replace=False)
    clicks = uv_all[sel] + rng.normal(0, 0.7, (len(sel), 2))       # human click precision

    import json
    points_json = os.path.join(tmp, "points.json")
    with open(points_json, "w") as f:
        json.dump({"image_size": list(gt.image_size), "floor_z": 0.0,
                   "points": [{"uv": list(map(float, c)), "world": list(map(float, w))}
                              for c, w in zip(clicks, grid[sel])]}, f)

    calib_json = os.path.join(tmp, "calib.json")
    check_png = os.path.join(tmp, "check.png")
    r = run("calibrate", "--points", points_json, "--out", calib_json,
            "--frame", frame_png, "--check", check_png, "--floor-z", "0")
    if r.returncode != 0:
        print(r.stdout, r.stderr)
    est = G.Calibration.from_json(calib_json)
    ang = np.degrees(np.arccos(np.clip((np.trace(gt.R @ est.R.T) - 1) / 2, -1, 1)))
    dpos = np.linalg.norm(gt.camera_position_world - est.camera_position_world)
    ref = G.circle_reference(1.0, 0.73, 720)
    circ_err = np.median(np.linalg.norm(est.project(ref) - gt.project(ref), axis=1))
    check("floor-click calibration", circ_err < 4.0 and ang < 1.0,
          f"{len(sel)} clicks @0.7px noise -> circle off by {circ_err:.2f} px median, "
          f"pose {ang:.3f} deg / {dpos:.3f} m, f err {abs(est.focal_px-gt.focal_px)/gt.focal_px*100:.2f}%")
    check("check image written", os.path.exists(check_png))

    # --------------------------------------------- 3. does the circle land on the ball?
    lap = (ts > 9.0) & (ts < 9.0 + 30.0)
    ball_uv = gt.project(path[lap])
    drawn = est.project(ref)
    d = np.min(np.linalg.norm(ball_uv[:, None, :] - drawn[None, :, :], axis=2), axis=1)
    check("drawn circle passes through the ball", np.median(d) < 5.0,
          f"median ball-to-curve distance {np.median(d):.2f} px, max {d.max():.2f} px")

    # ------------------------------------------------ 4. bag route: tracking + alignment
    track, fps_r = bag_calib.track_red_ball(video, stride=1, progress=False)
    det_rate = len(track) / len(ts)
    err_track = np.linalg.norm(track[:, 2:4] - gt.project(path[track[:, 0].astype(int)]), axis=1)
    check("red-ball tracker", det_rate > 0.85 and np.median(err_track) < 2.0,
          f"{det_rate*100:.0f}% of frames detected, centroid err median "
          f"{np.median(err_track):.2f} px")

    OFFSET = 17.35                       # t_bag = t_video + OFFSET
    bag_t = np.arange(-OFFSET, dur - OFFSET + 20.0, 0.01)
    bag_track = np.column_stack([bag_t + OFFSET, flight_path(bag_t)])
    calib2, off = bag_calib.align_and_calibrate(track, bag_track, gt.image_size,
                                                optimize_focal=True, verbose=False)
    ang2 = np.degrees(np.arccos(np.clip((np.trace(gt.R @ calib2.R.T) - 1) / 2, -1, 1)))
    dpos2 = np.linalg.norm(gt.camera_position_world - calib2.camera_position_world)
    circ2 = np.median(np.linalg.norm(calib2.project(ref) - gt.project(ref), axis=1))
    check("time offset recovered", abs(off - OFFSET) < 0.05,
          f"recovered {off:+.3f} s vs true {OFFSET:+.3f} s")
    check("bag calibration", circ2 < 2.0 and ang2 < 0.3,
          f"circle off by {circ2:.2f} px median, pose {ang2:.3f} deg / {dpos2:.3f} m, "
          f"f err {abs(calib2.focal_px-gt.focal_px)/gt.focal_px*100:.2f}%")

    # ------------------------------- 4b. tracker vs. the robot's own red motor pods
    # The real failure mode: the four motor units are the same red as the ball and
    # are bigger, and a static red chair sits in the corner of the lab.
    dvid = os.path.join(tmp, "distract.mp4")
    vw = cv2.VideoWriter(dvid, cv2.VideoWriter_fourcc(*"mp4v"), fps, gt.image_size)
    body_off = np.array([0.0, 0.0, -0.216])            # ball centre -> airframe plane
    ROTORS = np.array([[0.1948, 0.1948, 0], [-0.1948, 0.1948, 0],
                       [-0.1948, -0.1948, 0], [0.1948, -0.1948, 0]])
    sub = ts[::3]
    for p_ball in flight_path(sub):
        f = render_scene(gt, p_ball, board, T)
        body = p_ball + body_off
        for rot in ROTORS:                             # elongated red motor pods
            P = (body + rot).reshape(1, 3)
            if gt.depths(P)[0] < 0.2:
                continue
            uvp = gt.project(P)[0]
            if not np.isfinite(uvp).all():
                continue
            ax = max(4, int(gt.focal_px * 0.075 / gt.depths(P)[0]))
            cv2.ellipse(f, tuple(np.round(uvp).astype(int)), (ax, max(2, ax // 3)),
                        20.0, 0, 360, (40, 36, 200), -1, cv2.LINE_AA)
        cv2.rectangle(f, (30, gt.image_size[1] - 220), (150, gt.image_size[1] - 40),
                      (44, 40, 190), -1)               # a red chair, always in frame
        vw.write(f)
    vw.release()
    dtrack, _ = bag_calib.track_red_ball(dvid, stride=1, progress=False)
    truth = gt.project(flight_path(sub))
    got = np.full(len(sub), np.nan)
    for row in dtrack:
        got[int(row[0])] = np.linalg.norm(row[2:4] - truth[int(row[0])])
    okf = np.isfinite(got)
    check("tracker rejects the red motor pods and the red chair",
          okf.mean() > 0.8 and np.median(got[okf]) < 4.0 and np.nanmax(got) < 40.0,
          f"{okf.mean()*100:.0f}% detected, err median {np.median(got[okf]):.2f} px, "
          f"max {np.nanmax(got):.1f} px")

    # ------------------------------------------------------------------- 5. CLI: render
    out_mp4 = os.path.join(tmp, "out.mp4")
    r = run("render", "--video", video, "--calib", calib_json, "--out", out_mp4,
            "--floor-z", "0", "--hud", "--axes")
    cap = cv2.VideoCapture(out_mp4)
    n_out = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.release()
    check("CLI render", r.returncode == 0 and n_out >= len(ts) - 2,
          f"{n_out} frames written" + ("" if r.returncode == 0 else " " + r.stderr[-300:]))

    prev = os.path.join(tmp, "preview.png")
    r = run("preview", "--frame", frame_png, "--calib", calib_json, "--out", prev,
            "--floor-z", "0", "--grid", "--axes", "--hud")
    check("CLI preview", r.returncode == 0 and os.path.exists(prev), r.stderr[-200:])

    # --------------------------------------------------- 6. behind-camera edge handling
    close = make_gt(width=1280, height=720, hfov=64.0, cam_pos=(0.0, 0.2, 0.73),
                    look_at=(0.0, 3.0, 0.73))         # camera INSIDE the circle
    segs = R.project_polyline(close, ref, closed=True)
    allpts = np.vstack(segs) if segs else np.zeros((0, 2))
    check("no wild segments when the camera is inside the circle",
          len(segs) >= 1 and np.abs(allpts).max() < 1e5,
          f"{len(segs)} segment(s), max |px| = {np.abs(allpts).max():.0f}")

    print(f"\nartifacts kept in {tmp}")
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print(f"{'='*70}\n{len(RESULTS)-n_fail}/{len(RESULTS)} passed"
          + ("" if n_fail == 0 else f"  --  {n_fail} FAILED"))
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
