"""Automatic camera calibration by matching the tracked ball against mocap ground truth.

This is the accurate route.  The red ball on top of the robot is the `ee_contact`
frame, i.e. exactly the point the MPC tracks and exactly the point the reference
in trajs.py commands.  Segmenting it in the video and pairing those pixels with
/<robot>/uav/ee_contact/odom from the flight rosbag gives hundreds of 3D<->2D
correspondences spread over a large volume, which pins down the camera pose (and
focal length) far better than any single-frame construction -- and, crucially,
needs no assumption about where the mocap origin sits, which way its axes point,
or how high the floor is.
"""

from __future__ import annotations

import numpy as np
import cv2

from geometry import Calibration, calibrate_from_correspondences


# --------------------------------------------------------------------------------------
# Ball tracking
# --------------------------------------------------------------------------------------
DEFAULT_RED = dict(s_min=110, v_min=60, h_lo=8, h_hi=172, min_area=40, max_area=200000)

ROTATIONS = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def apply_rotation(frame, rotate: int):
    """Rotate a decoded frame. Phones store an orientation flag that OpenCV ignores,
    so the frames can come out sideways relative to what a player shows."""
    code = ROTATIONS.get(int(rotate) % 360, "bad")
    if code == "bad":
        raise ValueError(f"--rotate must be one of {sorted(ROTATIONS)}, got {rotate}")
    return frame if code is None else cv2.rotate(frame, code)


def _red_mask(bgr, p):
    hsv_img = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv_img, (0, p["s_min"], p["v_min"]), (p["h_lo"], 255, 255))
    m2 = cv2.inRange(hsv_img, (p["h_hi"], p["s_min"], p["v_min"]), (179, 255, 255))
    mask = cv2.morphologyEx(cv2.bitwise_or(m1, m2), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))


def track_red_ball(video_path: str, hsv=None, stride: int = 1, roi=None,
                   max_frames: int | None = None, progress=True, rotate: int = 0,
                   start_frame: int = 0, end_frame: int | None = None,
                   min_radius: float = 3.0, max_radius: float = 400.0,
                   min_circularity: float = 0.60, debug_video: str | None = None):
    """Track the saturated red ball through the video.

    Returns ((N, 5) array of frame_index, t_video_seconds, u, v, area), fps.

    Red wraps around hue 0, so both ends of the hue circle are taken.  Two things
    make a naive "largest red blob" tracker fail on this footage: the robot's own
    motor pods are the same red as the ball, and they merge into one large blob
    when the airframe is seen edge-on.  So candidates are scored on circularity
    (a sphere silhouette is a disc; a motor arm is not) and, once the track is
    running, on agreement with the predicted position and the running median
    radius.  A candidate that is far from the prediction *and* the wrong size is
    rejected rather than accepted as the new best.
    """
    p = dict(DEFAULT_RED, **(hsv or {}))
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"cannot open video: {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)

    writer = None
    out, idx = [], 0
    prev, vel, radii = None, np.zeros(2), []
    if start_frame:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(start_frame))
        idx = int(start_frame)

    while True:
        if end_frame is not None and idx >= end_frame:
            break
        ok, frame = cap.read()
        if not ok:
            break
        if idx % stride == 0:
            frame = apply_rotation(frame, rotate)
            sub, ox, oy = (frame, 0, 0)
            if roi is not None:
                x, y, w, h = roi
                sub, ox, oy = frame[y:y + h, x:x + w], x, y
            mask = _red_mask(sub, p)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            r_med = float(np.median(radii)) if len(radii) >= 5 else None
            pred = (prev + vel) if prev is not None else None

            best, best_score = None, 0.0
            for c in cnts:
                a = cv2.contourArea(c)
                if not (p["min_area"] <= a <= p["max_area"]):
                    continue
                (cx, cy), r = cv2.minEnclosingCircle(c)
                if not (min_radius <= r <= max_radius):
                    continue
                circ = a / max(np.pi * r * r, 1e-9)          # 1.0 for a perfect disc
                if circ < min_circularity:
                    continue
                score = circ ** 2
                if r_med is not None:                        # size agreement
                    score *= np.exp(-0.5 * ((r - r_med) / (0.45 * r_med)) ** 2)
                else:
                    score *= a ** 0.25
                if pred is not None:                         # motion agreement
                    d = np.hypot(cx + ox - pred[0], cy + oy - pred[1])
                    gate = 6.0 * max(r_med or r, 8.0)
                    score *= np.exp(-0.5 * (d / gate) ** 2)
                if score > best_score:
                    best, best_score = (cx + ox, cy + oy, a, r), score

            if best is not None:
                cur = np.array(best[:2])
                vel = 0.5 * vel + 0.5 * (cur - prev) if prev is not None else np.zeros(2)
                prev = cur
                radii.append(best[3])
                if len(radii) > 200:
                    radii.pop(0)
                out.append((idx, idx / fps, best[0], best[1], best[2]))
            else:
                prev, vel = None, np.zeros(2)                # lost: restart the track

            if debug_video:
                vis = frame.copy()
                vis[..., 2] = np.maximum(vis[..., 2],
                                         cv2.copyMakeBorder(mask, oy, vis.shape[0] - oy -
                                                            mask.shape[0], ox,
                                                            vis.shape[1] - ox - mask.shape[1],
                                                            cv2.BORDER_CONSTANT, value=0) // 2)
                if best is not None:
                    cv2.circle(vis, (int(best[0]), int(best[1])), int(best[3]) + 6,
                               (0, 255, 0), 2, cv2.LINE_AA)
                if writer is None:
                    writer = cv2.VideoWriter(debug_video, cv2.VideoWriter_fourcc(*"mp4v"),
                                             fps / max(1, stride),
                                             (vis.shape[1], vis.shape[0]))
                writer.write(vis)
        idx += 1
        if progress and n_total and idx % 200 == 0:
            print(f"\r  tracking {idx}/{n_total} frames, {len(out)} detections", end="")
        if max_frames and idx >= max_frames:
            break
    cap.release()
    if writer is not None:
        writer.release()
        print(f"\n  wrote tracker debug video to {debug_video}")
    if progress:
        print(f"\r  tracked {idx} frames, {len(out)} detections" + " " * 24)
    return np.array(out, float).reshape(-1, 5), fps


# --------------------------------------------------------------------------------------
# Mocap ground truth
# --------------------------------------------------------------------------------------
def read_ee_track(bag_path: str, topic: str | None = None, robot: str = "beetle1"):
    """Read the end-effector (ball) world trajectory from a flight rosbag.

    Returns (t_seconds_from_bag_start, x, y, z) as an (N, 4) array.  The epoch is
    the bag start, not the topic's first message, so these times can be compared
    directly against wall-clock reasoning about when the video started.
    """
    import rosbag

    if topic is None:
        topic = f"/{robot}/uav/ee_contact/odom"
    rows = []
    with rosbag.Bag(bag_path) as bag:
        t0 = bag.get_start_time()
        available = set(bag.get_type_and_topic_info().topics.keys())
        if topic not in available:
            cands = sorted(t for t in available if t.endswith("ee_contact/odom"))
            if not cands:
                raise KeyError(f"{topic} not in bag. Odometry topics present: "
                               f"{sorted(t for t in available if 'odom' in t)}")
            topic = cands[0]
        for _, msg, t in bag.read_messages(topics=[topic]):
            ts = msg.header.stamp.to_sec() or t.to_sec()
            p = msg.pose.pose.position
            rows.append((ts - t0, p.x, p.y, p.z))
    return np.array(rows, float), topic


def _interp_track(track: np.ndarray, times: np.ndarray):
    """Linearly interpolate an (N, 4) (t, x, y, z) track at `times`; NaN outside range."""
    t, P = track[:, 0], track[:, 1:4]
    out = np.column_stack([np.interp(times, t, P[:, i]) for i in range(3)])
    out[(times < t[0]) | (times > t[-1])] = np.nan
    return out


# --------------------------------------------------------------------------------------
# Time alignment + calibration
# --------------------------------------------------------------------------------------
def _spread(P: np.ndarray) -> float:
    """Second singular value of the centred point cloud [m].

    Near zero means the paired 3D points lie on a line, which any PnP can fit
    perfectly and meaninglessly -- the classic way a time-offset search locks onto
    a tiny bogus overlap instead of the true one.
    """
    if len(P) < 3:
        return 0.0
    sv = np.linalg.svd(P - P.mean(axis=0), compute_uv=False)
    return float(sv[1] / np.sqrt(len(P)))


def off_orbit_weights(P: np.ndarray, floor: float = 0.05) -> np.ndarray:
    """Weight each 3D sample by how much it departs from the dominant circular orbit.

    This is what makes the time alignment work at all on a CircleTraj flight.  A
    circle at constant height is invariant under rotation about the world z axis,
    so shifting the video against the bag by *any* amount inside the looping phase
    still admits a perfectly consistent camera -- one whose world frame is simply
    yawed.  Plain reprojection error therefore has an almost flat, and in practice
    actively misleading, minimum: on the synthetic scene the true offset scores
    2.8 px while wrong offsets score 1.0 px.

    Only the samples that break the symmetry -- takeoff, landing, the transit out
    to the circle -- actually carry offset information, so they are what the
    alignment score is weighted towards.  For a trajectory with no such symmetry
    (e.g. the lemniscate) the weights are broadly uniform and this is a no-op.
    """
    P = np.asarray(P, float).reshape(-1, 3)
    cx, cy = np.median(P[:, 0]), np.median(P[:, 1])
    r = np.hypot(P[:, 0] - cx, P[:, 1] - cy)
    d = np.hypot(np.abs(r - np.median(r)), np.abs(P[:, 2] - np.median(P[:, 2])))
    scale = max(float(np.percentile(d, 90)), 1e-6)
    return floor + np.clip(d / scale, 0.0, 1.0)


def align_and_calibrate(pixel_track: np.ndarray, bag_track: np.ndarray, image_size,
                        focal_px: float | None = None, optimize_focal: bool = True,
                        optimize_k1: bool = False, offset_range=None,
                        coarse_step: float = 0.2, min_overlap_frac: float = 0.5,
                        min_spread_m: float = 0.15, min_pairs: int = 60,
                        fixed_offset: float | None = None, search_subsample: int = 1,
                        verbose: bool = True):
    """Find the video<->bag time offset and calibrate from the resulting pairs.

    Convention: ``t_bag = t_video + offset``.

    Candidate offsets are scored by an *off-orbit weighted* reprojection RMSE (see
    `off_orbit_weights`), and only when they pair enough of the recording
    (`min_overlap_frac` of the shorter take) and when the paired 3D points span a
    surface rather than a line (`min_spread_m`).  Without the gates the search
    locks onto a one-second sliver in which the robot descends on the spot: those
    points are collinear, PnP fits them with no residual, and that bogus offset
    wins on error alone.

    Pass `fixed_offset` to skip the search entirely when the offset is already
    known (e.g. from the video and bag wall-clock timestamps, or from a visible
    cue such as the takeoff frame).
    """
    t_img, uv = pixel_track[:, 1], pixel_track[:, 2:4]
    # The scan runs a PnP per candidate offset, so it is subsampled; the final
    # calibration below always uses every detection.
    k = max(1, int(search_subsample))
    ts_img, s_uv = t_img[::k], uv[::k]
    t_bag = bag_track[:, 0]
    vid_dur = float(t_img[-1] - t_img[0])
    bag_dur = float(t_bag[-1] - t_bag[0])
    shorter = max(1e-6, min(vid_dur, bag_dur))

    if offset_range is None:
        offset_range = (float(t_bag[0] - t_img[-1]) - 1.0, float(t_bag[-1] - t_img[0]) + 1.0)

    def overlap_frac(offset):
        lo = max(t_img[0] + offset, t_bag[0])
        hi = min(t_img[-1] + offset, t_bag[-1])
        return max(0.0, hi - lo) / shorter

    def score(offset):
        """Off-orbit weighted reprojection RMSE, or inf if the pairing is untrustworthy."""
        if overlap_frac(offset) < min_overlap_frac:
            return np.inf, None
        P = _interp_track(bag_track, ts_img + offset)
        m = np.isfinite(P).all(axis=1)
        if m.sum() < min_pairs / k or _spread(P[m]) < min_spread_m:
            return np.inf, None
        try:
            c = calibrate_from_correspondences(s_uv[m], P[m], image_size, focal_px=focal_px,
                                               optimize_focal=False)
        except Exception:
            return np.inf, None
        err2 = np.sum((c.project(P[m]) - s_uv[m]) ** 2, axis=1)
        w = off_orbit_weights(P[m])
        return float(np.sqrt(np.sum(w * err2) / np.sum(w))), c

    if fixed_offset is not None:
        offset = float(fixed_offset)
        s, _ = score(offset)
        if not np.isfinite(s):
            raise RuntimeError(f"the supplied offset {offset:+.3f} s does not give a usable "
                               f"pairing (overlap {overlap_frac(offset):.0%})")
        landscape = None
        if verbose:
            print(f"  using the supplied offset {offset:+.3f} s (weighted RMSE {s:.2f} px)")
    else:
        # --- coarse seed: cross-correlate image v (down-positive) against mocap z ---
        dt = 0.05
        tb = np.arange(t_bag[0], t_bag[-1], dt)
        tv = np.arange(t_img[0], t_img[-1], dt)
        zs = np.interp(tb, t_bag, bag_track[:, 3])
        vs = np.interp(tv, t_img, -uv[:, 1])
        zs = (zs - zs.mean()) / (zs.std() + 1e-9)
        vs = (vs - vs.mean()) / (vs.std() + 1e-9)
        xc = np.correlate(zs, vs, mode="full")
        # np.correlate index k pairs zs[n + k - (len(vs)-1)] with vs[n]
        lag = (tb[0] - tv[0]) + (int(np.argmax(xc)) - (len(vs) - 1)) * dt
        if verbose:
            print(f"  cross-correlation seed: t_bag = t_video + {lag:+.2f} s")

        grid = np.arange(offset_range[0], offset_range[1] + 1e-9, coarse_step)
        search = np.unique(np.concatenate([grid, lag + np.arange(-3.0, 3.0001, coarse_step / 4)]))
        search = search[(search >= offset_range[0]) & (search <= offset_range[1])]

        landscape = []
        best = (np.inf, None, None)
        for off in search:
            s, c = score(off)
            landscape.append((float(off), s))
            if s < best[0]:
                best = (s, c, off)
        if best[1] is None:
            raise RuntimeError(
                "time alignment failed: no offset gave a trustworthy pairing. Either the bag "
                "and the video are not from the same flight, the overlap is shorter than "
                f"{min_overlap_frac:.0%} of the shorter take, or the paired motion is "
                "degenerate. Try --stride 1, a tighter --roi, or pass --offset yourself.")
        if verbose:
            print(f"  coarse offset {best[2]:+.2f} s  (weighted RMSE {best[0]:.2f} px, "
                  f"overlap {overlap_frac(best[2]):.0%})")

        for step in (coarse_step / 4, 0.01, 0.002):
            centre = best[2]
            for off in np.arange(centre - 6 * step, centre + 6 * step + 1e-9, step):
                s, c = score(off)
                if s < best[0]:
                    best = (s, c, off)
        offset = float(best[2])
        if verbose:
            print(f"  refined offset {offset:+.3f} s")

    P = _interp_track(bag_track, t_img + offset)
    m = np.isfinite(P).all(axis=1)
    calib = calibrate_from_correspondences(uv[m], P[m], image_size, focal_px=focal_px,
                                           optimize_focal=optimize_focal,
                                           optimize_k1=optimize_k1)
    calib.notes["method"] = "ball tracking + mocap PnP"
    calib.notes["time_offset_video_to_bag_s"] = offset
    calib.notes["n_paired_frames"] = int(m.sum())
    calib.notes["overlap_frac"] = float(overlap_frac(offset))
    calib.notes["pair_spread_m"] = float(_spread(P[m]))

    if landscape:
        rival = _rival_minima(landscape, offset, tol=1.5)
        if rival:
            calib.notes["ambiguous_offsets_s"] = rival
            if verbose:
                print(f"  !! the alignment is ambiguous: offsets {rival} score almost as well.\n"
                      "     A constant-height circle is invariant to rotation about world z, so\n"
                      "     the recovered world yaw may be wrong. The *projected circle* is\n"
                      "     unaffected by that, but the axes overlay and the start marker are\n"
                      "     not -- verify them, or pass --offset explicitly.")
    return calib, offset


def _rival_minima(landscape, best_offset, tol=1.5, exclude_s=2.0):
    """Offsets outside +/-exclude_s of the winner that score within `tol` x the best."""
    arr = np.array([(o, s) for o, s in landscape if np.isfinite(s)])
    if not len(arr):
        return []
    best = arr[:, 1].min()
    far = np.abs(arr[:, 0] - best_offset) > exclude_s
    cand = arr[far & (arr[:, 1] < tol * best)]
    if not len(cand):
        return []
    keep, out = cand[np.argsort(cand[:, 1])], []
    for o, _ in keep:
        if all(abs(o - k) > exclude_s for k in out):
            out.append(round(float(o), 2))
        if len(out) >= 3:
            break
    return out


def read_ref_traj(bag_path: str, t0: float | None = None, t1: float | None = None,
                  topic: str | None = None, robot: str = "beetle1"):
    """Read the reference actually commanded during a flight, from /<robot>/set_ref_traj.

    Times are seconds from the bag start.  This is preferable to re-deriving the
    curve from trajs.py: it is what the controller was really given on the day,
    including whichever height offsets and phase the code had at that time.

    Returns ((N, 4) array of t, x, y, z, first point of each MultiDOFJointTrajectory).
    """
    import rosbag

    if topic is None:
        topic = f"/{robot}/set_ref_traj"
    rows = []
    with rosbag.Bag(bag_path) as bag:
        start = bag.get_start_time()
        available = set(bag.get_type_and_topic_info().topics.keys())
        if topic not in available:
            cands = sorted(t for t in available if t.endswith("set_ref_traj"))
            if not cands:
                raise KeyError(f"{topic} not in bag")
            topic = cands[0]
        for _, msg, t in bag.read_messages(topics=[topic]):
            ts = t.to_sec() - start
            if (t0 is not None and ts < t0) or (t1 is not None and ts > t1):
                continue
            if not len(msg.points):
                continue
            tr = msg.points[0].transforms[0].translation
            rows.append((ts, tr.x, tr.y, tr.z))
    return np.array(rows, float).reshape(-1, 4)


def segment_reference(ref: np.ndarray, gap: float = 0.5):
    """Split a reference track into contiguous segments separated by >`gap` seconds."""
    if not len(ref):
        return []
    cuts = np.nonzero(np.diff(ref[:, 0]) > gap)[0]
    return [ref[s] for s in np.split(np.arange(len(ref)), cuts + 1)]


# --------------------------------------------------------------------------------------
# Guided re-tracking
# --------------------------------------------------------------------------------------
def predict_pixels(calib, bag_track: np.ndarray, t_video: np.ndarray, offset: float):
    """Where the ball should appear, per video timestamp. NaN where the bag has no data."""
    P = _interp_track(bag_track, np.asarray(t_video, float) + offset)
    ok = np.isfinite(P).all(axis=1)
    uv = np.full((len(P), 2), np.nan)
    depth = np.full(len(P), np.nan)
    if ok.any():
        uv[ok] = calib.project(P[ok])
        depth[ok] = calib.depths(P[ok])
    return uv, depth, P


def retrack_guided(video_path: str, calib, bag_track: np.ndarray, offset: float,
                   ball_radius_m: float, hsv=None, stride: int = 1, rotate: int = 0,
                   window_px: float = 220.0, radius_tol: float = 0.40,
                   min_circularity: float = 0.45, progress=True, debug_video=None):
    """Re-detect the ball inside a window predicted from mocap + a current calibration.

    A free red-blob tracker cannot reliably tell the ball from the robot's own
    motor pods -- they are the same red, and on this footage roughly half the
    airborne detections landed on a pod.  But mocap already says where the ball
    is to within ~100 px, and how far away it is, which fixes its expected
    apparent radius as f * ball_radius / depth.  Searching only near the
    prediction and only for a blob of about the right size removes the ambiguity
    entirely; the pods are both offset from the prediction and the wrong size.

    Returns an (N, 5) array of (frame_index, t_video, u, v, area).
    """
    p = dict(DEFAULT_RED, **(hsv or {}))
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"cannot open video: {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)

    idx_all = np.arange(0, n_total if n_total else 10 ** 7, stride)
    uv_pred, depth, _ = predict_pixels(calib, bag_track, idx_all / fps, offset)
    r_pred = calib.focal_px * ball_radius_m / np.where(depth > 0.05, depth, np.nan)

    writer = None
    out, idx, slot = [], 0, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % stride == 0:
            slot = idx // stride
            if slot < len(uv_pred) and np.isfinite(uv_pred[slot]).all() and np.isfinite(r_pred[slot]):
                frame = apply_rotation(frame, rotate)
                cu, cv_ = uv_pred[slot]
                rp = float(r_pred[slot])
                half = window_px + 2.0 * rp
                x0 = int(max(0, cu - half)); x1 = int(min(frame.shape[1], cu + half))
                y0 = int(max(0, cv_ - half)); y1 = int(min(frame.shape[0], cv_ + half))
                if x1 - x0 > 8 and y1 - y0 > 8:
                    mask = _red_mask(frame[y0:y1, x0:x1], p)
                    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    best, best_score = None, 0.0
                    for c in cnts:
                        a = cv2.contourArea(c)
                        (bx, by), r = cv2.minEnclosingCircle(c)
                        if r < 3 or a < 20:
                            continue
                        if abs(r - rp) > radius_tol * rp:      # wrong apparent size
                            continue
                        circ = a / max(np.pi * r * r, 1e-9)
                        if circ < min_circularity:
                            continue
                        d = np.hypot(bx + x0 - cu, by + y0 - cv_)
                        score = circ ** 2 * np.exp(-0.5 * (d / max(window_px, 1.0)) ** 2)
                        if score > best_score:
                            best, best_score = (bx + x0, by + y0, a, r), score
                    if best is not None:
                        out.append((idx, idx / fps, best[0], best[1], best[2]))
                    if debug_video:
                        vis = frame.copy()
                        cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 200, 0), 3)
                        cv2.circle(vis, (int(cu), int(cv_)), int(rp), (0, 160, 255), 3)
                        if best is not None:
                            cv2.circle(vis, (int(best[0]), int(best[1])), int(best[3]) + 5,
                                       (0, 255, 0), 4)
                        if writer is None:
                            writer = cv2.VideoWriter(debug_video,
                                                     cv2.VideoWriter_fourcc(*"mp4v"),
                                                     fps / max(1, stride),
                                                     (vis.shape[1], vis.shape[0]))
                        writer.write(vis)
        idx += 1
        if progress and n_total and idx % 400 == 0:
            print(f"\r  re-tracking {idx}/{n_total}, {len(out)} hits", end="")
    cap.release()
    if writer is not None:
        writer.release()
    if progress:
        print(f"\r  re-tracked {idx} frames, {len(out)} detections" + " " * 24)
    return np.array(out, float).reshape(-1, 5)


def decimate_static(track: np.ndarray, world: np.ndarray, move_thresh: float = 0.02,
                    keep_every: int = 20):
    """Thin out stretches where the robot is not moving.

    While the robot sits on the ground, hundreds of frames measure the *same* 3D
    point.  Statistically they are repeats of one observation, but a plain least
    squares counts them as hundreds of independent constraints and lets that
    single image location outvote the entire flight.  Keeping one in
    `keep_every` restores a balanced fit without discarding the information.
    """
    W = np.asarray(world, float)
    speed = np.zeros(len(W))
    if len(W) > 2:
        dt = np.gradient(np.asarray(track[:, 1], float))
        dt[dt == 0] = 1e-6
        speed = np.linalg.norm(np.gradient(W, axis=0), axis=1) / np.abs(dt)
    moving = speed > move_thresh
    keep = moving.copy()
    keep[::keep_every] = True
    return keep


def calibrate_iterative(video_path: str, bag_track: np.ndarray, image_size,
                        pixel_track: np.ndarray, offset: float, focal_px=None,
                        free=("rvec", "tvec", "f", "pp", "k1", "k2"),
                        iters=((260, 0.55, 2), (140, 0.40, 2), (90, 0.32, 1)),
                        hsv=None, rotate: int = 0, f_scale: float = 6.0,
                        offset_halfwidth: float = 0.25, offset_step: float = 0.01,
                        ball_radius_m: float = 0.041, verbose: bool = True):
    """Alternate guided re-tracking with a robust bundle refinement.

    The free tracker cannot separate the ball from the robot's own motor pods, and
    a pinhole-only model cannot absorb the lens distortion; each defect corrupts
    the other's fix.  Alternating them converges: a rough camera predicts where
    the ball must be and how big it must look, that yields a clean track, and a
    clean track supports a full intrinsic model, which sharpens the prediction.

    Returns (Calibration, offset, pixel_track).
    """
    from geometry import refine_full

    def paired(track, off):
        P = _interp_track(bag_track, track[:, 1] + off)
        m = np.isfinite(P).all(axis=1)
        U, W = track[m, 2:4], P[m]
        keep = decimate_static(track[m], W)
        return U[keep], W[keep]

    U, W = paired(pixel_track, offset)
    calib, err = refine_full(U, W, image_size, focal_px=focal_px, free=free,
                             loss="huber", f_scale=f_scale)
    if verbose:
        print(f"  seed: n={len(U)} median {np.median(err):.2f} px  f={calib.focal_px:.0f}")

    for i, (win, rtol, stride) in enumerate(iters, 1):
        uvp, dep, _ = predict_pixels(calib, bag_track, pixel_track[:, 1], offset)
        good = np.isfinite(dep) & (np.linalg.norm(uvp - pixel_track[:, 2:4], axis=1) < 25)
        if good.sum() > 50:
            ball_radius_m = float(np.median(np.sqrt(pixel_track[good, 4] / np.pi)
                                            * dep[good] / calib.focal_px))
        if verbose:
            print(f"  iter {i}: window +/-{win} px, radius tol {rtol:.0%}, "
                  f"ball radius {ball_radius_m*100:.1f} cm")
        track2 = retrack_guided(video_path, calib, bag_track, offset, ball_radius_m,
                                hsv=hsv, stride=stride, rotate=rotate, window_px=win,
                                radius_tol=rtol, progress=verbose)
        if len(track2) < 100:
            if verbose:
                print("    too few guided detections; keeping the previous track")
            break
        pixel_track = track2

        best = (np.inf, offset, calib)
        for off in np.arange(offset - offset_halfwidth, offset + offset_halfwidth + 1e-9,
                             offset_step):
            try:
                U, W = paired(pixel_track, off)
                c, e = refine_full(U[::3], W[::3], image_size, focal_px=calib.focal_px,
                                   free=free, loss="huber", f_scale=f_scale, max_nfev=200)
            except Exception:
                continue
            s_ = float(np.median(e))
            if s_ < best[0]:
                best = (s_, off, c)
        offset = float(best[1])
        U, W = paired(pixel_track, offset)
        calib, err = refine_full(U, W, image_size, focal_px=best[2].focal_px, free=free,
                                 loss="huber", f_scale=f_scale)
        if verbose:
            print(f"    offset {offset:+.3f} s  n={len(U)}  median {np.median(err):.2f} px  "
                  f"p90 {np.percentile(err,90):.1f}  f={calib.focal_px:.0f} "
                  f"k1={calib.dist[0]:+.4f}")

    calib.notes["time_offset_video_to_bag_s"] = offset
    calib.notes["ball_radius_m"] = ball_radius_m
    calib.notes["method"] = "guided ball re-tracking + robust bundle refinement"
    return calib, offset, pixel_track


def read_ref_poses(bag_path: str, t0: float | None = None, t1: float | None = None,
                   topic: str | None = None, robot: str = "beetle1"):
    """Read the commanded reference *pose* (position and orientation) from the bag.

    Like `read_ref_traj`, but keeps the quaternion too, which is what a setpoint
    trajectory is actually about -- `SetPointTraj` commands attitude as well as
    position, and the attitude is the whole point of the manoeuvre.

    Returns an (N, 8) array of (t, x, y, z, qw, qx, qy, qz), t in seconds from the
    bag start.
    """
    import rosbag

    if topic is None:
        topic = f"/{robot}/set_ref_traj"
    rows = []
    with rosbag.Bag(bag_path) as bag:
        start = bag.get_start_time()
        available = set(bag.get_type_and_topic_info().topics.keys())
        if topic not in available:
            cands = sorted(t for t in available if t.endswith("set_ref_traj"))
            if not cands:
                raise KeyError(f"{topic} not in bag")
            topic = cands[0]
        for _, msg, t in bag.read_messages(topics=[topic]):
            ts = t.to_sec() - start
            if (t0 is not None and ts < t0) or (t1 is not None and ts > t1):
                continue
            if not len(msg.points):
                continue
            tf_ = msg.points[0].transforms[0]
            p, q = tf_.translation, tf_.rotation
            rows.append((ts, p.x, p.y, p.z, q.w, q.x, q.y, q.z))
    return np.array(rows, float).reshape(-1, 8)


def segment_poses(poses: np.ndarray, pos_tol: float = 1e-3, ang_tol_deg: float = 0.5,
                  min_duration: float = 0.5):
    """Group a pose stream into the plateaus where the commanded pose is constant.

    A setpoint trajectory is a staircase: the reference jumps between a handful of
    fixed poses and holds each one. This returns one entry per hold, which is what
    you actually want to draw -- a single wireframe per commanded pose, not 400
    identical ones.

    Short plateaus (< `min_duration`) are dropped: they are the one- or two-sample
    slivers left at the edges of a time window, not real commands.
    """
    poses = np.asarray(poses, float).reshape(-1, 8)
    if not len(poses):
        return []
    cos_tol = np.cos(np.radians(ang_tol_deg) / 2.0)

    def same(a, b):
        if np.linalg.norm(a[1:4] - b[1:4]) > pos_tol:
            return False
        # quaternions are a double cover: q and -q are the same rotation
        return abs(float(np.dot(a[4:8], b[4:8]))) >= cos_tol

    groups, cur = [], [0]
    for i in range(1, len(poses)):
        if same(poses[i], poses[cur[0]]):
            cur.append(i)
        else:
            groups.append(cur); cur = [i]
    groups.append(cur)

    out = []
    for g in groups:
        t_start, t_end = poses[g[0], 0], poses[g[-1], 0]
        if (t_end - t_start) < min_duration:
            continue
        out.append({"t0": float(t_start), "t1": float(t_end), "n": len(g),
                    "pos": poses[g[0], 1:4].copy(), "quat": poses[g[0], 4:8].copy()})
    return out
