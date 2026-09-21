#!/usr/bin/env python3
"""Project the mocap-frame reference trajectory of trajs.py into a video recording.

Typical single-frame workflow (no rosbag needed)::

    ./project_reference.py frame     --video flight.mp4 --out frame0.png
    ./project_reference.py picker    --frame frame0.png --out picker.html
    #   ... open picker.html in a browser, click floor landmarks, save points.json
    ./project_reference.py calibrate --points points.json --out calib.json \
                                     --check check.png
    ./project_reference.py render    --video flight.mp4 --calib calib.json \
                                     --out flight_with_ref.mp4

Rosbag workflow (more accurate, and free of frame-convention guesswork)::

    ./project_reference.py calibrate-bag --video flight.mp4 --bag flight.bag \
                                         --out calib.json --check check.png
    ./project_reference.py render --video flight.mp4 --calib calib.json --out out.mp4

Created for the energy-regularized neural-MPC paper video.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import geometry as G           # noqa: E402
import render as R             # noqa: E402


# CircleTraj defaults, mirrored from aerial_robot_planning/scripts/trajs.py
CIRCLE_RADIUS = 1.0
CIRCLE_Z = 0.2 + 0.27 + 0.26           # = 0.73 m, the commanded ball (ee_contact) height


# --------------------------------------------------------------------------------------
# One definition, in render.py, next to the STYLE block it serves.
hex_to_bgr = R.hex_to_bgr


# Arguments that name an *input* file. Checked before any work starts so a typo or a
# copy-pasted placeholder fails with one clear line instead of a rosbag/cv2 traceback.
INPUT_PATH_ARGS = ("calib", "bag", "video", "points", "frame", "ref_from_bag",
                   "track_overlay")


def check_input_paths(args):
    for name in INPUT_PATH_ARGS:
        path = getattr(args, name, None)
        if path is None or os.path.exists(path):
            continue
        flag = "--" + name.replace("_", "-")
        msg = [f"{flag}: no such file: {path}"]
        if not os.path.isabs(path):
            msg.append(f"  (looked in {os.getcwd()})")
        if path.lower().endswith(".bag"):
            d = "/home/jojo_ws/rosbags"
            if os.path.isdir(d):
                bags = sorted(f for f in os.listdir(d) if f.endswith(".bag"))
                if bags:
                    msg.append(f"  bags available in {d}:")
                    msg += [f"    {b}" for b in bags[:8]]
                    if len(bags) > 8:
                        msg.append(f"    ... and {len(bags)-8} more")
        sys.exit("\n".join(msg))


def resolve_style(args, scope: str = "reference"):
    """CLI flags win; anything left unset falls through to render.py's STYLE block.

    `scope="pose"` picks the wireframe's own weight and dash pitch (a pose is a far
    smaller figure than a trajectory). The colour is shared either way.
    """
    thick_default = (R.POSE_THICKNESS_PX if scope == "pose" else R.REFERENCE_THICKNESS_PX)
    dash_default = R.POSE_DASH if scope == "pose" else R.REFERENCE_DASH
    color = R.REFERENCE_COLOR if getattr(args, "color", None) is None else args.color
    thickness = (thick_default if getattr(args, "thickness", None) is None
                 else args.thickness)
    if getattr(args, "solid", False):
        dash = None
    elif getattr(args, "dash", None):
        dash = (float(args.dash[0]), float(args.dash[1]))
    else:
        dash = dash_default
    halo = R.REFERENCE_HALO if getattr(args, "halo", None) is None else args.halo
    return color, thickness, dash, halo


def load_points(path: str):
    """Read a picker/points JSON.  Returns (uv, world, image_size, meta)."""
    with open(path) as f:
        d = json.load(f)
    pts = d["points"]
    uv = np.array([p["uv"] for p in pts], float)
    world = np.array([p["world"] for p in pts], float)
    if world.shape[1] == 2:            # allow 2D entries on the floor plane
        z = float(d.get("floor_z", 0.0))
        world = np.column_stack([world, np.full(len(world), z)])
    return uv, world, tuple(d.get("image_size", (0, 0))), d


_REF_CACHE = {}
_TRACK_CACHE = {}


def build_reference(args):
    """The 3D curve to draw: either read from the bag, or CircleTraj from parameters.

    Cached: `render` calls this once per frame, and re-reading a 200 MB rosbag for
    every one of 5000 frames is the difference between seconds and hours.
    """
    key = (getattr(args, "ref_from_bag", None), tuple(args.ref_window or ()),
           args.radius, args.z, args.samples)
    if key not in _REF_CACHE:
        _REF_CACHE[key] = _build_reference_uncached(args)
    return _REF_CACHE[key]


def _build_reference_uncached(args):
    if getattr(args, "ref_from_bag", None):
        import bag_calib
        ref = bag_calib.read_ref_traj(args.ref_from_bag, robot=getattr(args, "robot", "beetle1"))
        segs = bag_calib.segment_reference(ref)
        if args.ref_window:
            t0, t1 = args.ref_window
            ref = ref[(ref[:, 0] >= t0) & (ref[:, 0] <= t1)]
            if len(ref) < 2:
                sys.exit(f"no reference points in bag window [{t0}, {t1}] s. Segments present: "
                         + "; ".join(f"[{s[0,0]:.1f},{s[-1,0]:.1f}]s" for s in segs))
        elif len(segs) > 1:
            sys.exit("the bag holds several reference segments; pick one with --ref-window T0 T1:\n"
                     + "\n".join(f"  [{s[0,0]:7.2f}, {s[-1,0]:7.2f}] s  "
                                  f"x[{s[:,1].min():+.2f},{s[:,1].max():+.2f}] "
                                  f"y[{s[:,2].min():+.2f},{s[:,2].max():+.2f}] "
                                  f"z[{s[:,3].min():+.2f},{s[:,3].max():+.2f}]" for s in segs))
        return ref[:, 1:4]
    return G.circle_reference(radius=args.radius, z=args.z, n=args.samples)


def add_style_args(ap):
    ap.add_argument("--radius", type=float, default=CIRCLE_RADIUS, help="circle radius [m]")
    ap.add_argument("--z", type=float, default=CIRCLE_Z,
                    help="reference height [m] in the mocap frame (default 0.73 = CircleTraj)")
    ap.add_argument("--samples", type=int, default=720)
    # These default to None on purpose: the fallback lives in render.py's STYLE block,
    # so editing that one place actually changes the output. A literal default here
    # would silently override it.
    ap.add_argument("--color", default=None,
                    help=f"reference colour #RRGGBB (default {R.REFERENCE_COLOR} "
                         f"from render.py STYLE)")
    ap.add_argument("--thickness", type=int, default=None,
                    help=f"line width in px (default {R.REFERENCE_THICKNESS_PX})")
    ap.add_argument("--dash", type=float, nargs=2, metavar=("ON", "OFF"), default=None,
                    help=f"dash pattern in px (default {R.REFERENCE_DASH_ON_PX} "
                         f"{R.REFERENCE_DASH_OFF_PX})")
    ap.add_argument("--solid", action="store_true", help="draw a solid line instead of dashed")
    ap.add_argument("--halo", action="store_true", default=None,
                    help="add a dark contrast halo behind the line")
    ap.add_argument("--floor-z", type=float, default=None,
                    help="floor height in the mocap frame [m]; enables the ground shadow")
    ap.add_argument("--no-shadow", action="store_true")
    ap.add_argument("--droppers", type=int, default=12,
                    help="number of vertical height cues (0 to disable)")
    ap.add_argument("--axes", action="store_true", help="draw the world-frame triad at the origin")
    ap.add_argument("--grid", action="store_true", help="overlay the metric floor grid")
    ap.add_argument("--grid-step", type=float, default=0.5)
    ap.add_argument("--grid-extent", type=float, default=3.0)
    ap.add_argument("--hud", action="store_true", help="print calibration info on the frame")
    ap.add_argument("--ref-from-bag", metavar="BAG",
                    help="draw the reference actually commanded, read from /<robot>/set_ref_traj, "
                         "instead of re-deriving CircleTraj from --radius/--z")
    ap.add_argument("--ref-window", type=float, nargs=2, metavar=("T0", "T1"),
                    help="bag-time window [s] selecting which reference segment to draw")
    ap.add_argument("--track-overlay", metavar="BAG",
                    help="also draw the measured CoG trajectory from /<robot>/uav/cog/odom")
    ap.add_argument("--track-window", type=float, nargs=2, metavar=("T0", "T1"))
    ap.add_argument("--track-color", default="#FF5A78")


def paint(img, calib, args):
    ref = build_reference(args)
    closed = bool(np.linalg.norm(ref[0] - ref[-1]) < 0.05)
    color, thickness, dash, halo = resolve_style(args)
    # describe what is actually being drawn, not what the defaults say
    ctr = ref[:, :2].mean(axis=0)
    rad = float(np.median(np.linalg.norm(ref[:, :2] - ctr, axis=1)))
    zlo, zhi = float(ref[:, 2].min()), float(ref[:, 2].max())
    if args.grid:
        R.draw_floor_grid(img, calib, floor_z=(args.floor_z or 0.0),
                          extent=args.grid_extent, step=args.grid_step)
    if getattr(args, "track_overlay", None):
        tkey = (args.track_overlay, tuple(args.track_window or ()))
        if tkey not in _TRACK_CACHE:
            import bag_calib
            tr, _ = bag_calib.read_ee_track(
                args.track_overlay, topic=f"/{getattr(args,'robot','beetle1')}/uav/cog/odom")
            if args.track_window:
                tr = tr[(tr[:, 0] >= args.track_window[0]) & (tr[:, 0] <= args.track_window[1])]
            _TRACK_CACHE[tkey] = tr
        tr = _TRACK_CACHE[tkey]
        R.draw_world_polyline(img, calib, tr[:, 1:4], color=hex_to_bgr(args.track_color),
                              thickness=max(2, thickness - 1), alpha=0.9)
    R.draw_reference(img, calib, ref,
                     floor_z=None if (args.no_shadow or args.floor_z is None) else args.floor_z,
                     color=color, thickness=thickness, dash=dash, halo=halo,
                     droppers=args.droppers, closed=closed)
    if args.axes:
        R.draw_world_axes(img, calib, length=0.5, origin=(0, 0, args.floor_z or 0.0))
    if args.hud:
        n = calib.notes
        acc = (f"median {n['reprojection_median_px']:.2f} px, p90 {n['reprojection_p90_px']:.1f} px"
               if "reprojection_median_px" in n
               else f"RMSE {n.get('reprojection_rmse_px', float('nan')):.2f} px")
        zdesc = f"z={zlo:.2f} m" if (zhi - zlo) < 1e-3 else f"z={zlo:.2f}..{zhi:.2f} m"
        R.draw_hud(img, [
            f"f = {calib.focal_px:.0f} px   HFOV = {calib.horizontal_fov_deg():.1f} deg"
            f"   k1 = {calib.dist[0]:+.3f}",
            "cam @ world (" + ", ".join(f"{v:+.2f}" for v in calib.camera_position_world) + ") m",
            f"reprojection: {acc}",
            f"reference: r={rad:.2f} m about ({ctr[0]:+.2f}, {ctr[1]:+.2f}), {zdesc}",
        ], scale=1.6 if img.shape[1] > 2000 else 0.7)
    return img


# --------------------------------------------------------------------------------------
def cmd_frame(args):
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit(f"cannot open video: {args.video}")
    if args.time is not None:
        cap.set(cv2.CAP_PROP_POS_MSEC, args.time * 1000.0)
    elif args.index:
        cap.set(cv2.CAP_PROP_POS_FRAMES, args.index)
    ok, frame = cap.read()
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    if not ok:
        sys.exit("failed to read the requested frame")
    import bag_calib
    frame = bag_calib.apply_rotation(frame, args.rotate)
    cv2.imwrite(args.out, frame)
    print(f"wrote {args.out}  ({frame.shape[1]}x{frame.shape[0]}); "
          f"video has {n} frames @ {fps:.3f} fps")


def cmd_picker(args):
    import picker
    frame = cv2.imread(args.frame)
    if frame is None:
        sys.exit(f"cannot read frame: {args.frame}")
    picker.write_picker_html(frame, args.out, tile=args.tile)
    print(f"wrote {args.out}  --  open it in a browser, click landmarks, save the JSON")


def cmd_calibrate(args):
    uv, world, size, meta = load_points(args.points)
    frame = cv2.imread(args.frame) if args.frame else None
    if frame is not None:
        size = (frame.shape[1], frame.shape[0])
    if not size or not all(size):
        sys.exit("image_size missing: pass --frame, or put image_size in the points file")

    focal = args.focal_px
    if focal is None and args.hfov_deg is not None:
        focal = G.focal_from_hfov(args.hfov_deg, size[0])
    dist = np.array([args.k1, args.k2, 0.0, 0.0, 0.0], float)

    zs = world[:, 2]
    coplanar = float(np.ptp(zs)) < 1e-6
    if coplanar:
        print(f"{len(uv)} coplanar points at z = {zs[0]:.3f} m -> ground-plane homography")
        calib = G.calibrate_from_ground_points(uv, world, size, focal_px=focal,
                                               dist=dist if np.any(dist) else None)
    else:
        print(f"{len(uv)} points spanning z = [{zs.min():.3f}, {zs.max():.3f}] m -> PnP")
        calib = G.calibrate_from_correspondences(
            uv, world, size, focal_px=focal, dist=dist if np.any(dist) else None,
            optimize_focal=(focal is None or args.optimize_focal),
            optimize_k1=args.optimize_k1)

    calib.notes["points_file"] = os.path.abspath(args.points)
    calib.notes.update({k: v for k, v in meta.items() if k in ("floor_z", "note", "tile")})
    calib.to_json(args.out)
    report(calib, uv, world)

    if args.check:
        if frame is None:
            sys.exit("--check needs --frame")
        img = frame.copy()
        args.grid = True
        paint(img, calib, args)
        for p, w in zip(calib.project(world), world):
            cv2.drawMarker(img, tuple(np.round(p).astype(int)), (0, 0, 255),
                           cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
        for p in uv:
            cv2.circle(img, tuple(np.round(p).astype(int)), 6, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imwrite(args.check, img)
        print(f"wrote {args.check}  (green circles = clicked, red crosses = reprojected)")


def cmd_calibrate_bag(args):
    import bag_calib
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit(f"cannot open video: {args.video}")
    ok0, probe = cap.read()
    cap.release()
    if not ok0:
        sys.exit("cannot read the first frame")
    probe = bag_calib.apply_rotation(probe, args.rotate)
    size = (probe.shape[1], probe.shape[0])

    roi = tuple(args.roi) if args.roi else None
    if args.track_npy and os.path.exists(args.track_npy):
        track = np.load(args.track_npy)
        print(f"reusing cached pixel track {args.track_npy} ({len(track)} detections)")
    else:
        print("tracking the ball in the video ...")
        hsv = dict(s_min=args.hsv_smin, v_min=args.hsv_vmin, h_lo=args.hsv_hlo,
                   h_hi=args.hsv_hhi, min_area=args.min_area, max_area=args.max_area)
        track, fps = bag_calib.track_red_ball(
            args.video, hsv=hsv, stride=args.stride, roi=roi, rotate=args.rotate,
            debug_video=args.debug_track, min_radius=args.min_radius,
            max_radius=args.max_radius, min_circularity=args.min_circularity)
        if args.track_npy:
            np.save(args.track_npy, track)
    if len(track) < 50:
        sys.exit(f"only {len(track)} ball detections -- tune --roi or the HSV thresholds")

    print("reading the mocap track from the bag ...")
    bag_track, topic = bag_calib.read_ee_track(args.bag, topic=args.topic, robot=args.robot)
    print(f"  {len(bag_track)} samples from {topic}, "
          f"t = [0, {bag_track[-1,0]:.1f}] s, "
          f"z = [{bag_track[:,3].min():.2f}, {bag_track[:,3].max():.2f}] m")

    focal = args.focal_px
    if focal is None and args.hfov_deg is not None:
        focal = G.focal_from_hfov(args.hfov_deg, size[0])

    print("aligning time and solving for the camera ...")
    calib, offset = bag_calib.align_and_calibrate(
        track, bag_track, size, focal_px=focal,
        optimize_focal=args.optimize_focal, optimize_k1=args.optimize_k1,
        fixed_offset=args.offset, min_overlap_frac=args.min_overlap_frac)

    if args.guided_iters > 0:
        print("refining by guided re-tracking (the free tracker confuses the ball with "
              "the motor pods) ...")
        hsv = dict(s_min=args.hsv_smin, v_min=args.hsv_vmin, h_lo=args.hsv_hlo,
                   h_hi=args.hsv_hhi, min_area=args.min_area, max_area=args.max_area)
        schedule = [(260, 0.55, 2), (140, 0.40, 2), (90, 0.32, 1)][:args.guided_iters]
        calib, offset, track = bag_calib.calibrate_iterative(
            args.video, bag_track, size, track, offset, focal_px=calib.focal_px,
            free=tuple(args.free), iters=tuple(schedule), hsv=hsv, rotate=args.rotate,
            f_scale=args.f_scale)
        if args.track_npy:
            np.save(args.track_npy, track)
    calib.to_json(args.out)
    report(calib)

    if args.check:
        cap = cv2.VideoCapture(args.video)
        ok, frame = cap.read(); cap.release()
        if ok:
            frame = bag_calib.apply_rotation(frame, args.rotate)
            args.grid = True
            paint(frame, calib, args)
            uv = track[:, 2:4]
            for p in uv[::max(1, len(uv) // 400)]:
                cv2.circle(frame, tuple(np.round(p).astype(int)), 2, (0, 255, 0), -1, cv2.LINE_AA)
            cv2.imwrite(args.check, frame)
            print(f"wrote {args.check}  (green dots = the tracked ball over the whole flight)")


def report(calib, uv=None, world=None):
    print(f"\nwrote calibration")
    print(f"  focal          : {calib.focal_px:.1f} px   (HFOV {calib.horizontal_fov_deg():.1f} deg)")
    print(f"  camera position: ({', '.join(f'{v:+.3f}' for v in calib.camera_position_world)}) m "
          f"in the mocap frame")
    print(f"  distortion     : {np.asarray(calib.dist).ravel()[:2]}")
    for k, v in calib.notes.items():
        print(f"  {k:15s}: {v}")
    if uv is not None:
        err = np.linalg.norm(calib.project(world) - uv, axis=1)
        print(f"  per-point reprojection error: max {err.max():.2f} px, "
              f"median {np.median(err):.2f} px")
        if err.max() > 6.0:
            print("  !! a point is off by more than 6 px -- check it in the --check image")


def cmd_measure(args):
    """Back-project image points onto a horizontal plane and report world distances.

    The point of this is to check the metric assumption that went into the
    calibration.  The robot itself is a ruler: PhysParamBeetleOmniJetson.yaml puts
    the rotors at +/-0.1948 m in x and y, so adjacent motor centres are 0.3896 m
    apart and diagonal ones 0.5510 m.  Click those on a plane of known height and
    the numbers should come back right; if they do not, the tile size (or the
    plane height) fed to `calibrate` was wrong.
    """
    calib = G.Calibration.from_json(args.calib)
    uv = np.array(args.points, float).reshape(-1, 2)
    P = plane_backproject(calib, uv, args.plane_z)
    print(f"back-projected onto z = {args.plane_z:.3f} m:")
    for i, p in enumerate(P):
        print(f"  p{i+1}: image ({uv[i,0]:8.1f}, {uv[i,1]:8.1f}) -> world "
              f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m")
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            print(f"  |p{i+1} p{j+1}| = {np.linalg.norm(P[i] - P[j]):.4f} m")
    print("\nreference lengths on the robot (PhysParamBeetleOmniJetson.yaml):")
    print("  adjacent motor centres 0.3896 m   diagonal motor centres 0.5510 m")
    print("  ball (ee_contact) centre sits 0.264 m above the CoG")


def plane_backproject(calib: G.Calibration, uv: np.ndarray, plane_z: float) -> np.ndarray:
    """Intersect the rays through `uv` with the horizontal plane z = plane_z."""
    uv = np.asarray(uv, float).reshape(-1, 1, 2)
    rays = cv2.undistortPoints(uv, calib.K, calib.dist).reshape(-1, 2)
    d_cam = np.column_stack([rays, np.ones(len(rays))])
    d_world = d_cam @ calib.R                       # R^T d, written as a row-vector product
    C = calib.camera_position_world
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (plane_z - C[2]) / d_world[:, 2]
    return C[None, :] + s[:, None] * d_world


def cmd_overlay(args):
    """Write the reference curve alone to a transparent PNG, for compositing by hand.

    No video frame, no robot -- just the line on alpha. The camera is static, so one
    PNG serves the whole clip; drop it on top of the footage in any editor.
    """
    calib = G.Calibration.from_json(args.calib)
    size = tuple(args.size) if args.size else tuple(calib.image_size)
    if not size or not all(size):
        sys.exit("output size unknown: pass --size W H, or use a calibration that "
                 "records image_size")
    if tuple(calib.image_size) != tuple(size):
        print(f"note: drawing at {size[0]}x{size[1]}, calibration was made at "
              f"{calib.image_size[0]}x{calib.image_size[1]}")

    ref = build_reference(args)
    closed = bool(np.linalg.norm(ref[0] - ref[-1]) < 0.05)
    color, thickness, dash, halo = resolve_style(args)

    extra = []
    if not args.no_shadow and args.floor_z is not None:
        gnd = ref.copy(); gnd[:, 2] = args.floor_z
        extra.append((gnd, closed))
        if args.droppers:
            idx = np.linspace(0, len(ref) - 1, int(args.droppers), endpoint=False).astype(int)
            for i in idx:
                extra.append((np.array([ref[i], [ref[i, 0], ref[i, 1], args.floor_z]]), False))

    rgba = R.render_reference_rgba(size, calib, ref, color=color, thickness=thickness,
                                   dash=dash, closed=closed, halo=halo,
                                   extra_polylines=extra)
    if args.scale != 1.0:
        # Safe here only because the colour channels are uniform everywhere, including
        # under fully transparent pixels -- interpolating them cannot introduce fringes.
        w, h = int(round(size[0] * args.scale)), int(round(size[1] * args.scale))
        rgba = cv2.resize(rgba, (w, h), interpolation=cv2.INTER_AREA)

    if not args.out.lower().endswith(".png"):
        sys.exit("the overlay must be written as .png -- no other format keeps the alpha")
    if not cv2.imwrite(args.out, rgba):
        sys.exit(f"failed to write {args.out}")

    a = rgba[..., 3]
    cov = float((a > 0).mean())
    print(f"wrote {args.out}  ({rgba.shape[1]}x{rgba.shape[0]} BGRA)")
    print(f"  colour     : {color}  -> BGR {hex_to_bgr(color)}"
          f"{'' if args.color else '   [render.py STYLE: REFERENCE_COLOR]'}")
    print(f"  line       : {thickness} px, " +
          (f"dashed {dash[0]:g} on / {dash[1]:g} off px" if dash else "solid") +
          (", with halo" if halo else ""))
    print(f"  background : transparent ({(1-cov)*100:.2f}% of pixels fully clear)")
    print(f"  composite  : place this PNG over the video at {rgba.shape[1]}x{rgba.shape[0]}, "
          "normal/over blending, 1:1 with no offset")


def _distinct_poses(plateaus, pos_tol=1e-3, ang_tol_deg=0.5):
    """Collapse repeated visits to the same commanded pose into one entry.

    A setpoint run typically returns to its neutral pose at both ends of a window,
    which is one pose seen twice, not two poses. `hold` keeps the *longest single*
    plateau, so a pose clipped by the window edge is not credited with the sum of
    its fragments.
    """
    cos_tol = np.cos(np.radians(ang_tol_deg) / 2.0)
    out = []
    for p in plateaus:
        for o in out:
            if (np.linalg.norm(o["pos"] - p["pos"]) <= pos_tol
                    and abs(float(np.dot(o["quat"], p["quat"]))) >= cos_tol):
                o["hold"] = max(o["hold"], p["t1"] - p["t0"])
                o["visits"] += 1
                o["t1"] = max(o["t1"], p["t1"])
                break
        else:
            out.append({"t0": p["t0"], "t1": p["t1"], "pos": p["pos"], "quat": p["quat"],
                        "hold": p["t1"] - p["t0"], "visits": 1})
    return out


def cmd_pose_overlay(args):
    """One transparent PNG per commanded *pose*, drawn as a dashed drone wireframe.

    For a setpoint trajectory the reference is not a path but a staircase of held
    poses, and the attitude is half the command -- so each pose gets its own image
    showing where the airframe was told to be and how it was told to be oriented.
    """
    import bag_calib

    calib = G.Calibration.from_json(args.calib)
    size = tuple(args.size) if args.size else tuple(calib.image_size)
    if not size or not all(size):
        sys.exit("output size unknown: pass --size W H")

    window = args.window
    if args.video_window:
        off = args.offset
        if off is None:
            off = calib.notes.get("time_offset_video_to_bag_s")
        if off is None:
            sys.exit("--video-window needs the video->bag time offset: pass --offset, or use "
                     "a calibration whose notes carry time_offset_video_to_bag_s")
        window = [args.video_window[0] + off, args.video_window[1] + off]
        print(f"video window {args.video_window[0]:.2f}-{args.video_window[1]:.2f} s "
              f"-> bag {window[0]:.2f}-{window[1]:.2f} s   (offset {off:+.2f} s)")
    if not window:
        sys.exit("pass --window T0 T1 (bag seconds) or --video-window T0 T1 (video seconds)")

    poses = bag_calib.read_ref_poses(args.bag, t0=window[0], t1=window[1], robot=args.robot)
    if not len(poses):
        sys.exit(f"no reference poses on /{args.robot}/set_ref_traj in bag window "
                 f"[{window[0]:.2f}, {window[1]:.2f}] s")
    plateaus = bag_calib.segment_poses(poses)
    found = _distinct_poses(plateaus)
    kept = [p for p in found if p["hold"] >= args.min_hold]

    print(f"\n{len(poses)} reference samples -> {len(plateaus)} plateau(s) -> "
          f"{len(found)} distinct pose(s); {len(kept)} held >= {args.min_hold:g} s")
    for i, p in enumerate(found, 1):
        rpy = np.degrees(G.rot_to_rpy(G.quat_to_rot(p["quat"])))
        mark = "  " if p in kept else " (skipped: held %.2f s)" % p["hold"]
        print(f"  pose {i}: pos=({p['pos'][0]:+.3f}, {p['pos'][1]:+.3f}, {p['pos'][2]:+.3f}) m   "
              f"rpy=({rpy[0]:+6.1f}, {rpy[1]:+6.1f}, {rpy[2]:+6.1f}) deg   "
              f"held {p['hold']:.2f} s x{p['visits']}{mark}")
    if not kept:
        sys.exit("nothing to draw; lower --min-hold")
    if args.pose_index:
        sel = [i - 1 for i in args.pose_index]
        bad = [i + 1 for i in sel if not (0 <= i < len(kept))]
        if bad:
            sys.exit(f"--pose-index {bad} out of range (1..{len(kept)})")
        kept = [kept[i] for i in sel]

    color, thickness, dash, halo = resolve_style(args, scope="pose")
    geom = G.load_robot_geometry()
    cam = calib.camera_position_world
    stem, ext = os.path.splitext(args.out)
    if ext.lower() != ".png":
        sys.exit("the overlay must be written as .png -- no other format keeps the alpha")

    print()
    for i, p in enumerate(kept, 1):
        wf = G.drone_wireframe(
            p["pos"], p["quat"], geom=geom, camera_position=cam,
            arms=not args.no_arms, props=not args.no_props,
            mast=not args.no_mast, ball=not args.no_ball, square=args.square,
            prop_radius=(None if args.prop_diameter_in is None
                         else 0.5 * args.prop_diameter_in * 0.0254))
        rgba = R.render_reference_rgba(size, calib, polylines=wf, color=color,
                                       thickness=thickness, dash=dash, halo=halo)
        if args.scale != 1.0:
            w, h = int(round(size[0] * args.scale)), int(round(size[1] * args.scale))
            rgba = cv2.resize(rgba, (w, h), interpolation=cv2.INTER_AREA)
        out = args.out if len(kept) == 1 else f"{stem}_pose{i}{ext}"
        if not cv2.imwrite(out, rgba):
            sys.exit(f"failed to write {out}")
        rpy = np.degrees(G.rot_to_rpy(G.quat_to_rot(p["quat"])))
        clear = float((rgba[..., 3] == 0).mean()) * 100.0
        print(f"wrote {out}  ({rgba.shape[1]}x{rgba.shape[0]} BGRA, {clear:.2f}% clear)")
        print(f"     pose: CoG at ({p['pos'][0]:+.3f}, {p['pos'][1]:+.3f}, {p['pos'][2]:+.3f}) m, "
              f"roll {rpy[0]:+.1f} deg, pitch {rpy[1]:+.1f} deg, yaw {rpy[2]:+.1f} deg")
        print(f"     held bag t = {p['t0']:.2f} s")
    print(f"\n  colour {color} -> BGR {hex_to_bgr(color)}, {thickness} px, " +
          (f"dashed {dash[0]:g}/{dash[1]:g} px" if dash else "solid"))
    pr = (geom["prop_radius"] if args.prop_diameter_in is None
          else 0.5 * args.prop_diameter_in * 0.0254)
    parts = []
    if not args.no_arms:
        parts.append("4 arms")
    if not args.no_props:
        parts.append(f"4 propellers ({pr*2/0.0254:.1f} in dia = {pr*2:.4f} m)")
    if args.square:
        parts.append("rotor square")
    if not args.no_mast:
        parts.append("mast")
    if not args.no_ball:
        parts.append(f"ball (r={geom['ball_radius']:.3f} m)")
    print(f"  geometry from {geom['source']}")
    print(f"  drawn: {', '.join(parts) if parts else 'nothing (everything switched off)'}")
    print(f"  composite 1:1 over the video at {size[0]}x{size[1]}, normal/over blending")


def cmd_preview(args):
    frame = cv2.imread(args.frame)
    if frame is None:
        sys.exit(f"cannot read frame: {args.frame}")
    calib = G.Calibration.from_json(args.calib)
    cv2.imwrite(args.out, paint(frame, calib, args))
    print(f"wrote {args.out}")


def cmd_render(args):
    import bag_calib
    calib = G.Calibration.from_json(args.calib)
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit(f"cannot open video: {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    ok0, probe = cap.read()
    if not ok0:
        sys.exit("cannot read the first frame")
    h, w = bag_calib.apply_rotation(probe, args.rotate).shape[:2]
    if calib.image_size and tuple(calib.image_size) != (w, h):
        sys.exit(f"frame size {w}x{h} does not match the calibration "
                 f"{calib.image_size[0]}x{calib.image_size[1]} -- wrong --rotate, or a "
                 f"different video?")
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    sc = float(args.scale)
    ow, oh = int(round(w * sc)), int(round(h * sc))
    writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*args.fourcc), fps, (ow, oh))
    if not writer.isOpened():
        sys.exit(f"cannot open the writer with fourcc {args.fourcc}; try --fourcc avc1 or MJPG")

    f0 = int(round((args.start or 0.0) * fps))
    f1 = int(round(args.end * fps)) if args.end is not None else (n or 10 ** 9)
    if f0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, f0)
    total = max(1, f1 - f0)

    i, idx = 0, f0
    while idx < f1:
        ok, frame = cap.read()
        if not ok:
            break
        out = paint(bag_calib.apply_rotation(frame, args.rotate), calib, args)
        if sc != 1.0:
            out = cv2.resize(out, (ow, oh), interpolation=cv2.INTER_AREA)
        writer.write(out)
        i += 1; idx += 1
        if i % 60 == 0:
            print(f"\r  {i}/{total} frames", end="")
    cap.release(); writer.release()
    print(f"\rwrote {args.out}  ({i} frames, {fps:.3f} fps, {ow}x{oh})" + " " * 20)
    print("note: OpenCV drops the audio track. To keep it, mux the original audio back in:")
    print(f"  ffmpeg -i {args.out} -i {args.video} -c copy -map 0:v:0 -map 1:a:0? final.mp4")


# --------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("frame", help="extract one frame from the video")
    p.add_argument("--video", required=True); p.add_argument("--out", required=True)
    p.add_argument("--index", type=int, default=0); p.add_argument("--time", type=float)
    p.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270])
    p.set_defaults(func=cmd_frame)

    p = sub.add_parser("picker", help="build a standalone HTML point picker for a frame")
    p.add_argument("--frame", required=True); p.add_argument("--out", default="picker.html")
    p.add_argument("--tile", type=float, default=0.5, help="floor tile size [m]")
    p.set_defaults(func=cmd_picker)

    p = sub.add_parser("calibrate", help="calibrate from clicked 2D<->3D correspondences")
    p.add_argument("--points", required=True); p.add_argument("--out", default="calib.json")
    p.add_argument("--frame"); p.add_argument("--check")
    p.add_argument("--focal-px", type=float); p.add_argument("--hfov-deg", type=float)
    p.add_argument("--k1", type=float, default=0.0); p.add_argument("--k2", type=float, default=0.0)
    p.add_argument("--optimize-focal", action="store_true")
    p.add_argument("--optimize-k1", action="store_true")
    add_style_args(p); p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("calibrate-bag", help="calibrate by matching the tracked ball to mocap")
    p.add_argument("--video", required=True); p.add_argument("--bag", required=True)
    p.add_argument("--out", default="calib.json"); p.add_argument("--check")
    p.add_argument("--robot", default="beetle1"); p.add_argument("--topic")
    p.add_argument("--stride", type=int, default=2)
    p.add_argument("--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"))
    p.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270],
                   help="rotate decoded frames; phones store an orientation flag "
                        "that OpenCV ignores")
    p.add_argument("--debug-track", metavar="MP4",
                   help="write a video of the red mask and the accepted detection")
    p.add_argument("--hsv-smin", type=int, default=70, help="min HSV saturation for red")
    p.add_argument("--hsv-vmin", type=int, default=40, help="min HSV value for red")
    p.add_argument("--hsv-hlo", type=int, default=12, help="upper hue of the low-red lobe")
    p.add_argument("--hsv-hhi", type=int, default=166, help="lower hue of the high-red lobe")
    p.add_argument("--min-area", type=int, default=600, help="min blob area [px^2]")
    p.add_argument("--max-area", type=int, default=120000, help="max blob area [px^2]")
    p.add_argument("--min-radius", type=float, default=8.0)
    p.add_argument("--max-radius", type=float, default=200.0)
    p.add_argument("--min-circularity", type=float, default=0.62)
    p.add_argument("--track-npy", help="cache the pixel track here (reused if it exists)")
    p.add_argument("--guided-iters", type=int, default=3,
                   help="rounds of guided re-tracking + robust bundle refinement (0 to skip)")
    p.add_argument("--free", nargs="+", default=["rvec", "tvec", "f", "pp", "k1", "k2"],
                   help="intrinsic/pose groups to optimise in the final refinement")
    p.add_argument("--f-scale", type=float, default=6.0,
                   help="Huber transition in px; roughly the accuracy you expect")
    p.add_argument("--focal-px", type=float); p.add_argument("--hfov-deg", type=float)
    p.add_argument("--optimize-focal", action="store_true", default=True)
    p.add_argument("--no-optimize-focal", dest="optimize_focal", action="store_false")
    p.add_argument("--optimize-k1", action="store_true")
    p.add_argument("--offset", type=float,
                   help="known video->bag time offset in seconds (t_bag = t_video + offset); "
                        "skips the search")
    p.add_argument("--min-overlap-frac", type=float, default=0.5)
    add_style_args(p); p.set_defaults(func=cmd_calibrate_bag)

    p = sub.add_parser("measure", help="back-project image points to a plane and measure")
    p.add_argument("--calib", required=True)
    p.add_argument("--points", type=float, nargs="+", required=True,
                   metavar="U V", help="pixel coordinates, e.g. --points 812 903 1044 878")
    p.add_argument("--plane-z", type=float, default=0.0)
    p.set_defaults(func=cmd_measure)

    p = sub.add_parser("overlay", help="write the reference alone to a transparent PNG")
    p.add_argument("--calib", required=True)
    p.add_argument("--out", default="reference_overlay.png")
    p.add_argument("--size", type=int, nargs=2, metavar=("W", "H"),
                   help="output size; defaults to the calibration's image_size")
    p.add_argument("--scale", type=float, default=1.0,
                   help="e.g. 0.5 to emit 1080p for a 4K calibration")
    p.add_argument("--robot", default="beetle1")
    add_style_args(p); p.set_defaults(func=cmd_overlay)

    p = sub.add_parser("pose-overlay",
                       help="one transparent PNG per commanded pose, as a drone wireframe")
    p.add_argument("--calib", required=True)
    p.add_argument("--bag", required=True)
    p.add_argument("--out", default="pose.png",
                   help="output path; with several poses, _pose1/_pose2/... is inserted")
    p.add_argument("--window", type=float, nargs=2, metavar=("T0", "T1"),
                   help="bag-time window [s]")
    p.add_argument("--video-window", type=float, nargs=2, metavar=("T0", "T1"),
                   help="video-time window [s]; converted with the calibration's offset")
    p.add_argument("--offset", type=float,
                   help="video->bag offset [s]; defaults to the calibration's notes")
    p.add_argument("--min-hold", type=float, default=1.0,
                   help="ignore poses held for less than this [s]; filters the fragments "
                        "left at the edges of the window (default 1.0)")
    p.add_argument("--pose-index", type=int, nargs="+",
                   help="render only these poses, 1-based, from the kept list")
    p.add_argument("--size", type=int, nargs=2, metavar=("W", "H"))
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--robot", default="beetle1")
    p.add_argument("--no-arms", action="store_true")
    p.add_argument("--no-props", action="store_true")
    p.add_argument("--no-mast", action="store_true")
    p.add_argument("--no-ball", action="store_true")
    p.add_argument("--square", action="store_true",
                   help="also outline the rotor plane (off by default)")
    p.add_argument("--prop-diameter-in", type=float, default=None,
                   help=f"propeller diameter in inches (default "
                        f"{G.PROP_DIAMETER_M/0.0254:g})")
    add_style_args(p); p.set_defaults(func=cmd_pose_overlay)

    p = sub.add_parser("preview", help="draw the reference on a single frame")
    p.add_argument("--frame", required=True); p.add_argument("--calib", required=True)
    p.add_argument("--out", default="preview.png")
    add_style_args(p); p.set_defaults(func=cmd_preview)

    p = sub.add_parser("render", help="draw the reference on every frame of the video")
    p.add_argument("--video", required=True); p.add_argument("--calib", required=True)
    p.add_argument("--out", default="out.mp4"); p.add_argument("--fourcc", default="mp4v")
    p.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270],
                   help="must match the --rotate used when calibrating")
    p.add_argument("--start", type=float, help="first video second to render")
    p.add_argument("--end", type=float, help="last video second to render")
    p.add_argument("--scale", type=float, default=1.0,
                   help="output scale, e.g. 0.5 to halve 4K to 1080p (drawing still "
                        "happens at full resolution)")
    add_style_args(p); p.set_defaults(func=cmd_render)

    args = ap.parse_args()
    check_input_paths(args)
    args.func(args)


if __name__ == "__main__":
    main()
