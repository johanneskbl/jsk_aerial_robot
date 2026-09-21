# HANDOFF — notes to myself for the next session

Working notes for resuming or extending this tool. Terse on purpose. The
human-facing explanation is in `README.md`; this file is the operational residue:
exact numbers, exact paths, what breaks, and what not to re-derive.

Written 2026-09-11 after building the tool and solving
`PXL_20260329_094920628.mp4`.

---

## 1. Fast path

If the job is "overlay a reference on a flight video" and a same-flight bag exists:

```bash
cd /home/jojo_ws/src/jsk_aerial_robot/aerial_robot_control/scripts/neural_mpc/video_overlay
source /opt/ros/*/setup.bash          # required for `import rosbag`

# 1. what was actually commanded, and when
python3 -c "
import sys; sys.path.insert(0,'.')
import bag_calib as B
ref = B.read_ref_traj('FLIGHT.bag')
for s in B.segment_reference(ref):
    print(f'[{s[0,0]:7.2f},{s[-1,0]:7.2f}]s  x[{s[:,1].min():+.2f},{s[:,1].max():+.2f}] '
          f'y[{s[:,2].min():+.2f},{s[:,2].max():+.2f}] z[{s[:,3].min():+.2f},{s[:,3].max():+.2f}]')"

# 2. calibrate  (~7.5 min at 4K: 138 s free track + 3 guided passes)
python3 project_reference.py calibrate-bag --video V.mp4 --bag B.bag --out calib.json \
    --hsv-smin 70 --hsv-vmin 40 --min-area 600 --guided-iters 3 --debug-track dbg.mp4

# 3. render -- burned into the video
python3 project_reference.py render --video V.mp4 --calib calib.json --out out.mp4 \
    --ref-from-bag B.bag --ref-window T0 T1 --start A --end B --scale 0.5 \
    --floor-z 0 --thickness 9 --droppers 12

# 3b. or export the reference alone on alpha, to composite by hand
python3 project_reference.py overlay --calib calib.json --out ref.png \
    --ref-from-bag B.bag --ref-window T0 T1

# 3c. setpoint runs: one wireframe PNG per commanded pose (position AND attitude)
python3 project_reference.py pose-overlay --calib calib.json --bag B.bag \
    --video-window T0 T1 --min-hold 4.0 --out pose.png
```

Placeholders above (`V.mp4`, `B.bag`) are placeholders -- pass real paths. Input
paths are validated before any work starts, so a typo prints one clear line rather
than a rosbag traceback.

**Estimate the time offset from the filenames before running anything.** Bag names
are UTC (`YYYY-MM-DD-HH-MM-SS`), Pixel names are UTC (`PXL_YYYYMMDD_HHMMSSmmm`).
`offset ≈ video_start − bag_start`. It was off by 1.7 s here (phone vs ROS clock),
so use it as a sanity bound, not a value — but it instantly tells you if the
solver has landed somewhere absurd.

---

## 2. Environment

| | |
|---|---|
| workspace | `/home/jojo_ws` (catkin, `devel/setup.bash`); repo `src/jsk_aerial_robot` |
| ROS | needs `source /opt/ros/*/setup.bash` in every shell before `import rosbag` |
| python | 3.10.12 · cv2 **4.5.4** · numpy 1.24.4 · scipy 1.15.3 |
| missing | **no ffmpeg, no ffprobe, no exiftool** — do not plan around them |
| video I/O | cv2 VideoWriter works with `mp4v`, `avc1`, `MJPG`, `XVID`; reading 4K is ~13 ms/frame |
| bags | `/home/jojo_ws/rosbags/` (~1.8 GB) |

**Bash tool timeout is 120 s by default, 600 s max.** Anything longer: `nohup … &`
then a separate `run_in_background` call with `until grep -q DONE log; do sleep 10; done`.
Chained `sleep N; cmd` in the foreground is blocked by the harness.

`cv2.CAP_PROP_POS_MSEC` returns **0** for the read past EOF — drop the last sample
before computing frame timing. (Checked here: the video is constant-rate, ±13 ms
jitter, no drift. Don't re-investigate VFR for Pixel footage.)

---

## 3. Robot / system facts (do not re-derive)

Source: `robots/beetle_omni/config/PhysParamBeetleOmniJetson.yaml`,
`robots/beetle_omni/urdf/beetle_omni_jetson.urdf.xacro`,
`aerial_robot_base/launch/external_module/mocap.launch`.

- Robot namespace `beetle1`; tilting omnidirectional quad, `beetle_omni_jetson`.
- Rotors at (±0.194824, ±0.194652/−0.195008, −0.00368) in the **CoG** frame.
  Adjacent spacing **0.3897 m**, diagonal **0.5511 m**. Mass 3.146 kg.
- `ball_effector_p = [0, 0, 0.264]` — `ee_contact` is the **centre of the red ball**,
  0.264 m above the CoG (measured 0.2645 in the bag; the yaml is right).
- Ball physical radius ≈ **0.041 m** (recovered from the tracker + mocap depth).
- Mocap: OptiTrack via `mocap_optitrack`, `parent_frame_id: world`,
  `child_frame_id: baselink`. **z = 0 is the floor.**
- Landed heights, March 2026 bag: cog 0.0399, ee_contact 0.3042, baselink 0.0885 m.
  July 2026 bags show baselink ≈ 0.18 — the legs changed. **Do not assume a leg
  height across sessions; read it from the bag.**
- Topics that matter:
  `/beetle1/set_ref_traj` (MultiDOFJointTrajectory; `points[0]` is the current ref),
  `/beetle1/uav/cog/odom`, `/beetle1/uav/ee_contact/odom`,
  `/beetle1/uav/baselink/odom`, `/beetle1/mocap/pose`, `/beetle1/flight_state`.
- `flight_state` transitions: `2→3` takeoff, `3→5` hover/track, `5→6→0` land.
- `pub_mpc_base.py` subscribes to `ee_contact/odom` when present, but that is only
  for the error printout — **what the controller tracks is decided in the C++ node.**
  Determine it empirically: compare `cog/odom` and `ee_contact/odom` against
  `set_ref_traj` over the segment and take whichever has the lower RMSE.
- `CircleTraj.z` history: **0.47 m** (= 0.2 + 0.27, CoG era, ≤ ~2026-04) →
  **0.73 m** (+0.26 for EE tracking, current). Always read from the bag.

---

## 4. Solved: PXL_20260329_094920628.mp4

Calibration checked in as `calib_PXL_20260329_094920628.json`. Re-use it directly
if the same clip comes up again — do not recalibrate.

| | |
|---|---|
| video | `/home/jojo_ws/rosbags/PXL_20260329_094920628.mp4` — 3840×2160, 29.9951 fps, 5014 frames, 167.16 s |
| bag | `/home/jojo_ws/rosbags/2026-03-29-09-49-16_mode_11_model_214_success.bag` — start epoch 1774777756.566 (2026-03-29T09:49:16.566Z), 160.93 s |
| offset | **t_bag = t_video + 2.40 s** (filenames imply 4.06 → phone/ROS clocks differ ~1.7 s) |
| camera | f = 1872 px, pp (1886, 1131), k1 +0.0373, k2 −0.0252, at (−0.172, −1.611, +1.109) m in mocap; HFOV 91.5° |
| accuracy | 4.35 px median / 2351 pts · 4.14 px on the circle window · **7.43 px hold-out** |
| reproducibility | independent CLI re-run: f 1858, camera within 1.6 cm, circle within 5.1 px median |
| HSV that works | `s_min 70, v_min 40, h_lo 12, h_hi 166, min_area 600` (at 4K) |

Reference segments in that bag (bag seconds):

| window | what | note |
|---|---|---|
| 55.16–57.20 | hold at (1, 0, 0.47) | circle entry point |
| **69.50–79.50** | **CircleTraj** r = 1.00000, centre (0,0), **z = 0.47000**, CCW | exactly one 10 s lap; = video 67.10–77.10 |
| 85.63–108.29 | LemniscateTraj | x ±1, y ±0.5, z 0.37–0.97 |
| 113.64–144.15 | **SetPointTraj**, four plateaus — see below | x ±0.3, y 0–0.2, z 0.7–1.2 |

SetPointTraj plateaus (bag s; video = bag − 2.40). It commands **attitude too**,
which is the whole point of the manoeuvre:

| bag window | CoG position | roll / pitch / yaw |
|---|---|---|
| 113.64–123.07 | (0, 0, 0.70) | 0° / 0° / 0° |
| **123.09–131.07** | **(+0.300, +0.200, +1.200)** | **+28.6° / 0° / +17.2°** |
| **131.09–139.08** | **(−0.300, 0, +1.000)** | **+28.6° / +28.6° / −17.2°** |
| 139.10–144.15 | (0, 0, 0.70) | 0° / 0° / 0° |

Note the roll carries into the third plateau: `SetPointTraj.get_3d_orientation`
sets `roll` in the `24 > t > 8` branch and never resets it, so the `24 > t > 16`
branch adds pitch and yaw on top. The two bolded poses are what
`--video-window 118 138 --min-hold 4.0` selects.

CoG-vs-reference RMSE over the circle **0.158 m**, ball 0.347 m → **CoG was tracked**.

The calibration JSON now carries `notes.time_offset_video_to_bag_s = 2.40`, so
`--video-window` resolves video seconds without restating the offset.

Outputs in `/home/jojo_ws/rosbags/`: `PXL_20260329_094920628_circle_ref_1080p.mp4`,
`…_circle_ref_frame.png`, `…_reference_overlay.png` (the circle alone, transparent),
`…_setpoint_pose1.png` / `…_setpoint_pose2.png` (the two commanded setpoint poses as
dashed wireframes, transparent).

---

## 5. Code map and invariants

| file | holds |
|---|---|
| `geometry.py` | `load_robot_geometry`, `drone_wireframe`, `quat_to_rot`, `rot_to_rpy`, `Calibration`, `focal_from_homography`, `pose_from_plane_homography`, `calibrate_from_ground_points`, `calibrate_from_correspondences`, **`refine_full`** (robust bundle adjust — the workhorse), `circle_reference` |
| `bag_calib.py` | `track_red_ball`, **`retrack_guided`**, `read_ee_track`, `read_ref_traj`, `read_ref_poses`, `segment_reference`, `segment_poses`, `align_and_calibrate`, `off_orbit_weights`, `decimate_static`, **`calibrate_iterative`** |
| `render.py` | **STYLE block at the top — the single place for colour/thickness/dash**; `project_polyline` (splits at the horizon), `draw_reference`, `render_reference_rgba` (transparent PNG), `_dash`, `_cap_extension`, `draw_floor_grid`, `draw_world_axes` |
| `project_reference.py` | CLI: `frame picker calibrate calibrate-bag measure overlay pose-overlay preview render` |
| `picker.py` | self-contained HTML click tool (works over remote VS Code; cv2 GUI does not) |

**Invariants — these are fixed bugs, do not reintroduce:**

1. `focal_from_homography` must solve the IAC constraints as a **homogeneous 2×2
   SVD null space**. Do *not* divide by `c1*c2`, and do *not* row-normalise the
   system. When the camera has no roll `c1` is exactly 0, the first constraint
   collapses to 0 = 0, and either treatment promotes round-off to a full-strength
   equation. Symptom: 24% focal error on *exact* synthetic data.
2. `align_and_calibrate` needs the overlap fraction and 3D-spread gates. Without
   them the offset search locks onto a 1-second sliver where the robot descends on
   the spot — collinear points, zero residual, completely wrong offset.
3. `off_orbit_weights` is required whenever the flight is circle-dominated; plain
   reprojection error is worse than useless there (true offset scored 2.8 px,
   wrong offsets 1.0 px).
4. `decimate_static` before any fit that includes pre-takeoff frames.
5. `build_reference` / track overlays **must stay cached**. They are called once
   per frame by `render`; re-reading a 200 MB bag 5000 times turned a 36 s render
   into hours. `_REF_CACHE` / `_TRACK_CACHE` in `project_reference.py`.
6. `render` refuses to run if the frame size disagrees with the calibration —
   that check catches a mismatched `--rotate`.
7. **Transparent PNG: never draw anti-aliased strokes onto a BGRA canvas.** Build
   the alpha as a separate single-channel mask and fill BGR uniformly, otherwise
   every edge pixel blends toward black and fringes on compositing. `_compose` also
   fills the colour under fully transparent pixels, which is what makes `--scale`
   safe. `test_overlay.py` asserts both the good behaviour and that the naive
   version really would fringe.
8. **Dash lengths must be cap-compensated.** OpenCV paints a thick AA stroke
   `2*ceil(t/2)+1` px longer than its endpoints (measured for t = 2…12). Without
   `_cap_extension`, a 16/13 pattern at thickness 6 renders 23 on / 5.5 off.
9. **Closed-loop dashes must be tiled to a whole number of periods, not marched
   from arc length 0.** `on+off` essentially never divides a circle's projected
   circumference evenly, so fixed-step placement leaves a leftover sliver where
   the loop closes on itself -- two dashes butt together right at the seam (the
   bug the user reported). Fixed by detecting `seg[0] == seg[-1]` (bit-exact --
   `project_polyline` duplicates the point when it closes the curve) and
   rescaling `on`/`off` by one small factor so `n = round(circumference/period)`
   whole periods tile the loop exactly. `_resample_wrapped` handles a dash that
   straddles the seam by interpolating mod the circumference; `np.interp` only
   needs its reference axis sorted, not the query points, so this just works.
   9 tests in `test_overlay.py` cover it, including 5 on/off/thickness
   combinations and a check that open (camera-occluded) arcs are untouched.
10. **Pose wireframe = 4 arms (single lines) + 4 propellers + mast + ball.**
    Rotor positions and `ball_effector_p` from `PhysParamBeetleOmniJetson.yaml`
    (with a built-in fallback a test checks still agrees). Two numbers are *not*
    in any config file and live as constants in `geometry.py`:
    `BALL_RADIUS_M = 0.041` (from the calibration) and `PROP_DIAMETER_M` = 9 in,
    as specified by the user. It draws the **CoG** pose with untilted rotors,
    because `set_ref_traj` commands a body pose, not servo angles. The rotor
    square is off by default (`--square` brings it back).
    Arm *width* is not in the yaml either; measured off this footage it is
    **19–30 mm** (spread from wiring, silver joints and the safety cord). Arms
    were briefly drawn as twin rails 25 mm apart and then reverted to single
    lines on request — keep the measurement here in case it is wanted again.
    Careful: `_circle_3d` duplicates its closing point on purpose — the closed-loop
    dash tiling keys off `seg[0] == seg[-1]` — so a naive centroid of a circle is
    biased by r/n. Drop the last point before averaging.
11. **Deduplicate poses before counting them.** A window clips a pose at each
    edge and a trajectory usually revisits its neutral pose, so raw plateaus
    over-count. `_distinct_poses` merges by position + quaternion (handling the
    `q`/`−q` double cover) and keeps `hold` as the *longest single* visit, never
    the sum — otherwise two clipped fragments masquerade as one long hold.
12. Style defaults live in `render.py`'s STYLE block and the argparse defaults are
    `None` on purpose. Putting a literal back into `add_style_args` would silently
    override the block — there is a test asserting the `default=None`.

**Tests**: `python3 test_geometry.py` (22), `python3 test_overlay.py` (35),
`python3 test_pose_overlay.py` (35 — needs ROS sourced for its synthetic-bag CLI
test) and `python3 test_end_to_end.py` (12, ~2 min, renders a synthetic 1350-frame
video). Run all four after any change to `geometry.py`, `render.py` or
`bag_calib.py`. **104 checks total.**

---

## 6. Timings at 4K (5014 frames)

All measured on this box (4K, 5014 frames), not estimated:

| step | wall time |
|---|---|
| free `track_red_ball`, stride 1 | **138 s** |
| `retrack_guided`, one pass | **~74 s** — same at stride 1 or 2; 4K *decoding* dominates, not the per-frame work, so there is little to gain from striding |
| `calibrate-bag`, cached track + 3 guided iters | **~5 min** |
| `calibrate-bag` from scratch | **~7.5 min** |
| offset scan, subsampled ×10–12 | ~40 s |
| `render`, 420 frames 4K→1080p | **36 s** |

Cache the pixel track with `--track-npy`; it is the expensive part and is reusable
across calibration experiments.

---

## 7. Do not re-attempt

- **Deriving the focal length from the carpet floor.** Four methods tried (Hough
  vanishing points, FFT, autocorrelation, synthetic-checkerboard correlation,
  scanline edge back-projection); all inconclusive. Root cause: candidate
  calibrations are all fitted to the same mocap, so they agree over the region the
  robot flew, and the floor is inside it — a 33% change in f moves the measured
  floor scale by 5%. See README "Dead ends".
- **The rotor-square projection check** as a scale arbiter: the red blob centroid
  sits inboard of the rotor axis (the arm is red too), so it always reads small.
- **VFR investigation for Pixel footage** — measured constant-rate, no drift.

---

## 8. Still open

- HFOV 91.5° is wide for a phone main camera. The fit prefers it strongly
  (forcing 2500 px doubles the error) and the projection is validated regardless,
  but if a checkerboard calibration of that phone ever becomes available, plug it
  in via `--focal-px`/`--k1` and re-check. Ask the user which lens was used.
- The circle's near arc falls outside the frame (camera only 1.61 m from the
  origin, circle radius 1 m). Geometrically unavoidable for this shot.
- Not rendered yet: full 167 s clip, 4K version, `--track-overlay` variant showing
  the measured CoG alongside the reference. All one command each; user was asked
  and had not answered.
- `calibrate-bag` assumes a **static camera**. Handheld footage would need
  per-frame pose tracking — not implemented.
