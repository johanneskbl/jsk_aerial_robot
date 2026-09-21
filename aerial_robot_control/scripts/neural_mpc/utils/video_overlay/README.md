# Projecting the MPC reference into a video recording

Draws the reference trajectory commanded by
`aerial_robot_planning/scripts/trajs.py` onto footage of the flight, for the
paper video.

## What the reference actually is — read it from the bag

`CircleTraj` is a circle of radius `r` about the world z axis, centred on the
mocap origin, traversed counter-clockwise from `(+r, 0)`:

```python
x = r*cos(w t),  y = r*sin(w t),  z = const
```

**Its height has changed over time, so do not assume it — read it from the bag.**
In `trajs.py` today `z = 0.2 + 0.27 + 0.26 = 0.73 m`, where the last term appeared
when the stack switched to tracking the ball end-effector (`ee_contact`, which
`PhysParamBeetleOmniJetson.yaml` puts `0.264 m` above the CoG). Earlier recordings
tracked the **CoG** and used `z = 0.2 + 0.27 = 0.47 m`.

`/<robot>/set_ref_traj` records what the controller was actually given, so
`--ref-from-bag BAG --ref-window T0 T1` draws the truth rather than a
reconstruction. `--ref-window` also picks one segment out of a session containing
several trajectories; run it without a window and the tool lists the segments it
found.

The `world` frame is OptiTrack's, via `mocap_optitrack`
(`aerial_robot_base/launch/external_module/mocap.launch`, `parent_frame_id: world`).
On this lab's setup **z = 0 is the floor** — verified here by rectifying the floor
to a metric top-down view and recovering ~0.5 m carpet tiles.

## Calibrating from the flight rosbag (the accurate route)

```bash
./project_reference.py calibrate-bag \
    --video flight.mp4 --bag flight.bag --out calib.json \
    --hsv-smin 70 --hsv-vmin 40 --min-area 600 \
    --guided-iters 3 --debug-track track_debug.mp4
```

It segments the red ball in every frame, pairs it with
`/<robot>/uav/ee_contact/odom`, recovers the video↔bag time offset, and
bundle-adjusts the camera. Mocap supplies the origin, the axes, the floor height
and the scale, so nothing has to be assumed about the room.

Use the ball (`ee_contact`) for the *correspondences* even when the controller was
tracking the CoG — the ball is the thing you can see, and mocap knows where it was.

### The four things that made this work

Each of these was a real failure on the real footage, not a hypothetical.

1. **The free tracker cannot tell the ball from the robot's own motor pods.** They
   are the same red, and bigger. Roughly half the airborne detections landed on a
   pod, which capped the fit at ~10 px median error. `--guided-iters` fixes it:
   mocap already says where the ball is to within ~100 px and how far away it is,
   which fixes its expected apparent radius as `f * r_ball / depth`. Searching only
   near the prediction, for a blob of about the right size, removes the ambiguity.
   The pods are both offset from the prediction and the wrong size.

2. **A pinhole model is not enough.** Residuals grew from +2.9 px at the image
   centre to +11.4 px at r≈1000 px — a radial signature. Fitting `k1`/`k2` halved
   the median error. `--free` selects the parameter groups; the default
   `rvec tvec f pp k1 k2` is what this footage needed.

3. **Frames where the robot sits still swamp the fit.** Before takeoff, ~1200
   frames measure the *same* 3D point. Statistically that is one observation, but
   a plain least squares counts it as 1200 constraints and lets that single image
   location outvote the whole flight. `decimate_static` thins those stretches.

4. **A constant-height circle is invariant to rotation about world z**, so on a
   pure-circle flight the time offset is weakly observable and plain reprojection
   error is actively misleading — on the synthetic scene the true offset scored
   2.8 px while wrong offsets scored 1.0 px. The alignment therefore weights the
   samples that *break* the symmetry (takeoff, landing, transit); see
   `off_orbit_weights`. Rival offsets are reported in `notes.ambiguous_offsets_s`.
   A wrong world yaw does **not** move the drawn circle, only the axes overlay and
   the start marker.

Useful flags: `--offset` when you already know the time offset (video and bag
filenames usually give it to within a second or two); `--rotate {0,90,180,270}`
because phones store an orientation flag OpenCV ignores; `--roi X Y W H` to
exclude something red in the room.

## Calibrating from one frame (no rosbag)

```bash
./project_reference.py frame  --video flight.mp4 --out frame0.png
./project_reference.py picker --frame frame0.png --out picker.html
#   open picker.html, click floor landmarks, type their world (X, Y), save points.json
./project_reference.py calibrate --points points.json --frame frame0.png \
                                 --out calib.json --check check.png --floor-z 0
```

The carpet tiles are the metric reference. Four points is the minimum; 8–12 spread
over the whole floor is much better. The focal length comes from the two
constraints a plane puts on the image of the absolute conic — pass `--hfov-deg` or
`--focal-px` instead if the camera is known. **Always look at `check.png`**: it
overlays the projected 0.5 m world grid, and if that does not sit on the real tile
edges nothing downstream is trustworthy.

## Rendering

```bash
./project_reference.py render --video flight.mp4 --calib calib.json \
    --out out.mp4 --ref-from-bag flight.bag --ref-window 69.5 79.5 \
    --start 65.5 --end 79.5 --scale 0.5 --floor-z 0 --thickness 9 --droppers 12
```

`--floor-z` enables the dashed ground shadow and `--droppers` the vertical height
cues. Keep them: without them a circle at 0.47 m and a circle painted on the floor
look identical in a 2D video. Other options: `--color`, `--thickness`, `--axes`,
`--grid`, `--hud`, `--track-overlay BAG` (also draw the measured CoG),
`--start/--end` to trim, `--scale` to downsample 4K to 1080p.

OpenCV drops the audio track; `render` prints the ffmpeg line that muxes it back.

## Exporting the reference alone, as a transparent PNG

For compositing by hand in an editor, rather than burning the overlay in:

For this project's recording, copy-pasteable as-is:

```bash
./project_reference.py overlay \
    --calib calib_PXL_20260329_094920628.json \
    --out reference_overlay.png \
    --ref-from-bag /home/jojo_ws/rosbags/2026-03-29-09-49-16_mode_11_model_214_success.bag \
    --ref-window 69.5 79.5
```

`--ref-from-bag` needs the real path to the bag; a bare filename is only found if you
happen to be standing in that directory. Every input path is checked up front, so a
wrong one prints one line naming the flag and lists the bags it can see.

Just the curve on alpha — no video frame, no robot. The camera is static, so **one
PNG serves the whole clip**: drop it on the footage at 1:1 with no offset, normal
("over") blending. `--scale 0.5` emits 1080p from a 4K calibration; `--size W H`
overrides the output size. It refuses to write anything but `.png`, since no other
format here keeps the alpha.

Add `--floor-z 0` if you also want the dashed ground shadow and the height
droppers in the same PNG; by default it is the reference path and nothing else.

### Setpoint trajectories: one image per commanded pose

`SetPointTraj` is not a path but a staircase of held poses, and it commands
**attitude as well as position** — so each pose gets its own PNG showing the
airframe as a dashed wireframe at the commanded CoG position and orientation:

```bash
./project_reference.py pose-overlay \
    --calib calib_PXL_20260329_094920628.json \
    --bag /home/jojo_ws/rosbags/2026-03-29-09-49-16_mode_11_model_214_success.bag \
    --video-window 118 138 --min-hold 4.0 \
    --out setpoint_reference.png
```

`--video-window` takes **video** seconds and converts them with the offset stored
in the calibration (`notes.time_offset_video_to_bag_s`); `--window` takes bag
seconds directly. The command prints every distinct pose it found and writes
`_pose1`, `_pose2`, … — one file per pose, or exactly `--out` when there is only
one. `--pose-index N …` renders a chosen subset.

`--min-hold` is the knob that matters. A time window almost always clips a pose at
each edge, and a trajectory that returns to its neutral pose visits the *same* pose
twice. Poses are deduplicated (quaternion double cover included: `q` and `−q` are
one rotation), `hold` is the longest single visit rather than the sum, and anything
held for less than `--min-hold` is reported but skipped. On this recording,
`--min-hold 4.0` cleanly keeps the two 8 s setpoints and drops the clipped hover
fragments at both ends.

The wireframe draws the arms, the propellers, the mast and the ball:

| part | geometry | source |
|---|---|---|
| arms | one line from the frame centre to each rotor | endpoints `p1..p4` from `PhysParamBeetleOmniJetson.yaml` |
| propellers | a circle at each rotor, in the rotor plane | 9 in diameter = 0.2286 m |
| mast + ball | centre up to the ball, then the ball's silhouette | `ball_effector_p` from the same yaml; 0.041 m radius measured during calibration |

The ball is drawn as a true silhouette — a circle in the plane facing the camera.
Propellers are drawn untilted: the reference commands a body pose, not servo angles.
Everything is built in body coordinates and then rigidly transformed, so the figure
cannot be distorted by the pose.

`--no-arms`, `--no-props`, `--no-mast`, `--no-ball` drop parts; `--square` adds the
rotor-plane outline back (off by default); `--prop-diameter-in` overrides the 9 in.

Note this draws the **CoG** pose, because that is what `set_ref_traj` commands and
what the controller tracked on this recording. Rotors are drawn untilted: the
reference commands a body pose, not servo angles.

### Changing the colour

One place, `render.py`, top of file:

```python
REFERENCE_COLOR = "#3C3C3C"      # dark gray -- shared by everything
REFERENCE_THICKNESS_PX = 6       # trajectories
REFERENCE_DASH_ON_PX = 16
REFERENCE_DASH_OFF_PX = 13

POSE_THICKNESS_PX = 8            # pose wireframes
POSE_DASH_ON_PX = 24
POSE_DASH_OFF_PX = 15
```

Every command — `overlay`, `pose-overlay`, `render`, `preview` — reads these, so
editing that block is enough. The colour is shared; only the line weight and dash
pitch are split, because a pose wireframe is a far smaller figure than a whole
trajectory (an arm is ~360 px where the circle is ~4000 px around, so the
trajectory's pitch would put barely three dashes on an arm). Set the `POSE_*`
values equal to the `REFERENCE_*` ones for identical styling. The matching CLI flags (`--color`, `--thickness`, `--dash`, `--solid`,
`--halo`) all default to *unset* rather than to a literal, precisely so they cannot
silently shadow the block.

Two details that are easy to get wrong and are handled here:

- **Alpha is built as its own coverage mask, and the colour channels are filled
  uniformly.** Drawing anti-aliased strokes straight onto a transparent BGRA canvas
  blends colour toward the background as well as alpha, so every edge pixel comes
  out part-black and the line picks up a dark fringe the moment it is composited.
  A test asserts that *every* pixel in the output carries exactly the configured
  colour, and a second test confirms the naive approach really would fringe, so the
  guard cannot rot. It also makes `--scale` safe: interpolating uniform colour
  channels cannot pull in a foreign colour.
- **The dash pattern describes what you see.** OpenCV paints a thick stroke
  `2*ceil(t/2)+1` px longer than its endpoints, so an uncompensated 16/13 pattern
  at 6 px renders as 23 on / 5.5 off and stops reading as dashed. Each dash is
  shortened by that amount first; measured output is 16.5 on / 12.2 off.
- **A closed loop is tiled with a whole number of dashes, so the seam looks like
  every other gap.** `on + off` essentially never divides a circle's projected
  circumference evenly; marching from arc length 0 in fixed steps leaves a
  leftover sliver exactly where the curve closes on itself, so two dashes end up
  butting together there. `on`/`off` are rescaled by one small factor so a whole
  number of periods spans the loop exactly, no remainder anywhere -- including
  the dash that straddles the seam.

The burned-in `render` path uses the same sub-pixel rasteriser as the PNG (IoU 1.0
between them in the tests), so the two agree pixel for pixel.

## Checking the metric assumptions

The robot is a ruler. `PhysParamBeetleOmniJetson.yaml` puts the rotors at
±0.1948 m in x and y, so adjacent motor centres are 0.3896 m apart and diagonal
ones 0.5510 m:

```bash
./project_reference.py measure --calib calib.json --plane-z 0.35 --points 812 903 1044 878
```

## Result for PXL_20260329_094920628.mp4

Calibration stored in `calib_PXL_20260329_094920628.json`.

| | |
|---|---|
| video | 3840×2160, 29.9951 fps, 5014 frames (167.2 s) |
| bag | `2026-03-29-09-49-16_mode_11_model_214_success.bag`, 160.9 s |
| time offset | `t_bag = t_video + 2.40 s` (filenames suggested 4.06 s; the fit is sharp to ±0.02 s) |
| circle segment | bag 69.50–79.50 s = video 67.10–77.10 s, exactly one 10 s lap |
| reference | r = 1.00000 m about (0, 0), **z = 0.47000 m**, CCW — CoG, as confirmed by CoG-vs-reference RMSE 0.158 m against 0.347 m for the ball |
| camera | f = 1872 px (HFOV 91.5°), k1 = +0.037, k2 = −0.025, at (−0.17, −1.61, +1.11) m in the mocap frame |
| accuracy | 4.35 px median over 2351 correspondences; **4.14 px on the circle window**; 7.43 px on hold-out (fit without the circle, tested on it) — ≈1.3 cm at the robot's distance |
| reproducibility | a second, independent run of the command below landed on f = 1858 px, camera within 1.6 cm, offset 2.412 s, and put the drawn circle within **5.1 px median** of the first run |

The focal length of 1872 px (HFOV 91.5°) is wide for a phone, so it was checked
three ways: forcing f = 2500 px more than doubles the median error (4.35 -> 10.09 px);
rectifying the floor to a metric top-down view yields carpet tiles of ~0.45-0.55 m
against a nominal 0.5 m; and the robot's known 0.39 m rotor square projects onto
the motor pods more closely at 1872 px than at 2500 px. The last two checks are
consistent but not sharp — they cannot separate f on their own, because both
calibrations are fitted to the same mocap and so agree closely near the region the
robot actually flew. What is well established is the projection itself, which is
what the overlay depends on.

### Commands used

```bash
./project_reference.py calibrate-bag \
    --video PXL_20260329_094920628.mp4 \
    --bag  2026-03-29-09-49-16_mode_11_model_214_success.bag \
    --out  calib.json --hsv-smin 70 --hsv-vmin 40 --min-area 600 --guided-iters 3

./project_reference.py render \
    --video PXL_20260329_094920628.mp4 --calib calib.json \
    --out circle_ref_1080p.mp4 \
    --ref-from-bag 2026-03-29-09-49-16_mode_11_model_214_success.bag \
    --ref-window 69.5 79.5 --start 65.5 --end 79.5 \
    --scale 0.5 --floor-z 0 --thickness 9 --droppers 12
```

Other reference segments in the same bag, if you want them: lemniscate at
85.63–108.29 s, setpoints at 113.64–144.15 s.

## Insights worth carrying to the next job like this

**Read the commanded reference out of the bag, never re-derive it from source.**
`trajs.py` is live code. Between this March recording and September the circle's
height moved from 0.47 m to 0.73 m, because the stack switched from tracking the
CoG to tracking the ball. Re-deriving the curve from today's source would have put
the circle 26 cm too high with nothing on screen to reveal the mistake — the
overlay would still have looked plausible. `/set_ref_traj` is what the controller
was actually handed.

**The thing you can see is not the thing being controlled.** The controller
tracked the CoG; the only feature a tracker can lock onto is the ball, 0.264 m
above it. Both are needed and they play different roles: the ball supplies the
2D↔3D correspondences for calibration, the CoG defines what gets drawn. Confusing
them costs a quarter of a metre.

**Ask whether an unstable parameter actually moves the deliverable.** The focal
length wandered between 1870 and 2500 px across fits and resisted several attempts
to pin down independently. The question that ended the investigation was not "what
is f" but "how far apart do these calibrations put the projected circle" — 2.7 to
13.5 px among all the well-fitting variants, i.e. nothing. Time spent chasing a
parameter that does not move the output is time wasted.

**Hold-out is the only honest accuracy number.** Fitting everything and reporting
the residual measures how flexible the model is. Fitting with the circle segment
excluded and then testing on it (7.43 px) measures whether the overlay can be
trusted where it matters.

**Beware repeated measurements masquerading as independent ones.** 1200 frames of
a robot sitting still are one observation of one point, but least squares counts
them as 1200 constraints and lets that single image location outvote the entire
flight. The tell was a residual profile that was excellent (0.8 px) before takeoff
and terrible (10–96 px) after it.

**Build the geometry against synthetic ground truth first.** The 22-check suite was
written before any real footage was touched and immediately caught two genuine
bugs in the focal-length solver that would have been nearly impossible to
distinguish from bad data later. Everything real is ambiguous; only synthetic
scenes let you separate "my code is wrong" from "my data is hard".

## Dead ends, so they are not walked again

Four attempts to pin the focal length down from the floor, all inconclusive:

| attempt | outcome |
|---|---|
| Hough lines → orthogonal vanishing points | the receding tile-edge family is too low-contrast on grey carpet; 2 segments found against 149 for the other family |
| FFT / autocorrelation of the rectified floor | swamped by carpet texture and a monotonic low-frequency component |
| correlating a synthetic checkerboard against the rectified floor | best correlation only 0.17–0.20; period 0.42–0.45 m for *both* candidate focal lengths |
| back-projecting scanline intensity edges | texture edges outnumber tile edges; spacings biased low (0.43–0.45 m) |

The underlying reason is worth remembering: **the floor cannot arbitrate the focal
length here.** Both candidates were fitted to the same mocap data, so they agree
closely over the region the robot actually flew, and the floor lies mostly inside
that region. A 33% change in f moved the measured floor scale by only 5%.

The robot's known 0.39 m rotor square is likewise biased as a check: the red blob
centroid sits inboard of the rotor axis because the arm is red too, so the observed
square always measures smaller than the true one.

## How this was built

Roughly in this order, and the order mattered:

1. Establish the *semantics* first — what the reference is, which frame it is in,
   what tracks it — by reading the launch files, URDF, yaml and bag before writing
   any geometry.
2. Write the geometry and validate it against synthetic cameras with known
   answers. Two bugs fell out immediately.
3. Build a synthetic lab scene — checkerboard floor, red ball flying CircleTraj,
   known camera — and run the whole pipeline over it end to end, including the
   distractors (motor pods, a red chair) that the real scene contains.
4. Only then touch the real recording, and when it misbehaved, diagnose by
   *decomposing the residuals* (against time, image radius, and image-space
   velocity) rather than by tuning parameters. That is what separated "outliers"
   from "missing distortion term" from "timing error" in one pass.
5. Verify against something the fit never saw: hold-out, the measured CoG track,
   the floor plane, and an independent re-run through the documented command.

## Tests

```bash
python3 test_geometry.py      # 22 checks against synthetic ground truth
python3 test_end_to_end.py    # renders a synthetic lab video, runs the whole pipeline
```

`test_end_to_end.py` builds a video of a ball flying CircleTraj over a 0.5 m
checkerboard from a known camera, then verifies that 12 simulated clicks
(0.7 px noise) place the drawn circle within 0.5 px of the ball, that the bag
route recovers a 17.35 s offset to 1 ms, and that the tracker still finds the ball
with four red motor pods and a red chair in frame.
