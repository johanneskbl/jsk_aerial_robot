"""Draw a mocap-frame reference trajectory onto video frames, or to a transparent PNG.

Handles the two things a naive `cv2.projectPoints` + `cv2.polylines` gets wrong:
points behind the camera (which project to plausible-looking garbage) and
polyline segments that straddle the horizon, which must be split rather than
connected across the image.
"""

from __future__ import annotations

import numpy as np
import cv2

from geometry import Calibration


# ======================================================================================
#  STYLE  --  the single place to change how the reference line looks.
#  Every command (`overlay`, `render`, `preview`) picks these up unless a CLI flag
#  explicitly overrides them, so editing here is enough.
# ======================================================================================

REFERENCE_COLOR = "#2FBA00"      # any "#RRGGBB"; e.g. "#3C3C3C" dark gray, "#FFDC3C" amber
REFERENCE_THICKNESS_PX = 12       # line width in pixels, at the video's own resolution
REFERENCE_DASH_ON_PX = 54        # dash length;  set REFERENCE_DASH to None for solid
REFERENCE_DASH_OFF_PX = 30       # gap length
REFERENCE_DASH = (REFERENCE_DASH_ON_PX, REFERENCE_DASH_OFF_PX)

# Optional contrast halo behind the line. Off by default: a transparent overlay is
# composited by hand, and a second colour would defeat "one place to set the colour".
REFERENCE_HALO = False
REFERENCE_HALO_COLOR = "#141414"
REFERENCE_HALO_EXTRA_PX = 4

# --- pose wireframe (the `pose-overlay` command) ------------------------------------
# A commanded *pose* is a much smaller figure than a whole trajectory: an arm is ~360 px
# where the circle was ~4000 px around, so the trajectory's dash pitch would put only
# three dashes on an arm. Same colour, finer pitch and lighter weight. Set these equal
# to the values above if you would rather both be styled identically.
POSE_THICKNESS_PX = 8
POSE_DASH_ON_PX = 54
POSE_DASH_OFF_PX = 35
POSE_DASH = (POSE_DASH_ON_PX, POSE_DASH_OFF_PX)

# The dash and thickness numbers above are in pixels and were chosen at 3840x2160.
# ======================================================================================


def hex_to_bgr(s):
    """'#RRGGBB' (or an already-BGR tuple) -> OpenCV BGR tuple."""
    if not isinstance(s, str):
        return tuple(int(v) for v in s)
    t = s.lstrip("#")
    if len(t) != 6:
        raise ValueError(f"colour must be '#RRGGBB', got {s!r}")
    return (int(t[4:6], 16), int(t[2:4], 16), int(t[0:2], 16))


# --------------------------------------------------------------------------------------
# Safe projection of world polylines
# --------------------------------------------------------------------------------------
def project_polyline(calib: Calibration, points_world: np.ndarray, closed: bool = False,
                     min_depth: float = 0.05, max_norm_radius: float = 8.0):
    """Project a world-space polyline into a list of drawable pixel segments.

    Splits the curve wherever it passes behind the camera or shoots off towards a
    vanishing point, so no segment is ever drawn across the image between two
    points that are not actually connected on screen.

    Returns a list of (M, 2) float arrays, each a contiguous run of pixels.
    """
    P = np.asarray(points_world, float).reshape(-1, 3)
    if closed and len(P) > 1 and not np.allclose(P[0], P[-1]):
        P = np.vstack([P, P[0]])

    depth = calib.depths(P)
    # Radius in normalised camera coordinates - guards against distortion blow-up
    # for points far outside the field of view.
    Pc = P @ calib.R.T + np.asarray(calib.tvec, float).reshape(1, 3)
    with np.errstate(divide="ignore", invalid="ignore"):
        rn = np.hypot(Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2])
    good = (depth > min_depth) & np.isfinite(rn) & (rn < max_norm_radius)
    if not good.any():
        return []

    uv = calib.project(P)
    good &= np.isfinite(uv).all(axis=1)

    segments, cur = [], []
    for i in range(len(P)):
        if good[i]:
            cur.append(uv[i])
        elif cur:
            segments.append(np.array(cur)); cur = []
    if cur:
        segments.append(np.array(cur))
    return [s for s in segments if len(s) >= 2]


def draw_world_polyline(img, calib: Calibration, points_world, color=(60, 220, 255),
                        thickness=4, closed=False, halo=True, halo_color=(20, 20, 20),
                        halo_extra=4, alpha=1.0, dash=None):
    """Draw a world-space polyline with an optional dark halo for contrast.

    `dash=(on, off)` in pixels renders a dashed line (used for the ground shadow).
    """
    segs = project_polyline(calib, points_world, closed=closed)
    if not segs:
        return img
    if dash is not None:
        cap = _cap_extension(thickness)
        segs = [d for s in segs for d in _dash(s, dash[0], dash[1], cap=cap)]

    layer = img if alpha >= 1.0 else img.copy()
    # Sub-pixel endpoints, exactly as the transparent-PNG path does, so the burned-in
    # overlay and the exported PNG rasterise to the same pixels.
    k = 1 << _SHIFT
    if halo:
        for s in segs:
            cv2.polylines(layer, [np.round(np.asarray(s, float) * k).astype(np.int32)],
                          False, halo_color, max(1, int(thickness + halo_extra)),
                          cv2.LINE_AA, shift=_SHIFT)
    for s in segs:
        cv2.polylines(layer, [np.round(np.asarray(s, float) * k).astype(np.int32)],
                      False, color, max(1, int(thickness)), cv2.LINE_AA, shift=_SHIFT)
    if alpha < 1.0:
        cv2.addWeighted(layer, alpha, img, 1.0 - alpha, 0.0, dst=img)
    return img


def _cap_extension(thickness: float) -> float:
    """How much longer than its endpoints OpenCV actually paints a thick AA stroke.

    Measured, not assumed: a stroke of width t overshoots each end by
    ceil(t/2) + 0.5 px, so a dash is drawn `2*ceil(t/2) + 1` px longer than asked.
    Left uncompensated, a 16/13 pattern at thickness 6 renders as 23 on / 5.5 off --
    the gaps nearly close and the line stops reading as dashed.
    """
    t = max(1.0, float(thickness))
    return 2.0 * np.ceil(t / 2.0) + 1.0


def _dash(seg: np.ndarray, on: float, off: float, phase: float = 0.0, cap: float = 0.0):
    """Split a pixel polyline into dashes of `on` px separated by `off` px.

    Dashes are laid out by arc length in *image* space, which is what makes a
    projected circle look evenly dashed: spacing the dashes evenly in world space
    instead would crowd them together at the far side of the curve.

    Duplicate consecutive points are dropped first -- `np.interp` needs a strictly
    increasing arc-length axis, and a projected polyline routinely contains repeats
    where the curve turns away from the camera.

    **Closed loops are detected and tiled exactly.** `on + off` almost never
    divides the loop's circumference evenly (a 1 m-radius circle is ~6.28 m
    around; 16+13 px does not divide that either, whatever it projects to), so
    naively marching from arc length 0 in fixed-size steps leaves a leftover
    sliver at the point where the curve closes on itself -- the last dash butts
    straight against the first with little or no gap between them. Detected via
    `seg[0] == seg[-1]` (exact: `project_polyline` duplicates the same point when
    it closes the curve, so this is bit-exact, not approximate), the number of
    (dash + gap) periods `n` is rounded to the nearest whole count that fits the
    loop, and `on`/`off` are both rescaled by the same small factor so that
    exactly `n` periods span the circumference with no remainder -- every dash
    and every gap the same length, including the one straddling the seam.
    """
    seg = np.asarray(seg, float).reshape(-1, 2)
    if len(seg) < 2:
        return []
    step = np.linalg.norm(np.diff(seg, axis=0), axis=1)
    keep = np.concatenate([[True], step > 1e-9])
    seg = seg[keep]
    if len(seg) < 2:
        return []
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))])
    total, period = float(s[-1]), float(on) + float(off)
    if total <= 0.0 or period <= 0.0 or on <= 0.0:
        return [seg]

    half = 0.5 * max(0.0, float(cap))       # trimmed off each end, see _cap_extension
    closed = float(np.hypot(seg[-1, 0] - seg[0, 0], seg[-1, 1] - seg[0, 1])) <= max(1e-6, 1e-6 * total)
    out = []

    def emit(a, b):
        a2, b2 = a + half, b - half
        if b2 <= a2:                        # dash shorter than the stroke is wide:
            mid = 0.5 * (a + b)             # degenerate to a dot at its centre
            a2 = b2 = mid if closed else min(max(mid, 0.0), total)
        if closed:
            # a2/b2 may run past `total`; _resample_wrapped takes that as wrapping
            # back through arc length 0, which is the same pixel as `total` here.
            out.append(_resample_wrapped(seg, s, total, a2, b2))
        else:
            out.append(_resample(seg, s, min(max(a2, 0.0), total), min(max(b2, 0.0), total)))

    if closed:
        n = max(1, int(round(total / period)))
        scale = (total / n) / period
        on_e, off_e = on * scale, off * scale
        per = on_e + off_e
        start0 = (-float(phase)) % per
        for i in range(n):
            a = start0 + i * per
            emit(a, a + on_e)
    else:
        start = -float(phase) % period
        if start > 0:                       # partial dash at the very beginning
            a, b = 0.0, min(max(0.0, start - off), total)
            if b > a:
                emit(a, b)
        while start < total:
            a, b = start, min(start + on, total)
            if b > a:
                emit(a, b)
            start += period
    return [d for d in out if len(d) >= 2]


def _resample_wrapped(seg, s, total, a, b):
    """Like `_resample`, but arc-length positions wrap modulo `total`.

    Used for a dash that straddles the seam of a closed loop. Valid because
    `s[0]` and `s[-1]` address the same pixel on a closed loop (`seg[0] ==
    seg[-1]`), so interpolating across the `total`/`0` boundary is continuous --
    `np.interp` only requires its reference x-axis (`s`) to be increasing, not
    the query points, so out-of-range-then-wrapped queries resolve correctly.
    """
    t = np.linspace(a, b, max(2, int((b - a) / 3.0) + 2))
    tm = np.mod(t, total)
    return np.column_stack([np.interp(tm, s, seg[:, 0]), np.interp(tm, s, seg[:, 1])])


def _resample(seg, s, a, b):
    """Points of `seg` between arc lengths a and b, densely enough to stay smooth."""
    t = np.linspace(a, b, max(2, int((b - a) / 3.0) + 2))
    return np.column_stack([np.interp(t, s, seg[:, 0]), np.interp(t, s, seg[:, 1])])


# --------------------------------------------------------------------------------------
# Transparent-PNG rendering
# --------------------------------------------------------------------------------------
_SHIFT = 4          # 1/16 px sub-pixel accuracy for line endpoints


def _draw_alpha(size, segments, thickness: float) -> np.ndarray:
    """Rasterise pixel-space polylines into an 8-bit coverage mask.

    The mask is built on its own rather than by drawing onto a BGRA canvas. Drawing
    anti-aliased strokes straight onto a transparent canvas blends *colour* toward
    the background as well as alpha, so every edge pixel comes out part-black and
    the line acquires a dark fringe the moment it is composited over footage.
    Keeping coverage separate and filling the colour channels uniformly gives
    correct straight (unassociated) alpha with no fringing.

    Sub-pixel endpoints come from OpenCV's `shift`, which -- unlike supersampling --
    costs nothing and leaves `thickness` in real pixels (verified, not assumed).
    """
    w, h = int(size[0]), int(size[1])
    mask = np.zeros((h, w), np.uint8)
    k = 1 << _SHIFT
    t = max(1, int(round(thickness)))
    for seg in segments:
        pts = np.round(np.asarray(seg, float) * k).astype(np.int32)
        if len(pts) >= 2:
            cv2.polylines(mask, [pts], False, 255, t, cv2.LINE_AA, shift=_SHIFT)
    return mask


def _layer(size, segments, color, thickness):
    """(alpha mask, BGR colour) for one stroke layer."""
    return _draw_alpha(size, segments, thickness), np.array(hex_to_bgr(color), float)


def _compose(size, layers):
    """Composite bottom-to-top straight-alpha layers into one BGRA image.

    `layers` is a list of (alpha_uint8, bgr_float) with the last drawn on top.
    """
    w, h = int(size[0]), int(size[1])
    out_a = np.zeros((h, w), np.float32)
    out_c = np.zeros((h, w, 3), np.float32)
    for a8, bgr in layers:                       # painter's order: bottom first
        a = a8.astype(np.float32) / 255.0
        src_c = np.broadcast_to(bgr.astype(np.float32), (h, w, 3))
        # standard "over": src on top of what is already accumulated
        new_a = a + out_a * (1.0 - a)
        num = src_c * a[..., None] + out_c * out_a[..., None] * (1.0 - a[..., None])
        with np.errstate(invalid="ignore", divide="ignore"):
            out_c = np.where(new_a[..., None] > 1e-6, num / np.maximum(new_a[..., None], 1e-6),
                             out_c)
        out_a = new_a
    bgra = np.zeros((h, w, 4), np.uint8)
    bgra[..., :3] = np.clip(np.round(out_c), 0, 255).astype(np.uint8)
    bgra[..., 3] = np.clip(np.round(out_a * 255.0), 0, 255).astype(np.uint8)
    # Colour is undefined where nothing was drawn; fill it with the topmost layer's
    # colour so that any later resize or filtering cannot pull black into the edges.
    if layers:
        empty = bgra[..., 3] == 0
        bgra[..., :3][empty] = np.clip(np.round(layers[-1][1]), 0, 255).astype(np.uint8)
    return bgra


def render_reference_rgba(size, calib: Calibration, points_world=None, color=None,
                          thickness=None, dash=..., closed=True, halo=None,
                          halo_color=None, halo_extra=None, extra_polylines=(),
                          polylines=None):
    """Render the reference geometry alone onto a transparent BGRA canvas.

    `size` is (width, height) and must match the video frame the PNG will be
    composited over. Defaults come from the STYLE block at the top of this module.

    Pass `dash=None` for a solid line; omit it to use `REFERENCE_DASH`.
    `extra_polylines` is a list of (points_world, closed) drawn in the same style.
    `polylines` supersedes `points_world`/`closed`/`extra_polylines` entirely and is
    the convenient form for a multi-part figure such as a drone wireframe.
    """
    color = REFERENCE_COLOR if color is None else color
    thickness = REFERENCE_THICKNESS_PX if thickness is None else thickness
    dash = REFERENCE_DASH if dash is ... else dash
    halo = REFERENCE_HALO if halo is None else halo
    halo_color = REFERENCE_HALO_COLOR if halo_color is None else halo_color
    halo_extra = REFERENCE_HALO_EXTRA_PX if halo_extra is None else halo_extra

    if polylines is None:
        if points_world is None:
            raise ValueError("pass either points_world or polylines")
        polylines = [(points_world, closed)] + list(extra_polylines)
    segs = []
    for pts, cl in polylines:
        segs += project_polyline(calib, pts, closed=cl)
    if dash is not None:
        cap = _cap_extension(thickness)
        segs = [d for sgm in segs for d in _dash(sgm, dash[0], dash[1], cap=cap)]

    layers = []
    if halo:
        layers.append(_layer(size, segs, halo_color, thickness + halo_extra))
    layers.append(_layer(size, segs, color, thickness))
    return _compose(size, layers)


def draw_marker_world(img, calib: Calibration, point_world, color=(60, 220, 255),
                      radius=9, thickness=-1, label=None, font_scale=0.8):
    P = np.asarray(point_world, float).reshape(1, 3)
    if calib.depths(P)[0] <= 0.05:
        return img
    uv = calib.project(P)[0]
    if not np.isfinite(uv).all():
        return img
    c = (int(round(uv[0])), int(round(uv[1])))
    cv2.circle(img, c, radius + 3, (20, 20, 20), -1, cv2.LINE_AA)
    cv2.circle(img, c, radius, color, thickness, cv2.LINE_AA)
    if label:
        cv2.putText(img, label, (c[0] + radius + 8, c[1] - radius - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (20, 20, 20), 4, cv2.LINE_AA)
        cv2.putText(img, label, (c[0] + radius + 8, c[1] - radius - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2, cv2.LINE_AA)
    return img


# --------------------------------------------------------------------------------------
# The reference overlay
# --------------------------------------------------------------------------------------
def draw_reference(img, calib: Calibration, ref_points: np.ndarray, floor_z: float | None = None,
                   color=None, thickness=None, shadow=True,
                   shadow_color=(150, 150, 150), droppers=8, start_marker=True,
                   dropper_color=(120, 200, 230), closed=True, dash=..., halo=None):
    """Draw the reference curve, plus optional ground shadow and vertical droppers.

    The shadow and droppers are what make the height of the reference legible in a
    2D video; without them a circle at 0.73 m and a circle painted on the floor
    look identical.

    Style defaults (colour, thickness, dash, halo) come from the STYLE block at the
    top of this module, so the burned-in overlay and the transparent PNG match.
    """
    color = hex_to_bgr(REFERENCE_COLOR) if color is None else hex_to_bgr(color)
    thickness = REFERENCE_THICKNESS_PX if thickness is None else thickness
    dash = REFERENCE_DASH if dash is ... else dash
    halo = REFERENCE_HALO if halo is None else halo
    ref = np.asarray(ref_points, float).reshape(-1, 3)

    if shadow and floor_z is not None:
        gnd = ref.copy(); gnd[:, 2] = floor_z
        draw_world_polyline(img, calib, gnd, color=shadow_color,
                            thickness=max(1, thickness - 2), closed=closed, alpha=0.55,
                            dash=(14, 12))
        if droppers:
            idx = np.linspace(0, len(ref) - 1, int(droppers), endpoint=False).astype(int)
            for i in idx:
                seg = np.array([ref[i], [ref[i, 0], ref[i, 1], floor_z]])
                draw_world_polyline(img, calib, seg, color=dropper_color,
                                    thickness=max(1, thickness - 2), alpha=0.45,
                                    halo_extra=2)

    draw_world_polyline(img, calib, ref, color=color, thickness=thickness, closed=closed,
                        dash=dash, halo=halo)

    if start_marker and len(ref):
        draw_marker_world(img, calib, ref[0], color=color, radius=max(5, thickness + 2))
    return img


def draw_world_axes(img, calib: Calibration, length=0.5, origin=(0.0, 0.0, 0.0),
                    thickness=4, labels=True):
    """Draw an RGB world-frame triad -- the fastest way to spot a wrong axis convention."""
    o = np.asarray(origin, float)
    axes = [(np.array([length, 0, 0]), (60, 60, 255), "x"),
            (np.array([0, length, 0]), (60, 220, 60), "y"),
            (np.array([0, 0, length]), (255, 160, 60), "z")]
    for d, col, name in axes:
        seg = np.array([o, o + d])
        draw_world_polyline(img, calib, seg, color=col, thickness=thickness)
        if labels:
            draw_marker_world(img, calib, o + d, color=col, radius=4, label=name)
    return img


def draw_floor_grid(img, calib: Calibration, floor_z=0.0, extent=3.0, step=0.5,
                    color=(0, 255, 255), thickness=1, alpha=0.5):
    """Overlay the metric floor grid -- the calibration sanity check.

    If this does not sit on the real tile edges, the calibration is wrong and the
    projected reference will be wrong too.
    """
    n = int(round(extent / step))
    layer = img.copy()
    for i in range(-n, n + 1):
        a = i * step
        line_x = np.column_stack([np.full(41, a), np.linspace(-extent, extent, 41),
                                  np.full(41, floor_z)])
        line_y = np.column_stack([np.linspace(-extent, extent, 41), np.full(41, a),
                                  np.full(41, floor_z)])
        for ln in (line_x, line_y):
            draw_world_polyline(layer, calib, ln, color=color, thickness=thickness,
                                halo=False)
    cv2.addWeighted(layer, alpha, img, 1.0 - alpha, 0.0, dst=img)
    return img


def draw_hud(img, lines, org=(24, 44), color=(255, 255, 255), scale=0.7):
    for i, text in enumerate(lines):
        p = (org[0], org[1] + int(i * 32 * scale / 0.7))
        cv2.putText(img, text, p, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, text, p, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
    return img
