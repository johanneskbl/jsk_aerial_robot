"""Tests for the transparent-PNG reference overlay.  Run: python3 test_overlay.py"""
import importlib
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
             cam_pos=(-0.17, -1.9, 1.2), look_at=(0.0, 0.0, 0.5))
REF = G.circle_reference(1.0, 0.47, 720)


# ------------------------------------------------------------------ 1. transparency
def test_transparent_background():
    rgba = R.render_reference_rgba(SIZE, GT, REF)
    check("output is 4-channel BGRA", rgba.ndim == 3 and rgba.shape[2] == 4
          and rgba.shape[:2] == (SIZE[1], SIZE[0]), f"shape {rgba.shape}, dtype {rgba.dtype}")
    a = rgba[..., 3]
    clear = (a == 0).mean()
    check("background is fully transparent", clear > 0.9 and (a == 255).any(),
          f"{clear*100:.2f}% of pixels alpha=0, {(a==255).mean()*100:.2f}% fully opaque")
    # a corner far from the curve must be untouched
    check("corners are clear", a[0, 0] == 0 and a[-1, 0] == 0 and a[0, -1] == 0,
          "all four corner samples alpha=0")


# --------------------------------------------------- 2. no colour fringing anywhere
def test_no_colour_fringe():
    """The whole point of building alpha separately: edge pixels must keep the
    exact line colour, not a blend toward black."""
    col = R.hex_to_bgr(R.REFERENCE_COLOR)
    rgba = R.render_reference_rgba(SIZE, GT, REF)
    bgr = rgba[..., :3]
    uniform = np.all(bgr == np.array(col, np.uint8), axis=2)
    check("every pixel carries the exact line colour", uniform.all(),
          f"{(~uniform).sum()} deviating pixels (expect 0); colour {col}")
    edge = (rgba[..., 3] > 0) & (rgba[..., 3] < 255)
    check("anti-aliased edge pixels exist and are unfringed",
          edge.any() and np.all(bgr[edge] == np.array(col, np.uint8)),
          f"{edge.sum()} partially transparent edge pixels, all exactly {col}")

    # contrast: the naive approach (draw AA straight onto BGRA) does fringe
    naive = np.zeros((SIZE[1], SIZE[0], 4), np.uint8)
    segs = R.project_polyline(GT, REF, closed=True)
    for s in segs:
        cv2.polylines(naive, [np.round(s).astype(np.int32)], False,
                      (col[0], col[1], col[2], 255), 6, cv2.LINE_AA)
    ne = (naive[..., 3] > 0) & (naive[..., 3] < 255)
    fringed = ne.any() and not np.all(naive[..., :3][ne] == np.array(col, np.uint8))
    check("the naive BGRA approach really would fringe (guard against regressing)",
          fringed, f"{ne.sum()} edge pixels, colour deviates -> confirms why we build alpha apart")


# ------------------------------------------------------------------- 3. dash geometry
def test_dash_geometry():
    seg = np.column_stack([np.linspace(100, 1100, 2001), np.full(2001, 500.0)])
    on, off = 16.0, 13.0
    dashes = R._dash(seg, on, off)
    lens = np.array([np.linalg.norm(d[-1] - d[0]) for d in dashes])
    gaps = np.array([dashes[i + 1][0, 0] - dashes[i][-1, 0] for i in range(len(dashes) - 1)])
    check("dash lengths match the pattern", np.allclose(lens[:-1], on, atol=0.6),
          f"{len(dashes)} dashes, length {lens[:-1].mean():.2f}+/-{lens[:-1].std():.2f} px "
          f"(asked {on})")
    check("gap lengths match the pattern", np.allclose(gaps, off, atol=0.6),
          f"gap {gaps.mean():.2f}+/-{gaps.std():.2f} px (asked {off})")

    # duplicate points must not break the arc-length interpolation
    dup = np.vstack([seg[:50], seg[49:50], seg[49:50], seg[50:]])
    check("duplicate points tolerated", len(R._dash(dup, on, off)) > 10,
          f"{len(R._dash(dup, on, off))} dashes from a polyline with repeated points")
    check("degenerate input returns nothing", R._dash(np.zeros((1, 2)), on, off) == []
          and R._dash(np.zeros((5, 2)), on, off) == [],
          "single point and all-identical points both yield no dashes")


def test_dashed_vs_solid():
    """Walk the projected curve and confirm the alpha really alternates on/off.

    Counting covered pixels is a poor proxy: every dash adds its own anti-aliased
    end caps, which inflates the count well above the on/period duty cycle. Sampling
    along the curve measures the thing that actually matters -- that a viewer sees
    dashes of about the requested length separated by gaps of about the requested
    length.
    """
    dashed = R.render_reference_rgba(SIZE, GT, REF)
    solid = R.render_reference_rgba(SIZE, GT, REF, dash=None)
    a_d, a_s = dashed[..., 3], solid[..., 3]

    seg = max(R.project_polyline(GT, REF, closed=True), key=len)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))])
    t = np.arange(0.0, s[-1], 0.25)
    u = np.interp(t, s, seg[:, 0]); v = np.interp(t, s, seg[:, 1])
    inside = (u > 1) & (u < SIZE[0] - 2) & (v > 1) & (v < SIZE[1] - 2)
    t, u, v = t[inside], u[inside], v[inside]
    hit_d = a_d[np.round(v).astype(int), np.round(u).astype(int)] > 100
    hit_s = a_s[np.round(v).astype(int), np.round(u).astype(int)] > 100

    check("a solid line is continuous along the curve", hit_s.mean() > 0.97,
          f"{hit_s.mean()*100:.1f}% of curve samples covered")

    edges = np.nonzero(np.diff(hit_d.astype(int)))[0]
    runs = np.diff(t[edges]) if len(edges) > 2 else np.array([])
    on_runs = runs[0::2] if hit_d[edges[0] + 1] else runs[1::2]
    off_runs = runs[1::2] if hit_d[edges[0] + 1] else runs[0::2]
    on_t, off_t = R.REFERENCE_DASH_ON_PX, R.REFERENCE_DASH_OFF_PX
    check("dash runs measure about the requested length",
          len(on_runs) > 20 and abs(np.median(on_runs) - on_t) < 0.30 * on_t,
          f"{len(on_runs)} dashes, median {np.median(on_runs):.1f} px (asked {on_t})")
    check("gap runs measure about the requested length",
          len(off_runs) > 20 and abs(np.median(off_runs) - off_t) < 0.35 * off_t,
          f"{len(off_runs)} gaps, median {np.median(off_runs):.1f} px (asked {off_t})")
    check("duty cycle is in the right ballpark",
          0.35 < hit_d.mean() < 0.75,
          f"{hit_d.mean()*100:.1f}% of curve covered (nominal "
          f"{on_t/(on_t+off_t)*100:.0f}%)")


# -------------------------------------------------- 4. one place controls the colour
def test_single_point_of_control():
    original = R.REFERENCE_COLOR
    try:
        R.REFERENCE_COLOR = "#3C3C3C"
        gray = R.render_reference_rgba(SIZE, GT, REF)
        R.REFERENCE_COLOR = "#FF0000"
        red = R.render_reference_rgba(SIZE, GT, REF)
    finally:
        R.REFERENCE_COLOR = original
    check("editing REFERENCE_COLOR changes the rendered colour",
          tuple(gray[0, 0, :3]) == (60, 60, 60) and tuple(red[0, 0, :3]) == (0, 0, 255),
          f"gray -> {tuple(int(v) for v in gray[0,0,:3])}, red -> {tuple(int(v) for v in red[0,0,:3])}")
    check("alpha is unaffected by the colour change",
          np.array_equal(gray[..., 3], red[..., 3]), "identical coverage masks")

    # and the CLI must not shadow it with a literal default
    src = open(os.path.join(HERE, "project_reference.py")).read()
    check("CLI --color/--thickness default to None so STYLE wins",
          '"--color", default=None' in src and '"--thickness", type=int, default=None' in src,
          "a literal argparse default here would silently override render.py")


# ------------------------------------------------------------------ 5. geometry match
def test_matches_burned_in_projection():
    """The PNG must land on exactly the same pixels as the burned-in render."""
    rgba = R.render_reference_rgba(SIZE, GT, REF, dash=None, thickness=6)
    frame = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    R.draw_world_polyline(frame, GT, REF, color=(255, 255, 255), thickness=6,
                          closed=True, halo=False, dash=None)
    a = rgba[..., 3] > 128
    b = frame[..., 0] > 128
    iou = (a & b).sum() / max((a | b).sum(), 1)
    check("overlay aligns with the burned-in render", iou > 0.95,
          f"IoU {iou:.4f} over {a.sum()} / {b.sum()} pixels")


# ----------------------------------------------------------------- 6. png round-trip
def test_png_roundtrip_and_scale():
    tmp = tempfile.mkdtemp()
    p = os.path.join(tmp, "ov.png")
    rgba = R.render_reference_rgba(SIZE, GT, REF)
    cv2.imwrite(p, rgba)
    back = cv2.imread(p, cv2.IMREAD_UNCHANGED)
    check("PNG keeps 4 channels through a round-trip",
          back is not None and back.shape == rgba.shape and np.array_equal(back, rgba),
          f"shape {None if back is None else back.shape}")
    col = R.hex_to_bgr(R.REFERENCE_COLOR)
    small = cv2.resize(rgba, (SIZE[0] // 2, SIZE[1] // 2), interpolation=cv2.INTER_AREA)
    check("downscaling introduces no foreign colour",
          np.all(small[..., :3] == np.array(col, np.uint8)),
          "uniform colour channels make INTER_AREA safe on straight alpha")


# --------------------------------------------------------------------------- 7. CLI
def test_cli():
    tmp = tempfile.mkdtemp()
    cal = os.path.join(tmp, "c.json"); GT.to_json(cal)
    out = os.path.join(tmp, "ov.png")
    r = subprocess.run([sys.executable, os.path.join(HERE, "project_reference.py"), "overlay",
                        "--calib", cal, "--out", out, "--radius", "1.0", "--z", "0.47"],
                       capture_output=True, text=True)
    ok = r.returncode == 0 and os.path.exists(out)
    img = cv2.imread(out, cv2.IMREAD_UNCHANGED) if ok else None
    check("CLI overlay writes a transparent PNG",
          ok and img is not None and img.shape[2] == 4 and (img[..., 3] == 0).mean() > 0.9,
          (r.stderr[-200:] if not ok else f"{img.shape}, "
           f"{(img[...,3]==0).mean()*100:.1f}% clear"))

    r2 = subprocess.run([sys.executable, os.path.join(HERE, "project_reference.py"), "overlay",
                         "--calib", cal, "--out", os.path.join(tmp, "j.jpg")],
                        capture_output=True, text=True)
    check("CLI refuses a lossy container that would drop alpha",
          r2.returncode != 0 and "png" in (r2.stdout + r2.stderr).lower(),
          (r2.stdout + r2.stderr).strip()[-90:])


# -------------------------------------------------- 8. closed-loop dash wraps seamlessly
def _dash_gaps(dashes):
    """Euclidean pixel gap between the end of each dash and the start of the next,
    treating the list as circular (last -> first is the wrap-around gap)."""
    gaps = []
    for i in range(len(dashes)):
        j = (i + 1) % len(dashes)
        gaps.append(float(np.linalg.norm(dashes[j][0] - dashes[i][-1])))
    return np.array(gaps)


def test_closed_dash_wraps_seamlessly():
    """The reported bug: on a full loop, the dash pattern almost never divides the
    circumference evenly, so naive fixed-step placement leaves the seam where the
    curve closes on itself with a leftover sliver -- two dashes end up touching or
    nearly touching right where the curve starts repeating. Every gap, including
    that one, must come out the same length.
    """
    segs = R.project_polyline(GT, REF, closed=True)
    seg = max(segs, key=len)
    check("test setup: the projected circle is a single closed loop",
          np.allclose(seg[0], seg[-1], atol=1e-6) and len(segs) == 1,
          f"{len(segs)} segment(s), endpoint gap {np.linalg.norm(seg[0]-seg[-1]):.2e} px")

    on, off = 25.0, 20.0            # the user's own settings that exposed the bug
    cap = R._cap_extension(12.0)
    dashes = R._dash(seg, on, off, cap=cap)
    check("more than one dash was produced", len(dashes) > 3, f"{len(dashes)} dashes")

    gaps = _dash_gaps(dashes)
    check("all gaps are close to uniform, including the wrap",
          gaps.std() < 0.10 * gaps.mean(),
          f"{len(gaps)} gaps: mean {gaps.mean():.2f}px std {gaps.std():.2f}px "
          f"min {gaps.min():.2f} max {gaps.max():.2f}")
    check("the wrap gap specifically matches the others (was the reported bug)",
          abs(gaps[-1] - np.median(gaps[:-1])) < 0.25 * np.median(gaps[:-1]),
          f"wrap gap {gaps[-1]:.2f}px vs median of the rest {np.median(gaps[:-1]):.2f}px")

    lens = np.array([np.linalg.norm(d[-1] - d[0]) for d in dashes])
    check("all dash lengths are close to uniform, including the one at the seam",
          lens.std() < 0.10 * lens.mean(),
          f"{len(lens)} dashes: mean {lens.mean():.2f}px std {lens.std():.2f}px")

    # n*(on_e+off_e) must equal the circumference exactly -> no remainder anywhere
    n = max(1, int(round(seg_total(seg) / (on + off))))
    check("dash+gap count matches the nearest whole tiling of the loop",
          len(dashes) == n, f"{len(dashes)} dashes, expected round(circumference/period)={n}")


def seg_total(seg):
    return float(np.sum(np.linalg.norm(np.diff(seg, axis=0), axis=1)))


def test_closed_dash_various_thicknesses_and_patterns():
    """The fix must hold for arbitrary on/off/thickness combinations, not just the
    one pattern spot-checked above -- including ones where the circumference is
    (coincidentally) very close to a whole number of periods already."""
    segs = R.project_polyline(GT, REF, closed=True)
    seg = max(segs, key=len)
    total = seg_total(seg)
    for on, off, thickness in [(16, 13, 6), (40, 30, 12), (8, 8, 3),
                               (total / 5, total / 20, 5),      # near-exact tiling
                               (3, 2, 1)]:
        cap = R._cap_extension(thickness)
        dashes = R._dash(seg, on, off, cap=cap)
        if len(dashes) < 3:
            check(f"on={on:.2f} off={off:.2f} t={thickness}: enough dashes to check",
                  False, f"only {len(dashes)} dashes")
            continue
        gaps = _dash_gaps(dashes)
        ok = gaps.std() < 0.12 * gaps.mean()
        check(f"on={on:.2f} off={off:.2f} t={thickness}: uniform gaps incl. wrap", ok,
              f"{len(gaps)} gaps, mean {gaps.mean():.2f}px std {gaps.std():.2f}px")


def test_open_curve_dash_unaffected():
    """An open arc (camera close enough that part of the circle is culled) must not
    be mistaken for a closed loop and forcibly tiled -- it keeps its natural
    possibly-partial dash at each end, same as before this fix."""
    close_cam = make_gt(width=SIZE[0], height=SIZE[1], hfov=75.0,
                        cam_pos=(0.0, 0.05, 0.47), look_at=(0.0, 3.0, 0.47))
    segs = R.project_polyline(close_cam, REF, closed=True)
    check("test setup: the curve is actually broken into open arc(s) here",
          all(not np.allclose(s[0], s[-1], atol=1e-6) for s in segs) and len(segs) >= 1,
          f"{len(segs)} segment(s)")
    for seg in segs:
        dashes = R._dash(seg, 25.0, 20.0, cap=R._cap_extension(12.0))
        if len(dashes) < 2:
            continue
        # open-curve behaviour: dash length is exactly `on` in the interior; only
        # the first/last dash may be shorter (a partial dash at a cut end).
        lens = np.array([np.linalg.norm(d[-1] - d[0]) for d in dashes])
        interior = lens[1:-1] if len(lens) > 2 else lens[:0]
        if len(interior):
            check("open arc: interior dashes keep the exact requested length",
                  np.allclose(interior, 25.0 - R._cap_extension(12.0), atol=1.5),
                  f"{len(interior)} interior dashes, "
                  f"{interior.mean():.2f}+/-{interior.std():.2f} px "
                  f"(expected {25.0 - R._cap_extension(12.0):.1f})")


if __name__ == "__main__":
    for fn in [test_transparent_background, test_no_colour_fringe, test_dash_geometry,
               test_dashed_vs_solid, test_single_point_of_control,
               test_matches_burned_in_projection, test_png_roundtrip_and_scale, test_cli,
               test_closed_dash_wraps_seamlessly, test_closed_dash_various_thicknesses_and_patterns,
               test_open_curve_dash_unaffected]:
        print(f"\n=== {fn.__name__} ===")
        fn()
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print(f"\n{'='*70}\n{len(RESULTS)-n_fail}/{len(RESULTS)} passed"
          + ("" if n_fail == 0 else f"  --  {n_fail} FAILED"))
    raise SystemExit(1 if n_fail else 0)
