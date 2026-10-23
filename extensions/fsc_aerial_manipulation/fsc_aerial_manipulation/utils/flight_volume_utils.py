#!/usr/bin/env python
"""
| File: flight_volume_utils.py
| Author: Shiqi Gao (shiqi.gao907@gmail.com)
| License: BSD-3-Clause
| Description: Draw the indoor flight/mocap volume as a wireframe box in the
|              viewport, so a simulated trajectory can be checked against the
|              real room BEFORE it is flown on hardware.

Pure VISUAL geometry — `UsdGeom.BasisCurves` line segments with no
`CollisionAPI` and no rigid body, so the box never interacts with physics and
costs nothing per step (unlike debug_draw, which must be re-issued every frame).

Typical use, matching the FSC indoor mocap field (4.5 x 4.5 x 2.0 m):

    from fsc_aerial_manipulation.utils import draw_flight_volume, volume_report

    draw_flight_volume(stage, size=(4.5, 4.5, 2.0), center=(0.0, 0.0))

`volume_report` answers the question the box is really there to answer — does
this trajectory fit, and with how much margin — without needing to eyeball it.
"""

import numpy as np

# `pxr` is imported lazily inside draw_flight_volume so that volume_bounds /
# volume_report — pure numpy — stay usable from plain python (offline log
# analysis, unit tests) where USD is not available.


def volume_bounds(size, center=(0.0, 0.0), floor_z=0.0):
    """(lo, hi) corner arrays of a volume `size` centred on `center` in xy and
    resting on `floor_z`."""
    sx, sy, sz = (float(v) for v in size)
    cx, cy = (float(v) for v in center)
    lo = np.array([cx - sx / 2.0, cy - sy / 2.0, float(floor_z)])
    hi = np.array([cx + sx / 2.0, cy + sy / 2.0, float(floor_z) + sz])
    return lo, hi


def draw_flight_volume(stage, size=(4.5, 4.5, 2.0), center=(0.0, 0.0),
                       floor_z=0.0, prim_path="/World/FlightVolume",
                       color=(0.15, 0.85, 0.35), width=0.012,
                       floor_color=(0.08, 0.50, 0.22)):
    """Wireframe box marking the usable flight volume.

    size      (x, y, z) extent [m] — z is the CEILING height above floor_z
    center    (x, y) centre of the box in the inertial frame [m]
    floor_z   height of the box's floor [m] (0 = the ground plane)
    color     RGB of the 8 vertical/ceiling edges
    floor_color RGB of the 4 floor edges — a DEEPER GREEN than the rest, so the
              whole box reads as green while the footprint still stands out
              from a top-down view
    width     line width [m]

    Returns the created BasisCurves prim. Re-calling with the same prim_path
    overwrites the previous box.
    """
    from pxr import Gf, Sdf, UsdGeom          # Isaac/USD only — see module note

    lo, hi = volume_bounds(size, center, floor_z)
    x0, y0, z0 = lo
    x1, y1, z1 = hi

    # 8 corners: 0-3 floor (CCW), 4-7 ceiling (CCW)
    c = [Gf.Vec3f(x0, y0, z0), Gf.Vec3f(x1, y0, z0),
         Gf.Vec3f(x1, y1, z0), Gf.Vec3f(x0, y1, z0),
         Gf.Vec3f(x0, y0, z1), Gf.Vec3f(x1, y0, z1),
         Gf.Vec3f(x1, y1, z1), Gf.Vec3f(x0, y1, z1)]
    floor_edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    other_edges = [(4, 5), (5, 6), (6, 7), (7, 4),      # ceiling
                   (0, 4), (1, 5), (2, 6), (3, 7)]      # verticals

    pts, counts, cols = [], [], []
    for edges, col in ((floor_edges, floor_color), (other_edges, color)):
        for a, b in edges:
            pts += [c[a], c[b]]
            counts.append(2)
            cols.append(Gf.Vec3f(*col))

    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(Sdf.Path(prim_path))
    curves = UsdGeom.BasisCurves.Define(stage, Sdf.Path(prim_path))
    curves.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
    curves.CreatePointsAttr().Set(pts)
    curves.CreateCurveVertexCountsAttr().Set(counts)
    curves.CreateWidthsAttr().Set([float(width)] * len(pts))
    curves.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
    # one colour per CURVE (uniform), so the floor ring can differ from the rest
    curves.CreateDisplayColorAttr().Set(cols)
    curves.GetDisplayColorPrimvar().SetInterpolation(UsdGeom.Tokens.uniform)
    # never occlude the vehicle
    curves.CreateDoubleSidedAttr().Set(True)
    return curves


def volume_report(points, size=(4.5, 4.5, 2.0), center=(0.0, 0.0), floor_z=0.0,
                  label="trajectory", clearance=None):
    """Check an Nx3 path against the volume and return a printable summary.

    `points` should include EVERY part that must stay inside — the body, the
    end-effector, and (for a real room) the ROTOR TIPS, which sit above the
    body origin and are what actually hits the ceiling. Pass `clearance` to
    inflate the path by a fixed radius [m] to stand in for that.

    Returns (ok, text). `ok` is False if any point is outside.
    """
    P = np.asarray(points, float).reshape(-1, 3)
    lo, hi = volume_bounds(size, center, floor_z)
    pad = float(clearance or 0.0)
    p_lo, p_hi = P.min(0) - pad, P.max(0) + pad
    margin_lo, margin_hi = p_lo - lo, hi - p_hi
    ok = bool((margin_lo >= 0).all() and (margin_hi >= 0).all())
    ax = "xyz"
    lines = [f"[flight volume] {label} vs {size[0]}x{size[1]}x{size[2]} m box "
             f"centred ({center[0]}, {center[1]}) floor {floor_z} m"
             + (f", inflated by {pad:.3f} m" if pad else "")]
    for i in range(3):
        flag = "  <-- OUTSIDE" if (margin_lo[i] < 0 or margin_hi[i] < 0) else ""
        lines.append(f"    {ax[i]}: path [{p_lo[i]:+.2f}, {p_hi[i]:+.2f}] "
                     f"box [{lo[i]:+.2f}, {hi[i]:+.2f}] "
                     f"margin [{margin_lo[i]:+.2f}, {margin_hi[i]:+.2f}] m{flag}")
    lines.append("    => FITS" if ok else "    => DOES NOT FIT")
    return ok, "\n".join(lines)


# ===========================================================================
# Scene markers — waypoints and poses
# ===========================================================================
# Same BasisCurves approach as the volume box: persistent, collider-free, and
# free per physics step (unlike debug_draw, which must be re-issued each frame).

def _basis_curves(stage, prim_path, segments, width):
    """segments = [(p0, p1, (r,g,b)), ...] -> one BasisCurves prim."""
    from pxr import Gf, Sdf, UsdGeom

    pts, counts, cols = [], [], []
    for a, b, c in segments:
        pts += [Gf.Vec3f(*[float(v) for v in a]), Gf.Vec3f(*[float(v) for v in b])]
        counts.append(2)
        cols.append(Gf.Vec3f(*[float(v) for v in c]))
    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(Sdf.Path(prim_path))
    cur = UsdGeom.BasisCurves.Define(stage, Sdf.Path(prim_path))
    cur.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
    cur.CreatePointsAttr().Set(pts)
    cur.CreateCurveVertexCountsAttr().Set(counts)
    cur.CreateWidthsAttr().Set([float(width)] * len(pts))
    cur.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
    cur.CreateDisplayColorAttr().Set(cols)
    cur.GetDisplayColorPrimvar().SetInterpolation(UsdGeom.Tokens.uniform)
    cur.CreateDoubleSidedAttr().Set(True)
    return cur


def draw_pose_marker(stage, prim_path, position, heading=None, size=0.10,
                     color=(1.0, 0.85, 0.2), width=0.010, point_scale=2.6,
                     head_frac=0.30, head_half_angle_deg=24.0):
    """Marker for a commanded END-EFFECTOR pose, drawn the way the command is
    actually structured: a POINT for the 3-D position and an ARROW for the 1-D
    yaw. Nothing else — the EE task has no other controlled degree of freedom.

    position  world point [m]                       -> rendered as a round dot
    heading   the commanded heading (e.g. b1_de)    -> rendered as the arrow;
              omit for a position-only dot
    size      arrow length [m]; the dot scales with it via `point_scale`

    The arrowhead is FOUR barbs in a cross around the shaft, so the direction
    reads the same from any camera angle (two barbs vanish edge-on).
    """
    from pxr import Gf, Sdf, UsdGeom

    p = np.asarray(position, float)
    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(Sdf.Path(prim_path))
    UsdGeom.Xform.Define(stage, Sdf.Path(prim_path))

    # --- the POINT (3-D position) ---
    pts = UsdGeom.Points.Define(stage, Sdf.Path(prim_path + "/point"))
    pts.CreatePointsAttr().Set([Gf.Vec3f(*[float(v) for v in p])])
    pts.CreateWidthsAttr().Set([float(width) * float(point_scale)])
    pts.CreateDisplayColorAttr().Set([Gf.Vec3f(*[float(c) for c in color])])

    # --- the ARROW (1-D yaw) ---
    if heading is None:
        return pts
    h = np.asarray(heading, float)
    n = np.linalg.norm(h)
    if n < 1e-9:
        return pts
    h = h / n
    L = float(size)
    tip = p + h * L
    # two unit vectors spanning the plane perpendicular to the shaft
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(h @ ref)) > 0.9:
        ref = np.array([1.0, 0.0, 0.0])
    u = np.cross(h, ref)
    u /= np.linalg.norm(u)
    v = np.cross(h, u)
    hl = L * float(head_frac)
    a = np.radians(float(head_half_angle_deg))
    back = tip - h * hl * np.cos(a)
    r = hl * np.sin(a)
    seg = [(p, tip, color)]
    for d in (u, -u, v, -v):
        seg.append((tip, back + d * r, color))
    _basis_curves(stage, prim_path + "/arrow", seg, width)
    return pts


def draw_pose_axes(stage, prim_path, position, R=None, size=0.12, width=0.008):
    """RGB = XYZ triad at a pose. `R` is a 3x3 rotation (columns = axes);
    identity if omitted."""
    p = np.asarray(position, float)
    R = np.eye(3) if R is None else np.asarray(R, float)
    cols = ((0.9, 0.2, 0.2), (0.2, 0.9, 0.2), (0.25, 0.45, 1.0))
    seg = [(p, p + R[:, i] * float(size), cols[i]) for i in range(3)]
    return _basis_curves(stage, prim_path, seg, width)
