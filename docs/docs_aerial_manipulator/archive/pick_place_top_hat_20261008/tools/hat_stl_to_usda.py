#!/usr/bin/env python3
"""Top_Hat.STL (SolidWorks, mm, +Z up) -> Top_Hat.usda (m, +Z up) (2026-10-08).

    /usr/bin/python3 hat_stl_to_usda.py [--stl ...] [--out ...]

(usd-core lives in the user site here: do NOT set PYTHONNOUSERSITE.)

THE PILLAR HAT (user, 2026-10-08): the platform the payload is picked from and
placed on. It sits on the pillar top: a 200.6 mm disc, 5 mm thick, on a
122 mm sleeve that runs 55 mm down over the pillar. The sleeve's bore is
110 mm with a lead-in chamfer at its mouth, and FOUR INTERNAL RIBS (8 mm wide,
at 0 / 90 / 180 / 270 deg, from 10 mm above the mouth to the bore ceiling)
narrow it to 97.86 mm: the ribs, not the bore, centre the hat on the pillar.
The bore ceiling rests on the pillar top, 8 mm under the platform top.

HAT FRAME (the root prim's own frame, unscaled):
  origin = the SEAT: the bore ceiling's centre, i.e. the pillar top it rests on
  +z     = up (CAD +Z), x / y = CAD X / Y (the ribs on the axes)

WHAT IS AUTHORED:
  Visual             the full STL mesh, welded, in metres (display only)
  Collision/platform a solid cylinder, the platform disc
  Collision/sleeve   a solid cylinder, the sleeve's outside (it overlaps the
                     static pillar inside the bore -- harmless, both are static)
  fsc:* attributes   the dimensions the scene sizes the pillar and places the
                     payload with (all derived from the mesh, checked below)
The hat is STATIC: no rigid body, no mass. Friction and placement are the
scene's job (07_px4_t650_aerial_manipulator_pick_and_place.py).
"""
import argparse
import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ASSETS = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..",
    "extensions", "fsc_aerial_manipulation", "fsc_aerial_manipulation", "rotorcraft", "assets"))


def read_stl(path):
    raw = open(path, "rb").read()
    n = int(np.frombuffer(raw[80:84], "<u4")[0])
    if 84 + 50 * n != len(raw):
        raise SystemExit(f"{path}: not a binary STL (or truncated)")
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(raw[84:84 + 50 * n], dt)["v"].astype(float)      # (n, 3, 3) mm


def near(value, expected, tol, what):
    if abs(value - expected) > tol:
        raise SystemExit(f"unexpected {what}: {value:.3f} mm (expected {expected} +- {tol}) "
                         f"-- not the 2026-10-08 Top_Hat.STL?")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=os.path.join(ASSETS, "Top_Hat.STL"))
    ap.add_argument("--out", default=os.path.join(ASSETS, "Top_Hat.usda"))
    a = ap.parse_args()

    T = read_stl(a.stl)
    V = np.unique(np.round(T.reshape(-1, 3), 4), axis=0)
    lo, hi = V.min(0), V.max(0)
    c_xy = 0.5 * (lo[:2] + hi[:2])                    # the platform disc is the bounding square
    r = np.linalg.norm(V[:, :2] - c_xy, axis=1)
    z = V[:, 2]

    top, bottom = hi[2], lo[2]
    r_plat = r.max()
    # the platform's underside: the lowest z of the vertices on the platform rim
    z_plat_lo = z[r > r_plat - 0.01].min()
    # the sleeve: the outermost vertices below the platform
    below = z < z_plat_lo - 1e-3
    r_sleeve = r[below].max()
    # the bore ceiling (the SEAT): the highest vertex inside the sleeve below the platform
    inner = below & (r < r_sleeve - 1.0)
    z_seat = z[inner].max()
    # the bore wall and the ribs, between the chamfered mouth and the ceiling
    mid = inner & (z > bottom + 15.0) & (z < z_seat - 1.0)
    mid |= inner & (np.abs(z - z_seat) < 1e-3) & (r > 1.0)       # the ceiling's rim carries both radii
    r_bore, r_rib = r[mid].max(), r[mid].min()
    r_mouth = r[inner & (np.abs(z - bottom) < 1e-3)].min()

    near(top - bottom, 60.0, 0.01, "height")
    near(2 * r_plat, 200.576, 0.05, "platform diameter")
    near(top - z_plat_lo, 5.0, 0.01, "platform thickness")
    near(2 * r_sleeve, 122.0, 0.05, "sleeve diameter")
    near(z_seat - bottom, 52.0, 0.01, "bore depth")
    near(2 * r_bore, 110.0, 0.05, "bore diameter")
    near(2 * r_rib, 97.86, 0.05, "rib inner diameter")

    seat = np.array([c_xy[0], c_xy[1], z_seat])

    def to_hat(p_mm):
        return (np.asarray(p_mm) - seat) * 1e-3

    # weld the visual mesh
    P = to_hat(T.reshape(-1, 3))
    key = np.round(P / 1e-7).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    pts = np.zeros((len(uniq), 3))
    pts[inv] = P
    idx = inv.reshape(-1, 3)

    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Top_Hat")
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    root.GetPrim().SetDocumentation(
        "Pillar hat (the pick-and-place platform), 2026-10-08. Frame: origin = the seat (the bore "
        "ceiling's centre = the pillar top it rests on), +z up, the four ribs on the x / y axes. Static. "
        "Generated by docs/docs_aerial_manipulator/archive/pick_place_top_hat_20261008/tools/"
        "hat_stl_to_usda.py from Top_Hat.STL -- regenerate, do not edit.")

    vis = UsdGeom.Mesh.Define(stage, "/Top_Hat/Visual")
    vis.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
    vis.CreateFaceVertexCountsAttr([3] * len(idx))
    vis.CreateFaceVertexIndicesAttr(idx.flatten().tolist())
    vis.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    vis.CreateDisplayColorAttr([Gf.Vec3f(0.92, 0.92, 0.88)])
    vis.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])

    UsdGeom.Scope.Define(stage, "/Top_Hat/Collision")

    def cylinder(path, radius_mm, z_lo_mm, z_hi_mm):
        cyl = UsdGeom.Cylinder.Define(stage, path)
        cyl.CreateRadiusAttr(float(radius_mm * 1e-3))
        cyl.CreateHeightAttr(float((z_hi_mm - z_lo_mm) * 1e-3))
        cyl.CreateAxisAttr("Z")
        UsdGeom.Xformable(cyl).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, float((0.5 * (z_lo_mm + z_hi_mm) - z_seat) * 1e-3)))
        cyl.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        return cyl

    cylinder("/Top_Hat/Collision/platform", r_plat, z_plat_lo, top)
    cylinder("/Top_Hat/Collision/sleeve", r_sleeve, bottom, z_plat_lo)

    feats = {
        "fsc:platform_top_z_m": (top - z_seat) * 1e-3,           # above the seat (the pillar top)
        "fsc:platform_radius_m": r_plat * 1e-3,
        "fsc:platform_thickness_m": (top - z_plat_lo) * 1e-3,
        "fsc:sleeve_radius_m": r_sleeve * 1e-3,
        "fsc:sleeve_bottom_z_m": (bottom - z_seat) * 1e-3,       # below the seat
        "fsc:bore_diameter_m": 2 * r_bore * 1e-3,
        "fsc:bore_mouth_diameter_m": 2 * r_mouth * 1e-3,
        "fsc:rib_inner_diameter_m": 2 * r_rib * 1e-3,           # the largest pillar that fits
    }
    for k, v in feats.items():
        root.GetPrim().CreateAttribute(k, Sdf.ValueTypeNames.Double, custom=True).Set(float(v))
    stage.GetRootLayer().Save()

    print(f"wrote {a.out}")
    print(f"  {len(idx)} visual triangles ({len(pts)} welded points), 2 cylinder colliders")
    for k, v in feats.items():
        print(f"  {k} = {v * 1e3:+.3f} mm")


if __name__ == "__main__":
    main()
