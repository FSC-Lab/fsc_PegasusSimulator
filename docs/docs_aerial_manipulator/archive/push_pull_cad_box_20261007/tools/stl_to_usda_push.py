#!/usr/bin/env python3
"""Box_Push.stl (CAD, metres, +Z up) -> Box_Push.usda (2026-10-07).

    /usr/bin/python3 stl_to_usda_push.py [--stl ...] [--out ...] [--mass 0.4]

(pxr comes from usd-core in the user site, so NOT with PYTHONNOUSERSITE=1.)

The push-and-pull box is ONE rigid body (user, 2026-10-07): a closed box
240 x 160 x 100 mm with a 3D-printed CLAMP BRACKET on its top centre --
two framed side plates (4 mm) hanging 40 mm down the box's 160 mm faces, a
4 mm strip across the top joining them, a 60 x 60 mm screw pad, a 10 mm rib,
and a FIN 60 mm long x 5 mm thick seated in a slot in the rib, its top
258 mm above the table. The jaws close across the fin's 5 mm.

ASSET FRAME (the root prim's own frame, unscaled):
  origin = the BOX centre (the mocap body obj_0 of the push-and-pull scene)
  +z     = up (CAD +Z)
  +y     = CAD +X = the box's 240 mm axis = along the fin = the PUSH direction
  +x     = CAD -Y = the box's 160 mm axis = across the fin = the jaws close along it
  (Rz(+90) of the CAD frame: the scene's box convention, BOX_SIZE x 160 / y 240.)

WHAT IS AUTHORED:
  Visual             the full STL mesh, welded (display only; 8880 triangles)
  Collision/box      the box, one convex box
  Collision/<part>   the bracket as EXACT convex boxes, read off the CAD's own
                     faces (every bracket piece is an axis-aligned block; the
                     screw holes, the pad's pocket and the rib's pin hole are
                     left solid -- nothing touches them)
  MassAPI            mass, centre of mass and inertia of the CLOSED STL solids
                     at uniform density, scaled to --mass (only ~3 % of the
                     volume is bracket, so a uniform density puts ~12 g in it;
                     --handle-mass gives it its own mass instead)
  fsc:* attributes   the numbers the scene and the planner need (box size, fin
                     geometry), so nothing downstream re-measures the mesh
The rigid body, the friction and the placement are the scene's job
(07_px4_direct_t650_aerial_manipulator_push_and_pull.py, PEGASUS_PUSH_HANDLE=cad).
"""
import argparse
import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ASSETS = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..",
    "extensions", "fsc_aerial_manipulation", "fsc_aerial_manipulation", "rotorcraft", "assets"))

# CAD frame -> asset frame: Rz(+90): x = -Y, y = X, z = Z (a proper rotation)
R_CAD = np.array([[0.0, -1.0, 0.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0]])

# THE BRACKET, CAD millimetres (read off the STL's horizontal and vertical faces,
# 2026-10-07): (x_lo, x_hi), (y_lo, y_hi), (z_lo, z_hi). The CAD box spans
# x -124.385..115.615, y -69.938..90.062, z -33.108..66.892.
_X60 = (-34.385, 25.615)                   # every bracket piece is 60 mm wide in CAD x
_LEG_Y = {"a": (-73.938, -69.938), "b": (90.062, 94.062)}
BRACKET_MM = {}
for _k, _y in _LEG_Y.items():              # each side plate: a frame of four bars round a 40 x 30 window
    BRACKET_MM[f"leg_{_k}_post_lo"] = ((-34.385, -24.385), _y, (26.892, 70.892))
    BRACKET_MM[f"leg_{_k}_post_hi"] = ((15.615, 25.615), _y, (26.892, 70.892))
    BRACKET_MM[f"leg_{_k}_bar_bottom"] = ((-24.385, 15.615), _y, (26.892, 33.892))
    BRACKET_MM[f"leg_{_k}_bar_top"] = ((-24.385, 15.615), _y, (63.892, 70.892))
BRACKET_MM["strip"] = (_X60, (-73.938, 94.062), (66.892, 70.892))
BRACKET_MM["pad"] = (_X60, (-19.938, 40.062), (70.892, 74.892))
BRACKET_MM["rib"] = (_X60, (5.062, 15.062), (74.892, 94.892))
BRACKET_MM["fin"] = (_X60, (7.562, 12.562), (74.892, 224.892))
GRIP_PARTS = ("fin", "rib")                # what the claw may touch


def read_stl(path):
    raw = open(path, "rb").read()
    n = int(np.frombuffer(raw[80:84], "<u4")[0])
    if 84 + 50 * n != len(raw):
        raise SystemExit(f"{path}: not a binary STL (or truncated)")
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(raw[84:84 + 50 * n], dt)["v"].astype(float)      # (n, 3, 3), CAD units


def solid_mass_props(T):
    """Volume, centroid and inertia (about the centroid, unit density) of closed
    triangle meshes, by signed tetrahedra from the origin (Tonon 2004)."""
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = v6.sum() / 6.0
    com = ((a + b + c) * v6[:, None]).sum(0) / (4.0 * v6.sum())
    C = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            s = (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j]) * 2.0 \
                + a[:, i] * b[:, j] + b[:, i] * a[:, j] + a[:, i] * c[:, j] + c[:, i] * a[:, j] \
                + b[:, i] * c[:, j] + c[:, i] * b[:, j]
            C[i, j] = (v6 * s).sum() / 120.0
    C -= vol * np.outer(com, com)
    I = np.trace(C) * np.eye(3) - C
    return vol, com, I


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=os.path.join(ASSETS, "Box_Push.stl"))
    ap.add_argument("--out", default=os.path.join(ASSETS, "Box_Push.usda"))
    ap.add_argument("--mass", type=float, default=0.400, help="total mass, box + bracket [kg]")
    ap.add_argument("--handle-mass", type=float, default=0.0,
                    help="the bracket's own mass [kg] (0 = uniform density over the whole CAD solid)")
    a = ap.parse_args()

    T_cad = read_stl(a.stl) * 1e3                                          # mm
    V = T_cad.reshape(-1, 3)
    if np.ptp(V, 0).max() > 1000.0 or np.ptp(V, 0).max() < 100.0:
        raise SystemExit(f"unexpected extent {np.ptp(V, 0)} mm -- not the metre-unit 2026-10-07 box?")

    # the box: the 12 triangles whose vertices all sit on the box's corners
    box_lo = np.array([-124.385, -69.938, -33.108])
    box_hi = np.array([115.615, 90.062, 66.892])
    on = np.all([np.isclose(T_cad[:, :, k][:, :, None], [box_lo[k], box_hi[k]], atol=1e-2).any(2)
                 for k in range(3)], axis=0).all(1)
    if on.sum() != 12:
        raise SystemExit(f"found {on.sum()} box-corner triangles, expected 12 -- the CAD changed?")
    H = T_cad[~on]                                                          # the bracket, mm
    Hv = H.reshape(-1, 3)
    # every bracket vertex must lie in (or on) one of the collider boxes, and the
    # boxes must stay inside the bracket's bounding box: the colliders ARE the bracket
    inside = np.zeros(len(Hv), bool)
    for (xl, xh), (yl, yh), (zl, zh) in BRACKET_MM.values():
        inside |= ((Hv[:, 0] >= xl - 0.01) & (Hv[:, 0] <= xh + 0.01) & (Hv[:, 1] >= yl - 0.01)
                   & (Hv[:, 1] <= yh + 0.01) & (Hv[:, 2] >= zl - 0.01) & (Hv[:, 2] <= zh + 0.01))
    if not inside.all():
        raise SystemExit(f"{(~inside).sum()} bracket vertices outside the collider boxes, e.g. {Hv[~inside][:3]}")
    c_box = 0.5 * (box_lo + box_hi)

    def to_asset(p_mm):
        return ((np.asarray(p_mm) - c_box) @ R_CAD.T) * 1e-3

    T = to_asset(T_cad.reshape(-1, 3)).reshape(-1, 3, 3)                    # m, asset frame
    if a.handle_mass > 0.0:
        vb, cb, Ib = solid_mass_props(T[on])
        vh, ch, Ih = solid_mass_props(T[~on])
        mb, mh = a.mass - a.handle_mass, a.handle_mass
        com = (mb * cb + mh * ch) / a.mass
        def shift(I, m, c):
            d = c - com
            return I + m * (d @ d * np.eye(3) - np.outer(d, d))
        I = shift(Ib * mb / vb, mb, cb) + shift(Ih * mh / vh, mh, ch)
        vol, rho = vb + vh, float("nan")
    else:
        vol, com, I = solid_mass_props(T)
        if vol <= 0:
            raise SystemExit("negative volume: the STL's winding is inverted")
        rho = a.mass / vol
        I = I * rho
    w, Q = np.linalg.eigh(I)
    if np.linalg.det(Q) < 0:
        Q[:, 0] = -Q[:, 0]
    qa = Gf.Quatf(Gf.Matrix3d(*Q.T.flatten().tolist()).ExtractRotation().GetQuat())

    P = T.reshape(-1, 3)
    key = np.round(P / 1e-7).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    pts = np.zeros((len(uniq), 3))
    pts[inv] = P
    idx = inv.reshape(-1, 3)

    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Box_Push")
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    root.GetPrim().SetDocumentation(
        "Push-and-pull box with its clamp bracket and fin, 2026-10-07. Frame: origin = box centre, "
        "+z up, +y = the box's 240 mm axis (along the fin, the push direction), +x = the 160 mm axis "
        "(across the fin: the jaws close along it). Generated by docs/docs_aerial_manipulator/archive/"
        "push_pull_cad_box_20261007/tools/stl_to_usda_push.py from Box_Push.stl -- regenerate, do not edit.")

    vis = UsdGeom.Mesh.Define(stage, "/Box_Push/Visual")
    vis.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
    vis.CreateFaceVertexCountsAttr([3] * len(idx))
    vis.CreateFaceVertexIndicesAttr(idx.flatten().tolist())
    vis.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    vis.CreateDisplayColorAttr([Gf.Vec3f(0.80, 0.55, 0.25)])
    vis.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])

    UsdGeom.Scope.Define(stage, "/Box_Push/Collision")

    def convex_box(name, lo_mm, hi_mm):
        c = [to_asset([x, y, z]) for x in (lo_mm[0], hi_mm[0]) for y in (lo_mm[1], hi_mm[1])
             for z in (lo_mm[2], hi_mm[2])]
        c = np.array(c)
        lo, hi = c.min(0), c.max(0)
        corners = [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        m = UsdGeom.Mesh.Define(stage, f"/Box_Push/Collision/{name}")
        m.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in corners])
        m.CreateFaceVertexCountsAttr([4] * 6)
        m.CreateFaceVertexIndicesAttr([0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3])
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        m.GetPrim().CreateAttribute("fsc:grip", Sdf.ValueTypeNames.Bool, custom=True).Set(name in GRIP_PARTS)
        return lo, hi

    convex_box("box", box_lo, box_hi)
    parts = {}
    for name, (xs, ys, zs) in BRACKET_MM.items():
        parts[name] = convex_box(name, (xs[0], ys[0], zs[0]), (xs[1], ys[1], zs[1]))

    UsdPhysics.MassAPI.Apply(root.GetPrim())
    mapi = UsdPhysics.MassAPI(root.GetPrim())
    mapi.CreateMassAttr(float(a.mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, w)))
    mapi.CreatePrincipalAxesAttr(qa)

    fin_lo, fin_hi = parts["fin"]
    rib_lo, rib_hi = parts["rib"]
    bs = (box_hi - box_lo) @ np.abs(R_CAD.T) * 1e-3
    feats = {
        "fsc:box_size_x_m": float(bs[0]), "fsc:box_size_y_m": float(bs[1]), "fsc:box_size_z_m": float(bs[2]),
        "fsc:box_half_height_m": float(0.5 * bs[2]),
        "fsc:fin_thickness_m": float(fin_hi[0] - fin_lo[0]),          # across the jaws (asset x)
        "fsc:fin_length_m": float(fin_hi[1] - fin_lo[1]),             # along the push (asset y)
        "fsc:fin_centre_x_m": float(0.5 * (fin_lo[0] + fin_hi[0])),
        "fsc:fin_centre_y_m": float(0.5 * (fin_lo[1] + fin_hi[1])),
        "fsc:fin_top_z_m": float(fin_hi[2]),                          # from the box centre
        "fsc:fin_exposed_bottom_z_m": float(rib_hi[2]),               # the rib top: the fin is bare above it
        "fsc:rib_thickness_m": float(rib_hi[0] - rib_lo[0]),
    }
    for k, v in feats.items():
        root.GetPrim().CreateAttribute(k, Sdf.ValueTypeNames.Double, custom=True).Set(v)
    stage.GetRootLayer().Save()

    hb = 0.5 * bs[2]
    print(f"wrote {a.out}")
    print(f"  {len(idx)} visual triangles ({len(pts)} welded points), 1 box + {len(BRACKET_MM)} bracket colliders")
    print(f"  box {np.round(bs, 4)} m; every bracket vertex lies in its collider boxes")
    print(f"  solid volume {vol * 1e6:.1f} cm^3 (bracket {solid_mass_props(T[~on])[0] * 1e6:.1f}) -> "
          + (f"density {rho:.0f} kg/m^3 for {a.mass} kg" if a.handle_mass <= 0 else
             f"box {a.mass - a.handle_mass:.3f} kg + bracket {a.handle_mass:.3f} kg"))
    print(f"  CoM {np.round(com * 1e3, 2)} mm from the box centre ({(hb + com[2]) * 1e3:.1f} mm above the table), "
          f"principal inertia {np.round(w * 1e6, 1)} x1e-6 kg m^2")
    for k, v in feats.items():
        print(f"  {k} = {v:+.4f}" + (f"   ({(hb + v) * 1e3:.1f} mm above the table)" if k.endswith("_z_m") and "size" not in k else ""))


if __name__ == "__main__":
    main()
