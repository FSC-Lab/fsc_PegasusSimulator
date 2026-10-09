#!/usr/bin/env python3
"""Box_Payload.STL (SolidWorks, mm, +Y up) -> Box_Payload.usda (m, +Z up) (2026-10-07).

    PYTHONNOUSERSITE=1 /usr/bin/python3 stl_to_usda.py [--stl ...] [--out ...] [--mass 0.2]

The payload is ONE rigid body (user, 2026-10-07): an open, perforated basket
110 x 110 x 65 mm with a flat wire HANGER on top -- two diagonal struts and a
cross strut meeting a vertical stem (3 x 2.5 mm) that ends in an arch (the
"horizontal bar"). The claw takes the stem with one finger on each side, both
under the arch; lifted, the arch rests on the fingers.

PAYLOAD FRAME (the root prim's own frame, unscaled):
  origin = the BOX centre (the basket's bounding box, feet included -- the
           mocap body obj_0 of the pick-and-place scene)
  +z     = up (CAD +Y)
  +x     = CAD +X = the hanger plane's NORMAL: the claw approaches along it
  +y     = CAD -Z = along the arch: the jaws close along it
The hanger plane sits at x = -27.5 mm (off the box centre, toward -x).

WHAT IS AUTHORED:
  Visual          the full STL mesh, welded, in metres (display only)
  Collision/basket one convex box, the basket's bounding box (it only ever
                  touches the pillar cap)
  Collision/hanger_NN one convex prism per triangle of the hanger's flat
                  face, extruded through its 2.5 mm thickness: the union IS
                  the hanger, exactly (no decomposition heuristics on the
                  part the fingers touch)
  MassAPI         mass, centre of mass and inertia of the CLOSED STL solid at
                  uniform density, scaled to --mass (PhysX would otherwise
                  derive them from the colliders, whose solid box is not the
                  hollow basket)
The rigid body, friction and the scene placement are the scene's job
(07_px4_t650_aerial_manipulator_pick_and_place.py).
"""
import argparse
import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ASSETS = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..",
    "extensions", "fsc_aerial_manipulation", "fsc_aerial_manipulation", "rotorcraft", "assets"))

# CAD frame -> payload frame: x = X, y = -Z, z = Y (a proper rotation)
R_CAD = np.array([[1.0, 0.0, 0.0],
                  [0.0, 0.0, -1.0],
                  [0.0, 1.0, 0.0]])


def read_stl(path):
    raw = open(path, "rb").read()
    n = int(np.frombuffer(raw[80:84], "<u4")[0])
    if 84 + 50 * n != len(raw):
        raise SystemExit(f"{path}: not a binary STL (or truncated)")
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(raw[84:84 + 50 * n], dt)["v"].astype(float)      # (n, 3, 3) mm


def solid_mass_props(T):
    """Volume, centroid and inertia (about the centroid, unit density) of a closed
    triangle mesh, by signed tetrahedra from the origin."""
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = v6.sum() / 6.0
    com = ((a + b + c) * v6[:, None]).sum(0) / (4.0 * v6.sum())
    # second moments: integral over each tet of x_i x_j (Tonon 2004)
    C = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            s = (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j]) * 2.0 \
                + a[:, i] * b[:, j] + b[:, i] * a[:, j] + a[:, i] * c[:, j] + c[:, i] * a[:, j] \
                + b[:, i] * c[:, j] + c[:, i] * b[:, j]
            C[i, j] = (v6 * s).sum() / 120.0
    C -= vol * np.outer(com, com)                       # about the centroid
    I = np.trace(C) * np.eye(3) - C
    return vol, com, I


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=os.path.join(ASSETS, "Box_Payload.STL"))
    ap.add_argument("--out", default=os.path.join(ASSETS, "Box_Payload.usda"))
    ap.add_argument("--mass", type=float, default=0.200)
    a = ap.parse_args()

    T_cad = read_stl(a.stl)
    V = T_cad.reshape(-1, 3)
    lo, hi = V.min(0), V.max(0)
    # the basket: its walls at mid-height give x / z (the hanger's strut feet
    # overhang the rim by 1.25 mm), its feet and rim give the height
    box_top = 959.319
    hang = (T_cad[:, :, 1] > box_top + 0.3).all(1)      # triangles wholly above the rim
    mid = V[(V[:, 1] > lo[1] + 10.0) & (V[:, 1] < box_top - 10.0)]
    box_lo = np.array([mid[:, 0].min(), lo[1], mid[:, 2].min()])
    box_hi = np.array([mid[:, 0].max(), box_top, mid[:, 2].max()])
    c_box = 0.5 * (box_lo + box_hi)
    if abs((box_hi - box_lo)[1] - 65.0) > 0.5 or abs((box_hi - box_lo)[0] - 110.0) > 0.5:
        raise SystemExit(f"unexpected basket size {box_hi - box_lo} mm -- not the 2026-10-07 payload?")

    def to_payload(p_mm):
        return ((np.asarray(p_mm) - c_box) @ R_CAD.T) * 1e-3

    T = to_payload(T_cad.reshape(-1, 3)).reshape(-1, 3, 3)            # m, payload frame
    vol, com, I = solid_mass_props(T)
    if vol <= 0:
        raise SystemExit("negative volume: the STL's winding is inverted")
    rho = a.mass / vol
    I = I * rho
    w, Q = np.linalg.eigh(I)
    if np.linalg.det(Q) < 0:
        Q[:, 0] = -Q[:, 0]
    qa = Gf.Quatf(Gf.Matrix3d(*Q.T.flatten().tolist()).ExtractRotation().GetQuat())

    # weld the visual mesh
    P = T.reshape(-1, 3)
    key = np.round(P / 1e-7).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    pts = np.zeros((len(uniq), 3))
    pts[inv] = P
    idx = inv.reshape(-1, 3)

    # the hanger: its +x face triangles (CAD +X), extruded through the thickness
    H = T_cad[hang]
    n = np.cross(H[:, 1] - H[:, 0], H[:, 2] - H[:, 0])
    nx = n[:, 0] / np.linalg.norm(n, axis=1)
    face = H[nx > 0.99]
    back = H[nx < -0.99]
    x_hi = float(np.median(face[:, :, 0]))              # the flat +x face
    x_lo = float(np.median(back[:, :, 0]))              # the flat -x face
    if not (0.0 < x_hi - x_lo < 10.0) or np.ptp(face[:, :, 0]) > 1e-3:
        raise SystemExit(f"hanger faces not flat/parallel: x {x_lo}..{x_hi}")

    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Box_Payload")
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    root.GetPrim().SetDocumentation(
        "Box payload, 2026-10-07. Frame: origin = basket centre, +z up, +x = hanger-plane normal "
        "(the claw approaches along it), +y = along the arch (the jaws close along it). "
        "Generated by docs/docs_aerial_manipulator/archive/pick_place_box_payload_20261007/tools/"
        "stl_to_usda.py from Box_Payload.STL -- regenerate, do not edit.")

    vis = UsdGeom.Mesh.Define(stage, "/Box_Payload/Visual")
    vis.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
    vis.CreateFaceVertexCountsAttr([3] * len(idx))
    vis.CreateFaceVertexIndicesAttr(idx.flatten().tolist())
    vis.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    vis.CreateDisplayColorAttr([Gf.Vec3f(0.80, 0.55, 0.25)])
    vis.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])

    UsdGeom.Scope.Define(stage, "/Box_Payload/Collision")

    def convex(path, verts):
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in verts])
        # faces are irrelevant for a convex hull, but USD wants a valid mesh:
        # fan every point set as one polygon list (hull is computed by PhysX)
        nv = len(verts)
        if nv == 8:
            f = [0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3]
            m.CreateFaceVertexCountsAttr([4] * 6)
        else:   # triangular prism 0,1,2 / 3,4,5
            f = [0, 2, 1, 3, 4, 5, 0, 1, 4, 3, 1, 2, 5, 4, 2, 0, 3, 5]
            m.CreateFaceVertexCountsAttr([3, 3, 4, 4, 4])
        m.CreateFaceVertexIndicesAttr(f)
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        return m

    bl, bh = to_payload(box_lo), to_payload(box_hi)
    blo, bhi = np.minimum(bl, bh), np.maximum(bl, bh)
    corners = [[x, y, z] for x in (blo[0], bhi[0]) for y in (blo[1], bhi[1]) for z in (blo[2], bhi[2])]
    convex("/Box_Payload/Collision/basket", corners)
    for k, t in enumerate(face):
        lo_t = t.copy(); lo_t[:, 0] = x_lo
        hi_t = t.copy(); hi_t[:, 0] = x_hi
        convex(f"/Box_Payload/Collision/hanger_{k:02d}", to_payload(np.vstack([lo_t, hi_t])))

    UsdPhysics.MassAPI.Apply(root.GetPrim())
    mapi = UsdPhysics.MassAPI(root.GetPrim())
    mapi.CreateMassAttr(float(a.mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, w)))
    mapi.CreatePrincipalAxesAttr(qa)

    # numbers the scene and the planner need, kept on the asset
    feats = {
        "fsc:hanger_x_m": float(to_payload([0.5 * (x_lo + x_hi), 0, c_box[2]])[0]),
        "fsc:hanger_thickness_m": float((x_hi - x_lo) * 1e-3),
        "fsc:box_half_height_m": float(0.5 * (box_hi - box_lo)[1] * 1e-3),
        "fsc:hanger_top_z_m": float((hi[1] - c_box[1]) * 1e-3),
    }
    for k, v in feats.items():
        root.GetPrim().CreateAttribute(k, Sdf.ValueTypeNames.Double, custom=True).Set(v)
    stage.GetRootLayer().Save()

    print(f"wrote {a.out}")
    print(f"  {len(idx)} visual triangles ({len(pts)} welded points), "
          f"1 basket + {len(face)} hanger convex colliders")
    print(f"  basket {np.round(bhi - blo, 4)} m, payload bbox {np.round(to_payload(hi) - to_payload(lo), 4)}")
    print(f"  solid volume {vol * 1e6:.1f} cm^3 -> density {rho:.0f} kg/m^3 for {a.mass} kg")
    print(f"  CoM {np.round(com * 1e3, 2)} mm, principal inertia {np.round(w * 1e6, 1)} g*m^2... (x1e-6 kg m^2)")
    for k, v in feats.items():
        print(f"  {k} = {v:+.4f}")


if __name__ == "__main__":
    main()
