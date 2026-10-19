#!/usr/bin/env python3
"""Basket + the screwed-on HOLDER -> Box_Payload_Holder.usda (2026-10-13).

    /usr/bin/python3 holder_stl_to_usda.py [--basket-stl ...] [--holder-stl ...] [--out ...]
                                           [--mass 0.2] [--fin centre|vehicle]

The pick-and-place payload of the 2026-10-10 flight: the basket of
Box_Payload.STL (its one-piece wire hanger dropped) carrying the user's
Holder.STL -- the same arch and struts, 10 mm deep instead of 2.5, with a FIN
under the arch (3 mm thick, 20 mm deep, 50 mm tall) that the jaws close on, and
two lugs screwed to the basket's side walls.

Box_Payload.STL holds TWO closed shells: the basket (110 x 110 x 65 mm) and the
old hanger. Only the basket is kept.

HOW THE HOLDER SITS ON THE BASKET (derived, checked below, not assumed):
  * its lugs take the place of the old hanger's feet: same outer width
    (115.25 mm), same 15 mm height, and each has two 3.3 mm holes 5 mm apart.
    The old feet have the same two holes. So the hole line of the holder is put
    on the hole line of the old feet: 25 mm behind the basket centre, 23 and
    28 mm above it. That fixes the height and the fore-aft position.
  * which way it faces is the one thing the holes do not fix (they are on the
    lugs' centre line). --fin centre (default): the fin points to the basket
    centre, away from the approaching vehicle. The flat part then sits with its
    back face flush with the lugs' back face, as the old hanger did with its
    feet, its mid-plane stays where the old hanger's was (27.5 mm behind the
    centre), and the claw point of EE Offset x = -0.02 lands inside the fin.
    --fin vehicle turns it round (fin toward the vehicle).

PAYLOAD FRAME (the root prim's own frame, unscaled), as in Box_Payload.usda:
  origin = the BASKET centre (the mocap body obj_0 of the scene)
  +z     = up
  +x     = the hanger plane's normal: the claw approaches along it
  +y     = along the arch: the jaws close along it

WHAT IS AUTHORED:
  Visual/basket, Visual/holder   the two STL meshes, welded, in metres
  Collision/basket               one convex box (it only touches the platform)
  Collision/slab_NN              the holder's flat part (arch, stem, struts,
                                 the lugs' rear 10 mm): one convex prism per
                                 triangle of its back face, extruded through
                                 the 10 mm -- the union IS the part
  Collision/fin                  the fin, a convex prism over its outline
  Collision/lug_0, lug_1         the two lugs, boxes
  MassAPI                        mass, centre of mass and inertia of the two
                                 closed solids at one uniform density, scaled
                                 to --mass
  fsc:* attributes               the numbers the scene needs (fin and stem
                                 extents for the grip pad, heights)
The rigid body, friction, the grip pad and the placement are the scene's job
(07_px4_t650_aerial_manipulator_pick_and_place.py).
"""
import argparse
import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ASSETS = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..",
    "extensions", "fsc_aerial_manipulation", "fsc_aerial_manipulation", "rotorcraft", "assets"))

# old one-piece CAD frame (SolidWorks, mm, +Y up) -> payload frame: x = X, y = -Z, z = Y
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


def shells(T, tol=1e-3):
    """Connected components of a triangle soup (vertices welded at tol)."""
    key = np.round(T.reshape(-1, 3) / tol).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    idx = inv.reshape(-1, 3)
    parent = np.arange(inv.max() + 1)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b, c in idx:
        ra = find(a)
        parent[find(b)] = ra
        parent[find(c)] = ra
    root = np.array([find(a) for a in idx[:, 0]])
    return [T[root == r] for r in np.unique(root)]


def solid_integrals(T):
    """Volume, first moment and second-moment matrix (about the ORIGIN, unit
    density) of a closed triangle mesh, by signed tetrahedra (Tonon 2004)."""
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = v6.sum() / 6.0
    m1 = ((a + b + c) * v6[:, None]).sum(0) / 24.0
    C = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            s = (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j]) * 2.0 \
                + a[:, i] * b[:, j] + b[:, i] * a[:, j] + a[:, i] * c[:, j] + c[:, i] * a[:, j] \
                + b[:, i] * c[:, j] + c[:, i] * b[:, j]
            C[i, j] = (v6 * s).sum() / 120.0
    return vol, m1, C


def weld(T, tol=1e-7):
    P = T.reshape(-1, 3)
    key = np.round(P / tol).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    pts = np.zeros((len(uniq), 3))
    pts[inv] = P
    return pts, inv.reshape(-1, 3)


def convex_outline(pts2):
    """Convex hull of 2-D points, counter-clockwise (monotone chain)."""
    p = np.unique(np.round(pts2, 6), axis=0)
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def half(seq):
        h = []
        for q in seq:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (q[1] - h[-2][1])
                                   - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0])) <= 1e-9:
                h.pop()
            h.append(q)
        return h
    lo, up = half(p), half(p[::-1])
    return np.array(lo[:-1] + up[:-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--basket-stl", default=os.path.join(ASSETS, "Box_Payload.STL"))
    ap.add_argument("--holder-stl", default=os.path.join(ASSETS, "Holder.STL"))
    ap.add_argument("--out", default=os.path.join(ASSETS, "Box_Payload_Holder.usda"))
    ap.add_argument("--mass", type=float, default=0.200)
    ap.add_argument("--fin", choices=("centre", "vehicle"), default="centre",
                    help="which way the fin points: to the basket centre (default) or to the vehicle")
    a = ap.parse_args()

    # ── the basket and the old hanger: two closed shells of the old file ──
    parts = sorted(shells(read_stl(a.basket_stl)), key=len, reverse=True)
    if len(parts) != 2:
        raise SystemExit(f"{a.basket_stl}: expected 2 shells (basket + old hanger), found {len(parts)}")
    basket_cad, old_cad = parts
    lo, hi = basket_cad.reshape(-1, 3).min(0), basket_cad.reshape(-1, 3).max(0)
    if np.abs((hi - lo) - np.array([110.0, 65.0, 110.0])).max() > 0.5:
        raise SystemExit(f"unexpected basket size {hi - lo} mm -- not the 2026-10-07 basket?")
    c_box = 0.5 * (lo + hi)
    basket = ((basket_cad.reshape(-1, 3) - c_box) @ R_CAD.T).reshape(-1, 3, 3)     # mm, payload frame
    old = ((old_cad.reshape(-1, 3) - c_box) @ R_CAD.T).reshape(-1, 3, 3)
    half = 0.5 * (hi - lo)[[0, 2, 1]]                                               # x, y, z half sizes

    # the old feet: the part of the old hanger beside the walls; their hole line
    ov = old.reshape(-1, 3)
    foot = ov[ov[:, 2] < half[2] + 0.6]
    foot_x = 0.5 * (foot[:, 0].min() + foot[:, 0].max())          # the hole line, fore-aft
    foot_z0, foot_z1 = foot[:, 2].min(), foot[:, 2].max()
    old_w = np.ptp(ov[:, 1])

    # ── the holder, in its own file frame (mm) ──
    H = read_stl(a.holder_stl)
    if len(shells(H)) != 1:
        raise SystemExit(f"{a.holder_stl}: expected one closed shell")
    hv = H.reshape(-1, 3)
    hlo, hhi = hv.min(0), hv.max(0)
    if np.abs(hlo).max() > 1e-3:
        raise SystemExit(f"{a.holder_stl}: expected its bounding box to start at the origin, got {hlo}")
    W, D, Ht = hhi
    lug = hv[hv[:, 2] > Ht - 14.0]                               # the lugs: the last 15 mm in z
    lug_y0, lug_y1 = lug[:, 1].min(), lug[:, 1].max()
    lug_h = Ht - hv[(np.abs(hv[:, 0]) < 1e-3)][:, 2].min()        # the outer face x = 0 is a lug face
    lug_t = hv[(hv[:, 2] > Ht - 1.0) & (hv[:, 0] < 0.5 * W)][:, 0].max()
    # the flat part: the back face y = D carries the whole silhouette
    n = np.cross(H[:, 1] - H[:, 0], H[:, 2] - H[:, 0])
    ny = n[:, 1] / np.linalg.norm(n, axis=1)
    back = H[(ny > 0.99) & (np.abs(H[:, :, 1] - D).max(1) < 1e-3)]
    area = 0.5 * np.linalg.norm(np.cross(back[:, 1] - back[:, 0], back[:, 2] - back[:, 0]), axis=1).sum()
    # the fin: everything in front of the flat part that is not a lug
    fin = hv[(hv[:, 1] < lug_y0 - 1e-3)]
    fin_x0, fin_x1 = fin[:, 0].min(), fin[:, 0].max()
    fin_z0, fin_z1 = fin[:, 2].min(), fin[:, 2].max()
    # the flat part's front face, the plane the fin stands on: the -y plane of largest area
    front = H[(ny < -0.99)]
    fy = np.round(front[:, :, 1].mean(1), 3)
    areas = {float(v): 0.5 * np.linalg.norm(np.cross(front[fy == v][:, 1] - front[fy == v][:, 0],
                                                      front[fy == v][:, 2] - front[fy == v][:, 0]), axis=1).sum()
             for v in np.unique(fy)}
    slab_front = max(areas, key=areas.get)
    slab_t = D - slab_front
    v_h, _, _ = solid_integrals(H)
    v_model = area * slab_t + 2.0 * lug_t * lug_h * (slab_front - lug_y0)
    fin_outline = convex_outline(hv[(hv[:, 1] < slab_front + 1e-3) & (np.abs(hv[:, 0] - fin_x0) < 1e-3) &
                                    (hv[:, 2] > fin_z0 - 1e-3) & (hv[:, 2] < fin_z1 + 1e-3)][:, [1, 2]])
    fa = 0.5 * abs(np.dot(fin_outline[:, 0], np.roll(fin_outline[:, 1], -1)) -
                   np.dot(fin_outline[:, 1], np.roll(fin_outline[:, 0], -1)))
    v_model += fa * (fin_x1 - fin_x0)
    holes = v_model - v_h                                          # what the screw holes take out

    checks = [
        ("outer width = the old hanger's", abs(W - old_w) < 0.05, f"{W:.3f} vs {old_w:.3f} mm"),
        ("lug height = the old feet's", abs(lug_h - (foot_z1 - foot_z0)) < 0.05, f"{lug_h:.3f} vs {foot_z1 - foot_z0:.3f} mm"),
        ("lugs' inner gap takes the basket", 0.0 <= (W - 2 * lug_t) - 2 * half[1] < 1.0,
         f"{W - 2 * lug_t:.2f} vs basket {2 * half[1]:.2f} mm"),
        ("flat part + fin + lugs = the solid, less four screw holes", 40.0 < holes < 130.0,
         f"model {v_model:.1f} - solid {v_h:.1f} = {holes:.1f} mm^3 (4 holes of 3.3 mm through {lug_t} mm: "
         f"{4 * np.pi * 1.65 ** 2 * lug_t:.1f})"),
    ]
    for name, ok, txt in checks:
        print(f"  [{'ok' if ok else 'FAIL'}] {name}: {txt}")
    if not all(ok for _, ok, _ in checks):
        raise SystemExit("the holder is not the part this converter was written for -- see the FAIL lines")

    # ── holder file frame -> payload frame (mm) ──
    hole_y = 0.5 * (lug_y0 + lug_y1)                               # the lugs' centre line
    if a.fin == "centre":                                          # back face toward the vehicle (-x)
        Rm = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        t = np.array([foot_x + hole_y, 0.5 * W, foot_z0 + Ht])
    else:                                                          # turned round: fin toward the vehicle
        Rm = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        t = np.array([foot_x - hole_y, -0.5 * W, foot_z0 + Ht])
    assert abs(np.linalg.det(Rm) - 1.0) < 1e-12

    def to_p(p_file):
        return np.asarray(p_file, float) @ Rm.T + t                # mm, payload frame

    holder = to_p(hv).reshape(-1, 3, 3)

    # ── mass properties: two closed solids, one density ──
    vb, m1b, Cb = solid_integrals(basket * 1e-3)
    vh, m1h, Ch = solid_integrals(holder * 1e-3)
    if vb <= 0 or vh <= 0:
        raise SystemExit("negative volume: a mesh's winding is inverted")
    vol = vb + vh
    com = (m1b + m1h) / vol
    C = Cb + Ch - vol * np.outer(com, com)
    rho = a.mass / vol
    I = (np.trace(C) * np.eye(3) - C) * rho
    w, Q = np.linalg.eigh(I)
    if np.linalg.det(Q) < 0:
        Q[:, 0] = -Q[:, 0]
    qa = Gf.Quatf(Gf.Matrix3d(*Q.T.flatten().tolist()).ExtractRotation().GetQuat())

    # ── the stage ──
    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Box_Payload")
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    root.GetPrim().SetDocumentation(
        "Basket + screwed-on holder (fin under the arch), 2026-10-13. Frame: origin = basket centre, +z up, "
        "+x = hanger-plane normal (the claw approaches along it), +y = along the arch (the jaws close along it). "
        "Generated by docs/docs_aerial_manipulator/archive/pick_place_holder_20261013/tools/"
        f"holder_stl_to_usda.py from Box_Payload.STL (basket shell) and Holder.STL (--fin {a.fin}) -- "
        "regenerate, do not edit.")

    UsdGeom.Scope.Define(stage, "/Box_Payload/Visual")
    for name, mesh, rgb in (("basket", basket, (0.80, 0.55, 0.25)), ("holder", holder, (0.20, 0.22, 0.26))):
        pts, idx = weld(mesh * 1e-3)
        vis = UsdGeom.Mesh.Define(stage, f"/Box_Payload/Visual/{name}")
        vis.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
        vis.CreateFaceVertexCountsAttr([3] * len(idx))
        vis.CreateFaceVertexIndicesAttr(idx.flatten().tolist())
        vis.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        vis.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        vis.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])

    UsdGeom.Scope.Define(stage, "/Box_Payload/Collision")

    def convex(path, verts_mm, counts, faces):
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in np.asarray(verts_mm) * 1e-3])
        m.CreateFaceVertexCountsAttr(counts)
        m.CreateFaceVertexIndicesAttr(faces)
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")

    def box(path, lo_mm, hi_mm):
        l, h = np.minimum(lo_mm, hi_mm), np.maximum(lo_mm, hi_mm)
        v = [[x, y, z] for x in (l[0], h[0]) for y in (l[1], h[1]) for z in (l[2], h[2])]
        convex(path, v, [4] * 6, [0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3])

    def prism(path, poly_a, poly_b):
        """Two congruent N-gons (corresponding vertices) joined by N quads."""
        k = len(poly_a)
        counts = [k, k] + [4] * k
        faces = list(range(k - 1, -1, -1)) + list(range(k, 2 * k))
        for i in range(k):
            j = (i + 1) % k
            faces += [i, j, k + j, k + i]
        convex(path, np.vstack([poly_a, poly_b]), counts, faces)

    box("/Box_Payload/Collision/basket", -half, half)
    for k, tri in enumerate(back):                                 # the flat part, exactly
        near = tri.copy()
        near[:, 1] = slab_front
        prism(f"/Box_Payload/Collision/slab_{k:02d}", to_p(tri), to_p(near))
    out = fin_outline
    if len(out) > 28:                                              # PhysX hulls: keep it small
        out = out[np.round(np.linspace(0, len(out) - 1, 28)).astype(int)]
    pa = np.column_stack([np.full(len(out), fin_x0), out[:, 0], out[:, 1]])
    pb = np.column_stack([np.full(len(out), fin_x1), out[:, 0], out[:, 1]])
    prism("/Box_Payload/Collision/fin", to_p(pa), to_p(pb))
    for k, (x0, x1) in enumerate(((0.0, lug_t), (W - lug_t, W))):
        box(f"/Box_Payload/Collision/lug_{k}", to_p([x0, lug_y0, Ht - lug_h]), to_p([x1, lug_y1, Ht]))

    UsdPhysics.MassAPI.Apply(root.GetPrim())
    mapi = UsdPhysics.MassAPI(root.GetPrim())
    mapi.CreateMassAttr(float(a.mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, w)))
    mapi.CreatePrincipalAxesAttr(qa)

    # the numbers the scene needs, kept on the asset (m, payload frame)
    def span(p0, p1, axis):
        v = sorted((to_p(p0)[axis] * 1e-3, to_p(p1)[axis] * 1e-3))
        return float(v[0]), float(v[1])
    slab_x = span([0, slab_front, 0], [0, D, 0], 0)
    fin_x = span([0, fin[:, 1].min(), 0], [0, slab_front, 0], 0)
    fin_z = span([0, 0, fin_z0], [0, 0, fin_z1], 2)
    # the arch's flat underside: in the file (upside down) its +z face next to z = 0
    nz = n[:, 2] / np.linalg.norm(n, axis=1)
    under = H[(nz > 0.99) & (H[:, :, 2].mean(1) < 5.0)]
    arch_under = float(to_p([0, 0, np.median(under[:, :, 2])])[2] * 1e-3)
    feats = {
        "fsc:box_half_height_m": float(half[2] * 1e-3),
        "fsc:hanger_x_m": 0.5 * (slab_x[0] + slab_x[1]),          # the flat part's mid-plane
        "fsc:hanger_thickness_m": slab_x[1] - slab_x[0],
        "fsc:hanger_top_z_m": float(to_p([0, 0, 0])[2] * 1e-3),
        "fsc:arch_underside_z_m": arch_under,
        "fsc:stem_x_lo_m": slab_x[0], "fsc:stem_x_hi_m": slab_x[1],
        "fsc:fin_x_lo_m": fin_x[0], "fsc:fin_x_hi_m": fin_x[1],
        "fsc:fin_z_lo_m": fin_z[0], "fsc:fin_z_hi_m": fin_z[1],
        "fsc:fin_thickness_m": float((fin_x1 - fin_x0) * 1e-3),
        "fsc:fin_centre_y_m": float(to_p([0.5 * (fin_x0 + fin_x1), 0, 0])[1] * 1e-3),
        "fsc:hole_x_m": float(foot_x * 1e-3),
        "fsc:holder_mass_fraction": float(vh / vol),
    }
    for k, v in feats.items():
        root.GetPrim().CreateAttribute(k, Sdf.ValueTypeNames.Double, custom=True).Set(v)
    root.GetPrim().CreateAttribute("fsc:fin_points_to", Sdf.ValueTypeNames.String, custom=True).Set(a.fin)
    stage.GetRootLayer().Save()

    hp = holder.reshape(-1, 3)
    print(f"wrote {a.out}")
    print(f"  basket {np.round(2 * half, 2)} mm ({len(basket)} triangles), holder {len(holder)} triangles, "
          f"fin toward the {a.fin}; colliders: 1 basket + {len(back)} slab prisms + 1 fin ({len(out)}-gon) + 2 lugs")
    print(f"  holder in the payload frame [mm]: x {hp[:, 0].min():+.2f}..{hp[:, 0].max():+.2f}, "
          f"y {hp[:, 1].min():+.2f}..{hp[:, 1].max():+.2f}, z {hp[:, 2].min():+.2f}..{hp[:, 2].max():+.2f}; "
          f"screw-hole line x {foot_x:+.2f}, z {foot_z0 + 5:+.1f} / {foot_z0 + 10:+.1f} (the old feet's)")
    print(f"  old hanger for comparison: plate mid-plane x {ov[ov[:, 2] > 60][:, 0].mean():+.2f} mm, top z {ov[:, 2].max():+.2f}")
    print(f"  volume basket {vb * 1e6:.1f} + holder {vh * 1e6:.1f} cm^3 -> density {rho:.0f} kg/m^3 for {a.mass} kg "
          f"(holder {1e3 * a.mass * vh / vol:.1f} g)")
    print(f"  CoM {np.round(com * 1e3, 2)} mm, principal inertia {np.round(w * 1e6, 1)} x1e-6 kg m^2")
    for k, v in feats.items():
        print(f"  {k} = {v:+.5f}")


if __name__ == "__main__":
    main()
