#!/usr/bin/env python3
"""Box_Assembly.stl (the real payload, metres, +z up) -> Box_Payload_Assembly.usda (2026-10-14).

    /usr/bin/python3 assembly_stl_to_usda.py [--stl ...] [--out ...] [--mass 0.2] [--marker-mass 0.003]

The payload flown from 2026-10-14: the basket, the screwed-on holder WITHOUT
its arch (the T's horizontal bar removed: the jaws clamp the stem + fin and
release it by opening), five mocap markers on posts and two marker brackets.
The file holds them as separate closed shells; this script tells them apart by
shape (and refuses a file that does not fit), so no part is guessed:
  basket   the largest shell: 110 x 110 x 65 mm body (+ its marker arms / post)
  holder   the 115 x 30 x 181.5 mm shell: stem + fin (3 mm thick, 20 deep)
  markers  the 30 x 30 x 55 mm shells: a 30 mm ball on a post
  brackets the rest

PAYLOAD FRAME (as Box_Payload*.usda; the scene's obj_0):
  origin = the BASKET centre
  +z     = up
  +x     = the direction the CLAW moves when it slides in: the holder sits on
           the basket's -x side and its fin points to -x, toward the vehicle
           (read off the assembly: the fin points AWAY from the basket centre)
  +y     = the jaws' closing axis (the fin's 3 mm)

COLLIDERS: the basket body (box) + boxes over its arms / post; the holder
exactly (its flat part as one prism per triangle of its back face, the fin as
a prism over its outline, the lugs as boxes); each marker a sphere (the ball)
+ a box (the post); each bracket a box. MASS: the markers are hollow balls --
--marker-mass each -- and the printed parts share the rest of --mass at one
density (their closed solids give the CoM and inertia).
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
    return np.frombuffer(raw[84:84 + 50 * n], dt)["v"].astype(float)


def shells(T, tol=1e-3):
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
    """Volume, first moment, second-moment matrix about the origin (unit density)."""
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


def hull2d(pts2):
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


def area_by_plane(T, axis, sign, tol=0.99):
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    a = 0.5 * np.linalg.norm(n, axis=1)
    m = (n[:, axis] / np.maximum(2 * a, 1e-12)) * sign > tol
    pos = np.round(T[m][:, :, axis].mean(1), 2)
    out = {}
    for p, ar in zip(pos, a[m]):
        out[float(p)] = out.get(float(p), 0.0) + ar
    return out, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=os.path.join(ASSETS, "Box_Assembly.stl"))
    ap.add_argument("--out", default=os.path.join(ASSETS, "Box_Payload_Assembly.usda"))
    ap.add_argument("--mass", type=float, default=0.200, help="the whole payload [kg]")
    ap.add_argument("--marker-mass", type=float, default=0.003, help="each mocap marker (ball + post) [kg]")
    a = ap.parse_args()

    T = read_stl(a.stl) * 1e3                                   # mm
    parts = shells(T)
    bbox = lambda S: (S.reshape(-1, 3).min(0), S.reshape(-1, 3).max(0))
    vols = [abs(solid_integrals(S)[0]) for S in parts]
    basket = parts[int(np.argmax(vols))]
    holder, markers, brackets = None, [], []
    for S, v in zip(parts, vols):
        if S is basket:
            continue
        d = np.sort(bbox(S)[1] - bbox(S)[0])
        if np.allclose(d, [30.0, 115.25, 181.5], atol=1.5):
            if holder is not None:
                raise SystemExit("two holder-shaped shells")
            holder = S
        elif np.allclose(d, [30.0, 30.0, 55.0], atol=1.0):
            markers.append(S)
        else:
            brackets.append(S)
    if holder is None or len(markers) != 5:
        raise SystemExit(f"expected one holder and five markers, found holder={holder is not None}, "
                         f"markers={len(markers)} -- not the 2026-10-14 assembly?")

    # the basket body: the walls just under the rim span 110 x 110, 65 mm tall
    B = basket.reshape(-1, 3)
    z_lo = B[:, 2].min()
    rim = B[(B[:, 2] > z_lo + 55.0) & (B[:, 2] < z_lo + 64.9)]     # the top 10 mm of the walls (no arms there)
    over = B[(B[:, 2] > z_lo + 65.5) & (B[:, 2] < z_lo + 80.0)]    # above the rim: only the marker post
    span = rim[:, :2].max(0) - rim[:, :2].min(0)
    if np.abs(span - 110.0).max() > 0.6 or (len(over) and (over[:, :2].max(0) - over[:, :2].min(0)).min() > 60):
        raise SystemExit(f"the basket body is not 110 x 110 x 65 mm (span {span} at its rim)")
    c = np.array([*(0.5 * (rim[:, :2].max(0) + rim[:, :2].min(0))), z_lo + 32.5])
    half = np.array([55.0, 55.0, 32.5])

    # the holder: its 30 mm depth axis must be x or y; the side it sits on and
    # where its fin points decide the frame
    H = holder.reshape(-1, 3)
    hd = H.max(0) - H.min(0)
    ax = int(np.argmin(hd[:2]))                                  # depth axis (stl)
    if abs(hd[ax] - 30.0) > 1.0:
        raise SystemExit(f"holder depth {hd[ax]} mm along stl axis {ax}")
    side = np.sign(0.5 * (H[:, ax].max() + H[:, ax].min()) - c[ax])
    near = H[:, ax].min() if side > 0 else H[:, ax].max()        # the face toward the basket
    far = H[:, ax].max() if side > 0 else H[:, ax].min()
    # the fin = the 3 mm plate; it must reach the FAR side (it points outward)
    thin = 1 - ax
    fin_pts = H[np.abs(H[:, ax] - far) < 0.5]
    if np.ptp(fin_pts[:, thin]) > 3.5:
        raise SystemExit("the holder's far side is not the 3 mm fin edge -- fin points inward?")
    # payload axes: +x = toward the basket centre from the holder side
    ex = np.zeros(3)
    ex[ax] = -side
    ez = np.array([0.0, 0.0, 1.0])
    ey = np.cross(ez, ex)
    R = np.vstack([ex, ey, ez])                                  # rows: payload axes in stl coords

    def to_p(P):
        return (np.asarray(P, float) - c) @ R.T                  # mm, payload frame

    P_all = {"basket": to_p(basket.reshape(-1, 3)).reshape(-1, 3, 3),
             "holder": to_p(holder.reshape(-1, 3)).reshape(-1, 3, 3)}
    Hp = P_all["holder"]
    Hv = Hp.reshape(-1, 3)
    x_back = Hv[:, 0].max()                                      # the face toward the centre
    # the slab front: the largest -x face; the lug fronts: the next -x plane
    fronts, _ = area_by_plane(Hp, 0, -1)
    fronts = {k: v for k, v in fronts.items() if v > 20.0}
    x_front = max(fronts, key=fronts.get)
    lug_planes = [k for k in fronts if k < x_front - 1.0 and k > Hv[:, 0].min() + 1.0]
    backs, bm = area_by_plane(Hp, 0, +1)
    back = Hp[bm & (np.abs(Hp[:, :, 0].mean(1) - x_back) < 0.05)]
    back_area = 0.5 * np.linalg.norm(np.cross(back[:, 1] - back[:, 0], back[:, 2] - back[:, 0]), axis=1).sum()
    # the fin: everything beyond the slab front, above the lugs
    lug_z_top = Hv[:, 2].min() + 15.0
    finv = Hv[(Hv[:, 0] < x_front - 0.05) & (Hv[:, 2] > lug_z_top + 1.0)]
    fy0, fy1 = finv[:, 1].min(), finv[:, 1].max()
    fz0, fz1 = finv[:, 2].min(), finv[:, 2].max()
    fx0 = finv[:, 0].min()
    face = Hv[(np.abs(Hv[:, 1] - fy0) < 0.05) & (Hv[:, 0] <= x_front + 0.05) & (Hv[:, 2] > fz0 - 0.05)]
    # the stem and the fin are ONE coplanar face, so its triangles need not have
    # vertices on the fin's root line x = x_front: close the outline there
    outline = hull2d(np.vstack([face[:, [0, 2]], [[x_front, fz0], [x_front, fz1]]]))   # (x, z)
    fin_area = 0.5 * abs(np.dot(outline[:, 0], np.roll(outline[:, 1], -1)) -
                         np.dot(outline[:, 1], np.roll(outline[:, 0], -1)))
    # the lugs: the bottom 15 mm, one each side across y
    lugv = Hv[Hv[:, 2] < lug_z_top + 0.05]
    lugs = [lugv[lugv[:, 1] < 0.0], lugv[lugv[:, 1] > 0.0]]
    slab_t = x_back - x_front
    v_h = abs(solid_integrals(Hp)[0])
    lug_extra = sum((x_front - L[:, 0].min()) * np.ptp(L[:, 1]) * np.ptp(L[:, 2]) for L in lugs)
    v_model = back_area * slab_t + fin_area * (fy1 - fy0) + lug_extra
    holes = v_model - v_h
    print(f"  (holder pieces: back face {back_area:.1f} mm^2 x slab {slab_t:.2f}, fin {fin_area:.1f} mm^2 x "
          f"{fy1 - fy0:.2f} ({len(outline)}-gon, x {fx0:.2f}..{x_front:.2f}, z {fz0:.2f}..{fz1:.2f}), lugs +{lug_extra:.1f})")
    checks = [
        ("basket body 110 x 110 x 65", True, f"centre {np.round(c, 2)} mm (stl)"),
        ("holder slab 10 mm, fin 20 x 3 mm", abs(slab_t - 10.0) < 0.2 and abs((x_front - fx0) - 20.0) < 0.3
         and abs((fy1 - fy0) - 3.0) < 0.1, f"slab {slab_t:.2f}, fin {x_front - fx0:.2f} x {fy1 - fy0:.2f} mm"),
        ("the holder's screw-hole line 25 mm from the basket centre", abs(0.5 * (x_back + min(lug_planes or [x_front])) + 25.0) < 0.5,
         f"lugs x {min(lug_planes or [x_front]):.2f}..{x_back:.2f} mm"),
        ("no arch: the holder is no wider than its fin at the top", np.ptp(Hv[Hv[:, 2] > fz1 - 2.0][:, 1]) < 3.5,
         f"top {np.ptp(Hv[Hv[:, 2] > fz1 - 2.0][:, 1]):.2f} mm wide"),
        ("flat part + fin + lugs = the solid, less the screw holes", 40.0 < holes < 130.0,
         f"model {v_model:.1f} - solid {v_h:.1f} = {holes:.1f} mm^3"),
    ]
    for name, ok, txt in checks:
        print(f"  [{'ok' if ok else 'FAIL'}] {name}: {txt}")
    if not all(ok for _, ok, _ in checks):
        raise SystemExit("the assembly is not the part this converter was written for -- see the FAIL lines")

    # mass: printed parts at one density, markers at --marker-mass each
    printed = [P_all["basket"], Hp] + [to_p(S.reshape(-1, 3)).reshape(-1, 3, 3) for S in brackets]
    mk = [to_p(S.reshape(-1, 3)).reshape(-1, 3, 3) for S in markers]
    m_printed = a.mass - len(mk) * a.marker_mass
    if m_printed <= 0:
        raise SystemExit("--mass is less than the markers")
    vp = sum(solid_integrals(S * 1e-3)[0] for S in printed)
    rho = m_printed / vp
    M, M1, M2 = 0.0, np.zeros(3), np.zeros((3, 3))
    for S in printed:
        v, m1, C = solid_integrals(S * 1e-3)
        M += rho * v
        M1 += rho * m1
        M2 += rho * C
    for S in mk:
        v, m1, C = solid_integrals(S * 1e-3)
        k = a.marker_mass / v
        M += k * v
        M1 += k * m1
        M2 += k * C
    com = M1 / M
    C = M2 - M * np.outer(com, com)
    I = np.trace(C) * np.eye(3) - C
    w, Q = np.linalg.eigh(I)
    if np.linalg.det(Q) < 0:
        Q[:, 0] = -Q[:, 0]
    qa = Gf.Quatf(Gf.Matrix3d(*Q.T.flatten().tolist()).ExtractRotation().GetQuat())

    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Box_Payload")
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    root.GetPrim().SetDocumentation(
        "The 2026-10-14 payload: basket + holder WITHOUT its arch (clamp grasp on the fin) + 5 mocap markers "
        "+ 2 brackets. Frame: origin = basket centre, +z up, +x = the claw's slide-in direction (the fin points "
        "to -x), +y = the jaws' closing axis. Generated by docs/docs_aerial_manipulator/archive/"
        "pick_place_drop_20261014/tools/assembly_stl_to_usda.py from Box_Assembly.stl -- regenerate, do not edit.")
    UsdGeom.Scope.Define(stage, "/Box_Payload/Visual")
    vis_parts = [("basket", P_all["basket"], (0.80, 0.55, 0.25)), ("holder", Hp, (0.20, 0.22, 0.26))]
    vis_parts += [(f"marker_{k}", S, (0.92, 0.92, 0.92)) for k, S in enumerate(mk)]
    vis_parts += [(f"bracket_{k}", to_p(S.reshape(-1, 3)).reshape(-1, 3, 3), (0.30, 0.30, 0.32))
                  for k, S in enumerate(brackets)]
    for name, mesh, rgb in vis_parts:
        pts, idx = weld(mesh * 1e-3)
        vm = UsdGeom.Mesh.Define(stage, f"/Box_Payload/Visual/{name}")
        vm.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
        vm.CreateFaceVertexCountsAttr([3] * len(idx))
        vm.CreateFaceVertexIndicesAttr(idx.flatten().tolist())
        vm.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        vm.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        vm.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])

    UsdGeom.Scope.Define(stage, "/Box_Payload/Collision")

    def convex(path, verts_mm, counts, faces):
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in np.asarray(verts_mm) * 1e-3])
        m.CreateFaceVertexCountsAttr(counts)
        m.CreateFaceVertexIndicesAttr(faces)
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")

    def box(path, lo, hi):
        l, h = np.minimum(lo, hi), np.maximum(lo, hi)
        v = [[x, y, z] for x in (l[0], h[0]) for y in (l[1], h[1]) for z in (l[2], h[2])]
        convex(path, v, [4] * 6, [0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3])

    def prism(path, pa, pb):
        k = len(pa)
        faces = list(range(k - 1, -1, -1)) + list(range(k, 2 * k))
        for i in range(k):
            j = (i + 1) % k
            faces += [i, j, k + j, k + i]
        convex(path, np.vstack([pa, pb]), [k, k] + [4] * k, faces)

    def sphere(path, centre_mm, r_mm):
        s = UsdGeom.Sphere.Define(stage, path)
        s.CreateRadiusAttr(float(r_mm * 1e-3))
        UsdGeom.Xformable(s).AddTranslateOp().Set(Gf.Vec3d(*map(float, np.asarray(centre_mm) * 1e-3)))
        s.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(s.GetPrim())

    n_col = 0
    box("/Box_Payload/Collision/basket", -half, half)
    n_col += 1
    # the basket shell's parts outside its body: one box per side they stick out of
    Bp = P_all["basket"].reshape(-1, 3)
    out = Bp[(np.abs(Bp) > half + 0.5).any(1)]
    for nm, sel in (("arm_px", out[:, 0] > half[0] + 0.5), ("arm_nx", out[:, 0] < -half[0] - 0.5),
                    ("arm_py", out[:, 1] > half[1] + 0.5), ("arm_ny", out[:, 1] < -half[1] - 0.5),
                    ("post_top", (out[:, 2] > half[2] + 0.5) & (np.abs(out[:, :2]) <= half[:2] + 0.5).all(1))):
        if sel.sum() >= 4:
            box(f"/Box_Payload/Collision/basket_{nm}", out[sel].min(0), out[sel].max(0))
            n_col += 1
    for k, tri in enumerate(back):
        near_t = tri.copy()
        near_t[:, 0] = x_front
        prism(f"/Box_Payload/Collision/slab_{k:02d}", tri, near_t)
        n_col += 1
    o = outline if len(outline) <= 28 else outline[np.round(np.linspace(0, len(outline) - 1, 28)).astype(int)]
    prism("/Box_Payload/Collision/fin", np.column_stack([o[:, 0], np.full(len(o), fy0), o[:, 1]]),
          np.column_stack([o[:, 0], np.full(len(o), fy1), o[:, 1]]))
    n_col += 1
    for k, L in enumerate(lugs):
        box(f"/Box_Payload/Collision/lug_{k}", L.min(0), L.max(0))
        n_col += 1
    for k, S in enumerate(mk):
        V = S.reshape(-1, 3)
        lo, hi = V.min(0), V.max(0)
        long = int(np.argmax(hi - lo))
        r = 0.5 * np.sort(hi - lo)[0]
        far_end = hi[long] if abs(hi[long]) > abs(lo[long]) else lo[long]   # the ball: away from the basket
        cen = 0.5 * (lo + hi)
        cen[long] = far_end - np.sign(far_end) * r
        sphere(f"/Box_Payload/Collision/marker_{k}_ball", cen, r)
        post = V[np.abs(V[:, long] - cen[long]) > r + 0.5]
        post = post[np.sign(post[:, long] - cen[long]) != np.sign(far_end)]
        if len(post) >= 4:
            box(f"/Box_Payload/Collision/marker_{k}_post", post.min(0), post.max(0))
            n_col += 1
        n_col += 1
    for k, S in enumerate(brackets):
        V = to_p(S.reshape(-1, 3))
        box(f"/Box_Payload/Collision/bracket_{k}", V.min(0), V.max(0))
        n_col += 1

    UsdPhysics.MassAPI.Apply(root.GetPrim())
    mapi = UsdPhysics.MassAPI(root.GetPrim())
    mapi.CreateMassAttr(float(a.mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, w)))
    mapi.CreatePrincipalAxesAttr(qa)

    allv = np.vstack([p.reshape(-1, 3) for _, p, _ in vis_parts])
    feats = {
        "fsc:box_half_height_m": half[2] * 1e-3,
        "fsc:hanger_x_m": 0.5 * (x_back + x_front) * 1e-3,
        "fsc:hanger_thickness_m": slab_t * 1e-3,
        "fsc:hanger_top_z_m": Hv[:, 2].max() * 1e-3,
        "fsc:stem_x_lo_m": x_front * 1e-3, "fsc:stem_x_hi_m": x_back * 1e-3,
        "fsc:fin_x_lo_m": fx0 * 1e-3, "fsc:fin_x_hi_m": x_front * 1e-3,
        "fsc:fin_z_lo_m": fz0 * 1e-3, "fsc:fin_z_hi_m": fz1 * 1e-3,
        "fsc:fin_thickness_m": (fy1 - fy0) * 1e-3,
        "fsc:fin_centre_y_m": 0.5 * (fy0 + fy1) * 1e-3,
        "fsc:extent_lo_x_m": allv[:, 0].min() * 1e-3, "fsc:extent_hi_x_m": allv[:, 0].max() * 1e-3,
        "fsc:extent_lo_y_m": allv[:, 1].min() * 1e-3, "fsc:extent_hi_y_m": allv[:, 1].max() * 1e-3,
        "fsc:extent_lo_z_m": allv[:, 2].min() * 1e-3, "fsc:extent_hi_z_m": allv[:, 2].max() * 1e-3,
    }
    for k, v in feats.items():
        root.GetPrim().CreateAttribute(k, Sdf.ValueTypeNames.Double, custom=True).Set(float(v))
    root.GetPrim().CreateAttribute("fsc:has_arch", Sdf.ValueTypeNames.Bool, custom=True).Set(False)
    stage.GetRootLayer().Save()

    print(f"wrote {a.out}")
    print(f"  parts: basket, holder, {len(mk)} markers, {len(brackets)} brackets; {n_col} colliders")
    print(f"  holder (payload frame, mm): slab x {x_front:+.2f}..{x_back:+.2f}, fin x {fx0:+.2f}..{x_front:+.2f}, "
          f"y {fy0:+.2f}..{fy1:+.2f}, z {fz0:+.2f}..{fz1:+.2f} (no arch)")
    print(f"  whole payload extents (mm): x {allv[:,0].min():+.1f}..{allv[:,0].max():+.1f}, "
          f"y {allv[:,1].min():+.1f}..{allv[:,1].max():+.1f}, z {allv[:,2].min():+.1f}..{allv[:,2].max():+.1f}")
    print(f"  mass {a.mass} kg = printed {m_printed * 1e3:.0f} g (rho {rho:.0f} kg/m^3) + {len(mk)} x "
          f"{a.marker_mass * 1e3:.0f} g markers; CoM {np.round(com * 1e3, 2)} mm, "
          f"principal inertia {np.round(w * 1e6, 1)} x1e-6 kg m^2")


if __name__ == "__main__":
    main()
