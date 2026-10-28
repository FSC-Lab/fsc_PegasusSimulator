#!/usr/bin/env python3
"""Valve_Assembly.stl (CAD, metres, +Z up) -> Valve_Assembly.usda (2026-10-15).

    /usr/bin/python3 stl_to_usda_valve.py [--stl ...] [--out ...] [--density 1240 | --mass 0.243]
    /usr/bin/python3 stl_to_usda_valve.py --stl .../Valve_Assembly_v2.stl --out .../Valve_Assembly_v2.usda

V2 (2026-10-27, Valve_Assembly_v2.stl): the same three shells plus TWO STATIC HALF-RINGS
around the hub (inner radius 27.2 mm against the 25.0 mm hub, 20 mm tall from the plate's
pocket), holding one layer of fabric-faced pad on the hub -- the bearing's friction brake
(bench: ~72 mN.m Coulomb, archive/valve_damper_spin_20261027). They join the BASE's visual;
no collider (nothing reaches them: the beam passes 10 mm above, the claw grips at r 0.160).
The pad's friction is the scene's PEGASUS_VALVE_FRICTION, not geometry. The rotor shell is
v1's, byte for byte.

V2 WITH THE MOCAP MARKERS (2026-10-28, the user's Valve_Assembly_v2.stl): + FIVE 30 mm
marker balls on stems, on four printed mounts (a fifth mount is fused to the rotor shell),
all on the ROTOR. They join the rotor's visual; the balls are sphere colliders (so a claw or
a skid that touched one would show in the sim); none of it enters the rotor's mass (the
scene's PEGASUS_VALVE_ROTOR_MASS sets that). The ball centres -- what the mocap triangulates
-- and their CENTROID (Motive's DEFAULT pivot of the rigid body made from them) are written
as fsc:mocap_* attributes in the asset frame: the scene publishes obj_0 there, the planner
takes push_pull_valve_axis_offset = minus the centroid.

(pxr comes from usd-core in the user site, so NOT with PYTHONNOUSERSITE=1.)

THE VALVE-TURNING MECHANISM (user, 2026-10-15): a 150 x 50 x 20 mm base plate
with an 8 mm shaft and a 608 bearing (22 mm OD, 8 mm bore, 7 mm) seated in its
centre, and ONE rotating part on the bearing's outer race -- a 50 mm hub
carrying a 225 x 50 x 14 mm BEAM (200 mm one way from the axis, 25 the other)
with, at radius 0.160 m, a vertical FIN HANDLE 5 mm thick (radial) x 40 mm
wide (tangential) x 130 mm tall on a 10 x 50 x 20 mm root. The fin top is
204.5 mm above the plate bottom. The STL holds three closed shells: the
plate (+ its shaft), the bearing ring, the rotor (hub + beam + root + fin).

ASSET FRAME (the root prim's own frame, unscaled):
  origin = the BEARING AXIS at the PLATE-BOTTOM plane (set the plate on the
           table top and the origin sits on the table top, on the axis)
  +z     = up (CAD +Z)
  +x     = CAD +X = the beam's long direction, toward the fin
  (CAD axis at (-1.1, 3.0) mm, plate bottom at z = -4.5 mm: a pure translation.)

WHAT IS AUTHORED:
  /Valve_Assembly/base         KINEMATIC rigid body (fixed; a joint needs a
                               body) -- visual = plate + bearing shells,
                               Collision/plate = the plate as one convex box
  /Valve_Assembly/rotor        DYNAMIC rigid body at the SAME origin (so the
                               published pose of this prim IS the axis + the
                               beam's yaw = mocap obj_0) -- visual = the rotor
                               shell; Collision/{beam, root, fin} = exact convex
                               boxes read off the CAD's faces (the beam's
                               pockets left solid; the hub is NOT a collider:
                               it sits over the plate's bearing pocket, and
                               nothing touches it); MassAPI from the closed
                               solid at uniform density (--density) or --mass
  /Valve_Assembly/joint        UsdPhysics.RevoluteJoint, axis Z, base -> rotor,
                               both local frames at the origin; an "angular"
                               DriveAPI with stiffness 0 and the bearing's
                               viscous damping (the scene sets it: PhysX
                               maximal joints have no Coulomb friction knob)
  fsc:* attributes             the numbers the scene and the planner need: the
                               fin's radius / bearing / thickness / width /
                               top / bare-bottom, the beam's reach and top, the
                               plate size -- nothing downstream re-measures
The placement, the friction materials, the grip shim and the damping value are
the scene's job (07_px4_t650_aerial_manipulator_valve_turning.py).
"""
import argparse
import collections
import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ASSETS = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..",
    "extensions", "fsc_aerial_manipulation", "fsc_aerial_manipulation", "rotorcraft", "assets"))

# CAD millimetres (read off the STL's axis-aligned faces, 2026-10-15)
AXIS_MM = np.array([-1.1, 3.0])        # the bearing / shaft centre (x, y)
PLATE_BOTTOM_MM = -4.5
PLATE_MM = ((-76.1, 73.9), (-22.0, 28.0), (-4.5, 15.5))
BEARING_MM = ((-12.1, 9.9), (-8.0, 14.0), (5.5, 12.5))
RING_MM = ((-33.3, 31.0), (-36.7, 42.7), (5.5, 25.5))      # v2: the two static half-rings, together
# the rotor's parts: (x_lo, x_hi), (y_lo, y_hi), (z_lo, z_hi)
ROTOR_MM = {
    # the hub: a 50 mm cylinder on the bearing's outer race (bore 22 mm at the
    # bottom), from the plate's pocket up to the beam -- NOT a collider (it
    # overlaps the plate's box in z, and nothing touches it)
    "hub": ((-26.1, 23.9), (-21.9, 28.0), (5.5, 35.5)),
    "beam": ((-26.1, 199.0), (-22.0, 28.7), (35.5, 49.5)),
    "root": ((153.9, 163.9), (-21.4, 28.6), (49.5, 69.5)),
    "fin": ((156.4, 161.4), (-16.4, 23.6), (69.5, 199.5)),
}
# (2026-10-28) the marker mount FUSED to the rotor shell in the markers' CAD (marker E's,
# beside the fin root): its vertices stand above the beam top, inside this block
ROTOR_MOUNT_MM = ((113.9, 134.0), (-21.7, 8.4), (45.5, 59.0))
MARKER_BALL_R_MM = 15.0                 # the CAD's marker balls (30 mm)
COLLIDERS = ("beam", "root", "fin")
GRIP_PARTS = ("fin", "root")               # what the claw may touch


def read_stl(path):
    raw = open(path, "rb").read()
    n = int(np.frombuffer(raw[80:84], "<u4")[0])
    if 84 + 50 * n != len(raw):
        raise SystemExit(f"{path}: not a binary STL (or truncated)")
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(raw[84:84 + 50 * n], dt)["v"].astype(float)      # (n, 3, 3), CAD units


def shells(T):
    """Connected components of the triangle mesh (shared vertices): one label
    per triangle."""
    V = np.round(T.reshape(-1, 3), 6)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3)
    parent = np.arange(len(uniq))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b, c in inv:
        parent[find(a)] = find(b)
        parent[find(b)] = find(c)
    roots = np.array([find(i) for i in range(len(uniq))])
    return roots[inv[:, 0]]


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


def inside(V, blocks, tol=0.2):
    ok = np.zeros(len(V), bool)
    for (xl, xh), (yl, yh), (zl, zh) in blocks:
        ok |= ((V[:, 0] >= xl - tol) & (V[:, 0] <= xh + tol) & (V[:, 1] >= yl - tol)
               & (V[:, 1] <= yh + tol) & (V[:, 2] >= zl - tol) & (V[:, 2] <= zh + tol))
    return ok


def marker_ball(Tm, r_mm=MARKER_BALL_R_MM):
    """The ball of a marker part (a ball on a stem, or a mount with none): its centre
    [mm] by a least-squares sphere fit at either end of the part's long axis, or None.
    A ball must put >= 1000 vertices within 0.3 mm of a sphere of radius r_mm +- 0.5."""
    V = np.unique(np.round(Tm.reshape(-1, 3), 4), axis=0)
    lo, hi = V.min(0), V.max(0)
    ax = int(np.argmax(hi - lo))
    ctr = 0.5 * (lo + hi)
    best = None
    for end, sgn in ((lo[ax], 1.0), (hi[ax], -1.0)):
        c0 = ctr.copy()
        c0[ax] = end + sgn * r_mm
        r = r_mm
        for _ in range(8):
            sel = np.abs(np.linalg.norm(V - c0, axis=1) - r_mm) < 1.0
            if sel.sum() < 20:
                break
            P = V[sel]
            sol = np.linalg.lstsq(np.c_[2 * P, np.ones(len(P))], (P ** 2).sum(1), rcond=None)[0]
            c0, r = sol[:3], float(np.sqrt(sol[3] + sol[:3] @ sol[:3]))
        n_on = int((np.abs(np.linalg.norm(V - c0, axis=1) - r) < 0.3).sum())
        if n_on >= 1000 and abs(r - r_mm) < 0.5 and (best is None or n_on > best[0]):
            best = (n_on, c0)
    return None if best is None else best[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stl", default=os.path.join(ASSETS, "Valve_Assembly_v1.stl"))   # renamed by the user 2026-10-27
    ap.add_argument("--out", default=os.path.join(ASSETS, "Valve_Assembly.usda"))
    ap.add_argument("--density", type=float, default=1240.0,
                    help="the rotor's density [kg/m^3] (solid PLA ~1240; a printed part at 20 %% infill is ~1/3 of it)")
    ap.add_argument("--mass", type=float, default=0.0, help="the rotor's mass [kg] (overrides --density)")
    a = ap.parse_args()

    T_cad = read_stl(a.stl) * 1e3                                          # mm
    V = T_cad.reshape(-1, 3)
    if np.ptp(V, 0).max() > 1000.0 or np.ptp(V, 0).max() < 100.0:
        raise SystemExit(f"unexpected extent {np.ptp(V, 0)} mm -- not the metre-unit 2026-10-15 valve?")
    lab = shells(T_cad)
    counts = collections.Counter(lab.tolist())
    if len(counts) < 3:
        raise SystemExit(f"expected 3 closed shells (plate, bearing, rotor), 5 (v2: + two half-rings) or "
                         f"more (+ the mocap markers), found {len(counts)} -- the CAD changed?")
    order = [k for k, _ in counts.most_common()]          # rotor (most triangles), bearing, plate
    part_of = {}
    for i, k in enumerate(order):
        sv = T_cad[lab == k].reshape(-1, 3)
        lo, hi = sv.min(0), sv.max(0)
        if i == 0 and hi[2] > 100.0:
            part_of[k] = "rotor"
        elif hi[0] - lo[0] > 100.0:
            part_of[k] = "plate"
        elif inside(sv, (BEARING_MM,)).all():
            part_of[k] = "bearing"
        elif inside(sv, (RING_MM,)).all():
            part_of[k] = "ring"
        elif lo[2] > ROTOR_MM["hub"][2][1]:              # above the hub: a marker or its mount
            part_of[k] = "marker"
        else:
            raise SystemExit(f"a shell at {lo.round(1)}..{hi.round(1)} mm is none of the known parts")
    core = sorted(v for v in part_of.values() if v != "marker")
    if core not in (["bearing", "plate", "rotor"], ["bearing", "plate", "ring", "ring", "rotor"]):
        raise SystemExit(f"could not tell the shells apart: {part_of}")
    n_marker_parts = sum(v == "marker" for v in part_of.values())
    rotor = T_cad[np.array([part_of[k] == "rotor" for k in lab])]
    base = T_cad[np.array([part_of[k] in ("plate", "bearing", "ring") for k in lab])]
    marker_parts = [T_cad[lab == k] for k in order if part_of[k] == "marker"]
    # the rotor's vertices must all lie in its declared blocks: the colliders
    # ARE the rotor (the pockets in the beam are inside the beam block) -- with
    # the markers, plus the one mount fused to the rotor shell
    rv = rotor.reshape(-1, 3)
    ok = inside(rv, list(ROTOR_MM.values()) + ([ROTOR_MOUNT_MM] if n_marker_parts else []))
    if not ok.all():
        raise SystemExit(f"{(~ok).sum()} rotor vertices outside the declared blocks, e.g. {rv[~ok][:5]}")
    bv = base.reshape(-1, 3)
    ok = inside(bv, (PLATE_MM, BEARING_MM, RING_MM))
    if not ok.all():
        raise SystemExit(f"{(~ok).sum()} base vertices outside the plate / bearing blocks, e.g. {bv[~ok][:5]}")
    # the fin IS at the declared radius: its x faces must be the block's
    for fx in ROTOR_MM["fin"][0]:
        if not np.isclose(rv[:, 0], fx, atol=0.06).any():
            raise SystemExit(f"no rotor vertex on the fin's x = {fx} mm face -- the fin moved?")

    origin_mm = np.array([AXIS_MM[0], AXIS_MM[1], PLATE_BOTTOM_MM])

    def to_asset(p_mm):
        return (np.asarray(p_mm, float) - origin_mm) * 1e-3

    R = to_asset(rotor.reshape(-1, 3)).reshape(-1, 3, 3)                   # m, asset frame
    B = to_asset(base.reshape(-1, 3)).reshape(-1, 3, 3)
    vol, com, I = solid_mass_props(R)
    if vol <= 0:
        raise SystemExit("negative rotor volume: the STL's winding is inverted")
    mass = a.mass if a.mass > 0.0 else a.density * vol
    I = I * (mass / vol)
    w, Q = np.linalg.eigh(I)
    if np.linalg.det(Q) < 0:
        Q[:, 0] = -Q[:, 0]
    qa = Gf.Quatf(Gf.Matrix3d(*Q.T.flatten().tolist()).ExtractRotation().GetQuat())
    # inertia about the AXIS (the z through the origin), what the push feels
    I_axis = I[2, 2] + mass * (com[0] ** 2 + com[1] ** 2)

    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Valve_Assembly")
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    n_ring = sum(v == "ring" for v in part_of.values())
    root.GetPrim().SetDocumentation(
        ("V2 (2026-10-27): + two static half-rings holding a fabric pad on the hub (base visual only). "
         if n_ring else "") + ("MOCAP MARKERS (2026-10-28): on the rotor, visual + sphere colliders, fsc:mocap_*. "
         if n_marker_parts else "") + "Valve-turning mechanism, 2026-10-15: a base plate with a 608 bearing and a rotating beam "
        "carrying a fin handle at radius 0.160 m. Frame: origin = the bearing axis on the plate-"
        "bottom plane, +z up, +x = the beam toward the fin. /base is a kinematic rigid body (fixed), "
        "/rotor a dynamic one on a z revolute joint at the origin. Generated by docs/docs_aerial_"
        "manipulator/archive/valve_turning_20261015/tools/stl_to_usda_valve.py from "
        + os.path.basename(a.stl) + " -- regenerate, do not edit.")

    def mesh(path, Tm, rgb):
        P = Tm.reshape(-1, 3)
        key = np.round(P / 1e-7).astype(np.int64)
        uniq, inv = np.unique(key, axis=0, return_inverse=True)
        pts = np.zeros((len(uniq), 3))
        pts[inv] = P
        idx = inv.reshape(-1, 3)
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
        m.CreateFaceVertexCountsAttr([3] * len(idx))
        m.CreateFaceVertexIndicesAttr(idx.flatten().tolist())
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        m.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        m.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])
        return len(idx), len(pts)

    def convex_box(path, blk, grip):
        (xl, xh), (yl, yh), (zl, zh) = blk
        lo, hi = to_asset([xl, yl, zl]), to_asset([xh, yh, zh])
        corners = [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in corners])
        m.CreateFaceVertexCountsAttr([4] * 6)
        m.CreateFaceVertexIndicesAttr([0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3])
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        m.GetPrim().CreateAttribute("fsc:grip", Sdf.ValueTypeNames.Bool, custom=True).Set(bool(grip))
        return lo, hi

    # the base: a KINEMATIC rigid body (never moves; a joint needs a body on
    # each side and a kinematic body collides as a static one does)
    base_x = UsdGeom.Xform.Define(stage, "/Valve_Assembly/base")
    rb = UsdPhysics.RigidBodyAPI.Apply(base_x.GetPrim())
    rb.CreateKinematicEnabledAttr(True)
    nb = mesh("/Valve_Assembly/base/Visual", B, (0.35, 0.35, 0.38))
    UsdGeom.Scope.Define(stage, "/Valve_Assembly/base/Collision")
    plate_lo, plate_hi = convex_box("/Valve_Assembly/base/Collision/plate", PLATE_MM, False)

    rotor_x = UsdGeom.Xform.Define(stage, "/Valve_Assembly/rotor")
    UsdPhysics.RigidBodyAPI.Apply(rotor_x.GetPrim())
    nr = mesh("/Valve_Assembly/rotor/Visual", R, (0.85, 0.45, 0.15))
    UsdGeom.Scope.Define(stage, "/Valve_Assembly/rotor/Collision")
    parts = {}
    for name in COLLIDERS:
        parts[name] = convex_box(f"/Valve_Assembly/rotor/Collision/{name}", ROTOR_MM[name], name in GRIP_PARTS)
    UsdPhysics.MassAPI.Apply(rotor_x.GetPrim())
    mapi = UsdPhysics.MassAPI(rotor_x.GetPrim())
    mapi.CreateMassAttr(float(mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, w)))
    mapi.CreatePrincipalAxesAttr(qa)

    # the bearing: a revolute joint about z at the origin of BOTH bodies
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/Valve_Assembly/joint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path("/Valve_Assembly/base")])
    joint.CreateBody1Rel().SetTargets([Sdf.Path("/Valve_Assembly/rotor")])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateCollisionEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)                      # the scene sets the bearing's viscous damping
    drive.CreateTargetVelocityAttr(0.0)

    fin_lo, fin_hi = parts["fin"]
    root_lo, root_hi = parts["root"]
    beam_lo, beam_hi = parts["beam"]
    feats = {
        "fsc:fin_radius_m": float(0.5 * (fin_lo[0] + fin_hi[0])),           # along the beam (asset x)
        "fsc:fin_bearing_y_m": float(0.5 * (fin_lo[1] + fin_hi[1])),       # its offset across (asset y)
        "fsc:fin_thickness_m": float(fin_hi[0] - fin_lo[0]),               # radial: the jaws close along it
        "fsc:fin_width_m": float(fin_hi[1] - fin_lo[1]),                   # tangential
        "fsc:fin_top_z_m": float(fin_hi[2]),                               # above the plate bottom
        "fsc:fin_exposed_bottom_z_m": float(root_hi[2]),                   # the root top: bare above it
        "fsc:root_thickness_m": float(root_hi[0] - root_lo[0]),
        "fsc:beam_reach_m": float(beam_hi[0]),                             # the beam's far end from the axis
        "fsc:beam_back_m": float(-beam_lo[0]),                             # ... and the near end
        "fsc:beam_width_m": float(beam_hi[1] - beam_lo[1]),
        "fsc:beam_top_z_m": float(beam_hi[2]),
        "fsc:beam_bottom_z_m": float(beam_lo[2]),
        "fsc:plate_size_x_m": float(plate_hi[0] - plate_lo[0]),
        "fsc:plate_size_y_m": float(plate_hi[1] - plate_lo[1]),
        "fsc:plate_top_z_m": float(plate_hi[2]),
        "fsc:hub_pad_rings": float(n_ring),                                # v2: the pad's two half-rings
        "fsc:rotor_mass_kg": float(mass),
        "fsc:rotor_inertia_axis_kgm2": float(I_axis),
    }
    # THE MOCAP MARKERS (2026-10-28): the rotor's visual + a sphere collider per ball;
    # the centres and their centroid (Motive's default pivot) for the scene and planner
    balls = []
    if marker_parts:
        mesh("/Valve_Assembly/rotor/Markers", to_asset(np.concatenate(marker_parts).reshape(-1, 3)).reshape(-1, 3, 3),
             (0.92, 0.92, 0.95))
        for Tm in marker_parts:
            c = marker_ball(Tm)
            if c is not None:
                balls.append(to_asset(c))
        if len(balls) < 3:
            raise SystemExit(f"found {len(balls)} marker balls among {len(marker_parts)} marker parts -- a "
                             f"rigid body needs 3 or more (not {2 * MARKER_BALL_R_MM:.0f} mm balls?)")
        balls.sort(key=lambda b: (round(b[0], 4), round(b[1], 4)))
        for i, b in enumerate(balls):
            sph = UsdGeom.Sphere.Define(stage, f"/Valve_Assembly/rotor/Collision/marker{i}")
            sph.CreateRadiusAttr(MARKER_BALL_R_MM * 1e-3)
            sph.AddTranslateOp().Set(Gf.Vec3d(*map(float, b)))
            sph.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(sph.GetPrim())
            sph.GetPrim().CreateAttribute("fsc:grip", Sdf.ValueTypeNames.Bool, custom=True).Set(False)
        pivot = np.mean(balls, axis=0)
        feats.update({"fsc:mocap_marker_count": float(len(balls)),
                      "fsc:mocap_pivot_x_m": float(pivot[0]), "fsc:mocap_pivot_y_m": float(pivot[1]),
                      "fsc:mocap_pivot_z_m": float(pivot[2]),
                      "fsc:mocap_marker_radius_m": MARKER_BALL_R_MM * 1e-3})
        root.GetPrim().CreateAttribute("fsc:mocap_markers_m", Sdf.ValueTypeNames.Double3Array, custom=True).Set(
            [Gf.Vec3d(*map(float, b)) for b in balls])
    for k, v in feats.items():
        root.GetPrim().CreateAttribute(k, Sdf.ValueTypeNames.Double, custom=True).Set(v)
    stage.GetRootLayer().Save()

    print(f"wrote {a.out}")
    print(f"  base: {nb[0]} visual triangles ({n_ring} half-rings among them), 1 plate collider; rotor: {nr[0]} triangles, {len(COLLIDERS)} colliders "
          f"({', '.join(COLLIDERS)}); every rotor vertex lies in its declared blocks")
    print(f"  rotor solid {vol * 1e6:.1f} cm^3 -> {mass * 1e3:.1f} g "
          + (f"at {a.density:.0f} kg/m^3" if a.mass <= 0 else "(--mass)")
          + f"; CoM {np.round(com * 1e3, 1)} mm from the axis, principal inertia {np.round(w * 1e6, 1)} x1e-6 kg m^2, "
          f"about the axis {I_axis * 1e6:.1f} x1e-6 kg m^2")
    for k, v in feats.items():
        print(f"  {k} = {v:+.4f}")
    if balls:
        d = np.linalg.norm(np.array(balls)[:, None] - np.array(balls)[None], axis=2)[np.triu_indices(len(balls), 1)]
        print(f"  {len(balls)} MOCAP MARKERS ({n_marker_parts} marker parts) at, asset frame [mm]:")
        for b in balls:
            print(f"    {np.round(b * 1e3, 1)}")
        print(f"  their centroid (Motive's default pivot) {np.round(pivot * 1e3, 1)} mm: "
              f"{np.hypot(pivot[0], pivot[1]) * 1e3:.1f} mm off the axis -> push_pull_valve_axis_offset "
              f"[{-pivot[0]:.4f}, {-pivot[1]:.4f}, {-pivot[2]:.4f}]")
        print(f"  pairwise marker distances [mm]: {np.sort(d * 1e3).round(1)}")


if __name__ == "__main__":
    main()
