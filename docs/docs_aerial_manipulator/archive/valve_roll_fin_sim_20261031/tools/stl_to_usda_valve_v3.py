#!/usr/bin/env python3
"""Valve_Assembly_v3.stl (the ROLL-FIN handle, 2026-10-31) -> Valve_Assembly_v3.usda.

    /usr/bin/python3 stl_to_usda_valve_v3.py [--stl ...] [--out ...]

(pxr comes from usd-core in the user site, so NOT with PYTHONNOUSERSITE=1.)

v3 = the v2 model (plate, bearing, the pad's two half-rings, the rotor, the five mocap markers)
with the fin cut 30 mm above its root (z 104) and the roll-fin handle of
archive/valve_handle_joint_20261030 fitted (its tools/check_on_model.py --asset wrote the STL):
a printed BASE clamped over the stump (2 x M3 bolts + nuts) holding an M4 x 70 screw along the
ROLL AXIS N -- in the fin's plane, 30 deg above horizontal, rising toward +y (the turn's
direction), through the grip point G -- and a printed ROLL FIN (56 x 52 x 8 mm plate) turning
on that screw, an O-ring between the base's neck and the fin giving it 2-4 mN.m of friction.

The v1/v2 converter (valve_turning_20261015/tools/stl_to_usda_valve.py) is NOT changed: its
helpers are imported, so v1 / v2 regenerate exactly as before. It cannot read v3 itself: two
marker mounts merged into the rotor shell on that export, and the rotor's fin top is gone.

ASSET FRAME: v1/v2's (the bearing axis on the plate-bottom plane, +z up, +x along the beam
toward the fin; the CAD is a pure translation of it).

WHAT IS AUTHORED (v2's, plus the roll fin):
  /Valve_Assembly/base        KINEMATIC rigid body: plate + bearing + the pad's half-rings (visual),
                              the plate as one convex box (collider)
  /Valve_Assembly/rotor       DYNAMIC rigid body at the origin (its pose = the axis + the beam's yaw):
                              visual = the rotor shell + the handle base + M4 screw + O-ring + the M3
                              bolts / nuts + the markers; colliders = beam, root (convex boxes, v2's),
                              the handle base (convexDecomposition: clamp, web, boss, neck), the M4
                              screw (convexHull), the five marker balls (spheres)
  /Valve_Assembly/roll_fin    DYNAMIC rigid body at the origin: visual + convexHull collider of the
                              roll fin (fsc:grip -- the claw's grip material)
  /Valve_Assembly/joint       the bearing, v1/v2's (z, base -> rotor, angular drive: the scene)
  /Valve_Assembly/roll_joint  UsdPhysics.RevoluteJoint rotor -> roll_fin about N through G (joint
                              axis Y, both local frames at G turned Rx(30 deg) so Y = N), collisions
                              between the pair off, an angular DriveAPI with stiffness / damping /
                              max force 0: the scene sets the O-ring's friction (or locks it)
  fsc:*                       v2's attributes (so the scene and the planner read the same numbers)
                              + fsc:roll_fin = 1, fsc:grasp_z_m (G's height: the claw point -- the
                              planner's push_pull_ee_offset z; v2 had it as fin top - 25 mm),
                              fsc:roll_point_m (G), fsc:roll_axis (N), fsc:roll_fin_c_axis (C),
                              fsc:roll_fin_{s,c}_{lo,hi}_m (its outline in the (s, c) plane about G),
                              fsc:roll_fin_mass_kg, fsc:roll_fin_inertia_axis_kgm2 (about N)

MASSES (the design record's densities: printed PLA 1040 kg/m^3 = what the printed rotor and payload
weigh per CAD volume, steel 7850): the rotor body = the rotor shell + the handle base + the steel
(M4, M3 x 2, nuts x 2) + the O-ring; the roll fin its own. The scene uses these when the asset has
a roll fin (its PEGASUS_VALVE_ROTOR_MASS default becomes 0 = the asset's). v2's 204 g was the rotor
shell alone at ~1032 kg/m^3; the markers are not in either (visual + colliders only, as v2).
"""
import argparse
import collections
import importlib.util
import os
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
V2_TOOL = os.path.join(HERE, "..", "..", "valve_turning_20261015", "tools", "stl_to_usda_valve.py")
_spec = importlib.util.spec_from_file_location("stl_to_usda_valve", V2_TOOL)
v2 = importlib.util.module_from_spec(_spec)
_argv, sys.argv = sys.argv, sys.argv[:1]
_spec.loader.exec_module(v2)
sys.argv = _argv

ASSETS = v2.ASSETS
ORIGIN_MM = np.array([v2.AXIS_MM[0], v2.AXIS_MM[1], v2.PLATE_BOTTOM_MM])   # the asset origin in CAD mm

# the roll fin's design (archive/valve_handle_joint_20261030/tools/roll_grip_design.py), ASSET mm
G = np.array([160.0, 0.6, 179.0])                       # the grip point = push_pull_ee_offset
N = np.array([0.0, np.cos(np.radians(30)), np.sin(np.radians(30))])   # the roll axis
C = np.array([0.0, 0.5, -np.sqrt(3) / 2])               # in the fin's plane, square to N (down, +y)
X = np.array([1.0, 0.0, 0.0])
FIN_S, FIN_C, FIN_T = (-34.0, 22.0), (-22.0, 30.0), 8.0
RHO_PLA, RHO_STEEL, RHO_NBR = 1040.0, 7850.0, 1200.0    # kg/m^3

# the v3 rotor shell's blocks, ASSET mm: v2's (CAD blocks moved to the asset frame) with the fin cut
# at z 104, + the two marker mounts the v3 export merged into the shell
def _asset_blk(blk):
    return tuple((lo - o, hi - o) for (lo, hi), o in zip(blk, ORIGIN_MM))
ROTOR_BLOCKS = {k: _asset_blk(b) for k, b in v2.ROTOR_MM.items()}
ROTOR_BLOCKS["fin"] = (ROTOR_BLOCKS["fin"][0], ROTOR_BLOCKS["fin"][1], (ROTOR_BLOCKS["fin"][2][0], 104.0))
ROTOR_BLOCKS["mount_E"] = _asset_blk(v2.ROTOR_MOUNT_MM)
ROTOR_BLOCKS["mount_merged_a"] = ((29.9, 50.2), (-25.0, 5.3), (51.9, 104.1))
ROTOR_BLOCKS["mount_merged_b"] = ((-0.2, 30.1), (4.9, 25.2), (49.9, 72.1))


def fin_frame(V):
    d = np.asarray(V, float) - G
    return d @ N, d @ C, d @ X


def classify(T):
    """One label per shell of the v3 export (asset mm triangles)."""
    lab = v2.shells(T)
    names = {}
    for k in collections.Counter(lab.tolist()):
        Tk = T[lab == k]
        V = Tk.reshape(-1, 3)
        lo, hi = V.min(0), V.max(0)
        vol = v2.solid_mass_props(Tk)[0] * 1e-3            # cm^3
        s, c, x = fin_frame(V)
        r_axis = np.linalg.norm((V - G) - np.outer(s, N), axis=1)
        if hi[0] - lo[0] > 200.0:
            names[k] = "rotor"
        elif hi[0] - lo[0] > 140.0 and lo[2] < 1.0:
            names[k] = "plate"
        elif v2.inside(V, (_asset_blk(v2.BEARING_MM),)).all():
            names[k] = "bearing"
        elif v2.inside(V, (_asset_blk(v2.RING_MM),)).all():
            names[k] = "ring"
        elif vol > 10.0 and np.abs(x).max() <= FIN_T / 2 + 0.05 and s.min() >= FIN_S[0] - 0.05 \
                and s.max() <= FIN_S[1] + 0.05 and c.min() >= FIN_C[0] - 0.05 and c.max() <= FIN_C[1] + 0.05:
            names[k] = "roll_fin"
        elif vol > 10.0 and lo[0] > 153.0 and hi[0] < 167.0 and abs(lo[2] - 74.0) < 0.5:
            names[k] = "handle_base"
        elif 0.5 < vol < 2.0 and r_axis.max() < 3.6:
            names[k] = "m4_screw"
        elif vol < 0.05 and r_axis.max() < 3.1 and -35.5 < s.min() and s.max() < -33.0:
            names[k] = "o_ring"
        elif 0.1 < vol < 0.3 and lo[0] > 150.0 and hi[0] < 170.0:
            names[k] = "m3_bolt"
        elif vol < 0.1 and lo[0] > 164.0 and hi[0] < 167.0:
            names[k] = "m3_nut"
        elif lo[2] > 45.0:
            names[k] = "marker"
        else:
            raise SystemExit(f"a shell at {lo.round(1)}..{hi.round(1)} mm ({vol:.2f} cm^3) is none of the known parts")
    got = collections.Counter(names.values())
    want = {"rotor": 1, "plate": 1, "bearing": 1, "ring": 2, "roll_fin": 1, "handle_base": 1, "m4_screw": 1,
            "o_ring": 1, "m3_bolt": 2, "m3_nut": 2}
    for n, w in want.items():
        if got.get(n, 0) != w:
            raise SystemExit(f"expected {w} '{n}' shell(s), found {got.get(n, 0)}: {dict(got)} -- not the 2026-10-31 v3 export?")
    return lab, names


def combine(parts):
    """Mass, CoM and inertia (about the CoM) of several closed meshes, each with its density."""
    M, S, items = 0.0, np.zeros(3), []
    for Tm, rho in parts:
        vol, com, I = v2.solid_mass_props(Tm)
        m = rho * vol
        items.append((m, com, I * rho))
        M += m
        S += m * com
    com = S / M
    I = np.zeros((3, 3))
    for m, c, Ic in items:
        d = c - com
        I += Ic + m * (d @ d * np.eye(3) - np.outer(d, d))
    return M, com, I


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stl", default=os.path.join(ASSETS, "Valve_Assembly_v3.stl"))
    ap.add_argument("--out", default=os.path.join(ASSETS, "Valve_Assembly_v3.usda"))
    a = ap.parse_args()

    T_mm = v2.read_stl(a.stl) * 1e3 - ORIGIN_MM                       # asset frame, mm
    lab, names = classify(T_mm)
    part = lambda n: np.concatenate([T_mm[lab == k] for k, v in names.items() if v == n])
    rv = part("rotor").reshape(-1, 3)
    ok = v2.inside(rv, list(ROTOR_BLOCKS.values()))
    if not ok.all():
        raise SystemExit(f"{(~ok).sum()} rotor vertices outside its blocks, e.g. {rv[~ok][:5]}")
    if part("rotor").reshape(-1, 3)[:, 2].max() > 104.1:
        raise SystemExit("the rotor shell still reaches above z 104: the fin was not cut -- not v3?")
    to_m = lambda Tm: Tm * 1e-3

    # mass properties [kg, m]
    rotor_parts = [(to_m(part("rotor")), RHO_PLA), (to_m(part("handle_base")), RHO_PLA),
                   (to_m(part("m4_screw")), RHO_STEEL), (to_m(part("m3_bolt")), RHO_STEEL),
                   (to_m(part("m3_nut")), RHO_STEEL), (to_m(part("o_ring")), RHO_NBR)]
    m_rot, com_rot, I_rot = combine(rotor_parts)
    m_fin, com_fin, I_fin = combine([(to_m(part("roll_fin")), RHO_PLA)])
    I_axis_rot = I_rot[2, 2] + m_rot * (com_rot[0] ** 2 + com_rot[1] ** 2)
    Gm, Nv = G * 1e-3, N
    dG = com_fin - Gm
    d_perp = dG - (dG @ Nv) * Nv
    I_fin_N = Nv @ I_fin @ Nv + m_fin * (d_perp @ d_perp)
    I_fin_axis = I_fin[2, 2] + m_fin * (com_fin[0] ** 2 + com_fin[1] ** 2)

    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Valve_Assembly")
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    root.GetPrim().SetDocumentation(
        "V3 (2026-10-31): the ROLL-FIN handle -- the v2 valve (pad half-rings, mocap markers) with the fin "
        "cut at z 104 and a printed roll fin turning on an M4 screw along N (the fin's plane, 30 deg up "
        "toward +y) through the grip point G, /roll_fin on /roll_joint (rotor -> roll_fin). Frame: the "
        "bearing axis on the plate-bottom plane, +z up, +x along the beam toward the fin. Generated by "
        "docs/docs_aerial_manipulator/archive/valve_roll_fin_sim_20261031/tools/stl_to_usda_valve_v3.py "
        "from " + os.path.basename(a.stl) + " -- regenerate, do not edit.")

    def mesh(path, Tm_mm, rgb):
        return v2_mesh(stage, path, to_m(Tm_mm), rgb)

    def collider(prim, approx, grip):
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
        prim.CreateAttribute("fsc:grip", Sdf.ValueTypeNames.Bool, custom=True).Set(bool(grip))

    def box_collider(path, blk_mm, grip):
        lo, hi = np.array([b[0] for b in blk_mm]) * 1e-3, np.array([b[1] for b in blk_mm]) * 1e-3
        corners = [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in corners])
        m.CreateFaceVertexCountsAttr([4] * 6)
        m.CreateFaceVertexIndicesAttr([0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3])
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        collider(m.GetPrim(), "convexHull", grip)
        return lo, hi

    def mass_api(prim, m, com, I):
        w, Q = np.linalg.eigh(I)
        if np.linalg.det(Q) < 0:
            Q[:, 0] = -Q[:, 0]
        qa = Gf.Quatf(Gf.Matrix3d(*Q.T.flatten().tolist()).ExtractRotation().GetQuat())
        mp = UsdPhysics.MassAPI.Apply(prim)
        mp.CreateMassAttr(float(m))
        mp.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        mp.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, w)))
        mp.CreatePrincipalAxesAttr(qa)

    # the base: kinematic, v2's
    base_x = UsdGeom.Xform.Define(stage, "/Valve_Assembly/base")
    UsdPhysics.RigidBodyAPI.Apply(base_x.GetPrim()).CreateKinematicEnabledAttr(True)
    mesh("/Valve_Assembly/base/Visual", np.concatenate([part("plate"), part("bearing"), part("ring")]), (0.35, 0.35, 0.38))
    UsdGeom.Scope.Define(stage, "/Valve_Assembly/base/Collision")
    plate_lo, plate_hi = box_collider("/Valve_Assembly/base/Collision/plate", _asset_blk(v2.PLATE_MM), False)

    # the rotor: the shell, the handle base, the hardware
    rotor_x = UsdGeom.Xform.Define(stage, "/Valve_Assembly/rotor")
    UsdPhysics.RigidBodyAPI.Apply(rotor_x.GetPrim())
    mesh("/Valve_Assembly/rotor/Visual", part("rotor"), (0.85, 0.45, 0.15))
    mesh("/Valve_Assembly/rotor/HandleBase", part("handle_base"), (0.20, 0.45, 0.85))
    mesh("/Valve_Assembly/rotor/Screw", np.concatenate([part("m4_screw"), part("m3_bolt"), part("m3_nut")]), (0.70, 0.70, 0.72))
    mesh("/Valve_Assembly/rotor/ORing", part("o_ring"), (0.05, 0.05, 0.05))
    UsdGeom.Scope.Define(stage, "/Valve_Assembly/rotor/Collision")
    beam_lo, beam_hi = box_collider("/Valve_Assembly/rotor/Collision/beam", ROTOR_BLOCKS["beam"], False)
    root_lo, root_hi = box_collider("/Valve_Assembly/rotor/Collision/root", ROTOR_BLOCKS["root"], False)
    hb = mesh("/Valve_Assembly/rotor/Collision/handle_base", part("handle_base"), (0.2, 0.45, 0.85))
    hbp = stage.GetPrimAtPath("/Valve_Assembly/rotor/Collision/handle_base")
    UsdGeom.Mesh(hbp).CreatePurposeAttr(UsdGeom.Tokens.guide)
    collider(hbp, "convexDecomposition", False)
    mesh("/Valve_Assembly/rotor/Collision/m4_screw", part("m4_screw"), (0.7, 0.7, 0.72))
    sp = stage.GetPrimAtPath("/Valve_Assembly/rotor/Collision/m4_screw")
    UsdGeom.Mesh(sp).CreatePurposeAttr(UsdGeom.Tokens.guide)
    collider(sp, "convexHull", False)
    mass_api(rotor_x.GetPrim(), m_rot, com_rot, I_rot)

    # the markers: rotor visual + sphere colliders, v2's
    marker_parts = [T_mm[lab == k] for k, v in names.items() if v == "marker"]
    mesh("/Valve_Assembly/rotor/Markers", np.concatenate(marker_parts), (0.92, 0.92, 0.95))
    balls = [b for b in (v2.marker_ball(Tm) for Tm in marker_parts) if b is not None]
    if len(balls) != 5:
        raise SystemExit(f"found {len(balls)} marker balls, expected the five of v2")
    balls = sorted((np.asarray(b) * 1e-3 for b in balls), key=lambda b: (round(b[0], 4), round(b[1], 4)))
    for i, b in enumerate(balls):
        sph = UsdGeom.Sphere.Define(stage, f"/Valve_Assembly/rotor/Collision/marker{i}")
        sph.CreateRadiusAttr(v2.MARKER_BALL_R_MM * 1e-3)
        sph.AddTranslateOp().Set(Gf.Vec3d(*map(float, b)))
        sph.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(sph.GetPrim())
        sph.GetPrim().CreateAttribute("fsc:grip", Sdf.ValueTypeNames.Bool, custom=True).Set(False)
    pivot = np.mean(balls, axis=0)

    # the roll fin: its own dynamic body at the origin
    fin_x = UsdGeom.Xform.Define(stage, "/Valve_Assembly/roll_fin")
    UsdPhysics.RigidBodyAPI.Apply(fin_x.GetPrim())
    mesh("/Valve_Assembly/roll_fin/Visual", part("roll_fin"), (0.95, 0.75, 0.10))
    UsdGeom.Scope.Define(stage, "/Valve_Assembly/roll_fin/Collision")
    mesh("/Valve_Assembly/roll_fin/Collision/fin", part("roll_fin"), (0.95, 0.75, 0.10))
    fp = stage.GetPrimAtPath("/Valve_Assembly/roll_fin/Collision/fin")
    UsdGeom.Mesh(fp).CreatePurposeAttr(UsdGeom.Tokens.guide)
    collider(fp, "convexHull", True)
    mass_api(fin_x.GetPrim(), m_fin, com_fin, I_fin)

    # the bearing: v1/v2's joint
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/Valve_Assembly/joint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path("/Valve_Assembly/base")])
    joint.CreateBody1Rel().SetTargets([Sdf.Path("/Valve_Assembly/rotor")])
    joint.CreateAxisAttr("Z")
    for f in (joint.CreateLocalPos0Attr, joint.CreateLocalPos1Attr):
        f(Gf.Vec3f(0.0, 0.0, 0.0))
    for f in (joint.CreateLocalRot0Attr, joint.CreateLocalRot1Attr):
        f(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateCollisionEnabledAttr(False)
    d = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    d.CreateTypeAttr("force"); d.CreateStiffnessAttr(0.0); d.CreateDampingAttr(0.0); d.CreateTargetVelocityAttr(0.0)

    # the roll joint: rotor -> roll fin about N through G (joint Y turned onto N by Rx(30 deg))
    rj = UsdPhysics.RevoluteJoint.Define(stage, "/Valve_Assembly/roll_joint")
    rj.CreateBody0Rel().SetTargets([Sdf.Path("/Valve_Assembly/rotor")])
    rj.CreateBody1Rel().SetTargets([Sdf.Path("/Valve_Assembly/roll_fin")])
    rj.CreateAxisAttr("Y")
    qr = Gf.Quatf(float(np.cos(np.radians(15))), float(np.sin(np.radians(15))), 0.0, 0.0)
    for f in (rj.CreateLocalPos0Attr, rj.CreateLocalPos1Attr):
        f(Gf.Vec3f(*map(float, Gm)))
    for f in (rj.CreateLocalRot0Attr, rj.CreateLocalRot1Attr):
        f(qr)
    rj.CreateCollisionEnabledAttr(False)
    d = UsdPhysics.DriveAPI.Apply(rj.GetPrim(), "angular")
    d.CreateTypeAttr("force"); d.CreateStiffnessAttr(0.0); d.CreateDampingAttr(0.0); d.CreateTargetVelocityAttr(0.0)
    # check: Rx(30) maps Y onto N
    Ry = Gf.Rotation(Gf.Quatd(float(np.cos(np.radians(15))), float(np.sin(np.radians(15))), 0.0, 0.0)).TransformDir(Gf.Vec3d(0, 1, 0))
    if np.linalg.norm(np.array(Ry) - N) > 1e-9:
        raise SystemExit(f"the roll joint's frame maps Y to {np.array(Ry)}, not N")

    rf_v = part("roll_fin").reshape(-1, 3)
    feats = {
        "fsc:fin_radius_m": float(Gm[0]), "fsc:fin_bearing_y_m": float(Gm[1]),
        "fsc:fin_thickness_m": FIN_T * 1e-3,
        "fsc:fin_width_m": float((rf_v[:, 1].max() - rf_v[:, 1].min()) * 1e-3),
        "fsc:fin_top_z_m": float(rf_v[:, 2].max() * 1e-3),
        "fsc:fin_exposed_bottom_z_m": float(rf_v[:, 2].min() * 1e-3),
        "fsc:root_thickness_m": float(root_hi[0] - root_lo[0]),
        "fsc:beam_reach_m": float(beam_hi[0]), "fsc:beam_back_m": float(-beam_lo[0]),
        "fsc:beam_width_m": float(beam_hi[1] - beam_lo[1]), "fsc:beam_top_z_m": float(beam_hi[2]),
        "fsc:beam_bottom_z_m": float(beam_lo[2]),
        "fsc:plate_size_x_m": float(plate_hi[0] - plate_lo[0]), "fsc:plate_size_y_m": float(plate_hi[1] - plate_lo[1]),
        "fsc:plate_top_z_m": float(plate_hi[2]),
        "fsc:hub_pad_rings": 2.0,
        "fsc:rotor_mass_kg": float(m_rot), "fsc:rotor_inertia_axis_kgm2": float(I_axis_rot),
        "fsc:mocap_marker_count": float(len(balls)), "fsc:mocap_pivot_x_m": float(pivot[0]),
        "fsc:mocap_pivot_y_m": float(pivot[1]), "fsc:mocap_pivot_z_m": float(pivot[2]),
        "fsc:mocap_marker_radius_m": v2.MARKER_BALL_R_MM * 1e-3,
        "fsc:roll_fin": 1.0, "fsc:grasp_z_m": float(Gm[2]),
        "fsc:roll_fin_s_lo_m": FIN_S[0] * 1e-3, "fsc:roll_fin_s_hi_m": FIN_S[1] * 1e-3,
        "fsc:roll_fin_c_lo_m": FIN_C[0] * 1e-3, "fsc:roll_fin_c_hi_m": FIN_C[1] * 1e-3,
        "fsc:roll_fin_mass_kg": float(m_fin), "fsc:roll_fin_inertia_n_kgm2": float(I_fin_N),
        "fsc:roll_fin_inertia_axis_kgm2": float(I_fin_axis),
    }
    for k, v in feats.items():
        root.GetPrim().CreateAttribute(k, Sdf.ValueTypeNames.Double, custom=True).Set(v)
    for k, v in (("fsc:roll_point_m", Gm), ("fsc:roll_axis", N), ("fsc:roll_fin_c_axis", C)):
        root.GetPrim().CreateAttribute(k, Sdf.ValueTypeNames.Double3, custom=True).Set(Gf.Vec3d(*map(float, v)))
    root.GetPrim().CreateAttribute("fsc:mocap_markers_m", Sdf.ValueTypeNames.Double3Array, custom=True).Set(
        [Gf.Vec3d(*map(float, b)) for b in balls])
    stage.GetRootLayer().Save()

    print(f"wrote {a.out}")
    print(f"  shells: {dict(collections.Counter(names.values()))}")
    print(f"  rotor body {m_rot * 1e3:.1f} g (shell + handle base PLA {RHO_PLA:.0f}, screws steel), CoM "
          f"{np.round(com_rot * 1e3, 1)} mm, about the axis {I_axis_rot * 1e6:.0f} x1e-6 kg m^2")
    print(f"  roll fin {m_fin * 1e3:.1f} g, CoM {np.round(com_fin * 1e3, 1)} mm "
          f"({np.linalg.norm(d_perp) * 1e3:.1f} mm off the roll axis, c {dG @ C * 1e3:+.1f} mm), "
          f"about N {I_fin_N * 1e9:.0f} x1e-9 kg m^2, about the valve axis {I_fin_axis * 1e6:.0f} x1e-6")
    print(f"  together {(m_rot + m_fin) * 1e3:.1f} g, about the valve axis {(I_axis_rot + I_fin_axis) * 1e6:.0f} x1e-6 kg m^2")
    print(f"  markers' centroid {np.round(pivot * 1e3, 1)} mm -> push_pull_valve_axis_offset "
          f"[{-pivot[0]:.4f}, {-pivot[1]:.4f}, {-pivot[2]:.4f}]")
    for k, v in feats.items():
        print(f"  {k} = {v:+.5f}")


def v2_mesh(stage, path, Tm, rgb):
    """v2's mesh writer (stl_to_usda_valve.py main()'s inner mesh()), metres in."""
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


if __name__ == "__main__":
    main()
