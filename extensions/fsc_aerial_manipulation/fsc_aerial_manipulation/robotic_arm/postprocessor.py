"""
| File: postprocessor.py
| Author: Shiqi Gao (shiqi.gao907@gmail.com)
| Author: Ben Natra (send2ben123@gmail.com)
| License: BSD-3-Clause
| Copyright (c) 2026, Shiqi Gao and Ben Natra. All rights reserved.

onshape_usd_postprocessor.py

Paste into Isaac Sim Script Editor (Window > Script Editor) and run immediately
after the OnShape importer finishes. Automates the full cleanup sequence:

   1. Move vehicle subtree from nested import path to stage root
   2. Delete empty wrapper xforms
   3. Set the vehicle as defaultPrim
   4. Rename links from OnShape-generated names to simulation names
   5. Clear instanceable on the whole subtree
   6. Reparent joints to sit under their parent xform
   7. Set each joint's body0/body1 (swap if reversed)          [JOINT_OVERRIDES]
   8. Create the gripper-finger prismatic joints               [JOINT_OVERRIDES]
   9. Realign body frames to world (frames + meshes + joints)  [REALIGN_TARGETS]
  10. Realign joint local frames to identity + axis + limits   [JOINT_OVERRIDES]
  11. Set MassAPI (mass, center-of-mass, inertia) on each link [MASS_OVERRIDES]
  12. Add convex-decomposition colliders to every mesh
  13. Apply joint drive gains (stiffness/damping/maxForce)     [JOINT_OVERRIDES]
  14. DEBUG: draw authored joint local frames as RGB axes (comment out for final)
  15. Export a flattened USD ready for simulation

Edit only the CONFIG section below. The script body never needs to change.
JOINT_OVERRIDES is the single source of truth for every joint (bodies, axis,
drive gains, limits); MASS_OVERRIDES for the inertial properties.
"""

import traceback
import numpy as np
import omni.usd
import omni.kit.commands
import carb
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

# ============================================================================
# PART 1 · PARAMETER SETTINGS   — edit these for each OnShape import
# ============================================================================

# ── Prim paths ─────────────────────────────────────────────────────────────

# Full prim path of the innermost vehicle xform right after OnShape import.
# Open the Stage panel in Isaac Sim and expand the tree to find the path.
# Example observed in gripper_bat: /World/gripper_drone/gripper_drone
VEH_NAME = "gripper_drone_zz"  # name of the imported vehicle xform
VEHICLE_ROOT_SRC = f"/World/{VEH_NAME}/{VEH_NAME}"

# Desired root path in the cleaned USD (can rename here too).
VEHICLE_ROOT_DST = "/gripper_bat"

# Wrapper xforms above VEHICLE_ROOT_SRC to delete after the vehicle is moved.
# List innermost-first so each becomes empty before it is deleted.
WRAPPER_XFORMS_TO_DELETE = [
    f"/World/{VEH_NAME}",   # inner wrapper (becomes empty after vehicle move)
    "/World",                  # outer wrapper (or "Environment" node)
]

# Output path for the flattened, cleaned USD.
# OUTPUT_USD = "c:/Workspaces/fsc_PegasusSimulator/application/robotic_arm/assets/isaac_sim_models/gripper_bat.usd"
OUTPUT_USD = "/home/shiqi/fsc_PegasusSimulator/application/robotic_arm/assets/isaac_sim_models/gripper_bat.usd"

# ── Link renames ───────────────────────────────────────────────────────────
# OnShape generates generic part names; rename them to meaningful sim names.
# key = name as imported, value = desired name.
LINK_RENAMES: dict[str, str] = {
    "link_prop_left_01":  "rotor0",
    "link_prop_left":     "rotor1",
    "link_prop_right":    "rotor2",
    "link_prop_right_01": "rotor3",
    "body_with_bat":      "body",
    "PR33_ILR_A01_01":    "left_link",
    "PR33_ILR_A01":       "right_link",
}

# ── Joint reparenting ──────────────────────────────────────────────────────
# After import all joints are flat siblings; move each under its body0 xform.
# key   = joint name (at vehicle root level after step 1)
# value = name of the desired parent link (after any renames in LINK_RENAMES)
JOINT_REPARENTS: dict[str, str] = {
    "joint0":       "rotor0",
    "joint1":       "rotor1",
    "joint2":       "rotor2",
    "joint3":       "rotor3",
    "manip_joint1": "link2",
    "manip_joint2": "link3",
    "manip_joint3": "link4",
    "manip_joint4": "manip_base",
    "gripper_joint": "hub",
    "ll_rev":       "left_link",
    "lg_rev":       "left_grip",
    "rl_rev":       "right_link",
    "rg_rev":       "right_grip",
}

# ── Joint overrides (single source of truth for every joint) ───────────────
# One entry per joint — the ONE place to check/verify ALL joint settings. Fields:
#   "type"  : "revolute" | "prismatic"  (revolute → angular DOF, prismatic → linear)
#   "body0" : parent link,  "body1" : child link   (Isaac convention: parent → child)
#             step7 brings each joint to this body0/body1 (swapping body0<->body1 +
#             localPos/localRot if reversed, so the joint stays physically in place);
#             step8 reads body0/body1 from here for the prismatics it creates.
#   "axis"  : "X" | "Y" | "Z" — the DOF axis AFTER step10 world-aligns the joint
#             frames (localRot0/localRot1 → identity). step10 sets physics:axis to
#             this and verifies it matches the joint's geometric DOF, warning on
#             mismatch. (Some arm joints change Z→X once the frame is world-aligned.)
#   "drive" : None (passive) or {"stiffness","damping","maxForce"} — applied by step13.
#   "limit" : None (free) or (lower, upper) in DEGREES (revolute) / METERS (prismatic),
#             applied by step10. Use None on a side to leave it open, e.g. (None, 0.0).
#             ⚠ For a joint whose DOF pointed along a NEGATIVE world axis, step10 makes
#             + = +world-axis, so its limits here are MIRRORED vs the raw import
#             ([lo,hi] → [-hi,-lo]); the same sign flip applies to any command you send.
JOINT_OVERRIDES: dict[str, dict] = {
    # rotors — velocity-driven spin (body → rotor), continuous, DOF +Z
    "joint0":        {"body0": "body",  "body1": "rotor0",
                      "type": "revolute", "axis": "Z",  "limit": None,
                      "drive": {"stiffness": 0.0, "damping": 0.002, "maxForce": 1e6}},
    "joint1":        {"body0": "body",       "body1": "rotor1",
                      "type": "revolute", "axis": "Z",  "limit": None,
                      "drive": {"stiffness": 0.0, "damping": 0.002, "maxForce": 1e6}},
    "joint2":        {"body0": "body",       "body1": "rotor2",
                      "type": "revolute", "axis": "Z",  "limit": None,
                      "drive": {"stiffness": 0.0, "damping": 0.002, "maxForce": 1e6}},
    "joint3":        {"body0": "body",       "body1": "rotor3",
                      "type": "revolute", "axis": "Z",  "limit": None,
                      "drive": {"stiffness": 0.0, "damping": 0.002, "maxForce": 1e6}},
    # manipulator chain — position-driven (body → link2 → link3 → link4 → manip_base → hub)
    "manip_joint1":  {"body0": "body",       "body1": "link2",
                      "type": "revolute", "axis": "Z",  "limit": (-35.0, 35.0),
                      "drive": {"stiffness": 40.0, "damping": 0.8, "maxForce": 4.1}},
    "manip_joint2":  {"body0": "link2",      "body1": "link3",
                      "type": "revolute", "axis": "X",  "limit": (-90.0, 50.0),
                      "drive": {"stiffness": 80.0, "damping": 1.5, "maxForce": 4.1}},
    "manip_joint3":  {"body0": "link3",      "body1": "link4",
                      "type": "revolute", "axis": "X",  "limit": (-90.0, 50.0),
                      "drive": {"stiffness": 40.0, "damping": 0.7, "maxForce": 4.1}},
    "manip_joint4":  {"body0": "link4",      "body1": "manip_base",
                      "type": "revolute", "axis": "Z",  "limit": (-180.0, 180.0),
                      "drive": {"stiffness": 10.0, "damping": 0.1, "maxForce": 4.1}},
    "gripper_joint": {"body0": "manip_base", "body1": "hub",
                      "type": "revolute", "axis": "Z",  "limit": (-50.0, 50.0),
                      "drive": {"stiffness": 1000.0, "damping": 20.0, "maxForce": 4.1}},
    # gripper linkage — passive revolutes (hub → link → grip), no drive
    "ll_rev":        {"body0": "hub",        "body1": "right_link",
                      "type": "revolute", "axis": "Z",  "limit": None,
                      "drive": None},
    "rl_rev":        {"body0": "hub",        "body1": "left_link",
                      "type": "revolute", "axis": "Z",  "limit": None,
                      "drive": None},
    "lg_rev":        {"body0": "right_link", "body1": "left_grip",
                      "type": "revolute", "axis": "Z",  "limit": None,
                      "drive": None},
    "rg_rev":        {"body0": "left_link",  "body1": "right_grip",
                      "type": "revolute", "axis": "Z",  "limit": None,
                      "drive": None},
    # gripper finger prismatics (created in step8 from these entries), DOF +X
    "PrismaticJoint":  {"body0": "manip_base", "body1": "right_grip",
                        "type": "prismatic", "axis": "X",  "limit": None,
                        "drive": None},
    "PrismaticJoint0": {"body0": "manip_base", "body1": "left_grip",
                        "type": "prismatic", "axis": "X",  "limit": None,
                        "drive": None},
}

# ── Frame realignment targets ──────────────────────────────────────────────
# Target local orientation R_new (quaternion w,x,y,z) each body frame should have
# AFTER step9_realign_body_frames, expressed in the parent frame. Identity aligns a
# link's axes with its parent; since the chain up to the root is un-rotated, that
# is the world frame. step9 reads each link's CURRENT orient, sets the frame to
# the target, and rotates the JOINT anchors of joints touching that link so the
# joints stay physically in place. Meshes, centerOfMass, principalAxes, inertia
# and mass are NOT touched (mass properties are not authored until step11).
REALIGN_TARGETS: dict[str, tuple] = {
    "body":       (1.0, 0.0, 0.0, 0.0),
    "link2":      (1.0, 0.0, 0.0, 0.0),
    "link3":      (1.0, 0.0, 0.0, 0.0),
    "link4":      (1.0, 0.0, 0.0, 0.0),
    "manip_base": (1.0, 0.0, 0.0, 0.0),
    "hub":        (1.0, 0.0, 0.0, 0.0),
    "left_grip":  (1.0, 0.0, 0.0, 0.0),
    "right_grip": (1.0, 0.0, 0.0, 0.0),
    "left_link":  (1.0, 0.0, 0.0, 0.0),
    "right_link": (1.0, 0.0, 0.0, 0.0),
    "rotor0":     (1.0, 0.0, 0.0, 0.0),
    "rotor1":     (1.0, 0.0, 0.0, 0.0),
    "rotor2":     (1.0, 0.0, 0.0, 0.0),
    "rotor3":     (1.0, 0.0, 0.0, 0.0),
}

# ── Joint local-frame realignment (step10 fine-tuning) ─────────────────────
# step10 always runs: it re-expresses every joint's local frames to IDENTITY so
# the joint gizmo axes line up with the world-aligned bodies, and sets
# physics:axis + limits from JOINT_OVERRIDES (see step10's docstring for details).
# ⚠ For a joint whose DOF pointed along a NEGATIVE world axis, positive rotation
# now = +world axis (sign flipped). JOINT_OVERRIDES limits are already mirrored to
# match, but any external controller / command convention must flip too.
#
# Joints to leave untouched by step10 (by name) — keeps their current frame / axis
# / limits. Empty = normalize all joints.
JOINT_REALIGN_EXCLUDE: list[str] = []
# A joint's DOF must align with a world axis within this tolerance (dot ≥ value)
# to be snapped; a more-oblique joint is warned about and skipped (not forced).
JOINT_AXIS_ALIGN_TOL = 0.99   # ≈ 8.1° max off-axis

# ── Inertia representation ─────────────────────────────────────────────────
# How step11 authors each "inertia" tensor into USD (which stores only
# diagonalInertia + principalAxes):
#   "diagonal" = author diag(Ixx,Iyy,Izz) with identity principalAxes. Body/mass
#         axes stay world-aligned (Physics-debugger "Body Axes" match the body
#         frame); the off-diagonal products of inertia are dropped.
#   "exact" = eigen-diagonalize the full tensor: eigenvalues → diagonalInertia,
#         eigenvectors → principalAxes. Products of inertia preserved, but the
#         body/mass axes tilt to the true principal directions.
# Either way, step11 prints the true principal-axis tilt per link so you can see
# the difference (dropped in "diagonal", applied in "exact").
INERTIA_MODE = "diagonal"   # "diagonal" (aligned axes) or "exact" (tilted axes)

# ── Mass overrides ─────────────────────────────────────────────────────────
# Fill these from the OnShape mass properties report (Inspect > Mass Properties).
# key   = link name at vehicle root (after renames)
# value = dict with "mass" (kg), "com" (m), "inertia" (kg·m²).
#         "com" is measured in the BODY frame: in OnShape we add a mate connector
#         aligned with the corresponding body prim and read the center of mass
#         relative to it, so it maps directly to physics:centerOfMass.
#         "inertia" is the FULL 3x3 inertia tensor about the COM, measured in
#         OnShape in the COM frame — which shares the orientation of that mate
#         connector (= the body prim), so the tensor is expressed in the body
#         frame's orientation:
#             ( (Ixx, Ixy, Ixz),
#               (Ixy, Iyy, Iyz),
#               (Ixz, Iyz, Izz) )
#         Because step9 makes every body frame world-aligned and the mate
#         connectors match it, the measured tensor is already in the body frame.
#         USD only stores physics:diagonalInertia + physics:principalAxes; how
#         step11 reduces this tensor to those two attrs is controlled by
#         INERTIA_MODE above ("diagonal" = drop products / identity axes,
#         "exact" = eigen-diagonalization / tilted axes). A diagonal tensor yields
#         identity principalAxes in either mode.
#         NOTE: "inertia" may be a full 3x3 tensor (preferred) or a plain 3-tuple
#         of moments (treated as already-diagonal → zero tilt, both modes equal).
#
# TODO: replace with values from the gripper_bat OnShape mass properties report.
MASS_OVERRIDES: dict[str, dict] = {
    "body":       {"mass": 2.47607955,
                   "com": (0.00001074, -0.00032571, 0.04042483),
                   "inertia": (( 0.06301228,  0.00000185,  0.00000234),
                               ( 0.00000185,  0.06334175,  -0.0000542),
                               ( 0.00000234,  -0.0000542,  0.09868092))},
    "link2":      {"mass": 0.1048326,
                   "com": (0.0004604, -0.00048043, -0.03027735),
                   "inertia": ((  0.0000372,   -2.209e-8,   -6.364e-8),
                               (  -2.209e-8,    3.749e-5,    7.309e-7),
                               (  -6.364e-8,    7.309e-7,    2.161e-5))},
    "link3":      {"mass": 0.1423463,
                   "com": (0.02031963, 0.01008911, -0.09729514),
                   "inertia": (( 0.00035937,   -2.013e-7,    1.555e-6),
                               (  -2.013e-7,  0.00034859,    4.468e-5),
                               (   1.555e-6,    4.468e-5,    5.506e-5))},
    "link4":      {"mass": 0.13992727,
                   "com": (0.01939176, 0.08316447, -0.00086609),
                   "inertia": (( 0.00031302,   -2.098e-6,   -3.216e-9),
                               (  -2.098e-6,    3.253e-5,    3.340e-6),
                               (   3.340e-6,    3.340e-6,  0.00031847))},
    "manip_base": {"mass": 0.161,
                   "com": (0.00000296, 0.00356301, -0.02709745),
                   "inertia": ((   6.111e-5,    1.222e-7,    1.388e-8),
                               (   1.222e-7,    9.355e-5,    5.501e-6),
                               (   1.388e-8,    5.501e-6,  0.00010707))},
    "hub":        {"mass": 0.0096343,
                   "com": (0, 0, -0.00626411),
                   "inertia": ((   1.410e-6,   -3.709e-9,   4.589e-13),
                               (  -3.709e-9,    4.052e-7,   1.232e-10),
                               (  4.589e-13,   1.232e-10,    1.435e-6))},
    "left_link":  {"mass": 0.01048464,
                   "com": (-0.02, 0, -0.00175),
                   "inertia": ((   4.654e-7,   -8.394e-7,           0),
                               (  -8.394e-7,    2.208e-6,           0),
                               (          0,           0,    2.653e-6))},
    "right_link": {"mass": 0.01048464,
                   "com": (-0.02, 0, -0.00175),
                   "inertia": ((   4.564e-7,   -8.299e-7,           0),
                               (  -8.299e-7,    2.217e-6,           0),
                               (          0,           0,    2.653e-6))},
    "left_grip":  {"mass": 0.02895701,
                   "com": (-0.00830567, 2.225e-7, -0.01950101),
                   "inertia": ((   2.025e-5,   -1.617e-8,   -2.351e-6),
                               (  -1.617e-8,    1.578e-5,    8.407e-9),
                               (  -2.351e-6,    8.407e-9,    8.472e-6))},
    "right_grip": {"mass": 0.02895701,
                   "com": (0.00830573, -4.155e-7, -0.01950101),
                   "inertia": ((   2.025e-5,   -1.625e-8,    2.351e-6),
                               (  -1.625e-8,    1.578e-5,   -8.327e-9),
                               (   2.351e-6,   -8.327e-9,    8.471e-6))},
    "rotor0":     {"mass": 0.0398865,
                   "com": (0, 0, -0.00143967),
                   "inertia": ((   3.799e-4,   -1.642e-5,   6.768e-14),
                               (  -1.642e-5,    2.980e-6,   5.501e-14),
                               (  6.768e-14,   5.501e-14,  0.00038254))},
    "rotor1":     {"mass": 0.0398865,
                   "com": (0, 0, -0.00143967),
                   "inertia": (( 0.00037993,   -1.642e-5,   6.772e-14),
                               (  -1.642e-5,    2.980e-6,   5.691e-14),
                               (  6.772e-14,   5.691e-14,  0.00038254))},
    "rotor2":     {"mass": 0.0398865,
                   "com": (0, 0, -0.00143967),
                   "inertia": (( 0.00037994,    1.642e-5,   6.485e-14),
                               (   1.642e-5,    2.980e-6,  -5.825e-14),
                               (  6.485e-14,  -5.825e-14,  0.00038255))},
    "rotor3":     {"mass": 0.0398865,
                   "com": (0, 0, -0.00143967),
                   "inertia": (( 0.00037994,    1.642e-5,   6.467e-14),
                               (   1.642e-5,    2.980e-6,  -5.450e-14),
                               (  6.467e-14,  -5.450e-14,  0.00038255))},
}

# ── Mesh colliders ─────────────────────────────────────────────────────────
# Every mesh under the vehicle gets a collider so contacts are simulated.
# "convexDecomposition" approximates each mesh as a SET of convex hulls — far
# more accurate for concave parts (gripper fingers, arm links) than a single
# "convexHull", at higher cooking/runtime cost.
COLLISION_APPROXIMATION = "convexDecomposition"   # or "convexHull", "boundingCube", "none"
# Mesh path substrings to skip (no collider). Empty = collide every mesh.
# Tip: if the spinning rotors self-collide / destabilize the sim, add "rotor"
# here (or give those a cheaper "convexHull") — props rarely need exact hulls.
COLLIDER_EXCLUDE: list[str] = ["rotor"]   # props must not collide (eject/spin)
# Convex-decomposition tuning (higher = more hulls / finer detail = more
# accurate, slower to cook and simulate).
CONVEX_DECOMP = {
    "maxConvexHulls":  32,
    "hullVertexLimit": 64,
    "voxelResolution": 500_000,
}

# ============================================================================
# PART 2 · UTILITY & STEP FUNCTIONS   — no edits needed
# ============================================================================

# ── helper utilities ──────────────────────────────────────────────────────────

def _abs(relative_path: str) -> str:
    return f"{VEHICLE_ROOT_DST}/{relative_path}"


def _move(src: str, dst: str) -> bool:
    ok, _ = omni.kit.commands.execute(
        "MovePrimCommand",
        path_from=src,
        path_to=dst,
        keep_world_transform=False,
    )
    if not ok:
        carb.log_error(f"[POSTPROC] MovePrimCommand failed: {src} -> {dst}")
    return ok


def _joint_body_name(rel) -> str:
    """Last path component of a joint body0/body1 relationship (None if empty)."""
    t = rel.GetTargets()
    return t[0].name if t else None


def _swap_attr(a, b):
    """Swap the authored values of two attributes (skips a side that is unset)."""
    va, vb = a.Get(), b.Get()
    if vb is not None:
        a.Set(vb)
    if va is not None:
        b.Set(va)


_IDENTITY_QUATD = Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))


def _to_quatd(q) -> Gf.Quatd:
    """Gf.Quatf/Quatd/tuple(w,x,y,z) → Gf.Quatd."""
    if isinstance(q, (tuple, list)):
        return Gf.Quatd(float(q[0]), Gf.Vec3d(float(q[1]), float(q[2]), float(q[3])))
    im = q.GetImaginary()
    return Gf.Quatd(q.GetReal(), Gf.Vec3d(im[0], im[1], im[2]))


def _rotate_vec(qR: Gf.Quatd, v) -> Gf.Vec3d:
    """Rotate a vector by qR:  v' = qR · (0,v) · qR⁻¹."""
    qv = Gf.Quatd(0.0, Gf.Vec3d(v[0], v[1], v[2]))
    return (qR * qv * qR.GetInverse()).GetImaginary()


def _get_orient(xf: UsdGeom.Xformable) -> Gf.Quatd:
    """Current orient op of an Xformable as Quatd (identity if none)."""
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            val = op.Get()
            return _to_quatd(val) if val is not None else Gf.Quatd(_IDENTITY_QUATD)
    return Gf.Quatd(_IDENTITY_QUATD)


def _set_orient(xf: UsdGeom.Xformable, q: Gf.Quatd):
    """Set (or add) the orient op, preserving any translate and its precision."""
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            if op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
                op.Set(Gf.Quatd(q.GetReal(), Gf.Vec3d(q.GetImaginary())))
            else:
                im = q.GetImaginary()
                op.Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(im[0], im[1], im[2])))
            return
    op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    op.Set(Gf.Quatd(q.GetReal(), Gf.Vec3d(q.GetImaginary())))


_AXIS_VEC = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


def _negate_authored(prim, attr_name) -> bool:
    """Negate an attribute in place iff it is authored. Returns True if changed."""
    a = prim.GetAttribute(attr_name)
    if a and a.HasAuthoredValue():
        v = a.Get()
        if v is not None:
            a.Set(-v)
            return True
    return False


def _set_or_clear(prim, attr_name, value):
    """Author a float attr, or Clear() it when value is None (that side stays open)."""
    a = prim.GetAttribute(attr_name)
    if not a:
        return
    if value is None:
        a.Clear()
    else:
        a.Set(float(value))


def _matrix_to_quatf(R) -> Gf.Quatf:
    """Convert a 3x3 proper rotation matrix (columns = axis directions) to a
    Gf.Quatf(w, x, y, z)."""
    m00, m11, m22 = R[0, 0], R[1, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif m00 > m11 and m00 > m22:
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return Gf.Quatf(float(w), float(x), float(y), float(z))


def _fmt_deg(angles) -> str:
    """Format a tuple of angles (degrees) as e.g. '(8.5deg, 8.5deg, 0.3deg)'."""
    return "(" + ", ".join(f"{float(a):.1f}deg" for a in angles) + ")"


def _analyze_principal_axes(inertia) -> dict:
    """Eigen-decompose a 3x3 inertia tensor (about the COM, in the body frame)
    and describe both representations plus how far the principal axes tilt from
    the body/identity axes. Accepts a full 3x3 tensor or a plain 3-tuple/vector
    of moments (already diagonal → identity axes, zero tilt).

    Returns dict:
      diag       : (Ixx, Iyy, Izz)  raw body-frame diagonal    → Option A moments
      eigvals    : (I1, I2, I3)      principal moments, ascending → Option B moments
      quat       : Gf.Quatf          principalAxes = eigenvectors → Option B axes
      quat_angle : float, deg        net rotation of `quat`. NOTE this includes
                                     the axis RELABELING from eigh's ascending
                                     sort, so it can read ~90/180° even when the
                                     physical tilt is tiny — read `axis_tilt`/
                                     `max_tilt` for the true misalignment.
      axis_tilt  : (aX, aY, aZ) deg  angle from each BODY axis to its NEAREST
                                     principal axis — order/sign independent,
                                     i.e. the TRUE physical misalignment.
      max_tilt   : float, deg        max(axis_tilt): worst-case physical tilt.
    """
    arr = np.asarray(inertia, dtype=float)
    if arr.shape == (3,):
        arr = np.diag(arr)
    if arr.shape != (3, 3):
        raise ValueError(f"inertia must be 3x3 or length-3, got shape {arr.shape}")

    sym = 0.5 * (arr + arr.T)   # symmetrize to kill any numeric asymmetry
    diag = (float(sym[0, 0]), float(sym[1, 1]), float(sym[2, 2]))

    # eigh: ascending eigenvalues, orthonormal eigenvectors as COLUMNS (each
    # column = a principal axis expressed in the body frame).
    eigvals, eigvecs = np.linalg.eigh(sym)
    if np.linalg.det(eigvecs) < 0.0:      # force a right-handed (proper) rotation
        eigvecs[:, 0] = -eigvecs[:, 0]
    quat = _matrix_to_quatf(eigvecs)
    quat_angle = float(np.degrees(2.0 * np.arccos(min(1.0, abs(float(quat.GetReal()))))))

    # For each body axis e_i, the largest |component| across the principal axes
    # (row i of eigvecs) is its alignment with the nearest principal axis; the
    # angle to it is label/sign independent, so it ignores the eigh sort permute.
    axis_tilt = tuple(
        float(np.degrees(np.arccos(min(1.0, float(np.max(np.abs(eigvecs[i, :])))))))
        for i in range(3)
    )
    return {
        "diag": diag,
        "eigvals": (float(eigvals[0]), float(eigvals[1]), float(eigvals[2])),
        "quat": quat,
        "quat_angle": quat_angle,
        "axis_tilt": axis_tilt,
        "max_tilt": max(axis_tilt),
    }


# ── pipeline steps (executed in order by run()) ───────────────────────────────

def step1_move_vehicle(stage: Usd.Stage) -> bool:
    src_prim = stage.GetPrimAtPath(VEHICLE_ROOT_SRC)
    if not src_prim or not src_prim.IsValid():
        carb.log_error(f"[POSTPROC] Source prim not found: {VEHICLE_ROOT_SRC}")
        carb.log_error("[POSTPROC] Check VEHICLE_ROOT_SRC against the Stage panel.")
        return False
    if VEHICLE_ROOT_SRC == VEHICLE_ROOT_DST:
        print("[POSTPROC] Step 1 skipped - src == dst.", flush=True)
        return True
    print(f"[POSTPROC] Step 1: {VEHICLE_ROOT_SRC} -> {VEHICLE_ROOT_DST}", flush=True)
    return _move(VEHICLE_ROOT_SRC, VEHICLE_ROOT_DST)


def step2_delete_wrappers(stage: Usd.Stage):
    print("[POSTPROC] Step 2: removing wrapper xforms...", flush=True)
    for path in WRAPPER_XFORMS_TO_DELETE:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            print(f"  already gone: {path}", flush=True)
            continue
        children = list(prim.GetChildren())
        if children:
            print(f"  skipping non-empty xform {path} ({len(children)} children)", flush=True)
            continue
        stage.RemovePrim(Sdf.Path(path))
        print(f"  deleted: {path}", flush=True)


def step3_set_default_prim(stage: Usd.Stage) -> bool:
    print(f"[POSTPROC] Step 3: defaultPrim -> {VEHICLE_ROOT_DST}", flush=True)
    vehicle = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not vehicle or not vehicle.IsValid():
        carb.log_error(f"[POSTPROC] Vehicle prim missing at {VEHICLE_ROOT_DST}")
        return False
    stage.SetDefaultPrim(vehicle)
    return True


def step4_rename_links(stage: Usd.Stage):
    if not LINK_RENAMES:
        print("[POSTPROC] Step 4 skipped - no link renames configured.", flush=True)
        return
    print("[POSTPROC] Step 4: renaming links...", flush=True)
    for old_name, new_name in LINK_RENAMES.items():
        src = _abs(old_name)
        dst = _abs(new_name)
        prim = stage.GetPrimAtPath(src)
        if not prim or not prim.IsValid():
            carb.log_warn(f"  link not found (already renamed?): {src}")
            continue
        ok = _move(src, dst)
        print(f"  {'OK' if ok else 'FAIL'}: {old_name} -> {new_name}", flush=True)


def step5_clear_instanceable(stage: Usd.Stage):
    """Recursively set instanceable=False on the entire vehicle subtree.

    OnShape imports links with instanceable=True, which (a) blocks MovePrimCommand
    from placing joints under them and (b) prevents PhysX from syncing cooked
    colliders to Fabric ("Cannot sync Solid Collision Mesh to Fabric") because the
    mesh sits behind an instance.

    Must walk top-down on REAL prims (not instance proxies, which are read-only):
    clear a prim's instanceable flag first, then its authored children become
    traversable. A plain PrimRange snapshot won't see children newly exposed by
    clearing, so we use an explicit stack instead.
    """
    print("[POSTPROC] Step 5: clearing instanceable on whole subtree...", flush=True)
    vehicle = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not vehicle or not vehicle.IsValid():
        carb.log_error(f"[POSTPROC] Vehicle prim missing: {VEHICLE_ROOT_DST}")
        return
    count = 0
    stack = [vehicle]
    while stack:
        prim = stack.pop()
        if prim.IsInstanceable():
            prim.SetInstanceable(False)   # do this BEFORE reading children
            count += 1
            print(f"  cleared: {prim.GetPath().pathString}", flush=True)
        stack.extend(prim.GetChildren())
    print(f"  {count} prim(s) cleared.", flush=True)


def step6_reparent_joints(stage: Usd.Stage):
    if not JOINT_REPARENTS:
        print("[POSTPROC] Step 6 skipped - no joint reparents configured.", flush=True)
        return
    print("[POSTPROC] Step 6: reparenting joints...", flush=True)
    for joint_name, parent_name in JOINT_REPARENTS.items():
        src = _abs(joint_name)
        dst = _abs(f"{parent_name}/{joint_name}")
        prim = stage.GetPrimAtPath(src)
        if not prim or not prim.IsValid():
            carb.log_warn(f"  joint not found (already moved?): {src}")
            continue
        ok = _move(src, dst)
        print(f"  {'OK' if ok else 'FAIL'}: {joint_name} -> {parent_name}/{joint_name}", flush=True)


def step7_set_joint_bodies(stage: Usd.Stage):
    """Bring each joint's body0/body1 to its JOINT_OVERRIDES assignment.

    Reproduces the Physics-UI "swap bodies" button (which is what fixing body0/
    body1 by hand actually does): when a joint is the exact REVERSE of its target
    it swaps body0<->body1 AND localPos0<->localPos1 AND localRot0<->localRot1, so
    the joint stays physically in place and the gizmo is correct. It does NOT touch
    axis, limits, drive, or joint state. Per-joint idempotent: a joint already
    matching its target is left untouched, so re-runs and mixed states converge.

    The two prismatic joints are listed in JOINT_OVERRIDES too but are created
    later (step8), so they show up as 'not found' here and get their bodies then.
    """
    if not JOINT_OVERRIDES:
        print("[POSTPROC] Step 7 skipped - no joints configured.", flush=True)
        return
    print("[POSTPROC] Step 7: setting joint body0/body1 targets...", flush=True)
    root = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not root or not root.IsValid():
        carb.log_error(f"[POSTPROC] Vehicle root missing: {VEHICLE_ROOT_DST}")
        return
    joints_by_name = {p.GetName(): p for p in Usd.PrimRange(root)
                      if p.IsA(UsdPhysics.Joint)}
    swapped = already = 0
    for jname, tgt in JOINT_OVERRIDES.items():
        prim = joints_by_name.get(jname)
        if prim is None:
            carb.log_warn(f"  joint not found (skipped): {jname}")
            continue
        jt = UsdPhysics.Joint(prim)
        r0, r1 = jt.GetBody0Rel(), jt.GetBody1Rel()
        b0, b1 = _joint_body_name(r0), _joint_body_name(r1)
        want0, want1 = tgt["body0"], tgt["body1"]

        if b0 == want0 and b1 == want1:
            already += 1
            print(f"  {jname}: already body0={want0}, body1={want1}", flush=True)
            continue
        if b0 == want1 and b1 == want0:
            t0, t1 = r0.GetTargets(), r1.GetTargets()
            r0.SetTargets(t1)
            r1.SetTargets(t0)
            _swap_attr(jt.GetLocalPos0Attr(), jt.GetLocalPos1Attr())
            _swap_attr(jt.GetLocalRot0Attr(), jt.GetLocalRot1Attr())
            swapped += 1
            print(f"  {jname}: swapped -> body0={want0}, body1={want1}", flush=True)
            continue
        carb.log_warn(f"  {jname}: current (body0={b0}, body1={b1}) matches neither "
                      f"target order (body0={want0}, body1={want1}) - skipped.")
    print(f"  {swapped} swapped, {already} already correct.", flush=True)


def step8_add_prismatic_joints(stage: Usd.Stage):
    """Create PhysicsPrismaticJoint prims for the gripper finger constraints.

    The anchor frames are NOT hardcoded — they are computed from the two bodies'
    current world poses, exactly like the Physics-UI "select two bodies → create
    joint" does: the joint frame is anchored at body0's origin
    (localPos0 = 0, localRot0 = identity) and body1's local frame is derived from
    the relative pose so the joint is satisfied at the current configuration:

        L1 = T(body0)_world · T(body1)_world⁻¹    (→ localPos1 / localRot1)

    Runs before step9/step10, so the bodies are still at their imported poses;
    step9 then rotates these anchors to keep the joint in place while the frames
    realign, and step10 normalizes localRot to identity + sets the axis.
    Which joints to create, plus body0/body1 and axis, come from JOINT_OVERRIDES
    (the single source of truth): every entry with "type" == "prismatic".
    """
    prismatics = [(n, s) for n, s in JOINT_OVERRIDES.items()
                  if s.get("type") == "prismatic"]
    if not prismatics:
        print("[POSTPROC] Step 8 skipped - no prismatic joints in JOINT_OVERRIDES.", flush=True)
        return
    print("[POSTPROC] Step 8: adding prismatic joints...", flush=True)
    xf_cache = UsdGeom.XformCache()
    for name, tgt in prismatics:
        body0, body1 = tgt["body0"], tgt["body1"]

        b0_prim = stage.GetPrimAtPath(_abs(body0))
        b1_prim = stage.GetPrimAtPath(_abs(body1))
        if not (b0_prim and b0_prim.IsValid() and b1_prim and b1_prim.IsValid()):
            carb.log_warn(f"  {name}: body prim(s) missing ({body0}, {body1}) - skipped.")
            continue

        # Joint frame at body0's origin; body1 side = its relative pose. USD uses
        # row-vector convention, so L1 = T0 · T1⁻¹ maps body0 coords → body1 coords.
        T0 = xf_cache.GetLocalToWorldTransform(b0_prim)
        T1 = xf_cache.GetLocalToWorldTransform(b1_prim)
        L1 = T0 * T1.GetInverse()
        pos1 = L1.ExtractTranslation()
        rot1 = L1.ExtractRotation().GetQuat()     # Gf.Quatd(w, (x, y, z))
        im1 = rot1.GetImaginary()

        # The joint prim lives under body0 (organizational only — it references its
        # bodies by relationship, so the location does not affect the physics).
        path = _abs(f"{body0}/{name}")
        prim_path = Sdf.Path(path)

        # Remove stale prim if re-running the script
        if stage.GetPrimAtPath(prim_path).IsValid():
            stage.RemovePrim(prim_path)

        joint = UsdPhysics.PrismaticJoint.Define(stage, prim_path)
        joint.CreateAxisAttr(tgt["axis"])
        joint.CreateBody0Rel().SetTargets([Sdf.Path(_abs(body0))])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(_abs(body1))])

        joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(float(pos1[0]), float(pos1[1]), float(pos1[2])))
        joint.CreateLocalRot1Attr(Gf.Quatf(float(rot1.GetReal()),
                                           Gf.Vec3f(float(im1[0]), float(im1[1]), float(im1[2]))))

        joint.CreateBreakForceAttr(3.4028235e38)
        joint.CreateBreakTorqueAttr(3.4028235e38)
        joint.CreateExcludeFromArticulationAttr(True)

        print(f"  created: {body0}/{name}  ({body0} <-> {body1})  "
              f"localPos1=({pos1[0]:.5f}, {pos1[1]:.5f}, {pos1[2]:.5f})", flush=True)


def step9_realign_body_frames(stage: Usd.Stage):
    """Re-orient each body frame in REALIGN_TARGETS to its target, keeping the
    MESHES and JOINTS physically in place (mass properties are NOT touched).

    Per link:  R = R_new⁻¹ · R_old   (read from the link's current orient)
      - frame:  xformOp:orient → R_new                       (translate unchanged)
      - mesh:   xformOp:orient → R · mesh_orient   (only the mesh's transform op
                is rotated, never its vertices, so geometry stays put in world)
      - joints: every joint side referencing a realigned body has its anchor
                rotated by that body's R:  localPos → R·localPos,
                localRot → R·localRot   (so the joint stays where it was).
    centerOfMass / principalAxes / diagonalInertia / mass are intentionally left
    alone — they are not authored until step11. Idempotent: once a frame sits at
    its target, R collapses to identity and re-runs are no-ops.
    """
    if not REALIGN_TARGETS:
        print("[POSTPROC] Step 9 skipped - no realign targets configured.", flush=True)
        return
    print("[POSTPROC] Step 9: realigning body frames (frames + meshes + joints)...", flush=True)
    root = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not root or not root.IsValid():
        carb.log_error(f"[POSTPROC] Vehicle root missing: {VEHICLE_ROOT_DST}")
        return

    # 1) Read current orient and compute R per link BEFORE touching any frame.
    plan = {}
    for name, target in REALIGN_TARGETS.items():
        prim = stage.GetPrimAtPath(_abs(name))
        if not prim or not prim.IsValid():
            carb.log_warn(f"  link not found (skipped): {_abs(name)}")
            continue
        R_old = _get_orient(UsdGeom.Xformable(prim))
        R_new = _to_quatd(target)
        plan[name] = {"R": R_new.GetInverse() * R_old, "R_new": R_new}
    if not plan:
        carb.log_error("[POSTPROC] Nothing to realign - no valid links.")
        return

    # 2) Frame: orient → R_new.
    for name, p in plan.items():
        _set_orient(UsdGeom.Xformable(stage.GetPrimAtPath(_abs(name))), p["R_new"])
        print(f"  {name}: frame orient -> target", flush=True)

    # 3) Meshes: rotate each body's mesh transform by that body's R so the
    #    geometry stays put in world (only the transform op is touched, never the
    #    mesh vertices). Instanceable was cleared in step5, so PrimRange descends
    #    into the referenced prototype and reaches the real Mesh prims.
    for name, p in plan.items():
        link = stage.GetPrimAtPath(_abs(name))
        for mesh in (pr for pr in Usd.PrimRange(link) if pr.IsA(UsdGeom.Mesh)):
            xf = UsdGeom.Xformable(mesh)
            _set_orient(xf, p["R"] * _get_orient(xf))
            print(f"  {name}: mesh {mesh.GetName()} compensated", flush=True)

    # 4) Joints: rotate each side's anchor by the R of the body it references.
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        jt = UsdPhysics.Joint(prim)
        for body_rel, pos_attr, rot_attr in (
            (jt.GetBody0Rel(), jt.GetLocalPos0Attr(), jt.GetLocalRot0Attr()),
            (jt.GetBody1Rel(), jt.GetLocalPos1Attr(), jt.GetLocalRot1Attr()),
        ):
            bname = _joint_body_name(body_rel)
            if bname not in plan:
                continue
            R = plan[bname]["R"]
            pos = pos_attr.Get()
            if pos is not None:
                pn = _rotate_vec(R, pos)
                pos_attr.Set(Gf.Vec3f(pn[0], pn[1], pn[2]))
            rot = rot_attr.Get()
            if rot is not None:
                rn = R * _to_quatd(rot)
                im = rn.GetImaginary()
                rot_attr.Set(Gf.Quatf(rn.GetReal(), Gf.Vec3f(im[0], im[1], im[2])))
            print(f"  joint {prim.GetName()}: anchor on {bname} rotated by R", flush=True)


def step10_realign_joint_frames(stage: Usd.Stage):
    """Re-express every joint's local frames (localRot0/localRot1) as IDENTITY so
    the joint gizmo axes align with the world-aligned bodies, WITHOUT moving the
    joint, then apply each joint's declared axis + limits from JOINT_OVERRIDES.

    Per joint (driven by JOINT_OVERRIDES):
      1. After step9 the joint's two frames coincide in world, so localRot0 IS the
         joint frame's world orientation; the current DOF world direction is
         d = R(localRot0) · e_(current axis).
      2. Verify d snaps to a world axis (|d·axis| ≥ JOINT_AXIS_ALIGN_TOL) and that
         its dominant letter matches the declared "axis"; warn + skip on mismatch.
      3. Set localRot0 = localRot1 = identity and physics:axis = declared axis.
      4. Apply the declared "limit" (already in the post-realign convention).
      5. If d pointed along the NEGATIVE world axis the DOF sign flips (physics:axis
         is always positive); the declared limits already account for that, and any
         authored drive target / joint state is negated to stay consistent.

    Must run AFTER step9 (frames coincident) and step8 (prismatics exist).
    """
    print("[POSTPROC] Step 10: realigning joint frames + axis + limits...", flush=True)
    root = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not root or not root.IsValid():
        carb.log_error(f"[POSTPROC] Vehicle root missing: {VEHICLE_ROOT_DST}")
        return
    joints_by_name = {p.GetName(): p for p in Usd.PrimRange(root)
                      if p.IsA(UsdPhysics.Joint)}
    ident = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    done = flipped = skipped = 0
    for jname, spec in JOINT_OVERRIDES.items():
        if jname in JOINT_REALIGN_EXCLUDE:
            print(f"  {jname}: excluded - left unchanged.", flush=True)
            continue
        prim = joints_by_name.get(jname)
        if prim is None:
            carb.log_warn(f"  {jname}: joint not found - skipped.")
            skipped += 1
            continue
        inst = "angular" if spec.get("type") == "revolute" else "linear"
        declared = spec.get("axis")
        if declared not in _AXIS_VEC:
            carb.log_warn(f"  {jname}: declared axis {declared!r} invalid - skipped.")
            skipped += 1
            continue

        jt = UsdPhysics.Joint(prim)
        cur_axis_attr = prim.GetAttribute("physics:axis")
        cur_letter = cur_axis_attr.Get() if cur_axis_attr else None
        q0 = jt.GetLocalRot0Attr().Get()
        Q0 = _to_quatd(q0) if q0 is not None else Gf.Quatd(_IDENTITY_QUATD)
        probe = cur_letter if cur_letter in _AXIS_VEC else declared
        d = _rotate_vec(Q0, _AXIS_VEC[probe])             # current DOF dir in world
        comps = [abs(d[0]), abs(d[1]), abs(d[2])]
        i = comps.index(max(comps))
        geo_letter = "XYZ"[i]
        if comps[i] < JOINT_AXIS_ALIGN_TOL:
            carb.log_warn(f"  {jname}: DOF {tuple(round(x, 3) for x in d)} not world-"
                          f"aligned (max|comp|={comps[i]:.3f}) - skipped.")
            skipped += 1
            continue
        if geo_letter != declared:
            carb.log_warn(f"  {jname}: declared axis '{declared}' != geometric "
                          f"'{geo_letter}' - skipped (fix JOINT_OVERRIDES).")
            skipped += 1
            continue
        reversed_dof = d[i] < 0.0

        # Identity frames + declared axis.
        jt.GetLocalRot0Attr().Set(ident)
        jt.GetLocalRot1Attr().Set(ident)
        if cur_axis_attr:
            cur_axis_attr.Set(declared)

        # Limits, in the post-realign convention (None side = open).
        lim = spec.get("limit")
        lo, hi = (None, None) if lim is None else (lim[0], lim[1])
        _set_or_clear(prim, "physics:lowerLimit", lo)
        _set_or_clear(prim, "physics:upperLimit", hi)

        note = f"axis {cur_letter}->{declared}, limit={lim}"
        if reversed_dof:
            # + now = +world axis (was -world axis): declared limits already mirror
            # this; negate any authored target/state to stay consistent (0 at import).
            _negate_authored(prim, f"drive:{inst}:physics:targetPosition")
            _negate_authored(prim, f"drive:{inst}:physics:targetVelocity")
            _negate_authored(prim, f"state:{inst}:physics:position")
            _negate_authored(prim, f"state:{inst}:physics:velocity")
            note += ", DOF sign flipped"
            flipped += 1
        done += 1
        print(f"  {jname}: frames->identity, {note}", flush=True)
    print(f"  {done} joint(s) configured ({flipped} DOF-flipped), {skipped} skipped.", flush=True)


def step11_set_inertial_properties(stage: Usd.Stage):
    """Author mass, center of mass, and inertia on each body in MASS_OVERRIDES.

    The imported model already provides rigid bodies, so we only set the inertial
    parameters here: mass, centerOfMass (body frame), and the inertia tensor.
    How the tensor is authored depends on INERTIA_MODE:

      "diagonal" — author diag(Ixx,Iyy,Izz) with an identity principalAxes. The
            Physics-debugger "Body Axes" (the mass frame) stay aligned with the
            world-aligned body frame; the off-diagonal products of inertia are
            dropped.
      "exact" — eigen-diagonalize the full tensor into diagonalInertia
            (eigenvalues) + principalAxes (eigenvectors). Products of inertia are
            preserved, but the body/mass axes tilt to the true principal axes.

    In BOTH modes the true principal-axis tilt is computed and printed, so you
    can see the difference: in "diagonal" it is the misalignment you are DROPPING;
    in "exact" it is the misalignment you are APPLYING. `max_tilt`/`axis_tilt` are
    the physical angles (label/sign independent); the raw quaternion angle is also
    shown in "exact" but includes the eigen-sort axis relabeling (see
    _analyze_principal_axes).
    """
    if not MASS_OVERRIDES:
        print("[POSTPROC] Step 11 skipped - no mass overrides configured.", flush=True)
        return
    if INERTIA_MODE not in ("diagonal", "exact"):
        carb.log_error(f"[POSTPROC] INERTIA_MODE must be 'diagonal' or 'exact', got {INERTIA_MODE!r}")
        return
    label = ("diagonal (drop products, identity axes)" if INERTIA_MODE == "diagonal"
             else "exact (full tensor, tilted principal axes)")
    print(f"[POSTPROC] Step 11: setting inertial properties (mass, com, inertia) "
          f"- mode {label}...", flush=True)
    for link_name, props in MASS_OVERRIDES.items():
        path = _abs(link_name)
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            carb.log_warn(f"  link not found: {path}")
            continue

        # Mass + center of mass. "com" is authored in the body frame, which maps
        # directly to physics:centerOfMass (see MASS_OVERRIDES notes).
        mass_api = (UsdPhysics.MassAPI(prim) if prim.HasAPI(UsdPhysics.MassAPI)
                    else UsdPhysics.MassAPI.Apply(prim))
        mass_api.CreateMassAttr().Set(float(props["mass"]))
        if "com" in props:
            cx, cy, cz = props["com"]
            mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(cx, cy, cz))

        msg = f"  {link_name}: mass={props['mass']} kg"
        if "inertia" in props:
            rpt = _analyze_principal_axes(props["inertia"])
            if INERTIA_MODE == "diagonal":
                ix, iy, iz = rpt["diag"]
                mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(ix, iy, iz))
                mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                msg += (f"  [diagonal] diag=({ix:.3e}, {iy:.3e}, {iz:.3e}) principalAxes=identity"
                        f"  | DROPPED tilt: max {rpt['max_tilt']:.1f}deg "
                        f"per-axis {_fmt_deg(rpt['axis_tilt'])}")
            else:  # "exact"
                ix, iy, iz = rpt["eigvals"]
                q = rpt["quat"]
                im = q.GetImaginary()
                mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(ix, iy, iz))
                mass_api.CreatePrincipalAxesAttr().Set(q)
                msg += (f"  [exact] eig=({ix:.3e}, {iy:.3e}, {iz:.3e}) "
                        f"principalAxes=({q.GetReal():.4f}, {im[0]:.4f}, {im[1]:.4f}, {im[2]:.4f})"
                        f"  | APPLIED tilt: max {rpt['max_tilt']:.1f}deg "
                        f"per-axis {_fmt_deg(rpt['axis_tilt'])} "
                        f"(quat angle {rpt['quat_angle']:.1f}deg incl. axis relabel)")
        print(msg, flush=True)


def step12_add_colliders(stage: Usd.Stage):
    """Apply a collider to every mesh under the vehicle, using the configured
    approximation (convex decomposition by default for accurate concave shapes).

    Walks the whole vehicle subtree, so it covers body, arm links, gripper
    fingers and rotors. Re-running is safe — Apply() is idempotent and just
    re-sets the approximation token / decomposition params.
    """
    print(f"[POSTPROC] Step 12: adding mesh colliders ({COLLISION_APPROXIMATION})...", flush=True)
    vehicle = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not vehicle or not vehicle.IsValid():
        carb.log_error(f"[POSTPROC] Vehicle prim missing: {VEHICLE_ROOT_DST}")
        return
    count = skipped = 0
    # step5 has already cleared instanceable on the whole subtree, so a plain
    # PrimRange reaches every (now-real) mesh prim and can author colliders on it.
    for prim in Usd.PrimRange(vehicle):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        path_str = prim.GetPath().pathString
        if any(ex in path_str for ex in COLLIDER_EXCLUDE):
            print(f"  skipped (excluded): {path_str}", flush=True)
            skipped += 1
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        mesh_col = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_col.CreateApproximationAttr().Set(COLLISION_APPROXIMATION)
        if COLLISION_APPROXIMATION == "convexDecomposition":
            cda = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            cda.CreateMaxConvexHullsAttr().Set(int(CONVEX_DECOMP["maxConvexHulls"]))
            cda.CreateHullVertexLimitAttr().Set(int(CONVEX_DECOMP["hullVertexLimit"]))
            cda.CreateVoxelResolutionAttr().Set(int(CONVEX_DECOMP["voxelResolution"]))
        count += 1
        print(f"  collider: {path_str}", flush=True)
    print(f"  {count} collider(s) added, {skipped} skipped.", flush=True)


def step13_apply_joint_drives(stage: Usd.Stage):
    """Apply PhysicsDriveAPI gains (stiffness/damping/maxForce) from JOINT_OVERRIDES
    to every driven joint. Passive joints ("drive": None) are left undriven."""
    if not JOINT_OVERRIDES:
        print("[POSTPROC] Step 13 skipped - no joints configured.", flush=True)
        return
    print("[POSTPROC] Step 13: applying joint drives...", flush=True)
    root = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not root or not root.IsValid():
        carb.log_error(f"[POSTPROC] Vehicle root missing: {VEHICLE_ROOT_DST}")
        return
    joints_by_name = {p.GetName(): p for p in Usd.PrimRange(root)
                      if p.IsA(UsdPhysics.Joint)}
    n = 0
    for jname, spec in JOINT_OVERRIDES.items():
        drv = spec.get("drive")
        if not drv:
            continue
        prim = joints_by_name.get(jname)
        if prim is None:
            carb.log_warn(f"  {jname}: joint not found - skipped.")
            continue
        inst = "angular" if spec.get("type") == "revolute" else "linear"
        drive = UsdPhysics.DriveAPI.Apply(prim, inst)
        drive.CreateStiffnessAttr().Set(float(drv["stiffness"]))
        drive.CreateDampingAttr().Set(float(drv["damping"]))
        drive.CreateMaxForceAttr().Set(float(drv["maxForce"]))
        n += 1
        print(f"  {jname}: {inst} drive  k={drv['stiffness']}  d={drv['damping']}  "
              f"maxF={drv['maxForce']}", flush=True)
    print(f"  {n} drive(s) applied.", flush=True)


def step14_draw_joint_frames(stage: Usd.Stage):
    """DEBUG-ONLY: draw each joint's AUTHORED local frames as solid RGB axis
    segments, so you can actually verify localRot0/localRot1.

    Isaac's built-in joint gizmo shows PhysX's *cooked* frame (rotation axis put
    on X), NOT localRot0/1 — so it can't be trusted to check the authored frames.
    This step draws the real ones directly:
      - frame 0 (body0 side): SHORTER + LIGHTER
      - frame 1 (body1 side): LONGER  + DARKER
      - axes X=red, Y=green, Z=blue
    Both frames sit at the joint anchor; if the joint is assembled correctly they
    overlap and are world-aligned. Curves live under <root>/_joint_frames.
    Comment this step out of run() for the final asset (viewing-only geometry).
    """
    # ── tunables ────────────────────────────────────────────────────────────
    LEN0, LEN1 = 0.030, 0.055     # axis lengths (m): frame0 shorter, frame1 longer
    WID0, WID1 = 0.0015, 0.0025   # line widths (m): frame0 thinner, frame1 thicker
    COL0 = [Gf.Vec3f(1.0, 0.5, 0.5), Gf.Vec3f(0.5, 1.0, 0.5), Gf.Vec3f(0.5, 0.5, 1.0)]  # light RGB
    COL1 = [Gf.Vec3f(0.6, 0.0, 0.0), Gf.Vec3f(0.0, 0.5, 0.0), Gf.Vec3f(0.0, 0.0, 0.6)]  # dark  RGB
    # ─────────────────────────────────────────────────────────────────────────

    print("[POSTPROC] Step 14: drawing joint local frames (debug)...", flush=True)
    root = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not root or not root.IsValid():
        carb.log_error(f"[POSTPROC] Vehicle root missing: {VEHICLE_ROOT_DST}")
        return

    scope_path = f"{VEHICLE_ROOT_DST}/_joint_frames"
    if stage.GetPrimAtPath(Sdf.Path(scope_path)).IsValid():
        stage.RemovePrim(Sdf.Path(scope_path))        # clear stale draw on re-run
    UsdGeom.Scope.Define(stage, Sdf.Path(scope_path))

    xf = UsdGeom.XformCache()
    T_root_inv = xf.GetLocalToWorldTransform(root).GetInverse()

    def _frame_in_root(pos_attr, rot_attr, body_prim):
        """World pose of a joint's local frame, expressed in the vehicle-root frame."""
        pos = pos_attr.Get() or Gf.Vec3f(0.0, 0.0, 0.0)
        rot = rot_attr.Get()
        q = _to_quatd(rot) if rot is not None else Gf.Quatd(_IDENTITY_QUATD)
        m = Gf.Matrix4d(1.0)
        m.SetRotate(q)
        m.SetTranslateOnly(Gf.Vec3d(pos[0], pos[1], pos[2]))
        # frame->body  *  body->world  *  world->root
        return m * xf.GetLocalToWorldTransform(body_prim) * T_root_inv

    def _draw_axes(path, F, length, colors, width):
        o = F.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
        ex = F.Transform(Gf.Vec3d(length, 0.0, 0.0))
        ey = F.Transform(Gf.Vec3d(0.0, length, 0.0))
        ez = F.Transform(Gf.Vec3d(0.0, 0.0, length))
        crv = UsdGeom.BasisCurves.Define(stage, Sdf.Path(path))
        crv.CreateTypeAttr(UsdGeom.Tokens.linear)
        crv.CreateCurveVertexCountsAttr([2, 2, 2])       # 3 segments: X, Y, Z
        crv.CreatePointsAttr([Gf.Vec3f(p) for p in (o, ex, o, ey, o, ez)])
        crv.CreateWidthsAttr([width])
        crv.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        crv.CreateDisplayColorAttr(colors)               # one color per segment
        crv.GetDisplayColorPrimvar().SetInterpolation(UsdGeom.Tokens.uniform)

    n = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        jt = UsdPhysics.Joint(prim)
        name = prim.GetName()
        b0, b1 = _joint_body_name(jt.GetBody0Rel()), _joint_body_name(jt.GetBody1Rel())
        p0 = stage.GetPrimAtPath(_abs(b0)) if b0 else None
        p1 = stage.GetPrimAtPath(_abs(b1)) if b1 else None
        if not (p0 and p0.IsValid() and p1 and p1.IsValid()):
            carb.log_warn(f"  {name}: body prim(s) missing ({b0}, {b1}) - skipped.")
            continue
        F0 = _frame_in_root(jt.GetLocalPos0Attr(), jt.GetLocalRot0Attr(), p0)
        F1 = _frame_in_root(jt.GetLocalPos1Attr(), jt.GetLocalRot1Attr(), p1)
        _draw_axes(f"{scope_path}/{name}_f0", F0, LEN0, COL0, WID0)
        _draw_axes(f"{scope_path}/{name}_f1", F1, LEN1, COL1, WID1)
        n += 1
        print(f"  {name}: drew frame0 (body0={b0}) + frame1 (body1={b1})", flush=True)
    print(f"  {n} joint frame-pair(s) drawn under {scope_path}.", flush=True)


def step15_export(stage: Usd.Stage):
    print(f"[POSTPROC] Step 15: exporting flattened USD -> {OUTPUT_USD}", flush=True)
    flat_layer = stage.Flatten()
    flat_layer.Export(OUTPUT_USD)
    print(f"[POSTPROC] Saved: {OUTPUT_USD}", flush=True)


# ============================================================================
# PART 3 · MAIN
# ============================================================================

def run():
    stage = omni.usd.get_context().get_stage()
    print("=" * 60, flush=True)
    print("[POSTPROC] Starting OnShape USD post-processor...", flush=True)
    print("=" * 60, flush=True)
    
    # Pipeline steps in execution order: (number, label, callable). A step that
    # returns False (step1/step3 on error) or raises aborts the run; any other
    # step returning None counts as success. Grouped by phase for readability.
    steps = [
        # hierarchy cleanup
        ( 1, "move vehicle to root",    step1_move_vehicle),
        ( 2, "delete wrapper xforms",   step2_delete_wrappers),
        ( 3, "set defaultPrim",         step3_set_default_prim),
        ( 4, "rename links",            step4_rename_links),
        ( 5, "clear instanceable",      step5_clear_instanceable),
        # joints & articulation
        ( 6, "reparent joints",         step6_reparent_joints),
        ( 7, "set joint bodies",        step7_set_joint_bodies),
        ( 8, "add prismatic joints",    step8_add_prismatic_joints),
        # frame realignment
        ( 9, "realign body frames",     step9_realign_body_frames),
        (10, "realign joint frames",    step10_realign_joint_frames),
        # inertia / collision / drives
        (11, "set inertial properties", step11_set_inertial_properties),
        (12, "add colliders",           step12_add_colliders),
        (13, "apply joint drives",      step13_apply_joint_drives),
        # debug: draw authored joint frames for visual checking (comment out for final asset)
        # (14, "draw joint frames (debug)", step14_draw_joint_frames),
        # (15, "export flattened USD",    step15_export),   # uncomment to export
    ]

    results = []            # list of (num, label, ok)
    for num, label, func in steps:
        print("", flush=True)                   # blank line separates each step
        try:
            ok = func(stage) is not False        # None/True → ok; explicit False → fail
        except Exception:
            carb.log_error(f"[POSTPROC] Step {num} ({label}) raised an exception:")
            traceback.print_exc()
            ok = False
        results.append((num, label, ok))
        if not ok:
            break                                # abort on the first failure

    # ---- run summary: which steps ran, and whether each succeeded ----
    print("", flush=True)
    print("=" * 60, flush=True)
    print("[POSTPROC] RUN SUMMARY", flush=True)
    ok_by_num = {num: ok for num, _, ok in results}
    for num, label, _ in steps:
        if num not in ok_by_num:
            mark = "[ -- ]"                       # never reached (aborted earlier)
        else:
            mark = "[ OK ]" if ok_by_num[num] else "[FAIL]"
        print(f"  {mark}  step {num:>2}  {label}", flush=True)
    n_ok = sum(1 for _, _, ok in results if ok)
    n_total = len(steps)
    if n_ok == n_total:
        print(f"[POSTPROC] All {n_total} steps succeeded.", flush=True)
    else:
        failed = next((num for num, _, ok in results if not ok), None)
        print(f"[POSTPROC] {n_ok}/{n_total} succeeded - aborted at step {failed}.", flush=True)
    print("=" * 60, flush=True)


run()
