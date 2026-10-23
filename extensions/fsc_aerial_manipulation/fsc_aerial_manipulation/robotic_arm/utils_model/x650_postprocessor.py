"""
| File: x650_postprocessor.py
| Author: Shiqi Gao (shiqi.gao907@gmail.com)
| License: BSD-3-Clause
| Copyright (c) 2026, Shiqi Gao. All rights reserved.

x650_postprocessor.py

Post-processor for the **plain X650 quadrotor** (no robotic arm). Sibling of
``postprocessor.py`` (which handles the gripper_bat arm+drone). Paste into the
Isaac Sim Script Editor (Window > Script Editor) with the imported X650 stage
open, edit the CONFIG block, and run. It also runs head-less via
``x650_postprocess_headless.py`` for CI-style checks.

What it does (the X650 mesh already ships mass + inertia on its referenced
prototype meshes, so — unlike the arm postprocessor — this script authors NO
mass/inertia; it reorganises, renames, and upgrades the colliders):

   1. Move the vehicle subtree from the nested import path to a single stage root
   2. Delete the now-empty import wrappers + the viewport/env junk prims
   3. Set the vehicle as defaultPrim
   4. Rename the propeller links to rotor0..rotor3, assigned BY PHYSICAL CORNER
      (read from each rotor joint) to match the PX4 `none_iris` mixer  [CORNER_TO_ROTOR]
   5. Rename the rotor joints so joint_i drives rotor_i (same corner classification)
   6. Clear instanceable on the whole subtree (so colliders sync to Fabric)
   7. Swap each rotor joint to body0=body, body1=rotor (articulation base = body) [JOINT_OVERRIDES]
   8. Normalise each joint's local frames to identity + set axis/limits     [JOINT_OVERRIDES]
   9. Re-author every mesh collider to convexDecomposition                 [COLLISION_APPROXIMATION]
  10. Export the flattened asset — compact .usd (sim) + readable .usda    [OUTPUT_USD/USDA]

── Rotor-index convention (verified against iris.usd) ───────────────────────
The PX4 `none_iris` airframe has a FIXED mixer: motor channel i must sit at a
specific corner and spin a specific way, else roll/pitch/yaw invert. Body frame
is FLU (+X front, +Y left, +Z up):

    idx    corner              spin   rot_dir (QuadraticThrustCurve default)
    rotor0 front-right (+X,-Y) CCW    -1
    rotor1 rear-left   (-X,+Y) CCW    -1
    rotor2 front-left  (+X,+Y) CW     +1
    rotor3 rear-right  (-X,-Y) CW     +1

The X650's props are CAD-labelled by blade shape (prop_clock / prop_counter_clock),
which happens to be the OPPOSITE hand to the iris slot at each corner. That is a
purely cosmetic mesh mismatch: flight physics uses the thrust curve's rot_dir
(above), and handle_propeller_visual() spins the joint by rot_dir too — so the
motors behave correctly; only the moulded blade angle looks reversed.

rotorN is assigned strictly by CORNER (step4/step5 read each joint's body-side
attach point and classify the quadrant via CORNER_TO_ROTOR), NOT by CAD prop
name. So re-orienting the model in Onshape needs NO edits here: the corners are
re-derived from geometry every run, and the PX4 mixer stays satisfied as long as
the airframe remains a symmetric quad_x.
"""

import traceback
import omni.usd
import omni.kit.commands
import carb
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

# ============================================================================
# PART 1 · PARAMETER SETTINGS   — edit these for each X650 import
# ============================================================================

# ── Prim paths ─────────────────────────────────────────────────────────────
# Full prim path of the innermost vehicle xform right after import (the one
# carrying PhysicsArticulationRootAPI). Expand the Stage panel to confirm.
VEHICLE_ROOT_SRC = "/World/X650/X650"

# Desired single root path in the cleaned USD (also the defaultPrim + the name
# passed to the spawner as articulation_path="/x650").
VEHICLE_ROOT_DST = "/x650"

# Wrapper xforms above VEHICLE_ROOT_SRC to delete once the vehicle is moved out.
# List innermost-first so each is empty by the time it is deleted (step 2 skips
# any that are still non-empty).
WRAPPER_XFORMS_TO_DELETE = [
    "/World/X650",   # inner assembly wrapper (empty after the vehicle move)
    "/World",        # outer wrapper (empty once /World/X650 is gone)
]

# Viewport / environment junk prims dropped in by the GUI import. Force-deleted
# (children and all) so the exported asset is only the vehicle. Harmless if
# absent (head-less imports usually have none of these).
JUNK_PRIMS_TO_DELETE = [
    "/Environment",
    "/Render",
    "/OmniverseKit_Persp",
    "/OmniverseKit_Front",
    "/OmniverseKit_Top",
    "/OmniverseKit_Right",
]

# Output paths for the flattened, cleaned asset. step10 writes BOTH:
#   OUTPUT_USD  — binary crate (compact, ~50 MB); this is what the sim loads.
#   OUTPUT_USDA — ASCII (human-readable / git-diffable, but ~400 MB).
# USD picks the format from the extension. If you don't need the big readable
# copy, set OUTPUT_USDA = None (step10 then skips it).
_ASSETS_DIR = ("/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/"
               "fsc_aerial_manipulation/rotorcraft/assets")
OUTPUT_USD = f"{_ASSETS_DIR}/x650.usd"
OUTPUT_USDA = f"{_ASSETS_DIR}/x650.usda"

# ── Rotor-index convention (POSITION-BASED — rotation-proof) ─────────────────
# rotorN is assigned by the CORNER each propeller physically occupies in the body
# frame (FLU: +X front, +Y left, +Z up), NOT by CAD prop name. This makes the
# pipeline immune to re-orienting the model in Onshape: step4/step5 read each
# rotor joint's body-side attach point (its moment arm in the body frame),
# classify the quadrant by (sign x, sign y), and map it to the PX4 quad_x slot
# below. As long as the airframe stays a symmetric quad_x, no table ever needs
# editing after a CAD rotation.
#
#   (sign X, sign Y)  corner               rotor   PX4 motor  spin  rot_dir
#   (+, -)            front-right (+X,-Y)  rotor0     1        CCW    -1
#   (-, +)            rear-left   (-X,+Y)  rotor1     2        CCW    -1
#   (+, +)            front-left  (+X,+Y)  rotor2     3        CW     +1
#   (-, -)            rear-right  (-X,-Y)  rotor3     4        CW     +1
#
# rot_dir is the QuadraticThrustCurve default [-1,-1,1,1] in the SPAWNER config;
# it lines up with the corners above, so fixing the corners fixes flight. This is
# exactly the numbering in the reference top-view (PX4 motor 1 front-right, 2
# rear-left, 3 front-left, 4 rear-right).
CORNER_TO_ROTOR: dict[tuple, int] = {
    (+1, -1): 0,   # front-right (+X,-Y)
    (-1, +1): 1,   # rear-left   (-X,+Y)
    (+1, +1): 2,   # front-left  (+X,+Y)
    (-1, -1): 3,   # rear-right  (-X,-Y)
}

# ── Joint overrides (single source of truth for every joint) ───────────────
# Keyed by the FINAL joint name (joint0..3, after step5's by-corner rename). Fields:
#   "type"  : "revolute" (rotor spin DOF).
#   "body0" : parent link, "body1" : child link  (Isaac convention parent->child).
#             The import authors these reversed (body0=prop, body1=body); step7
#             swaps body0<->body1 (+localPos/localRot) so the articulation base
#             is 'body' and each rotor hangs off it, matching iris.
#   "axis"  : DOF axis after step8 identity-normalises the joint frames (Z spin).
#   "limit" : None -> free continuous spin (rotors are never position-limited).
#   "drive" : None  -> no DriveAPI; the visual spin is applied kinematically via
#             set_dof_velocity (see MultirotorMod.handle_propeller_visual), and a
#             stiffness/damping drive would fight it. Matches iris (no rotor drive).
JOINT_OVERRIDES: dict[str, dict] = {
    "joint0": {"body0": "body", "body1": "rotor0", "type": "revolute",
               "axis": "Z", "limit": None, "drive": None},
    "joint1": {"body0": "body", "body1": "rotor1", "type": "revolute",
               "axis": "Z", "limit": None, "drive": None},
    "joint2": {"body0": "body", "body1": "rotor2", "type": "revolute",
               "axis": "Z", "limit": None, "drive": None},
    "joint3": {"body0": "body", "body1": "rotor3", "type": "revolute",
               "axis": "Z", "limit": None, "drive": None},
}

# A joint's DOF must align with a world axis within this tolerance (dot >= value)
# for step8 to snap it; a more-oblique joint is warned about and skipped.
JOINT_AXIS_ALIGN_TOL = 0.99   # ~= 8.1deg max off-axis

# ── Mesh colliders ─────────────────────────────────────────────────────────
# The X650 import ships each mesh with a "convexHull" collider. A single convex
# hull fills the concave gaps of the frame (between arms/legs), giving a blobby,
# too-large collider. "convexDecomposition" approximates each mesh as a SET of
# convex hulls, so concave shapes collide accurately. step9 re-authors every
# vehicle mesh collider to this approximation (overwriting the import's convexHull).
COLLISION_APPROXIMATION = "convexDecomposition"   # or "convexHull", "boundingCube", "none"
# Mesh path substrings to skip (leave their collider untouched). Empty = convert
# EVERY mesh, so no convexHull remains anywhere in the asset. The thin props are
# ~convex, so decomposing them yields ~1 hull (cheap) — kept in for completeness;
# add "rotor" here if the spinning-prop colliders ever cost too much to cook.
COLLIDER_EXCLUDE: list[str] = []
# Convex-decomposition tuning (higher = more hulls / finer detail = more accurate,
# slower to cook and simulate).
CONVEX_DECOMP = {
    "maxConvexHulls":  32,
    "hullVertexLimit": 64,
    "voxelResolution": 500_000,
}

# ============================================================================
# PART 2 · UTILITY & STEP FUNCTIONS   — no edits needed
# ============================================================================

_IDENTITY_QUATD = Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
_AXIS_VEC = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


def _abs(relative_path: str) -> str:
    return f"{VEHICLE_ROOT_DST}/{relative_path}"


def _move(src: str, dst: str) -> bool:
    ok, _ = omni.kit.commands.execute(
        "MovePrimCommand", path_from=src, path_to=dst, keep_world_transform=False,
    )
    if not ok:
        carb.log_error(f"[X650] MovePrimCommand failed: {src} -> {dst}")
    return ok


def _joint_body_name(rel) -> str:
    """Last path component of a joint body0/body1 relationship (None if empty)."""
    t = rel.GetTargets()
    return t[0].name if t else None


def _joint_rotor_index(jt):
    """Classify a rotor joint by the CORNER of its body-side attach point.

    Returns (rotor_idx, prop_side_name):
      rotor_idx      — PX4 quad_x slot from CORNER_TO_ROTOR, or None if this is
                       not a body<->prop rotor joint (or it lands off-quadrant).
      prop_side_name — name of the non-'body' side (the propeller link), whatever
                       it is called at call time (prop_* before step4, rotorN after).

    The body-side localPos is the moment arm expressed in the body frame — exactly
    what the PX4 mixer cares about — so classifying by its (sign x, sign y) is
    robust to any whole-vehicle CAD rotation. Handles either body ordering: the
    raw import authors body0=prop/body1=body; step7 later swaps to
    body0=body/body1=prop.
    """
    r0, r1 = jt.GetBody0Rel(), jt.GetBody1Rel()
    b0, b1 = _joint_body_name(r0), _joint_body_name(r1)
    if b1 == "body":
        body_pos, prop = jt.GetLocalPos1Attr().Get(), b0
    elif b0 == "body":
        body_pos, prop = jt.GetLocalPos0Attr().Get(), b1
    else:
        return None, None
    if body_pos is None:
        return None, None
    sx = 1 if body_pos[0] >= 0.0 else -1
    sy = 1 if body_pos[1] >= 0.0 else -1
    return CORNER_TO_ROTOR.get((sx, sy)), prop


def _swap_attr(a, b):
    """Swap the authored values of two attributes (skips a side that is unset)."""
    va, vb = a.Get(), b.Get()
    if vb is not None:
        a.Set(vb)
    if va is not None:
        b.Set(va)


def _to_quatd(q) -> Gf.Quatd:
    """Gf.Quatf/Quatd/tuple(w,x,y,z) -> Gf.Quatd."""
    if isinstance(q, (tuple, list)):
        return Gf.Quatd(float(q[0]), Gf.Vec3d(float(q[1]), float(q[2]), float(q[3])))
    im = q.GetImaginary()
    return Gf.Quatd(q.GetReal(), Gf.Vec3d(im[0], im[1], im[2]))


def _rotate_vec(qR: Gf.Quatd, v) -> Gf.Vec3d:
    """Rotate a vector by qR:  v' = qR . (0,v) . qR^-1."""
    qv = Gf.Quatd(0.0, Gf.Vec3d(v[0], v[1], v[2]))
    return (qR * qv * qR.GetInverse()).GetImaginary()


def _set_or_clear(prim, attr_name, value):
    """Author a float attr, or Clear() it when value is None (that side stays open)."""
    a = prim.GetAttribute(attr_name)
    if not a:
        return
    if value is None:
        a.Clear()
    else:
        a.Set(float(value))


# ── pipeline steps (executed in order by run()) ───────────────────────────────

def step1_move_vehicle(stage: Usd.Stage) -> bool:
    src_prim = stage.GetPrimAtPath(VEHICLE_ROOT_SRC)
    if not src_prim or not src_prim.IsValid():
        carb.log_error(f"[X650] Source prim not found: {VEHICLE_ROOT_SRC}")
        carb.log_error("[X650] Check VEHICLE_ROOT_SRC against the Stage panel.")
        return False
    if VEHICLE_ROOT_SRC == VEHICLE_ROOT_DST:
        print("[X650] Step 1 skipped - src == dst.", flush=True)
        return True
    print(f"[X650] Step 1: {VEHICLE_ROOT_SRC} -> {VEHICLE_ROOT_DST}", flush=True)
    return _move(VEHICLE_ROOT_SRC, VEHICLE_ROOT_DST)


def step2_delete_wrappers(stage: Usd.Stage):
    print("[X650] Step 2: removing wrapper + junk prims...", flush=True)
    # Empty wrapper xforms above the old vehicle path (skip if still non-empty).
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
    # Viewport / environment junk - force-delete children and all.
    for path in JUNK_PRIMS_TO_DELETE:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            stage.RemovePrim(Sdf.Path(path))
            print(f"  deleted junk: {path}", flush=True)


def step3_set_default_prim(stage: Usd.Stage) -> bool:
    print(f"[X650] Step 3: defaultPrim -> {VEHICLE_ROOT_DST}", flush=True)
    vehicle = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not vehicle or not vehicle.IsValid():
        carb.log_error(f"[X650] Vehicle prim missing at {VEHICLE_ROOT_DST}")
        return False
    stage.SetDefaultPrim(vehicle)
    return True


def step4_rename_links(stage: Usd.Stage):
    """Rename each propeller link to rotor0..3 BY CORNER (see CORNER_TO_ROTOR).

    Walks every rotor joint, classifies the corner from its body-side attach point,
    and renames that joint's propeller link to the matching rotorN. The prop_* and
    rotorN namespaces are disjoint, so a single pass cannot collide. Renaming a
    link auto-updates the joint relationship that targets it (MovePrimCommand
    fixes connections), so step7 later sees the new rotorN names.
    """
    print("[X650] Step 4: renaming propeller links -> rotor0..3 (by corner)...", flush=True)
    root = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not root or not root.IsValid():
        carb.log_error(f"[X650] Vehicle root missing: {VEHICLE_ROOT_DST}")
        return
    assigned: dict[int, str] = {}
    for jp in [p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)]:
        idx, prop = _joint_rotor_index(UsdPhysics.Joint(jp))
        if idx is None:
            carb.log_warn(f"  {jp.GetName()}: could not classify corner - skipped.")
            continue
        if idx in assigned:
            carb.log_error(f"  corner clash: rotor{idx} from both '{assigned[idx]}' and "
                           f"'{prop}' - is the airframe a symmetric quad_x?")
            continue
        assigned[idx] = prop
        src, dst = _abs(prop), _abs(f"rotor{idx}")
        if src == dst:
            print(f"  {prop}: already rotor{idx}", flush=True)
            continue
        ok = _move(src, dst)
        print(f"  {'OK' if ok else 'FAIL'}: {prop} -> rotor{idx} "
              f"(corner from {jp.GetName()})", flush=True)
    if len(assigned) != 4:
        carb.log_warn(f"  assigned {len(assigned)}/4 rotors - check the airframe geometry.")


def step5_rename_joints(stage: Usd.Stage):
    """Rename each rotor joint to joint0..3 so joint_i drives rotor_i, BY CORNER.

    Same corner classification as step4 (by now the prop side is already 'rotorN',
    but the body-side localPos — hence the corner — is unchanged). Applied in two
    passes through unique temp names so a cyclic permutation (e.g. joint0<->joint2)
    never collides with an existing prim.
    """
    print("[X650] Step 5: renaming rotor joints -> joint0..3 (by corner)...", flush=True)
    root = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not root or not root.IsValid():
        carb.log_error(f"[X650] Vehicle root missing: {VEHICLE_ROOT_DST}")
        return
    # plan: current joint name -> desired jointN (derived from geometry)
    plan: dict[str, str] = {}
    for jp in [p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)]:
        idx, _ = _joint_rotor_index(UsdPhysics.Joint(jp))
        if idx is None:
            carb.log_warn(f"  {jp.GetName()}: could not classify corner - skipped.")
            continue
        plan[jp.GetName()] = f"joint{idx}"
    if len(set(plan.values())) != len(plan):
        carb.log_error("  duplicate joint targets - aborting joint rename "
                       "(check the airframe geometry).")
        return
    # pass 1: old -> old__tmp
    for old_name in plan:
        _move(_abs(old_name), _abs(f"{old_name}__tmp"))
    # pass 2: old__tmp -> new
    for old_name, new_name in plan.items():
        ok = _move(_abs(f"{old_name}__tmp"), _abs(new_name))
        print(f"  {'OK' if ok else 'FAIL'}: {old_name} -> {new_name}", flush=True)


def step6_clear_instanceable(stage: Usd.Stage):
    """Recursively set instanceable=False on the entire vehicle subtree.

    The GUI/OnShape import references each link's mesh as an instanceable
    prototype, which prevents PhysX from syncing the cooked collider to Fabric
    ("Cannot sync Solid Collision Mesh to Fabric"). Walk top-down on REAL prims
    (clear a prim's flag BEFORE reading its children, so newly-exposed children
    are reachable).
    """
    print("[X650] Step 6: clearing instanceable on whole subtree...", flush=True)
    vehicle = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not vehicle or not vehicle.IsValid():
        carb.log_error(f"[X650] Vehicle prim missing: {VEHICLE_ROOT_DST}")
        return
    count = 0
    stack = [vehicle]
    while stack:
        prim = stack.pop()
        if prim.IsInstanceable():
            prim.SetInstanceable(False)   # do this BEFORE reading children
            count += 1
        stack.extend(prim.GetChildren())
    print(f"  {count} prim(s) cleared.", flush=True)


def step7_set_joint_bodies(stage: Usd.Stage):
    """Bring each joint's body0/body1 to its JOINT_OVERRIDES assignment.

    Reproduces the Physics-UI "swap bodies" button: when a joint is the exact
    REVERSE of its target it swaps body0<->body1 AND localPos0<->localPos1 AND
    localRot0<->localRot1, so the joint stays physically in place. Per-joint
    idempotent: a joint already matching its target is left untouched.
    """
    if not JOINT_OVERRIDES:
        print("[X650] Step 7 skipped - no joints configured.", flush=True)
        return
    print("[X650] Step 7: setting joint body0/body1 (base = body)...", flush=True)
    root = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not root or not root.IsValid():
        carb.log_error(f"[X650] Vehicle root missing: {VEHICLE_ROOT_DST}")
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


def step8_realign_joint_frames(stage: Usd.Stage):
    """Re-express every joint's local frames (localRot0/localRot1) as IDENTITY so
    the gizmo axes align with the world-aligned bodies, then set each joint's
    declared axis + limits from JOINT_OVERRIDES. The X650 import already ships
    identity joint rotations, so this is mostly a verify-and-set pass, but it
    guards against any residual tilt and makes axis/limits explicit.
    """
    print("[X650] Step 8: normalising joint frames + axis + limits...", flush=True)
    root = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not root or not root.IsValid():
        carb.log_error(f"[X650] Vehicle root missing: {VEHICLE_ROOT_DST}")
        return
    joints_by_name = {p.GetName(): p for p in Usd.PrimRange(root)
                      if p.IsA(UsdPhysics.Joint)}
    ident = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    done = skipped = 0
    for jname, spec in JOINT_OVERRIDES.items():
        prim = joints_by_name.get(jname)
        if prim is None:
            carb.log_warn(f"  {jname}: joint not found - skipped.")
            skipped += 1
            continue
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
        d = _rotate_vec(Q0, _AXIS_VEC[probe])            # current DOF dir in world
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
        jt.GetLocalRot0Attr().Set(ident)
        jt.GetLocalRot1Attr().Set(ident)
        if cur_axis_attr:
            cur_axis_attr.Set(declared)
        lim = spec.get("limit")
        lo, hi = (None, None) if lim is None else (lim[0], lim[1])
        _set_or_clear(prim, "physics:lowerLimit", lo)
        _set_or_clear(prim, "physics:upperLimit", hi)
        done += 1
        print(f"  {jname}: frames->identity, axis {cur_letter}->{declared}, limit={lim}", flush=True)
    print(f"  {done} joint(s) configured, {skipped} skipped.", flush=True)


def step9_set_colliders(stage: Usd.Stage):
    """Re-author every vehicle mesh collider to COLLISION_APPROXIMATION.

    The X650 import ships "convexHull" colliders (which over-fill the concave
    frame). Walks the whole subtree and sets each mesh's approximation to
    convexDecomposition (default), attaching the PhysxConvexDecompositionCollisionAPI
    tuning. step6 already cleared instanceable, so a plain PrimRange reaches every
    real Mesh prim and can author on it. Apply() is idempotent, so this both
    upgrades the existing colliders and adds one to any mesh that lacked it.
    """
    print(f"[X650] Step 9: setting mesh colliders ({COLLISION_APPROXIMATION})...", flush=True)
    vehicle = stage.GetPrimAtPath(VEHICLE_ROOT_DST)
    if not vehicle or not vehicle.IsValid():
        carb.log_error(f"[X650] Vehicle prim missing: {VEHICLE_ROOT_DST}")
        return
    count = skipped = 0
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
    print(f"  {count} collider(s) set to {COLLISION_APPROXIMATION}, {skipped} skipped.", flush=True)


def step10_export(stage: Usd.Stage):
    # Export() picks the crate/ASCII format from the extension: ".usd" -> binary
    # crate (compact), ".usda" -> ASCII. Flatten once, write both targets.
    print("[X650] Step 10: exporting flattened asset...", flush=True)
    flat_layer = stage.Flatten()
    for target in (OUTPUT_USD, OUTPUT_USDA):
        if not target:
            continue
        flat_layer.Export(target)
        print(f"  saved: {target}", flush=True)


# ============================================================================
# PART 3 · MAIN
# ============================================================================

def run(export: bool = False):
    """Run the pipeline on the current stage.

    Args:
        export: when True, also flatten + export to OUTPUT_USD/OUTPUT_USDA (step 10). Left
            False for interactive Script-Editor runs so you can eyeball the
            result first; the head-less harness passes export=True.
    """
    stage = omni.usd.get_context().get_stage()
    print("=" * 60, flush=True)
    print("[X650] Starting X650 quadrotor USD post-processor...", flush=True)
    print("=" * 60, flush=True)

    steps = [
        (1, "move vehicle to root",     step1_move_vehicle),
        (2, "delete wrappers + junk",   step2_delete_wrappers),
        (3, "set defaultPrim",          step3_set_default_prim),
        (4, "rename links",             step4_rename_links),
        (5, "rename joints",            step5_rename_joints),
        (6, "clear instanceable",       step6_clear_instanceable),
        (7, "set joint bodies",         step7_set_joint_bodies),
        (8, "realign joint frames",     step8_realign_joint_frames),
        (9, "set mesh colliders",       step9_set_colliders),
    ]
    if export:
        steps.append((10, "export flattened USD", step10_export))

    results = []
    for num, label, func in steps:
        print("", flush=True)
        try:
            ok = func(stage) is not False
        except Exception:
            carb.log_error(f"[X650] Step {num} ({label}) raised an exception:")
            traceback.print_exc()
            ok = False
        results.append((num, label, ok))
        if not ok:
            break

    print("", flush=True)
    print("=" * 60, flush=True)
    print("[X650] RUN SUMMARY", flush=True)
    ok_by_num = {num: ok for num, _, ok in results}
    for num, label, _ in steps:
        if num not in ok_by_num:
            mark = "[ -- ]"
        else:
            mark = "[ OK ]" if ok_by_num[num] else "[FAIL]"
        print(f"  {mark}  step {num:>2}  {label}", flush=True)
    n_ok = sum(1 for _, _, ok in results if ok)
    n_total = len(steps)
    if n_ok == n_total:
        print(f"[X650] All {n_total} steps succeeded.", flush=True)
    else:
        failed = next((num for num, _, ok in results if not ok), None)
        print(f"[X650] {n_ok}/{n_total} succeeded - aborted at step {failed}.", flush=True)
    print("=" * 60, flush=True)
    return n_ok == n_total


# Interactive Script-Editor entrypoint: reorganise the current stage in place
# (no export, so you can inspect first — then call run(export=True) to write the
# asset, or use x650_postprocess_headless.py).
#
# The head-less harness sets X650_POSTPROC_AUTORUN=0 so it can import these
# functions and drive run(export=True) itself; a Script-Editor paste leaves the
# var unset and auto-runs in place.
import os as _os
if _os.environ.get("X650_POSTPROC_AUTORUN", "1") == "1":
    run()
