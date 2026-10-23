#!/usr/bin/env python
"""
make_x_forward_asset.py — one-shot asset transform: AM_realign.usda -> AM_xfwd.usda

Re-authors the aerial-manipulator asset so its MECHANICAL FRONT (the arm side,
body +y in AM_realign) lies on BODY +X, matching the T650/X650 bare-frame
convention and PX4's x-forward assumption. The original file is never touched:
the script copies it first and edits only the copy.

Run under Isaac's python (pxr needed; a SimulationApp is started headless):

    ~/isaacsim/python_r_fsc.sh \
        extensions/.../robotic_arm/utils_model/make_x_forward_asset.py

WHY THIS EXISTS (2026-08-10). Two problems traced to the asset's frame on the
first AM-T650 DIRECT-stack flight:
  1. The mechanical front on +y forced the path_yaw_deg=90 workaround in every
     whole-body demo, and under the fsc_autopilot stack the vehicle would fly
     arm-sideways.
  2. AM_realign's rotor CHANNEL indexing is mirrored vs x650_new.usd
     (ch0 front-LEFT instead of front-RIGHT, etc.), which negates the ROLL row
     of any standard Quad-X allocation — PX4's own SAFETY-mode mixer included.
     The vehicle flipped on lift-off (omega = [427,467,466,427]: PX4 pushed
     what it believed was the right-side pair down; on this asset that pair is
     the LEFT side).

THE TRANSFORM. Rotate the vehicle's CONTENT by Rz(-90 deg) — (x,y,z) ->
(y,-x,z), mapping +y onto +x — while keeping the /body PRIM's frame fixed.
AM_realign is postprocessed such that every link prim and every joint local
frame is authored at IDENTITY orientation, which makes the edit exact and
small:

  * every top-level link EXCEPT /body: translate <- R@t, orient <- qR
    (frame rotates WITH the content, so everything inside the link — meshes,
    CoM, inertia, child-side joint anchors — stays valid untouched);
  * /body: frame untouched; its xformable children (the body mesh) get
    R composed into their local ops; CoM <- R@com; diagonal inertia
    (Ixx,Iyy,Izz) -> (Iyy,Ixx,Izz) (exact for a 90-deg z-rotation with
    identity principal axes);
  * joints anchored on /body (joint0..3, manip_joint1): localPos0 <- R@p0,
    localRot0 <- qR (times the old identity). All other joints connect two
    rotated links on both sides and need no change.

RESULT (verified by this script's own check pass):
  * arm mount (manip_joint1 anchor on body): (0.0005, 0.002, ..) ->
    (0.002, -0.0005, ..) — the arm chain now extends along +x;
  * rotor channels become r0 front-right, r1 rear-left, r2 REAR-RIGHT,
    r3 FRONT-LEFT. That is x650_new.usd's convention with CHANNELS 2 AND 3
    SWAPPED (the CW pair sits on swapped prims). Consumers must remap:
    ROTOR_PATHS = [rotor0, rotor1, rotor3, rotor2] presents the exact
    PX4 Quad-X order (FR, RL, FL, RR) with rot_dir [-1,-1,1,1].

DOWNSTREAM (unchanged consumers keep using AM_realign.usda): the whole-body
demos (01/02/03), make_params, RotorMixer, the planner's z-x-z recovery and
the measured EE_SAG all encode the OLD frame and are NOT migrated by this
script. Only 04_px4_direct_t650_aerial_manipulator_hold.py uses AM_xfwd.usda, with a one-line
frame adapter for its gravity comp (R0_model = R0_actual @ Rz(-90)).
"""

import os
import shutil
import sys

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf  # noqa: E402

ASSETS = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                      "..", "..", "rotorcraft", "assets"))
SRC = os.path.join(ASSETS, "AM_realign.usda")
DST = os.path.join(ASSETS, "AM_xfwd.usda")
ROOT = "/gripper_bat"
BODY = ROOT + "/body"

# Rz(-90 deg): (x, y, z) -> (y, -x, z). EXACT integer arithmetic on vectors.
def rot_v(v):
    return type(v)(v[1], -v[0], v[2])

# Quaternion for Rz(-90): w = cos(45), z = -sin(45).
_S = 0.7071067811865476
Q_R = Gf.Quatd(_S, Gf.Vec3d(0.0, 0.0, -_S))

def rot_q(q):
    """Left-compose Rz(-90) with an existing orientation, preserving type."""
    qd = Gf.Quatd(q.GetReal(), Gf.Vec3d(*q.GetImaginary()))
    out = Q_R * qd
    if isinstance(q, Gf.Quatf):
        return Gf.Quatf(out.GetReal(), Gf.Vec3f(*out.GetImaginary()))
    if isinstance(q, Gf.Quath):
        return Gf.Quath(out.GetReal(), Gf.Vec3h(*out.GetImaginary()))
    return out

def get_ops(prim):
    x = UsdGeom.Xformable(prim)
    return {op.GetOpName(): op for op in x.GetOrderedXformOps()}

def is_identity_q(q, tol=1e-6):
    return abs(q.GetReal() - 1.0) < tol and Gf.Vec3d(*q.GetImaginary()).GetLength() < tol


def main():
    if not os.path.isfile(SRC):
        print(f"ERROR: source asset not found: {SRC}")
        sys.exit(1)
    if os.path.isfile(DST) and "--force" not in sys.argv:
        print(f"ERROR: {DST} already exists. Pass --force to overwrite it.")
        sys.exit(1)

    print(f"[xfwd] copying {os.path.basename(SRC)} -> {os.path.basename(DST)} "
          f"({os.path.getsize(SRC)/1e6:.0f} MB)...", flush=True)
    shutil.copyfile(SRC, DST)

    stage = Usd.Stage.Open(DST)
    root = stage.GetPrimAtPath(ROOT)
    body = stage.GetPrimAtPath(BODY)
    assert root and body, "root/body prim missing — wrong asset?"

    # ── sanity: the transform below is only valid on the postprocessed layout ──
    root_ops = get_ops(root)
    for nm, op in root_ops.items():
        if "orient" in nm or "rotate" in nm:
            val = op.Get()
            if val is not None and not is_identity_q(val):
                print(f"ABORT: root prim carries a rotation ({nm}={val}) — "
                      f"transform assumptions do not hold.")
                sys.exit(1)

    links = [p for p in root.GetChildren() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for p in links:
        ops = get_ops(p)
        q = ops.get("xformOp:orient")
        if q is not None and q.Get() is not None and not is_identity_q(q.Get()):
            print(f"ABORT: link {p.GetPath()} orient is not identity: {q.Get()}")
            sys.exit(1)
    joints = [p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)]
    for j in joints:
        jj = UsdPhysics.Joint(j)
        for a in (jj.GetLocalRot0Attr(), jj.GetLocalRot1Attr()):
            v = a.Get()
            if v is not None and not is_identity_q(v):
                print(f"ABORT: joint {j.GetPath()} has non-identity local rot {v}")
                sys.exit(1)
    print(f"[xfwd] sanity OK: {len(links)} links, {len(joints)} joints, "
          f"all frames identity", flush=True)

    # ── 1. rotate every top-level link EXCEPT /body (frame moves with content) ──
    n = 0
    for p in links:
        if p.GetPath() == body.GetPath():
            continue
        ops = get_ops(p)
        t_op = ops.get("xformOp:translate")
        if t_op is not None and t_op.Get() is not None:
            t_op.Set(rot_v(t_op.Get()))
        q_op = ops.get("xformOp:orient")
        if q_op is not None and q_op.Get() is not None:
            q_op.Set(rot_q(q_op.Get()))
        # authored initial velocities (world/body ambiguity moot: they are zero,
        # but rotate for correctness if present)
        rb = UsdPhysics.RigidBodyAPI(p)
        for attr in (rb.GetVelocityAttr(), rb.GetAngularVelocityAttr()):
            v = attr.Get() if attr else None
            if v is not None and Gf.Vec3d(*v).GetLength() > 0:
                attr.Set(rot_v(v))
        n += 1
    print(f"[xfwd] rotated {n} link prims (frame + content together)", flush=True)

    # ── 2. /body: frame fixed, content rotated ────────────────────────────────
    ops = get_ops(body)
    t_op = ops.get("xformOp:translate")
    if t_op is not None and t_op.Get() is not None:
        t_op.Set(rot_v(t_op.Get()))     # z-only in practice; exact anyway
    # xformable children of /body (the body mesh; Looks is a Scope, skipped)
    m = 0
    for c in body.GetChildren():
        if not c.IsA(UsdGeom.Xformable):
            continue
        if c.IsA(UsdPhysics.Joint):
            continue                     # none exist under /body, but be safe
        c_ops = get_ops(c)
        ct = c_ops.get("xformOp:translate")
        if ct is not None and ct.Get() is not None:
            ct.Set(rot_v(ct.Get()))
        cq = c_ops.get("xformOp:orient")
        if cq is not None and cq.Get() is not None:
            cq.Set(rot_q(cq.Get()))
        elif "xformOp:orient" not in c_ops:
            # content with no orient op would stay unrotated — add one
            x = UsdGeom.Xformable(c)
            op = x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
            op.Set(Gf.Quatd(Q_R))
        m += 1
    # physics: CoM rotates; diagonal inertia swaps Ixx<->Iyy (exact for 90 deg
    # about z with identity principal axes)
    mass_api = UsdPhysics.MassAPI(body)
    com_a = mass_api.GetCenterOfMassAttr()
    if com_a and com_a.Get() is not None:
        com_a.Set(rot_v(com_a.Get()))
    in_a = mass_api.GetDiagonalInertiaAttr()
    if in_a and in_a.Get() is not None:
        I = in_a.Get()
        in_a.Set(type(I)(I[1], I[0], I[2]))
    print(f"[xfwd] /body: content rotated ({m} xformable children), CoM rotated, "
          f"inertia Ixx<->Iyy", flush=True)

    # ── 3. joints anchored on /body: rotate the body-side frame ──────────────
    k = 0
    for j in joints:
        jj = UsdPhysics.Joint(j)
        for rel, pos_a, rot_a in (
            (jj.GetBody0Rel(), jj.GetLocalPos0Attr(), jj.GetLocalRot0Attr()),
            (jj.GetBody1Rel(), jj.GetLocalPos1Attr(), jj.GetLocalRot1Attr()),
        ):
            targets = [str(t) for t in rel.GetTargets()]
            if targets != [BODY]:
                continue
            if pos_a.Get() is not None:
                pos_a.Set(rot_v(pos_a.Get()))
            if rot_a.Get() is not None:
                rot_a.Set(rot_q(rot_a.Get()))
            k += 1
            print(f"[xfwd]   body-side joint frame rotated: {j.GetPath()}", flush=True)
    print(f"[xfwd] {k} body-anchored joint sides rotated", flush=True)

    print("[xfwd] saving (255 MB, takes a moment)...", flush=True)
    stage.GetRootLayer().Save()
    del stage

    # ── verification pass on the saved file ──────────────────────────────────
    print("\n[xfwd] ===== VERIFY (reopened from disk) =====", flush=True)
    vs = Usd.Stage.Open(DST)
    vroot = vs.GetPrimAtPath(ROOT)
    ok = True

    def expect(name, got, want, tol=1e-6):
        nonlocal ok
        good = all(abs(g - w) < tol for g, w in zip(got, want))
        ok = ok and good
        print(f"  {'OK ' if good else 'FAIL'} {name}: {tuple(round(x,6) for x in got)}"
              + ("" if good else f"  (wanted {want})"), flush=True)

    # rotor channel positions
    want = {"rotor0": (0.22990664, -0.22990664), "rotor1": (-0.22990664, 0.22990664),
            "rotor2": (-0.22990664, -0.22990664), "rotor3": (0.22990664, 0.22990664)}
    for nm, (wx, wy) in want.items():
        t = UsdGeom.Xformable(vs.GetPrimAtPath(f"{ROOT}/{nm}")
                              ).GetOrderedXformOps()[0].Get()
        expect(f"{nm} xy", (t[0], t[1]), (wx, wy), tol=1e-4)
    # arm mount on body now along +x
    mj1 = UsdPhysics.Joint(vs.GetPrimAtPath(f"{ROOT}/link2/manip_joint1"))
    p0 = mj1.GetLocalPos0Attr().Get()
    expect("manip_joint1 body-side anchor", (p0[0], p0[1]), (0.002, -0.0005), tol=1e-6)
    # manip_base link now on +x side
    t = UsdGeom.Xformable(vs.GetPrimAtPath(f"{ROOT}/manip_base")
                          ).GetOrderedXformOps()[0].Get()
    expect("manip_base position xy (arm side -> +x)", (t[0], t[1]),
           (0.14299719, -0.00152066), tol=1e-5)
    # body untouched frame, swapped inertia
    b = vs.GetPrimAtPath(BODY)
    bi = UsdPhysics.MassAPI(b).GetDiagonalInertiaAttr().Get()
    expect("body inertia (Ixx<->Iyy)", tuple(bi), (0.06334175, 0.06301228, 0.09868092),
           tol=1e-6)
    bq = get_ops(b).get("xformOp:orient").Get()
    expect("body orient stays identity", (bq.GetReal(),), (1.0,), tol=1e-6)

    print(f"\n[xfwd] {'ALL CHECKS PASSED' if ok else '*** CHECKS FAILED ***'} — "
          f"{DST}", flush=True)
    app.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
