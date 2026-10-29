#!/usr/bin/env python3
"""The valve's friction model on its own -- no vehicle, no stack (2026-10-26, the sweep's pre-check 1).

    $ISAAC_PY friction_check.py [--out ../analysis/friction_check.txt]

The valve asset (Valve_Assembly.usda) on its own in a 250 Hz PhysX world, authored as the valve
scene (07_px4_t650_aerial_manipulator_valve_turning.py) authors it: the printed rotor's 204 g
(inertia scaled with it), the revolute joint's angular drive. For each friction model, each
friction tau_f in {0.10, 0.25} N.m and each applied torque {0, 0.9, 1.1} x tau_f (a constant torque
about the axis, applied every physics step): does the rotor stay put below tau_f, turn above it,
and sit still at rest?
  tanh   the scene's default: -f tanh(w / 0.05) as an explicit torque, momentum-clamped (I |w| / dt)
  drive  PEGASUS_VALVE_FRICTION_MODEL=drive: the joint drive as a velocity constraint, damping
         D (N.m.s/rad), force limit f
"""
import argparse
import math
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np                                              # noqa: E402
import omni.usd                                                 # noqa: E402
from omni.isaac.core.world import World                         # noqa: E402
from omni.isaac.dynamic_control import _dynamic_control         # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics                    # noqa: E402

ASSET = os.path.expanduser("~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/"
                           "fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly.usda")
ROTOR_MASS = 0.204
DT = 1.0 / 250.0
W_TANH = 0.05
VALVE = "/World/valve"


def build(model, friction, stiction):
    """A fresh stage with the valve, authored like the scene. Returns the rotor's axis inertia."""
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    v = UsdGeom.Xform.Define(stage, VALVE)
    v.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.8))
    v.AddRotateZOp().Set(180.0)
    v.GetPrim().GetReferences().AddReference(ASSET)
    rotor = stage.GetPrimAtPath(VALVE + "/rotor")
    mass = UsdPhysics.MassAPI(rotor)
    m_asset = float(mass.GetMassAttr().Get())
    k = ROTOR_MASS / m_asset
    mass.GetMassAttr().Set(float(ROTOR_MASS))
    mass.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*[float(k * x) for x in mass.GetDiagonalInertiaAttr().Get()]))
    joint = stage.GetPrimAtPath(VALVE + "/joint")
    drive = UsdPhysics.DriveAPI.Get(joint, "angular") or UsdPhysics.DriveAPI.Apply(joint, "angular")
    drive.CreateStiffnessAttr(0.0)
    if model == "drive" and friction > 0.0:
        drive.CreateDampingAttr(float(stiction * math.pi / 180.0))
        drive.CreateMaxForceAttr(float(friction))
    else:
        drive.CreateDampingAttr(0.0)
    drive.CreateTargetVelocityAttr(0.0)
    st = Usd.Stage.Open(ASSET)              # kept in a variable: a prim of a dropped stage expires
    I_asset = st.GetDefaultPrim().GetAttribute("fsc:rotor_inertia_axis_kgm2").Get()
    return float(I_asset) * k


def run_case(model, friction, ratio, stiction, T=6.0):
    I = build(model, friction, stiction)
    world = World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0, backend="numpy", device="cpu")
    world.reset()
    dc = _dynamic_control.acquire_dynamic_control_interface()
    h = dc.get_rigid_body(VALVE + "/rotor")
    tau_a = ratio * friction
    log = []

    def step(dt):
        w = float(dc.get_rigid_body_angular_velocity(h).z)
        tau = tau_a
        if model == "tanh" and friction > 0.0:
            f = -friction * math.tanh(w / W_TANH)
            cap = I * abs(w) / max(dt, 1e-6)
            tau += max(-cap, min(cap, f))
        if tau != 0.0:
            dc.apply_body_torque(h, [0.0, 0.0, tau], True)
        r = dc.get_rigid_body_pose(h).r
        yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        log.append((world.current_time, yaw, w))

    world.add_physics_callback("fric", step)
    for _ in range(int(round(T / DT))):
        world.step(render=False)
    world.remove_physics_callback("fric")
    world.stop()
    world.clear_instance()
    a = np.array(log)
    a[:, 1] = np.unwrap(a[:, 1])
    return I, a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "analysis", "friction_check.txt"))
    ap.add_argument("--stiction", type=float, nargs="+", default=[100.0, 1000.0])
    a = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    rows = []
    models = [("tanh", 0.0)] + [("drive", s) for s in a.stiction]
    for model, stc in models:
        for f in (0.10, 0.25):
            for ratio in (0.0, 0.9, 1.1, -0.9, -1.1):
                I, x = run_case(model, f, ratio, stc)
                t, th, w = x[:, 0] - x[0, 0], x[:, 1], x[:, 2]
                tail = t >= 1.0
                turned = math.degrees(th[-1] - th[0])
                w_mean = math.degrees(np.mean(w[tail]))
                w_rms_dev = math.degrees(np.std(w[tail]))
                # the slip's acceleration over its first second vs (|tau_a| - f) / I
                k1 = t <= 1.0
                acc = np.polyfit(t[k1], w[k1], 1)[0] if k1.sum() > 10 else float("nan")
                acc_pred = (abs(ratio) - 1.0) * f / I * np.sign(ratio) if abs(ratio) > 1.0 else 0.0
                rows.append((model, stc, f, ratio, I, turned, w_mean, w_rms_dev, acc, acc_pred,
                             math.degrees(np.max(np.abs(w)))))
                print(f"{model:5s} D={stc:6.0f}  f={f:.2f}  tau_a={ratio:+.1f} f  ->  turned {turned:+9.3f} deg in 6 s, "
                      f"mean rate {w_mean:+8.3f} deg/s (std {w_rms_dev:.4f}), accel {acc:+7.3f} rad/s^2 "
                      f"(Coulomb {acc_pred:+7.3f}), |w|max {math.degrees(np.max(np.abs(w))):.3f} deg/s", flush=True)
    with open(a.out, "w") as fo:
        fo.write("# friction_check.py: the valve alone, 250 Hz, rotor 204 g, I = %.6f kg m^2\n" % rows[0][4])
        fo.write("# model D[N.m.s/rad] f[N.m] tau_a/f turned_deg_6s mean_rate_dps(t>1s) rate_std_dps accel_rad_s2 "
                 "coulomb_accel_rad_s2 max_rate_dps\n")
        for r in rows:
            fo.write(f"{r[0]:5s} {r[1]:6.0f} {r[2]:.2f} {r[3]:+.1f} {r[5]:+10.4f} {r[6]:+9.4f} {r[7]:.5f} "
                     f"{r[8]:+8.3f} {r[9]:+8.3f} {r[10]:.4f}\n")
    print(f"wrote {a.out}")
    app.close()


if __name__ == "__main__":
    main()
