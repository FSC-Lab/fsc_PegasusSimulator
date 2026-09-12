#!/usr/bin/env python3
"""Phase 1 of the tank plan: the slosh MODEL COPY exists, and it changes nothing yet.

Run with any python3 (no Isaac, no numpy needed):

    python3 tests/test_slosh_scaffold.py

Two halves:

  A. THE PHYSICS NUMBERS, from the shipped `dynamics/slosh.py` -- imported by
     file path, not re-derived here.  `tests/test_actuator_models.py` re-derives
     the thrust model because `vtol.py` cannot be imported outside Isaac, and
     that is exactly how two copies of a model start to drift.  The slosh maths
     was put in an Isaac-free module so this test can check the REAL code.
     The numbers it pins are the ones the tank plan was decided on: get a
     different answer and either the plan or the code is wrong.

  B. THE COPY IS STILL A COPY.  `VTOLSlosh` must add exactly one no-op hook to
     the baseline vehicle and nothing else, and `vtol.py` must be untouched by
     the tank work.  Several of these assertions are SUPPOSED to fail when
     phase 3 (the pendulum) lands -- each says so in its message.  That is the
     point: the scaffold cannot quietly grow physics without this gate noticing.
"""
import ast
import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
LOGIC = os.path.join(ROOT, "extensions", "pegasus.simulator", "pegasus",
                     "simulator", "logic")
SLOSH_PY = os.path.join(LOGIC, "dynamics", "slosh.py")
VTOL_PY = os.path.join(LOGIC, "vehicles", "vtol.py")
VTOL_SLOSH_PY = os.path.join(LOGIC, "vehicles", "vtol_slosh.py")
UTILS_PY = os.path.join(ROOT, "extensions", "fsc_aerial_manipulation",
                        "fsc_aerial_manipulation", "rotorcraft", "vtol_utils.py")
SCENE_PY = os.path.join(ROOT, "application", "vtol", "03_vtol_px4_hil.py")

# The airframe the tank has to fly on.
M_AIRFRAME = 3.45          # [kg]
V_STALL = 14.13            # [m/s] at the airframe mass
V_CRUISE = 16.0            # [m/s]
F_SHORT_PERIOD = 1.42      # [Hz] the mode this aircraft is marginal on

_spec = importlib.util.spec_from_file_location("slosh", SLOSH_PY)
slosh = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(slosh)


def near(a, b, tol, what, bad):
    if abs(a - b) > tol:
        bad.append(f"{what}: {a!r} != {b!r} (tol {tol})")


# ===========================================================================
#  A. PHYSICS
# ===========================================================================

def check_physics(bad):
    t = slosh.TankConfig()
    p = slosh.slosh_parameters(t)
    print("  " + slosh.describe(t))

    # -- the plan's baseline table ------------------------------------------
    near(p.freq_hz, 3.32, 0.01, "first-mode frequency [Hz]", bad)
    near(p.fluid_mass, 0.502, 0.002, "water mass [kg]", bad)
    near(p.slosh_mass, 0.101, 0.001, "slosh mass m1 [kg]", bad)
    near(p.slosh_mass_frac, 0.202, 0.005, "slosh mass fraction", bad)
    near(p.pendulum_length, 0.0225, 0.0005, "pendulum length [m]", bad)
    near(p.zeta, 0.0049, 0.0003, "smooth-wall damping ratio", bad)

    # -- invariants that must hold for ANY tank ------------------------------
    for R, fill in ((0.0414, 0.75), (0.03, 0.5), (0.10, 0.9), (0.06, 0.25)):
        q = slosh.slosh_parameters(slosh.TankConfig(radius=R, height=3 * R, fill_frac=fill))
        near(q.rigid_mass + q.slosh_mass, q.fluid_mass, 1e-12,
             f"m0+m1 != m_f at R={R}, fill={fill}", bad)
        # the pendulum analogue must REPRODUCE the slosh frequency, or it is not
        # an equivalent model at all
        near(math.sqrt(slosh.G / q.pendulum_length), q.omega, 1e-9,
             f"sqrt(g/L1) != omega at R={R}, fill={fill}", bad)
        if not 0.0 < q.slosh_mass < q.fluid_mass:
            bad.append(f"slosh mass out of range at R={R}, fill={fill}: {q.slosh_mass}")

    # -- frequency follows RADIUS, not fill ---------------------------------
    # (tanh saturates above h/R ~ 1.5, so a 60% and a 90% tank ring alike)
    fs = [slosh.slosh_parameters(slosh.TankConfig(fill_frac=f)).freq_hz
          for f in (0.6, 0.7, 0.75, 0.8, 0.9)]
    if max(fs) - min(fs) > 0.01 * max(fs):
        bad.append(f"frequency moved with fill: {['%.3f' % f for f in fs]}")
    print(f"  fill 60-90%: f = {min(fs):.3f}-{max(fs):.3f} Hz  (radius sets the note)")

    freqs = []
    for R in (0.03, 0.0414, 0.06, 0.10, 0.15, 0.225):
        q = slosh.slosh_parameters(slosh.TankConfig(radius=R, height=3 * R))
        freqs.append((R, q.freq_hz, q.freq_hz / F_SHORT_PERIOD, q.fluid_mass))
    for (R0, f0, _, _), (R1, f1, _, _) in zip(freqs, freqs[1:]):
        if f1 >= f0:
            bad.append(f"frequency not falling with radius: {R0}->{f0}, {R1}->{f1}")
    print("   R [m]   f [Hz]   vs short period   water [kg]")
    for R, f, ratio, m in freqs:
        print(f"   {R:5.3f}  {f:6.2f}   {ratio:5.2f}x           {m:7.3f}")

    # -- the default tank must clear BOTH constraints ------------------------
    if p.freq_hz / F_SHORT_PERIOD < 2.0:
        bad.append(f"default tank sits {p.freq_hz / F_SHORT_PERIOD:.2f}x the short period "
                   f"-- the plan requires clear separation from 1.42 Hz")
    stall = V_STALL * math.sqrt((M_AIRFRAME + p.fluid_mass) / M_AIRFRAME)
    print(f"  mass budget: +{p.fluid_mass:.3f} kg -> stall {stall:.2f} m/s, "
          f"cruise margin {V_CRUISE - stall:+.2f} m/s")
    if stall >= V_CRUISE:
        bad.append(f"default tank makes the aircraft unflyable: stall {stall:.2f} "
                   f">= cruise {V_CRUISE}")
    if V_CRUISE - stall < 0.5:
        bad.append(f"default tank leaves only {V_CRUISE - stall:.2f} m/s of cruise margin")

    # -- endpoints of the phase-4 fill sweep must be valid -------------------
    empty = slosh.slosh_parameters(slosh.TankConfig(fill_frac=0.0))
    if (empty.fluid_mass, empty.slosh_mass, empty.omega) != (0.0, 0.0, 0.0):
        bad.append(f"empty tank is not zeroed: {empty}")
    full = slosh.slosh_parameters(slosh.TankConfig(fill_frac=1.0))
    if not full.fluid_mass > 0:
        bad.append("full tank has no fluid mass")

    # -- damping: smooth wall resonates, baffles do not ----------------------
    baffled = slosh.slosh_parameters(slosh.TankConfig().baffled(0.05))
    print(f"  decay to half amplitude: smooth {p.half_amplitude_cycles:.0f} cycles, "
          f"baffled(5%) {baffled.half_amplitude_cycles:.1f}")
    if p.half_amplitude_cycles < 10:
        bad.append("smooth-wall tank is not the lightly damped resonator the plan describes")
    if baffled.half_amplitude_cycles > 5:
        bad.append("baffled tank does not damp")
    near(baffled.freq_hz, p.freq_hz, 1e-12, "baffling changed the frequency", bad)
    near(baffled.fluid_mass, p.fluid_mass, 1e-12, "baffling changed the mass", bad)

    # -- a bad tank must be rejected, not quietly accepted -------------------
    for kwargs in ({"fill_frac": 1.5}, {"fill_frac": -0.1}, {"radius": 0.0},
                   {"height": -1.0}, {"damping_zeta": 1.5}):
        try:
            slosh.TankConfig(**kwargs)
            bad.append(f"TankConfig accepted invalid {kwargs}")
        except ValueError:
            pass


# ===========================================================================
#  B. THE COPY IS STILL A COPY
# ===========================================================================

# Everything VTOLSlosh is allowed to define in phase 1.  Overriding anything
# else means the copy has started to diverge from the baseline plant.
ALLOWED_METHODS = {"__init__", "update", "_apply_slosh"}


def check_copy(bad):
    src = open(VTOL_SLOSH_PY).read()
    tree = ast.parse(src)
    classes = {n.name: n for n in tree.body if isinstance(n, ast.ClassDef)}

    for name in ("VTOLSloshConfig", "VTOLSlosh"):
        if name not in classes:
            bad.append(f"vtol_slosh.py does not define {name}")
    if not all(n in classes for n in ("VTOLSloshConfig", "VTOLSlosh")):
        return

    bases = {ast.unparse(b) for b in classes["VTOLSlosh"].bases}
    if bases != {"VTOL"}:
        bad.append(f"VTOLSlosh must subclass VTOL (a fork drifts from the plant); bases={bases}")
    if {ast.unparse(b) for b in classes["VTOLSloshConfig"].bases} != {"VTOLConfig"}:
        bad.append("VTOLSloshConfig must subclass VTOLConfig so new baseline fields carry over")

    methods = {m.name: m for m in classes["VTOLSlosh"].body
               if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef))}
    extra = set(methods) - ALLOWED_METHODS
    if extra:
        bad.append(f"VTOLSlosh overrides {sorted(extra)} -- the copy is diverging from "
                   f"the baseline plant. If that is deliberate, add it to ALLOWED_METHODS "
                   f"with a reason.")
    print(f"  VTOLSlosh defines {sorted(methods)}")

    # update() must EXTEND the baseline step, not replace it
    upd = methods.get("update")
    if upd is not None:
        calls = {ast.unparse(n.func) for n in ast.walk(upd) if isinstance(n, ast.Call)}
        if "super().update" not in calls:
            bad.append("VTOLSlosh.update does not call super().update -- it is replacing the "
                       "baseline step instead of extending it")
        if "self._apply_slosh" not in calls:
            bad.append("VTOLSlosh.update never calls _apply_slosh")

    # phase 1: the hook is a no-op. THIS ASSERTION IS MEANT TO FAIL AT PHASE 3 --
    # when the pendulum lands, move it to that phase's gate (frequency and mass
    # against the closed form) rather than deleting it.
    hook = methods.get("_apply_slosh")
    if hook is not None:
        body = [n for n in hook.body if not (isinstance(n, ast.Expr)
                                             and isinstance(n.value, ast.Constant))]
        if len(body) != 1 or not isinstance(body[0], ast.Return) or body[0].value is not None:
            bad.append("_apply_slosh is no longer a no-op: phase 1 claims the copy is "
                       "IDENTICAL to the baseline, so either that claim is now false or "
                       "this check belongs in the phase-3 gate")

    # a configured tank must be refused at CONSTRUCTION, not flown
    init = methods.get("__init__")
    if init is None or not any(isinstance(n, ast.Raise) and "NotImplementedError" in ast.unparse(n)
                               for n in ast.walk(init)):
        bad.append("VTOLSlosh.__init__ does not refuse a configured tank -- a tank that is "
                   "configured but not modelled is silent wrong physics")

    # the baseline plant must be untouched by the tank work
    base_src = open(VTOL_PY).read().lower()
    for word in ("slosh", "tank"):
        if word in base_src:
            bad.append(f"vtol.py mentions '{word}' -- the baseline plant is supposed to be "
                       f"untouched by the tank work")
    print("  vtol.py: no tank/slosh references (baseline untouched)")


def check_wiring(bad):
    src = open(UTILS_PY).read()
    tree = ast.parse(src)
    funcs = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}

    for name in ("fsc_cad_vtol_slosh_config", "_vehicle_class_for"):
        if name not in funcs:
            bad.append(f"vtol_utils.py does not define {name}")

    # the slosh airframe must be COPIED from the baseline airframe, not retyped
    if "fsc_cad_vtol_slosh_config" in funcs:
        body = ast.unparse(funcs["fsc_cad_vtol_slosh_config"])
        if "fsc_cad_vtol_config" not in body or "fields(" not in body:
            bad.append("fsc_cad_vtol_slosh_config must copy fsc_cad_vtol_config field-wise "
                       "(dataclasses.fields), or the two airframes will drift")

    # one selector: the config type picks the class
    if "_vehicle_class_for" in funcs:
        body = ast.unparse(funcs["_vehicle_class_for"])
        if "VTOLSloshConfig" not in body or "VTOL" not in body:
            bad.append("_vehicle_class_for must map VTOLSloshConfig -> VTOLSlosh, else -> VTOL")

    # the spawners take the override and DEFAULT TO NONE (= baseline behaviour)
    for name in ("spawn_vtol", "spawn_vtol_with_mavlink"):
        fn = funcs.get(name)
        if fn is None:
            bad.append(f"vtol_utils.py lost {name}")
            continue
        args = {a.arg: d for a, d in zip(fn.args.args[-len(fn.args.defaults):],
                                         fn.args.defaults)} if fn.args.defaults else {}
        if "vehicle_cls" not in args:
            bad.append(f"{name} has no vehicle_cls parameter")
        elif not (isinstance(args["vehicle_cls"], ast.Constant)
                  and args["vehicle_cls"].value is None):
            bad.append(f"{name}'s vehicle_cls must default to None (infer from the config), "
                       f"so the baseline path is unchanged")

    scene = open(SCENE_PY).read()
    if "VTOL_SLOSH" not in scene or "fsc_cad_vtol_slosh_config" not in scene:
        bad.append("03_vtol_px4_hil.py cannot select the slosh model (VTOL_SLOSH=1)")
    print("  spawners resolve the class from the config; scene selects with VTOL_SLOSH=1")


def main():
    bad = []
    print("A. slosh physics (shipped dynamics/slosh.py)")
    check_physics(bad)
    print("B. the model copy")
    check_copy(bad)
    check_wiring(bad)

    print()
    if bad:
        print(f"FAIL ({len(bad)}):")
        for b in bad:
            print("  -", b)
        return 1
    print("PASS -- tank maths matches the plan; the copy adds nothing to the baseline yet")
    return 0


if __name__ == "__main__":
    sys.exit(main())
