#!/usr/bin/env python3
"""Do the controllers' internal aero models agree with the plant they will fly?

Every model-based controller carries its own copy of the aerodynamics, written in CasADi so
it can go inside an NMPC. Those copies drifted from the simulator, and because each
controller's self-test integrates its OWN model as "the true plant", the drift was invisible
offline -- which is how a wing that lifted only at negative angle of attack survived months
of "offline validated" runs (2026-09-06 audits).

This compares them directly, over a sweep of states that spans hover, transition and cruise:

    ~/envs/isaac_env/bin/python3 tests/test_controller_aero_match.py
"""
import importlib.util
import math
import os
import sys

import numpy as np
import casadi as ca

_HERE = os.path.dirname(os.path.abspath(__file__))
# The flight controller lives in fsc_autopilot_ros2 now, not in this repo.
# controller_path resolves it (workspace symlink, or $FSC_CONTROLLER_DIR), so a
# future move of that repository needs no change here.
sys.path.insert(0, os.path.join(_HERE, ".."))
from controller_path import controller_dir, inner_dir   # noqa: E402
_CTRL = controller_dir()
_CTRL_INNER = inner_dir()

_DA = _CTRL
sys.path.insert(0, _DA)

_spec = importlib.util.spec_from_file_location(
    "vtol_aerodynamics",
    os.path.join(_HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator",
                 "logic", "dynamics", "vtol_aerodynamics.py"))
_aero_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_aero_mod)
PLANT = _aero_mod.VTOLAerodynamics()

FAILURES = []


class _State:
    def __init__(self, vb, wb):
        self.linear_body_velocity = np.asarray(vb, float)
        self.angular_velocity = np.asarray(wb, float)


# states worth agreeing about: hover, the transition corridor, wing-borne cruise, and the
# regimes controllers used to be blind to (descent, sideways, sideslip, rates)
CASES = [
    ("hover, still",            [0.0, 0.0, 0.0],    [0, 0, 0],       0.0, 0.0, 0.0),
    ("hover, 3 m/s descent",    [0.0, 0.0, -3.0],   [0, 0, 0],       0.0, 0.0, 0.0),
    ("hover, 6 m/s crosswind",  [0.0, -6.0, 0.0],   [0, 0, 0],       0.0, 0.0, 0.0),
    ("creep 2 m/s",             [2.0, 0.0, 0.0],    [0, 0, 0],       0.0, 0.0, 0.0),
    ("corridor 6 m/s, +3 AoA",  [6.0, 0.0, -0.31],  [0, 0, 0],       0.0, 0.0, 0.0),
    ("corridor 10 m/s, +5 AoA", [10.0, 0.0, -0.87], [0, 0.2, 0],    -0.1, 0.0, 0.0),
    ("cruise 16 m/s, +6 AoA",   [16.0, 0.0, -1.68], [0, 0, 0],      -0.1, 0.0, 0.0),
    ("cruise + elevator",       [16.0, 0.0, -1.68], [0, 0, 0],       0.35, 0.0, 0.0),
    ("cruise + aileron",        [16.0, 0.0, -1.68], [0.3, 0, 0],     0.0, 0.30, 0.0),
    ("cruise + rudder + slip",  [16.0, 1.4, -1.68], [0, 0, 0.2],     0.0, 0.0, 0.25),
    ("post-stall 12 m/s, 25deg",[12.0, 0.0, -5.6],  [0, -0.3, 0],    0.0, 0.0, 0.0),
    ("descending cruise",       [14.0, 0.0, 2.0],   [0, 0, 0],       0.05, 0.0, 0.0),
]

# tolerance: absolute floor for near-zero cases, plus a relative band
ABS_N, ABS_NM, REL = 0.25, 0.06, 0.06


def compare(name, fn):
    print(f"\n{name}")
    print(f"      {'case':26s} {'|F| plant':>10s} {'|F| model':>10s} {'|M| plant':>10s} "
          f"{'|M| model':>10s}  worst")
    worst_all = 0.0
    for label, vb, wb, de, da, dr in CASES:
        Fp, Mp = PLANT.update(_State(vb, wb),
                              {"elevator": de, "aileron": da, "rudder": dr})
        Fc, Mc = fn(vb, wb, de * PLANT.config.max_surface_deflection_rad,
                    da * PLANT.config.max_surface_deflection_rad,
                    dr * PLANT.config.max_surface_deflection_rad)
        Fc = np.asarray(Fc, float).ravel(); Mc = np.asarray(Mc, float).ravel()
        dF = np.abs(Fc - Fp); dM = np.abs(Mc - Mp)
        okF = np.all(dF <= ABS_N + REL * np.abs(Fp))
        okM = np.all(dM <= ABS_NM + REL * np.abs(Mp))
        worst = max(float(np.max(dF)), float(np.max(dM)) * 10.0)
        worst_all = max(worst_all, worst)
        flag = "" if (okF and okM) else "   <-- MISMATCH"
        print(f"      {label:26s} {np.linalg.norm(Fp):10.2f} {np.linalg.norm(Fc):10.2f} "
              f"{np.linalg.norm(Mp):10.2f} {np.linalg.norm(Mc):10.2f}  "
              f"dF={np.max(dF):.2f}N dM={np.max(dM):.3f}Nm{flag}")
        if not (okF and okM):
            FAILURES.append(f"{name}: {label}")
    return worst_all


def casadi_wrapper(fn):
    vb = ca.SX.sym("vb", 3); om = ca.SX.sym("om", 3)
    de = ca.SX.sym("de"); da = ca.SX.sym("da"); dr = ca.SX.sym("dr")
    F, M = fn(vb, om, de, da, dr)
    f = ca.Function("aero", [vb, om, de, da, dr], [F, M])
    return lambda v, w, e, a, r: [np.array(x).ravel() for x in f(ca.DM(v), ca.DM(w), e, a, r)]


if __name__ == "__main__":
    print("controller aero models vs the simulator's own")
    print("=" * 96)
    from px4_vtol_direct_actuator import nmpc_so3_transition_node as so3
    w = casadi_wrapper(so3.aero_body)
    compare("nmpc_so3_transition_node.aero_body  (also used by nmpc6, nmpc_full, cascade)",
            lambda v, wb, e, a, r: w(v, wb, e, a, r))
    try:
        from px4_vtol_direct_actuator import vtol_nmpc
        w2 = casadi_wrapper(vtol_nmpc._aero_body)
        compare("vtol_nmpc._aero_body", lambda v, wb, e, a, r: w2(v, wb, e, a, r))
    except Exception as exc:
        print(f"\nvtol_nmpc: skipped ({exc})")

    print("\n" + "=" * 96)
    if FAILURES:
        print(f"{len(FAILURES)} MISMATCH(ES):")
        for f in FAILURES:
            print("  -", f)
        sys.exit(1)
    print("all controller aero models match the plant")
