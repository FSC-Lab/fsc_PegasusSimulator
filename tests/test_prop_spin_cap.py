#!/usr/bin/env python3
"""Every prop link must have its PhysX angular-velocity cap raised above the rotor speed.

    python3 tests/test_prop_spin_cap.py

PhysX caps a rigid body's angular velocity at 100 rad/s unless told otherwise. The prop
links are velocity-driven to 400-750 rad/s, so the drive pushed against that cap forever
and the airframe took the reaction: measured with application/vtol/09_vtol_inertia_probe.py
(2026-09-13), a +-0.20 yaw-diagonal throttle step at hover with the modelled reaction
torque OFF still yawed the aircraft at 17 rad/s^2 -- 8.3 N.m no controller knew about,
3.3x the modelled reaction torque. In flight that was the rail-to-rail 2.7 Hz yaw thrash
of every cascade hover (bag slosh_20260912_235454, fitted as Izz 0.11 against the true
0.49), and through the 90 deg front pair a spurious ROLL torque in cruise. Lifting the cap
turns the step response into the physical spin-up impulse (-5 deg/s, implied prop inertia
7.5e-5 vs the configured 1e-4). The probe is the ground truth; this file only guards the
source so the call cannot be dropped from one of the two prop-link branches.
"""
import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
VTOL = os.path.join(HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator",
                    "logic", "vehicles", "vtol.py")


def main():
    src = open(VTOL).read()
    tree = ast.parse(src)
    bad = []
    fn = next((n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_unclamp_prop_spin"), None)
    if fn is None:
        bad.append("_unclamp_prop_spin is missing")
    else:
        body = ast.get_source_segment(src, fn)
        if "CreateMaxAngularVelocityAttr" not in body or "PhysxRigidBodyAPI" not in body:
            bad.append("_unclamp_prop_spin does not set physxRigidBody:maxAngularVelocity")
        if "rotor_omega_max" not in body:
            bad.append("the cap must scale with rotor_omega_max, not be a literal")
    br = next((n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_build_rotor"), None)
    if br is None:
        bad.append("_build_rotor is missing")
    else:
        calls = [n for n in ast.walk(br) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                 and n.func.attr == "_unclamp_prop_spin"]
        links = [n for n in ast.walk(br) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                 and n.func.attr == "_add_rigid_link"
                 and any(isinstance(k.value, ast.Name) and k.value.id == "prop_path" for k in n.keywords if k.arg == "path")]
        if len(links) != 2:
            bad.append(f"expected 2 prop-link builds (tiltable + fixed), found {len(links)}")
        if len(calls) < len(links):
            bad.append(f"{len(links)} prop links built but only {len(calls)} unclamped -- one branch lost the call")
    for b in bad:
        print("FAIL", b)
    print("PASS: every prop link is unclamped" if not bad else f"{len(bad)} problem(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
