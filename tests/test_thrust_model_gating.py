#!/usr/bin/env python3
"""The omega integration and the thrust computation must be gated by the SAME decision.

`thrust_model="momentum"` computes thrust from the rotor's spin speed, but the spin-speed
integration used to be gated on the separate `rotor_dynamics` flag. Selecting "momentum" while
leaving `rotor_dynamics=False` therefore left every omega at 0 -- and the aircraft would have made
ZERO THRUST, in a sim that otherwise looked perfectly healthy. Static analysis, no Isaac needed.
"""
import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator",
                   "logic", "vehicles", "vtol.py")


def main():
    tree = ast.parse(open(SRC).read())
    cls = next(n for n in ast.walk(tree)
               if isinstance(n, ast.ClassDef) and n.name == "VTOL")
    meths = {m.name: m for m in cls.body if isinstance(m, ast.FunctionDef)}
    bad = []

    if "_spin_model" not in meths:
        bad.append("VTOL._spin_model() is gone -- the gate is no longer centralised")

    # every site that decides whether rotors spin / how thrust is made must call _spin_model()
    for name in ("update", "_apply_propulsion", "_drive_actuators"):
        m = meths.get(name)
        if m is None:
            bad.append(f"VTOL.{name} missing")
            continue
        # Look at real ATTRIBUTE ACCESSES, not ast.dump() -- dump includes docstrings, so a
        # comment mentioning `config.rotor_dynamics` used to read as a violation.
        uses_gate = any(isinstance(n, ast.Attribute) and n.attr == "_spin_model"
                        for n in ast.walk(m))
        raw = any(isinstance(n, ast.Attribute) and n.attr == "rotor_dynamics"
                  for n in ast.walk(m))
        print(f"  {name:20s} uses _spin_model(): {uses_gate}   reads rotor_dynamics directly: {raw}")
        if not uses_gate:
            bad.append(f"{name} does not use _spin_model() -- gates can drift out of step")
        if raw:
            bad.append(f"{name} still reads cfg.rotor_dynamics directly; use _spin_model()")

    if bad:
        print("\nGATING BROKEN:")
        for b in bad:
            print("   -", b)
        return 1
    print("\nTHRUST-MODEL GATING IS CONSISTENT")
    return 0


if __name__ == "__main__":
    sys.exit(main())
