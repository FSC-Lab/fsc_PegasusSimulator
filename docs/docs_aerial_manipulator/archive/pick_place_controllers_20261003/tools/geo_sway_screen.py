#!/usr/bin/env python3
"""Damping of the decoupled law's ~0.6 Hz lateral/attitude sway, per gain set
(2026-10-03, pick-and-place). Uses the 10-02 linear one-axis model of the
geometric + L1 node (archive/decoupled_flight_20261002/tools/lateral_mode_model.py:
position PD -> attitude PD -> MN4010 rotor lag + the L1 torque channel).

Why: carrying the payload, the decoupled rig's 0.56 Hz roll/pitch sway grew to
the 15 deg guard (geo_5, geo_9 at 100 g; geo_10 at 50 g) -- the ~0.1 damping
the 10-02 hardware flights found in this tune, pushed over by the load. The
payload enters here as extra TRUE inertia J_t (the law keeps J_m = 0.119):
100 g at ~0.40 m from the CoM adds ~0.016 kg m^2.

    PYTHONNOUSERSITE=1 /usr/bin/python3 geo_sway_screen.py
"""
import itertools
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "decoupled_flight_20261002", "tools")))
import lateral_mode_model as LM   # noqa: E402

# Isaac's feedback on this rig: raw mocap emulator (60 Hz, ~10 ms) -> dp 3 ticks;
# PX4 attitude/gyro -> da 2 ticks. Plus a harder case.
DELAYS = ((3, 2), (6, 3))
J_TRUE = (0.119, 0.135, 0.150)
SHIPPED = dict(kp=30.0, kv=13.5, kr=3.337, kw=0.9505, a=-2.791, wc=1.0)    # the pick-and-place yaml (geo_8..10)
TUNE_1001 = dict(kp=20.11, kv=11.05, kr=3.337, kw=0.9505, a=-2.791, wc=1.0)


def worst(g):
    """least damping over the payload inertias and delays: (zeta, f_hz)"""
    out = (9.0, float("nan"))
    for J in J_TRUE:
        for dp, da in DELAYS:
            m = LM.modes(LM.build(g, J, dp, da, True))
            if m and m[0][1] < out[0]:
                out = (m[0][1], m[0][0])
    return out


def main():
    for nm, g in (("10-01 tune", TUNE_1001), ("pick-and-place yaml (kp 30)", SHIPPED)):
        z, f = worst(g)
        print(f"{nm:30s} worst zeta {z:+.3f} at {f:.2f} Hz")
    rows = []
    for kp, kv, kr, kw in itertools.product((10, 15, 20, 30), (8, 11, 14, 18, 22), (2.0, 2.5, 3.337, 4.5),
                                            (0.6, 0.95, 1.3, 1.7)):
        g = dict(TUNE_1001, kp=kp, kv=kv, kr=kr, kw=kw)
        z, f = worst(g)
        rows.append((z, f, kp, kv, kr, kw))
    rows.sort(reverse=True)
    print("\nbest 20 (worst-case zeta over J_true x delay):")
    print(" zeta    f Hz   kp    kv    kr     kw")
    for z, f, kp, kv, kr, kw in rows[:20]:
        print(f"{z:+.3f}  {f:5.2f}  {kp:4.0f}  {kv:4.0f}  {kr:5.3f}  {kw:4.2f}")
    print("\nkr / kw held at the 10-01 tune (only the position pair moves):")
    for z, f, kp, kv, kr, kw in rows:
        if abs(kr - 3.337) < 1e-6 and abs(kw - 0.95) < 1e-6:
            print(f"{z:+.3f}  {f:5.2f}  {kp:4.0f}  {kv:4.0f}")


if __name__ == "__main__":
    main()


def fine():
    """the position pair only, attitude at the 10-01 tune; zeta per payload"""
    print("\nposition pair only (kr 3.337 / kw 0.95 / L1 as tuned): worst zeta over delays, per payload J_t")
    print("  kp    kv   | J 0.119 (none)   J 0.135 (100 g)   J 0.150 (200 g)")
    for kp in (10.0, 12.5, 15.0, 17.5, 20.11):
        for kv in (5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.05):
            cells = []
            for J in J_TRUE:
                z = min((LM.modes(LM.build(dict(TUNE_1001, kp=kp, kv=kv), J, dp, da, True)) or [(0, 9)])[0][1]
                        for dp, da in DELAYS)
                cells.append(z)
            print(f"{kp:5.1f} {kv:5.1f}  |   {cells[0]:+.3f}            {cells[1]:+.3f}            {cells[2]:+.3f}")
