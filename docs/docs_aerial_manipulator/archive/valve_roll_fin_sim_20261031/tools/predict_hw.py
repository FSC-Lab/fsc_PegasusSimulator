#!/usr/bin/env python3
"""(2026-10-31) What the roll fin should do on HARDWARE, from the two 10-29 offline models (Isaac does not
reproduce the hardware's growing 0.45-0.6 Hz contact sway, so it cannot show this part).

1. Grip rotation left over (wb_valve_turning_flight_20261029/tools/handle_joint_mobility.py's arm model):
   with the claw clamped, a sideways move or a roll of the vehicle forces a claw rotation square to the
   claw axis; the roll fin frees the part along ITS pin -- fixed at 30 deg up in the valve frame (the
   10-29 tool re-aims its pin per pose; the printed one cannot). Per arm pose of the turns: the forced
   rotation per 10 mm sideways / per 1 deg roll, and what the fixed pin leaves (about the radial jaw
   axis, which no roll pin frees).
2. The sideways mode (lateral_roll_model.py, the 10-28 reduced model; it ranks, it never reproduced the
   flown growth): its decay rate at the grip roll stiffness k_s the hardware had (3-7 N.m/rad, calibrated
   there) and at what is left with the pin: k_s x (left / forced)^2 (the stiffness the leftover rotation
   still meets, assumed as stiff as before -- the conservative case) + the O-ring.
    PYTHONNOUSERSITE=1 /usr/bin/python3 predict_hw.py
"""
import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
T29 = os.path.join(HERE, "..", "..", "wb_valve_turning_flight_20261029", "tools")
sys.path.insert(0, T29)


def load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(T29, name + ".py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


hj = load("handle_joint_mobility")
lr = load("lateral_roll_model")
D = np.pi / 180
PIN = np.array([0.0, np.cos(30 * D), np.sin(30 * D)])     # model axes (radial, tangent, up) = the valve's (X, y, z)

POSES = [("flown grasp [0, 5, 25, 0]", [0, 5, 25, 0]), ("fold [0, 10, 20, 0]", [0, 10, 20, 0]),
         ("fold [0, -10, 45, 0]", [0, -10, 45, 0]), ("wrist swung [10, 5, 25, -13]", [10, 5, 25, -13]),
         ("wrist swung [-10, 5, 25, 13]", [-10, 5, 25, 13]), ("wrist swung [20, 5, 25, -26]", [20, 5, 25, -26])]

print("1. GRIP ROTATION the vehicle's motion forces, and what the 30 deg pin leaves")
print(f"   {'arm pose':30s} {'sideways 10 mm: forced -> left':>34s} {'roll 1 deg: forced -> left':>30s}")
fr_side = []
for name, q in POSES:
    J, e0 = hj.arm_twists(np.asarray(q, float) * D)
    cells = []
    for rot, tr in ((0 * hj.X, 0.010 * hj.X), (hj.Y * D, 0 * hj.X)):
        g = hj.grip_rotation(J, e0, rot, tr) / D
        left = g - (g @ PIN) * PIN
        cells.append((np.linalg.norm(g), np.linalg.norm(left)))
    fr_side.append(cells[0][1] / cells[0][0])
    print(f"   {name:30s} {cells[0][0]:9.2f} -> {cells[0][1]:5.2f} deg ({100 * cells[0][1] / cells[0][0]:3.0f} %)"
          f"      {cells[1][0]:6.2f} -> {cells[1][1]:5.2f} deg ({100 * cells[1][1] / cells[1][0]:3.0f} %)")


def slow(P):
    ev = np.linalg.eigvals(lr.A_of(P))
    o = [l for l in ev if 1.0 < l.imag < 6.0]
    return max(o, key=lambda v: v.real) if o else complex(np.nan, np.nan)


print("\n2. THE SIDEWAYS MODE (reduced model): decay rate [1/s] (more negative = better damped)")
print(f"   free flight (k_s 0): {slow(lr.with_(k_s=0.0)).real:+.2f}")
for ks in (3.0, 5.0, 7.0):
    fixed = slow(lr.with_(k_s=ks)).real
    nominal = slow(lr.with_(k_s=ks * fr_side[0] ** 2)).real
    worst = slow(lr.with_(k_s=ks * max(fr_side) ** 2)).real
    print(f"   hardware grip k_s {ks:.0f} N.m/rad: fixed fin {fixed:+.2f} | roll fin, flown pose (k_s x {fr_side[0] ** 2:.4f}) "
          f"{nominal:+.2f} | roll fin, wrist swung 20/26 deg (k_s x {max(fr_side) ** 2:.3f}) {worst:+.2f}")
print("   (the O-ring: Coulomb 2-4 mN.m, i.e. a couple capped at 2-4 mN.m against the ~70 mN.m rms the fixed fin "
      "passed on 10-29; gravity 0.8 mN.m/rad)")
