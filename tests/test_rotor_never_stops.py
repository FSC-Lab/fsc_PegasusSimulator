#!/usr/bin/env python3
"""No commanded moment may ever stop a rotor.

A rotor commanded to zero does not just lose its share of thrust -- it STOPS, and the spin-up lag
(rotor_tau_spin, ~0.12 s with the momentum thrust model) then leaves the airframe with a dead
corner for long enough to roll it over. Flown: at the instant the tilt reached 90 deg the mixer
commanded FR=0.00 against FL=0.71 for a single tick, a +0.70 diagonal impulse, and the aircraft
went to -21 deg of roll and never recovered (bag vtol_20260911_133910, t=22.3).

Two causes, both fixed and both guarded here:
  * the desaturation compared nom.min() against min(nom + dT) -- DIFFERENT ROTORS -- so the scale
    factor came out too large and left a negative thrust, which thrust_to_throttle clips to zero;
  * even scaled correctly, "keep it >= 0" lands the most-limited rotor EXACTLY at zero, which
    stops it. ROTOR_FLOOR_FRAC reserves a floor.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ros2", "px4_vtol_direct_actuator",
                                "px4_vtol_direct_actuator"))
import nmpc_cascade_node as C   # noqa: E402


def main():
    alloc = C.RotorAlloc()
    rng = np.random.default_rng(11)
    g = dict(kR_roll=3.0, kW_roll=1.8, kR_pitch=3.0, kW_pitch=1.8, kR_yaw=5.0, kW_yaw=2.0,
             k_bank=1.2, k_bank_d=0.0, bank_max=math.radians(25.0),
             kI_roll=0.0, kI_yaw=0.0, i_band=math.radians(12.0), i_max=1.5)
    n, zeros, worst, skipped = 8000, 0, 1.0, 0
    for _ in range(n):
        th = rng.uniform(-0.35, 0.35)
        b = rng.uniform(0.0, math.pi / 2)
        R = np.array([[math.cos(th), 0, -math.sin(th)], [0, 1, 0],
                      [math.sin(th), 0, math.cos(th)]])
        a = rng.uniform(-0.7, 0.7)
        R = R @ np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
        w = rng.normal(0.0, 1.5, 3)
        V = rng.uniform(0.0, 20.0)
        vb = np.array([V, rng.normal(0.0, 2.0), rng.normal(0.0, 1.5)])
        v_ax = float(vb[0] * math.sin(b) + vb[2] * math.cos(b))
        if C.thrust_available(v_ax) <= 1e-9:
            skipped += 1                 # genuinely beyond the prop's zero-thrust speed
            continue
        mot, _, _, _, _ = C.inner_step(R, w, th, rng.uniform(-1, 1), b, V,
                                       rng.uniform(1.0, 30.0), rng.uniform(1.0, 30.0),
                                       rng.uniform(-0.3, 0.3), g, alloc,
                                       vb=vb, ei=np.zeros(3), dt=0.008)
        mn = min(mot)
        worst = min(worst, mn)
        if mn <= 1e-9 and max(mot) > 0.05:
            zeros += 1

    print(f"  {n - skipped} random attitudes / rates / speeds / thrust splits")
    print(f"  lowest motor command anywhere: {worst:.4f}")
    print(f"  single-rotor dropouts:         {zeros}")
    print(f"  (skipped {skipped} states beyond the propeller's zero-thrust advance ratio,")
    print(f"   where NO thrust is physically available and a zero command is correct)")
    if zeros or worst <= 1e-9:
        print("\n*** A ROTOR CAN STILL BE COMMANDED TO STOP ***")
        return 1
    print("\nNO ROTOR IS EVER COMMANDED TO STOP")
    return 0


if __name__ == "__main__":
    sys.exit(main())
