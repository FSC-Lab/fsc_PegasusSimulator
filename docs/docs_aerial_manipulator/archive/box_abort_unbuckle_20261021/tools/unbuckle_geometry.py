#!/usr/bin/env python3
"""Where the claw goes when the arm unbuckles from a joint-guard trip pose to the push pose with
the CoM held (the planner's transition legs fly the CoM; the base moves to compensate the arm).
MODEL frame: +y = the arm's forward axis = the push direction, +z up. Pure numpy.

    /usr/bin/python3 unbuckle_geometry.py
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, *[".."] * 5))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

P = TP.make_params_t650(base_com=[0.0, -0.017854, 0.0])
D = np.pi / 180.0
PUSH = np.array([0.0, 32.0, 28.0, 0.0]) * D
HOME = np.array([0.0, 40.0, 40.0, 0.0]) * D


def rel(q):
    r0c, r0e, re = TP.arm_fk_model(q, P)
    return r0e - r0c, r0e, re


def pitch_deg(re):
    a = re[:, 0]                       # the claw's forward axis in the base frame (model)
    return np.degrees(np.arctan2(a[2], np.hypot(a[0], a[1])))


def path(qa, qb, n=41):
    """the planner's joint path is a straight line in joint space on one phase"""
    out = []
    for s in np.linspace(0.0, 1.0, n):
        q = qa + s * (qb - qa)
        d, r0e, re = rel(q)
        out.append((s, q, d, r0e, re))
    return out


def report(name, qa, qb):
    pa = path(qa, qb)
    d0 = pa[0][2]
    dy = np.array([p[2][1] - d0[1] for p in pa]) * 1e3
    dz = np.array([p[2][2] - d0[2] for p in pa]) * 1e3
    dx = np.array([p[2][0] - d0[0] for p in pa]) * 1e3
    sig = [TP._sigma_nd(p[1], P) for p in pa]
    print(f"{name}: q {np.round(qa / D, 1)} -> {np.round(qb / D, 1)} deg")
    print(f"   claw vs CoM (CoM held): forward {dy[-1]:+6.1f} mm (range {dy.min():+.1f}..{dy.max():+.1f}), "
          f"up {dz[-1]:+6.1f} mm (range {dz.min():+.1f}..{dz.max():+.1f}), sideways {dx[-1]:+.1f}")
    print(f"   claw pitch {pitch_deg(pa[0][4]):+.1f} -> {pitch_deg(pa[-1][4]):+.1f} deg, "
          f"sigma_nd {min(sig):.3f}..{max(sig):.3f} (margin {TP.SIGMA_ND_MARGIN})")
    print(f"   base->claw reach {np.linalg.norm(pa[0][3][1:]) * 1e3:.0f} -> {np.linalg.norm(pa[-1][3][1:]) * 1e3:.0f} mm")


def main():
    d, r0e, re = rel(PUSH)
    print(f"push pose: base->claw {np.round(r0e * 1e3, 1)} mm, CoM->claw {np.round(d * 1e3, 1)} mm, "
          f"claw pitch {pitch_deg(re):+.1f} deg, sigma_nd {TP._sigma_nd(PUSH, P):.3f}")
    print()
    # the 10-21 trip pose (hardware q1/q4 sign-flipped to the model; q2/q3 as measured)
    trip = np.array([1.9, 44.9, 5.1, -2.0]) * D
    report("10-21 trip pose -> push pose", trip, PUSH)
    report("10-21 trip pose, q2/q3 only -> push pose's q2/q3", trip,
           np.array([trip[0], PUSH[1], PUSH[2], trip[3]]))
    report("10-21 worst (q3 -6.3) -> push pose", np.array([2.3, 44.8, -6.3, -1.8]) * D, PUSH)
    report("10-21 trip pose -> home", trip, HOME)
    print()
    # the other side of the same swing: q3 on its +50 stop, q2 low
    report("opposite buckle (q2 13, q3 50) -> push pose", np.array([0, 13.0, 50.0, 0]) * D, PUSH)
    print()
    print("trip poses over the guard's reach (q3 < 10 for 0.2 s), q2 33..45:")
    for q2 in (33.0, 38.0, 42.0, 45.0):
        for q3 in (10.0, 5.0, 0.0, -5.0):
            qa = np.array([0, q2, q3, 0]) * D
            pa = path(qa, PUSH)
            d0 = pa[0][2]
            dz = np.array([p[2][2] - d0[2] for p in pa]) * 1e3
            dy = np.array([p[2][1] - d0[1] for p in pa]) * 1e3
            print(f"   q2 {q2:4.0f} q3 {q3:4.0f}: forward {dy[-1]:+6.1f} (min {dy.min():+6.1f}) mm, "
                  f"up {dz[-1]:+6.1f} (min {dz.min():+6.1f}) mm, sigma_nd at start {TP._sigma_nd(qa, P):.3f}")


if __name__ == "__main__":
    main()
