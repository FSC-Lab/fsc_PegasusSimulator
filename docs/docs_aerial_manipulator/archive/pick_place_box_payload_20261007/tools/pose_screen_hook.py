#!/usr/bin/env python3
"""Arm-pose candidates for the box payload's HOOK grasp (2026-10-07): the claw
near horizontal (beta = q2 + q3 ~ 80 deg), inside the real arm's q2 range
[-20, 45] deg and q3's +50 stop. Per pose: the claw axis tilt, reach and
height (body frame), the kinematic / law sigma_nd, the joint margins, and the
J2 / J3 hold torque with the 200 g payload hung from the fingers.

    PYTHONNOUSERSITE=1 /usr/bin/python3 pose_screen_hook.py
"""
import numpy as np
from pose_sweep import row, chain, g_arm

POSES = ([0, 40, 40, 0], [0, 37.5, 42.5, 0], [0, 35, 45, 0], [0, 32.5, 47.5, 0], [0, 30, 50, 0],
         [0, 35, 40, 0], [0, 40, 45, 0], [0, 38, 47, 0], [0, 30, 45, 0], [0, 35, 47, 0])


def hung_tau(qd, m=0.2):
    q = np.radians(qd)

    def claw(qq):
        return chain(qq)[1][-1]
    e = 1e-6
    J = np.array([(claw(q + e * np.eye(4)[j]) - claw(q - e * np.eye(4)[j])) / (2 * e) for j in range(4)]).T
    return g_arm(q) + J.T @ np.array([0.0, 0.0, m * 9.81])


print("pose                beta  claw tilt   reach  claw z   sig_kin sig_law  q2 to -20/+45  q3 to +50  |tau2|/|tau3| (200 g hung)")
for p in POSES:
    r = row(p)
    t = hung_tau(p)
    print(f"{str(p):19s} {r['beta']:4.0f}  {90 - r['beta']:4.1f} down  {r['reach'] * 1e3:5.0f}  {r['r0e'][2] * 1e3:6.0f}   "
          f"{r['sk']:.3f}   {r['sl']:.3f}    {p[1] + 20:5.1f}/{45 - p[1]:4.1f}     {50 - p[2]:4.1f}      "
          f"{abs(t[1]):.2f}/{abs(t[2]):.2f}")
