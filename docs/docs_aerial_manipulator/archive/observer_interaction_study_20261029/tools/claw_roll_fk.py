#!/usr/bin/env python3
"""Forward kinematics of the flown q1/q4 swing (q1 +16.4, q4 -18.3 deg, from energy_flow.py): how far the
claw moves relative to the base and about which axis it turns, at the flown fold and two larger ones."""
import numpy as np
import valve_bench as V
C, TP, CS = V.C, V.TP, V.CS
model = TP.make_params_t650(armature_diag=CS.J_ARM); N = 4

def pose(q):
    X = np.zeros(12 + 3 * N + 6); X[3:12] = np.eye(3).flatten(order="F"); X[12:16] = np.radians(q)
    d = C.dynamics(X, model); return d["r_0e_0"], d["R_e_0"]

for fold in ((5, 25), (20, 30), (30, 40)):
    r0, R0 = pose([0, *fold, 0])
    for a1, a4 in ((16.4, -18.3), (16.4, 0.0), (0.0, -18.3)):
        r1, R1 = pose([a1, *fold, a4]); E = R1 @ R0.T
        ang = np.degrees(np.arccos(np.clip((np.trace(E) - 1) / 2, -1, 1)))
        ax = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]]); ax /= np.linalg.norm(ax) + 1e-12
        print(f"beta {sum(fold):2d}: q1 {a1:+5.1f} q4 {a4:+5.1f} -> claw moves {np.linalg.norm(r1 - r0)*1e3:5.1f} mm, "
              f"turns {ang:5.1f} deg about (right {ax[0]:+.2f}, nose {ax[1]:+.2f}, up {ax[2]:+.2f})  [model frame]")
