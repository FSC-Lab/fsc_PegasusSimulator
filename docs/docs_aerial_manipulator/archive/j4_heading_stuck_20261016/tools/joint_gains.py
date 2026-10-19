#!/usr/bin/env python3
"""Joint-space stiffness / damping the whole-body law's EE impedance puts on each arm joint, at a pose.
Kq = G K_y J_y,q and Dq = G D_y J_y,q with G = J_3y^T (J_3y J_3y^T + lambda^2 I)^-1 Lambda_y M_y^-1 -- the
linearisation of u3's impedance term (wb_controller.cpp: -dls(Lam_Myinv (D_y e_vy + K_y e_y))) about a hover
at rest. Same construction as ../../arm_armature_20260926/arm_stiffness_sim.py terms.
usage: joint_gains.py [ky_psi dy_psi [my_psi]] ...   (default: the shipped 4-D hardware heading gains)"""
import os
import sys
import numpy as np
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), *[".."] * 5))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C          # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP    # noqa: E402

J_ARM = np.array([0.010, 0.0194, 0.0097, 0.0097])      # wb_armature_j1..j4 (hardware 4-D yaml)
KY, DY, LAM = 211.9, 26.82, 0.3                         # wb_ky/dy_x..z, wb_dls_lambda


def gains(q_deg, ky_psi, dy_psi, my_psi=0.05):
    P = TP.make_params_t650(armature_diag=J_ARM)
    X = np.zeros(26); X[3:12] = np.eye(3).reshape(9, order="F"); X[12:16] = np.radians(q_deg)
    d = C.dynamics(X, P)
    J3, Lam, Jq = d["J_3y"], d["Lambda_y"], d["J_y"][:, 6:]
    G = J3.T @ np.linalg.solve(J3 @ J3.T + LAM ** 2 * np.eye(4), Lam @ np.linalg.inv(np.diag([1, 1, 1, my_psi])))
    Kq = G @ np.diag([KY, KY, KY, ky_psi]) @ Jq
    Dq = G @ np.diag([DY, DY, DY, dy_psi]) @ Jq
    return Kq, Dq, d


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True, linewidth=160)
    a = [float(x) for x in sys.argv[1:]] or [0.2484, 0.2903]
    sets = [a[i:i + 2] for i in range(0, len(a), 2)]
    for q in ([0, 32, 38, 0], [0, 40, 40, 0], [0, 30, 30, 0]):
        for ky, dy in sets:
            Kq, Dq, d = gains(q, ky, dy)
            M4 = d["M_tilde"][9, 9]
            print(f"q {q}  K_psi {ky:.4g} D_psi {dy:.4g}:  Kq j4 {Kq[3, 3]:.4f} N.m/rad  Dq j4 {Dq[3, 3]:.4f} N.m.s/rad"
                  f"  | Kq j1 {Kq[0, 0]:.3f} j1<-j4 {Kq[0, 3]:+.4f} j4<-j1 {Kq[3, 0]:+.4f} | M_tilde j4 {M4:.4f}"
                  f"  -> wn {np.sqrt(Kq[3, 3] / M4):.2f} rad/s  zeta {Dq[3, 3] / (2 * np.sqrt(Kq[3, 3] * M4)):.2f}")
