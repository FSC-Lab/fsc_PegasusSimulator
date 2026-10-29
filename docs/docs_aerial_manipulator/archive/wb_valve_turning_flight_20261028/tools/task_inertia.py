#!/usr/bin/env python3
"""The natural task inertia Lambda_y at the valve's grasp pose [0, 5, 25, 0] deg (the flown T650 model:
base_com_y -0.017854, joint-diagonal armature), in the nose frame (along the nose = the turn's
tangent, lateral = radial at the fin), and the inertia-shaping gain (Lambda_y M_y^-1 - I) with which the
4-D law feeds the contact-force estimate F_hat_y back into u3 in contact (M_y = diag(1,1,1,0.05) flown).
Also at the box-push pose for comparison."""
import os, sys
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), *[".."] * 5))
sys.path.insert(0, os.path.join(ROOT, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C

P = TP.make_params_t650(base_com=[0.0, -0.017854, 0.0], armature_diag=[0.010, 0.0194, 0.0097, 0.0097])
def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
def lam(q_deg, psi_nose_deg):
    n = P["n"]; X = np.zeros(18 + 2 * n)
    X[3:12] = Rz(np.radians(psi_nose_deg) - np.pi / 2).flatten(order="F")   # R0_model = R0_actual Rz(-90)
    X[12:12 + n] = np.radians(q_deg)
    d = C.dynamics(X, P)
    return d["Lambda_y"]
for lab, q, psi in (("valve grasp [0,5,25,0], nose -85.7 (turn start)", [0, 5, 25, 0], -85.7),
                    ("valve, nose +4.3 (turn end, +90)", [0, 5, 25, 0], 4.3),
                    ("box push [0,28,32,0], nose -90", [0, 28, 32, 0], -90.0)):
    L = lam(q, psi); a = np.radians(psi); nose = np.array([np.cos(a), np.sin(a)]); lat = np.array([-np.sin(a), np.cos(a)])
    Lxy = L[:2, :2]
    l_n, l_l = nose @ Lxy @ nose, lat @ Lxy @ lat; l_nl = nose @ Lxy @ lat
    My = np.diag([1.0, 1.0, 1.0, 0.05]); G = L @ np.linalg.inv(My) - np.eye(4)
    print(f"{lab}:\n  Lambda along nose {l_n:.3f}  lateral {l_l:.3f}  cross {l_nl:+.3f}  vertical {L[2,2]:.3f} kg  heading {L[3,3]:.4f} kg m^2"
          f"\n  world diag {np.round(np.diag(L),3)}  ->  shaping gain (Lambda M_y^-1 - I) diag {np.round(np.diag(G),2)}")
