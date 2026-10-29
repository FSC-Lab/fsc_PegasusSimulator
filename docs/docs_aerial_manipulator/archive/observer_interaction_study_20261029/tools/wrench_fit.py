#!/usr/bin/env python3
"""Is the 10-28 contact-sway roll disturbance a CLAW WRENCH?  At every 20 ms of the sway window: the law's
model at the measured configuration (PX4 attitude -> model frame, measured joints), the observer's deadbeat
residual in physical coordinates w = T^T d^c, and the least-squares claw wrench W_e with J_e^T W_e = w.
Lock-in at the sway frequency of the fitted wrench (fin axes) and of what it leaves unexplained, row by row."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import valve_bench as V                                     # noqa: E402  (C, TP, CS on the path)
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_valve_turning_flight_20261028", "tools"))
from vt_common import Bag                                   # noqa: E402
sys.path.insert(0, os.path.join(V.WH.CB.__file__.rsplit("/docs/", 1)[0], "extensions", "fsc_aerial_manipulation",
                                "fsc_aerial_manipulation", "robotic_arm", "utils_controller"))
import l1_observer as L1                                    # noqa: E402

C, TP, CS = V.C, V.TP, V.CS
N = 4

def lockin(t, x, f):
    A = np.column_stack([np.cos(2*np.pi*f*t), np.sin(2*np.pi*f*t), np.ones_like(t), t - t.mean()])
    c, *_ = np.linalg.lstsq(A, x, rcond=None)
    return complex(c[0], -c[1])

def quatR(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

NED2ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]]); FRD2FLU = np.diag([1.0, -1.0, -1.0])
model = TP.make_params_t650(armature_diag=CS.J_ARM)
model["base_com"] = TP.R_MODEL.T @ np.array([-0.017854, 0.0, 0.0])

for tag, win, f0 in (("180652", (41.0, 47.5), 0.490), ("181833", (41.0, 44.0), 0.634)):
    b = Bag(tag)
    tw = b.t("wb"); W = b.f("wb", "data"); ok = np.isfinite(W[:, 100]); tw, W = tw[ok], W[ok]
    g = np.arange(win[0], win[1], 0.02); tt = g - g[0]
    col = lambda i: np.interp(g, tw, W[:, i])
    to = b.t("odom"); P = np.stack([b.f("odom", f"pose.pose.position.{c}") for c in "xyz"], 1)
    tv = b.t("vatt"); qv = b.f("vatt", "q")
    dc = np.stack([col(62 + i) for i in range(10)], 1)
    rows = []
    for k_, t in enumerate(g):
        X = np.zeros(12 + 3 * N + 6)
        X[0:3] = [np.interp(t, to, P[:, i]) for i in range(3)]
        R = NED2ENU @ quatR(*qv[min(np.searchsorted(tv, t), len(tv) - 1)]) @ FRD2FLU @ TP.R_MODEL
        X[3:12] = R.flatten(order="F"); X[12:12 + N] = [col(5 + i)[k_] for i in range(N)]
        dyn = C.dynamics(X, model)
        com_err = np.linalg.norm(X[0:3] + R @ dyn["r_0c_0"] - np.array([col(48 + i)[k_] for i in range(3)]))
        w = dyn["T"].T @ dc[k_]
        Je = L1.end_effector_jacobian(dyn)
        We, *_ = np.linalg.lstsq(Je.T, w, rcond=None)
        Rc = R @ dyn["R_e_0"]
        rows.append((w, Je.T @ We, Rc @ We[0:3], Rc @ We[3:6], com_err, R))
    w = np.array([r[0] for r in rows]); wfit = np.array([r[1] for r in rows])
    Fw = np.array([r[2] for r in rows]); Mw = np.array([r[3] for r in rows]); ce = np.array([r[4] for r in rows])
    R0 = rows[0][5]; nose = R0[:, 1]; nose[2] = 0; nose /= np.linalg.norm(nose); nrm = np.array([-nose[1], nose[0], 0.0])
    print(f"===== {tag} {win} at {f0} Hz   (model CoM vs logged CoM: max {ce.max()*1e3:.2f} mm -> frames right)")
    names = ["base v_x", "base v_y", "base v_z", "base w_x(right)", "base w_y(NOSE)", "base w_z", "q1", "q2", "q3", "q4"]
    print("   row               residual amp   fitted-by-claw-wrench amp   unexplained amp")
    for i, nm in enumerate(names):
        a, af, au = abs(lockin(tt, w[:, i], f0)), abs(lockin(tt, wfit[:, i], f0)), abs(lockin(tt, w[:, i] - wfit[:, i], f0))
        print(f"   {nm:16s}  {a:8.3f}        {af:8.3f}                   {au:8.3f}")
    for lab, V3 in (("claw FORCE  (N)  ", Fw), ("claw MOMENT (N.m)", Mw)):
        z = [abs(lockin(tt, V3 @ a_, f0)) for a_ in (nrm, nose, np.array([0, 0, 1.0]))]
        print(f"   {lab} at the sway:  across(radial) {z[0]:.3f}   along(nose,tangential) {z[1]:.3f}   vertical {z[2]:.3f}")
