#!/usr/bin/env python3
"""192853, the yaw channel continued: in the vehicle's own axes (x nose, y left), the claw's task error
(wb [24..26], world), the raw task force F_hat (wb [97..99], world, low-passed 0.5 s), the arm yaw joint q1 (measured
and its reference), its torque (wb [13]), and the fin's position relative to the claw (planner arrival_error [35]
across, [36] ahead of the bracket). The yaw moment a sideways claw force makes about the CoM is x_claw * F_y."""
import sys
import numpy as np
from vt_common import Bag, yaw_deg
t0, t1 = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) > 2 else (32.4, 34.4)
b = Bag("192853")
tw = b.t("wb"); W = np.asarray(b.f("wb", "data"), float)
tv = b.t("vrpn"); yv = np.unwrap(np.radians(yaw_deg(*[b.f("vrpn", f"pose.orientation.{c}") for c in "wxyz"])))
P = np.stack([b.f("vrpn", f"pose.position.{c}") for c in "xyz"], 1)
tj = b.t("js"); J = np.asarray(b.f("js", "position"), float)   # [joint2, joint3, joint1, joint4, gl, gr]
ta = b.t("pl_arr"); A = np.asarray(b.f("pl_arr", "data"), float)
to = b.t("obj"); yo = np.degrees(np.unwrap(np.radians(yaw_deg(*[b.f("obj", f"pose.orientation.{c}") for c in "wxyz"]))))
y_get = np.median(yo[(to > 9) & (to < 11)])
# 0.5 s low-pass of the raw task force (it is a 250 Hz momentum difference: noisy)
dt = np.median(np.diff(tw)); a = dt / 0.5
F = np.nan_to_num(W[:, 97:100]); Fl = np.zeros_like(F)
for i in range(1, len(F)):
    Fl[i] = Fl[i - 1] + a * (F[i] - Fl[i - 1])
print("   t   | yaw  valve | claw err nose left up [mm] | Fhat(0.5s) nose left up [N] | q1 q1_d [deg] tau_j1 | "
      "fin across ahead [mm] | base v nose left [mm/s]")
vp = np.gradient(P, tv, axis=0)
for t in np.arange(t0, t1, 0.1):
    i = min(np.searchsorted(tw, t), len(tw) - 1); w = W[i]
    iv = min(np.searchsorted(tv, t), len(tv) - 1); ps = yv[iv]
    nose = np.array([np.cos(ps), np.sin(ps), 0.0]); left = np.array([-np.sin(ps), np.cos(ps), 0.0])
    e = w[24:27] * 1e3; f = Fl[i]
    ij = min(np.searchsorted(tj, t), len(tj) - 1); q1 = np.degrees(J[ij, 2])
    k = min(np.searchsorted(ta, t), len(ta) - 1); r = A[k]
    io = min(np.searchsorted(to, t), len(to) - 1)
    v = np.mean(vp[max(iv - 6, 0):iv + 6], axis=0) * 1e3
    print(f" {t:5.1f} | {np.degrees(ps):+6.1f} {yo[io]-y_get:+5.1f} | {e@nose:+6.1f} {e@left:+6.1f} {e[2]:+6.1f} | "
          f"{f@nose:+5.2f} {f@left:+5.2f} {f[2]:+5.2f} | {q1:+5.1f} {np.degrees(w[9]):+5.1f} {w[13]:+.2f} | "
          f"{r[35]*1e3:+6.1f} {r[36]*1e3:+6.1f} | {v@nose:+5.0f} {v@left:+5.0f}")
