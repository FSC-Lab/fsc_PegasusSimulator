#!/usr/bin/env python3
"""Flight-wide windows (0.5 s) of the whole-body law's own debug stream: CoM error split along the nose /
sideways / vertical (body heading frame), attitude error (model frame) roll/pitch/yaw, collective, body
torque (FLU), arm torques, motor commands (min / max), allocator saturation count, and the joints."""
import sys
import numpy as np
from vt_common import Bag
tag = sys.argv[1] if len(sys.argv) > 1 else "145526"
t0, t1, W = (float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (28.0, 46.4, 0.5)
b = Bag(tag)
tw = b.t("wb"); D = b.f("wb", "data")
ok = np.isfinite(D[:, 50]) & (D[:, 0] == 1); tw, D = tw[ok], D[ok]
ta = b.t("vatt"); qa = b.f("vatt", "q")
yaw_ned = np.arctan2(2*(qa[:, 0]*qa[:, 3]+qa[:, 1]*qa[:, 2]), 1-2*(qa[:, 2]**2+qa[:, 3]**2))
psi = np.interp(tw, ta, np.unwrap(np.pi/2 - yaw_ned))     # ENU heading of the nose
nose = np.stack([np.cos(psi), np.sin(psi)], 1); side = np.stack([-np.sin(psi), np.cos(psi)], 1)
ec = D[:, 48:51] - D[:, 45:48]
e_f = np.sum(ec[:, :2]*nose, 1)*1e3; e_s = np.sum(ec[:, :2]*side, 1)*1e3; e_z = ec[:, 2]*1e3
eR = np.degrees(np.arcsin(np.clip(D[:, 28:31], -1, 1)))
mot = D[:, 41:45]; tau = D[:, 13:17]; tb = D[:, 21:24]; u1 = D[:, 17]; nsat = D[:, 51]; q = np.degrees(D[:, 5:9])
tbt = b.t("batt"); vb = b.f("batt", "voltage_v")
print(f"=== {tag}   cols: CoM err fwd/side/z sd|range mm | eR roll/pitch/yaw sd deg | u1 N | tau_b FLU x/y/z mean+-sd | |tau_arm|max | motors min/max | sat% | q1..q4 mean | V")
for a in np.arange(t0, t1, W):
    w = (tw >= a) & (tw < a + W)
    if w.sum() < 5: continue
    r = lambda x: f"{np.mean(x[w]):+6.1f}({np.ptp(x[w]):4.0f})"
    s = f"{a:6.1f} | {r(e_f)} {r(e_s)} {r(e_z)} | " + " ".join(f"{np.mean(eR[w,i]):+5.1f}({np.ptp(eR[w,i]):4.1f})" for i in range(3))
    s += f" | {np.mean(u1[w]):5.1f} | " + " ".join(f"{np.mean(tb[w,i]):+5.2f}±{np.std(tb[w,i]):.2f}" for i in range(3))
    s += f" | " + " ".join(f"{np.abs(tau[w,i]).max():4.2f}" for i in range(4))
    s += f" | {mot[w].min():.2f}/{mot[w].max():.2f} | {100*np.mean(nsat[w] > 0):3.0f} | " + " ".join(f"{np.mean(q[w,i]):+5.1f}" for i in range(4))
    s += f" | {np.interp(a, tbt, vb):.2f}"
    print(s)
