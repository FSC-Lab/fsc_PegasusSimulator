#!/usr/bin/env python3
"""Does the bearing axis' tilt explain the spin-to-spin scatter of the mean deceleration?

Energy from fit start to rest (all coasts here spin +):  a_seg*D = a*D + g*[cos(phi0-b) - cos(phis-b)]
with phi = the CoM's azimuth (taken = the marker centroid's circle angle, the markers sit on the beam)
and b = the azimuth of the rotation plane's lowest point. Linear in (a, g cos b, g sin b).
Prediction from geometry: tilt 0.55 deg toward b = -163 deg (the marker circle's plane fit);
g = g0 * r_cm * sin(tilt) / k^2 with the CAD r_cm 0.0851 m and k^2 = I/m = 0.003058/0.24278 m^2.
"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
k = np.load("kin.npz"); t, th = k["t"], k["th"]
S = np.load("stopping.npz"); D, sid, tt, info = S["D"], S["sid"], S["tt"], S["info"]
F = np.load("decel_fit.npz"); aseg_all = F["aseg"]
off = np.radians(265.24)               # marker-centroid direction in the body frame (median phi - yaw)
rows = []
for n in np.unique(sid):
    m = sid == n
    t0, ts = tt[m].min(), tt[m].max()
    th0 = np.interp(t0, t, th); ths = np.interp(ts, t, th)
    Dn = D[m].max()
    rows.append((n, Dn, th0 + off, ths + off))
rows = np.array(rows)
# the per-coast mean decel as decel_fit defines it is a slope; use the energy form directly instead:
w0sq = []
for n in rows[:, 0]:
    m = sid == n; o = np.argsort(D[m]); d = D[m][o]; w = S["w"][m][o]
    kk = d > np.radians(15); w0sq.append((d[kk] @ (0.5 * w[kk] ** 2)) / (d[kk] @ d[kk]))
aseg = np.array(w0sq)
med = np.median(aseg); mad = 1.4826 * np.median(np.abs(aseg - med)); keep = np.abs(aseg - med) <= 3 * mad
R = rows[keep]; A_ = aseg[keep]
Dn = R[:, 1]; p0 = R[:, 2]; ps = R[:, 3]
X = np.c_[Dn, np.cos(p0) - np.cos(ps), np.sin(p0) - np.sin(ps)]
y = A_ * Dn
c, res, *_ = np.linalg.lstsq(X, y, rcond=None)
a, gc, gs = c; g = np.hypot(gc, gs); b = np.degrees(np.arctan2(gs, gc))
pred = 9.81 * 0.0851 * np.sin(np.radians(0.55)) / (0.003058 / 0.24278)
r0 = A_ - A_.mean(); r1 = y / Dn - X @ c / Dn
print(f"coasts {keep.sum()}: mean decel {A_.mean():.3f} +- {A_.std(ddof=1):.3f} (spread)")
print(f"with the tilt term: a = {a:.3f} rad/s^2, g = {g:.3f} rad/s^2 at b = {b:.0f} deg; residual spread {r1.std(ddof=3):.3f}")
print(f"geometry predicts g = {pred:.3f} rad/s^2 at b = -163 deg")
for a_fix in (pred,):
    Xg = np.cos(p0 - np.radians(-163)) - np.cos(ps - np.radians(-163))
    a2 = ((A_ * Dn - a_fix * Xg) @ Dn) / (Dn @ Dn)
    print(f"tilt term fixed at the geometric prediction: a = {a2:.3f}, residual spread {np.std(A_ - a_fix * Xg / Dn - a2, ddof=1):.3f}")
