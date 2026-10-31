#!/usr/bin/env python3
"""Contact phase of a valve flight on a 50 Hz grid: how the valve's angle, the claw heading (planner FK),
the arm yaw q1 / wrist roll q4, the vehicle's yaw and the CoM error along the nose / sideways move
together.  Prints amplitude, dominant frequency, and cross-correlations (with lag) between them."""
import sys
import numpy as np
from vt_common import Bag
from valve_axis import obj, step_times
tag = sys.argv[1] if len(sys.argv) > 1 else "145526"
b = Bag(tag); st = step_times(b)
ta_ = float(sys.argv[2]) if len(sys.argv) > 2 else st["push"]
tb_ = float(sys.argv[3]) if len(sys.argv) > 3 else st.get("abort", st["push"] + 20) - 0.3
g = np.arange(ta_, tb_, 0.02)
to, R, p = obj(b); vy = np.degrees(np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0]))); th_v = np.interp(g, to, vy); th_v -= th_v[0]
tw = b.t("wb"); D = b.f("wb", "data"); ok = np.isfinite(D[:, 50]) & (D[:, 0] == 1); tw, D = tw[ok], D[ok]
col = lambda i: np.interp(g, tw, D[:, i])
q = np.degrees(np.stack([col(5+i) for i in range(4)], 1))
ta = b.t("vatt"); qa = b.f("vatt", "q")
yaw = np.degrees(np.unwrap(np.pi/2 - np.arctan2(2*(qa[:, 0]*qa[:, 3]+qa[:, 1]*qa[:, 2]), 1-2*(qa[:, 2]**2+qa[:, 3]**2))))
psi = np.interp(g, ta, yaw); psi_r = np.radians(psi)
nose = np.stack([np.cos(psi_r), np.sin(psi_r)], 1); side = np.stack([-np.sin(psi_r), np.cos(psi_r)], 1)
ec = np.stack([col(48+i) - col(45+i) for i in range(3)], 1)
e_f = np.sum(ec[:, :2]*nose, 1)*1e3; e_s = np.sum(ec[:, :2]*side, 1)*1e3
# planned turn angle: azimuth of the streamed claw reference about the planner's axis
ax = None
for t_, s in b.edges("rosout", "msg"):
    if "axis (" in s and "keep-outs placed" in s:
        a = s.split("axis (")[1].split(")")[0].split(","); ax = np.array([float(a[0]), float(a[1])])
tr = b.t("wbref"); red = np.stack([b.f("wbref", f"r_ed.{c}") for c in "xy"], 1)
az = np.degrees(np.unwrap(np.arctan2(red[:, 1]-ax[1], red[:, 0]-ax[0]))); th_p = np.interp(g, tr, az); th_p -= th_p[0]
# claw heading (planner FK) if present
eh = None
if b.has("ee_heading"):
    th_ = b.t("ee_heading"); fl = [k for k in b.z.files if k.startswith("ee_heading__")]
    v = b.f("ee_heading", "data") if "ee_heading__data" in b.z.files else None
    if v is not None:
        v = np.degrees(np.unwrap(np.radians(np.asarray(v, float).reshape(len(th_), -1)[:, 0])))
        eh = np.interp(g, th_, v); eh -= eh[0]
yaw_err = psi - psi[0] - th_p
S = {"valve-plan": th_v - th_p, "q1": q[:, 0] - q[0, 0], "q4": q[:, 3] - q[0, 3], "q2": q[:, 1] - q[0, 1], "q3": q[:, 2] - q[0, 2],
     "yaw-plan": yaw_err, "e_fwd": e_f, "e_side": e_s}
if eh is not None: S["clawhdg-plan"] = eh - th_p
def bp(x, lo=0.2, hi=1.2):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 0.02); X[(f < lo) | (f > hi)] = 0; return np.fft.irfft(X, len(x)), f
print(f"=== {tag}  window {ta_:.2f}-{tb_:.2f} s  axis {ax}")
for k, x in S.items():
    xb, f = bp(x); P = np.abs(np.fft.rfft(x - x.mean()))**2; m = (f > 0.15) & (f < 2)
    print(f"  {k:12s} rms {np.std(x):6.1f}  band 0.2-1.2 Hz rms {np.std(xb):6.1f}  peak f {f[m][np.argmax(P[m])]:.2f} Hz  range [{x.min():+.1f}, {x.max():+.1f}]")
def xc(a, b_, maxlag=1.5):
    a = bp(a)[0]; b_ = bp(b_)[0]; L = int(maxlag/0.02); best = (0, 0)
    for k in range(-L, L+1):
        if k >= 0: c = np.corrcoef(a[k:], b_[:len(b_)-k])[0, 1]
        else: c = np.corrcoef(a[:k], b_[-k:])[0, 1]
        if abs(c) > abs(best[0]): best = (c, k*0.02)
    c0 = np.corrcoef(a, b_)[0, 1]
    return c0, best
pairs = [("valve-plan", "e_fwd"), ("valve-plan", "q1"), ("valve-plan", "q4"), ("valve-plan", "yaw-plan"), ("q1", "e_side"), ("q4", "e_side"),
         ("e_fwd", "e_side"), ("q1", "q4"), ("q3", "e_fwd"), ("q2", "q3")]
if eh is not None: pairs += [("valve-plan", "clawhdg-plan")]
print("  band-passed correlations: corr at lag 0 | best corr at lag (s, + = second LAGS first)")
for a, c in pairs:
    c0, (cb, lag) = xc(S[a], S[c])
    sl = np.polyfit(bp(S[c])[0], bp(S[a])[0], 1)[0]
    print(f"   {a:12s} vs {c:12s}  {c0:+.2f} | {cb:+.2f} @ {lag:+.2f} s   slope {a}/{c} {sl:+.3f}")
np.savez(f"../npz/coupling_{tag}.npz", g=g, **{k.replace('-', '_'): v for k, v in S.items()}, th_v=th_v, th_p=th_p)
