#!/usr/bin/env python3
"""Free coast-downs of the 10-27 valve (coarse pad on the hub) -> coasts.npz + a table.

A flick: |w| peak >= 150 deg/s. The coast runs from DELAY after the peak (the hand is off) to the
first rest (angle range < 0.4 deg over 0.15 s). The pad stops the rotor in 0.1-0.3 s, so the
coasts are short (10-35 samples at 120 Hz); w comes from a +-0.03 s local quadratic.
Per coast: w0 (at the start), travel D to rest, time T to rest, and two model-free decelerations:
  a_D = w0^2 / (2 D)   (energy)      a_T = w0 / T   (impulse)
Pure Coulomb: a_D = a_T. The speed dependence (a viscous part) is fit.py's job.
Usage: coasts.py [angle key th_q|th_c] [DELAY s]
"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from kin import local_poly
key = sys.argv[1] if len(sys.argv) > 1 else "th_q"
DELAY = float(sys.argv[2]) if len(sys.argv) > 2 else 0.025
A = np.load("angle.npz"); t = A["t"]; th = A[key]
_, w, _ = local_poly(t, th, 0.03, 2)
W = np.abs(w)

def rest_at(i0, win=0.15, tol=np.radians(0.4), tmax=1.2):
    for i in range(i0, len(t)):
        if t[i] > t[i0] + tmax:
            return None
        j = np.searchsorted(t, t[i] + win)
        if j >= len(t):
            return None
        s = th[i:j]
        if np.ptp(s) < tol and j - i > 10:
            return i
    return None

rows = []; segs = []
i = 1
while i < len(t) - 1:
    if W[i] >= np.radians(150) and W[i] == W[max(0, i - 60):i + 60].max():
        s = np.sign(w[i]); i0 = np.searchsorted(t, t[i] + DELAY)
        ir = rest_at(i0)
        ok = ir is not None
        if ok:
            # no re-push: |w| must not rise by > 30 deg/s between start and rest
            ws = s * w[i0:ir + 1]; runmin = np.minimum.accumulate(ws)
            ok = (ws - runmin).max() < np.radians(30) and np.all(ws > -np.radians(20))
        if ok:
            th_rest = np.median(th[ir:np.searchsorted(t, t[ir] + 0.15)])
            D = s * (th_rest - th[i0]); T = t[ir] - t[i0]; w0 = s * w[i0]
            rows.append((t[i], s, np.degrees(th[i0]) % 360, np.degrees(w0), np.degrees(D), T,
                         w0 ** 2 / (2 * D), w0 / T, np.degrees(W[i])))
            segs.append((i0, ir, s, th_rest))
        i += 60
    else:
        i += 1
rows = np.array(rows)
print(f"angle {key}, delay {DELAY*1e3:.0f} ms: {len(rows)} coasts")
print("  t_peak dir  angle  w_pk  w0[deg/s]  D[deg]  T[s]   a_D   a_T [rad/s^2]")
for r in rows:
    print(f"  {r[0]:6.2f} {int(r[1]):+d}  {r[2]:5.0f}  {r[8]:4.0f}   {r[3]:5.0f}    {r[4]:5.1f}  {r[5]:.3f}  {r[6]:5.1f} {r[7]:5.1f}")
aD, aT = rows[:, 6], rows[:, 7]
print(f"a_D median {np.median(aD):.1f} mean {aD.mean():.1f} std {aD.std(ddof=1):.1f} | a_T median {np.median(aT):.1f} "
      f"mean {aT.mean():.1f} std {aT.std(ddof=1):.1f}  |  + dir a_D {aD[rows[:,1]>0].mean():.1f}  - dir {aD[rows[:,1]<0].mean():.1f}")
np.savez(f"coasts_{key}_{int(DELAY*1e3)}.npz", rows=rows, segs=np.array(segs), t=t, th=th, w=w)
