#!/usr/bin/env python3
"""Remaining-travel analysis of the coast-downs (model-light).

For each coast that ends at REST: theta_stop = the rest angle (median over 0.15 s of rest within 0.6 s of the coast window end,
measured to ~0.02 deg), D(t) = remaining travel to it. With friction torque tau(w),
I w dw/dtheta = -tau  =>  w^2 / 2 = int_0^D tau/I dD'.  Pure Coulomb: w^2 = 2 a D, a straight
line through the origin. Deceleration at a given speed = slope d(w^2/2)/dD.
Writes stopping.npz (per-sample D, w, segment id, rest info).
"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from segments import find
from kin import local_poly

def rest_after(t, th, t_from, t_max=0.6, win=0.15, tol=np.radians(1.0)):
    i0 = np.searchsorted(t, t_from - 0.3)
    for i in range(i0, len(t)):
        if t[i] > t_from + t_max: return None
        j = np.searchsorted(t, t[i] + win)
        if j >= len(t): return None
        seg = th[i:j]
        if seg.max() - seg.min() < tol and j - i > 10:
            return t[i], np.median(seg)
    return None

if __name__ == "__main__":
    k = np.load(sys.argv[1]); t, th = k["t"], k["th"]
    _, w, _ = local_poly(t, th, 0.10, 2)
    segs = find(t, k["w"])
    out = dict(D=[], w=[], sid=[], tt=[]); info = []
    for n, (t0, t1, s, tp) in enumerate(segs):
        r = rest_after(t, th, t1)
        if r is None:
            print(f"{tp:7.2f}: no rest -> skipped"); continue
        tr, ths = r
        m = (t >= t0) & (t <= tr)
        D = s * (ths - th[m]); W = s * w[m]
        out["D"] += list(D); out["w"] += list(W); out["sid"] += [n] * m.sum(); out["tt"] += list(t[m])
        info.append((tp, t0, tr, np.degrees(D[0]), np.degrees(W[0])))
        print(f"{tp:7.2f}: rest at {tr:7.2f} (theta {np.degrees(ths):8.1f}), travel {np.degrees(D[0]):5.0f} deg from w0 {np.degrees(W[0]):4.0f} deg/s")
    np.savez("stopping.npz", **{k_: np.array(v) for k_, v in out.items()}, info=np.array(info))
