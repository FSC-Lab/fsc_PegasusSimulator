#!/usr/bin/env python3
"""Friction law from the remaining-travel data (stopping.npz), linear and model-light.

Energy:  w^2/2 = a*D + c*J,   J(D) = int_0^D w dD'   (a = Coulomb/I, c = viscous/I).
Per coast: through-origin slope of w^2/2 vs D (= the coast's mean deceleration).
Bootstrap over coasts for the uncertainty (the spin-to-spin spread, not the sample noise).
"""
import sys, numpy as np
S = np.load(sys.argv[1] if len(sys.argv) > 1 else "stopping.npz")
D, w, sid, info = S["D"], S["w"], S["sid"], S["info"]
Dmin = np.radians(15)                       # skip the last 15 deg: the angle error (+-3 deg) dominates D there
ids = np.unique(sid)
rows = []
for n in ids:
    m = sid == n
    d, v = D[m], w[m]
    o = np.argsort(d); d, v = d[o], v[o]
    J = np.concatenate([[0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(d))])
    k = d > Dmin
    rows.append(dict(n=n, d=d[k], y=0.5 * v[k] ** 2, J=J[k], v=v[k]))
def fit_ac(R):
    A = np.concatenate([np.c_[r["d"], r["J"]] for r in R]); y = np.concatenate([r["y"] for r in R])
    return np.linalg.lstsq(A, y, rcond=None)[0]
def fit_a(R):
    d = np.concatenate([r["d"] for r in R]); y = np.concatenate([r["y"] for r in R])
    return (d @ y) / (d @ d)
print("coast  w0[deg/s]  travel[deg]  mean decel a_seg [rad/s^2]")
aseg = []
for r, inf in zip(rows, info):
    a = (r["d"] @ r["y"]) / (r["d"] @ r["d"]); aseg.append(a)
    print(f"{inf[0]:6.1f}  {inf[4]:6.0f}  {inf[3]:7.0f}   {a:.3f}")
aseg = np.array(aseg)
med = np.median(aseg); mad = 1.4826 * np.median(np.abs(aseg - med))
keep = np.abs(aseg - med) <= 3 * mad
for r, inf, a, k_ in zip(rows, info, aseg, keep):
    if not k_: print(f"  outlier dropped: coast {inf[0]:.1f} (a_seg {a:.2f} vs median {med:.2f}, 3 MAD = {3*mad:.2f})")
rows = [r for r, k_ in zip(rows, keep) if k_]; info = info[keep]; aseg = aseg[keep]
print(f"per-coast mean decel: mean {aseg.mean():.3f}  std {aseg.std(ddof=1):.3f}  SE {aseg.std(ddof=1)/np.sqrt(len(aseg)):.3f}  (n {len(aseg)})")
a1 = fit_a(rows); ac = fit_ac(rows)
print(f"pooled Coulomb-only a = {a1:.3f};  Coulomb+viscous a = {ac[0]:.3f}, c = {ac[1]:.3f} 1/s")
rng = np.random.default_rng(1); B1 = []; B2 = []
for _ in range(4000):
    R = [rows[i] for i in rng.integers(0, len(rows), len(rows))]
    B1.append(fit_a(R)); B2.append(fit_ac(R))
B1 = np.array(B1); B2 = np.array(B2)
p = lambda x: f"{np.median(x):.3f} [{np.percentile(x, 2.5):.3f}, {np.percentile(x, 97.5):.3f}]"
print("bootstrap over coasts (median [95%]):  Coulomb-only a", p(B1), "|  a", p(B2[:, 0]), " c", p(B2[:, 1]))
for wq in (0.42, 2.0, 6.0):
    q = B2[:, 0] + B2[:, 1] * wq
    print(f"  decel at {np.degrees(wq):4.0f} deg/s (a + c w): {p(q)}")
np.savez("decel_fit.npz", aseg=aseg, B1=B1, B2=B2, info=info)
