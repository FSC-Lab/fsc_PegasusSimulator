#!/usr/bin/env python3
"""Friction law of the 10-27 valve from its coast-downs (coasts_<key>_<delay>.npz).

Model during a coast in direction s:   th'' = -s (a + c |w|) + A sin(th) + B cos(th)
  a = Coulomb / I, c = viscous / I [1/s], A, B = gravity on the 2.6 deg tilted axis (/ I).
  The 1/rev term (A, B) is fitted for the record only: it does not predict a left-out coast
  better than Coulomb alone (tools/onerev.py), so the bootstrap uses a and a + c.
Two independent fits, bootstrap over coasts (the coast-to-coast spread, not the sample noise):
  E: energy, linear:  w_i^2/2 = a D_i + c J_i - A (cos th_i - cos th_s) - B (sin th_s - sin th_i)
  O: trajectory: integrate the ODE per coast (own th0, w0), fit the ANGLE samples, sticking at w = 0
Usage: fit.py coasts_th_q_25.npz [nboot]
"""
import sys, numpy as np
from scipy.optimize import least_squares
C = np.load(sys.argv[1]); NB = int(sys.argv[2]) if len(sys.argv) > 2 else 400
t, th, w, segs = C["t"], C["th"], C["w"], C["segs"]
segs = [(int(a), int(b), float(s), float(r)) for a, b, s, r in segs]

def energy_rows(seg):
    i0, ir, s, ths = seg
    x = s * (th[i0:ir + 1] - th[i0]); v = s * w[i0:ir + 1]; D = s * (ths - th[i0:ir + 1])
    v = np.clip(v, 0, None); D = np.clip(D, 0, None)
    # J(D) = int_0^D w dD' measured from rest: integrate w over travel from the end
    xr = x[::-1]; vr = v[::-1]
    Jr = np.concatenate([[0], np.cumsum(0.5 * (vr[1:] + vr[:-1]) * -np.diff(xr))])
    J = Jr[::-1]
    k = D > np.radians(2.0)                                  # drop the last 2 deg: angle noise dominates D
    g1 = -(np.cos(th[i0:ir + 1]) - np.cos(ths)); g2 = -(np.sin(ths) - np.sin(th[i0:ir + 1]))
    return np.c_[D, J, g1, g2][k], 0.5 * v[k] ** 2

E = [energy_rows(sg) for sg in segs]
def fitE(idx, cols):
    X = np.concatenate([E[i][0][:, cols] for i in idx]); y = np.concatenate([E[i][1] for i in idx])
    return np.linalg.lstsq(X, y, rcond=None)[0]

def fitO(idx, model):
    """Trajectory fit, all coasts integrated in parallel (fixed 0.5 ms step, sticking at w = 0).
    model: 'a', 'ac', 'acg' (gravity A, B)."""
    ng = {"a": 1, "ac": 2, "acg": 4}[model]
    S = [segs[i] for i in idx]; n = len(S)
    t0 = np.array([t[i0] for i0, _, _, _ in S]); sg = np.array([s for _, _, s, _ in S])
    ends = [min(len(t), np.searchsorted(t, t[ir] + 0.10)) for _, ir, _, _ in S]
    obs_t = [t[i0:e] - t[i0] for (i0, _, _, _), e in zip(S, ends)]
    obs = [th[i0:e] for (i0, _, _, _), e in zip(S, ends)]
    h = 0.0005; L = max(o[-1] for o in obs_t); N = int(np.ceil(L / h)) + 2; grid = np.arange(N) * h
    def traj(p):
        a, c, A, B = (list(p[:ng]) + [0, 0, 0])[:4]
        x = p[ng::2].copy(); v = p[ng + 1::2].copy(); X = np.empty((N, n)); live = np.ones(n, bool)
        for k in range(N):
            X[k] = x
            acc = -sg * (a + c * np.abs(v)) + A * np.sin(x) + B * np.cos(x)
            vn = v + acc * h
            stop = live & (sg * vn <= 0)
            x = np.where(live, x + 0.5 * (v + np.where(stop, 0.0, vn)) * h, x)
            v = np.where(live & ~stop, vn, 0.0); live &= ~stop
        return X
    def res(p):
        X = traj(p)
        return np.concatenate([np.interp(ot, grid, X[:, j]) - o for j, (ot, o) in enumerate(zip(obs_t, obs))])
    p0 = np.array([30.0, 0.0, 0.0, 0.0][:ng] + sum([[th[i0], w[i0]] for i0, _, _, _ in S], []))
    sizes = [len(o) for o in obs]; rowc = np.repeat(np.arange(n), sizes)
    sp = np.zeros((len(rowc), ng + 2 * n), bool); sp[:, :ng] = True
    sp[np.arange(len(rowc)), ng + 2 * rowc] = True; sp[np.arange(len(rowc)), ng + 2 * rowc + 1] = True
    lb = [0, -50, -50, -50][:ng] + [-np.inf] * (2 * n); ub = [500, 50, 50, 50][:ng] + [np.inf] * (2 * n)
    r = least_squares(res, p0, bounds=(lb, ub), loss="soft_l1", f_scale=np.radians(0.5),
                      jac_sparsity=sp, x_scale="jac")
    return r.x[:ng], np.degrees(np.sqrt(np.mean(r.fun ** 2)))

idx = list(range(len(segs)))
print(f"{len(segs)} coasts")
for cols, lab in [([0], "a"), ([0, 1], "a c"), ([0, 2, 3], "a A B"), ([0, 1, 2, 3], "a c A B")]:
    print(f"  E fit [{lab}]: {np.round(fitE(idx, cols), 3)}")
for m in ("a", "ac", "acg"):
    p, rms = fitO(idx, m); print(f"  O fit [{m}]: {np.round(p, 3)}  angle rms {rms:.2f} deg")
rng = np.random.default_rng(1); BE = []; BO = []
for b in range(NB):
    ii = list(rng.integers(0, len(segs), len(segs)))
    BE.append(np.r_[fitE(ii, [0]), fitE(ii, [0, 1])])
    if b < NB // 4:
        BO.append(np.r_[fitO(ii, "a")[0], fitO(ii, "ac")[0]])
BE = np.array(BE); BO = np.array(BO)
pr = lambda x: f"{np.median(x):7.3f} [{np.percentile(x, 2.5):7.3f}, {np.percentile(x, 97.5):7.3f}]"
print(f"bootstrap over coasts (median [95 %]), E {len(BE)} / O {len(BO)} resamples:")
print(f"  Coulomb only      a:  E {pr(BE[:, 0])}   O {pr(BO[:, 0])}")
print(f"  Coulomb+viscous   a:  E {pr(BE[:, 1])}   O {pr(BO[:, 1])}")
print(f"                    c:  E {pr(BE[:, 2])}   O {pr(BO[:, 2])}")
np.savez(sys.argv[1].replace("coasts_", "fit_"), BE=BE, BO=BO)
