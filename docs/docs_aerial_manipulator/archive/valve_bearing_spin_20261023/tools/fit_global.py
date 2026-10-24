#!/usr/bin/env python3
"""Global fit of the free coast-downs of the valve rotor.

Dynamics per segment (sign s of the spin):   dw/dt = -s*(a + c*|w| + d*w^2) - g*sin(theta - psi)
    a = Coulomb torque / I, c = viscous / I, d = quadratic (air) drag / I,
    g = gravity torque amplitude / I from the bearing axis' tilt (psi = its phase).
Measurement:  theta_meas = theta + sum_k (A_k cos k*theta + B_k sin k*theta)
    (the mocap angle error that depends on orientation: the yaw and the circle angle of the
     pivot disagree by the same +-3 deg pattern on every spin).
Per segment: theta0, w0. Robust (soft-l1) least squares.

Usage: fit_global.py kin.npz [--model a|ac|ad|acd] [--grav 0|1] [--harm K] [--start-after S]
       [--drop t1,t2] [--boot N]
"""
import sys, argparse, numpy as np
from scipy.optimize import least_squares
sys.path.insert(0, __import__("os").path.dirname(__file__))
from segments import find

def simulate(theta0, w0, s, tq, a, c, d, g, psi, dt=0.002):
    T = tq[-1]; n = int(np.ceil(T / dt)) + 1
    th = np.empty(n); th[0] = theta0; w = w0
    def f(x, v):
        return -s * (a + c * abs(v) + d * v * v) - g * np.sin(x - psi)
    x = theta0
    for i in range(1, n):
        k1v = f(x, w); k1x = w
        k2v = f(x + 0.5 * dt * k1x, w + 0.5 * dt * k1v); k2x = w + 0.5 * dt * k1v
        k3v = f(x + 0.5 * dt * k2x, w + 0.5 * dt * k2v); k3x = w + 0.5 * dt * k2v
        k4v = f(x + dt * k3x, w + dt * k3v); k4x = w + dt * k3v
        x += dt / 6 * (k1x + 2 * k2x + 2 * k3x + k4x); w += dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        th[i] = x
        if s * w <= 0:          # stopped: hold (the fit window ends before the stop)
            th[i:] = x; break
    return np.interp(tq, np.arange(n) * dt, th)

def build(t, th, segs):
    data = []
    for t0, t1, s, tp in segs:
        m = (t >= t0) & (t <= t1)
        tau = t[m] - t0; y = th[m]
        c = np.polyfit(tau, y, 2)
        data.append(dict(tau=tau, y=y, s=s, tp=tp, th0=c[2], w0=c[1]))
    return data

def unpack(p, model, grav, K, nseg):
    i = 0; a = p[i]; i += 1
    c = p[i] if "c" in model else 0.0; i += "c" in model
    d = p[i] if "d" in model else 0.0; i += "d" in model
    if grav: g = np.hypot(p[i], p[i + 1]); psi = np.arctan2(p[i + 1], p[i]); i += 2
    else: g, psi = 0.0, 0.0
    F = p[i:i + 2 * K]; i += 2 * K
    S = p[i:].reshape(nseg, 2)
    return a, c, d, g, psi, F, S

def resid(p, data, model, grav, K):
    a, c, d, g, psi, F, S = unpack(p, model, grav, K, len(data))
    out = []
    for (th0, w0), D in zip(S, data):
        x = simulate(th0, w0, D["s"], D["tau"], a, c, d, g, psi)
        e = sum(F[2 * k] * np.cos((k + 1) * x) + F[2 * k + 1] * np.sin((k + 1) * x) for k in range(K))
        out.append(x + e - D["y"])
    return np.concatenate(out)

def fit(data, model="a", grav=True, K=4):
    p0 = [3.3] + [0.0] * (("c" in model) + ("d" in model)) + ([0.0, 0.0] if grav else []) + [0.0] * (2 * K)
    for D in data: p0 += [D["th0"], D["w0"]]
    lb = [0] + [0] * (("c" in model) + ("d" in model)) + [-np.inf] * ((2 if grav else 0) + 2 * K + 2 * len(data))
    r = least_squares(resid, p0, args=(data, model, grav, K), loss="soft_l1", f_scale=np.radians(1.5),
                      bounds=(lb, [np.inf] * len(p0)), x_scale="jac")
    return r

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("kin"); ap.add_argument("--model", default="a"); ap.add_argument("--grav", type=int, default=1)
    ap.add_argument("--harm", type=int, default=4); ap.add_argument("--start-after", type=float, default=0.12)
    ap.add_argument("--drop", default=""); ap.add_argument("--boot", type=int, default=0)
    ap.add_argument("--extra", default="", help="extra segments t0:t1:sign,...")
    A = ap.parse_args()
    k = np.load(A.kin); t, th, w = k["t"], k["th"], k["w"]
    segs = find(t, w, start_after=A.start_after)
    drop = [float(x) for x in A.drop.split(",") if x]
    segs = [s for s in segs if not any(abs(s[3] - x) < 0.3 for x in drop)]
    for e in [x for x in A.extra.split(",") if x]:
        t0, t1, sg = e.split(":"); segs.append((float(t0), float(t1), float(sg), float(t0)))
    data = build(t, th, segs)
    r = fit(data, A.model, bool(A.grav), A.harm)
    a, c, d, g, psi, F, S = unpack(r.x, A.model, bool(A.grav), A.harm, len(data))
    rr = resid(r.x, data, A.model, bool(A.grav), A.harm)
    print(f"model {A.model} grav {A.grav} K {A.harm} start+{A.start_after}s  segments {len(data)}  samples {len(rr)}")
    print(f"  a {a:.4f} rad/s^2   c {c:.4f} 1/s   d {d:.5f} 1/rad   g {g:.4f} rad/s^2  psi {np.degrees(psi):.1f} deg")
    print(f"  resid rms {np.degrees(np.sqrt(np.mean(rr**2))):.3f} deg   median|r| {np.degrees(np.median(np.abs(rr))):.3f} deg   cost {r.cost:.4f}")
    print("  meas. Fourier (deg):", np.round(np.degrees(F), 2))
    for D, (t0_, w0_) in zip(data, S):
        print(f"    seg {D['tp']:7.2f}  w0 {np.degrees(w0_):6.0f} deg/s   T {D['tau'][-1]:.2f} s")
    if A.boot:
        rng = np.random.default_rng(0); B = []
        for b in range(A.boot):
            idx = rng.integers(0, len(data), len(data))
            rb = fit([data[i] for i in idx], A.model, bool(A.grav), A.harm)
            B.append(unpack(rb.x, A.model, bool(A.grav), A.harm, len(idx))[:4])
        B = np.array(B)
        for j, nme in enumerate("acdg"):
            print(f"  boot {nme}: median {np.median(B[:, j]):.4f}  5-95% [{np.percentile(B[:, j], 5):.4f}, {np.percentile(B[:, j], 95):.4f}]")
        np.save(f"boot_{A.model}_g{A.grav}.npy", B)
    np.save(f"fit_{A.model}_g{A.grav}.npy", r.x)
