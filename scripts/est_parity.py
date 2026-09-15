#!/usr/bin/env python3
"""Tuning PARITY for the cascade L1's residual estimators (piece 3 of the estimator comparison,
2026-09-15): the knob of each kind is set so that the three answer the same question with the
same cost, measured on an estimator-alone harness at the gate's inner rate (200 Hz) and noise.

    ~/envs/isaac_env/bin/python3 scripts/est_parity.py            # the table at the node's defaults
    ~/envs/isaac_env/bin/python3 scripts/est_parity.py --solve    # solve the parity knobs

TWO PARITY RULES, one per channel, because the channels are not alike:
  * MOMENT observers: EQUAL NOISE FLOOR -- the raw sigma noise under 1 deg/s of rate noise is
    what C(s) has to strip before the actuators, so it is the cost being paid. The STESO at the
    flown L 20 sets the target (0.068 N.m on pitch, = 0.185 rad/s^2 on every axis: rate-driven).
    At that floor the LESO matches the STESO's lag/gain at the tank's 1.14 Hz (a second-order
    observer either way); the momentum observer -- first order -- lags ~2x (81 vs 44 ms). So the
    MDOB is carried in two configurations: equal floor (K ~11) and equal lag (K ~22, 1.8x noise).
  * FORCE twins: EQUAL READING LAG at the tank's mode. The STESO force twin's floor (0.059 N at
    L_F 3) is entirely its own discretisation chatter (1.1 L_F dt m = 0.065 N), not filtered
    sensor noise; the linear twins sit at 0.002-0.03 N at ANY sensible bandwidth, so matching
    floors there would push them to absurd bandwidths. The lag at 1.14 Hz is what the slosh
    predictor's measurement model (meas_lag) needs, so it is the fair quantity: the linear twins
    are set to the STESO's closed-loop reading lag (10-20 ms on the gate) and their lower noise
    is recorded as a finding, not hidden.
The harness: a rigid body with the assembled inertia (moment) / a point mass (force), the gate's
noise -- 1 deg/s on the rate; for the force twin the MODEL force built from a 0.1 deg noisy
attitude (|F| sin 0.1 deg ~ 0.068 N per axis on a 39 N thrust vector), calibrated so the STESO
force twin reproduces the gate's floor -- and a 1.14 Hz sinusoidal residual for lag/gain.
"""
import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import inner_dir                       # noqa: E402

_spec = importlib.util.spec_from_file_location("so3_steso", os.path.join(inner_dir(), "so3_steso.py"))
st = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(st)

J = np.array([0.265, 0.342, 0.529])          # the tank aircraft's inertia (so3.IXX/IYY/IZZ with the tank folded)
M_KG = 3.9516
DT = 1.0 / 200.0                             # the gate's inner tick
F_TANK = 1.14                                # the 100 mm tank's mode [Hz]
NOISE_RATE_DPS = 1.0
NOISE_F = 0.068                              # model-force noise [N] from 0.1 deg of attitude on 39 N


def make(kind, knob, channel):
    Jv = J if channel == "M" else np.array([M_KG] * 3)
    kw = {"steso": dict(L=knob, mode="rate", eps="auto"), "leso": dict(wo=knob, mode="rate"), "mdob": dict(K=knob)}[kind]
    return st.make_estimator(kind, Jv, add_gyro=False, **kw), Jv


def run(kind, knob, channel, T=20.0, dist=None, noise=True, seed=0):
    """Returns (estimate trace in torque/force units, truth)."""
    rng = np.random.default_rng(seed)
    e, Jv = make(kind, knob, channel); Jm = np.diag(Jv)
    R = np.eye(3); w = np.zeros(3); n = int(T / DT); out = np.zeros((n, 3)); tru = np.zeros((n, 3))
    for k in range(n):
        t = k * DT
        M_cmd = np.array([0.2 * math.sin(math.pi * t), 0.1 * math.cos(2.0 * t), 0.05 * math.sin(t)]) if channel == "M" else np.zeros(3)
        M_model = M_cmd - (np.cross(w, Jm @ w) if channel == "M" else 0.0)
        d = dist(t) if dist else np.zeros(3)
        w_dot = np.linalg.solve(Jm, M_model + d)
        if channel == "M":
            wm = w + (rng.normal(0.0, math.radians(NOISE_RATE_DPS), 3) if noise else 0.0); Mm = M_model
        else:
            wm = w; Mm = M_model + (rng.normal(0.0, NOISE_F, 3) if noise else 0.0)
        e.step(R, Mm, DT, omega=wm)
        w = w + w_dot * DT
        out[k] = Jm @ e.sigma; tru[k] = d
    return out, tru


def floor(kind, knob, channel, axis=None):
    out, _ = run(kind, knob, channel)
    sd = out[2000:].std(axis=0)                      # last 10 s, no residual: pure noise floor
    return sd if axis is None else sd[axis]


def lag_gain(kind, knob, channel, f=F_TANK, amp=0.5):
    """Phase lag [s] and amplitude ratio of the reading vs a sinusoidal residual at f Hz, noise on."""
    ax = 1 if channel == "M" else 0
    def dist(t):
        d = np.zeros(3); d[ax] = amp * math.sin(2 * math.pi * f * t); return d
    out, _ = run(kind, knob, channel, dist=dist)
    t = np.arange(len(out)) * DT; w = t > 10.0
    A = np.column_stack([np.sin(2 * np.pi * f * t[w]), np.cos(2 * np.pi * f * t[w])])
    a, b = np.linalg.lstsq(A, out[w, ax], rcond=None)[0]
    return -math.atan2(b, a) / (2 * math.pi * f), math.hypot(a, b) / amp


def solve(fn, target, lo, hi, iters=18):
    """Bisection on log(knob) for a monotone fn(knob) -> value."""
    flo, fhi = fn(lo), fn(hi)
    inc = fhi > flo
    for _ in range(iters):
        mid = math.sqrt(lo * hi); fm = fn(mid)
        if (fm < target) == inc:
            lo = mid
        else:
            hi = mid
    return math.sqrt(lo * hi)


def node_defaults():
    """The knobs the cascade variant carries (import the node by path, no ROS, --est unset)."""
    sys.argv = [sys.argv[0]]
    p = os.path.join(inner_dir(), "nmpc_cascade_steso_node.py")
    sys.path.insert(0, os.path.dirname(os.path.dirname(p)))
    spec = importlib.util.spec_from_file_location("node", p); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    L = m.STESO_L if isinstance(m.STESO_L, (int, float)) else m.STESO_L[1]
    wo = m.LESO_WO if isinstance(m.LESO_WO, (int, float)) else m.LESO_WO[1]
    K = m.MDOB_K if isinstance(m.MDOB_K, (int, float)) else m.MDOB_K[1]
    return {"steso": (L, m.STESO_LF), "leso": (wo, m.LESO_WOF), "mdob": (K, m.MDOB_KF)}, getattr(m, "PRED_LAG_BY_KIND", None)


def table(knobs):
    print(f"{'kind':6s} {'moment knob':>11s} {'floor pitch [N.m]':>18s} {'lag@1.14Hz':>11s} {'gain':>6s}   {'force knob':>10s} {'floor x [N]':>11s} {'lag@1.14Hz':>11s} {'gain':>6s}")
    rows = {}
    for kind, (km, kf) in knobs.items():
        fm = floor(kind, km, "M", 1); lm, gm = lag_gain(kind, km, "M")
        ff = floor(kind, kf, "F", 0); lf, gf = lag_gain(kind, kf, "F")
        rows[kind] = dict(floor_M=fm, lag_M=lm, gain_M=gm, floor_F=ff, lag_F=lf, gain_F=gf)
        print(f"{kind:6s} {km:11.1f} {fm:18.4f} {1e3 * lm:9.1f} ms {gm:6.3f}   {kf:10.1f} {ff:11.4f} {1e3 * lf:9.1f} ms {gf:6.3f}")
    return rows


if __name__ == "__main__":
    if "--solve" in sys.argv:
        tM = floor("steso", 20.0, "M", 1)
        print(f"STESO L=20 pitch floor {tM:.4f} N.m -> equal-floor knobs:")
        wo = solve(lambda k: floor("leso", k, "M", 1), tM, 5.0, 200.0)
        K = solve(lambda k: floor("mdob", k, "M", 1), tM, 2.0, 100.0)
        print(f"  leso wo = {wo:.1f} (floor {floor('leso', wo, 'M', 1):.4f}), mdob K = {K:.1f} (floor {floor('mdob', K, 'M', 1):.4f})")
        lS, _ = lag_gain("steso", 20.0, "M")
        K2 = solve(lambda k: -lag_gain("mdob", k, "M")[0], -lS, 2.0, 200.0)
        print(f"  STESO moment lag at 1.14 Hz {1e3 * lS:.1f} ms -> equal-lag mdob K = {K2:.1f} (floor {floor('mdob', K2, 'M', 1):.4f}, {floor('mdob', K2, 'M', 1) / tM:.2f}x)")
        target_lag = float(os.environ.get("FORCE_LAG_TARGET", "0.015"))
        woF = solve(lambda k: -lag_gain("leso", k, "F")[0], -target_lag, 5.0, 400.0)
        KF = solve(lambda k: -lag_gain("mdob", k, "F")[0], -target_lag, 5.0, 400.0)
        print(f"force twins at the STESO's closed-loop reading lag {1e3 * target_lag:.0f} ms: leso wo_F = {woF:.1f}, mdob K_F = {KF:.1f}")
        print(f"  floors there: steso(L_F 3) {floor('steso', 3.0, 'F', 0):.4f}  leso {floor('leso', woF, 'F', 0):.4f}  mdob {floor('mdob', KF, 'F', 0):.4f} N")
    else:
        knobs, lag_by_kind = node_defaults()
        print("estimator-alone parity table at the node's defaults (200 Hz, 1 deg/s rate noise, 0.068 N model-force noise):")
        table(knobs)
        if lag_by_kind:
            print("predictor meas_lag per kind:", lag_by_kind)
