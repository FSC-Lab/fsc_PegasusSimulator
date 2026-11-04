#!/usr/bin/env python3
"""The yaw ring in still holds (home and work poses, z > 0.6), per flight: gyro yaw-rate rms 0.5-3 Hz, its peak
frequency, the motor yaw differential (CW FL+RR minus CCW FR+RL): mean and 0.5-3 Hz std, the law's joint-1 torque
command std (0.5-3 Hz), and whether the Valve tab's j1/j4 error-damping raise was on (the arm GS log)."""
import os, sys, glob
import numpy as np
import history as H
def bp(x, lo, hi, dt):
    n = np.arange(len(x)); y = x - np.polyval(np.polyfit(n, x, 1), n)
    F = np.fft.rfft(y); f = np.fft.rfftfreq(len(y), dt); F[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(F, len(y)), f, np.abs(np.fft.rfft(y)) ** 2
FL = [(ts, t) for ts, t in H.FLIGHTS if not ts.startswith("---")] + [("192853", "valve 11-03e")]
NPZ3 = "../../wb_valve_turning_flight_20261103/npz"
print(f"{'flight':18s} damp | hold s | yaw rate 0.5-3 Hz rms | peak Hz | yaw diff mean / ring std | j1 torque ring std")
for ts, task in FL:
    p = H.find_npz(ts) or os.path.join(NPZ3, f"b{ts}.npz")
    if not os.path.exists(p): continue
    H._c.NPZ = os.path.dirname(p); b = H._c.Bag(ts)
    try: t, D = H.wb(b)
    except Exception: continue
    if not b.has("act_motors"): continue
    msgs = [str(m) for m in b.f("rosout", "msg")] if b.has("rosout") else []
    damp = any("error damping [0.20" in m for m in msgs)
    segs = [(a, c) for a, c in H.static_segments(t, D, 2.0) if D[(t >= a) & (t <= c), 47].mean() > 0.6]
    if not segs: continue
    ts_ = b.t("sc"); gz = np.degrees(np.stack(b.f("sc", "gyro_rad")).astype(float)[:, 2])
    tm = b.t("act_motors"); mc = np.stack(b.f("act_motors", "control")).astype(float)[:, :4]; yd = (mc[:, 2] + mc[:, 3]) - (mc[:, 0] + mc[:, 1])
    tc = b.t("tcmd") if b.has("tcmd") else None
    if tc is not None: j1 = np.stack(b.f("tcmd", "effort")).astype(float)[:, 0]
    R, P, M, S, J, dur = [], [], [], [], [], 0
    for a, c in segs:
        T = np.arange(a, c, 0.005)
        if len(T) < 400: continue
        y_, f, pw = bp(np.interp(T, ts_, gz), 0.5, 3, 0.005); R.append(y_.std()); dur += c - a
        m = (f > 0.5) & (f < 3); P.append(f[m][np.argmax(pw[m])])
        yd_i = np.interp(T, tm, yd); M.append(yd_i.mean()); S.append(bp(yd_i, 0.5, 3, 0.005)[0].std())
        if tc is not None: J.append(bp(np.interp(T, tc, j1), 0.5, 3, 0.005)[0].std())
    if not R: continue
    print(f"{task:12s} {ts} {'0.2' if damp else ' - ':4s} | {dur:5.1f}  | {np.mean(R):5.2f} deg/s          | {np.median(P):4.2f}    | {np.mean(M):+.2f} / {np.mean(S):.3f}          | {np.mean(J) if J else np.nan:.3f} N.m")
