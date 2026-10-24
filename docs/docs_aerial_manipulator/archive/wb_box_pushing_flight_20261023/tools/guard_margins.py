#!/usr/bin/env python3
"""How close each box-pushing abort guard came to its limit on the two hardware pushes
(10-21 _170123, 10-23 _172853), in the window where that guard is armed. Signals are rebuilt the
way the planner / flight node compute them:
  planner tilt     acos(R22) of PX4 vehicle_attitude, armed after Go To Start completes
  claw force       |wb_control_debug[97..99]| low-passed at 3 rad/s (the planner's filter), armed
                   with the claw on the handle or the push flying
  box tilt / lag   push_pull/arrival_error [12] / [11] (the planner's own values)
  q3               joint_states joint3; the rate rule replayed as in trip_window.py
  mocap guard      state_estimator/mocap_guard [0] state, [2] age of the last good sample
  node watchdog    tilt (PX4 attitude), body rate |gyro| (sensor_combined), horizontal CoM error
Usage: /usr/bin/python3 guard_margins.py"""
import numpy as np
from bp_common import Bag

# (direct start, end of direct, claw on the handle from, end of the armed contact window)
WIN = {"170123": dict(direct=(12.58, 93.60), steps_done=31.32, handle=(36.51, 46.62)),
       "172853": dict(direct=(11.36, 111.47), steps_done=40.14, handle=(45.00, 48.84))}


def win(t, a, b):
    return (t >= a) & (t <= b)


def lpf(t, x, w):
    y = np.zeros_like(x)
    for i in range(1, len(x)):
        dt = np.clip(t[i] - t[i - 1], 0.0, 0.1)
        a = np.exp(-w * dt)
        y[i] = a * y[i - 1] + (1 - a) * x[i]
    return y


for tag, w in WIN.items():
    b = Bag(tag)
    d0, d1 = w["direct"]
    h0, h1 = w["handle"]
    print(f"\n===== {tag}  (DIRECT {d0:.1f}-{d1:.1f} s, claw on the handle {h0:.1f}-{h1:.1f} s)")
    tv, q = b.t("vatt"), b.f("vatt", "q")
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    m = win(tv, w["steps_done"], d1 - 0.5)
    print(f"  planner tilt (limit 10 deg, after Go To Start)  max {tilt[m].max():5.2f}  p99 {np.percentile(tilt[m], 99):5.2f}")
    m = win(tv, d0, d1 - 0.5)
    print(f"  node tilt watchdog (15 deg, all DIRECT)         max {tilt[m].max():5.2f}")
    ts, g = b.t("sc"), b.f("sc", "gyro_rad")
    rate = np.degrees(np.linalg.norm(g, axis=1))
    m = win(ts, d0, d1 - 0.5)
    print(f"  node rate watchdog (360 deg/s)                  max {rate[m].max():5.0f}  p99.9 {np.percentile(rate[m], 99.9):4.0f}")
    wt, W = b.t("wb"), b.f("wb", "data")
    dm = win(wt, d0, d1 - 0.5) & (W[:, 0] > 0.5)
    drift = np.hypot(W[dm, 48] - W[dm, 45], W[dm, 49] - W[dm, 46])
    print(f"  node drift watchdog (0.75 m)                    max {drift.max():5.3f} m")
    F = W[:, 97:100].astype(float)
    ok = np.all(np.isfinite(F), axis=1) & (W[:, 0] > 0.5)
    tf, F = wt[ok], F[ok]
    Ff = np.column_stack([lpf(tf, F[:, i], 3.0) for i in range(3)])
    n = np.linalg.norm(Ff, axis=1)
    mh = win(tf, h0, h1)
    print(f"  claw force (5 N for 0.3 s, on the handle)       max {n[mh].max():5.2f} N  (raw |F| rms {np.sqrt(np.mean(np.sum(F[mh]**2, 1))):4.1f} N)")
    ta, A = b.t("pl_arr"), b.f("pl_arr", "data")
    ma = win(ta, h0, h1)
    print(f"  box tilt (2 deg for 0.2 s, on the handle)       max {np.nanmax(A[ma, 12]):5.2f} deg")
    print(f"  box lag (40 mm for 0.2 s, push flying)          max {1e3*np.nanmax(np.abs(A[ma, 11])):5.1f} mm")
    tj, P = b.t("js"), np.degrees(b.f("js", "position"))
    mj = win(tj, h0, h1 + 0.6)
    print(f"  q3 (static 10 deg for 0.2 s)                    min {P[mj, 1].min():5.1f} deg in the window + 0.6 s")
    tg, G = b.t("guard"), b.f("guard", "data")
    mg = win(tg, d0, d1)
    bad = mg & (G[:, 0] > 0)
    print(f"  mocap guard (CONFIRMED 0.15 s / LOST 1.0 s)     non-MOCAP samples {int(bad.sum())}, "
          f"worst age {G[bad, 2].max() if bad.any() else 0:.3f} s, max state {int(G[mg, 0].max())}")
