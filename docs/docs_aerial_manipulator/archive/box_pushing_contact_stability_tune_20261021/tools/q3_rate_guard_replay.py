#!/usr/bin/env python3
"""Replay the planner's q3 guard, and a proposed RATE-AWARE one, on recorded runs.

    PYTHONNOUSERSITE=1 /usr/bin/python3 q3_rate_guard_replay.py ../runs/*.npz

Engaged window = the planner's: from ready:end (claw on the handle) until exit:start, or until
the run's first abort / DIRECT loss. q3 = the recorded joint state (250 Hz).

  CURRENT  (planner today): on the 10 Hz tick, q3 < LIM continuously for 0.2 s -> trip.
  RATE     (proposed): on every joint sample, p = q3 + TAU * q3_dot; trip when p < LIM
           and q3_dot < -MIN_RATE continuously for HOLD s; the CURRENT check stays as well.
           q3_dot = finite difference low-passed (first order, VEL_TC).
  HW       the same RATE rule on a hardware-like signal: q3 quantised to the encoder
           (4096 counts/rev = 0.088 deg), joint states at JS_HZ, velocity lagged by
           an extra VEL_LAG (the arm's velocity observer ~12 ms + the stream).

Prints, per run: the window's q3 min, min of p (the margin a completed run leaves the rule),
and, where the run failed, when each rule trips and q3 at that moment.
"""
import os
import re
import sys

import numpy as np

LIM = 10.0          # deg, push_pull_guard_q3_min_deg
PERSIST = 0.2       # s, push_pull_guard_box_persist_s
TICK = 0.1          # s, the planner's 10 Hz guard tick
TAU = float(os.environ.get("TAU", 0.2))       # s, look-ahead
HOLD = float(os.environ.get("HOLD", 0.05))    # s, persistence of the rate rule
MIN_RATE = float(os.environ.get("MIN_RATE", 5.0))  # deg/s, ignore a still joint's noise
VEL_TC = 0.02       # s
ENC = 360.0 / 4096
JS_HZ = 100.0
VEL_LAG = 0.03      # s


def ev_time(z, pats):
    for e in z["events"]:
        s = str(e)
        if any(p in s for p in pats):
            m = re.match(r"\[\s*([0-9.]+)s\]", s)
            if m:
                return float(m.group(1))
    return None


def lpf(t, x, tc):
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        a = 1.0 - np.exp(-(t[i] - t[i - 1]) / tc)
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def rate(t, q):
    d = np.gradient(q, t)
    return lpf(t, d, VEL_TC)


def current_trip(t, q, t0, t1):
    since = None
    for tk in np.arange(t0, t1, TICK):
        qk = np.interp(tk, t, q)
        if qk < LIM:
            since = tk if since is None else since
            if tk - since >= PERSIST - 1e-9:
                return tk
        else:
            since = None
    return None


def rate_trip(t, q, qd, t0, t1):
    m = (t >= t0) & (t <= t1)
    tt, p, v = t[m], (q + TAU * qd)[m], qd[m]
    bad = (p < LIM) & (v < -MIN_RATE)
    since = None
    for i in range(len(tt)):
        if bad[i]:
            since = tt[i] if since is None else since
            if tt[i] - since >= HOLD - 1e-9:
                return tt[i]
        else:
            since = None
    return None


def hw_signal(t, q):
    ts = np.arange(t[0], t[-1], 1.0 / JS_HZ)
    qs = np.round(np.interp(ts, t, q) / ENC) * ENC
    qd = rate(ts, qs)
    qd = np.interp(ts - VEL_LAG, ts, qd)
    return ts, qs, qd


def one(path):
    z = np.load(path, allow_pickle=True)
    tag = os.path.basename(path)[:-4]
    mk = {str(n): float(x) for n, x in zip(z["marks_name"], z["marks_t"])}
    if "ready:end" not in mk:
        return None
    J = z["joints"]
    t, q = J[:, 0], np.degrees(J[:, 3])
    t0 = mk["ready:end"]
    t_ab = ev_time(z, ["ABORTING (SAFETY GUARD"])
    t_nd = None
    for e in z["events"]:
        s = str(e)
        mm = re.match(r"\[\s*([0-9.]+)s\]", s)
        if mm and float(mm.group(1)) > t0 and "push_pull: NOT IN DIRECT" in s:
            t_nd = float(mm.group(1))
            break
    ends = [x for x in (mk.get("exit:start"), t_ab, t_nd, t[-1]) if x is not None]
    t1 = min(ends)
    # a failed run: score the trips up to 1.5 s after the end (the rule may fire after the
    # recorded abort -- it is reported only if it fires before it)
    failed = bool(z["aborted"]) and (t_ab is not None or t_nd is not None)
    qd = rate(t, q)
    w = (t >= t0) & (t <= t1)
    p = q + TAU * qd
    r = {"tag": tag, "failed": failed, "q3min": q[w].min(), "pmin": p[w].min(),
         "how": ("guard" if t_ab is not None and (t_nd is None or t_ab <= t_nd) else
                 "left DIRECT" if t_nd is not None else "")}
    if failed:
        tf = t1
        r["t_end"] = tf
        r["q3_end"] = np.interp(tf, t, q)
        lo = t0
        r["t_cur"] = current_trip(t, q, lo, tf + 1.5)
        r["t_rate"] = rate_trip(t, q, qd, lo, tf + 1.5)
        ts, qs, qdh = hw_signal(t, q)
        r["t_hw"] = rate_trip(ts, qs, qdh, lo, tf + 1.5)
        for k in ("t_cur", "t_rate", "t_hw"):
            r["q_" + k] = np.interp(r[k], t, q) if r[k] is not None else np.nan
        # the fastest fold in the last second before the end
        ww = (t >= tf - 1.0) & (t <= tf)
        r["vmin"] = qd[ww].min() if ww.any() else np.nan
    else:
        ts, qs, qdh = hw_signal(t, q)
        wh = (ts >= t0) & (ts <= t1)
        r["pmin_hw"] = (qs + TAU * qdh)[wh].min()
        r["false_rate"] = rate_trip(t, q, qd, t0, t1) is not None
        r["false_hw"] = rate_trip(ts, qs, qdh, t0, t1) is not None
    return r


def main():
    rows = [x for x in (one(p) for p in sorted(sys.argv[1:])) if x is not None]
    print(f"TAU {TAU} s, HOLD {HOLD} s, MIN_RATE {MIN_RATE} deg/s, LIM {LIM} deg\n")
    print("FAILED RUNS: when each rule trips (s after the claw reached the handle is not shown;"
          " times relative to the recorded end), q3 at the trip [deg]")
    print(f"{'run':14s} {'ended by':12s} {'q3 end':>7s} {'fold':>7s} | {'current':>15s} | "
          f"{'rate':>15s} | {'rate, hw-like':>15s}")
    for r in rows:
        if not r["failed"]:
            continue

        def f(k):
            if r[k] is None:
                return "     never     "
            return f"{r[k] - r['t_end']:+6.2f}s {r['q_' + k]:6.1f}"
        print(f"{r['tag']:14s} {r['how']:12s} {r['q3_end']:7.1f} {r['vmin']:6.0f}/s | {f('t_cur')} | "
              f"{f('t_rate')} | {f('t_hw')}")
    print("\nCOMPLETED / NON-ARM FAILURES: engaged-window q3 min and min of q3 + TAU*q3_dot [deg]")
    ok = [r for r in rows if not r["failed"]]
    fr = [r["tag"] for r in ok if r["false_rate"]]
    fh = [r["tag"] for r in ok if r["false_hw"]]
    for r in ok:
        print(f"{r['tag']:14s} q3 min {r['q3min']:6.1f}   p min {r['pmin']:6.1f}   p min hw {r['pmin_hw']:6.1f}"
              f"{'   FALSE TRIP' if r['false_rate'] or r['false_hw'] else ''}")
    print(f"\nfalse trips: rate {len(fr)}/{len(ok)} {fr}, hw-like {len(fh)}/{len(ok)} {fh}")


if __name__ == "__main__":
    main()
