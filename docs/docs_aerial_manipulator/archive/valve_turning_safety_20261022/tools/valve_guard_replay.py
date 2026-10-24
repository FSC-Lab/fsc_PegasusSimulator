#!/usr/bin/env python3
"""Replay the box push's joint and box guards on the recorded VALVE runs (2026-10-22).

    PYTHONNOUSERSITE=1 /usr/bin/python3 valve_guard_replay.py [--current] [run.npz ...]

Default runs: every whole-body valve run in valve_turning_20261015/runs (the coordinated-
admittance ones excluded) plus the four video takes (box_pushing_video_20261017/takes/
valve_rep*/mission.npz, the final config).

Per run, from the claw's arrival on the fin (ready:end) to the exit (or the run's first abort /
DIRECT loss / injected abort):
  * q3's minimum, and the box push's q3 guards replayed on a hardware-like signal (the
    encoder's 0.088 deg, joint states at 100 Hz, the velocity observer's ~30 ms):
      STATIC  q3 < LIM for 0.2 s on the 10 Hz tick (push_pull_guard_q3_min_deg)
      RATE    q3 + TAU q3_dot < LIM while q3_dot < -MIN_RATE for 0.05 s (..._q3_lookahead_s)
  * the TURN LAG: the planned claw's rotation about the valve's axis (the streamed r_ed)
    minus the rotor's measured rotation (obj_0's yaw), both from the push's start -- the
    valve's counterpart of the box push's box-lag guard.
A failed run's "end" is its tilt-guard abort or its DIRECT loss; the guards' trip times are
printed relative to it (negative = earlier).
"""
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", ".."))
ENC = 360.0 / 4096
JS_HZ = 100.0
VEL_LAG = 0.03
VEL_TC = 0.02


def ev_time(z, pats, after=0.0):
    for e in z["events"]:
        s = str(e)
        m = re.match(r"\[\s*([0-9.]+)s\]", s)
        if m and float(m.group(1)) >= after and any(p in s for p in pats):
            return float(m.group(1))
    return None


def lpf(t, x, tc):
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        a = 1.0 - np.exp(-(t[i] - t[i - 1]) / tc)
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def hw_q3(t, q):
    ts = np.arange(t[0], t[-1], 1.0 / JS_HZ)
    qs = np.round(np.interp(ts, t, q) / ENC) * ENC
    qd = lpf(ts, np.gradient(qs, ts), VEL_TC)
    qd = np.interp(ts - VEL_LAG, ts, qd)
    return ts, qs, qd


def static_trip(t, q, t0, t1, lim, persist=0.2, tick=0.1):
    since = None
    for tk in np.arange(t0, t1, tick):
        if np.interp(tk, t, q) < lim:
            since = tk if since is None else since
            if tk - since >= persist - 1e-9:
                return tk
        else:
            since = None
    return None


def rate_trip(t, q, qd, t0, t1, lim, tau, min_rate, hold=0.05):
    m = (t >= t0) & (t <= t1)
    tt = t[m]
    bad = ((q + tau * qd)[m] < lim) & (qd[m] < -min_rate)
    since = None
    for i in range(len(tt)):
        if bad[i]:
            since = tt[i] if since is None else since
            if tt[i] - since >= hold - 1e-9:
                return tt[i]
        else:
            since = None
    return None


def yaw_of(qwxyz):
    w, x, y, z = qwxyz.T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def config_of(path):
    """(pose, omega_c_t, tilt guard) from the run's saved yaml, or None."""
    y = path[:-4] + ".yaml"
    if not os.path.exists(y):
        return None
    txt = open(y).read()
    def g(k):
        m = re.search(rf"^ +{k}: ([^#\n]*)", txt, re.M)
        return m.group(1).strip() if m else None
    return g("push_pull_push_pose_deg"), g("wb_l1_omega_c_t"), g("push_pull_guard_tilt_deg")


def one(path, tag):
    z = np.load(path, allow_pickle=True)
    mk = {}
    for n, x in zip(z["marks_name"], z["marks_t"]):
        mk.setdefault(str(n), float(x))
    if "ready:end" not in mk or "push:start" not in mk:
        return None
    t0 = mk["ready:end"]
    t_ab = ev_time(z, ["ABORTING (SAFETY GUARD"], t0)
    t_inj = ev_time(z, ["INJECT"], t0)
    t_nd = ev_time(z, ["push_pull: NOT IN DIRECT"], t0)
    J = z["joints"]
    t, q = J[:, 0], np.degrees(J[:, 3])
    ends = [x for x in (mk.get("exit:start"), t_ab, t_nd, t_inj, t[-1]) if x is not None]
    t1 = min(ends)
    injected = t_inj is not None and t_inj <= t1 + 1e-6
    failed = not injected and bool(z["aborted"]) and (t_ab is not None or t_nd is not None)
    how = ("injected" if injected else
           ("tilt/guard" if t_ab is not None and (t_nd is None or t_ab <= t_nd) else "left DIRECT") if failed
           else "complete" if not bool(z["aborted"]) else "aborted later")
    ts, qs, qd = hw_q3(t, q)
    w = (ts >= t0) & (ts <= t1)
    r = dict(tag=tag, how=how, failed=failed, t0=t0, t1=t1, ts=ts, qs=qs, qd=qd,
             q3min=qs[w].min() if w.any() else np.nan, cfg=config_of(path))
    # the turn lag
    ref, box = z["ref"], z["box"]
    tp0 = mk["push:start"]
    tp1 = min(mk.get("push:end", t1), t1)
    r["lag_max"] = r["lag_end"] = np.nan
    r["lag_t"] = r["lag_v"] = None
    mb = (box[:, 0] >= tp0 - 0.2) & (box[:, 0] <= tp1)
    mr = (ref[:, 0] >= tp0 - 0.2) & (ref[:, 0] <= tp1)
    if mb.sum() > 20 and mr.sum() > 20:
        axis = box[mb][0, 1:3]
        tb = box[mb, 0]
        yb = np.unwrap(yaw_of(box[mb, 4:8]))
        yb = yb - np.interp(tp0, tb, yb)
        tr = ref[mr, 0]
        ar = np.unwrap(np.arctan2(ref[mr, 2] - axis[1], ref[mr, 1] - axis[0]))
        ar = ar - np.interp(tp0, tr, ar)
        sgn = 1.0 if ar[-1] >= 0 else -1.0
        tt = tb[tb >= tp0]
        lag = sgn * np.degrees(np.interp(tt, tr, ar) - np.interp(tt, tb, yb))
        r["lag_tt"], r["lag"] = tt, lag
        if len(lag):
            r["lag_max"] = np.abs(lag).max()
            r["lag_end"] = lag[-1]
    return r


def lag_trip(r, lim, persist=0.2):
    if "lag" not in r:
        return None
    since = None
    for ti, li in zip(r["lag_tt"], r["lag"]):
        if abs(li) > lim:
            since = ti if since is None else since
            if ti - since >= persist - 1e-9:
                return ti
        else:
            since = None
    return None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    paths = args or (sorted(p for p in glob.glob(os.path.join(ARCH, "valve_turning_20261015/runs/*.npz"))
                            if not re.search(r"/(ca_|final_ca)", p)) +
                     sorted(glob.glob(os.path.join(ARCH, "box_pushing_video_20261017/takes/valve_rep*/mission.npz"))))
    rows = []
    for p in paths:
        tag = os.path.basename(os.path.dirname(p)) if p.endswith("mission.npz") else os.path.basename(p)[:-4]
        r = one(p, tag)
        if r is not None:
            if tag.startswith("valve_rep"):
                r["cfg"] = ("[0.0, 5.0, 25.0, 0.0]", "2.0", "10.0")
            rows.append(r)
    cur = lambda r: r["cfg"] is not None and r["cfg"][0] == "[0.0, 5.0, 25.0, 0.0]" and r["cfg"][1] == "2.0"
    if "--current" in sys.argv:
        rows = [r for r in rows if cur(r)]
    ok = [r for r in rows if r["how"] == "complete"]
    bad = [r for r in rows if r["failed"]]
    print(f"{len(rows)} runs: {len(ok)} complete, {len(bad)} failed, "
          f"{sum(r['how'] == 'injected' for r in rows)} injected aborts\n")
    print("THE TURN LAG (planned claw rotation - measured rotor rotation, deg)")
    print(f"  complete runs: max |lag| {np.nanmax([r['lag_max'] for r in ok]):.1f} deg "
          f"(p95 of the per-run max {np.nanpercentile([r['lag_max'] for r in ok], 95):.1f})")
    for r in sorted(ok, key=lambda r: -np.nan_to_num(r["lag_max"]))[:6]:
        print(f"    {r['tag']:16s} {r['lag_max']:5.1f}")
    for lim in (8.0, 10.0, 12.0, 15.0, 20.0):
        ft = [r["tag"] for r in ok if lag_trip(r, lim) is not None]
        caught = []
        for r in bad:
            tl = lag_trip(r, lim)
            if tl is not None and tl <= r["t1"] + 0.05:
                caught.append(f"{r['tag']}({tl - r['t1']:+.1f}s)")
        print(f"  lag > {lim:4.1f} deg for 0.2 s: false trips {len(ft)}/{len(ok)} {ft[:6]}; "
              f"before the end in {len(caught)}/{len(bad)} failures {caught[:8]}")
    print("\nq3 (hardware-like signal)")
    qmins = np.array([r["q3min"] for r in ok])
    print(f"  complete runs' q3 min: lowest {qmins.min():.1f}, p5 {np.percentile(qmins, 5):.1f}, "
          f"median {np.median(qmins):.1f} deg")
    for r in sorted(ok, key=lambda r: r["q3min"])[:8]:
        print(f"    {r['tag']:16s} {r['q3min']:6.1f}  {'CURRENT' if cur(r) else ''}")
    print("\n  rule                                  false trips (complete)   failures caught before the end [s, q3 at trip]")
    for kind, lim, tau, mr in (("static", 10.0, 0, 0), ("rate", 10.0, 0.2, 5.0),
                               ("static", 5.0, 0, 0), ("rate", 5.0, 0.2, 5.0), ("rate", 5.0, 0.2, 20.0),
                               ("static", 0.0, 0, 0), ("rate", 0.0, 0.2, 5.0), ("rate", 0.0, 0.2, 20.0),
                               ("rate", 0.0, 0.3, 30.0), ("static", -3.0, 0, 0), ("rate", -3.0, 0.2, 20.0)):
        def trip(r, lo, hi):
            if kind == "static":
                return static_trip(r["ts"], r["qs"], lo, hi, lim)
            return rate_trip(r["ts"], r["qs"], r["qd"], lo, hi, lim, tau, mr)
        ft = [r["tag"] for r in ok if trip(r, r["t0"], r["t1"]) is not None]
        caught = []
        for r in bad:
            tq = trip(r, r["t0"], r["t1"] + 1.5)
            if tq is not None and tq <= r["t1"] + 0.05:
                caught.append(f"{r['tag']}({tq - r['t1']:+.1f},{np.interp(tq, r['ts'], r['qs']):.0f})")
        name = f"{kind} q3<{lim:g}" + (f" tau {tau} rate>{mr:g}" if kind == "rate" else "")
        print(f"  {name:36s} {len(ft):3d}/{len(ok):<3d}  {','.join(ft[:4]):30s}  "
              f"{len(caught)}/{len(bad)} {' '.join(caught[:7])}")
    print("\nFAILURES (end = tilt-guard abort or DIRECT loss): q3 at the end, the fastest fold in its last second")
    for r in bad:
        w = (r["ts"] >= r["t1"] - 1.0) & (r["ts"] <= r["t1"])
        print(f"  {r['tag']:16s} {r['how']:12s} q3 end {np.interp(r['t1'], r['ts'], r['qs']):6.1f}  "
              f"fold {r['qd'][w].min() if w.any() else np.nan:6.0f} deg/s  lag max {r['lag_max']:5.1f}  "
              f"{'CURRENT' if cur(r) else ''}")


if __name__ == "__main__":
    main()
