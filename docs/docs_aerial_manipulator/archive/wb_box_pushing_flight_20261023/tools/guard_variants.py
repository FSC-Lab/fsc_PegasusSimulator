#!/usr/bin/env python3
"""Replay the push-and-pull q3 guards, and candidate relaxations, on the two hardware pushes
(10-21 _170123, 10-23 _172853) and every Isaac run of the 2026-10-21/22 contact-stability
campaign (67 runs: completions, guard aborts and the three buckles that left DIRECT).

Engaged window = the planner's (claw on the handle or pushing): ready:end -> exit:start, the
first abort, or the DIRECT loss (+1.0 s after a DIRECT loss, so a rule that would have fired
during the buckle is scored). All rates: finite difference low-passed over 0.02 s, as in
plQ3RateGuard. Hardware joints use their header stamps.

Rules (each also keeps the STATIC check: q3 < 10 deg for 0.2 s on the 10 Hz tick):
  STATIC      static check only (the rate guard removed)
  RATE        today's rate guard: q3 + 0.2 q3dot < 10 while q3dot < -5 deg/s, for 0.05 s
  RATE_Q2CAP  the same prediction with q2's predicted motion capped at its limit Q2MAX:
              q3_pred = q3 + 0.2 q3dot + max(0, q2 + 0.2 q2dot - Q2MAX) -- kinematically
              q3 = (q2+q3) - q2, and q2 cannot pass its limit, so the reach motion (q2 up,
              q3 down at ~constant fold) cannot take q3 below fold - Q2MAX
  RATE_LIM0   today's rule predicting the singular branch itself (q3_pred < 0)
  RATE_T01    today's rule with a 0.1 s look-ahead
Usage: /usr/bin/python3 guard_variants.py [Q2MAX=45.0]"""
import glob
import os
import re
import sys
import numpy as np
from bp_common import Bag

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "..", "box_pushing_contact_stability_tune_20261021", "runs")
Q2MAX = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
LIM, PERSIST, TICK = 10.0, 0.2, 0.1


def lpf_rate(t, q, tc=0.02):
    v = np.zeros_like(q)
    for i in range(1, len(q)):
        d = t[i] - t[i - 1]
        if 0 < d < 0.1:
            v[i] = v[i - 1] + (1 - np.exp(-d / tc)) * ((q[i] - q[i - 1]) / d - v[i - 1])
        else:
            v[i] = v[i - 1] if d <= 0 else 0.0
    return v


def static_trip(t, q3, t0, t1):
    since = None
    for tk in np.arange(t0, t1, TICK):
        if np.interp(tk, t, q3) < LIM:
            since = tk if since is None else since
            if tk - since >= PERSIST - 1e-9:
                return tk
        else:
            since = None
    return None


def rate_trip(t, pred, v3, t0, t1, lim=LIM, vmin=5.0, hold=0.05):
    m = np.flatnonzero((t >= t0) & (t <= t1))
    since = None
    for i in m:
        if pred[i] < lim and v3[i] < -vmin:
            since = t[i] if since is None else since
            if t[i] - since >= hold - 1e-9:
                return t[i]
        else:
            since = None
    return None


def rules(t, q2, q3):
    v2, v3 = lpf_rate(t, q2), lpf_rate(t, q3)
    p02 = q3 + 0.2 * v3
    cap = np.maximum(0.0, q2 + 0.2 * v2 - Q2MAX)
    return {
        "RATE": (p02, LIM, v3),
        "RATE_Q2CAP": (p02 + cap, LIM, v3),
        "RATE_LIM0": (p02, 0.0, v3),
        "RATE_T01": (q3 + 0.1 * v3, LIM, v3),
    }


def first(*xs):
    xs = [x for x in xs if x is not None]
    return min(xs) if xs else None


def score(name, t, q2, q3, t0, t_end, outcome):
    """outcome: 'completed' | 'guard' (some other guard or the q3 guard aborted) | 'buckle'
    (left DIRECT: the unsafe case). t_end: the window's end."""
    t1 = t_end + (1.0 if outcome == "buckle" else 0.0)
    out = {"name": name, "outcome": outcome, "t_end": t_end}
    ts = static_trip(t, q3, t0, t1)
    out["STATIC"] = ts
    for k, (pred, lim, v3) in rules(t, q2, q3).items():
        out[k] = first(ts, rate_trip(t, pred, v3, t0, t1, lim))
    w = (t >= t0) & (t <= t_end)
    out["q3min"] = q3[w].min()
    out["q"] = (t, q2, q3)
    return out


def ev_time(events, pats, after):
    for e in events:
        s = str(e)
        m = re.match(r"\[\s*([0-9.]+)s\]", s)
        if m and float(m.group(1)) > after and any(p in s for p in pats):
            return float(m.group(1)), s
    return None, ""


def sim_runs():
    for f in sorted(glob.glob(os.path.join(SIM, "*.npz"))):
        z = np.load(f, allow_pickle=True)
        mk = {str(n): float(x) for n, x in zip(z["marks_name"], z["marks_t"])}
        if "ready:end" not in mk:
            continue
        J = z["joints"]
        t, q2, q3 = J[:, 0], np.degrees(J[:, 2]), np.degrees(J[:, 3])
        t0 = mk["ready:end"]
        t_nd, _ = ev_time(z["events"], ["push_pull: NOT IN DIRECT"], t0)
        t_ab, s_ab = ev_time(z["events"], ["push_pull: ABORTING"], t0)
        t_ex = mk.get("exit:start")
        cands = [(t_nd, "buckle"), (t_ab, "guard"), (t_ex, "completed")]
        cands = [c for c in cands if c[0] is not None]
        if not cands:
            cands = [(t[-1], "completed")]
        t_end, outc = min(cands)
        # a run whose abort was the q3 guard itself: the rules decide when it fires; score the
        # natural continuation 1.0 s on, but the abort changes the arm then -- mark it
        if outc == "guard" and "q3" in s_ab:
            outc = "q3guard"
        yield score("SIM " + os.path.basename(f)[:-4], t, q2, q3, t0, t_end, outc)


def hw_runs():
    for tag, t0, t_end, outc in (("170123", 36.50, 46.62, "q3guard"), ("172853", 45.00, 48.836, "q3guard")):
        b = Bag(tag)
        P = np.degrees(b.f("js", "position"))
        th = b.f("js", "hdr")
        t = b.t("js")[0] + (th - th[0])
        yield score("HW " + tag, t, P[:, 0], P[:, 1], t0, t_end, outc)


def fmt(r, k):
    v = r[k]
    if v is None:
        return "       -       "
    t, q2, q3 = r["q"]
    return f"{v - r['t_end']:+6.2f}s q3{np.interp(v, t, q3):5.1f}"


if __name__ == "__main__":
    rows = list(hw_runs()) + list(sim_runs())
    keys = ["STATIC", "RATE", "RATE_Q2CAP", "RATE_LIM0", "RATE_T01"]
    print(f"Q2MAX {Q2MAX} deg. Trip time relative to the window end (the recorded abort / DIRECT loss /"
          f" exit start) and q3 there. '-' = never fires in the window.\n")
    print(f"{'run':18s} {'outcome':9s} {'q3min':>6s} | " + " | ".join(f"{k:^15s}" for k in keys))
    for r in rows:
        if r["outcome"] == "completed" and all(r[k] is None for k in keys):
            continue
        print(f"{r['name']:18s} {r['outcome']:9s} {r['q3min']:6.1f} | " + " | ".join(fmt(r, k) for k in keys))
    ok = [r for r in rows if r["outcome"] == "completed"]
    print(f"\ncompleted runs (no guard of any kind): {len(ok)}; false trips:")
    for k in keys:
        print(f"  {k:11s} {sum(r[k] is not None for r in ok):2d}  {[r['name'] for r in ok if r[k] is not None]}")
    bk = [r for r in rows if r["outcome"] == "buckle"]
    print(f"\nbuckles (left DIRECT): {len(bk)}; caught BEFORE the DIRECT loss (lead time, q3):")
    for k in keys:
        s = [(r["name"], round(r["t_end"] - r[k], 2), round(float(np.interp(r[k], r['q'][0], r['q'][2])), 1))
             for r in bk if r[k] is not None and r[k] < r["t_end"]]
        print(f"  {k:11s} {len(s)}/{len(bk)}  {s}")
