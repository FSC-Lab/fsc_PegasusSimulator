#!/usr/bin/env python3
"""Score the four-edge guard runs (2026-11-04, sim/chain6.sh): for each run, the fin-guard scorer's summary
(sim_table.row: outcome, valve turned, tilt, q3, DIRECT) and, when a guard tripped:
  TRIP    where (the hand-over hold before the turn, or the turn and how far the valve had turned), the guard's
          reason, and its readings then (the claw's depth under the plate top, its offset across the plate's
          centre line, the gap from the claw's tip to the face; arrival_error [34..36] -> recorder columns 36..38)
  CLIMB   how fast the abort left: the vehicle 10 / 50 / 100 mm above its height at the trip (Isaac's truth)
  AFTER   what touched what after the trip (Isaac's contact report): the last gripper-valve contact, the largest
          force on the valve, any non-gripper contact; the valve's rotation after the trip; the peak tilt after it
Run: PYTHONNOUSERSITE=1 /usr/bin/python3 edge_score.py [tags...]   (default: the chain-6 runs s1..s6)"""
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plate_score as PS  # noqa: E402
import sim_table as ST    # noqa: E402

RUNS = os.path.join(HERE, "..", "sim", "runs")
TAGS = ["s1_nominal", "s2_drift3", "s3_outer", "s4_inner", "s5_top", "s6_bottom"]
DT = 0.004


def yaw_deg(q):
    return np.degrees(np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                                           1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))))


def score(tag):
    f = os.path.join(RUNS, tag + ".npz")
    if not os.path.exists(f):
        return f"{tag:12s} (no run)"
    base = ST.row(f)
    z = np.load(f, allow_pickle=True)
    mt = dict(zip(z["marks_name"], z["marks_t"]))
    out = [f"{tag:12s} {base['outcome'][:9]:9s} valve {base.get('valve', float('nan')):6.1f} deg | tilt max "
           f"{base.get('tilt', float('nan')):4.1f} deg | q3 min {base.get('q3min', float('nan')):5.1f} deg | DIRECT "
           f"{base.get('direct', '?')}"]
    trip_t, reason = None, ""
    for e in z["events"]:
        e = str(e)
        m = re.match(r"\[\s*([\d.]+)s\] push_pull: ABORTING \((.*?)\) --", e)
        if m:
            trip_t, reason = float(m.group(1)), m.group(2)
            break
    if trip_t is None:
        return "\n".join(out + ["             no guard tripped"])
    b = z["box"]; yv = yaw_deg(b[:, 4:8])
    r0 = mt.get("ready:start", mt.get("direct", 0.0))
    p0, p1 = mt.get("push:start"), mt.get("push:end")
    v_trip = float(np.interp(trip_t, b[:, 0], yv) - np.interp(r0, b[:, 0], yv))
    if p0 is None or trip_t < p0:
        where = "the hand-over hold (before the turn)"
    elif p1 is None or trip_t <= p1:
        where = f"the TURN, {trip_t - p0:.1f} s in, the valve at {v_trip:.1f} deg"
    else:
        where = "after the turn"
    a = z["arrival"]

    def at(c):
        w = np.isfinite(a[:, c])
        return float(np.interp(trip_t, a[w, 0], a[w, c]) * 1e3) if w.any() else float("nan")
    out.append(f"             TRIP  in {where}: {reason}")
    out.append(f"                   readings: depth {at(36):5.1f} mm under the top, across {at(37):+5.1f} mm, "
               f"tip-to-face gap {at(38):+5.1f} mm")
    bt = z["base_truth"]
    z0 = float(np.interp(trip_t, bt[:, 0], bt[:, 3]))
    w = bt[:, 0] >= trip_t

    def rise(dz):
        k = np.nonzero(bt[w, 3] > z0 + dz)[0]
        return f"{bt[w, 0][k[0]] - trip_t:4.2f}" if len(k) else "  - "
    out.append(f"             CLIMB +10 mm at +{rise(0.010)} s, +50 mm at +{rise(0.050)} s, +100 mm at +{rise(0.100)} s")
    od = z["odom"]; q = od[:, 7:11]
    tl = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    after = od[:, 0] >= trip_t
    v_after = float(yv[-1] - np.interp(trip_t, b[:, 0], yv))
    # the contacts after the trip (the scene's clock, aligned on the first contact as plate_score.py does)
    path = f[:-4] + ".contacts.csv"
    last_c, fmax, other = None, 0.0, {}
    if os.path.exists(path):
        C = []
        for line in open(path):
            v = line.rstrip().split(",")
            if v[0] == "C":
                C.append((float(v[1]), v[2], np.array([float(x) for x in v[10:13]])))
        c = z["contact"]; wp = c[c[:, 20] > 0, 0]
        if C and len(wp):
            off = min(t for t, *_ in C) - float(wp.min())
            steps = {}
            for t, link, imp in C:
                tn = t - off
                if tn < trip_t:
                    continue
                if link not in PS.GRIPPER:
                    other[link] = other.get(link, 0) + 1
                    continue
                last_c = tn
                steps[round(t, 4)] = steps.get(round(t, 4), 0) + imp
            if steps:
                fmax = max(np.linalg.norm(s) / DT for s in steps.values())
    lc = f"the last gripper contact +{last_c - trip_t:.2f} s, peak force on the valve {fmax:.2f} N" if last_c else \
        "no gripper contact"
    out.append(f"             AFTER {lc}; non-gripper contacts {other or 'none'}; the valve moved "
               f"{v_after:+.1f} deg; tilt max {tl[after].max():.1f} deg")
    return "\n".join(out)


def main():
    for t in (sys.argv[1:] or TAGS):
        print(score(t))


if __name__ == "__main__":
    main()
