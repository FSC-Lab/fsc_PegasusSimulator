#!/usr/bin/env python3
"""The free-axis anchor APPLIED (2026-10-21): the three validation flights on the shipped sim
box yaml (v75_n1 nominal, v75_heavy / v75_stop the 10-16 jam worst cases), against the 10-16
runs of the same worst cases without the anchor (h_heavy / h_stop, the old bare-plan guard).

    PYTHONNOUSERSITE=1 /usr/bin/python3 apply_metrics.py [tag ...]

Per run, from the claw's arrival on the handle (ready:end) to the end of the exit / abort:
  outcome   completed, or the guard that tripped and when (s after push:start)
  lag       the planner's box-lag reading (arrival [11]) max over the push [mm]; on the v75 runs it
            is measured against the LAW's target (the plan + the free-axis shift)
  shift     the law's free-axis shift (wb_control_debug [117]) range over the push [mm]
  bare      the lag against the bare plan, as the old guard read it = lag - shift [mm]
            (only for runs flown with the compensating planner: on the a75_* A/B runs, flown
            before it, arrival [11] IS the bare lag and the column means nothing)
  q3min / q2max, swing ((q2 - q3)/2 std over the push), tilt (vehicle, max) [deg]
  travel    the box along the push at the push end / the trip [mm]; lift = its highest rise [mm]
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")
OLD = os.path.join(HERE, "..", "..", "box_pushing_campaign_20261016", "runs")


def path_of(tag):
    for d in (RUNS, OLD):
        p = os.path.join(d, tag + ".npz")
        if os.path.isfile(p):
            return p
    raise SystemExit(f"no run {tag}")


def win(a, t0, t1):
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]


def zax(q):
    w, x, y, z = q.T
    return np.column_stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def one(tag):
    z = np.load(path_of(tag), allow_pickle=True)
    m = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
    ev = [str(e) for e in z["events"]]
    ps = m.get("push:start")
    if ps is None:
        return {"tag": tag, "note": "no push"}
    trip, t_trip = None, None
    for e in ev:
        if "SAFETY GUARD" in e and t_trip is None:
            t_trip = float(e[1:e.index("s]")])
            trip = e[e.index("SAFETY GUARD"):][:110]
    t_push_end = m.get("push:end", t_trip if t_trip is not None else ps + 30.0)
    t_tail = m.get("exit:end", (t_trip + 15.0) if t_trip is not None else t_push_end + 10.0)
    r = {"tag": tag, "ok": not bool(z["aborted"]) and trip is None, "trip": trip,
         "t_trip": None if t_trip is None else t_trip - ps}
    A = win(z["arrival"], ps, t_push_end + 0.3)
    lag = A[:, 12]
    D = z["dbg"].astype(float)
    shift = np.zeros_like(lag)
    if D.shape[1] > 2 + 117:
        shift = np.interp(A[:, 0], D[:, 0], D[:, 2 + 117])
    r["lag"], r["shift_lo"], r["shift_hi"] = np.nanmax(lag) * 1e3, np.nanmin(shift) * 1e3, np.nanmax(shift) * 1e3
    r["bare"] = np.nanmax(lag - shift) * 1e3
    re_ = m.get("ready:end", ps)
    J = win(z["joints"], re_, t_tail)
    r["q3min"], r["q2max"] = np.degrees(J[:, 3]).min(), np.degrees(J[:, 2]).max()
    jp = win(z["joints"], ps, t_push_end)
    r["swing"] = (np.degrees(jp[:, 2] - jp[:, 3]) / 2).std()
    bt = win(z["base_truth"], re_, t_tail)
    r["tilt"] = np.degrees(np.arccos(np.clip(zax(bt[:, 4:8])[:, 2], -1, 1))).max()
    B = z["box"]
    b0 = win(B, ps - 0.2, ps)[-1]
    bend = win(B, t_push_end - 0.1, t_push_end + 0.05)[-1]
    r["travel"] = -(bend[2] - b0[2]) * 1e3
    r["lift"] = (win(B, ps, t_tail)[:, 3].max() - b0[3]) * 1e3
    bf = z["base_truth"][-1]
    r["final_tilt"] = float(np.degrees(np.arccos(np.clip(zax(bf[None, 4:8])[0, 2], -1, 1))))
    return r


def main():
    tags = sys.argv[1:] or ["v75_n1", "v75_heavy", "h_heavy", "h_heavy2", "v75_stop", "h_stop"]
    print(f"{'run':10s} {'outcome':48s} {'lag':>6s} {'shift':>13s} {'bare':>6s} {'q3min':>6s} {'q2max':>6s} "
          f"{'swing':>6s} {'tilt':>5s} {'trav':>6s} {'lift':>5s} {'endtilt':>7s}")
    for t in tags:
        try:
            r = one(t)
        except SystemExit as e:
            print(f"{t:10s} {e}")
            continue
        if "note" in r:
            print(f"{t:10s} {r['note']}")
            continue
        out = "completed" if r["ok"] else (f"{r['t_trip']:.1f} s {r['trip']}" if r["trip"] else "aborted (driver)")
        print(f"{t:10s} {out[:48]:48s} {r['lag']:6.1f} {r['shift_lo']:+6.1f}..{r['shift_hi']:+5.1f} {r['bare']:6.1f} "
              f"{r['q3min']:6.1f} {r['q2max']:6.1f} {r['swing']:6.2f} {r['tilt']:5.1f} {r['travel']:6.1f} "
              f"{r['lift']:5.1f} {r['final_tilt']:7.1f}")


if __name__ == "__main__":
    main()
