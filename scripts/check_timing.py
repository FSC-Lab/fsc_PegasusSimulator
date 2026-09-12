#!/usr/bin/env python3
"""Verify that the control loop met its deadline, from a rosbag.

WHY THIS EXISTS
    Offline `--test` cannot see HIL timing at all: it runs the solver as fast as it likes with
    no Isaac, no PX4 and no DDS competing for CPU. MAX_ITER=50 cost nothing offline and 130-159 ms
    in flight against a 100 ms budget -- and an overrunning loop is a documented tumble cause in
    this project. So timing has to be measured from the flight, not predicted from the bench.

WHAT IT MEASURES
    * SOLVE time: what IPOPT took, parsed from the controller's own status line.
    * LOOP PERIOD: the interval between consecutive actuator publications. The node publishes
      once per control tick, so these deltas ARE the realised loop period -- the quantity that
      actually matters. A slow solve is harmless if the loop still hits its deadline; a loop that
      slips is dangerous even if each individual solve looks acceptable, because `latency_s` is
      tuned for a FIXED rate and silently under-compensates once the rate drops.
    * DEADLINE MISSES: ticks whose period exceeded the nominal budget.

Usage:  check_timing.py <bagdir> [nominal_hz]
"""
import glob
import math
import re
import sqlite3
import sys


def pct(xs, p):
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(len(s) * p))]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    bag = sys.argv[1].rstrip("/")
    # DEFAULT 10 Hz: restart_hil.sh launches the node with `-p rate_hz:=10.0` (lowered from
    # 15 after measuring that it only achieved 10.0). Budget is therefore 100 ms. Measuring
    # against the wrong budget makes a 2x overrun look like a 1.3x one.
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    budget_ms = 1000.0 / hz

    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    db = sorted(glob.glob(bag + "/*.db3"))
    if not db:
        print(f"  no .db3 in {bag}")
        return 1
    con = sqlite3.connect(db[0])
    tp = {r[0]: (r[1], r[2]) for r in con.execute("select id,name,type from topics")}

    def stamps(name):
        ids = [k for k, v in tp.items() if v[0] == name]
        if not ids:
            return []
        return [r[0] for r in con.execute(
            f"select timestamp from messages where topic_id={ids[0]} order by timestamp")]

    # --- solve time, and the window during which the NMPC was actually flying ---
    solve, t_first, t_last = [], None, None
    ids = [k for k, v in tp.items() if v[0] == "/rosout"]
    if ids:
        T = get_message(tp[ids[0]][1])
        for t, d in con.execute(f"select timestamp,data from messages where topic_id={ids[0]} order by timestamp"):
            m = deserialize_message(d, T)
            if m.msg.startswith("FULL "):
                mm = re.search(r"(\d+)ms", m.msg)
                if mm:
                    solve.append(int(mm.group(1)))
                if t_first is None:
                    t_first = t
                t_last = t

    # --- realised loop period, from actuator publications ---
    ts = stamps("/uav_0/fmu/in/actuator_motors")
    # ★ SCOPE BY THE NMPC'S ACTIVE WINDOW, not by gap size.
    # Both controllers run; only the ACTIVE one publishes, the safe one at 200 Hz and the NMPC at
    # `hz`. Filtering gaps by magnitude looks equivalent and is not: when a trial crashes early
    # the NMPC phase is short, every remaining gap is the safe controller's 5 ms, and the filter
    # discards the entire flight ("no actuator stream to measure" on a bag holding 20843
    # actuator messages). Take the window between the controller's first and last status line.
    if t_first is not None and t_last is not None and t_last > t_first:
        ts = [t for t in ts if t_first <= t <= t_last]
    gaps = [(ts[i + 1] - ts[i]) / 1e6 for i in range(len(ts) - 1)]     # ms
    nmpc_gaps = [g for g in gaps if g > 1.0]                           # drop duplicate stamps
    print(f"  bag: {bag.split('/')[-1]}")
    print(f"  nominal {hz:.0f} Hz -> budget {budget_ms:.0f} ms per tick")
    if solve:
        over = sum(1 for x in solve if x > budget_ms)
        print(f"  SOLVE  n={len(solve):4d}  median {pct(solve,0.5):5.0f}  p95 {pct(solve,0.95):5.0f}  "
              f"max {max(solve):5.0f} ms   over budget: {over} ({100.0*over/len(solve):.0f}%)")
    if nmpc_gaps:
        miss = sum(1 for g in nmpc_gaps if g > budget_ms * 1.25)
        eff = 1000.0 / (sum(nmpc_gaps) / len(nmpc_gaps))
        print(f"  LOOP   n={len(nmpc_gaps):4d}  median {pct(nmpc_gaps,0.5):5.0f}  "
              f"p95 {pct(nmpc_gaps,0.95):5.0f}  max {max(nmpc_gaps):5.0f} ms")
        print(f"         effective rate {eff:.1f} Hz (nominal {hz:.0f})   "
              f"deadline misses >25% late: {miss} ({100.0*miss/len(nmpc_gaps):.0f}%)")
        verdict = "OK" if (pct(nmpc_gaps, 0.95) <= budget_ms * 1.25 and eff > hz * 0.8) else "LOOP IS SLIPPING"
        print(f"  VERDICT: {verdict}")
    else:
        print("  no actuator stream to measure")
    return 0


if __name__ == "__main__":
    sys.exit(main())
