#!/usr/bin/env python3
"""Score the q3 rate-guard validation flights (v_*) beside last night's flights of the same
configuration and disturbance seed (section 6 of the README).

    PYTHONNOUSERSITE=1 /usr/bin/python3 validate_q3_rate.py

Per run: how it ended (completed / which guard aborted / the node left DIRECT), the guard's
reading at the trip, the lowest q3 after it, the largest tilt after it, and whether the vehicle
then stayed in DIRECT and landed. For completed runs: the engaged window's q3 minimum and the
minimum of the rate rule's prediction q3 + 0.2 s * q3_dot (its margin to 10 deg).
"""
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import q3_rate_guard_replay as R   # noqa: E402

RUNS = os.path.join(HERE, "..", "runs")
PAIRS = [   # (validation, last night's same config + seed, what it is)
    ("v_ship_5", "h_ship_5", "old config, 500 g / 0.50 m, seed 5"),
    ("v_ship_6", "z_ship_6", "old config, 500 g / 0.50 m, seed 6"),
    ("v_ship_7", "z_ship_7", "old config, 240 g / 0.10 m, seed 7"),
    ("v_ship_8", "z_ship_8", "old config, 240 g / 0.10 m, seed 8"),
    ("v_cal0_1", "g_cal0_1", "anchor off, 240 g / 0.10 m, seed 1"),
    ("v_cal0_2", None, "anchor off, 240 g / 0.10 m, seed 2"),
    ("v_fw_1", "g_fw_1", "shipped, 240 g / 0.10 m, seed 1"),
    ("v_fw_2", "g_fw_2", "shipped, 240 g / 0.10 m, seed 2"),
    ("v_fw_6", "z_final_6", "shipped, 500 g / 0.50 m, seed 6"),
]


def ev(z, pat, after=-1e9):
    for e in z["events"]:
        s = str(e)
        m = re.match(r"\[\s*([0-9.]+)s\]", s)
        if m and float(m.group(1)) > after and pat in s:
            return float(m.group(1)), s
    return None, None


def tilt(z, t0, t1):
    BT = z["base_truth"]
    w = (BT[:, 0] >= t0) & (BT[:, 0] <= t1)
    if not w.any():
        return np.nan
    q = BT[w][:, 4:8]
    wq, x, y, _ = q.T
    zz = 1 - 2 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(zz, -1, 1))).max()


def one(tag):
    p = os.path.join(RUNS, tag + ".npz")
    if not os.path.isfile(p):
        return None
    z = np.load(p, allow_pickle=True)
    mk = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
    J = z["joints"]
    t, q3 = J[:, 0], np.degrees(J[:, 3])
    # landed = the mission's Land step, or the driver's own landing after an abort (disarmed)
    r = {"tag": tag, "landed": "land:end" in mk or ev(z, "disarm: success=True")[0] is not None}
    t_ab, s_ab = ev(z, "ABORTING (SAFETY GUARD")
    t0 = mk.get("ready:end", mk.get("direct", 0.0))
    t_nd, _ = ev(z, "push_pull: NOT IN DIRECT", after=t0)
    # the driver's own landing switches to SAFETY (set_direct_mode(false)) -- not a loss of DIRECT
    t_cmd, _ = ev(z, "set_direct_mode(false)", after=t0)
    if t_nd is not None and t_cmd is not None and abs(t_cmd - t_nd) < 0.5:
        t_nd = None
    if t_ab is not None:
        m = re.search(r"SAFETY GUARD: (.*?)\)\s*--", s_ab) or re.search(r"SAFETY GUARD: (.*)", s_ab)
        g = m.group(1) if m else s_ab
        r["by"] = ("q3 RATE" if "falling at" in g else "q3 static" if g.startswith("q3 ") else
                   g.split(" ")[0] + " " + g.split(" ")[1] if " " in g else g)
        r["reading"] = g[:95]
        r["q3_trip"] = np.interp(t_ab, t, q3)
        after = (t >= t_ab) & (t <= t_ab + 8.0)
        r["q3_min_after"] = q3[after].min() if after.any() else np.nan
        r["tilt_after"] = tilt(z, t_ab, t_ab + 8.0)
        r["left_direct"] = t_nd is not None and t_nd > t_ab
    elif t_nd is not None and not r["landed"]:
        r["by"] = "LEFT DIRECT"
        r["reading"] = "node watchdog (no planner guard first)"
        r["q3_trip"] = np.interp(t_nd, t, q3)
        after = (t >= t_nd - 1.0) & (t <= t_nd + 3.0)
        r["q3_min_after"] = q3[after].min()
        r["tilt_after"] = tilt(z, t_nd - 1.0, t_nd + 8.0)
        r["left_direct"] = True
    else:
        r["by"] = "completed" if r["landed"] else "other: " + str(z["reason"])[:60]
        t1 = mk.get("exit:start", t[-1])
        w = (t >= t0) & (t <= t1)
        qd = R.rate(t, q3)
        r["q3_min"] = q3[w].min() if w.any() else np.nan
        r["p_min"] = (q3 + 0.2 * qd)[w].min() if w.any() else np.nan
        r["tilt_max"] = tilt(z, t0, t1)
    return r


def fmt(r):
    if r is None:
        return "(not flown)"
    if r["by"] == "completed":
        return (f"completed, landed | contact q3 min {r['q3_min']:.1f}, prediction min {r['p_min']:.1f} deg, "
                f"tilt <= {r['tilt_max']:.1f} deg")
    if "q3_trip" in r:
        end = "LEFT DIRECT" if r["left_direct"] else ("landed" if r["landed"] else "held, not landed")
        return (f"{r['by']} at q3 {r['q3_trip']:.1f} | q3 min after {r['q3_min_after']:.1f} | tilt after "
                f"<= {r['tilt_after']:.1f} | then {end}")
    return r["by"]


def main():
    for v, old, what in PAIRS:
        print(f"\n{what}")
        print(f"  now   {v:10s} {fmt(one(v))}")
        rv = one(v)
        if rv and "reading" in rv:
            print(f"        reading: {rv['reading']}")
        if old:
            print(f"  night {old:10s} {fmt(one(old))}")


if __name__ == "__main__":
    main()
