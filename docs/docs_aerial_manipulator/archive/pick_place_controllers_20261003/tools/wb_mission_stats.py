#!/usr/bin/env python3
"""Whole-mission stability statistics of pick-and-place runs, one line each:
roll / pitch rms (detrended over 2 s, i.e. the ripple, not the planned attitude),
body-rate rms, the claw's EE error rms, and the payload SWING in the jaws (the
box's horizontal offset under the claw, lift to place-start: p-p and rms).

    /usr/bin/python3 wb_mission_stats.py "label:run1,run2" ...

Per-event peak-to-peak values scatter 1-2 deg between identical runs; these do not.
"""
import os
import sys

import numpy as np

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")


def eul(q):
    w, x, y, z = q.T
    return (np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))))


def stats(fn):
    d = np.load(os.path.join(RUNS, fn + ".npz"), allow_pickle=True)
    od = d["odom"]; names = list(d["marks_name"]); mt = d["marks_t"]
    if "go_to_start:start" not in names:
        return None
    a = mt[names.index("go_to_start:start")]
    b = mt[names.index("execute_land:end")] if "execute_land:end" in names else od[-1, 0]
    m = (od[:, 0] > a) & (od[:, 0] < b) & (od[:, 11] > 0.5)
    t = od[m, 0]; r, p = eul(od[m, 7:11])
    n = max(int(2.0 / np.median(np.diff(t))), 1); k = np.ones(n) / n

    def hp(x):
        return x - np.convolve(x, k, "same")
    rr, pp = hp(r)[n:-n], hp(p)[n:-n]
    dt = np.gradient(t); rate = np.hypot(np.gradient(r) / dt, np.gradient(p) / dt)[5:-5]
    ee, ref = d["ee"], d["ref"]
    mm = (ee[:, 0] > a) & (ee[:, 0] < b)
    rp = np.stack([np.interp(ee[mm, 0], ref[:, 0], ref[:, 1 + i]) for i in range(3)], axis=1)
    err = np.linalg.norm(ee[mm, 1:4] - rp, axis=1) * 1e3
    out = dict(roll=np.std(rr), pitch=np.std(pp), rate=np.sqrt(np.mean(rate ** 2)), ee=np.sqrt(np.mean(err ** 2)),
               aborted=bool(d["aborted"]))
    if "exit_pick:start" in names and "go_to_place_start:end" in names:
        cl, pay = d["claw"], d["payload"]
        a2 = mt[names.index("exit_pick:start")] + 0.8; b2 = mt[names.index("go_to_place_start:end")]
        c = (cl[:, 0] > a2) & (cl[:, 0] < b2); tc = cl[c, 0]
        off = np.hypot(cl[c, 1] - np.interp(tc, pay[:, 0], pay[:, 1]),
                       cl[c, 2] - np.interp(tc, pay[:, 0], pay[:, 2])) * 1e3
        out["swing_pp"], out["swing_rms"] = float(np.ptp(off)), float(np.std(off))
    return out


def main():
    print(f"{'config':22s} {'run':16s} roll rms  pitch rms  rate rms   EE rms   swing p-p / rms (mm)")
    for arg in sys.argv[1:]:
        label, runs = arg.split(":")
        for fn in runs.split(","):
            s = stats(fn)
            if s is None:
                print(f"{label:22s} {fn:16s} (no mission)"); continue
            sw = f"{s['swing_pp']:5.1f} / {s['swing_rms']:4.1f}" if "swing_pp" in s else "--"
            print(f"{label:22s} {fn:16s} {s['roll']:6.2f}   {s['pitch']:6.2f}    {s['rate']:6.2f}   {s['ee']:6.1f}   {sw}"
                  f"{'   ABORTED' if s['aborted'] else ''}")


if __name__ == "__main__":
    main()
