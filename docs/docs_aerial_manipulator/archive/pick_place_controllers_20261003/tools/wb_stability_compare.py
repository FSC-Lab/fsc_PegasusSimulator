#!/usr/bin/env python3
"""Compare pick-and-place runs step by step: roll / pitch peak-to-peak per
mission step (the driver's marks), the EE error, and the placement.

    /usr/bin/python3 wb_stability_compare.py "old:wb_1,wb_2,wb_3,wb_4_newseq" "new:wb_5_kw15,wb_6_kw15"
"""
import os
import sys

import numpy as np

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")
STEPS = ["ready_pick", "pick", "gripper_close", "exit_pick", "go_to_place_start", "ready_place",
         "place", "gripper_open", "exit_place", "go_to_land_start"]


def eul(q):
    w, x, y, z = q.T
    return (np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))))


def per_step(fn):
    d = np.load(os.path.join(RUNS, fn + ".npz"), allow_pickle=True)
    od = d["odom"]; names = list(d["marks_name"]); mt = d["marks_t"]
    r, p = eul(od[:, 7:11]); t = od[:, 0]; dirm = od[:, 11] > 0.5
    starts = [(n[:-6], tm) for n, tm in zip(names, mt) if n.endswith(":start")]
    out = {}
    for i, (s, a) in enumerate(starts):
        b = starts[i + 1][1] if i + 1 < len(starts) else a + 5
        m = dirm & (t > a) & (t < b)
        if m.sum() > 50:
            out[s] = (float(np.ptp(r[m])), float(np.ptp(p[m])))
    ee, ref = d["ee"], d["ref"]
    rp = np.stack([np.interp(ee[:, 0], ref[:, 0], ref[:, 1 + i]) for i in range(3)], axis=1)
    live = (ee[:, 0] > ref[0, 0]) & (ee[:, 0] < ref[-1, 0])
    err = np.linalg.norm(ee[:, 1:4] - rp, axis=1) * 1e3
    out["_ee_rms"] = float(np.sqrt(np.mean(err[live] ** 2)))
    out["_aborted"] = bool(d["aborted"])
    return out


def main():
    groups = []
    for arg in sys.argv[1:]:
        name, runs = arg.split(":")
        groups.append((name, [per_step(r) for r in runs.split(",")]))
    print(f"{'step':18s} " + " ".join(f"{g:>22s}" for g, _ in groups) + "    (roll / pitch p-p deg, mean over runs)")
    for s in STEPS:
        cells = []
        for _, rs in groups:
            v = [x[s] for x in rs if s in x]
            cells.append(f"{np.mean([a for a, _ in v]):5.2f} / {np.mean([b for _, b in v]):5.2f}" if v else "--")
        print(f"{s:18s} " + " ".join(f"{c:>22s}" for c in cells))
    print(f"{'EE rms (mm)':18s} " + " ".join(f"{np.mean([x['_ee_rms'] for x in rs]):>22.1f}" for _, rs in groups))
    print(f"{'aborted':18s} " + " ".join(f"{sum(x['_aborted'] for x in rs):>19d}/{len(rs)}" for _, rs in groups))


if __name__ == "__main__":
    main()
