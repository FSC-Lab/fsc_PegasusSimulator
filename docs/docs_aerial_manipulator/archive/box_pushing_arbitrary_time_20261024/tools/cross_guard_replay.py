#!/usr/bin/env python3
"""Replay the planner's push_pull_guard_box_cross (2026-10-25) on recorded pushes: the box's
horizontal displacement ACROSS the push from its pose at the push step's start, above the
threshold for 0.2 s -> trip. Prints, per threshold, the trip time and where the box was (its
centre's distance to the table's near/far side edge; the CoM is over an edge at 0) at the trip
and 0.6 s later (the jaws take ~0.6 s to open in Isaac).

    PYTHONNOUSERSITE=1 /usr/bin/python3 tools/cross_guard_replay.py runs/*.npz
"""
import sys
import numpy as np

PERSIST = 0.2
TH = (0.040, 0.045, 0.050, 0.060)


def main():
    for p in sys.argv[1:]:
        d = np.load(p, allow_pickle=True)
        n = list(d["marks_name"]); ts = d["marks_t"]
        if "push:start" not in n:
            continue
        tp = ts[n.index("push:start")]
        te = ts[n.index("exit:start")] if "exit:start" in n else np.inf
        b = d["box"]; m = (b[:, 0] >= tp) & (b[:, 0] <= te)
        if not m.any():
            continue
        t, xy = b[m, 0], b[m, 1:3]
        u = np.array([0.0, -1.0])                       # the push, world -y (the box-pushing layout)
        dp = xy - xy[0]
        cross = np.abs(dp[:, 0] * u[1] - dp[:, 1] * u[0])
        x0 = d["box0"][0]                                # the table's centre line (box placed on it)
        edge = lambda x: 0.075 - abs(x - x0)             # 150 mm table: the side edges 75 mm off the line
        row = [f"{p.split('/')[-1]:<24} max cross {1e3 * cross.max():5.1f} mm"]
        for th in TH:
            over = cross > th; since = None; trip = None
            for i in range(len(t)):
                if over[i]:
                    since = t[i] if since is None else since
                    if t[i] - since >= PERSIST:
                        trip = i; break
                else:
                    since = None
            if trip is None:
                row.append(f"{1e3 * th:.0f}: --")
            else:
                j = np.argmin(abs(t - t[trip] - 0.6))
                row.append(f"{1e3 * th:.0f}: push+{t[trip] - tp:4.1f}s edge {1e3 * edge(xy[trip, 0]):3.0f}->{1e3 * edge(xy[j, 0]):3.0f} mm")
        print(" | ".join(row))


if __name__ == "__main__":
    main()
