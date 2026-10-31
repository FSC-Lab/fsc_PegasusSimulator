#!/usr/bin/env python3
"""carry_fold.py <run.npz>... -- the arm fold q2+q3 over the carry leg (planned 70 -> 80 deg for the
set-down profile's carry pose [12, 38, 42, 0]), the basket's tilt there, and whether/when the basket left
the hook (claw - basket height > 0.30 m; it hangs ~0.19 m below the claw). The claw pitches
90 - (q2+q3) deg below level, so a low fold tips the hook's fingers (2026-10-26)."""
import sys
import numpy as np
for f in sys.argv[1:]:
    d = np.load(f, allow_pickle=True)
    p, c, j = d["payload"], d["claw"], d["joints"]
    mk = dict(zip(d["marks_name"], d["marks_t"]))
    if "exit_pick:end" not in mk:
        print(f"{f.split('/')[-1]:28s} never lifted the basket"); continue
    t0 = mk["exit_pick:end"]; t1 = mk.get("go_to_place_start:end", t0 + 17.0)
    hang = np.interp(p[:, 0], c[:, 0], c[:, 3]) - p[:, 3]
    w = (p[:, 0] > t0) & (p[:, 0] < t1)
    slip = p[w, 0][hang[w] > 0.30]
    ts = slip[0] if len(slip) else None
    wj = (j[:, 0] > t0) & (j[:, 0] < (ts if ts else t1))
    s = np.degrees(j[wj, 2] + j[wj, 3])
    qx, qy = p[w, 5], p[w, 6]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (qx * qx + qy * qy), -1, 1)))
    print(f"{f.split('/')[-1]:28s} carry q2+q3 mean {s.mean():5.1f} min {s.min():5.1f} deg, basket tilt p95 "
          f"{np.percentile(tilt, 95):4.1f} deg, " + (f"LEFT THE HOOK {ts - t0:.1f} s into the carry" if ts else "stayed on the hook"))
