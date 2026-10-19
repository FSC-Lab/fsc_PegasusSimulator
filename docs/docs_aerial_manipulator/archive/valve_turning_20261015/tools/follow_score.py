#!/usr/bin/env python3
"""Score valve-turning runs for the PATH-FOLLOWING comparison (2026-10-18, README 11).

    PYTHONNOUSERSITE=1 /usr/bin/python3 follow_score.py runs/<tag>.npz [...]

One line per run: outcome, beam rotation, the push step's duration (scheduled 37.5 s),
and over the turn (push start -> push end or the last DIRECT sample): the planned-minus-
measured valve lag, the tilt, the wrist excursion |q4 - q4 planned|, the arm's margins to
its stops (q2 [-20, 45], q3 [10, 50]) and the contact force (the PhysX truth on the rotor);
for a governed run also the progress rate's minimum, the time it ran slowed (< 0.95) and
held (< 0.02), and the limiting channels. The recorded arrival rows are [t, arrival[0..12],
step, follow[0..7]] with follow = active, rate, progress, lead, tilt, wrist, limit, held.
"""
import sys

import numpy as np

LIMIT = {0: "none", 1: "lead", 2: "tilt", 3: "wrist", 4: "stale"}


def yaw_of(q):
    w, x, y, z = q.T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_of(q):
    w, x, y, z = q.T
    return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


def score(path):
    d = np.load(path, allow_pickle=True)
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    box, ref, odom, J, arr = d["box"], d["ref"], d["odom"], d["joints"], d["arrival"]
    name = path.split("/")[-1].replace(".npz", "")
    ab = bool(d["aborted"])
    out = {"name": name, "outcome": ("ABORT: " + str(d["reason"])[:70]) if ab else "complete"}
    if "push:start" not in marks or not len(ref):
        out["note"] = "no turn"
        return out
    axis = box[0, 1:4]
    beam = np.degrees(np.unwrap(yaw_of(box[:, 4:8])))
    rb = np.degrees(np.unwrap(np.arctan2(ref[:, 2] - axis[1], ref[:, 1] - axis[0])))
    t0 = marks["push:start"]
    direct = odom[:, 11] > 0.5
    t_end = odom[direct, 0].max() if direct.any() else t0
    t1 = min(marks.get("push:end", t_end), t_end)
    out["push_s"] = marks["push:end"] - t0 if "push:end" in marks else float("nan")
    tt = np.arange(t0, t1, 0.02)
    plan = np.interp(tt, ref[:, 0], rb) - np.interp(t0, ref[:, 0], rb)
    meas = np.interp(tt, box[:, 0], beam) - np.interp(t0, box[:, 0], beam)
    out["turned"] = meas[-1] if len(meas) else 0.0
    lag = plan - meas
    q = np.degrees(np.column_stack([np.interp(tt, J[:, 0], J[:, k]) for k in (1, 2, 3, 4)]))
    q4d = np.degrees(np.interp(tt, ref[:, 0], ref[:, 10]))   # ref cols 7..10 = q_d
    tilt = np.interp(tt, odom[:, 0], tilt_of(odom[:, 7:11]))
    out.update(lag_p95=np.percentile(lag, 95), lag_max=lag.max(), tilt_p95=np.percentile(tilt, 95),
               tilt_max=tilt.max(), wrist_max=np.abs(q[:, 3] - q4d).max(),
               q2_margin=np.minimum(q[:, 1] + 20.0, 45.0 - q[:, 1]).min(),
               q3_margin=np.minimum(q[:, 2] - 10.0, 50.0 - q[:, 2]).min(),
               stop_frac=np.mean((q[:, 1] < -19.0) | (q[:, 1] > 44.0) | (q[:, 2] > 49.0)))
    if "contact" in d.files and len(d["contact"]):
        c = d["contact"]
        m = (c[:, 0] >= t0) & (c[:, 0] <= t1)
        if m.any():
            f = np.linalg.norm(c[m, 1:4] + c[m, 4:7], axis=1)   # normal + friction, on the rotor
            out.update(force_p95=np.percentile(f, 95), force_max=f.max())
    if arr.shape[1] >= 23:
        m = (arr[:, 0] >= t0) & (arr[:, 0] <= t1) & (arr[:, 15] > 0.5)
        if m.any():
            r = arr[m]
            dt = np.diff(r[:, 0], prepend=r[0, 0])
            lim = r[:, 21].astype(int)
            out.update(governed=True, rate_min=r[:, 16].min(), slowed_s=float(np.sum(dt[r[:, 16] < 0.95])),
                       held_s=float(np.sum(dt[r[:, 16] < 0.02])),
                       limits=", ".join(f"{LIMIT[k]} {np.sum(dt[lim == k]):.1f}s" for k in sorted(set(lim)) if k))
    return out


def main():
    rows = [score(p) for p in sys.argv[1:]]
    for r in rows:
        print(f"{r['name']:12s} {r['outcome']}")
        if "note" in r:
            print(f"   {r['note']}")
            continue
        print(f"   turned {r['turned']:+6.1f} deg, push step {r.get('push_s', float('nan')):5.1f} s | lag p95/max "
              f"{r['lag_p95']:+5.1f}/{r['lag_max']:+5.1f} deg | tilt p95/max {r['tilt_p95']:4.1f}/{r['tilt_max']:4.1f} | "
              f"wrist max {r['wrist_max']:5.1f} | margins q2 {r['q2_margin']:+5.1f} q3 {r['q3_margin']:+5.1f}, on a stop "
              f"{100 * r['stop_frac']:4.1f} % | force p95/max {r.get('force_p95', float('nan')):4.2f}/"
              f"{r.get('force_max', float('nan')):5.2f} N")
        if r.get("governed"):
            print(f"   governed: rate min {r['rate_min']:.2f}, slowed {r['slowed_s']:.1f} s, held {r['held_s']:.1f} s"
                  f"{'; limited by ' + r['limits'] if r['limits'] else ''}")


if __name__ == "__main__":
    main()
