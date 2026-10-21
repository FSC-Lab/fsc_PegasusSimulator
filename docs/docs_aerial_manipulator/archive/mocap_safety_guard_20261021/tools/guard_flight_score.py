#!/usr/bin/env python3
"""Score an Isaac box-pushing flight for the raw-mocap guard (pl_mission.py npz).

  PYTHONNOUSERSITE=1 /usr/bin/python3 -I guard_flight_score.py <run.npz> [...]

Prints: the guard's state edges (driver time), the driver's guard / abort / fault events,
and over every episode where the guard was not MOCAP:
  - the BRIDGE error: the odometry the law flew on minus the TRUE body position
    (base_truth, Isaac's own pose), less the median offset between the two while MOCAP
    (the emulator's frame vs the backend's, constant);
  - how far the TRUE body moved from where it was when the episode began;
  - the box: travel during the episode and its tilt (it must not be dragged or tipped);
  - the jaws: when the gripper opened, relative to the episode start.
"""
import sys

import numpy as np

STATES = {0: "MOCAP", 1: "BRIDGING", 2: "CONFIRMED", 3: "LOST"}


def interp(t, tt, yy):
    return np.column_stack([np.interp(t, tt, yy[:, k]) for k in range(yy.shape[1])])


def quat_tilt_deg(q):
    w, x, y, z = q.T
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(r22, -1.0, 1.0)))


def score(path):
    d = np.load(path, allow_pickle=True)
    print(f"=== {path}")
    g = np.array([r[:18] for r in d["guard"] if len(r) >= 18], float)
    od = np.asarray(d["odom"], float)
    bt = np.asarray(d["base_truth"], float)
    bx = np.asarray(d["box"], float)
    gr = np.asarray(d["grip"], float)
    print("completed:", not bool(d["aborted"]), "| reason:", str(d["reason"]))
    for e in d["events"]:
        s = str(e)
        if any(k in s for k in ("mocap", "ABORT", "abort", "SAFETY GUARD", "HOVER", "refused",
                                "gripper open", "COMPLETE", "MARK push", "fault")):
            print("  ", s[:170])
    if not len(g):
        print("no guard rows")
        return
    st = g[:, 2].astype(int)
    edges = [0] + [i for i in range(1, len(st)) if st[i] != st[i - 1]]
    print("guard edges:", ", ".join(f"{g[i, 0]:.2f}s {STATES.get(st[i], st[i])}" for i in edges))
    # odometry vs truth: the constant frame offset, measured while MOCAP
    tb = bt[:, 0]
    truth_at_odom = interp(od[:, 0], tb, bt[:, 1:4])
    st_at_odom = np.interp(od[:, 0], g[:, 0], st.astype(float), left=0, right=0)
    diff = od[:, 1:4] - truth_at_odom
    ok = st_at_odom < 0.5
    off = np.median(diff[ok], axis=0)
    print(f"odometry - truth while MOCAP: median {np.round(off * 1e3, 2)} mm, "
          f"rms around it {1e3 * np.sqrt(np.mean(np.sum((diff[ok] - off) ** 2, 1))):.2f} mm")
    # episodes
    bad = st > 0
    starts = [i for i in range(len(st)) if bad[i] and (i == 0 or not bad[i - 1])]
    for i0 in starts:
        i1 = i0
        while i1 + 1 < len(st) and bad[i1 + 1]:
            i1 += 1
        t0, t1 = g[i0, 0], g[i1, 0]
        print(f"-- episode {t0:.2f}-{t1:.2f} s ({t1 - t0:.2f} s), cause {int(g[i0, 3])}, "
              f"states {sorted(set(st[i0:i1 + 1]))}")
        w = (od[:, 0] >= t0) & (od[:, 0] <= t1)
        e = diff[w] - off
        if w.any():
            eh = np.hypot(e[:, 0], e[:, 1])
            print(f"   bridge error (odometry - truth): horizontal max {1e3 * eh.max():.1f} mm, "
                  f"at the end {1e3 * eh[-1]:.1f} mm, vertical max {1e3 * np.abs(e[:, 2]).max():.1f} mm")
        p0 = interp(np.array([t0]), tb, bt[:, 1:4])[0]
        wt = (tb >= t0) & (tb <= t1)
        mv = bt[wt, 1:4] - p0
        if wt.any():
            print(f"   true body moved: horizontal max {1e3 * np.hypot(mv[:, 0], mv[:, 1]).max():.0f} mm, "
                  f"z {1e3 * mv[:, 2].min():+.0f}..{1e3 * mv[:, 2].max():+.0f} mm")
        wb = (bx[:, 0] >= t0 - 0.5) & (bx[:, 0] <= t1 + 3.0)
        if wb.any():
            b = bx[wb]
            print(f"   box during the episode (+3 s): travel {1e3 * np.hypot(*(b[-1, 1:3] - b[0, 1:3])):.1f} mm, "
                  f"max tilt {quat_tilt_deg(b[:, 4:8]).max():.2f} deg")
        wg = (gr[:, 0] >= t0) & (gr[:, 0] <= t1 + 5.0)
        if wg.any():
            gg = gr[wg]
            k = np.argmax(gg[:, 1] > 0.005) if (gg[:, 1] > 0.005).any() else None
            if k is not None:
                print(f"   jaws open (> 5 mm) at {gg[k, 0] - t0:+.2f} s after the episode began")
    # recovery: was the box left on the table, and where did the vehicle end
    print(f"box final moved {1e3 * np.hypot(*(d['box_final'][:2] - d['box0'][:2])):.1f} mm from its start; "
          f"final tilt {quat_tilt_deg(bx[-1:, 4:8])[0]:.2f} deg")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        score(p)
