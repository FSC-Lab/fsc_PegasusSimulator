#!/usr/bin/env python3
"""What the SAFETY guard actually did, from an am_ee_compare_driver recording.

    PYTHONNOUSERSITE=1 /usr/bin/python3 am_guard_report.py run.npz

Reads the run's `guard_trip` mark (the driver sets it when the flight NODE
leaves DIRECT on its own) and reports, over the window that follows:
  where it stopped   horizontal travel and the settled offset from the trip point
  hold or descend    altitude at the trip, after 5 s, and at the end of the window
  the arm            joint angles at the trip and at the end, against home
  the stream         whether the planner kept publishing a whole-body reference
"""
import sys
import numpy as np

HOME = np.array([0.0, 40.0, 40.0, 0.0])


def main(path):
    d = np.load(path, allow_pickle=True)
    marks = {}
    for m in d["marks"]:
        k, v = str(m).split("="); marks[k] = float(v)
    L, js, W = d["log"], d["js"], d["wbref"]
    print(f"--- {path}  aborted={bool(d['aborted'])} {str(d['reason'])}")
    if "guard_trip" not in marks:
        print("    NO GUARD TRIP in this run (the node stayed in DIRECT until the driver "
              "asked it to leave).")
        return
    t = marks["guard_trip"]; t1 = marks.get("guard_watch_end", t + 25.0)
    print(f"    guard trip at t = {t:.2f} s (run started {marks.get('run_start', float('nan')):.2f} s, "
          f"DIRECT entered {marks.get('direct_enter', float('nan')):.2f} s)")
    j0 = np.searchsorted(L[:, 0], t)
    win = (L[:, 0] >= t) & (L[:, 0] <= t1)
    P = L[win, 1:4]; p0 = L[j0, 1:4]
    if P.shape[0] < 5:
        print("    too few samples after the trip"); return
    hor = np.linalg.norm(P[:, :2] - p0[:2], axis=1)
    print(f"    HORIZONTAL: max travel {1e3*hor.max():.0f} mm, settled (last 2 s) "
          f"{1e3*np.linalg.norm(P[-1, :2]-p0[:2]):.0f} mm from the trip point")
    z = P[:, 2]; tt = L[win, 0] - t
    z5 = z[np.searchsorted(tt, 5.0)] if tt[-1] > 5 else z[-1]
    print(f"    ALTITUDE:   {p0[2]:.3f} m at the trip -> {z5:.3f} m after 5 s -> {z[-1]:.3f} m "
          f"after {tt[-1]:.0f} s   (rate {1e3*(z[-1]-p0[2])/tt[-1]:+.0f} mm/s)")
    print(f"                {'HOLDING' if abs(z[-1]-p0[2]) < 0.15 else 'DESCENDING/CLIMBING'}")
    qw = js[(js[:, 0] >= t) & (js[:, 0] <= t1)]
    if qw.shape[0] > 5:
        q0 = np.degrees(qw[0, 1:5]); q1 = np.degrees(qw[-1, 1:5])
        print(f"    ARM:        {np.round(q0,1)} deg at the trip -> {np.round(q1,1)} deg; "
              f"|q_end - home| = {np.abs(q1-HOME).max():.1f} deg")
    ws = W[(W[:, 0] > t + 1.0) & (W[:, 0] <= t1)]
    print(f"    PLANNER:    {ws.shape[0]} whole-body reference samples more than 1 s after the trip "
          f"({'STOPPED' if ws.shape[0] == 0 else 'STILL STREAMING'})")
    qx, qy = L[win, 7], L[win, 8]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (qx * qx + qy * qy), -1, 1)))
    print(f"    TILT:       {tilt.max():.1f} deg max in the window, {tilt[-1]:.1f} deg at the end")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)
