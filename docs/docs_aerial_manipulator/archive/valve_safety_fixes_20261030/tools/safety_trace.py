#!/usr/bin/env python3
"""(2026-10-31) Item 3 in Isaac: what the vehicle, the arm and the jaws do after a FORCED switch to SAFETY
5 s into the turn (sim/force_safety.py), with the RELEASE (wb_arm_hold_stream false, vs_safety3) and
without it (the node streams its arm hold, vs_safety_ctl). Isaac's truth (base_truth), the arm joints,
the gripper and the contact force, from the switch (the driver's 'push_pull: NOT IN DIRECT' after the
turn started) on.
    PYTHONNOUSERSITE=1 /usr/bin/python3 safety_trace.py [tag ...]      (default vs_safety3 vs_safety_ctl)
"""
import os
import sys

import numpy as np

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "runs")


def tilt(q):
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


def run(tag, span=6.0):
    z = np.load(os.path.join(RUNS, tag + ".npz"), allow_pickle=True)
    M = dict(zip([str(n) for n in z["marks_name"]], z["marks_t"]))
    t0 = M["push:start"]
    ev = [str(e) for e in z["events"]]
    ta = next(float(e.split("s]")[0].strip("[ ")) for e in ev
              if "NOT IN DIRECT" in e and float(e.split("s]")[0].strip("[ ")) > t0)
    bt, js, G, C = z["base_truth"], z["joints"], z["grip"], z["contact"]
    w = (bt[:, 0] >= ta) & (bt[:, 0] <= ta + span)
    p0 = bt[np.searchsorted(bt[:, 0], ta), 1:4]
    tl = tilt(bt[w, 4:8])
    drift = np.linalg.norm(bt[w, 1:3] - p0[:2], axis=1)
    wj = (js[:, 0] >= ta) & (js[:, 0] <= ta + 1.0)
    q = np.degrees(js[:, 1:5])
    qa = q[np.searchsorted(js[:, 0], ta)]
    g = G[(G[:, 0] >= ta)]
    t_open = g[np.argmax(g[:, 1] > -np.radians(1.0)), 0] - ta if (g[:, 1] > -np.radians(1.0)).any() else np.nan
    c = C[(C[:, 0] >= ta)]
    t_free = c[np.argmax(np.linalg.norm(c[:, 1:4], axis=1) < 0.5), 0] - ta
    i_tl = np.argmax(tl)
    return (f"{tag:14s} switch {ta - t0:4.1f} s into the turn | tilt max {tl.max():5.1f} deg at +{bt[w][i_tl, 0] - ta:4.2f} s, "
            f"drift max {drift.max():4.2f} m, z {p0[2]:.2f} -> min {bt[w, 3].min():.2f} m | in the first 1 s: "
            f"q2 {qa[1]:+5.1f} -> min {q[wj, 1].min():+5.1f}, q3 {qa[2]:+5.1f} -> min {q[wj, 2].min():+5.1f} deg | "
            f"contact < 0.5 N at +{t_free:4.2f} s, jaws open at +{t_open:4.2f} s")


if __name__ == "__main__":
    for t in sys.argv[1:] or ["vs_safety3", "vs_safety_ctl"]:
        try:
            print(run(t))
        except Exception as exc:
            print(f"{t:14s} ERROR {exc}")
