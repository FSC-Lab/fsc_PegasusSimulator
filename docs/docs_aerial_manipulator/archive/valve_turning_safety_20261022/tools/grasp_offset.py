#!/usr/bin/env python3
"""(2026-10-22) Where the jaws closed on the fin, from Isaac's truth: the claw (claw_0) in the
ROTOR's frame (obj_0) at the gripper's close command, the component ALONG the fin (the rotor
frame's y; the fin is 40 mm long that way, 5 mm thick radially), and how far the free rotor had
turned between the start of Ready (the descent with open jaws) and the close.
    PYTHONNOUSERSITE=1 /usr/bin/python3 grasp_offset.py tag [tag ...]   (runs/ of this campaign)"""
import os
import sys
import numpy as np

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")


def yaw(q):
    return np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))


def grasp(path):
    d = np.load(path, allow_pickle=True)
    cl, box = d["claw"], d["box"]
    m = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    if "gripper_close:start" not in m:
        return None
    t = m["gripper_close:start"]
    j0 = np.argmin(abs(box[:, 0] - m["ready:start"]))
    i = np.argmin(abs(cl[:, 0] - t)); j = np.argmin(abs(box[:, 0] - t))
    p = cl[i, 1:4] - box[j, 1:4]
    y = yaw(box[j:j + 1, 4:8])[0]
    c, s = np.cos(-y), np.sin(-y)
    turned = np.degrees((y - yaw(box[j0:j0 + 1, 4:8])[0] + np.pi) % (2 * np.pi) - np.pi)
    reason = str(d["reason"]) if bool(d["aborted"]) else "complete"
    return 1e3 * (s * p[0] + c * p[1]), 1e3 * (c * p[0] - s * p[1]) - 160.0, turned, reason


if __name__ == "__main__":
    print(f"{'run':16s} {'along fin':>9s} {'radial':>7s} {'rotor turned':>12s}   outcome")
    for t in sys.argv[1:]:
        r = grasp(os.path.join(RUNS, t + ".npz"))
        if r is None:
            print(f"{t:16s} (no close)")
            continue
        a, rad, tu, why = r
        print(f"{t:16s} {a:+8.1f}mm {rad:+6.1f}mm {tu:+10.1f}deg   {why[:90]}")
