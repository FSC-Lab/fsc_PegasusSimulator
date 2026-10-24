#!/usr/bin/env python3
"""Per-phase vehicle and arm motion, 10-23 vs 10-21: the law's CoM error x_c - x_cd split along
the nose / across it (world frame, nose = the mocap yaw), the base's horizontal motion, the
joints' ranges, the box's displacement and the vehicle tilt.
Usage: /usr/bin/python3 window_stats.py"""
import numpy as np
from bp_common import Bag, tilt_deg, yaw_deg

WIN = {
    "172853": [("hover, before Plan", 11.5, 15.2), ("hover above the handle", 40.14, 41.81),
               ("Ready descent (CoM anchor)", 41.82, 45.00), ("open jaws, WORLD-held", 45.00, 47.80),
               ("jaws closing / closed", 47.80, 48.84), ("hover after the abort", 60.4, 71.2)],
    "170123": [("hover above the handle", 31.32, 33.31), ("Ready descent (CoM anchor)", 33.31, 36.50),
               ("open jaws, WORLD-held", 36.50, 39.55), ("push (settle + slide)", 39.58, 45.84),
               ("after the push, jaws closed", 45.84, 46.62)],
}


def run(tag):
    b = Bag(tag)
    wt, W = b.t("wb"), b.f("wb", "data")
    tm = b.t("mocap")
    bp = np.column_stack([b.f("mocap", f"pose.position.{c}") for c in "xyz"])
    bq = [b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"]
    yaw, tilt = np.radians(yaw_deg(*bq)), tilt_deg(*bq)
    tj = b.t("js")
    P = np.degrees(b.f("js", "position"))
    q1, q2, q3, q4 = -P[:, 2], P[:, 0], P[:, 1], -P[:, 3]
    to = b.t("obj")
    op = np.column_stack([b.f("obj", f"pose.position.{c}") for c in "xy"])
    print(f"\n=== {tag}")
    print(f"{'phase':30s} {'dur':>4s} | {'eCoM along':>10s} {'across':>8s} (rms/max mm) | {'base xy std':>11s} {'p-p':>5s} | "
          f"{'q1 range':>11s} {'q2 range':>11s} {'q3 range':>11s} {'q4 range':>11s} | {'box moved':>9s} | tilt max")
    for name, t0, t1 in WIN[tag]:
        m = (wt >= t0) & (wt <= t1) & (W[:, 0] > 0.5)
        if not m.any():
            continue
        ec = W[m, 48:50] - W[m, 45:47]
        k = np.searchsorted(tm, wt[m])
        k = np.clip(k, 0, len(tm) - 1)
        c, s = np.cos(yaw[k]), np.sin(yaw[k])
        al = (c * ec[:, 0] + s * ec[:, 1]) * 1e3
        ac = (-s * ec[:, 0] + c * ec[:, 1]) * 1e3
        mb = (tm >= t0) & (tm <= t1)
        xy = bp[mb, :2]
        std = 1e3 * np.sqrt(xy[:, 0].var() + xy[:, 1].var())
        pp = 1e3 * np.max(np.hypot(*(xy - xy.mean(0)).T)) * 2
        mj = (tj >= t0) & (tj <= t1)
        rng = lambda q: f"{q[mj].min():5.1f}..{q[mj].max():5.1f}"
        mo = (to >= t0) & (to <= t1)
        bm = 1e3 * np.hypot(*(op[mo][-1] - op[mo][0])) if mo.sum() > 1 else np.nan
        print(f"{name:30s} {t1 - t0:4.1f} | {np.sqrt(np.mean(al**2)):4.0f}/{np.abs(al).max():4.0f}  "
              f"{np.sqrt(np.mean(ac**2)):4.0f}/{np.abs(ac).max():4.0f}            | {std:11.0f} {pp:5.0f} | "
              f"{rng(q1)} {rng(q2)} {rng(q3)} {rng(q4)} | {bm:9.1f} | {tilt[mb].max():5.1f}")


for tag in ("172853", "170123"):
    run(tag)
