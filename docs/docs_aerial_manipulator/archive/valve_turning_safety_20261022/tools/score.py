#!/usr/bin/env python3
"""(2026-10-22) One row per run of this campaign: the valve campaign's scorer
(valve_turning_20261015/tools/campaign_score.py: outcome, the beam's turn, the in-turn offset,
the arm, the tilt, the contact force, the abort's aftermath) pointed at this campaign's runs/,
plus the station's own timings (the automatic steps) and the guard / gate events.
    PYTHONNOUSERSITE=1 /usr/bin/python3 score.py tag [tag ...]     (no tags = every run here)
"""
import glob
import importlib.util
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs") + "/"
_spec = importlib.util.spec_from_file_location(
    "campaign_score", os.path.join(HERE, "..", "..", "valve_turning_20261015", "tools", "campaign_score.py"))
cs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cs)
cs.R = RUNS


def truth_after(tag, span=10.0):
    """Isaac's truth from the first ABORTING to +span s (the odometry is EKF2 dead-reckoning
    under a mocap fault): peak tilt, lowest altitude, horizontal travel, DIRECT kept,
    the tilt at the end of the record."""
    d = np.load(RUNS + tag + ".npz", allow_pickle=True)
    if not bool(d["aborted"]):
        return ""
    ta = None
    for e in d["events"].tolist():
        if "ABORTING" in e or "left DIRECT" in e:
            ta = float(e.split("s]")[0].strip("[ "))
            break
    if ta is None:
        return ""
    bt, od = d["base_truth"], d["odom"]
    w = (bt[:, 0] >= ta) & (bt[:, 0] <= ta + span)
    q = bt[w, 4:8]
    tl = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    qe = bt[-25:, 4:8]
    tle = np.degrees(np.arccos(np.clip(1 - 2 * (qe[:, 1] ** 2 + qe[:, 2] ** 2), -1, 1))).max()
    mo = (od[:, 0] >= ta) & (od[:, 0] <= ta + span)
    kept = bool(od[mo, 11].min() > 0.5) if mo.any() else False
    return (f"TRUTH after the abort ({span:.0f} s): tilt max {tl.max():5.1f}, z min {bt[w, 3].min():4.2f} m, "
            f"moved {np.linalg.norm(bt[w, 1:3][-1] - bt[w, 1:3][0]):4.2f} m, "
            f"{'DIRECT kept' if kept else 'DIRECT LOST'}; tilt at the record's end {tle:4.1f} deg")


def extra(tag):
    d = np.load(RUNS + tag + ".npz", allow_pickle=True)
    ev = [str(e) for e in d["events"]]
    out = []
    for e in ev:
        if "the GS's automatic" in e or "box mocap" in e or "mocap guard:" in e and "MOCAP (cause none" not in e:
            out.append(re.sub(r"^\[\s*[0-9.]+s\]\s*", "", e)[:150])
        m = re.search(r"SAFETY GUARD: ([^)]*)\)", e)
        if m and "ABORTING" in e:
            out.append("GUARD: " + m.group(1))
    return out


if __name__ == "__main__":
    tags = sys.argv[1:] or sorted(os.path.basename(p)[:-4] for p in glob.glob(RUNS + "*.npz"))
    for t in tags:
        try:
            r = cs.row(t)
            # campaign_score.py knows only the tilt / force guards; name the 2026-10-22 ones
            d = np.load(RUNS + t + ".npz", allow_pickle=True)
            why = str(d["reason"]) if bool(d["aborted"]) else ""
            for pat, lab in (("mocap CONFIRMED", "mocap guard"), ("mocap LOST", "mocap guard"),
                             ("box's mocap", "box feed  "), ("q3 ", "q3 guard  "), ("box tilt", "box tilt  ")):
                if pat in why and "watchdog   " in r:
                    r = r.replace("watchdog   ", f"{lab:11s}", 1)
                    break
            print(r)
            ta = truth_after(t)
            if ta:
                print(f"{'':16s}{ta}")
            for x in extra(t):
                print(f"{'':16s}{x}")
        except Exception as exc:
            print(f"{t:14s} ERROR {exc!r}")
