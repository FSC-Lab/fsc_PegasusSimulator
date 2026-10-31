#!/usr/bin/env python3
"""2026-10-17, the release-window fix: the old Exit To Push sequence against the new one.

    PYTHONNOUSERSITE=1 /usr/bin/python3 release_score.py [--csv out.csv]

Per run, over the RELEASE WINDOW = from the gripper OPEN command (release:start) to
the end of the exit's climb (exit:start + push_pull_exit_time_s 3.0 s, before the
arm folds home):

  open->exit  the time from the open command to the exit's start [s]
  q2 max      q2's peak [deg] (its stop is +45; the push pose holds 32)
  q3 min      q3's lowest [deg] (the planner's guard aborts below 10; through 0 is
              the elbow-singular branch; the push pose holds 28)
  arm dev     the largest |q - push pose| of q2 / q3 [deg] -- how much of the base's
              motion the arm absorbed
  base drift  the range of the airframe's horizontal position [mm] (truth)
  tilt        the airframe's peak tilt [deg]
  box moved   the box's horizontal travel from the open command to the end of the
              exit [mm] (an open claw catching the fin drags it)
  outcome     completed / ABORTED (what stopped it) / FLIPPED

Groups: OLD = the 10-16 final configuration with the old sequence (open; the
world-held claw until the exit 3 s later): the 10-16 s3_P pushes, the 10-17 control
pushes m_nom_1/2, the 10-17 results campaign (3 kept + the crashed attempt); NEW =
this campaign's rel_* runs (release, open, exit on the open report).
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
C = os.path.abspath(os.path.join(HERE, ".."))
A = os.path.abspath(os.path.join(C, ".."))
REPO = os.path.abspath(os.path.join(A, "..", "..", ".."))
EXIT_CLIMB_S = 3.0
POSE = {2: 32.0, 3: 28.0}     # the push pose's q2 / q3 [deg]

OLD = [("10-16 s3_P", os.path.join(A, "box_pushing_campaign_20261016", "runs", f"s3_P_{k}.npz")) for k in (1, 2, 3)]
OLD += [("10-17 control", os.path.join(A, "push_cruise_20261017", "runs", f"m_nom_{k}.npz")) for k in (1, 2)]
OLD += [("10-17 results", os.path.join(REPO, "results", "simulation_results", "Isaac_simulation_data", "box_pushing",
                                        f"whole_body_l1_push_run{k}.npz")) for k in (1, 2, 3)]
OLD += [("10-17 results", os.path.join(REPO, "results", "simulation_results", "Isaac_simulation_data", "box_pushing", "failed",
                                        "whole_body_l1_push_run3_attempt1", "results_r3_a1.npz"))]


def marks(z):
    return {str(n): float(t) for t, n in zip(z["marks_t"], z["marks_name"])}


def tilt_deg(qwxyz):
    x, y = qwxyz[:, 1], qwxyz[:, 2]
    return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


def score(path):
    z = np.load(path, allow_pickle=True)
    m = marks(z)
    names = [str(n) for n in z["joint_names"]]
    col = {i: 1 + names.index(f"joint{i}") for i in range(1, 5)}
    j, bt, box = z["joints"], z["base_truth"], z["box"]
    a = m["release:start"]
    b = m["exit:start"] + EXIT_CLIMB_S if "exit:start" in m else a + 6.0
    jm = (j[:, 0] >= a) & (j[:, 0] <= b)
    q2, q3 = np.degrees(j[jm, col[2]]), np.degrees(j[jm, col[3]])
    bm = (bt[:, 0] >= a) & (bt[:, 0] <= b)
    tilt = tilt_deg(bt[bm, 4:8])
    xy = bt[bm, 1:3]
    e = m.get("exit:end", b)
    k0 = np.searchsorted(box[:, 0], a)
    k1 = min(np.searchsorted(box[:, 0], e), len(box) - 1)
    aborted = bool(z["aborted"]) if "aborted" in z.files else False
    reason = str(z["reason"]) if "reason" in z.files else ""
    all_tilt = tilt_deg(bt[bt[:, 0] >= a, 4:8])
    outcome = "completed"
    if aborted:
        outcome = "FLIPPED" if all_tilt.max() > 90 else "ABORTED"
        if "GUARD" in reason:
            outcome += " (" + reason.split("SAFETY GUARD: ")[-1].split(" (")[0].split(")")[0] + ")"
    return dict(
        open_to_exit=(m["exit:start"] - a) if "exit:start" in m else np.nan,
        q2max=q2.max(), q3min=q3.min(),
        arm_dev=max(np.abs(q2 - POSE[2]).max(), np.abs(q3 - POSE[3]).max()),
        drift=1e3 * np.ptp(np.linalg.norm(xy - xy[0], axis=1)) if len(xy) else np.nan,
        tilt=tilt.max() if len(tilt) else np.nan,
        box=1e3 * np.linalg.norm(box[k1, 1:3] - box[k0, 1:3]),
        outcome=outcome,
        release_at_open=bool(z["release_at_open"]) if "release_at_open" in z.files else False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(C, "release_score.csv"))
    a = ap.parse_args()
    groups = [("OLD", g, p) for g, p in OLD if os.path.isfile(p)]
    groups += [("NEW", "10-17 release", p) for p in sorted(glob.glob(os.path.join(C, "runs", "*.npz")),
                                                           key=lambda s: (len(s), s))]
    rows = []
    hdr = (f"{'run':22s} {'seq':4s} {'group':14s} {'open->exit':>10s} {'q2 max':>7s} {'q3 min':>7s} "
           f"{'arm dev':>7s} {'drift':>6s} {'tilt':>5s} {'box':>5s}  outcome")
    print(hdr)
    print("-" * len(hdr))
    for seq, g, p in groups:
        r = score(p)
        name = os.path.basename(p)[:-4]
        rows.append(dict(run=name, seq=seq, group=g, **r))
        print(f"{name:22s} {seq:4s} {g:14s} {r['open_to_exit']:10.2f} {r['q2max']:7.1f} {r['q3min']:7.1f} "
              f"{r['arm_dev']:7.1f} {r['drift']:6.1f} {r['tilt']:5.1f} {r['box']:5.1f}  {r['outcome']}")
    print()
    for seq in ("OLD", "NEW"):
        rr = [r for r in rows if r["seq"] == seq]
        if not rr:
            continue
        ok = [r for r in rr if r["outcome"] == "completed"]
        f = lambda k, fn: fn([r[k] for r in rr])   # noqa: E731
        print(f"{seq}: {len(rr)} runs, {len(ok)} completed; q2 max <= {f('q2max', max):.1f}, "
              f"q3 min >= {f('q3min', min):.1f} (median {np.median([r['q3min'] for r in rr]):.1f}), "
              f"arm dev median {np.median([r['arm_dev'] for r in rr]):.1f} / max {f('arm_dev', max):.1f} deg, "
              f"box moved median {np.median([r['box'] for r in rr]):.1f} / max {f('box', max):.1f} mm, "
              f"tilt max {f('tilt', max):.1f} deg")
    with open(a.csv, "w") as fh:
        keys = list(rows[0].keys())
        fh.write(",".join(keys) + "\n")
        for r in rows:
            fh.write(",".join(f"{r[k]:.3f}" if isinstance(r[k], float) else str(r[k]) for k in keys) + "\n")
    print("wrote", a.csv)


if __name__ == "__main__":
    main()
