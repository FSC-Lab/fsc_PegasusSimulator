#!/usr/bin/env python3
"""(copied 2026-10-24 evening from ../../box_pushing_guard_gate_20261024/tools) The hand-over settle
(planner push_pull_handle_settle_* + push_pull_ready_max_speed / _settle_s) and the arm GS's calm exit,
against the 10-24 MORNING runs on the same force seeds (../../box_pushing_guard_gate_20261024/runs
gg_s<seed>: the q2-aware rate rule + the GS close gate, q3 floor 5 deg then). New = runs/sg_s<seed>.
Seeds 1, 2 at 240 g / 0.10 m, 5 at 500 g / 0.50 m.

Columns: outcome; held = the open-jaw room hold before the grasp [s]; at the close: the claw's
distance to the grasp point, across the fin [mm] and its speed as the GS measures it (|change of
arrival_error [3..5]| over the newest sample >= 0.3 s old) [mm/s]; q3 min in READY / the push /
the release, q2 max over READY + push [deg]; the push's box travel [mm]; the vehicle's tilt max over
the push [deg]. phase_metrics.py defines the windows.
    PYTHONNOUSERSITE=1 /usr/bin/python3 compare.py"""
import io
import os
import contextlib
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
with contextlib.redirect_stderr(io.StringIO()):
    import phase_metrics as PMx   # noqa: E402

NEW = os.path.join(HERE, "..", "runs")
OLD = os.path.join(HERE, "..", "..", "box_pushing_guard_gate_20261024", "runs")
BASE = {2: "gg_s2", 5: "gg_s5", 1: "gg_s1"}


def outcome(path):
    """complete, or the abort's cause; '+ LEFT DIRECT' when the node left DIRECT mid-mission (every
    flight's log also says NOT IN DIRECT before DIRECT entry and after the landing)"""
    import re
    z = np.load(path, allow_pickle=True)
    names = [str(n) for n in z["marks_name"]]
    mk = dict(zip(names, map(float, z["marks_t"])))
    lines = open(path[:-4] + ".log", errors="replace").read().splitlines() \
        if os.path.exists(path[:-4] + ".log") else []
    ev = []
    for ln in lines:
        m = re.match(r"\[\s*([0-9.]+)s\] (.*)", ln)
        if m:
            ev.append((float(m.group(1)), m.group(2)))
    t_dir = mk.get("direct", 0.0)
    t_abd = min([t for t, x in ev if "push_pull: ABORTED" in x] or [np.inf])
    t_end = mk.get("land:end", np.inf)
    left = any("NOT IN DIRECT" in x and t_dir + 1.0 < t < min(t_abd, t_end) for t, x in ev)
    if not bool(z["aborted"]) and "go_to_land_start:end" in names:
        res = "complete"
    else:
        res = None
        for t, x in ev:
            if "ABORTING (SAFETY GUARD:" in x:
                res = "guard: " + x.split("SAFETY GUARD:", 1)[1].split(" (the")[0].split(")")[0].strip()
                break
        if res is None:
            for t, x in ev:
                if x.startswith("ABORT:"):
                    res = x[:46]
                    break
        if res is None:
            res = (str(z["reason"]) or "aborted")[:46]
    return res + (" + LEFT DIRECT" if left else "")


def at_close(path):
    z = np.load(path, allow_pickle=True)
    mk = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
    tc = mk.get("gripper_close:start")
    if tc is None:
        return np.nan, np.nan, np.nan
    A = z["arrival"]
    A = A[(A[:, 1] == 1) & (A[:, 2] == 1)]        # Ready's samples, the claw's target
    t, R = A[:, 0], A[:, 1:]
    k = int(np.searchsorted(t, tc)) - 1           # the sample the GS closed on
    if k < 1:
        return np.nan, np.nan, np.nan
    j = np.flatnonzero(t[:k] <= t[k] - 0.3 + 1e-3)
    v = np.linalg.norm(R[k, 3:6] - R[j[-1], 3:6]) / (t[k] - t[j[-1]]) * 1e3 if len(j) else np.nan
    return R[k, 2] * 1e3, R[k, 9] * 1e3, v


def box_moved(path):
    """how far the box moved from Ready's start to the gripper close (or to Ready's end + 3 s) [mm]"""
    z = np.load(path, allow_pickle=True)
    mk = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
    if "ready:start" not in mk:
        return np.nan
    t1 = mk.get("gripper_close:start", mk.get("ready:end", mk["ready:start"]) + 3.0)
    B = z["box"]
    p0 = B[np.searchsorted(B[:, 0], mk["ready:start"]), 1:3]
    p1 = B[min(np.searchsorted(B[:, 0], t1), len(B) - 1), 1:3]
    return np.linalg.norm(p1 - p0) * 1e3


def row(tag, path):
    if not os.path.exists(path):
        return f"{tag:10s} (not flown)"
    with contextlib.redirect_stdout(io.StringIO()):
        r = PMx.one(path)
    e, x, v = at_close(path)
    g = lambda k, f="{:5.1f}": (f.format(r[k]) if r.get(k) is not None and np.isfinite(r[k]) else "    -")  # noqa: E731
    q2 = max([r[k] for k in ("rdy_q2", "psh_q2") if r.get(k) is not None and np.isfinite(r[k])] or [np.nan])
    return (f"{tag:10s} {outcome(path):46s} {g('held','{:4.2f}'):>5s} {e:5.1f} {x:+5.1f} {v:5.0f} "
            f"{box_moved(path):5.0f} | "
            f"{g('rdy_q3')} {g('psh_q3')} {g('rel_q3')} {q2:5.1f} | {g('trav')} {g('ptilt')}")


print(f"{'run':10s} {'outcome':46s} {'held':>5s} {'|e|':>5s} {'acrs':>5s} {'v':>5s} {'box':>5s} | "
      f"{'rdyq3':>5s} {'pshq3':>5s} {'relq3':>5s} {'q2mx':>5s} | {'trav':>5s} {'tilt':>5s}")
print(f"{'':10s} {'':46s} {'[s]':>5s} {'[mm]':>5s} {'[mm]':>5s} {'mm/s':>5s} {'[mm]':>5s} | {'[deg]':>5s}")
for s in BASE:
    print(row(f"AM s{s}", os.path.join(OLD, BASE[s] + ".npz")))
    print(row(f"PM s{s}", os.path.join(NEW, f"sg_s{s}.npz")))
    print()
