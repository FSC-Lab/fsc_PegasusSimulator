#!/usr/bin/env python3
"""(2026-11-02, Pegasus archive/valve_cup_abort_20261102) The Isaac check of the valve's HALF CLOSE and the
climb-on-open ABORT (../sim/chain.sh): the isolation's row per run (archive/valve_box_isolation_20261102/tools/
sim_score.py: outcome, beam turn, in-turn offset, arm, tilt, contact force, the abort's truth, the nodes' log
lines), plus

  CUP    the gripper's hub angle while on the handle (Isaac: open 0, the 20 mm shim stalls a full close at
         -0.40..-0.51 rad, the half close 0.012 m = -0.21 rad), the vehicle-rotor contact points and force
  TURN   the beam's rotation from the grasp to the end of the turn
  ABORT  from the ABORTING event: when the jaws read open (hub within 0.03 rad of 0), when the reference
         starts to climb (x_cd z +2 mm), the CoM-anchor weight 0.3 s in (wb_control_debug [129]: 1 = the claw
         follows the vehicle), the contact force and the beam's turn from the abort to 1 s after the climb
         started, and the claw's height gained by then

    PYTHONNOUSERSITE=1 /usr/bin/python3 cup_score.py [tag ...]       (no tags = every run)
"""
import glob
import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "sim", "runs") + "/"
_spec = importlib.util.spec_from_file_location(
    "simscore", os.path.join(HERE, "..", "..", "valve_box_isolation_20261102", "tools", "sim_score.py"))
ss = importlib.util.module_from_spec(_spec)
_argv, sys.argv = sys.argv, sys.argv[:1]
_spec.loader.exec_module(ss)
sys.argv = _argv
ss.RUNS = RUNS
ss.s.RUNS = RUNS
ss.s.cs.R = RUNS
ss.SOURCES[0] = (ss.SOURCES[0][0], ss.SOURCES[0][1], ss.SOURCES[0][2] + r"|HALF CLOSE|half close|reported to the planner")
ss.SOURCES[1] = (ss.SOURCES[1][0], ss.SOURCES[1][1], ss.SOURCES[1][2] + r"|jaws are open|READS open|climbing|follows the vehicle")

DBG_ANCHOR_W = 129 + 2   # the recorder's dbg columns are the debug index + 2


def mark(d, name):
    t = [float(tt) for tt, n in zip(d["marks_t"], d["marks_name"]) if str(n) == name]
    return t[-1] if t else None


def event_t(d, needle):
    for e in d["events"].tolist():
        if needle in e:
            return float(e.split("s]")[0].strip("[ "))
    return None


def yaw_of(q):   # box rows: [t, x, y, z, qw, qx, qy, qz, step]
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def beam_turn(d, t0, t1):
    b = d["box"]
    if not len(b) or t0 is None or t1 is None:
        return float("nan")
    yaw = np.unwrap(yaw_of(b[:, 4:8]))
    return float(np.degrees(np.interp(t1, b[:, 0], yaw) - np.interp(t0, b[:, 0], yaw)))


def force(d, t0, t1):
    c = d["contact"]
    m = (c[:, 0] >= t0) & (c[:, 0] <= t1)
    if not m.any():
        return np.zeros(1), np.zeros(1)
    return np.linalg.norm(c[m, 1:4] + c[m, 4:7], axis=1), c[m, 20]


def cup_lines(tag):
    d = np.load(RUNS + tag + ".npz", allow_pickle=True)
    out = []
    g = d["grip"]
    t_on, t_off = mark(d, "push:start"), mark(d, "push:end")
    t_ab = event_t(d, "push_pull: ABORTING")
    if t_on is not None:
        t_end = t_off if t_off is not None else (t_ab if t_ab is not None else g[-1, 0])
        if t_ab is not None:
            t_end = min(t_end, t_ab)
        m = (g[:, 0] >= t_on) & (g[:, 0] <= t_end)
        F, n = force(d, t_on, t_end)
        if m.any():
            out.append(f"CUP   hub {g[m, 1].min():+.3f}..{g[m, 1].max():+.3f} rad on the handle; contact points "
                       f"{np.mean(n > 0) * 100:3.0f}% of the time, |F| p95 {np.percentile(F, 95):4.2f} max {F.max():4.2f} N "
                       f"over {t_end - t_on:4.1f} s")
        if t_off is not None and (t_ab is None or t_ab > t_off):
            out.append(f"TURN  beam {beam_turn(d, t_on, t_off):+6.1f} deg from the grasp to the end of the turn")
    if t_ab is not None:
        mo = g[:, 0] >= t_ab
        op = g[mo & (g[:, 1] > -0.03), 0]
        t_open = float(op[0]) if len(op) else None
        # the planner's own word on the report (its log): "the jaws are open X s into the abort -- climbing now"
        said = ""
        for _, lab, _, txt in ss.logs(tag):
            if lab == "PLN" and ("into the abort" in txt or "as soon as it is planned" in txt):
                k = max(txt.find("the jaws are open"), txt.find("the gripper READS open"))
                said = txt[k if k >= 0 else 0:][:110]
                break
        r = d["ref"]
        ra = r[r[:, 0] > t_ab + 0.05]   # the abort plan (it starts at the MEASURED pose)
        zg = []
        for dt in (1.0, 2.0):
            m = ra[ra[:, 0] <= t_ab + dt] if len(ra) else ra
            zg.append(f"{(m[-1, 6] - ra[0, 6]) * 1e3:+4.0f}" if len(m) else "  --")
        db = d["dbg"]
        w = db[(db[:, 0] >= t_ab + 0.3) & (db[:, 0] <= t_ab + 0.4), DBG_ANCHOR_W]
        w03 = float(np.nanmean(w)) if len(w) else float("nan")
        F, _ = force(d, t_ab, t_ab + 2.0)
        cl = d["claw"]
        zc = [float(np.interp(t_ab + dt, cl[:, 0], cl[:, 3])) for dt in (0.0, 2.0)]
        out.append(
            "ABORT jaws read open " + (f"{t_open - t_ab:4.2f} s" if t_open is not None else "NEVER") +
            f" after ABORTING; planner: {said or 'no jaws report in its log'}")
        out.append(
            f"      reference z {zg[0]}/{zg[1]} mm at 1/2 s; CoM-anchor weight {w03:4.2f} at 0.3 s; "
            f"to 2 s: |F| max {F.max():4.2f} N, beam {beam_turn(d, t_ab, t_ab + 2.0):+5.1f} deg, "
            f"claw truth {(zc[1] - zc[0]) * 1e3:+4.0f} mm")
    return out


def main(tags):
    for t in tags:
        try:
            print(ss.s.cs.row(t))
            ta = ss.s.truth_after(t)
            if ta:
                print(f"{'':16s}{ta}")
            for x in cup_lines(t):
                print(f"{'':16s}{x}")
            for x in ss.s.extra(t):
                print(f"{'':16s}{x}")
            if os.environ.get("CUP_LOGS", "1") == "1":
                ev = ss.logs(t)
                tz = ev[0][0] if ev else 0.0
                for tt, lab, lvl, txt in ev:
                    if lab in ("GS", "PLN") and tt - tz > 10.0:
                        print(f"{'':16s}{tt - tz:7.2f} {lab:4s} {lvl} {txt[:170]}")
        except Exception as exc:
            print(f"{t:16s} ERROR {exc!r}")
        print()


if __name__ == "__main__":
    main(sys.argv[1:] or sorted(os.path.basename(p)[:-4] for p in glob.glob(RUNS + "*.npz")))
