#!/usr/bin/env python3
"""Replay the planner's AUTONOMOUS-DROP gates (2026-10-16, fsc_trajectory_planner evalDropAuto) on a flight's
drop leg and hold, with the PAYLOAD's own mocap (/obj_0/mocap, what the planner reads) -- can the payload's
pose gate the drop, and would it have passed?

    PYTHONNOUSERSITE=1 /usr/bin/python3 drop_auto_replay.py [run ...]      (default d1)

At the planner's 10 Hz arrival tick, the gates as coded (a sample counts only when its tilt < 90 deg, i.e. not a
flipped solve; frozen = >= 3 bit-identical samples in 0.3 s; speed over >= 0.1 s):
  claw  |e| (pick_place/arrival_error) <= 20 mm, claw speed <= 0.03 m/s
  payload  obj_0 fresh (< 0.25 s), not frozen, |obj_0_xy - drop point_xy| <= 25 mm, speed <= 0.04 m/s
  (the 2 s wait after the leg, the 0.5 s dwell and the q4 gate are reported, not replayed: D-1's loaded hold was
   0.67 s, so the WAIT alone would have held it)
plus what the gates rest on:
  hang  = (obj_0 - drop point) - (claw - claw target), horizontal: where the basket hangs relative to where the
          planner's EE offset puts it. If |hang| is not small, the claw gate and the payload gate cannot both pass.
  tilt  = the basket's own tilt from level (the candidate ORIENTATION gate)
"""
import sys
import numpy as np
from pd15_common import load, leg_windows, rosout, info_goal, mocap_pose, tilt_of, RUNS

CLAW_TOL, CLAW_V, PAY_TOL, PAY_V = 0.020, 0.03, 0.025, 0.04


def main(run):
    d, t0 = load(run)
    win = leg_windows(d, t0)
    a, b = win["execute_place (approach)"]
    I = d["pp_info__data"]
    ok = np.where(np.isfinite(I[:, 14 + 7 * 3 + 4]))[0]
    info = I[ok[-1]]
    _, yaw, claw_tgt = info_goal(info, 3)
    drop_pt = info[56 + 3 * 4:56 + 3 * 4 + 4]                   # the Drop row when it was captured with Get
    if not np.all(np.isfinite(drop_pt[:3])):
        # 10-15 TYPED the Drop row, so the planner never published it. Reconstruct it as release.py does:
        # claw target = drop point + Rz(yaw) * ee_offset + [0, 0, drop height] (hardware pick-and-drop yaml)
        from release import EE_OFF, PLACE_DROP_H
        c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
        off = np.array([c * EE_OFF[0] - s * EE_OFF[1], s * EE_OFF[0] + c * EE_OFF[1], EE_OFF[2] + PLACE_DROP_H])
        drop_pt = np.r_[claw_tgt - off, 1.0]
    opens = [t for t, n, m in rosout(d, t0, contains="gripper OPEN") if t > a]
    t_open = opens[0] if opens else d["law__recv"][-1] - t0
    to, Po, Qo, _ = mocap_pose(d, t0, "obj")                    # /obj_0/mocap, as the planner gets it
    tilt = tilt_of(Qo)
    ta = d["pp_arr__recv"] - t0; A = d["pp_arr__data"]          # e = target - claw (world), |e|
    print(f"\n===== {run} {RUNS[run][0]}: drop leg {a:.2f}-{b:.2f} s, Drop clicked {t_open:.2f} s "
          f"({t_open - b:+.2f} s after the leg)   drop point {np.round(drop_pt[:3], 3)}  claw target {np.round(claw_tgt, 3)}")
    print("   t     claw |e| mm  v m/s | obj_0: age ms  frozen  |xy - drop| mm  v m/s  tilt deg | hang (obj-drop)-(claw-tgt) mm | gates failing")
    prev = None
    for k in np.where((ta >= b - 3.0) & (ta <= t_open + 0.05))[0]:
        t = ta[k]
        e = A[k, 3:6]; en = A[k, 2]
        claw = claw_tgt - e
        cv = np.nan if prev is None else np.linalg.norm(claw - prev[1]) / max(t - prev[0], 1e-3)
        prev = (t, claw)
        m = (to <= t) & (tilt < 90.0)
        if not m.any():
            continue
        j = np.where(m)[0][-1]
        age = (t - to[j]) * 1e3
        rec = (to <= to[j]) & (to >= to[j] - 0.3) & (tilt < 90.0)
        frozen = rec.sum() >= 3 and np.all(Po[rec] == Po[j])
        old = np.where((to <= to[j] - 0.1) & (tilt < 90.0))[0]
        pv = np.linalg.norm(Po[j] - Po[old[-1]]) / (to[j] - to[old[-1]]) if len(old) else np.nan
        perr = np.linalg.norm(Po[j, :2] - drop_pt[:2])
        hang = (Po[j, :2] - drop_pt[:2]) - (claw[:2] - claw_tgt[:2])
        fails = []
        if en > CLAW_TOL: fails.append("claw err")
        if not cv <= CLAW_V: fails.append("claw v")
        if age > 250 or frozen: fails.append("obj stale/frozen")
        if perr > PAY_TOL: fails.append("payload err")
        if not pv <= PAY_V: fails.append("payload v")
        tag = "leg" if t < b else ("hold" if t < t_open else "OPEN")
        print(f"  {t:6.2f} {tag:4s} {en*1e3:6.1f}  {cv:5.3f} | {age:6.0f}  {str(frozen)[0]:>6s}  {perr*1e3:12.1f}  {pv:5.3f}  "
              f"{tilt[j]:6.1f} | [{hang[0]*1e3:6.1f} {hang[1]*1e3:6.1f}] |{np.linalg.norm(hang)*1e3:5.1f}| | "
              f"{', '.join(fails) or 'ALL PASS'}")
    # obj_0's quality over the whole carry (grasp -> click): flips, frozen runs, gaps
    g0 = win[[k for k in win if k.startswith("exit_pick")][0]][0]
    mc = (to >= g0) & (to <= t_open)
    flips = (tilt[mc] >= 90.0).sum()
    gaps = np.diff(to[mc]); same = np.all(np.diff(Po[mc], axis=0) == 0, axis=1)
    run_len = max((len(s) for s in "".join("1" if x else "0" for x in same).split("0")), default=0)
    print(f"  obj_0 over the carry ({g0:.1f}-{t_open:.1f} s): {mc.sum()} samples, flipped {flips}, largest gap "
          f"{gaps.max()*1e3:.0f} ms, longest bit-identical run {run_len} samples; tilt mean {tilt[mc & (tilt<90)].mean():.1f} "
          f"p-p {np.ptp(tilt[mc & (tilt<90)]):.1f} deg (resting on the pick hat before the grasp: "
          f"{tilt[(to > a - 60) & (to < g0 - 5) & (tilt < 90)].mean():.1f})")


if __name__ == "__main__":
    for r in (sys.argv[1:] or ["d1"]):
        main(r)
