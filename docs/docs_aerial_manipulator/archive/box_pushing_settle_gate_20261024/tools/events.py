#!/usr/bin/env python3
"""The 2026-10-24 evening changes, read off each new flight (runs/sg_s<seed>):
  - the planner's hand-over settle, from the driver's push_pull status log: Ready's end
    ("DONE ready"), how long the status said "settling", and the claw's error / speed at the
    hand-over (the last settling status);
  - the arm GS's side, from its own log (~/.ros/log/joint_plot_inverted_*.log, one per flight, in
    flight order): Ready refusals "still moving", the hand-over, the close, and the calm exit.
    PYTHONNOUSERSITE=1 /usr/bin/python3 events.py [gs_log ...]   (default: the newest logs, one per run)"""
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")
ORDER = ["sg_s2", "sg_s5", "sg_s1", "sg15_s1"]   # queue order = GS log order


def driver(tag):
    p = os.path.join(RUNS, tag + ".log")
    if not os.path.exists(p):
        return None
    ev = []
    for ln in open(p, errors="replace"):
        m = re.match(r"\[\s*([0-9.]+)s\] push_pull: (.*)", ln)
        if m:
            ev.append((float(m.group(1)), m.group(2).strip()))
    t_done = next((t for t, s in ev if s.startswith("DONE ready")), None)
    sett = [(t, s) for t, s in ev if s.startswith("DONE ready") and "settling" in s]
    t_hand = None
    if sett:
        t_hand = next((t for t, s in ev if t > sett[-1][0] and "settling" not in s), None)
    last = sett[-1][1] if sett else ""
    m = re.search(r"settling: the claw (\d+) mm from the grasp point, (\d+) mm/s", last)
    return dict(done=t_done, n_settle=len(sett), hand=t_hand,
                last_err=int(m.group(1)) if m else None, last_v=int(m.group(2)) if m else None)


def gs(path):
    keep = ("still moving", "held in the room", "gripper CLOSE", "did not settle", "the exit starts",
            "calm", "not in the close gate", "ABORT", "WRONG", "nothing in the jaws")
    out = []
    for ln in open(path, errors="replace"):
        if any(k in ln for k in keep):
            out.append(re.sub(r"^\[[^]]*\] \[[^]]*\] ", "", ln.strip())[:200])
    return out


logs = sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/.ros/log/joint_plot_inverted_*.log")),
                              key=os.path.getmtime)[-len(ORDER):]
for tag, lg in zip(ORDER, logs):
    d = driver(tag)
    print(f"== {tag}   (GS log {os.path.basename(lg)})")
    if d:
        hs = (d["hand"] - d["done"]) if (d["hand"] and d["done"]) else None
        print(f"  Ready done at {d['done']}, settling statuses {d['n_settle']}, hand-over "
              f"{'%.2f s after Ready' % hs if hs is not None else 'not seen'}; last settling status: "
              f"claw {d['last_err']} mm, {d['last_v']} mm/s")
    for x in gs(lg):
        print("  GS:", x)
    print()
