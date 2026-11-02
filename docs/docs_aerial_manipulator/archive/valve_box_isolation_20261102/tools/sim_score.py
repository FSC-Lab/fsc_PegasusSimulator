#!/usr/bin/env python3
"""(copied 2026-11-02 into archive/valve_box_isolation_20261102 from valve_safety_fixes_20261030/tools; the log
patterns follow the 2026-11-02 changes: the jaws closed in free flight, the grasp event, the contact flag, and any
GRIP / TWO SETS line, which must never appear now.) The Isaac check of the 10-30 safety fixes (sim/chain.sh): one row per run from the
10-22 valve scorer (outcome, beam turn, in-turn offset, arm, tilt, contact force, the abort's truth),
plus what the fixes themselves logged in the nodes' own ROS log files (~/.ros/log, picked by the
run's time window):
  station (arm GS)   jaws opened at Go To Start, the heading in the close gate, the j1/j4 error damping
  planner            sway guard, arm-guard aborts that drop the room hold
  whole-body node    free-axis heading, the arm RELEASE at the SAFETY handover
  arm controller     RELEASED the arm, error-damping changes
  estimator          the mocap guard (jump gate with the EKF2-velocity centre)
    PYTHONNOUSERSITE=1 /usr/bin/python3 sim_score.py [tag ...]       (no tags = every run)
"""
import glob
import importlib.util
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "sim", "runs") + "/"
_spec = importlib.util.spec_from_file_location(
    "score22", os.path.join(HERE, "..", "..", "valve_turning_safety_20261022", "tools", "score.py"))
s = importlib.util.module_from_spec(_spec)
_argv, sys.argv = sys.argv, sys.argv[:1]
_spec.loader.exec_module(s)
sys.argv = _argv
s.RUNS = RUNS
s.cs.R = RUNS
LOG = os.path.expanduser("~/.ros/log")

# (log-file prefix, label, patterns worth showing)
SOURCES = [
    ("joint_plot_inverted_", "GS", r"Valve Turning:.*(CLOSE|open|heading|damping|ABORT|grasp|RELEASE|refus|clear)"
     r"|Static hold|grasp (on|off)|error damping|trim_target|gripper (opens|closes|CLOSE)|MOCAP GUARD: fault|not met|did not settle"),
    ("whole_body_trajectory_planner_", "PLN", r"[Ss]way|ABORT|room|anchor|q3|guard|refus|grasp|contact phase|settled|WORLD"),
    ("autopilot_whole_body_l1_direct_actuation_node_", "NODE",
     r"RELEASE|SAFETY|[Ww]atchdog|WATCHDOG|EMERGENCY|LEFT OFFBOARD|left DIRECT|switch|GRIP|TWO SETS|CONTACT|contact phase"
     r"|free-axis anchor (LATCHED|released)|EE reference anchor|HEADING"),
    ("ros2_control_node_", "ARM", r"RELEASE|error damping|passthrough_error_damping|trim_target|hold"),
    ("indoor_mocap_feedback_node_", "EST", r"[Gg]uard|LOST|CONFIRMED|recover|jump"),
]


def window(tag):
    """[start, end] epoch s of a run: the .args file is written as it starts, the console log last."""
    a = os.path.getmtime(RUNS + tag + ".args")
    e = os.path.getmtime(RUNS + tag + ".console.log")
    return a - 10.0, e + 5.0


def logs(tag):
    t0, t1 = window(tag)
    out = []
    for pre, lab, pat in SOURCES:
        for f in sorted(glob.glob(os.path.join(LOG, pre + "*.log"))):
            ms = re.search(r"_(\d{13})\.log$", f)
            if not ms or not (t0 <= int(ms.group(1)) / 1e3 <= t1):
                continue
            for line in open(f, errors="replace"):
                if re.search(pat, line) and "Valve Turning: the gripper closes within" not in line \
                        and "starts by itself" not in line and "4D attribution" not in line:
                    m = re.match(r"\[(\w+)\] \[(\d+\.\d+)\] \[[^\]]*\]: (.*)", line.strip())
                    if m:
                        txt = re.sub(r"\x1b\[[0-9;]*m", "", m.group(3))
                        out.append((float(m.group(2)), lab, m.group(1)[0], txt))
    out.sort()
    return out


def main(tags):
    for t in tags:
        try:
            print(s.cs.row(t))
            ta = s.truth_after(t)
            if ta:
                print(f"{'':16s}{ta}")
            for x in s.extra(t):
                print(f"{'':16s}{x}")
            ev = logs(t)
            tz = ev[0][0] if ev else 0.0
            for tt, lab, lvl, txt in ev:
                print(f"{'':16s}{tt - tz:7.2f} {lab:4s} {lvl} {txt[:160]}")
        except Exception as exc:
            print(f"{t:16s} ERROR {exc}")
        print()


if __name__ == "__main__":
    main(sys.argv[1:] or sorted(os.path.basename(p)[:-4] for p in glob.glob(RUNS + "*.npz")))
