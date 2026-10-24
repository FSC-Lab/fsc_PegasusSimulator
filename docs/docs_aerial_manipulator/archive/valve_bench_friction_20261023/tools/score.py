#!/usr/bin/env python3
"""One row per run: the 10-22 campaign's scorer (valve_turning_safety_20261022/tools/score.py, itself
the valve campaign's campaign_score.py + the guard names + Isaac's truth after an abort), pointed at
this campaign's runs/ -- or at the 10-22 runs/ with --ref, for the paired 0.048 N.m flights.
    PYTHONNOUSERSITE=1 /usr/bin/python3 -I score.py [--ref] [tag ...]
"""
import glob, importlib.util, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "..", "valve_turning_safety_20261022", "tools", "score.py")
spec = importlib.util.spec_from_file_location("score22", SRC); s = importlib.util.module_from_spec(spec)
sys.argv, ref = [a for a in sys.argv if a != "--ref"], "--ref" in sys.argv
spec.loader.exec_module(s)
if not ref:
    s.RUNS = os.path.join(HERE, "..", "runs") + "/"; s.cs.R = s.RUNS
tags = sys.argv[1:] or sorted(os.path.basename(p)[:-4] for p in glob.glob(s.RUNS + "*.npz"))
for t in tags:
    try:
        r = s.cs.row(t)
        d = s.np.load(s.RUNS + t + ".npz", allow_pickle=True)
        why = str(d["reason"]) if bool(d["aborted"]) else ""
        for pat, lab in (("mocap CONFIRMED", "mocap guard"), ("mocap LOST", "mocap guard"),
                         ("box's mocap", "box feed  "), ("q3 ", "q3 guard  "), ("box tilt", "box tilt  ")):
            if pat in why and "watchdog   " in r:
                r = r.replace("watchdog   ", f"{lab:11s}", 1); break
        print(r)
        ta = s.truth_after(t)
        if ta: print(f"{'':16s}{ta}")
        for x in s.extra(t): print(f"{'':16s}{x}")
    except Exception as exc:
        print(f"{t:14s} ERROR {exc!r}")
