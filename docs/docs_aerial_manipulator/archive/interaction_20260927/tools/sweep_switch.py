"""Sweep A: the chi-switching logic. 100 g pick-and-place (8 s pressed placement)
and a 3 N radial / 1 N down step force, every trigger policy x reading bandwidth."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import interaction_bench as IB
jobs = []
for rd in (None, 2.0, 5.0):
    for chi in ["free", "contact", "gripper", "task", "free+thr0.5", "free+thr1.0", "gripper+thr1.0"]:
        if rd is None and chi in ("free",) and False:
            continue
        jobs.append(("payload", chi, dict(m=0.1, rd=rd)))
    for chi in ["free", "contact", "task", "free+thr0.5", "free+thr1.0", "task+thr1.0"]:
        jobs.append(("force", chi, dict(F=3.0, dir="radial", hold=15.0, rd=rd)))
        jobs.append(("force", chi, dict(F=1.5, dir="down", hold=15.0, rd=rd)))
IB.run_jobs(jobs, os.path.join(IB.HERE, "..", "analysis", "sweep_switch.json"))
