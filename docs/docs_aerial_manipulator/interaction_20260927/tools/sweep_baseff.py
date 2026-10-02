import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import interaction_bench as IB
jobs = []
for bff in (False, True):
    for m in (0.1, 0.3, 0.4, 0.5):
        jobs.append(("payload", "gripper", dict(m=m, rd=2.0, base_ff=bff)))
        jobs.append(("payload", "gripper", dict(m=m, rd=2.0, base_ff=bff, unload=True)))
    for d, F in (("down", 4.0), ("up", 8.0), ("radial", 8.0), ("radial", 10.0), ("-radial", 8.0), ("lateral", 1.0)):
        jobs.append(("force", "task", dict(F=F, dir=d, hold=15.0, rd=2.0, base_ff=bff)))
    for F in (6.0, 8.0, 10.0, 12.0):
        jobs.append(("box", "task", dict(F=F, dir="radial", rd=2.0, base_ff=bff)))
# free-mode windup against a pressed desk: long hold
for chi, rd in (("free", None), ("gripper", 2.0)):
    jobs.append(("payload", chi, dict(m=0.1, rd=rd, place_hold=25.0)))
# threshold-only trigger for the box (no task flag) at the hardware-safe threshold
for F in (3.0, 6.0):
    jobs.append(("box", "free+thr2.0", dict(F=F, dir="radial", rd=2.0)))
    jobs.append(("box", "free", dict(F=F, dir="radial")))
IB.run_jobs(jobs, os.path.join(IB.HERE, "..", "analysis", "sweep_baseff.json"), procs=30)
