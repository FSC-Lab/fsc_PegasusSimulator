import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import interaction_bench as IB
jobs = []
for m in (0.1, 0.3, 0.4, 0.5):
    jobs.append(("payload", "gripper", dict(m=m, rd=2.0, base_ff="rot")))
for d, F in (("down", 4.0), ("up", 8.0), ("radial", 8.0), ("radial", 10.0), ("-radial", 8.0)):
    jobs.append(("force", "task", dict(F=F, dir=d, hold=15.0, rd=2.0, base_ff="rot")))
for F in (6.0, 8.0, 10.0):
    jobs.append(("box", "task", dict(F=F, dir="radial", rd=2.0, base_ff="rot")))
IB.run_jobs(jobs, os.path.join(IB.HERE, "..", "analysis", "sweep_baseff_rot.json"), procs=30)
