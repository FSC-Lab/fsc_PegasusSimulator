"""Sweep B-D: capacity at the recommended switching config (reading omega_x 2 rad/s,
per-block feed-forward kept at H1b's 0.207; chi = gripper / task), hardware servo caps."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import interaction_bench as IB
jobs = []
FL = {"radial": [2, 4, 6, 8, 10, 12, 14, 16], "-radial": [2, 4, 6, 8, 10, 12, 14],
      "lateral": [0.5, 1.0, 1.5, 2.0, 3.0], "down": [1, 2, 3, 4, 5, 6, 7], "up": [2, 4, 6, 8, 10, 12]}
for d, L in FL.items():
    for F in L:
        jobs.append(("force", "task", dict(F=float(F), dir=d, hold=15.0, rd=2.0)))
for F in (4, 8, 12, 16):
    jobs.append(("force", "task", dict(F=float(F), dir="radial", hold=15.0, rd=2.0, hw_caps=False)))
for F in (2, 4, 6, 8):
    jobs.append(("force", "task", dict(F=float(F), dir="down", hold=15.0, rd=2.0, hw_caps=False)))
for F in (1.0, 2.0, 3.0, 4.0):
    jobs.append(("force", "task", dict(F=float(F), dir="lateral", hold=15.0, rd=2.0, hw_caps=False)))
for m in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8):
    jobs.append(("payload", "gripper", dict(m=m, rd=2.0)))
    jobs.append(("payload", "free", dict(m=m)))
for m in (0.4, 0.6, 0.8, 1.0):
    jobs.append(("payload", "gripper", dict(m=m, rd=2.0, hw_caps=False)))
for F in (1, 2, 3, 4, 6, 8, 10, 12):
    jobs.append(("box", "task", dict(F=float(F), dir="radial", rd=2.0)))
for F in (0.5, 1.0, 1.5, 2.0):
    jobs.append(("box", "task", dict(F=float(F), dir="lateral", rd=2.0)))
IB.run_jobs(jobs, os.path.join(IB.HERE, "..", "analysis", "sweep_capacity.json"), procs=30)
