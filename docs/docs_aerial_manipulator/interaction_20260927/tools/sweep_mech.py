import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import interaction_bench as IB
jobs = []
pols = ["free", "contact", "task", "thr1.0", "thr0.5", "thr1.0_fast", "thr2.0_fast", "thr1.0_mid"]
for chi in pols:
    jobs.append(("force", chi, dict(F=3.0, dir="radial", hold=15.0)))
for chi in ["free", "contact", "gripper", "task", "thr0.5", "thr1.0_fast", "thr0.5_mid", "thr1.0_fast+gripper"]:
    jobs.append(("payload", chi, dict(m=0.1)))
for chi in ["free", "task", "thr1.0_fast"]:
    jobs.append(("box", chi, dict(F=2.0, dir="radial")))
IB.run_jobs(jobs, os.path.join(IB.HERE, "..", "analysis", "sweep_mech.json"))
