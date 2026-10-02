"""Keep full traces of the handful of cases the report plots."""
import sys, os, numpy as np, multiprocessing as mp
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import interaction_bench as IB
CASES = {
 "force_free":      ("force", "free", dict(F=3.0, dir="radial", hold=15.0)),
 "force_thr1_slow": ("force", "free+thr1.0", dict(F=3.0, dir="radial", hold=15.0)),
 "force_thr2_fast": ("force", "free+thr2.0", dict(F=3.0, dir="radial", hold=15.0, rd=2.0)),
 "force_task_fast": ("force", "task", dict(F=3.0, dir="radial", hold=15.0, rd=2.0)),
 "place_free":      ("payload", "free", dict(m=0.1, place_hold=25.0)),
 "place_grip_slow": ("payload", "gripper", dict(m=0.1, place_hold=25.0)),
 "place_grip_fast": ("payload", "gripper", dict(m=0.1, place_hold=25.0, rd=2.0)),
 "pay300_grip":     ("payload", "gripper", dict(m=0.3, rd=2.0)),
 "pay300_grip_rot": ("payload", "gripper", dict(m=0.3, rd=2.0, base_ff="rot")),
 "box6_task":       ("box", "task", dict(F=6.0, dir="radial", rd=2.0)),
}
def job(k):
    scn, chi, kw = CASES[k]
    r = IB.simulate(scn, chi, keep=True, **dict(kw))
    d = {"t": r["H"]["t"]}
    for g in ("H", "H3", "H4"):
        for kk, v in r[g].items():
            d[f"{g}_{kk}"] = v
    if "box" in r: d["box"] = r["box"]
    d["marks"] = np.array([list(r["marks"].keys()), list(r["marks"].values())], dtype=object)
    np.savez_compressed(os.path.join(IB.HERE, "..", "data", f"trace_{k}.npz"), **d)
    return k, r["verdict"]
with mp.get_context("fork").Pool(6) as p:
    for k, v in p.imap_unordered(job, list(CASES)):
        print(k, v, flush=True)
