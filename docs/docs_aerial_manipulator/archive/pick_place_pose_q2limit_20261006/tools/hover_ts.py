import os, sys, numpy as np
T = "/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/pick_place_controllers_20261003/tools"
sys.path.insert(0, T)
import pose_hover_screen as PS
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pose_sweep import TP, P
HS = PS.HS
pose = tuple(float(x) for x in sys.argv[1].split(","))
out = {}
for s in range(3):
    r = HS.run(PS.GAINS, "world", T=60.0, seed=s, pose=pose, delay_ms=16.0, keep=True)
    q = np.asarray(r["Hq"]); t = np.asarray(r["H"]["t"])
    n = min(len(q), len(t)); q, t = q[:n], t[:n]
    sk = np.array([TP._sigma_nd(x, P) for x in q[::5]])
    out[f"t{s}"] = t; out[f"q{s}"] = q; out[f"sk{s}"] = sk; out[f"tsk{s}"] = t[::5]
    d = np.degrees(q)
    line = f"{str(pose):20s} seed {s}:"
    for a, b in ((0, 5), (5, 15), (15, 30), (30, 60)):
        m = (t >= a) & (t < b); ms = (t[::5] >= a) & (t[::5] < b)
        line += f" | {a:2d}-{b:2d}s q2 {d[m,1].min():6.1f} q3 {d[m,2].min():6.1f} sig {sk[ms].min():.3f}"
    print(line, flush=True)
np.savez(f"ts_{sys.argv[1].replace(',', '_')}.npz", **out)
