#!/usr/bin/env python3
"""Isaac: the release + exit per run -- the peak vehicle tilt and the peak PhysX rotor-vehicle contact
force from the jaws opening to 4 s into the exit, the claw's slide along the fin during the release,
and how the run ended.   exit_summary.py <runs dir> tag [tag ...]"""
import sys, os
import numpy as np
d = sys.argv[1]
print(f"{'tag':10s} {'peak tilt release/exit':>24s} {'peak contact F exit':>20s}  ended")
for tag in sys.argv[2:]:
    z = np.load(os.path.join(d, tag + ".npz"), allow_pickle=True); M = dict(zip([str(n) for n in z["marks_name"]], z["marks_t"]))
    O = z["odom"]; x, y = O[:, 8], O[:, 9]; tilt = np.degrees(np.arccos(np.clip(1 - 2*(x*x + y*y), -1, 1)))
    tr = M.get("gripper_release:start"); te = M.get("exit:start", tr)
    if tr is None: print(f"{tag:10s} no release"); continue
    a = lambda t0, t1: (lambda v: v.max() if v.size else float("nan"))(tilt[(O[:, 0] >= t0) & (O[:, 0] <= t1)])
    C = z["contact"]; F = np.linalg.norm(C[:, 1:4] + C[:, 4:7], axis=1); mc = (C[:, 0] >= te) & (C[:, 0] <= te + 4)
    end = "completed" if not bool(z["aborted"]) else str(z["reason"])[:70]
    print(f"{tag:10s} {a(tr, te):8.1f} / {a(te, te + 4):5.1f} deg {F[mc].max() if mc.any() else np.nan:14.1f} N    {end}")
