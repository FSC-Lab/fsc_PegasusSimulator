#!/usr/bin/env python3
"""Does the claw hold its heading on the fin? The claw's heading (the planner's current_ee: FK of the
measured base + joints, x = claw) minus the beam's yaw (obj_0), from the close to the release; a rigid
grasp keeps it constant. Also the law's own heading error e_y[3] vs the beam's lag behind the plan.
    grip_slip.py tag [tag ...]"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import yaw_score as YS
for tag in sys.argv[1:]:
    z = np.load(os.path.join(YS.RUNS, tag + ".npz"), allow_pickle=True)
    mk = YS.marks_of(z)
    ee, box, dbg, ref = z["ee"], z["box"], z["dbg"].astype(float), z["ref"]
    yee = np.unwrap(YS.yaw_q(ee[:, 4:8])); yb = np.unwrap(YS.yaw_q(box[:, 4:8]))
    t0, t1 = mk["gripper_close:end"], mk.get("release:start", mk.get("push:end", ee[-1, 0]))
    w = (ee[:, 0] >= t0) & (ee[:, 0] <= t1)
    rel = np.degrees(yee[w] - np.interp(ee[w, 0], box[:, 0], yb))
    rel -= rel[0]
    a = mk["push:start"] + 1.5
    b = a + 12.0
    def at(t): return float(np.interp(t, ee[w, 0], rel))
    eh = np.degrees(np.arcsin(np.clip(np.interp(ee[w, 0], dbg[:, 0], dbg[:, 26 + 3]), -1, 1)))
    print(f"{tag:22s} claw-minus-beam heading [deg] (0 at the close): at turn start {at(a):+5.1f}, "
          f"mid {at(a + 6):+5.1f}, end {at(b):+5.1f}, range {rel.min():+5.1f}..{rel.max():+5.1f} | "
          f"law e_psi mid {float(np.interp(a + 6, ee[w, 0], eh)):+5.2f} deg")
print("\n-- truth check: (true base yaw + q1) - beam yaw, 0 at the close")
for tag in sys.argv[1:]:
    z = np.load(os.path.join(YS.RUNS, tag + ".npz"), allow_pickle=True)
    mk = YS.marks_of(z)
    bt, box, js = z["base_truth"], z["box"], z["joints"]
    names = [str(n) for n in z["joint_names"]]
    j1 = names.index("joint1") if "joint1" in names else 0
    t0, t1 = mk["gripper_close:end"], mk.get("release:start", mk.get("push:end", bt[-1, 0]))
    t = np.arange(t0, t1, 0.02)
    psi = np.interp(t, bt[:, 0], np.unwrap(YS.yaw_q(bt[:, 4:8])))
    q1 = np.interp(t, js[:, 0], js[:, 1 + j1])
    yb = np.interp(t, box[:, 0], np.unwrap(YS.yaw_q(box[:, 4:8])))
    rel = np.degrees(psi + q1 - yb); rel -= rel[0]
    a = mk["push:start"] + 1.5
    def at(x): return float(np.interp(x, t, rel))
    print(f"{tag:22s} start {at(a):+5.1f}  mid {at(a + 6):+5.1f}  end {at(a + 12):+5.1f}  range {rel.min():+5.1f}..{rel.max():+5.1f}  "
          f"std {rel[(t >= a) & (t <= a + 12)].std():4.1f} deg")
