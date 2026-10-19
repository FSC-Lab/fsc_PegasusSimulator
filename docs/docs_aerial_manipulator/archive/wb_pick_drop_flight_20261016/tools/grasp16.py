"""Where the jaws closed on the basket: claw (FK) minus the basket's rest pose on the pick pillar, in the vehicle
frame (yaw only), at the GS's CLOSE and at the grasp report, plus the gripper position. 10-16 runs and, for
reference, the 10-15 loaded runs.  usage: grasp16.py"""
import os
import numpy as np
import pd15_common as P
from pd15_common import rosout, gripper, tilt_of
from drop16 import claw, Rz


def one(npz, tag):
    d = np.load(npz)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
    to, Po, Qo = P.obj_pose(d, t0)
    te, Pe, Qe = claw(d, t0)
    tb, _, _, yb = P.px4_rpy(d, t0)
    tc = [t for t, n, m in rosout(d, t0, contains="gripper CLOSE")][0]
    tx = [t for t, n, m in rosout(d, t0, contains="Exit To Pick at once")][0]
    tg, gp = gripper(d, t0)
    rest = (to > tc - 8) & (to < tc - 5)               # basket on the pillar before the descent
    pb = Po[rest].mean(0)
    for lab, t in (("CLOSE", tc), ("report", tx)):
        j = np.searchsorted(te, t)
        ps = np.interp(t, tb, np.unwrap(yb))
        rel = Rz(-ps) @ (Pe[j] - pb) * 1e3
        print(f"{tag:10s} {lab:6s} t {t:6.2f}  claw - basket rest (vehicle x fwd / y left / z, mm) {np.round(rel, 0)}"
              f"   gripper {gp[min(np.searchsorted(tg, t), len(tg) - 1), 0]:+.4f}")
    print(f"{'':10s} model claw - basket = Rz^T(-ee_offset)... claw = point + R(-0.02, 0, 0.16) -> vehicle (-20, 0, 160)"
          f"   basket tilt at rest {tilt_of(Qo[rest]).mean():.1f}")


if __name__ == "__main__":
    for r in ("d1", "d2"):
        one(os.path.join(P.NPZ, f"{r}.npz"), f"1016 {r}")
    old = os.path.join(P.NPZ, "..", "..", "wb_pick_drop_flight_20261015", "npz")
    for r in ("d1", "d2"):
        one(os.path.join(old, f"{r}.npz"), f"1015 {r}")
