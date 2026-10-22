#!/usr/bin/env python3
"""How an aborted box-push flight ended: the trip time into the push, the arm's extremes from the
claw's arrival to 12 s after the trip, the vehicle's tilt after the trip, the box's tilt and travel,
and whether the node stayed in DIRECT through the abort.   /usr/bin/python3 abort_end.py runs..."""
import sys
import numpy as np


def zax(q):
    w, x, y, z = q.T
    return np.column_stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


for p in sys.argv[1:]:
    z = np.load(p, allow_pickle=True)
    m = {str(k): float(v) for k, v in zip(z["marks_name"], z["marks_t"])}
    ev = [str(e) for e in z["events"]]
    ta = next((float(e.split("]")[0].strip("[ s")) for e in ev if "push_pull: ABORTING" in e), None)
    left = any("NOT IN DIRECT" in e for e in ev if ta is not None and float(e.split("]")[0].strip("[ s")) > ta
               and float(e.split("]")[0].strip("[ s")) < ta + 15)
    J, BT, B = z["joints"], z["base_truth"], z["box"]
    a = m.get("ready:end", J[0, 0])
    b = (ta or J[-1, 0]) + 12.0
    j = J[(J[:, 0] >= a) & (J[:, 0] <= b)]
    bt = BT[(BT[:, 0] >= (ta or a)) & (BT[:, 0] <= b)]
    tilt = np.degrees(np.arccos(np.clip(zax(bt[:, 4:8])[:, 2], -1, 1)))
    bz = zax(B[:, 4:8])
    btilt = np.degrees(np.arccos(np.clip(bz @ bz[0], -1, 1))).max()
    b0 = B[np.searchsorted(B[:, 0], m.get("push:start", B[0, 0]))]
    reason = next((e.split("SAFETY GUARD: ")[1][:60] for e in ev if "push_pull: ABORTING" in e and "SAFETY GUARD" in e), str(z["reason"])[:60])
    trip = "" if ta is None else "%5.1f s into the push" % (ta - m.get("push:start", ta))
    print(f"{p.split('/')[-1][:-4]:12s} trip {trip:26s} "
          f"| q3 min {np.degrees(j[:, 3]).min():5.1f} q2 max {np.degrees(j[:, 2]).max():5.1f} | tilt after the trip max {tilt.max():4.1f} "
          f"| node {'LEFT DIRECT' if left else 'stayed in DIRECT'} | box tilt {btilt:4.1f} travel {-(B[-1, 2] - b0[2]) * 1e3:6.1f} mm | {reason}")
