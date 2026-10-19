"""The basket on the pick platform before the grasp (PP-4): did it move as the vehicle arrived?  obj_0 processed + raw VRPN."""
from pp10_common import *
from scipy.spatial.transform import Rotation as Rot
d, t0 = load("p4")
tb, Pb, Qb, _ = mocap_pose(d, t0, "obj")
yaw = np.degrees(yaw_of(Qb)); tilt = tilt_of(Qb)
tm, Pm, Qm, _ = mocap_pose(d, t0, "mocap")
print("processed /obj_0/mocap: rate", len(tb) / (tb[-1] - tb[0]), "Hz, max gap", np.diff(tb).max())
tv = d["obj_vrpn__recv"] - t0
Pv = np.column_stack([d[f"obj_vrpn__pose.position.{k}"] for k in "xyz"])
print("raw vrpn obj_0: n", len(tv), "max gap", np.diff(tv).max(), "identical consecutive", int((np.abs(np.diff(Pv, axis=0)).sum(1) == 0).sum()))
print(" t     basket x      y      z    yaw   tilt | vehicle x     y     z   | horiz dist veh-basket")
for ts in list(np.arange(1.0, 20.0, 2.0)) + list(np.arange(20.0, 26.0, 0.2)):
    i = np.searchsorted(tb, ts); j = np.searchsorted(tm, ts)
    print(f"{tb[i]:6.2f} {Pb[i,0]:8.4f} {Pb[i,1]:8.4f} {Pb[i,2]:7.4f} {yaw[i]:7.2f} {tilt[i]:5.2f} | {Pm[j,0]:6.3f} {Pm[j,1]:6.3f} {Pm[j,2]:5.3f} | {np.linalg.norm(Pm[j,:2]-Pb[i,:2]):.3f}")
for a, b in ((2, 16), (16, 21.5), (23.5, 29.5)):
    m = (tb > a) & (tb < b)
    print(f"mean {a}-{b}s: p {Pb[m].mean(0).round(4)} std mm {(1e3*Pb[m].std(0)).round(2)} yaw {yaw[m].mean():.2f} +- {yaw[m].std():.2f} tilt {tilt[m].mean():.2f}; |yaw|>90: {(np.abs(yaw[m])>90).mean()*100:.1f} %")
# flips through the flight
m = (tb > 2) & (tb < 73)
tmv = np.interp(tb, tm, np.degrees(np.unwrap(yaw_of(Qm))))
rel = (yaw - tmv + 180) % 360 - 180
print("basket yaw minus vehicle yaw, by phase:")
for a, b, lab in pp_phases(d, t0):
    mm = (tb >= a) & (tb < b)
    if mm.sum() > 20:
        print(f"  {a:6.1f}-{b:6.1f} {lab[:40]:40s} rel yaw median {np.median(rel[mm]):7.1f}  min {rel[mm].min():7.1f} max {rel[mm].max():7.1f} | jumps>60deg {(np.abs(np.diff(yaw[mm]) + 180) % 360 - 180 > 60).sum() if False else int((np.abs((np.diff(yaw[mm]) + 180) % 360 - 180) > 60).sum())} | tilt med {np.median(tilt[mm]):.1f}")
