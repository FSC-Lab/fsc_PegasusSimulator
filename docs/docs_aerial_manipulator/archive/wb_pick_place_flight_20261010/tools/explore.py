"""Raw look at one window of PP-4: vehicle, basket, claw, gripper, joints.  usage: explore.py t_a t_b [step]"""
import sys
from pp10_common import *
import grasp_geom as GG
S = GG.series("p4"); d, t0, t = S["d"], S["t0"], S["t"]
ta, tb_ = float(sys.argv[1]), float(sys.argv[2]); step = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
for a, b, lab in pp_phases(d, t0):
    print(f"   {a:7.2f} {b:7.2f}  {lab}")
tv = d["vstatus__recv"] - t0
print("nav_state edges:", [(round(float(x), 2), int(v)) for x, v in zip(tv[np.r_[True, np.diff(d['vstatus__nav_state']) != 0]], d['vstatus__nav_state'][np.r_[True, np.diff(d['vstatus__nav_state']) != 0]])])
print("arming edges:", [(round(float(x), 2), int(v)) for x, v in zip(tv[np.r_[True, np.diff(d['vstatus__arming_state']) != 0]], d['vstatus__arming_state'][np.r_[True, np.diff(d['vstatus__arming_state']) != 0]])])
print("t | veh mocap xyz yaw tilt | veh odom xyz | basket xyz yaw tilt | claw(mocap) xyz | claw-in-basket mocapFK / odomFK [mm] | grip | q deg")
for ts in np.arange(ta, tb_, step):
    i = np.searchsorted(t, ts)
    if i >= len(t): break
    print(f"{t[i]:6.2f} | {S['Pm'][i,0]:6.3f} {S['Pm'][i,1]:6.3f} {S['Pm'][i,2]:5.3f} {np.degrees(yaw_of(S['Qv'][i:i+1]))[0]:7.1f} {tilt_of(S['Qv'][i:i+1])[0]:4.1f} | {S['Po'][i,0]:6.3f} {S['Po'][i,1]:6.3f} {S['Po'][i,2]:5.3f} | "
          f"{S['pb'][i,0]:6.3f} {S['pb'][i,1]:6.3f} {S['pb'][i,2]:5.3f} {np.degrees(S['yb'][i]):7.1f} {S['tiltb'][i]:5.1f} | {S['re_m'][i,0]:6.3f} {S['re_m'][i,1]:6.3f} {S['re_m'][i,2]:5.3f} | "
          f"[{1e3*S['rel_m'][i,0]:6.0f} {1e3*S['rel_m'][i,1]:6.0f} {1e3*S['rel_m'][i,2]:6.0f}] [{1e3*S['rel_o'][i,0]:6.0f} {1e3*S['rel_o'][i,1]:6.0f} {1e3*S['rel_o'][i,2]:6.0f}] | {np.round(S['g'][i],3)} | {np.round(np.degrees(S['q'][i]),1)}")
