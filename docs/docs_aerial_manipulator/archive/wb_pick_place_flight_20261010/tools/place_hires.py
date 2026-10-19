"""High-rate look at the place of PP-4: reference claw, measured claw (mocap FK / planner FK), basket, vehicle attitude.
usage: place_hires.py t_a t_b [step]"""
import sys
from pp10_common import *
import grasp_geom as GG
import tracking as T
from scipy.spatial.transform import Rotation as Rot
nm = "p4"
S = GG.series(nm); d, t0, t = S["d"], S["t0"], S["t"]
tr, ref = wbref(d, t0)
red = interp(t, tr, ref["r_ed"]); xcd = interp(t, tr, ref["x_cd"]); qd = np.degrees(interp(t, tr, ref["q_d"]))
tw, D = wbdebug(d, t0)
dh = interp(t, tw, D[:, WB["d_hat"]]); u1 = interp(t, tw, D[:, WB["u1"]]); mot = interp(t, tw, D[:, WB["motors"]]); tau = interp(t, tw, D[:, WB["tau"]])
eul = Rot.from_quat(S["Qv"][:, [1, 2, 3, 0]]).as_euler("ZYX", degrees=True)   # yaw, pitch, roll (ENU/FLU, pitch + = nose down)
Rb = Rot.from_quat(np.column_stack([interp(t, *(lambda tb, Pb, Qb, Vb: (tb, Qb))(*mocap_pose(d, t0, "obj")))])[:, [1, 2, 3, 0]]).as_matrix()
ta, tb_ = float(sys.argv[1]), float(sys.argv[2]); step = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
print("t | claw ref xyz | claw mocapFK xyz | claw plannerFK xyz | basket xyz tilt | veh x y z pitch roll | xc_ref x | q2 q3 (ref q2 q3) | grip | u1 dz | motors | F_raw")
Fr = interp(t, tw, D[:, WB["F_raw"]])
for ts in np.arange(ta, tb_, step):
    i = np.searchsorted(t, ts)
    if i >= len(t): break
    print(f"{t[i]:6.2f} | {red[i,0]:7.3f} {red[i,1]:7.3f} {red[i,2]:6.3f} | {S['re_m'][i,0]:7.3f} {S['re_m'][i,1]:7.3f} {S['re_m'][i,2]:6.3f} | {S['re_o'][i,0]:7.3f} {S['re_o'][i,1]:7.3f} {S['re_o'][i,2]:6.3f} | "
          f"{S['pb'][i,0]:7.3f} {S['pb'][i,1]:7.3f} {S['pb'][i,2]:6.3f} {S['tiltb'][i]:5.1f} | {S['Pm'][i,0]:7.3f} {S['Pm'][i,1]:7.3f} {S['Pm'][i,2]:6.3f} {eul[i,1]:6.1f} {eul[i,2]:6.1f} | {xcd[i,0]:7.3f} | "
          f"{np.degrees(S['q'][i,1]):5.1f} {np.degrees(S['q'][i,2]):5.1f} ({qd[i,1]:4.1f} {qd[i,2]:4.1f}) | {S['g'][i,0]:6.3f} | {u1[i]:5.1f} {dh[i,2]:5.1f} | {np.round(mot[i],2)} | {np.round(Fr[i,:3],1)}")
