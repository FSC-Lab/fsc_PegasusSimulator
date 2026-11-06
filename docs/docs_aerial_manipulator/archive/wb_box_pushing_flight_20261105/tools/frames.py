"""Mocap rigid-body definitions: drone mocap attitude vs PX4 attitude (roll/pitch offset, body frame), the drone's
pivot height on the ground (bag start / end), the box body's height and the planner's table top."""
import sys
import numpy as np


def R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


NED2ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
FRD2FLU = np.diag([1, -1, -1])
for f in sys.argv[1:]:
    z = np.load(f)
    vb, att = z['vrpn_uav'], z['att']
    # PX4 q is NED/FRD -> ENU/FLU
    ia = np.clip(np.searchsorted(att[:, 0], vb[:, 0]), 0, len(att) - 1)
    ok = np.abs(att[ia, 0] - vb[:, 0]) < 0.01
    dr = []
    for k in np.where(ok)[0][::10]:
        Rm = R(vb[k, 4:8])
        Rp = NED2ENU @ R(att[ia[k], 1:5]) @ FRD2FLU
        D = Rp.T @ Rm          # mocap body frame in the PX4 body frame
        dr.append([np.degrees(np.arctan2(D[2, 1], D[2, 2])), np.degrees(-np.arcsin(np.clip(D[2, 0], -1, 1))),
                   np.degrees(np.arctan2(D[1, 0], D[0, 0]))])
    dr = np.array(dr)
    zu = vb[:, 3]
    print(f'== {f.split("/")[-1]}: mocap body vs PX4 body  roll {np.median(dr[:,0]):+5.2f}  pitch {np.median(dr[:,1]):+5.2f}'
          f'  yaw {np.median(dr[:,2]):+6.2f} deg (median)   | drone pivot z: first 1 s {np.median(zu[vb[:,0]<vb[0,0]+1]):.3f}'
          f'  min {zu.min():.3f}  last 1 s {np.median(zu[vb[:,0]>vb[-1,0]-1]):.3f} m'
          f'')
