#!/usr/bin/env python3
"""Isaac: what does the claw hit in the exit?  The PhysX truth contact force between the rotor and the
vehicle (/push_pull_truth/contact [0:3] normal + [3:6] friction), and the claw's truth point in the
ROTOR frame (obj_0 = the marker centroid with the rotor's axes; rotor origin = obj_0 - R pivot), against
the fin (x 0.16, z 0.074..0.204) and the five 30 mm marker balls (fsc:mocap_markers_m).
    exit_contact.py <runs dir> tag [tag ...]"""
import sys, os
import numpy as np
PIV = np.array([0.07202723, -0.00931415, 0.08509223])
MK = np.array([(0.00494, 0.01504, 0.09249), (0.04006, -0.01983, 0.12549), (0.08515, -0.04267, 0.06849),
               (0.10483, 0.04341, 0.07049), (0.12515, -0.04252, 0.06849)])
def qR(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
d = sys.argv[1]
for tag in sys.argv[2:]:
    z = np.load(os.path.join(d, tag + ".npz"), allow_pickle=True); M = dict(zip([str(n) for n in z["marks_name"]], z["marks_t"]))
    C = z["contact"]; F = np.linalg.norm(C[:, 1:4] + C[:, 4:7], axis=1)
    t_rel = M["gripper_release:end"]; t_ex = M.get("exit:start", t_rel)
    B = z["box"]; K = z["claw"]
    print(f"===== {tag}: release done {t_rel:.2f}, exit start {t_ex:.2f}")
    m = C[:, 0] > t_rel - 0.5
    for i in np.where(m)[0][::3]:
        t = C[i, 0]
        if t > t_ex + 4: break
        j = min(np.searchsorted(B[:, 0], t), len(B)-1); k = min(np.searchsorted(K[:, 0], t), len(K)-1)
        R = qR(*B[j, 4:8]); o = B[j, 1:4] - R @ PIV
        c = R.T @ (K[k, 1:4] - o)
        dm = np.linalg.norm(MK - c, axis=1) - 0.015
        print(f"  t {t:6.2f} ({t-t_ex:+5.2f} s into exit)  |F| {F[i]:5.2f} N  claw in rotor frame [{c[0]*1e3:6.1f} {c[1]*1e3:6.1f} {c[2]*1e3:6.1f}] mm"
              f"  nearest marker surface {dm.min()*1e3:5.1f} mm (#{dm.argmin()})  fin x 160, top z 204")
