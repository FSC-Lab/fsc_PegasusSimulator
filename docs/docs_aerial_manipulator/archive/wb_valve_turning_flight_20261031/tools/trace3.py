#!/usr/bin/env python3
"""Base-loop trace: trace3.py <tag> <t0> <t1> [dt]. Mocap roll/pitch/yaw; law e_R [28..30] (x1000); body torque FLU
[21..23] N.m; u1 [17]; d_hat_t [31..33] N; d_hat_r [34..36] N.m; d_hat_q [37..40] N.m; n_sat [51]; unalloc thrust [55];
CoM err world [48..50]-[45..47] mm; motors m1..m4; ee_psi err deg [27]; q1 q4 deg."""
import sys, numpy as np
from vt_common import Bag, yaw_deg
tag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]); dt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1
b = Bag(tag)
names = [x.strip(" '[]") for x in str(b.f("js","name")[0]).split(",")]
tj = b.t("js"); P = b.f("js","position"); i1 = names.index("joint1"); i4 = names.index("joint4")
tm = b.t("mocap"); g = lambda k: b.f("mocap", k)
qw, qx, qy, qz = (g(f"pose.orientation.{c}") for c in "wxyz")
yaw = yaw_deg(qw,qx,qy,qz); roll = np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))); pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy-qz*qx), -1, 1)))
tw = b.t("wb"); W = b.f("wb","data"); ok = np.array([len(r) >= 120 for r in W]); Wm = np.array([r[:120] for r in W[ok]]); tw = tw[ok]
tmo = b.t("motors"); MO = b.f("motors","control") if "motors__control" in b.z.files else None
if MO is None:
    ks = [k for k in b.z.files if k.startswith("motors__") and "recv" not in k]; print("motor fields:", ks); MO = b.z[ks[0]]
def at(t, ts, arr):
    i = min(np.searchsorted(ts, t), len(ts)-1); return arr[i]
print("  t   | roll  pitch   yaw | eR_x  eR_y  eR_z | tau_x  tau_y  tau_z   u1  | dt_x  dt_y  dt_z | dr_x   dr_y   dr_z | dq1   dq2   dq3   dq4  | ns unal | ex    ey    ez  | m1   m2   m3   m4  | epsi  q1    q4")
for t in np.arange(t0, t1, dt):
    k = min(np.searchsorted(tm, t), len(tm)-1); j = min(np.searchsorted(tj, t), len(tj)-1); w = at(t, tw, Wm); mo = at(t, tmo, MO)
    print(f"{t:6.2f} | {roll[k]:+5.1f} {pitch[k]:+5.1f} {yaw[k]:+6.1f} | {1e3*w[28]:+5.0f} {1e3*w[29]:+5.0f} {1e3*w[30]:+5.0f} | "
          f"{w[21]:+6.2f} {w[22]:+6.2f} {w[23]:+6.2f} {w[17]:5.1f} | {w[31]:+5.2f} {w[32]:+5.2f} {w[33]:+5.2f} | {w[34]:+6.3f} {w[35]:+6.3f} {w[36]:+6.3f} | "
          f"{w[37]:+5.2f} {w[38]:+5.2f} {w[39]:+5.2f} {w[40]:+5.2f} | {w[51]:2.0f} {w[55]:+5.2f} | {1e3*(w[48]-w[45]):+5.0f} {1e3*(w[49]-w[46]):+5.0f} {1e3*(w[50]-w[47]):+5.0f} | "
          f"{mo[0]:4.2f} {mo[1]:4.2f} {mo[2]:4.2f} {mo[3]:4.2f} | {np.degrees(np.arcsin(np.clip(w[27],-1,1))):+5.1f} {np.degrees(P[j,i1]):+5.1f} {np.degrees(P[j,i4]):+5.1f}")
