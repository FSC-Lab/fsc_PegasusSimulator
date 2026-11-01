#!/usr/bin/env python3
"""Dense trace of the base, the arm and the law around a window: trace2.py <tag> <t0> <t1> [dt].
Columns: mocap base x y z [m], yaw/roll/pitch [deg]; q1..q4 [deg], gripper [mm]; law u3 j1..j4 [N.m] (wb [13..16]),
u1 [N] [17], body torque z [23]; e_y xyz [mm] [24..26], e_y psi [deg] [27]; CoM err x y z [mm] ([48..50]-[45..47]);
free-axis weight [116] / offset [mm] [117]; heading offset [deg] [119]; grip: closed [120], blend [121], hdg state [122],
K_psi [123], D_psi [124]; arm controller err-damping duty j1/j4 law[28]/[31]; the law torque the arm applied j1/j4 law[9]/[12]."""
import sys, numpy as np
from vt_common import Bag, yaw_deg, tilt_deg
tag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]); dt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1
b = Bag(tag)
names = [x.strip(" '[]") for x in str(b.f("js","name")[0]).split(",")]
tj = b.t("js"); P = b.f("js","position")
ii = [names.index(f"joint{k}") for k in (1,2,3,4)]; ig = names.index("gripper_left_joint")
tm = b.t("mocap"); g = lambda k: b.f("mocap", k)
qw, qx, qy, qz = (g(f"pose.orientation.{c}") for c in "wxyz")
yaw = yaw_deg(qw,qx,qy,qz); tilt = tilt_deg(qw,qx,qy,qz)
roll = np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))); pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy-qz*qx), -1, 1)))
px, py, pz = g("pose.position.x"), g("pose.position.y"), g("pose.position.z")
tw = b.t("wb"); W = b.f("wb","data"); ok = np.array([len(r) >= 126 for r in W]); Wm = np.array([r[:126] for r in W[ok]]); tw = tw[ok]
tl = b.t("law"); LD = np.array([r if len(r) >= 32 else np.full(32, np.nan) for r in b.f("law","data")])
def at(t, ts, arr):
    i = min(np.searchsorted(ts, t), len(ts)-1); return arr[i]
print("  t   |  x      y      z   yaw   roll  pitch | q1    q2    q3    q4   grip | u3_1   u3_2   u3_3   u3_4   u1   tz  | ey_x ey_y ey_z ey_psi | ex   ey   ez  | wf off  hdg | cl bl hs Kpsi | ed1  ed4 | app1  app4")
for t in np.arange(t0, t1, dt):
    k = min(np.searchsorted(tm, t), len(tm)-1); j = min(np.searchsorted(tj, t), len(tj)-1)
    w = at(t, tw, Wm); ld = at(t, tl, LD)
    q = np.degrees(P[j, ii]); gr = 1e3*P[j, ig]
    print(f"{t:6.2f} | {px[k]:+6.3f} {py[k]:+6.3f} {pz[k]:5.3f} {yaw[k]:+6.1f} {roll[k]:+5.1f} {pitch[k]:+5.1f} | "
          f"{q[0]:+5.1f} {q[1]:+5.1f} {q[2]:+5.1f} {q[3]:+5.1f} {gr:+5.1f} | "
          f"{w[13]:+6.2f} {w[14]:+6.2f} {w[15]:+6.2f} {w[16]:+6.2f} {w[17]:5.1f} {w[23]:+5.2f} | "
          f"{1e3*w[24]:+5.0f} {1e3*w[25]:+5.0f} {1e3*w[26]:+5.0f} {np.degrees(np.arcsin(np.clip(w[27],-1,1))):+6.1f} | "
          f"{1e3*(w[48]-w[45]):+5.0f} {1e3*(w[49]-w[46]):+5.0f} {1e3*(w[50]-w[47]):+5.0f} | "
          f"{w[116]:4.2f} {1e3*w[117]:+5.0f} {np.degrees(w[119]):+5.1f} | {w[120]:2.0f} {w[121]:4.2f} {w[122]:2.0f} {w[123]:4.2f} | "
          f"{ld[28]:+5.1f} {ld[31]:+5.1f} | {ld[9]:+5.2f} {ld[12]:+5.2f}")
