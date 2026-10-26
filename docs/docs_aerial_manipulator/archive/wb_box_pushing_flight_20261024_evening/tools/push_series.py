#!/usr/bin/env python3
"""Resample the push window of one bag onto a common 50 Hz grid: box travel / speed along the
push, the plan's, the vehicle (mocap) along / across the push, the law's CoM and EE errors,
joints (MODEL, deg), joint torques, the force reading, the planner's box guards.
Usage: push_series.py <tag> [t0 t1]  -> prints a 0.25 s table; saves ../npz/push_<tag>.npz"""
import sys, os
import numpy as np
from bp_common import Bag, tilt_deg, yaw_deg

tag = sys.argv[1]
b = Bag(tag)
t0, t1 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (None, None)
A = b.f("pl_arr", "data"); ta = b.t("pl_arr")
I = b.f("pl_info", "data")
push_yaw = np.radians(I[-1, 81])            # the table frame yaw = the push heading (info [81])
u = np.array([np.cos(push_yaw), np.sin(push_yaw)])     # along the push (world)
n = np.array([-u[1], u[0]])                            # across (left of the push)
if t0 is None:
    ps = ta[A[:, 0] == 2]
    t0, t1 = ps[0] - 2.0, ps[-1] + 3.0
T = np.arange(t0, t1, 0.02)

def interp(t, y):
    return np.interp(T, t, y)

to = b.t("obj")
oxy = np.column_stack([b.f("obj", f"pose.position.{c}") for c in "xy"])
oz = b.f("obj", "pose.position.z")
oq = [b.f("obj", f"pose.orientation.{c}") for c in "wxyz"]
otilt, oyaw = tilt_deg(*oq), yaw_deg(*oq)
tm = b.t("mocap")
mxy = np.column_stack([b.f("mocap", f"pose.position.{c}") for c in "xy"])
mz = b.f("mocap", "pose.position.z")
mq = [b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"]
mtilt, myaw = tilt_deg(*mq), yaw_deg(*mq)
# pitch about the push-perpendicular axis: nose-down positive along the push
R = None
tw, W = b.t("wb"), b.f("wb", "data")
D = W.shape[1]
tj = b.t("js"); P = np.degrees(b.f("js", "position"))
q = {1: -P[:, 2], 2: P[:, 0], 3: P[:, 1], 4: -P[:, 3]}
grip = P[:, 4]
ref0 = None
o_al = oxy @ u; o_ac = oxy @ n
m_al = mxy @ u; m_ac = mxy @ n
S = dict(T=T,
         box_al=interp(to, o_al), box_ac=interp(to, o_ac), box_z=interp(to, oz),
         box_tilt=interp(to, otilt), box_yaw=interp(to, oyaw),
         veh_al=interp(tm, m_al), veh_ac=interp(tm, m_ac), veh_z=interp(tm, mz),
         veh_tilt=interp(tm, mtilt), veh_yaw=interp(tm, myaw),
         q1=interp(tj, q[1]), q2=interp(tj, q[2]), q3=interp(tj, q[3]), q4=interp(tj, q[4]), grip=interp(tj, grip),
         step=np.interp(T, ta, A[:, 0], left=-1), lag=interp(ta, A[:, 11]), gtilt=interp(ta, A[:, 12]),
         fguard=interp(ta, A[:, 10]), mon_travel=interp(ta, A[:, 26]), mon_plan=interp(ta, A[:, 27]),
         mon_speed=interp(ta, A[:, 28]), mon_pspeed=interp(ta, A[:, 29]), mon_t=interp(ta, A[:, 23]))
mode = W[:, 0]
ok = mode > 0.5
t_last = tw[ok][-1] if ok.any() else -1.0
def wcol(i):         # the law's signals exist only in DIRECT: NaN outside it (no flat hold-over)
    if D <= i:
        return np.full_like(T, np.nan)
    y = np.interp(T, tw[ok], W[ok, i])
    return np.where((T >= tw[ok][0]) & (T <= t_last), y, np.nan)
xcd = np.column_stack([wcol(45 + k) for k in range(3)])
xc = np.column_stack([wcol(48 + k) for k in range(3)])
S["ecom_al"] = (xc[:, :2] - xcd[:, :2]) @ u
S["ecom_ac"] = (xc[:, :2] - xcd[:, :2]) @ n
S["ecom_z"] = xc[:, 2] - xcd[:, 2]
S["xcd_al"] = xcd[:, :2] @ u
for k in range(4):
    S[f"ey{k}"] = wcol(24 + k)
    S[f"tau{k+1}"] = wcol(13 + k)
    S[f"F{k}"] = wcol(97 + k)
for k in range(3):
    S[f"eR{k}"] = wcol(28 + k)
S["u1"] = wcol(17)
S["free_w"] = wcol(116); S["free_off"] = wcol(117)
for k in range(10):
    S[f"dhat{k}"] = wcol(31 + k)
for k in range(4):
    S[f"mot{k}"] = wcol(41 + k)
np.savez(os.path.join(os.path.dirname(__file__), "..", "npz", f"push_{tag}.npz"), **S)
if __name__ == "__main__":
    print(f"{tag}: push heading {np.degrees(push_yaw):.1f} deg, window {t0:.2f}..{t1:.2f}")
    b0 = S["box_al"][0]; v0 = S["veh_al"][0]
    hdr = ("t step | box_al  v_box gtilt lag | veh_al ecom_al ecom_ac | q2 q3 q1 q4 | tau2 tau3 | F_al(ey0?) Fg | eRx eRy")
    print(hdr)
    vb = np.gradient(S["box_al"], T)
    for i in range(0, len(T), 12):
        print(f"{T[i]:6.2f} {S['step'][i]:2.0f} | {1e3*(S['box_al'][i]-b0):6.1f} {1e3*vb[i]:6.1f} {S['gtilt'][i]:4.1f} {1e3*S['lag'][i]:5.1f} | "
              f"{1e3*(S['veh_al'][i]-v0):6.0f} {1e3*S['ecom_al'][i]:6.0f} {1e3*S['ecom_ac'][i]:6.0f} | "
              f"{S['q2'][i]:5.1f} {S['q3'][i]:5.1f} {S['q1'][i]:5.1f} {S['q4'][i]:5.1f} | {S['tau2'][i]:5.2f} {S['tau3'][i]:5.2f} | "
              f"{S['F0'][i]:5.2f} {S['F1'][i]:5.2f} {S['F2'][i]:5.2f} {S['fguard'][i]:4.2f} | {1e3*S['eR0'][i]:5.0f} {1e3*S['eR1'][i]:5.0f}")
