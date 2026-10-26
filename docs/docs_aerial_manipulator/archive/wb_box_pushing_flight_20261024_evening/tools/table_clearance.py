#!/usr/bin/env python3
"""How close did the vehicle come to the table? Per sample: the vehicle (mocap body origin) and the four
rotor hubs (+-0.2299 m in the x-forward body frame) in the TABLE frame the planner placed from the box
(push_pull/info [79..86]: origin, yaw, x/y extent, top z), the claw from FK (joint_states + mocap pose,
the planner's model, R0_model = R0_actual Rz(-90)), and the clearance of each above the table top
when over (or within a prop radius of) it.  Usage: table_clearance.py <tag> <t0> <t1> [dt]"""
import sys, os
import numpy as np
from bp_common import Bag
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), *[".."] * 5))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP   # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C          # noqa: E402

tag, a, c = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]); dt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1
b = Bag(tag)
I = b.f("pl_info", "data")[-1]
org, yaw, xr, yr, top = I[79:81], np.radians(I[81]), I[82:84], I[84:86], I[86]
Rt = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
def to_table(p):
    return Rt.T @ (p[:2] - org)
def dist_rect(pt):            # horizontal distance from a table-frame point to the table rectangle
    dx = max(xr[0] - pt[0], 0, pt[0] - xr[1]); dy = max(yr[0] - pt[1], 0, pt[1] - yr[1]); return np.hypot(dx, dy)
def R_of(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
model = TP.make_params_t650()
Rz90 = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])         # Rz(-90)
tm = b.t("mocap"); P = np.column_stack([b.f("mocap", f"pose.position.{k}") for k in "xyz"])
Q = np.column_stack([b.f("mocap", f"pose.orientation.{k}") for k in "wxyz"])
tj = b.t("js"); J = b.f("js", "position")
HUB = np.array([[0.2299, 0.2299, 0], [-0.2299, -0.2299, 0], [0.2299, -0.2299, 0], [-0.2299, 0.2299, 0]])
PROP_R = 0.19
# the T650 landing gear: two skids along body x, 0.28 m apart inside (~ +-0.15 m), 0.248 m under the body
# origin (the real T650's resting mocap height, 10-20 bag) -- sampled as 7 points each
SKID = np.array([[x, y, -0.248] for y in (-0.15, 0.15) for x in np.linspace(-0.15, 0.15, 7)])
print(f"{tag}: table frame origin {np.round(org,3)}, yaw {np.degrees(yaw):.1f}, x {np.round(xr,3)} y {np.round(yr,3)}, top {top:.3f} m")
print("   t   | skid: lowest point over the table, above top | body z  tilt | body in table frame (x,y), dist | lowest rotor hub: z, above top, horiz dist to table | claw z, above top, horiz dist | q2 q3")
worst = (9, None)
worst_sk = [9, None]
for t in np.arange(a, c, dt):
    i = np.searchsorted(tm, t); j = np.searchsorted(tj, t)
    R = R_of(Q[i]); p = P[i]
    hubs = p + (R @ HUB.T).T
    q = np.array([-J[j, 2], J[j, 0], J[j, 1], -J[j, 3]])
    X = np.zeros(12 + 3 * 4 + 6); Rm = R @ Rz90
    X[3:12] = Rm.flatten(order="F"); X[12:16] = q
    claw = p + Rm @ C.dynamics(X, model)["r_0e_0"]
    tb = to_table(p); db = dist_rect(tb)
    hz = [(h[2] - top, dist_rect(to_table(h))) for h in hubs]
    k = int(np.argmin([z for z, _ in hz]))
    tc = to_table(claw); dc = dist_rect(tc)
    sk = p + (R @ SKID.T).T
    over = [s_[2] - top for s_ in sk if dist_rect(to_table(s_)) < 0.005]
    skid_s = f"{min(over):+6.3f}" if over else "  --  "
    if over and min(over) < worst_sk[0]:
        worst_sk[0], worst_sk[1] = min(over), t
    tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
    # clearance that matters: a prop disc within PROP_R of the table, height above the top
    near = [z for z, d in hz if d < PROP_R]
    if near and min(near) < worst[0]:
        worst = (min(near), t)
    print(f"{t:6.2f} | {skid_s} | {p[2]:5.3f} {tilt:5.1f} | ({tb[0]:+5.2f},{tb[1]:+5.2f}) {db:4.2f} | {hubs[k,2]:5.3f} {hz[k][0]:+6.3f} {hz[k][1]:4.2f} | "
          f"{claw[2]:5.3f} {claw[2]-top:+6.3f} {dc:4.2f} | {np.degrees(q[1]):5.1f} {np.degrees(q[2]):5.1f}")
print(f"  lowest skid point OVER the table: {worst_sk[0]:+.3f} m above the top at t = {worst_sk[1]}")
print(f"  lowest rotor hub within a prop radius ({PROP_R} m) of the table: {worst[0]:+.3f} m above the top at t = {worst[1]}")
