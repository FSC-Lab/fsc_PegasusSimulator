#!/usr/bin/env python3
"""Data for the 3-D view of the 11-01 crash flight's abort (175932) on the logic comparison page:
-> ../logic_compare/abort3d.json. Frame (same as the page's replay): origin = the planned grasp point,
f = the vehicle's nose (world -y), l = its left (world +x), z up; metres.
- plan: the abort plan the planner published on whole_body_planner/viz_path at 43.91 s (120 samples,
  rows [x_cd, nose, r_ed, claw axis]), i.e. the reference it streamed: settle 2.0 s, climb 3.95 s, arm home 3.82 s.
- flown, 43.40-47.04 s every 0.02 s: the law's CoM (wb_control_debug[48..50]), the claw from the planner's arm FK
  (current_ee), the claw reference the law tracked (current_ee - EE task error [24..26]; equals r_ed + CoM - x_cd
  to a few mm once the claw is CoM-anchored), the CoM-anchor weight [129], the measured arm skeleton, mocap pose,
  jaws, valve rotation, tilt, and the planner's EE frame (x = claw, y = the jaws' closing axis; 15 Hz).
Run with /usr/bin/python3."""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_valve_turning_flight_20261031", "tools"))
import vt_common
vt_common._c.NPZ = os.path.join(HERE, "..", "npz")
b = vt_common.Bag("175932")
G0 = np.array([0.366, -0.141, 0.959])
F, L = np.array([0.0, -1.0, 0.0]), np.array([1.0, 0.0, 0.0])
AXOFF = np.array([-0.0935, -0.0014, -0.0899])
T_PLAN, PHASES = 43.91, [2.0, 3.95, 3.82]


def fr(p):
    d = np.asarray(p, float) - G0
    return [round(float(d @ F), 4), round(float(d @ L), 4), round(float(d[2]), 4)]


def frv(v):
    v = np.asarray(v, float)
    return [round(float(v @ F), 4), round(float(v @ L), 4), round(float(v[2]), 4)]


def Rq(q):
    qw, qx, qy, qz = q
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def at(t, ts, a):
    return a[min(np.searchsorted(ts, t), len(ts) - 1)]


# --- the abort plan
tv = b.t("pl_viz_path"); vd = b.f("pl_viz_path", "data")
k = int(np.argmin(np.abs(tv - T_PLAN)))
V = np.asarray(vd[k], float).reshape(-1, 12)
T = sum(PHASES)
plan = [dict(t=round(T * i / (len(V) - 1), 3), com=fr(r[0:3]), claw=fr(r[6:9]), axis=frv(r[9:12])) for i, r in enumerate(V)]

# --- flown
tm = b.t("mocap"); P = np.c_[[b.f("mocap", f"pose.position.{c}") for c in "xyz"]].T
Q = np.c_[[b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"]].T
tilt = vt_common.tilt_deg(*Q.T)
te = b.t("pl_current_ee"); E = np.c_[[b.f("pl_current_ee", f"pose.position.{c}") for c in "xyz"]].T
QE = np.c_[[b.f("pl_current_ee", f"pose.orientation.{c}") for c in "wxyz"]].T   # EE frame: x = claw, y = the jaws' closing axis
tw = b.t("wb"); W = np.array([np.asarray(r, float)[:131] for r in b.f("wb", "data")])
ts = b.t("skeleton"); SK = np.array([np.asarray(r, float)[:15] for r in b.f("skeleton", "data")])
names = [x.strip(" '[]") for x in str(b.f("js", "name")[0]).split(",")]
tj = b.t("js"); J = np.array([np.asarray(r, float) for r in b.f("js", "position")])
to = b.t("obj"); O = np.c_[[b.f("obj", f"pose.position.{c}") for c in "xyz"]].T
oyaw = np.radians(vt_common.yaw_deg(*(b.f("obj", f"pose.orientation.{c}") for c in "wxyz")))
pre = (to > 40.0) & (to < 43.0)
yaw0 = float(np.median(np.unwrap(oyaw[pre])))
ax = []
for i in np.where(pre)[0]:
    c, s = np.cos(oyaw[i]), np.sin(oyaw[i])
    ax.append(O[i] + np.array([c * AXOFF[0] - s * AXOFF[1], s * AXOFF[0] + c * AXOFF[1], AXOFF[2]]))
ax0 = np.median(np.array(ax), axis=0)

flown = []
for t in np.arange(43.40, 47.04 + 1e-9, 0.02):
    w = at(t, tw, W); claw = at(t, te, E)
    ok = bool(np.isfinite(w[24]) and np.any(w[45:51] != 0.0) and t < 47.02)  # the node left DIRECT at 47.02: debug zeros after
    q = at(t, tm, Q)  # w x y z, world <- body
    qw, qx, qy, qz = q
    R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                  [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                  [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    i = min(np.searchsorted(to, t), len(to) - 1)
    dyaw = float(np.degrees(np.angle(np.exp(1j * (oyaw[i] - yaw0)))))
    j = at(t, tj, J)
    flown.append(dict(
        t=round(float(t), 2),
        com=fr(w[48:51]) if ok else None,
        claw=fr(claw),
        ref=fr(claw - w[24:27]) if ok else None,
        anchor=round(float(w[129]), 2) if ok else None,
        base=fr(at(t, tm, P)), bx=frv(R[:, 0]), by=frv(R[:, 1]),
        ex=frv(Rq(at(t, te, QE))[:, 0]), ey=frv(Rq(at(t, te, QE))[:, 1]),
        sk=[fr(at(t, ts, SK)[3 * n:3 * n + 3]) for n in range(5)],
        grip=round(float(j[names.index("gripper_left_joint")]) * 1e3, 1),
        valve=round(dyaw, 1), tilt=round(float(at(t, tm, tilt)), 1)))

out = dict(
    frame="origin = planned grasp point; f = nose (world -y), l = left (world +x), z up; m",
    t_plan=T_PLAN, phases=PHASES, plan=plan, flown=flown,
    valve=dict(axis=fr(ax0), radius=0.160, fin=dict(thick=0.005, width=0.040, top=0.025, bottom=-0.105),
               beam=dict(r0=-0.065, r1=0.185, half=0.025, z0=round(float(ax0[2] + 0.060 - G0[2]), 4),
                         z1=round(float(ax0[2] + 0.074 - G0[2]), 4))),
    table=dict(corners=[fr([0.446, -0.637, 0.779]), fr([0.453, 0.363, 0.779]), fr([0.603, 0.362, 0.779]),
                        fr([0.596, -0.638, 0.779])]))
json.dump(out, open(os.path.join(HERE, "..", "logic_compare", "abort3d.json"), "w"), separators=(",", ":"))
print("plan rows", len(plan), "first", plan[0], "last", plan[-1])
print("flown rows", len(flown), "axis", out["valve"]["axis"])
r = flown[int((44.4 - 43.40) / 0.02)]
print("44.40:", r["base"], r["com"], r["claw"], r["ref"], r["sk"][-1], r["anchor"], r["grip"], r["valve"])
