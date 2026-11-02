#!/usr/bin/env python3
"""Synchronized approach traces for the 'Valve-turning vs Box-pushing logic' page: the 10-27 box push A
(171114) and the 11-01 valve crash (175932), t = 0 at the planner's DONE go_to_start. Frame: origin at
the planned grasp point, f = the vehicle's nose (world -y on both flights), l = left of the nose (+x), z up.
Claw reference = law EE position - EE task error (wb_control_debug[24..26]); CoM reference = base - CoM error."""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(A, "wb_valve_turning_flight_20261031", "tools"))
import vt_common
FLIGHTS = {
  "box": dict(npz="wb_box_pushing_flight_20261027/npz", tag="171114", t_gts=41.31,
              grasp=[0.771, 0.219, 1.030], approach=[0.771, 0.339, 1.180]),
  "valve": dict(npz="wb_valve_turning_flight_20261101/npz", tag="175932", t_gts=36.41,
                grasp=[0.366, -0.141, 0.959], approach=[0.365, -0.021, 1.059], t_end=10.75),
}
T0, T1, DT = -3.0, 13.0, 0.05
F, L = np.array([0.0, -1.0, 0.0]), np.array([1.0, 0.0, 0.0])
out = {}
for name, c in FLIGHTS.items():
    vt_common._c.NPZ = os.path.join(A, c["npz"]); b = vt_common.Bag(c["tag"])
    g = np.array(c["grasp"])
    def frame(p): d = p - g; return [round(float(d @ F), 4), round(float(d @ L), 4), round(float(d[2]), 4)]
    tm = b.t("mocap"); P = np.c_[[b.f("mocap", f"pose.position.{k}") for k in "xyz"]].T
    q = [b.f("mocap", f"pose.orientation.{k}") for k in "wxyz"]; tilt = vt_common.tilt_deg(*q)
    yaw = vt_common.yaw_deg(*q)
    te = b.t("pl_current_ee"); E = np.c_[[b.f("pl_current_ee", f"pose.position.{k}") for k in "xyz"]].T
    tw = b.t("wb"); W = b.f("wb", "data"); ok = np.array([len(r) >= 118 for r in W])
    Wm = np.array([r[:118] for r in W[ok]]); tw = tw[ok]
    names = [x.strip(" '[]") for x in str(b.f("js", "name")[0]).split(",")]
    tj = b.t("js"); J = b.f("js", "position")
    to = b.t("obj"); oyaw = vt_common.yaw_deg(*(b.f("obj", f"pose.orientation.{k}") for k in "wxyz"))
    oyaw0 = np.median(oyaw[(to > c["t_gts"] - 5) & (to < c["t_gts"])])
    def at(t, ts, a): return a[min(np.searchsorted(ts, t), len(ts) - 1)]
    rows = []
    for tr in np.arange(T0, T1 + 1e-9, DT):
        t = c["t_gts"] + min(tr, c.get("t_end", 99.0))   # the crash flight is frozen just after SAFETY
        w = at(t, tw, Wm); base = at(t, tm, P); claw = at(t, te, E)
        ex = w[48:51] - w[45:48]; ey = w[24:27]
        j = at(t, tj, J)
        rows.append([round(tr, 2), *frame(base), *frame(base - ex), *frame(claw), *frame(claw - ey),
                     round(float(np.hypot(ex[0], ex[1]) * 1e3)), round(float(np.linalg.norm(claw - g) * 1e3)),
                     round(float(np.degrees(j[names.index("joint1")])), 1),
                     round(float(np.degrees(j[names.index("joint3")])), 1),
                     round(float(j[names.index("gripper_left_joint")]), 4),
                     round(float(at(t, tm, tilt)), 1), round(float(at(t, tm, yaw)), 1),
                     round(float(((at(t, to, oyaw) - oyaw0 + 180) % 360) - 180), 1)])
    out[name] = dict(tag=c["tag"], t_gts=c["t_gts"], approach=frame(np.array(c["approach"])), rows=rows)
out["cols"] = ["t", "bf", "bl", "bz", "rf", "rl", "rz", "cf", "cl", "cz", "kf", "kl", "kz",
               "e_com_mm", "claw_to_grasp_mm", "q1", "q3", "grip", "tilt", "yaw", "handle_dyaw"]
json.dump(out, open(os.path.join(HERE, "trace.json"), "w"), separators=(",", ":"))
for n in ("box", "valve"):
    r = np.array(out[n]["rows"]); print(n, len(r), "rows; base f/l/z at t=0:", r[60, 1:4], "claw:", r[60, 7:10], "claw ref:", r[60, 10:13])
