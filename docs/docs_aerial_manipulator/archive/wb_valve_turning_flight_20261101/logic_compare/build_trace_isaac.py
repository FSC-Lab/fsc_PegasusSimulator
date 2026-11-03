#!/usr/bin/env python3
"""Add the U-push lane to trace.json: the Isaac end-to-end check n1 of archive/valve_upush_20261102 (the generated
valve yaml, the station's default, the station flying Ready / Turn / Exit by itself, the bare 5 mm fin).

Same columns, frame and sampling as build_trace.py's hardware lanes (t = 0 at the planner's DONE go_to_start;
origin = the planned grasp point; f = the vehicle's nose (world -y here too), l = its left, z up), from the test
driver's npz instead of a bag:
  base = the odometry (raw mocap), claw = the planner's current_ee, CoM reference = base - (x_c - x_cd),
  claw reference = claw - e_y (wb_control_debug[24..26]), grasp point = the planner's EE reference held after
  Ready, the point above the handle = the one held after Go To Start.
The gripper is Isaac's (0 rad open, -50 deg closed) mapped linearly onto the hardware reading the page's jaw band
uses (open +0.019 m, closed -0.010 m). Joint angles are Isaac's (model convention; q1 is near 0 anyway).
Also stores the claw-reference segments (follow / room, from the CoM-anchor weight wb_control_debug[129]) and
the contact flag ([120]).  Run with /usr/bin/python3 after build_trace.py."""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(A, "wb_valve_turning_flight_20261031", "tools"))
import vt_common
NPZ = os.path.join(A, "valve_upush_20261102", "sim", "runs", "n1_generated.npz")
T_GTS = 46.10                                   # the planner's DONE go_to_start, driver clock
T0, T1, DT = -3.0, 13.0, 0.05
F, L = np.array([0.0, -1.0, 0.0]), np.array([1.0, 0.0, 0.0])
z = np.load(NPZ, allow_pickle=True)


def at(t, ts, a):
    return a[min(np.searchsorted(ts, t), len(ts) - 1)]


ref = z["ref"]
grasp = at(T_GTS + 6.5, ref[:, 0], ref[:, 1:4])          # held from Ready's end to the Turn
approach = at(T_GTS + 1.5, ref[:, 0], ref[:, 1:4])       # held after Go To Start
od = z["odom"]; to = od[:, 0]; P = od[:, 1:4]
yaw_v = vt_common.yaw_deg(*od[:, 7:11].T)
assert abs(np.median(yaw_v[(to > T_GTS) & (to < T_GTS + 6)]) + 90.0) < 3.0, "the nose is not world -y"
tilt = vt_common.tilt_deg(*od[:, 7:11].T)
ee = z["ee"]; te = ee[:, 0]; E = ee[:, 1:4]
d = z["dbg"]; tw = d[:, 0]; W = d[:, 2:].astype(float)
jn = list(z["joint_names"]); js = z["joints"]; tj = js[:, 0]; J = js[:, 1:5]
gr = z["grip"]; tg = gr[:, 0]; G = 0.019 + gr[:, 1] / np.radians(50.0) * 0.029
bx = z["box"]; tb = bx[:, 0]; oyaw = vt_common.yaw_deg(*bx[:, 4:8].T)
oyaw0 = np.median(oyaw[(tb > T_GTS - 5) & (tb < T_GTS)])


def frame(p):
    dd = p - grasp
    return [round(float(dd @ F), 4), round(float(dd @ L), 4), round(float(dd[2]), 4)]


rows = []
for tr in np.arange(T0, T1 + 1e-9, DT):
    t = T_GTS + tr
    w = at(t, tw, W); base = at(t, to, P); claw = at(t, te, E)
    ex = w[48:51] - w[45:48]; ey = w[24:27]; j = at(t, tj, J)
    rows.append([round(tr, 2), *frame(base), *frame(base - ex), *frame(claw), *frame(claw - ey),
                 round(float(np.hypot(ex[0], ex[1]) * 1e3)), round(float(np.linalg.norm(claw - grasp) * 1e3)),
                 round(float(np.degrees(j[jn.index("joint1")])), 1), round(float(np.degrees(j[jn.index("joint3")])), 1),
                 round(float(at(t, tg, G)), 4), round(float(at(t, to, tilt)), 1), round(float(at(t, to, yaw_v)), 1),
                 round(float(((at(t, tb, oyaw) - oyaw0 + 180) % 360) - 180), 1)])

# the claw reference: CoM-anchor weight [129] (1 = follows the vehicle, 0 = held in the room); contact flag [120]
rel = tw - T_GTS
wgt, con = W[:, 129], W[:, 120]
i_dn = np.argmax((rel > 3) & (wgt < 0.999)); i_room = np.argmax((rel > 3) & (wgt < 0.001))
i_up = np.argmax((rel > 20) & (wgt > 0.001)); i_fol = np.argmax((rel > 20) & (wgt > 0.999))
i_con = np.argmax((rel > 3) & (con > 0.5)); i_coff = np.argmax((rel > 20) & (con < 0.5))
seg = lambda i: round(float(rel[i]), 2)
anchor = [[T0, seg(i_dn), "follow"], [seg(i_dn), seg(i_room), "toRoom"], [seg(i_room), T1, "room"]]
out = json.load(open(os.path.join(HERE, "trace.json")))
out["latest"] = dict(tag="isaac n1_generated", t_gts=T_GTS, approach=frame(approach), rows=rows, anchor=anchor,
                     contact_on=seg(i_con),
                     later=dict(room_until=seg(i_up), follow_from=seg(i_fol), contact_off=seg(i_coff)))
json.dump(out, open(os.path.join(HERE, "trace.json"), "w"), separators=(",", ":"))
r = np.array(rows)
print("latest", len(rows), "rows; approach", frame(approach), "grasp (world)", np.round(grasp, 4))
print("anchor", anchor, "contact on", seg(i_con), "| exit: room until", seg(i_up), "follows from", seg(i_fol),
      "contact off", seg(i_coff))
for k in (60, 120, 180, 200, 260, 319):
    print(f"t {r[k,0]:+.2f} base {r[k,1:4]} claw {r[k,7:10]} e_com {r[k,13]:.0f} claw-grasp {r[k,14]:.0f} "
          f"q1 {r[k,15]} q3 {r[k,16]} grip {r[k,17]} tilt {r[k,18]} dyaw {r[k,20]}")
