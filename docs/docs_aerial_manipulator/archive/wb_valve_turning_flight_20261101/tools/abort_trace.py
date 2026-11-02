#!/usr/bin/env python3
"""The 11-01 crash flight's abort (175932), 43.0-48.5 s bag time, in the valve's own frame.
Frame: origin = the live grasp point (the planned grasp point turned with the valve about its axis),
r = radial outward from the valve axis (= across the fin, the jaws' closing axis), t = tangential (the
vehicle's nose at the grasp), z up. Base = mocap position; its reference = base - CoM error
(wb_control_debug[48..50]-[45..47]). Claw = the planner's current_ee (arm FK on the encoders); its reference
= claw - EE task error ([24..26]). Prints a table every 0.1 s and writes ../npz/abort_175932.json.
Run with /usr/bin/python3."""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(A, "wb_valve_turning_flight_20261031", "tools"))
import vt_common
vt_common._c.NPZ = os.path.join(HERE, "..", "npz")
b = vt_common.Bag("175932")
AXOFF = np.array([-0.0935, -0.0014, -0.0899])     # obj_0 -> valve axis, obj_0 frame (11-01 build)
G0 = np.array([0.366, -0.141, 0.959])              # planned grasp point (Ready's hold capture)
T0, T1, DT = 43.0, 48.6, 0.05

def at(t, ts, a):
    return a[min(np.searchsorted(ts, t), len(ts) - 1)]

tm = b.t("mocap"); P = np.c_[[b.f("mocap", f"pose.position.{k}") for k in "xyz"]].T
qm = [b.f("mocap", f"pose.orientation.{k}") for k in "wxyz"]
tilt = vt_common.tilt_deg(*qm); yaw = vt_common.yaw_deg(*qm)
V = np.gradient(P, tm, axis=0)
te = b.t("pl_current_ee"); E = np.c_[[b.f("pl_current_ee", f"pose.position.{k}") for k in "xyz"]].T
tw = b.t("wb"); W = b.f("wb", "data"); ok = np.array([len(r) >= 118 for r in W])
Wm = np.array([r[:118] for r in W[ok]]); tw = tw[ok]
names = [x.strip(" '[]") for x in str(b.f("js", "name")[0]).split(",")]
tj = b.t("js"); J = np.array([np.asarray(r, float) for r in b.f("js", "position")])
to = b.t("obj"); O = np.c_[[b.f("obj", f"pose.position.{k}") for k in "xyz"]].T
qo = [b.f("obj", f"pose.orientation.{k}") for k in "wxyz"]; oyaw = np.radians(vt_common.yaw_deg(*qo))
tmo = b.t("motors"); M = np.array([np.asarray(r, float)[:4] for r in b.f("motors", "control")])

# the valve axis from obj_0 + its yaw (median over the 3 s before the jaws closed)
pre = (to > 40.0) & (to < 43.0)
yaw0 = float(np.median(np.unwrap(oyaw[pre])))
def axis_at(i):
    c, s = np.cos(oyaw[i]), np.sin(oyaw[i])
    return O[i] + np.array([c * AXOFF[0] - s * AXOFF[1], s * AXOFF[0] + c * AXOFF[1], AXOFF[2]])
ax0 = np.median(np.array([axis_at(i) for i in np.where(pre)[0]]), axis=0)
rad0 = (G0 - ax0)[:2]; rad0 /= np.linalg.norm(rad0)

rows = []
for t in np.arange(T0, T1 + 1e-9, DT):
    i = min(np.searchsorted(to, t), len(to) - 1)
    dyaw = float(np.angle(np.exp(1j * (oyaw[i] - yaw0))))
    c, s = np.cos(dyaw), np.sin(dyaw)
    rad = np.array([c * rad0[0] - s * rad0[1], s * rad0[0] + c * rad0[1]])
    tan = np.array([-rad[1], rad[0]])
    g = np.r_[ax0[:2] + rad * np.linalg.norm((G0 - ax0)[:2]), G0[2]]
    def fr(p):
        d = p - g; return [round(float(d[:2] @ rad) * 1e3), round(float(d[:2] @ tan) * 1e3), round(float(d[2]) * 1e3)]
    w = at(t, tw, Wm); base = at(t, tm, P); v = at(t, tm, V); claw = at(t, te, E)
    ex = w[48:51] - w[45:48]; ey = w[24:27]
    j = at(t, tj, J); m = at(t, tmo, M)
    vr = float(v[:2] @ rad); vt = float(v[:2] @ tan)
    rows.append(dict(t=round(float(t), 2), base=fr(base), base_ref=fr(base - ex), claw=fr(claw), claw_ref=fr(claw - ey),
                     v_r=round(vr * 1e3), v_t=round(vt * 1e3), v_z=round(float(v[2]) * 1e3),
                     com_err=round(float(np.hypot(ex[0], ex[1])) * 1e3),
                     grip=round(float(j[names.index("gripper_left_joint")]) * 1e3, 1),
                     q=[round(float(np.degrees(j[names.index(f"joint{k}")])), 1) for k in (1, 2, 3, 4)],
                     valve_dyaw=round(float(np.degrees(dyaw)), 1), valve_dz=round(float(O[i][2] - np.median(O[pre][:, 2])) * 1e3, 1),
                     F=[round(float(x), 1) for x in w[97:100]], tilt=round(float(at(t, tm, tilt)), 1),
                     yaw=round(float(at(t, tm, yaw)), 1), motors=[round(float(x), 2) for x in m]))
hdr = "   t   base r/t/z [mm]   base-ref r/t/z     v r/t [mm/s]  claw r/t/z      claw-ref r/t/z  grip  q1  q2  q3  dval  dz  |F| tilt motors"
print("valve axis", np.round(ax0, 3), "radial (world)", np.round(rad0, 3), "grasp->axis", round(float(np.linalg.norm((G0 - ax0)[:2])), 3))
print(hdr)
for r in rows[::2]:
    bb = np.array(r["base"]); br = np.array(r["base_ref"]); cc = np.array(r["claw"]); cr = np.array(r["claw_ref"])
    print(f"{r['t']:6.2f} {bb[0]:5d} {bb[1]:5d} {bb[2]:5d}  {bb[0]-br[0]:5d} {bb[1]-br[1]:5d} {bb[2]-br[2]:5d}  {r['v_r']:5d} {r['v_t']:5d}  "
          f"{cc[0]:5d} {cc[1]:5d} {cc[2]:5d}  {cc[0]-cr[0]:5d} {cc[1]-cr[1]:5d} {cc[2]-cr[2]:5d}  {r['grip']:5.1f} "
          f"{r['q'][0]:5.1f} {r['q'][1]:5.1f} {r['q'][2]:5.1f} {r['valve_dyaw']:6.1f} {r['valve_dz']:4.1f} {np.linalg.norm(r['F']):5.1f} {r['tilt']:5.1f} {r['motors']}")
json.dump(dict(axis=[round(float(x), 4) for x in ax0], rows=rows), open(os.path.join(HERE, "..", "npz", "abort_175932.json"), "w"),
          separators=(",", ":"))
