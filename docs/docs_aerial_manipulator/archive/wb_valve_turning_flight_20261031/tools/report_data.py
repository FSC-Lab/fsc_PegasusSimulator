#!/usr/bin/env python3
"""Chart data for the 10-31 valve report (report_template.html -> ../report.html via build_report.py).
    PYTHONNOUSERSITE=1 /usr/bin/python3 report_data.py > ../report_data.json
fig1 = the evening flight 211808, 21.6-27.4 s at 25 Hz: base offset from the room-hold point in the hold's nose frame
(lateral left +, along forward +, mm), q1..q4 (deg), tilt (deg), the law's roll / pitch attitude error (deg),
plus the event times. fig2 = all four flights from the room hold (-0.5 .. +4.3 s): lateral offset (mm) and q3 (deg)."""
import json
import numpy as np
from vt_common import Bag, yaw_deg, tilt_deg

BUDGET = 0.654
FS = 25.0


def base_frame(b, t_hold):
    tm = b.t("mocap"); g = lambda k: b.f("mocap", k)
    qw, qx, qy, qz = (g(f"pose.orientation.{c}") for c in "wxyz")
    x, y = g("pose.position.x"), g("pose.position.y")
    k = min(np.searchsorted(tm, t_hold), len(tm) - 1)
    psi0 = np.radians(yaw_deg(qw[k], qx[k], qy[k], qz[k]))
    dx, dy = x - x[k], y - y[k]
    along = np.cos(psi0) * dx + np.sin(psi0) * dy
    lat = -np.sin(psi0) * dx + np.cos(psi0) * dy
    return tm, 1e3 * lat, 1e3 * along, tilt_deg(qw, qx, qy, qz)


def joints(b):
    names = [s.strip(" '[]") for s in str(b.f("js", "name")[0]).split(",")]
    P = np.degrees(b.f("js", "position"))
    return b.t("js"), {f"q{k}": P[:, names.index(f"joint{k}")] for k in (1, 2, 3, 4)}


def r(a, n=1):
    return [None if not np.isfinite(v) else round(float(v), n) for v in a]


def t_hold_of(b):
    return [t for t, s in b.edges("pl_pstatus") if s.startswith("DONE go_to_start")][0]


out = {}

# ---- figure 1: the evening flight
b = Bag("211808")
t_hold = t_hold_of(b)
tu = np.arange(21.6, 27.4, 1 / FS)
tm, lat, along, tilt = base_frame(b, t_hold)
tj, Q = joints(b)
tw = b.t("wb"); W = b.f("wb", "data")
ok = np.array([len(r_) >= 120 for r_ in W]); Wm = np.array([r_[:120] for r_ in W[ok]]); tw = tw[ok]
# the law's attitude error (model axes: x = pitch, y = roll; small-angle, deg)
eRp = np.degrees(np.interp(tu, tw, Wm[:, 28])); eRr = np.degrees(np.interp(tu, tw, Wm[:, 29]))
eRp[tu > tw[-1]] = np.nan; eRr[tu > tw[-1]] = np.nan   # SAFETY from 27.24 s: the law stops ticking
ev = {}
for t, s in b.edges("pl_pstatus"):
    if s.startswith("DONE go_to_start") and "hold" not in ev: ev["hold"] = t
    if s.startswith("FLYING ready") and "ready" not in ev: ev["ready"] = t
    if s.startswith("ABORTING") and "abort" not in ev: ev["abort"] = t
q3u = np.interp(tu, tj, Q["q3"]); ev["q3zero"] = float(tu[np.argmax((tu > ev["ready"]) & (q3u <= 0))])
for t, s in b.edges("wbmode"):
    if s == "SAFETY" and t > ev["abort"]: ev["safety"] = t; break
ev["watchdog"] = 26.00               # rosout: DIRECT WATCHDOG TRIPPED 15.1/15.0 deg (emergency abort)
out["fig1"] = {
    "t": r(tu - 0, 2),
    "lat": r(np.interp(tu, tm, lat)), "along": r(np.interp(tu, tm, along)),
    **{k: r(np.interp(tu, tj, v)) for k, v in Q.items()},
    "tilt": r(np.interp(tu, tm, tilt), 2), "eRr": r(eRr, 2), "eRp": r(eRp, 2),
    "events": {k: round(float(v), 2) for k, v in ev.items()},
}

# ---- figure 2: the four flights from the room hold
fl = []
for tag, label in (("174943", "17:49 hold"), ("175725", "17:57 hold"), ("180423", "18:04 turn"), ("211808", "21:18 turn")):
    b = Bag(tag); th = t_hold_of(b)
    tr = [t for t, s in b.edges("pl_pstatus") if s.startswith("FLYING ready")][0]
    ta = [t for t, s in b.edges("pl_pstatus") if s.startswith("ABORTING")]
    tm, lat, along, tilt = base_frame(b, th); tj, Q = joints(b)
    rel = np.arange(-0.5, 4.3, 1 / FS)
    fl.append({"tag": tag, "label": label, "evening": tag == "211808",
               "ready": round(tr - th, 2), "abort": round(ta[0] - th, 2) if ta and ta[0] - th < 4.3 else None,
               "lat": r(np.interp(rel + th, tm, lat)), "q3": r(np.interp(rel + th, tj, Q["q3"]))})
out["fig2"] = {"t": r(rel, 2), "flights": fl}

# ---- figure 3: the 11-01 pre-grasp protections replayed on the evening flight (archive/valve_room_hold_guards_20261031)
# with the valve yamls' values: Ready gate CoM error <= 30 mm and its rate <= 60 mm/s for 0.3 s; leash 40 mm for
# 0.1 s; sway guard 150 mm/s; the CoM error = |wb[48,49] - wb[45,46]|, its rate low-passed 0.25 s (the sway guard's
# filter); the node's anchor hand-back = a linear ramp over wb_ee_anchor_blend_s (stepEeAnchor), 1.0 s today.
b = Bag("211808")
ed = b.edges("pl_pstatus")
th = [t for t, s in ed if s.startswith("DONE go_to_start")][0]
tr = [t for t, s in ed if s.startswith("FLYING ready")][0]
ta = [t for t, s in ed if s.startswith("ABORTING")][0]
tw = b.t("wb"); W = b.f("wb", "data")
ok = np.array([len(r_) >= 120 for r_ in W]); Wm = np.array([r_[:120] for r_ in W[ok]]); tw = tw[ok]
e = np.c_[Wm[:, 48] - Wm[:, 45], Wm[:, 49] - Wm[:, 46]]
en = 1e3 * np.linalg.norm(e, axis=1)
v = np.zeros(2); vn = np.zeros(len(tw))
for k in range(1, len(tw)):
    dt = tw[k] - tw[k - 1]
    if 0 < dt < 0.1:
        v += (1 - np.exp(-dt / 0.25)) * ((e[k] - e[k - 1]) / dt - v)
    vn[k] = 1e3 * np.linalg.norm(v)
m = tw >= th
t_gate = float(tw[m][np.argmax(en[m] > 30.0)])          # the base leaves the Ready gate's 30 mm
s0 = None; t_leash = None
for k in np.where(m)[0]:
    if en[k] > 40.0:
        s0 = tw[k] if s0 is None else s0
        if tw[k] - s0 >= 0.1 - 1e-9:
            t_leash = float(tw[k]); break
    else:
        s0 = None
tj, Q = joints(b)
q3 = Q["q3"]; t_q3 = float(tj[np.argmax((tj > tr) & (q3 <= 0))])
t_rate = 23.56                                          # guard_replay.py: the q3 rate guard, flown values
tu = np.arange(22.0, 24.4, 1 / FS)
hb = lambda T: np.clip((tu - t_leash) / T, 0, 1) * 100   # noqa: E731
out["fig3"] = {
    "t": r(tu, 2), "e": r(np.interp(tu, tw, en)), "v": r(np.interp(tu, tw, vn)), "q3": r(np.interp(tu, tj, q3)),
    "hb10": r(hb(1.0)), "hb025": r(hb(0.25)),
    "events": {"hold": round(th, 2), "gate": round(t_gate, 2), "leash": round(t_leash, 2), "ready": round(tr, 2),
               "rate": t_rate, "q3zero": round(t_q3, 2), "abort": round(ta, 2)},
    "at": {"e_ready": round(float(np.interp(tr, tw, en))), "v_ready": round(float(np.interp(tr, tw, vn))),
           "q3_leash": round(float(np.interp(t_leash, tj, q3)), 1),
           "hb_q3zero": round(float(np.clip((t_q3 - t_leash) / 1.0, 0, 1)) * 100)},
}
print(json.dumps(out, separators=(",", ":")))
