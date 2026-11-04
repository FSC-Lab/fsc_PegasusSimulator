#!/usr/bin/env python3
"""192853: time series for the "fin against the U" page (../viz/). Top view in the frame of the vehicle at 32.0 s
(up = its nose, right = its right), origin = the valve's shaft at rest. Writes ../viz/fin_in_u_192853.json."""
import os, json
import numpy as np
from vt_common import Bag, yaw_deg, tilt_deg
HERE = os.path.dirname(os.path.abspath(__file__))
AX = np.array([-0.0935, -0.0014, -0.0899])           # obj_0 -> bearing axis (rotor frame), the flown yaml
FIN_R, FIN_Y, FIN_HL, FIN_HT, BRACKET = 0.160, 0.0006, 0.020, 0.0025, 0.008
KM, KF, W_IDLE, W_MAX = 0.018164, 4.260431e-05, 64.0603, 730.0507
F_IDLE = KF * W_IDLE ** 2
IZ = 0.12                                             # yaw inertia with the arm, approx. [kg m^2]
def rotor_f(u):                                       # the allocator's own map: command -> omega (linear) -> f = kf w^2
    return KF * (W_IDLE + np.clip(u, 0, 1) * (W_MAX - W_IDLE)) ** 2
def R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
b = Bag("192853")
T = np.round(np.arange(30.0, 34.40, 0.02), 3)
def interp(t, X):
    X = np.asarray(X, float); o = np.argsort(t); t = np.asarray(t)[o]; X = X[o]
    t, k = np.unique(t, return_index=True); X = X[k]
    return np.stack([np.interp(T, t, X[:, i]) for i in range(X.shape[1])], 1) if X.ndim == 2 else np.interp(T, t, X)
# vehicle (mocap)
tv = b.t("vrpn"); Qv = np.stack([b.f("vrpn", f"pose.orientation.{c}") for c in "wxyz"], 1)
Pv = np.stack([b.f("vrpn", f"pose.position.{c}") for c in "xyz"], 1)
yv = np.unwrap(np.radians(yaw_deg(*Qv.T)))
base = interp(tv, Pv); psi = interp(tv, yv); tilt = interp(tv, tilt_deg(*Qv.T))
# valve (obj_0)
to = b.t("obj"); Po = np.stack([b.f("obj", f"pose.position.{c}") for c in "xyz"], 1); Qo = np.stack([b.f("obj", f"pose.orientation.{c}") for c in "wxyz"], 1)
yo = np.unwrap(np.radians(yaw_deg(*Qo.T))); y_get = np.median(yo[(to > 9) & (to < 11)])
rest = (to > 28) & (to < 32)
axis0 = np.mean([Po[i] + R(Qo[i]) @ AX for i in np.where(rest)[0]], 0)
axis = np.array([Po[i] + R(Qo[i]) @ AX for i in range(len(to))])
fin = np.array([Po[i] + R(Qo[i]) @ np.array([FIN_R, FIN_Y, 0.0]) + R(Qo[i]) @ AX for i in range(len(to))])
axisI = interp(to, axis); finI = interp(to, fin); valve = interp(to, yo - y_get)
# claw (planner FK) and the law
te = b.t("pl_current_ee"); E = np.stack([b.f("pl_current_ee", f"pose.position.{c}") for c in "xyz"], 1); claw = interp(te, E)
tw = b.t("wb"); W = np.asarray(b.f("wb", "data"), float)
dt = np.median(np.diff(tw)); a = dt / 0.5; F = np.nan_to_num(W[:, 97:100]); Fl = np.zeros_like(F)
for i in range(1, len(F)):
    Fl[i] = Fl[i - 1] + a * (F[i] - Fl[i - 1])
WI = interp(tw, W[:, :120]); FlI = interp(tw, Fl)
# the yaw the rotors delivered (allocator model, from the four motor commands; FR ccw, RL ccw, FL cw, RR cw)
fI = rotor_f(WI[:, 41:45]); tau_r = KM * (fI[:, 2] + fI[:, 3] - fI[:, 0] - fI[:, 1])
calm = (T > 30.0) & (T < 32.5); tau_sd = -np.mean(tau_r[calm])     # the standing loose-arm twist (held off in hover)
# the yaw torque from the contact (CCW +). The inertial term IZ*psi_ddot is left out: psi went 0 -> -5 deg in ~1 s,
# i.e. under 0.02 N.m, while a double difference of the mocap yaw is +-0.1 N.m of noise.
tau_e = -tau_r - tau_sd
# the display frame: the vehicle at 32.0 s
i0 = np.searchsorted(T, 32.0); p0 = psi[i0]
nose0 = np.array([np.cos(p0), np.sin(p0)]); right0 = np.array([np.sin(p0), -np.cos(p0)])
D = lambda v: [round(float(v[0] @ right0 if False else v @ right0) * 1e3, 1), round(float(v @ nose0) * 1e3, 1)]
rows = []
for k, t in enumerate(T):
    w = WI[k]
    nose = np.array([np.cos(psi[k]), np.sin(psi[k])]); right = np.array([np.sin(psi[k]), -np.cos(psi[k])])
    c = claw[k, :2]; tgt = c - w[24:26]
    vrel = valve[k]                                      # rad, CCW positive, since the Get
    yo_k = np.interp(t, to, yo)
    tdir = np.array([-np.sin(yo_k), np.cos(yo_k)])      # rotor +y = the fin's length (the push direction)
    fc = finI[k, :2]
    rel = np.degrees(np.arctan2(tdir @ (-right), tdir @ nose))   # fin length vs the U's axis (= the nose), CCW+
    d = fc - c
    rows.append(dict(
        t=float(t), base=D(base[k, :2] - axis0[:2]), psi=round(float(np.degrees(psi[k] - p0)), 2),
        claw=D(c - axis0[:2]), tgt=D(tgt - axis0[:2]), axis=D(axisI[k, :2] - axis0[:2]), fin=D(fc - axis0[:2]),
        fin_ang=round(float(np.degrees(np.arctan2(tdir @ (-right0), tdir @ nose0))), 2),
        valve=round(float(np.degrees(vrel)), 2), rel=round(float(rel), 2),
        fin_u=round(float(d @ nose) * 1e3, 1), fin_s=round(float(d @ right) * 1e3, 1),
        side_err=round(float((c - tgt) @ right) * 1e3, 1),          # claw right of its target, mm
        f_right=round(float(FlI[k, :2] @ right), 3), f_nose=round(float(FlI[k, :2] @ nose), 3),
        tj1=round(float(w[13]), 3), tz=round(float(w[23]), 3), u1=round(float(w[17]), 2),
        budget=round(float(KM * (w[17] - 4 * F_IDLE)), 3), m=[round(float(x), 3) for x in w[41:45]],
        unal_t=round(float(w[55]), 2), tilt=round(float(tilt[k]), 2),
        tau_r=round(float(tau_r[k]), 3), tau_e=round(float(tau_e[k]), 3),
        lever=round(float((c - w[48:50]) @ nose), 3),
        f_yaw=round(float(-tau_e[k] / max((c - w[48:50]) @ nose, 0.05)), 3)))
ev = [(30.21, "Ready starts (slide-in, 3.0 s)"), (33.10, "valve starts to turn: the bracket meets the fin"),
      (33.22, "Ready ends"), (33.70, "CCW pair at idle: yaw at its limit"), (33.90, "all four motors on a limit"),
      (34.28, "sway guard: abort")]
out = os.path.join(HERE, "..", "viz", "fin_in_u_192853.json")
json.dump(dict(rows=rows, events=ev, f_idle=F_IDLE, km=KM, tau_sd=round(float(tau_sd), 3)), open(out, "w"), separators=(",", ":"))
print(out, len(rows))
print("standing twist held off in hover", round(float(tau_sd), 3))
print("  t   base          claw         psi  valve  rel  fin_s  F_hat_right  F_yawbal  tau_r  tz  budget  lever")
for r in rows[::10]:
    if r["t"] < 31.9: continue
    print(f'{r["t"]:5.2f} {r["base"]} {r["claw"]} {r["psi"]:+5.1f} {r["valve"]:+5.1f} {r["rel"]:+5.1f} {r["fin_s"]:+5.1f}  {r["f_right"]:+5.2f}  {r["f_yaw"]:+5.2f}  {r["tau_r"]:.3f} {r["tz"]:.3f} {r["budget"]:.3f} {r["lever"]:.3f}')
