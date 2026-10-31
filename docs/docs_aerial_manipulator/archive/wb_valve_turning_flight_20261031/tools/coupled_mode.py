#!/usr/bin/env python3
"""Is the wrist swing a coupled mode with the airframe? Over a window, the amplitude and phase (relative to q4)
of every signal at the swing's frequency: q1, the base yaw, the base lateral / along-nose position (body frame
of the hold heading), the base roll/pitch, the claw position error e_y[0..2], the heading error e_y[3], the
heading-target offset [119], the free-axis offset [117], and the law's j1/j4 torques (model convention).
Usage: coupled_mode.py <tag> <t0> <t1>"""
import sys, numpy as np
from vt_common import Bag, yaw_deg
tag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
b = Bag(tag); fs = 100.0; tu = np.arange(t0, t1, 1/fs)
names = [x.strip(" '[]") for x in str(b.f("js","name")[0]).split(",")]
tj = b.t("js"); P = b.f("js","position")
q4 = np.interp(tu, tj, np.degrees(P[:, names.index("joint4")])); q1 = np.interp(tu, tj, np.degrees(P[:, names.index("joint1")]))
q2 = np.interp(tu, tj, np.degrees(P[:, names.index("joint2")])); q3 = np.interp(tu, tj, np.degrees(P[:, names.index("joint3")]))
tm = b.t("mocap"); g = lambda k: b.f("mocap", k)
qw, qx, qy, qz = (g(f"pose.orientation.{c}") for c in "wxyz")
yaw = np.unwrap(np.radians(yaw_deg(qw, qx, qy, qz)))
roll = np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))); pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy-qz*qx), -1, 1)))
px, py = g("pose.position.x"), g("pose.position.y")
yawu = np.interp(tu, tm, yaw); psi0 = yawu.mean()
xu, yu = np.interp(tu, tm, px), np.interp(tu, tm, py)
along = 1e3*(np.cos(psi0)*xu + np.sin(psi0)*yu); lat = 1e3*(-np.sin(psi0)*xu + np.cos(psi0)*yu)
tw = b.t("wb"); W = b.f("wb","data"); ok = np.array([len(r) >= 120 for r in W]); Wm = np.array([r[:120] for r in W[ok]]); tw = tw[ok]
wi = lambda k: np.interp(tu, tw, Wm[:, k])
sig = {"q4 [deg]": q4, "q1 [deg]": q1, "q2 [deg]": q2, "q3 [deg]": q3, "base yaw [deg]": np.degrees(yawu),
       "base roll [deg]": np.interp(tu, tm, roll), "base pitch [deg]": np.interp(tu, tm, pitch),
       "base along-nose [mm]": along, "base lateral [mm]": lat,
       "e_y x [mm]": 1e3*wi(24), "e_y y [mm]": 1e3*wi(25), "e_y z [mm]": 1e3*wi(26), "e_y psi [deg]": np.degrees(np.arcsin(np.clip(wi(27), -1, 1))),
       "hdg offset [deg]": np.degrees(wi(119)), "free offset [mm]": 1e3*wi(117),
       "CoM err x [mm]": 1e3*(wi(48)-wi(45)), "CoM err y [mm]": 1e3*(wi(49)-wi(46)),
       "tau j1 law [Nm]": wi(13), "tau j4 law [Nm]": wi(16), "tau body z [Nm]": wi(23), "u1 [N]": wi(17)}
x = q4 - q4.mean(); w = np.hanning(len(x)); F = np.fft.rfft(x*w); f = np.fft.rfftfreq(len(x), 1/fs)
m = (f > 0.1) & (f < 3); k = np.where(m)[0][np.argmax(np.abs(F[m]))]; f0 = f[k]
print(f"== {tag} {t0}-{t1} s: swing at {f0:.3f} Hz (period {1/f0:.2f} s)")
ref = None
for name, s in sig.items():
    s = s - np.polyval(np.polyfit(tu, s, 1), tu)
    c = np.sum(s*w*np.exp(-2j*np.pi*f0*tu)) * 2 / w.sum()
    if ref is None: ref = c
    ph = np.degrees(np.angle(c/ref))
    print(f"   {name:22s} amp {np.abs(c):8.3f}   phase vs q4 {ph:+7.1f} deg   rms {s.std():8.3f}")
