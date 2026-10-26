#!/usr/bin/env python3
"""Does Isaac show the hardware's contact-phase oscillation? For pl_mission.py recordings: the
vehicle's TRUE tilt (base_truth) in the 0.05-1 Hz band, along / across the push, in the open-jaw hold
before the close (step 'ready') vs clamped ('close' + 'push'); the box's speed statistics in the slide;
the arm's range. Same definitions as att_est_error.py / moment_fit.py on the flights.
Usage: isaac_contact_tilt.py <run.npz> ..."""
import sys, os
import numpy as np
STEPS = ["go_to_start", "ready", "close", "push", "release", "exit", "go_to_land_start", "land"]

def R_of(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
def band(t, x, f1=0.05, f2=1.0):
    T = np.arange(t[0], t[-1], 0.02); y = np.interp(T, t, x)
    X = np.fft.rfft(y - y.mean()); f = np.fft.rfftfreq(len(y), 0.02); X[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(X, len(y))

for p in sys.argv[1:]:
    z = np.load(p, allow_pickle=True)
    B = z["base_truth"]; t, st = B[:, 0], B[:, 8]
    tilt = np.array([R_of(q)[:, 2][:2] for q in B[:, 4:8]])
    info = z["info"]; yaw = np.radians(info[81]) if len(info) > 81 else -np.pi / 2
    u = np.array([np.cos(yaw), np.sin(yaw)]); n = np.array([-u[1], u[0]])
    out = []
    for nm, steps in (("open-jaw hold", [1]), ("clamped", [2, 3])):
        m = np.isin(st, steps)
        if m.sum() < 100:
            out.append(f"{nm}: --"); continue
        tt = t[m]
        a = np.degrees(np.std(band(tt, tilt[m] @ u))); c = np.degrees(np.std(band(tt, tilt[m] @ n)))
        out.append(f"{nm} ({tt[-1]-tt[0]:4.1f} s): tilt along {a:.2f} across {c:.2f} deg")
    bx = z["box"]; mb = bx[:, 8] == 3
    s = ""
    if mb.sum() > 50:
        tb = bx[mb, 0]; al = bx[mb, 1:3] @ u
        T = np.arange(tb[0], tb[-1], 0.02); v = np.gradient(np.interp(T, tb, al), 0.02) * 1e3
        v = np.convolve(v, np.ones(5) / 5, "same")
        s = f" | box in push: speed std {v.std():.0f} mm/s, back {np.mean(v < -5):.0%}"
    J = z["joints"]; mj = np.isin(J[:, 9], [2, 3]); jn = list(z["joint_names"])
    q2 = np.degrees(J[mj, 1 + jn.index("joint2")]) if "joint2" in jn else np.array([np.nan])
    q3 = np.degrees(J[mj, 1 + jn.index("joint3")]) if "joint3" in jn else np.array([np.nan])
    print(f"{os.path.basename(p):18s} " + "; ".join(out) + s + f" | q2 {q2.min():.0f}..{q2.max():.0f} q3 {q3.min():.0f}..{q3.max():.0f}")
