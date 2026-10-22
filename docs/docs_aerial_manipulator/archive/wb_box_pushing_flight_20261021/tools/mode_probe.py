#!/usr/bin/env python3
"""What is the 0.55 Hz world-held swing? Body roll/pitch (mocap, the vehicle's own axes), the law's
attitude error e_R, the commanded body torque, the arm swing (q2-q3)/2 and the CoM error along the
arm, in the CoM-anchored hover vs the world-held window of 170123 (and the Isaac v21_* runs):
amplitude at the dominant frequency and the phase of each signal against the arm swing."""
import os
import numpy as np
from bp_common import Bag
from phases import sim_run


def rpy(q):
    w, x, y, z = q.T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def body_tilt(q):
    """roll/pitch about the vehicle's own heading (yaw removed)"""
    r, p, y = rpy(q)
    return np.degrees(r), np.degrees(p)


def lockin(t, x, f):
    """amplitude and phase of x at frequency f (least squares, detrended)"""
    A = np.column_stack([np.cos(2 * np.pi * f * t), np.sin(2 * np.pi * f * t), np.ones_like(t), t])
    c, *_ = np.linalg.lstsq(A, x, rcond=None)
    return np.hypot(c[0], c[1]), np.degrees(np.arctan2(-c[1], c[0]))


def dominant(t, x):
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), t[1] - t[0])
    m = (f > 0.08) & (f < 3)
    return f[m][np.argmax(P[m])]


def probe(name, tb, qb, tj, J, tw, W, t0, t1, f=None):
    fs = 50.0
    t = np.arange(t0, t1, 1 / fs)
    roll, pitch = body_tilt(qb)
    sig = {
        "swing (q2-q3)/2 [deg]": np.interp(t, tj, np.degrees(J[:, 0] - J[:, 1]) / 2),
        "body roll [deg]": np.interp(t, tb, roll),
        "body pitch [deg]": np.interp(t, tb, pitch),
        "|e_R| [mrad]": np.interp(t, tw, 1e3 * np.linalg.norm(W[:, 28:31], axis=1)),
        "tau_body x [N.m]": np.interp(t, tw, W[:, 21]),
        "tau_body y [N.m]": np.interp(t, tw, W[:, 22]),
        "CoM err y [mm]": np.interp(t, tw, 1e3 * (W[:, 49] - W[:, 46])),
        "CoM err x [mm]": np.interp(t, tw, 1e3 * (W[:, 48] - W[:, 45])),
    }
    if f is None:
        f = dominant(t, sig["swing (q2-q3)/2 [deg]"])
    a0, p0 = lockin(t - t0, sig["swing (q2-q3)/2 [deg]"], f)
    print(f"\n{name}  {t1-t0:.1f} s, at {f:.2f} Hz")
    for k, x in sig.items():
        a, p = lockin(t - t0, x, f)
        rms = np.std(x - np.polyval(np.polyfit(t, x, 1), t))
        print(f"  {k:24s} amp {a:7.3f}  (rms {rms:7.3f})  phase vs swing {((p - p0 + 180) % 360) - 180:+6.0f} deg")


b = Bag("170123")
js = b.f("js", "position")
J = np.column_stack([js[:, 2], js[:, 0], js[:, 1], js[:, 3]])[:, 1:3]   # q2, q3
qb = np.column_stack([b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"])
args = (b.t("mocap"), qb, b.t("js"), J, b.t("wb"), b.f("wb", "data"))
probe("HW hover above handle, CoM anchor", *args, 31.4, 36.5, f=0.55)
probe("HW world-held, jaws open + contact", *args, 37.3, 46.6)
for tag in ("v21_1", "v21_2", "v21_3"):
    s = sim_run(tag)
    m = s["marks"]
    probe(f"SIM {tag} world-held + contact", s["base"][:, 0], s["base"][:, 4:8], s["joints"][:, 0],
          s["joints"][:, 2:4], s["dbg"][:, 0], s["dbg"][:, 2:], m["ready:end"], m["release:start"])
