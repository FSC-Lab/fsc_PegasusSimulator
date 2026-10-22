#!/usr/bin/env python3
"""Frequency and amplitude of the base wander and of the q2/q3 swing: hardware hover (CoM anchor,
164338 above the handle and 170123 before the world anchor), hardware world-held + contact
(170123), and the Isaac runs (v21_*) world-held + contact. Prints the dominant frequency (Welch-like
averaged periodogram, detrended) and the rms of the base along the arm (world y, the push axis),
across it (world x), and of q2 - q3 / 2."""
import os
import numpy as np
from bp_common import Bag
from phases import hw_run, sim_run, win


def resample(a, t0, t1, cols, fs=50.0):
    t = np.arange(t0, t1, 1 / fs)
    return t, np.column_stack([np.interp(t, a[:, 0], a[:, c]) for c in cols])


def dom(x, fs=50.0):
    x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
    n = len(x)
    w = np.hanning(n)
    P = np.abs(np.fft.rfft(x * w)) ** 2
    f = np.fft.rfftfreq(n, 1 / fs)
    m = (f > 0.08) & (f < 3.0)
    k = np.argmax(P[m])
    return f[m][k], np.sqrt(np.mean(x ** 2))


def report(name, r, t0, t1):
    t, B = resample(r["base"], t0, t1, [1, 2, 3])
    _, J = resample(r["joints"], t0, t1, [2, 3])
    d = win(r["dbg"], t0, t1)
    _, E = resample(r["dbg"], t0, t1, [2 + 48, 2 + 49, 2 + 45, 2 + 46])
    fx, ax = dom(B[:, 0]); fy, ay = dom(B[:, 1]); fz, az = dom(B[:, 2])
    sw = np.degrees(J[:, 0] - J[:, 1]) / 2
    fq, aq = dom(sw)
    fe, ae = dom(E[:, 1] - E[:, 3])
    print(f"{name:34s} {t1-t0:5.1f}s | base x {1e3*ax:5.1f}mm @{fx:4.2f}Hz  y {1e3*ay:5.1f}mm @{fy:4.2f}Hz  z {1e3*az:4.1f}mm @{fz:4.2f}Hz"
          f" | CoM err y {1e3*ae:5.1f}mm @{fe:4.2f}Hz | (q2-q3)/2 {aq:5.1f}deg @{fq:4.2f}Hz")


hw = hw_run()
report("HW 170123 hover above handle (CoM)", hw, 31.4, 36.6)
report("HW 170123 world-held, jaws open", hw, 37.3, 38.9)
report("HW 170123 world-held, clamped+push", hw, 39.6, 46.6)
report("HW 170123 world-held all", hw, 37.3, 46.6)
b = Bag("164338")
r2 = {"base": np.column_stack([b.t("mocap")] + [b.f("mocap", f"pose.position.{c}") for c in "xyz"]),
      "joints": np.column_stack([b.t("js"), b.f("js", "position")[:, [2, 0, 1, 3]]]),
      "dbg": np.column_stack([b.t("wb"), np.zeros(len(b.t("wb"))), b.f("wb", "data")])}
report("HW 164338 hover above 'handle' (CoM)", r2, 39.5, 55.3)
for tag in ("v21_1", "v21_2", "v21_3"):
    s = sim_run(tag)
    m = s["marks"]
    report(f"SIM {tag} world-held, clamped+push", s, m["ready:end"], m["release:start"])
    # sim hover: the go_to_start hold is short; use the 4 s before go_to_start (takeoff hover)
    report(f"SIM {tag} hover before go_to_start", s, m["go_to_start:start"] - 8, m["go_to_start:start"])
