#!/usr/bin/env python3
"""Phase 1: does an Isaac condition reproduce the 10-21 hardware push? Prints, for each run, the
same lines as ../../wb_box_pushing_flight_20261021/spectra.txt (the hardware's are repeated first):
base wander and the q2/q3 swing (q2-q3)/2 amplitude (rms, detrended) + dominant frequency, in
  * the hover before Go To Start (8 s, the CoM anchor -- the hardware's 'hover above handle')
  * the world-held window ready:end -> release:start (the hardware's 'world-held all')

    /usr/bin/python3 calib.py ../runs/cal_*.npz
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
HWTXT = os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "spectra.txt")


def resample(a, t0, t1, cols, fs=50.0):
    t = np.arange(t0, t1, 1 / fs)
    return t, np.column_stack([np.interp(t, a[:, 0], a[:, c]) for c in cols])


def dom(x, fs=50.0):
    x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
    n = len(x)
    P = np.abs(np.fft.rfft(x * np.hanning(n))) ** 2
    f = np.fft.rfftfreq(n, 1 / fs)
    m = (f > 0.08) & (f < 3.0)
    return f[m][np.argmax(P[m])], np.sqrt(np.mean(x ** 2))


def report(name, z, t0, t1):
    base, joints, dbg = z["base_truth"], z["joints"], z["dbg"].astype(float)
    t, B = resample(base, t0, t1, [1, 2, 3])
    _, J = resample(joints, t0, t1, [2, 3])
    _, E = resample(dbg, t0, t1, [2 + 48, 2 + 49, 2 + 45, 2 + 46])
    fx, ax = dom(B[:, 0]); fy, ay = dom(B[:, 1]); fz, az = dom(B[:, 2])
    fq, aq = dom(np.degrees(J[:, 0] - J[:, 1]) / 2)
    fe, ae = dom(E[:, 1] - E[:, 3])
    print(f"{name:36s} {t1-t0:5.1f}s | base x {1e3*ax:5.1f}mm @{fx:4.2f}Hz  y {1e3*ay:5.1f}mm @{fy:4.2f}Hz"
          f"  z {1e3*az:4.1f}mm @{fz:4.2f}Hz | CoM err y {1e3*ae:5.1f}mm @{fe:4.2f}Hz | (q2-q3)/2 {aq:5.1f}deg @{fq:4.2f}Hz")


def main():
    print("--- hardware 10-21 (spectra.txt) ---")
    for ln in open(HWTXT):
        if ln.startswith("HW"):
            print(ln.rstrip())
    print("--- Isaac ---")
    for p in sys.argv[1:]:
        z = np.load(p, allow_pickle=True)
        tag = os.path.basename(p)[:-4]
        m = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
        if "go_to_start:start" in m:
            report(f"{tag} hover before go_to_start", z, m["go_to_start:start"] - 8, m["go_to_start:start"])
        if "ready:end" in m:
            t1 = m.get("release:start", m.get("push:end", float(z["joints"][-1, 0])))
            if bool(z["aborted"]):
                t1 = min(t1, float(z["joints"][-1, 0]))
            report(f"{tag} world-held all", z, m["ready:end"], t1)
        if bool(z["aborted"]):
            print(f"   ABORTED: {str(z['reason'])[:120]}")


if __name__ == "__main__":
    main()
