#!/usr/bin/env python3
"""How big and how slow is the force that wanders the vehicle? The law's filtered translational
disturbance estimate d_hat_t (wb_control_debug [31..33], world N) and its CoM error, hardware 10-21
vs Isaac runs: detrended rms (the varying part; the constant body-fixed bias removed), its dominant
frequency, and the CoM error rms. Hover window + the room-held window.
    /usr/bin/python3 dist_level.py [isaac.npz ...]"""
import os
import sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "npz")


def stat(t, X, t0, t1, fs=50.0):
    tt = np.arange(t0, t1, 1 / fs)
    Y = np.column_stack([np.interp(tt, t, X[:, i]) for i in range(X.shape[1])])
    Y = Y - np.array([np.polyval(np.polyfit(tt, Y[:, i], 1), tt) for i in range(Y.shape[1])]).T
    P = np.abs(np.fft.rfft(Y[:, :2] * np.hanning(len(tt))[:, None], axis=0)) ** 2
    f = np.fft.rfftfreq(len(tt), 1 / fs)
    m = (f > 0.05) & (f < 3)
    return np.sqrt(np.mean(np.sum(Y[:, :2] ** 2, axis=1))), f[m][np.argmax(P[m].sum(axis=1))]


def report(name, t, dbg, wins):
    for wn, (a, b) in wins.items():
        dh, fd = stat(t, dbg[:, 31:34], a, b)
        ex, fe = stat(t, dbg[:, 48:51] - dbg[:, 45:48], a, b)
        print(f"{name:12s} {wn:22s} {b - a:5.1f}s | d_hat_t xy {dh:5.2f} N @{fd:4.2f} Hz | CoM err xy {1e3 * ex:5.1f} mm @{fe:4.2f} Hz")


def main():
    for tag, wins in (("170123", {"hover (CoM anchor)": (28.9, 36.9), "world-held + contact": (37.3, 46.6)}),
                      ("164338", {"hover 16 s": (39.5, 55.3)})):
        d = np.load(os.path.join(NPZ, f"b{tag}.npz"), allow_pickle=True)
        t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
        report("HW " + tag, d["wb__recv"] - t0, d["wb__data"].astype(float), wins)
    for p in sys.argv[1:]:
        z = np.load(p, allow_pickle=True)
        m = {str(k): float(v) for k, v in zip(z["marks_name"], z["marks_t"])}
        D = z["dbg"].astype(float)
        wins = {"hover before go_to_start": (m["go_to_start:start"] - 8, m["go_to_start:start"])}
        if "push:end" in m:
            wins["world-held + contact"] = (m["ready:end"], m["push:end"])
        report(os.path.basename(p)[:-4], D[:, 0], D[:, 2:], wins)


if __name__ == "__main__":
    main()
