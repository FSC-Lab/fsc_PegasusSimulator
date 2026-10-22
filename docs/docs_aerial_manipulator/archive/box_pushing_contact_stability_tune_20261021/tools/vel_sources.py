#!/usr/bin/env python3
"""Hardware velocity sources on the 10-21 box-push flight: the law's raw-mocap odometry velocity
(the processor's finite difference) vs EKF2's velocity (/fmu/out/vehicle_odometry), each against a
zero-phase smoothed derivative of the mocap position. Error rms below 2 Hz (the band of the wander
and the 0.55 Hz arm swing) and broadband, and EKF2's lag (cross-correlation).
    PYTHONNOUSERSITE=1 /usr/bin/python3 vel_sources.py"""
import os
import numpy as np
from scipy.signal import butter, filtfilt
HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "npz")
FS = 100.0


def load(tag):
    d = np.load(os.path.join(NPZ, f"b{tag}.npz"), allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
    def xyz(pref, field):
        a = d[f"{pref}__{field}"]
        return a if a.ndim == 2 else np.column_stack([d[f"{pref}__{field}.{c}"] for c in "xyz"])
    mp_t = d["mocap__recv"] - t0
    mp = np.column_stack([d[f"mocap__pose.position.{c}"] for c in "xyz"])
    od_t = d["odom__recv"] - t0
    od_v = np.column_stack([d[f"odom__twist.twist.linear.{c}"] for c in "xyz"])
    ek_t = d["vodom_px__recv"] - t0
    v = d["vodom_px__velocity"]
    ek_v = np.column_stack([v[:, 1], v[:, 0], -v[:, 2]])      # NED -> ENU
    return mp_t, mp, od_t, od_v, ek_t, ek_v


def lp(x, fc, fs=FS):
    b, a = butter(2, fc / (fs / 2))
    return filtfilt(b, a, x, axis=0)


def main():
    for tag, wins in (("170123", {"hover (CoM anchor)": (28.9, 36.9), "world-held + contact": (37.3, 46.6)}),
                      ("164338", {"hover 16 s": (39.5, 55.3)})):
        mp_t, mp, od_t, od_v, ek_t, ek_v = load(tag)
        for name, (a, b) in wins.items():
            t = np.arange(a - 2.0, b + 2.0, 1 / FS)
            P = np.column_stack([np.interp(t, mp_t, mp[:, i]) for i in range(3)])
            Vref = np.gradient(lp(P, 4.0), 1 / FS, axis=0)
            Vod = np.column_stack([np.interp(t, od_t, od_v[:, i]) for i in range(3)])
            Vek = np.column_stack([np.interp(t, ek_t, ek_v[:, i]) for i in range(3)])
            m = (t >= a) & (t <= b)
            out = [f"{tag} {name:22s}"]
            for lab, V in (("raw mocap", Vod), ("EKF2", Vek)):
                e = V - Vref
                e_lo = lp(e, 2.0)
                out.append(f"{lab}: <2 Hz xy {1e3*np.sqrt(np.mean(e_lo[m, :2]**2)):5.1f} z {1e3*np.sqrt(np.mean(e_lo[m, 2]**2)):5.1f} mm/s"
                           f", all xy {1e3*np.sqrt(np.mean(e[m, :2]**2)):5.1f} mm/s")
            # EKF2 lag against the reference (horizontal), by cross-correlation within +-100 ms
            lags = np.arange(-10, 11)
            r = [np.mean(np.sum(np.roll(Vek[m, :2], -k, axis=0) * Vref[m, :2], axis=1)) for k in lags]
            out.append(f"EKF2 lag {lags[int(np.argmax(r))] * 10:+d} ms")
            print(" | ".join(out))


if __name__ == "__main__":
    main()
