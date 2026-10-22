#!/usr/bin/env python3
"""PX4's attitude estimate (vehicle_attitude, what the law uses) vs the mocap orientation on the
10-21 flight: the tilt difference (constant mount offset removed), its varying part and frequency,
and the horizontal force a thrust T along the wrong axis makes (T x the tilt error).
    /usr/bin/python3 att_err.py"""
import os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "npz")


def R_of(q):   # w, x, y, z
    w, x, y, z = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


def main():
    for tag, wins in (("170123", {"hover (CoM anchor)": (28.9, 36.9), "world-held + contact": (37.3, 46.6)}),
                      ("164338", {"hover 16 s": (39.5, 55.3)})):
        d = np.load(os.path.join(NPZ, f"b{tag}.npz"), allow_pickle=True)
        t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
        tm = d["mocap__recv"] - t0
        qm = np.column_stack([d[f"mocap__pose.orientation.{c}"] for c in "wxyz"])
        tv = d["vatt__recv"] - t0
        q = d["vatt__q"]                       # PX4: w x y z, NED/FRD
        # NED/FRD -> ENU/FLU: R_enu_flu = T R_ned_frd T, T = [[0,1,0],[1,0,0],[0,0,-1]] (both sides)
        T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
        F = np.diag([1.0, -1.0, -1.0])
        Rv = T @ R_of(q) @ F
        for name, (a, b) in wins.items():
            tt = np.arange(a, b, 0.02)
            iv = np.clip(np.searchsorted(tv, tt), 0, len(tv) - 1)
            im = np.clip(np.searchsorted(tm, tt), 0, len(tm) - 1)
            Rm = R_of(qm[im])
            zv, zm = Rv[iv][:, :, 2], Rm[:, :, 2]          # body z axes in the world
            dz = zv - zm                                     # small-angle tilt difference (x, y)
            dz = dz - dz.mean(axis=0)                        # the constant mount offset removed
            e = np.degrees(np.linalg.norm(dz[:, :2], axis=1))
            x = dz[:, 0] - np.polyval(np.polyfit(tt, dz[:, 0], 1), tt)
            P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
            f = np.fft.rfftfreq(len(x), 0.02)
            mm = (f > 0.05) & (f < 3)
            print(f"HW {tag} {name:22s} | tilt error (mean removed) rms {np.sqrt(np.mean(e ** 2)):.2f} deg, "
                  f"max {e.max():.2f} | x @{f[mm][np.argmax(P[mm])]:.2f} Hz | x 37.4 N -> "
                  f"{37.4 * np.sqrt(np.mean(np.sum(dz[:, :2] ** 2, axis=1))):.2f} N rms")


if __name__ == "__main__":
    main()
