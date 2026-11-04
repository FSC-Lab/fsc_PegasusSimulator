#!/usr/bin/env python3
"""Per circle lap (the planner's EXECUTING T >= 10 s runs; the 11-02 two-lap run split at its middle): the law's slow
moment estimate M_hat (0.05-1 Hz), its translational estimate F_hat (0.1-1 Hz), the IMU external force (0.1-1 Hz),
the law's attitude error e_R (roll/pitch, rms), and the battery."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import ab
H = ab.H
for tag, d, lab in (("144602", "wb_hover_wander_history_20261103", "10-28"), ("150821", "wb_hover_wander_history_20261103", "10-28"),
                    ("182844", "wb_sidewall_ab_20261103", "11-02")):
    H._c.NPZ = os.path.join(ab.ARCH, d, "npz"); b = H._c.Bag(tag)
    md = b.edges("wbmode"); t_dir = [t for t, s in md if s == "DIRECT"][0]
    T = np.arange(t_dir + 2.0, b.t("wbmode")[-1] - 0.5, ab.DT)
    tw, D = H.wb(b)
    Mh = np.stack([ab.bp(np.interp(T, tw, D[:, k]), 0.05, 1.0) for k in (34, 35)], 1)
    Fh = np.stack([ab.bp(np.interp(T, tw, D[:, k]), 0.1, 1.0) for k in (31, 32)], 1)
    eR = np.stack([np.interp(T, tw, D[:, k]) for k in (28, 29)], 1)
    ts_ = b.t("sc"); acc = np.stack(b.f("sc", "accelerometer_m_s2")).astype(float)
    Fi = np.stack([ab.bp(np.interp(T, ts_, acc[:, k]) * ab.M, 0.1, 1.0) for k in range(2)], 1)
    st = b.edges("pl_status")
    runs = [(t, st[i + 1][0]) for i, (t, v) in enumerate(st) if v.startswith("EXECUTING T=") and float(v.split("=")[1].rstrip("s")) >= 10 and i + 1 < len(st)]
    laps = []
    for c0, c1 in runs:
        if c1 - c0 > 40: laps += [(c0, (c0 + c1) / 2), ((c0 + c1) / 2, c1)]
        else: laps.append((c0, c1))
    for k, (c0, c1) in enumerate(laps):
        m = (T >= c0) & (T <= c1)
        r = lambda s_: np.sqrt((s_[m] ** 2).mean(0))
        print(f"{lab} {tag} lap {k+1}: {c1-c0:5.1f} s | M_hat rms x/y {r(Mh)[0]*1e3:4.0f}/{r(Mh)[1]*1e3:4.0f} mN.m | F_hat {r(Fh)[0]:4.2f}/{r(Fh)[1]:4.2f} N | "
              f"F_imu {r(Fi)[0]:4.2f}/{r(Fi)[1]:4.2f} N | e_R rms x/y {np.degrees(eR[m].std(0))[0]:4.2f}/{np.degrees(eR[m].std(0))[1]:4.2f} deg")
