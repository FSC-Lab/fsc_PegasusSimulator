#!/usr/bin/env python3
"""The drone's feedback quality per flight, mid-field vs at the table: the raw VRPN position jitter (> 8 Hz part of 2 s blocks after a cubic detrend, mm),
repeated (bit-identical) VRPN frames, and the velocity the law feeds back (odometry twist) minus EKF2's (rms, mm/s).
A wall can degrade the motion capture (occlusion, reflections) without any wind."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import ab
H = ab.H
FL = [("144602", "wb_hover_wander_history_20261103", "circle 10-28", 0), ("150821", "wb_hover_wander_history_20261103", "circle 10-28", 0),
      ("165603", "wb_hover_wander_history_20261103", "PnD 10-28", 0), ("180652", "wb_valve_turning_flight_20261028", "valve 10-28", 0),
      ("145526", "wb_valve_turning_flight_20261029", "valve 10-29", 0), ("150625", "wb_hover_wander_history_20261103", "fig-8 10-29", 0),
      ("182844", "wb_sidewall_ab_20261103", "circle 11-02", 1), ("192323", "wb_sidewall_ab_20261103", "PnD 11-02", 1),
      ("174943", "wb_valve_turning_flight_20261031", "valve 10-31", 1), ("173835", "wb_valve_turning_flight_20261101", "valve 11-01", 1),
      ("165100", "wb_valve_turning_flight_20261103", "valve 11-03", 1)]
for tag, d, lab, wall in FL:
    H._c.NPZ = os.path.join(ab.ARCH, d, "npz"); b = H._c.Bag(tag)
    md = b.edges("wbmode"); t_dir = [t for t, s in md if s == "DIRECT"][0]
    tv = b.t("vrpn"); P = np.stack([b.f("vrpn", f"pose.position.{k}") for k in "xyz"], 1)
    w = tv > t_dir + 2
    tv, P = tv[w], P[w]
    rep = np.mean(np.all(np.diff(P, axis=0) == 0, axis=1)) * 100
    # jitter: > 8 Hz content of the position, on a 120 Hz grid, in 2 s blocks (robust to gaps)
    T = np.arange(tv[0], tv[-1], 1 / 120.0); p = np.stack([np.interp(T, tv, P[:, k]) for k in range(3)], 1)
    site = np.where(p[:, 0] > 0.25, 1, 0)        # x > 0.25: the table side (box/valve/mailbox/pick work sites)
    out = {}
    for s_ in (0, 1):
        js = []
        for i in range(0, len(T) - 240, 240):
            if np.all(site[i:i + 240] == s_):
                tt = np.arange(240) / 120.0
                y = p[i:i + 240, :2] - np.stack([np.polyval(np.polyfit(tt, p[i:i + 240, k], 3), tt) for k in range(2)], 1)
                F = np.fft.rfft(y, axis=0); f = np.fft.rfftfreq(240, 1 / 120.0); F[f < 8] = 0
                js.append(np.fft.irfft(F, 240, axis=0).std(0))
        out[s_] = np.median(np.array(js), 0) * 1e3 if js else None
    to = b.t("odom"); vo = np.stack([b.f("odom", f"twist.twist.linear.{k}") for k in "xyz"], 1)
    tp = b.t("vodom_px"); vp = np.stack(b.f("vodom_px", "velocity")).astype(float)
    wo = to > t_dir + 2
    vpi = np.stack([np.interp(to[wo], tp, vp[:, 1]), np.interp(to[wo], tp, vp[:, 0])], 1)   # NED -> ENU x, y
    dv = vo[wo, :2] - vpi
    f = lambda v: f"{v[0]:4.2f}/{v[1]:4.2f}" if v is not None else "  --   "
    print(f"{lab:12s} {tag} {'wall' if wall else '    '} | VRPN jitter > 8 Hz x/y: mid-field {f(out[0])} mm, table side {f(out[1])} mm | repeated frames {rep:4.1f} % | "
          f"odom - EKF2 velocity rms {dv[:,0].std()*1e3:4.0f}/{dv[:,1].std()*1e3:4.0f} mm/s")
