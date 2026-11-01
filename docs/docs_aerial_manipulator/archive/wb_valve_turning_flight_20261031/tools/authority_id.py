#!/usr/bin/env python3
"""Do the rotors deliver the commanded roll / pitch torque? Closed-loop fit of the gyro's angular acceleration on the
torque the motor commands produce (both PX4-clock stamped: /fmu/in/actuator_motors and sensor_combined), band-passed
0.5-4 Hz so a slow external torque drops out, best lag 0-200 ms. Gain = 1 / effective inertia (1/(kg.m^2)).
Thrust per motor T = kf (w_idle + u (w_max - w_idle))^2 (hardware allocator kf; a battery-voltage scale is common to
roll and pitch, so compare the roll/pitch RATIO across flights). FRD, allocator geometry (all arms 0.2299 m):
roll = 0.2299 (T1 + T2 - T0 - T3), pitch = 0.2299 (T0 + T2 - T1 - T3).
    PYTHONNOUSERSITE=1 /usr/bin/python3 authority_id.py"""
import numpy as np
import vt_common as V

A = "/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/"
FL = [("wb_valve_turning_flight_20261028", "180652", "10-28 18:06"), ("wb_valve_turning_flight_20261028", "181833", "10-28 18:18"),
      ("wb_valve_turning_flight_20261029", "145526", "10-29 14:55"), ("wb_valve_turning_flight_20261031", "174943", "10-31 17:49"),
      ("wb_valve_turning_flight_20261031", "175725", "10-31 17:57"), ("wb_valve_turning_flight_20261031", "180423", "10-31 18:04"),
      ("wb_valve_turning_flight_20261031", "211808", "10-31 21:18")]
KF, W0, W1, L = 4.260431e-05, 64.06, 730.05, 0.22990663
FS = 250.0


def bp(x, lo=0.5, hi=4.0):
    n = len(x); f = np.fft.rfftfreq(n, 1 / FS); X = np.fft.rfft(x - x.mean())
    X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n)


def fit(tau, acc):
    best = None
    for lag in range(0, int(0.2 * FS)):
        a = acc[lag:]; t_ = tau[:len(tau) - lag] if lag else tau
        k = np.dot(t_, a) / np.dot(t_, t_); r = np.corrcoef(t_, a)[0, 1]
        if best is None or r > best[2]:
            best = (k, lag / FS * 1e3, r)
    return best


print("flight       window | roll gain  lag ms  r   | pitch gain lag ms  r   | roll/pitch | roll tau sd N.m")
for d, tag, lab in FL:
    V._c.NPZ = A + d + "/npz"
    b = V.Bag(tag)
    ed = b.edges("pl_pstatus")
    tg = [t for t, s in ed if s.startswith("FLYING go_to_start")][0]
    tr = [t for t, s in ed if s.startswith("FLYING ready")][0]
    off = np.median(np.asarray(b.f("sc", "timestamp"), float) * 1e-6 - b.t("sc"))
    ts = np.asarray(b.f("sc", "timestamp"), float) * 1e-6 - off
    G = np.array([np.asarray(g, float) for g in b.f("sc", "gyro_rad")])
    tm = np.asarray(b.f("act_motors", "timestamp"), float) * 1e-6 - off
    U = np.array([np.asarray(u, float)[:4] for u in b.f("act_motors", "control")])
    T = KF * (W0 + np.clip(U, 0, 1) * (W1 - W0)) ** 2
    roll = L * (T[:, 1] + T[:, 2] - T[:, 0] - T[:, 3]); pitch = L * (T[:, 0] + T[:, 2] - T[:, 1] - T[:, 3])
    for wn, (a_, b_) in (("hover", (tg - 6, tg)), ("GTS+hold", (tg, tr))):
        tu = np.arange(a_, b_, 1 / FS)
        tr_ = bp(np.interp(tu, tm, roll)); tp_ = bp(np.interp(tu, tm, pitch))
        p = np.interp(tu, ts, G[:, 0]); q = np.interp(tu, ts, G[:, 1])
        pd = bp(np.gradient(p, 1 / FS)); qd = bp(np.gradient(q, 1 / FS))
        kr = fit(tr_, pd); kp = fit(tp_, qd)
        print(f"{lab:12s} {wn:8s} | {kr[0]:7.2f}  {kr[1]:5.0f}  {kr[2]:.2f} | {kp[0]:7.2f}  {kp[1]:5.0f}  {kp[2]:.2f} | {kr[0]/kp[0]:6.2f}     | {tr_.std():.3f}")
