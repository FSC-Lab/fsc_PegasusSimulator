"""External roll / pitch moment the airframe felt, back-solved from the rigid-body equation, in chosen hover windows.

    tau_ext = I * d(omega)/dt  -  tau_rotors(lagged)            (FLU body frame, gyroscopic term negligible at hover)

  omega      : sensor_combined gyro (FRD -> FLU), low-passed at 8 Hz before differentiating
  tau_rotors : motor commands -> omega_cmd (alloc map) -> first-order rotor lag, exact ZOH, lambda = 10.0265 1/s
               (MN4010 bench) -> f = kf omega^2 -> moments about the body origin (rotor_wrench in yaw_history.py)
  I          : coupled rotational inertia at the folded home pose, actual roll/pitch = (0.112, 0.132) kg m^2
Both topics carry PX4-synced timestamps from the same clock (checked: sub-ms jitter), so they align exactly.
The MEAN of tau_ext is not meaningful (CoM offset, kf and inertia errors all land there); its FLUCTUATION in the
0.3-3 Hz band is: it is the moment the controller had to fight, separated from the vehicle's own response.
A wrong model gain leaks part of the commanded moment into the residual, so compare windows, not absolutes.
usage: moment_residual.py  (windows hard-coded below; writes ../analysis/moment_residual.json)
"""
import os, json
import numpy as np
from scipy.signal import butter, filtfilt, welch
from pd_common import load, NPZ, HIST, OUT, px4_rpy
import yaw_history as YH

LAM = 10.0265
I_RP = np.array([0.112, 0.132])


def residual(d, t0):
    ts = d["sc__timestamp"] * 1e-6
    g = d["sc__gyro_rad"] * np.array([1.0, -1.0, -1.0])            # FRD -> FLU
    fs = 1.0 / np.median(np.diff(ts))
    b, a = butter(2, 8.0 / (fs / 2))
    gf = filtfilt(b, a, g, axis=0)
    wd = np.gradient(gf, ts, axis=0)
    ta = d["act_motors__timestamp"] * 1e-6
    U = d["act_motors__control"][:, :4]
    ok = np.all(np.isfinite(U), 1) & (U.min(1) > 0.02)
    ta, U = ta[ok], U[ok]
    om_c = YH.W0 + np.clip(U, 0, 1) * (YH.W1 - YH.W0)
    om = np.empty_like(om_c); om[0] = om_c[0]
    for k in range(1, len(om_c)):
        dt = ta[k] - ta[k - 1]
        if dt > 0.05:                       # stream gap (SAFETY stretch): restart at the command
            om[k] = om_c[k]; continue
        e = np.exp(-LAM * dt)
        om[k] = om_c[k - 1] + (om[k - 1] - om_c[k - 1]) * e
    f = YH.KF * om ** 2
    tx = (f * YH.Y).sum(1); ty = -(f * YH.X).sum(1)
    inside = (ts >= ta[0]) & (ts <= ta[-1])
    # mask IMU samples that fall into a motor-stream gap
    idx = np.clip(np.searchsorted(ta, ts), 1, len(ta) - 1)
    gap = (ta[idx] - ta[idx - 1]) > 0.05
    m = inside & ~gap
    t = ts[m]
    tau_rot = np.column_stack([np.interp(t, ta, tx), np.interp(t, ta, ty)])
    tau_in = wd[m, :2] * I_RP
    res = tau_in - tau_rot
    rec0 = d["sc__recv"][0] - d["sc__timestamp"][0] * 1e-6        # PX4 stamp -> receive clock, then -> bag time
    return t + rec0 - t0, res, tau_rot, tau_in, fs


WINDOWS = [
    ("d1", "PD-1 pick hover 1.15 m (pick hat)", 46.0, 49.2),
    ("d1", "PD-1 Drop Start 0.70 m", 70.5, 76.2),
    ("d1", "PD-1 OVER FULL BIN 1.35 m", 86.0, 92.2),
    ("d1", "PD-1 over full bin 1.65 m (after abort)", 103.0, 115.0),
    ("d1", "PD-1 open hover (0,0) 1.68 m", 128.6, 131.7),
    ("flight_wb_l1_4d_pick_and_place_20261009_172557", "PP-1 10-09 same point 1.32 m (hat)", 124.0, 132.0),
    ("flight_wb_l1_4d_pick_and_place_20261009_172557", "PP-1 10-09 same point 1.31 m #2", 140.0, 146.0),
    ("flight_wb_l1_4d_pick_and_place_20261009_180228", "PP-2 10-09 same point 1.31 m (hat, basket)", 101.0, 116.0),
    ("flight_wb_l1_4d_pick_and_place_20261010_190719", "PP-4 10-10 same point 1.33 m (hat, basket)", 59.0, 70.0),
    ("d2", "PD-2 hover 1.0 m (start)", 21.0, 27.0),
]


def main():
    cache, out = {}, []
    print(f"{'window':46s} {'res roll':>9s} {'res pitch':>9s}   {'rotor roll':>10s} {'rotor pitch':>11s}   {'I*wdot p':>8s}  {'pitch att':>9s}  peak f (res pitch)")
    for run, lab, a, b in WINDOWS:
        if run not in cache:
            p = os.path.join(NPZ, f"{run}.npz") if run in ("d1", "d2") else os.path.join(HIST, f"{run}.npz")
            d, t0 = load(p)
            cache[run] = (residual(d, t0), px4_rpy(d, t0))
        (t, res, rot, tin, fs), (tp, ro, pi_, ya) = cache[run]
        bb, aa = butter(2, [0.3 / (fs / 2), 3.0 / (fs / 2)], "band")
        rf = filtfilt(bb, aa, res, axis=0); of = filtfilt(bb, aa, rot, axis=0); inf = filtfilt(bb, aa, tin, axis=0)
        s = (t >= a) & (t < b); sp = (tp >= a) & (tp < b)
        fr, P = welch(rf[s, 1], fs=fs, nperseg=min(s.sum(), 2048))
        band = (fr > 0.2) & (fr < 3.0)
        row = dict(window=lab, t0=a, t1=b, res_roll=float(rf[s, 0].std()), res_pitch=float(rf[s, 1].std()),
                   rot_roll=float(of[s, 0].std()), rot_pitch=float(of[s, 1].std()), inertial_pitch=float(inf[s, 1].std()),
                   pitch_std_deg=float(np.degrees(pi_[sp].std())), f_peak=float(fr[band][np.argmax(P[band])]))
        out.append(row)
        print(f"{lab:46s} {row['res_roll']:9.3f} {row['res_pitch']:9.3f}   {row['rot_roll']:10.3f} {row['rot_pitch']:11.3f}   "
              f"{row['inertial_pitch']:8.3f}  {row['pitch_std_deg']:9.2f}  {row['f_peak']:.2f} Hz")
    json.dump(out, open(os.path.join(OUT, "moment_residual.json"), "w"), indent=1)


if __name__ == "__main__":
    main()


def robust(windows=WINDOWS, mass=3.746):
    """Worst-case-for-the-claim check: per window, fit the rotor-moment gain kappa (and a +-60 ms shift) that MINIMISES the
    0.3-3 Hz residual, so whatever is left cannot be a model-gain or lag artefact. Also the vertical aerodynamic force:
    the body-z accelerometer measures (thrust + aero)/m directly, so F_aero,z = m a_z - T_model(lagged)."""
    cache, out = {}, []
    print(f"{'window':46s} {'min res pitch':>13s} {'(kappa, shift ms)':>18s} {'min res roll':>12s} {'Fz aero 0.3-3Hz':>16s} {'Fz 3-8 Hz':>9s}")
    for run, lab, a, b in windows:
        if run not in cache:
            p = os.path.join(NPZ, f"{run}.npz") if run in ("d1", "d2") else os.path.join(HIST, f"{run}.npz")
            d, t0 = load(p)
            ts = d["sc__timestamp"] * 1e-6
            acc = d["sc__accelerometer_m_s2"] * np.array([1.0, -1.0, -1.0])     # FRD -> FLU specific force
            ta = d["act_motors__timestamp"] * 1e-6; U = d["act_motors__control"][:, :4]
            ok = np.all(np.isfinite(U), 1) & (U.min(1) > 0.02); ta, U = ta[ok], U[ok]
            om_c = YH.W0 + np.clip(U, 0, 1) * (YH.W1 - YH.W0); om = np.empty_like(om_c); om[0] = om_c[0]
            for k in range(1, len(om_c)):
                dt = ta[k] - ta[k - 1]
                om[k] = om_c[k] if dt > 0.05 else om_c[k - 1] + (om[k - 1] - om_c[k - 1]) * np.exp(-LAM * dt)
            T = (YH.KF * om ** 2).sum(1)
            rec0 = d["sc__recv"][0] - d["sc__timestamp"][0] * 1e-6
            cache[run] = (residual(d, t0), ts + rec0 - t0, acc, ta + rec0 - t0, T)
        (t, res, rot, tin, fs), tsb, acc, tab, T = cache[run]
        bb, aa = butter(2, [0.3 / (fs / 2), 3.0 / (fs / 2)], "band")
        s = (t >= a) & (t < b)
        best = {}
        for ax in (0, 1):
            y = filtfilt(bb, aa, tin[:, ax]); r = filtfilt(bb, aa, rot[:, ax])
            bestv = (np.inf, 1, 0)
            for sh in range(-24, 25, 2):                         # samples at ~400 Hz -> +-60 ms
                rs = np.roll(r, sh)
                k = (y[s] @ rs[s]) / (rs[s] @ rs[s])
                v = (y[s] - k * rs[s]).std()
                if v < bestv[0]:
                    bestv = (v, k, sh / fs * 1e3)
            best[ax] = bestv
        sb = (tsb >= a - 5) & (tsb < b + 5)
        Fz = mass * acc[sb, 2] - np.interp(tsb[sb], tab, T)
        b2, a2 = butter(2, [3.0 / (fs / 2), 8.0 / (fs / 2)], "band")
        fz1 = filtfilt(bb, aa, Fz); fz2 = filtfilt(b2, a2, Fz)
        ss = (tsb[sb] >= a) & (tsb[sb] < b)
        row = dict(window=lab, minres_pitch=float(best[1][0]), kappa_pitch=float(best[1][1]), shift_ms=float(best[1][2]),
                   minres_roll=float(best[0][0]), Fz_aero_rms=float(fz1[ss].std()), Fz_aero_3_8=float(fz2[ss].std()))
        out.append(row)
        print(f"{lab:46s} {row['minres_pitch']:13.3f} ({row['kappa_pitch']:5.2f}, {row['shift_ms']:+4.0f}) {row['minres_roll']:12.3f}"
              f" {row['Fz_aero_rms']:16.3f} {row['Fz_aero_3_8']:9.3f}")
    json.dump(out, open(os.path.join(OUT, "moment_residual_robust.json"), "w"), indent=1)


if __name__ == "__main__" and os.environ.get("ROBUST"):
    robust()
