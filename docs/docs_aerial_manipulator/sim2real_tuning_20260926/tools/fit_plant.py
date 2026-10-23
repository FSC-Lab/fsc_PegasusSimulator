#!/usr/bin/env python3
"""Identify the AM-T650 plant from the 0918 / 0921 / 0924 whole-body flights.

Everything here is CONTROLLER-INDEPENDENT or uses the law's own published
quantities, so the numbers are plant properties the simulator's mirror
config (..._whole_body_l1_4d_..._sim.yaml) can carry:

  1. HOVER THRUST BALANCE   kf_true = kf_believed * m g / u1   (holds only)
                            + its drift over the flight (battery sag)
  2. SAME-COMMAND TEST      recorded motor commands through the SIM plant
                            (kf, c, lambda, coupled inertia at home) vs the
                            measured gyro/accel response: per-axis gain and
                            the effective actuation delay beyond the lag
  3. ARM VELOCITY FEEDBACK  lag + quantisation of joint_states.velocity
                            (the Present Velocity the law consumes)
  4. OBSERVER RESIDUALS     filtered d_hat at hover -> standing force /
                            moment the plant carries and the model does not

Run with the extractor's python (numpy 2.x user-site):
    /usr/bin/python3 fit_plant.py <data dir> [flights...]
Writes <data dir>/../analysis/fit_plant.json and .txt.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, "/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
sys.path.insert(0, "/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/"
                   "fsc_aerial_manipulation/rotorcraft")
import t650_params as P                                                  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT  # noqa

G = 9.81
MASS = 3.746170                     # the one mass the stack flies (yaml)
KF_BEL = 4.260431e-05               # alloc_thrust_coeff flown 0918..0924
KM_BEL = 0.018164                   # alloc_rotor*_km flown
ARM_LEN = 0.22990663                # alloc_rotor*_px/py
Q_HOME = np.array([0.0, np.radians(40.0), np.radians(40.0), 0.0])
FLIGHTS = {"f18": "0918 mission (raw mocap 60 Hz)",
           "c3": "0921 #3 go-to-start (raw mocap 120 Hz)",
           "a1": "0924 F1 circle (fused)", "a2": "0924 F2 circle (fused)",
           "a3": "0924 F3 circle 12 s (fused)", "a4": "0924 F4 circle 6 s (fused)"}
# debug layout (wb_l1_metrics.py)
D_MODE, D_U1 = 0, 17
D_Q, D_QD, D_TAU = slice(5, 9), slice(9, 13), slice(13, 17)
D_EY, D_ER = slice(24, 28), slice(28, 31)
D_DHAT_T, D_DHAT_R, D_DHAT_Q = slice(31, 34), slice(34, 37), slice(37, 41)
D_MOT = slice(41, 45)
D_XCD, D_XC = slice(45, 48), slice(48, 51)

# PX4 FRD rotor geometry = the allocator's table (plant_replay.py convention)
ROTOR_FRD = np.array([[+ARM_LEN, +ARM_LEN], [-ARM_LEN, -ARM_LEN],
                      [+ARM_LEN, -ARM_LEN], [-ARM_LEN, +ARM_LEN]])
KM_SIGN = np.array([+1.0, +1.0, -1.0, -1.0])


def lp(x, fs, fc):
    """Zero-phase windowed-sinc low-pass (no scipy: the apt scipy is numpy-1.x)."""
    numtaps = int(4 * fs / fc) | 1
    n = np.arange(numtaps) - (numtaps - 1) / 2.0
    h = np.sinc(2.0 * fc / fs * n) * np.hamming(numtaps)
    h /= h.sum()
    pad = numtaps // 2

    def one(v):
        vp = np.concatenate([np.full(pad, v[0]), v, np.full(pad, v[-1])])
        return np.convolve(vp, h, mode="valid")
    return one(x) if x.ndim == 1 else np.stack([one(x[:, j]) for j in range(x.shape[1])], 1)


def lag_omega(u, dt, lam, w_idle, w_max):
    wc = np.clip(u, 0.0, 1.0) * (w_max - w_idle) + w_idle
    w = np.empty_like(wc)
    w[0] = wc[0]
    for i in range(1, len(wc)):
        w[i] = wc[i] + (w[i - 1] - wc[i]) * np.exp(-lam * dt[i - 1])
    return w


def plant_inertia_home():
    """Coupled rotational inertia about the body at the folded home, ACTUAL
    (x fwd) frame, from the whole-body model the C++ law carries."""
    p = TP.make_params_t650()
    n = p["n"]
    X = np.concatenate([np.zeros(3), np.eye(3).flatten(order="F"), Q_HOME[:n],
                        np.zeros(3), np.zeros(3), np.zeros(n)])
    M = CT.dynamics(X, p)["M"]
    Mr = M[3:6, 3:6]                 # MODEL frame: model +y = actual +x
    # actual x = model y, actual y = -model x, actual z = model z
    R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])
    Mr_act = R @ Mr @ R.T
    return np.diag(Mr_act), Mr_act, p


def windows(d, t0):
    """DIRECT span, planner HOLD windows (excluding 4 s after entry/legs)."""
    tm = d["wbmode__recv"] - t0
    mv = [str(x) for x in d["wbmode__data"]]
    ti = [tm[i] for i in range(len(mv)) if mv[i] == "DIRECT"]
    if not ti:
        return None, []
    t_dir = (ti[0], tm[[i for i in range(len(mv)) if mv[i] == "DIRECT"][-1]])
    # DIRECT ends at the first SAFETY after entry
    after = [tm[i] for i in range(len(mv)) if mv[i] == "SAFETY" and tm[i] > ti[0]]
    t_dir = (ti[0], after[0] if after else tm[-1])
    ts = d["pl_status__recv"] - t0
    sv = [str(x) for x in d["pl_status__data"]]
    holds = []
    cur = None
    for i, s in enumerate(sv):
        if s.startswith("HOLD") and cur is None:
            cur = ts[i]
        elif not s.startswith("HOLD") and cur is not None:
            holds.append((cur, ts[i]))
            cur = None
    if cur is not None:
        holds.append((cur, t_dir[1]))
    holds = [(max(a, t_dir[0]) + 4.0, min(b, t_dir[1]) - 0.5) for a, b in holds]
    holds = [(a, b) for a, b in holds if b - a > 3.0]
    return t_dir, holds


def analyse(nm, path, Ir, Mr):
    d = np.load(path, allow_pickle=True)
    t0 = d["wb__recv"][0]
    t = d["wb__recv"] - t0
    D = d["wb__data"]
    t_dir, holds = windows(d, t0)
    out = {"flight": FLIGHTS.get(nm, nm), "direct": [float(t_dir[0]), float(t_dir[1])],
           "holds": [[float(a), float(b)] for a, b in holds]}
    direct = (t >= t_dir[0]) & (t <= t_dir[1]) & (D[:, D_MODE] > 0.5)

    # ---- 1. hover thrust balance ------------------------------------------
    u1 = D[:, D_U1]
    dz = D[:, 31 + 2]
    hb = []
    for a, b in holds:
        m = (t >= a) & (t <= b) & direct
        if m.sum() < 100:
            continue
        u1m = float(u1[m].mean())
        hb.append({"t": [a, b], "u1": u1m, "dhat_z": float(dz[m].mean()),
                   "motors": [float(v) for v in D[m][:, D_MOT].mean(0)],
                   "kf_true": KF_BEL * MASS * G / u1m,
                   "kf_scale_vs_sim": KF_BEL * MASS * G / u1m / P.ROTOR_CONSTANT,
                   "dhat_t": [float(v) for v in D[m][:, D_DHAT_T].mean(0)],
                   "dhat_r": [float(v) for v in D[m][:, D_DHAT_R].mean(0)],
                   "dhat_q": [float(v) for v in D[m][:, D_DHAT_Q].mean(0)],
                   "eR": [float(v) for v in D[m][:, D_ER].mean(0)],
                   "batt_v": float(np.interp(0.5 * (a + b), d["batt__recv"] - t0, d["batt__voltage_v"]))})
    out["hover_balance"] = hb
    # drift over the whole DIRECT window: kf_true(t) from a 5 s low-pass of u1
    if direct.sum() > 2000:
        td = t[direct]
        fs = 1.0 / np.median(np.diff(td))
        u1f = lp(u1[direct], fs, 0.2)
        kf_t = KF_BEL * MASS * G / u1f
        A = np.column_stack([np.ones_like(td), (td - td[0]) / 60.0])
        c, *_ = np.linalg.lstsq(A, kf_t, rcond=None)
        out["kf_drift"] = {"kf_at_entry": float(c[0]), "per_min": float(c[1]),
                           "frac_per_min": float(c[1] / c[0]),
                           "span_s": float(td[-1] - td[0]),
                           "batt_v_start_end": [float(np.interp(td[0], d["batt__recv"] - t0, d["batt__voltage_v"])),
                                                float(np.interp(td[-1], d["batt__recv"] - t0, d["batt__voltage_v"]))]}

    # ---- 2. same-command plant test ---------------------------------------
    t_sc = d["sc__timestamp"].astype(np.float64) * 1e-6
    t_mot = d["act_motors__timestamp"].astype(np.float64) * 1e-6
    u_all = d["act_motors__control"][:, :4].astype(np.float64)
    good = ~np.isnan(u_all).any(1)
    t_mot, u_all = t_mot[good], u_all[good]
    # the DIRECT window on the PX4 clock: actuator_motors is only non-NaN in
    # DIRECT, so its own span is the window
    lo, hi = t_mot[0] + 2.0, t_mot[-1] - 0.5
    m = (t_sc >= lo) & (t_sc <= hi)
    ts = t_sc[m]
    gyro = d["sc__gyro_rad"][m].astype(np.float64)
    accel = d["sc__accelerometer_m_s2"][m].astype(np.float64)
    idx = np.clip(np.searchsorted(t_mot, ts, side="right") - 1, 0, len(u_all) - 1)
    u = u_all[idx]
    dt = np.diff(ts)
    fs = 1.0 / np.median(dt)
    res = {"window_s": float(hi - lo), "fs_hz": float(fs), "inertia_actual_diag": [float(v) for v in Ir]}

    def predict(lam, kf, c):
        w = np.stack([lag_omega(u[:, j], dt, lam, P.ZERO_POSITION_ARMED, P.MAX_ROTOR_VEL)
                      for j in range(4)], 1)
        f = kf * w ** 2
        tau = np.stack([-(f * ROTOR_FRD[:, 1]).sum(1), +(f * ROTOR_FRD[:, 0]).sum(1),
                        (c * w ** 2 * KM_SIGN).sum(1)], 1)
        return tau, tau / Ir, -f.sum(1) / MASS

    fc, fhp = 8.0, 0.3
    alpha_meas = lp(np.gradient(gyro, ts, axis=0), fs, fc)
    az_meas = lp(accel[:, 2], fs, fc)
    hp = lambda x: x - lp(x, fs, fhp)  # noqa: E731
    am = hp(alpha_meas)
    tau_p, alpha_p, az_p = predict(P.ROTOR_LAMBDA, P.ROTOR_CONSTANT, P.ROLLING_MOMENT_COEFFICIENT)
    ap = hp(lp(alpha_p, fs, fc))
    azp = lp(az_p, fs, fc)
    names = ["roll", "pitch", "yaw"]
    for j, n_ in enumerate(names):
        mm, pp = am[:, j], ap[:, j]
        res[n_] = {"gain": float((pp @ mm) / (pp @ pp)), "corr": float(np.corrcoef(mm, pp)[0, 1]),
                   "rms_meas": float(np.sqrt((mm ** 2).mean())), "rms_pred": float(np.sqrt((pp ** 2).mean()))}
    # delay scan: shift the prediction later by k samples
    scan = {}
    for j, n_ in enumerate(names):
        best = (0, -2.0)
        row = []
        for k in range(0, int(0.16 * fs) + 1, 2):
            pp = ap[:-k, j] if k else ap[:, j]
            mm = am[k:, j]
            cc = float(np.corrcoef(mm, pp)[0, 1])
            row.append((k / fs, cc))
            if cc > best[1]:
                best = (k / fs, cc)
        scan[n_] = {"best_delay_s": best[0], "corr_at_best": best[1],
                    "gain_at_best": float(((ap[:-int(round(best[0] * fs)) or None, j]) @ am[int(round(best[0] * fs)):, j])
                                          / ((ap[:-int(round(best[0] * fs)) or None, j]) @ (ap[:-int(round(best[0] * fs)) or None, j])))
                    if best[0] > 0 else res[n_]["gain"]}
    res["delay_scan"] = scan
    # thrust: DC (specific thrust) and dynamic
    res["thrust"] = {"gain_dc": float(az_meas.mean() / azp.mean()),
                     "gain_ac": float((hp(azp) @ hp(az_meas)) / (hp(azp) @ hp(azp))),
                     "corr_ac": float(np.corrcoef(hp(azp), hp(az_meas))[0, 1]),
                     "kf_dc": float(P.ROTOR_CONSTANT * az_meas.mean() / azp.mean())}
    # bench-c yaw and the c that would match
    _, alpha_pb, _ = predict(P.ROTOR_LAMBDA, P.ROTOR_CONSTANT, P.BENCH_ROLLING_MOMENT_COEFFICIENT)
    apb = hp(lp(alpha_pb, fs, fc))[:, 2]
    res["yaw_gain_bench_c"] = float((apb @ am[:, 2]) / (apb @ apb))
    res["standing_trim_nm"] = [float(v) for v in lp(tau_p, fs, fc).mean(0)]
    out["same_command"] = res

    # ---- 3. arm velocity feedback -----------------------------------------
    names_js = list(d["js__name"][0])
    order = [names_js.index(f"joint{k}") for k in (1, 2, 3, 4)]
    tj = d["js__recv"] - t0
    q = d["js__position"][:, order]
    qd = d["js__velocity"][:, order]
    mj = (tj >= t_dir[0]) & (tj <= t_dir[1])
    tj, q, qd = tj[mj], q[mj], qd[mj]
    fsj = 1.0 / np.median(np.diff(tj))
    arm = {"rate_hz": float(fsj)}
    quant = []
    for j in range(4):
        dv = np.abs(np.diff(qd[:, j]))
        dv = dv[dv > 1e-9]
        quant.append(float(np.min(dv)) if dv.size else 0.0)
    arm["velocity_quantum_rad_s"] = quant
    # lag of reported velocity behind the FD of the reported position (j2, j3)
    lags = {}
    for j in (1, 2):
        fd = lp(np.gradient(q[:, j], tj), fsj, 6.0)
        v = lp(qd[:, j], fsj, 6.0)
        if np.std(v) < 1e-3:
            continue
        best = (0.0, -2.0)
        for k in range(0, int(0.2 * fsj)):
            cc = float(np.corrcoef(fd[:len(fd) - k] if k else fd, v[k:])[0, 1])
            if cc > best[1]:
                best = (k / fsj, cc)
        lags[f"j{j+1}"] = {"lag_s": best[0], "corr": best[1], "ratio_std": float(np.std(v) / np.std(fd))}
    arm["velocity_lag"] = lags
    out["arm_velocity_feedback"] = arm

    # ---- 4. feedback noise in holds ---------------------------------------
    to = d["odom__recv"] - t0
    ov = np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
    op = np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"])
    fb = []
    for a, b in holds:
        m = (to >= a) & (to <= b)
        if m.sum() > 50:
            fb.append({"vel_std_cm_s": [float(v) for v in ov[m].std(0) * 100],
                       "pos_std_mm": [float(v) for v in op[m].std(0) * 1000]})
    out["feedback_holds"] = fb
    out["odom_rate_hz"] = float(1.0 / np.median(np.diff(to)))
    return out


def main():
    ddir = sys.argv[1]
    names = sys.argv[2:] or ["f18", "c3", "a1", "a2", "a3", "a4"]
    Ir, Mr, p = plant_inertia_home()
    print(f"sim plant coupled inertia at home, actual frame diag: {Ir.round(4)}")
    print(f"sim plant kf {P.ROTOR_CONSTANT:.6e} c {P.ROLLING_MOMENT_COEFFICIENT:.6e} "
          f"lambda {P.ROTOR_LAMBDA}; believed kf {KF_BEL:.6e} km {KM_BEL}")
    allres = {"sim_plant": {"kf": P.ROTOR_CONSTANT, "c": P.ROLLING_MOMENT_COEFFICIENT,
                            "lambda": P.ROTOR_LAMBDA, "inertia_home_actual": [float(v) for v in Ir]},
              "believed": {"kf": KF_BEL, "km": KM_BEL}}
    lines = []
    for nm in names:
        path = os.path.join(ddir, f"{nm}.npz")
        if not os.path.exists(path):
            continue
        r = analyse(nm, path, Ir, Mr)
        allres[nm] = r
        lines.append(f"\n===== {nm}: {r['flight']}  DIRECT {r['direct'][0]:.1f}..{r['direct'][1]:.1f} s")
        for h in r["hover_balance"]:
            lines.append(f"  hold {h['t'][0]:6.1f}-{h['t'][1]:6.1f}: u1 {h['u1']:6.2f} N  dhat_z {h['dhat_z']:+6.2f} N"
                         f"  -> kf_true {h['kf_true']:.4e} (x{h['kf_scale_vs_sim']:.4f} of sim plant)"
                         f"  motors {np.round(h['motors'], 3)}  V {h['batt_v']:.2f}"
                         f"\n        dhat_t {np.round(h['dhat_t'], 3)} N  dhat_r {np.round(h['dhat_r'], 3)} N.m"
                         f"  dhat_q {np.round(h['dhat_q'], 3)}  eR {np.round(h['eR'], 4)}")
        if "kf_drift" in r:
            k = r["kf_drift"]
            lines.append(f"  kf drift: {k['kf_at_entry']:.4e} at entry, {k['frac_per_min']*100:+.2f} %/min over "
                         f"{k['span_s']:.0f} s, battery {k['batt_v_start_end'][0]:.2f} -> {k['batt_v_start_end'][1]:.2f} V")
        s = r["same_command"]
        lines.append(f"  same-command ({s['window_s']:.0f} s @ {s['fs_hz']:.0f} Hz, I_home {np.round(s['inertia_actual_diag'], 4)}):")
        for n_ in ("roll", "pitch", "yaw"):
            g = s[n_]
            sc = s["delay_scan"][n_]
            lines.append(f"    {n_:5}: gain {g['gain']:.3f} corr {g['corr']:.3f} (meas rms {g['rms_meas']:.3f}, pred {g['rms_pred']:.3f} rad/s^2)"
                         f" | best delay {sc['best_delay_s']*1000:.0f} ms corr {sc['corr_at_best']:.3f} gain {sc['gain_at_best']:.3f}")
        lines.append(f"    yaw gain with BENCH c: {s['yaw_gain_bench_c']:.3f}")
        lines.append(f"    thrust: DC gain {s['thrust']['gain_dc']:.4f} (kf {s['thrust']['kf_dc']:.4e}), AC gain {s['thrust']['gain_ac']:.3f} corr {s['thrust']['corr_ac']:.3f}")
        lines.append(f"    standing trim the symmetric model implies: {np.round(s['standing_trim_nm'], 3)} N.m (FRD)")
        a = r["arm_velocity_feedback"]
        lines.append(f"  arm joint_states {a['rate_hz']:.0f} Hz, velocity quantum {np.round(a['velocity_quantum_rad_s'], 4)} rad/s, "
                     f"lag vs FD(position): {a['velocity_lag']}")
        lines.append(f"  odom {r['odom_rate_hz']:.0f} Hz; hold velocity std (cm/s) {[np.round(f['vel_std_cm_s'], 2).tolist() for f in r['feedback_holds']]}")
    txt = "\n".join(lines)
    print(txt)
    adir = os.path.join(ddir, "..", "analysis")
    os.makedirs(adir, exist_ok=True)
    with open(os.path.join(adir, "fit_plant.json"), "w") as fh:
        json.dump(allres, fh, indent=1)
    with open(os.path.join(adir, "fit_plant.txt"), "w") as fh:
        fh.write(txt + "\n")


if __name__ == "__main__":
    main()
