#!/usr/bin/env python3
"""Score the valve-turning HEADING-CHANNEL sweep (2026-10-26, README.md beside tools/): every check
of the plan, one row per run, from the pl_mission.py recordings in valve_turning_20261015/runs/yw_*.

    PYTHONNOUSERSITE=1 /usr/bin/python3 yaw_score.py [tag ...] [--ref yw_ref_f048]
        (no tags = every yw_* run; writes ../analysis/scores.json + scores.txt)

Windows (driver marks): TURN STEP = push:start -> push:end (the 1.5 s settle + the 12 s turn);
TURN = the motion, push:start + 1.5 s -> + T_turn; MID = its middle 60 % (the near-constant-speed
part); AFTER = release:start -> the end of the record.

Signals:
  heading couple M   the TRUE contact moment about the vertical at the claw point (the rotor's
                     PhysX contact report, pl_metrics.analyse, the force channels' sign rule; the
                     sign fixed per run so the vehicle's mean power on the rotor is positive),
                     reported RESISTING the turn as positive, low-passed 5 Hz zero-phase
  rendered           M_psi e_psi'' + D_psi e_psi' + K_psi e_psi on the law's own e_y[3] (same filter)
  F_t                the contact force on the claw along the turn's tangent at the claw, resisting +
  task force         the planner's force-guard signal: wb_control_debug [97..99] low-passed 3 rad/s
  yaw torque         wb_control_debug [23] (the law's body torque, actual FLU); its ceiling = the
                     allocator's free yaw torque at the run's mean collective in the turn,
                     4 km min(f_max - f_h, f_h - f_min) from the yaml's alloc_* keys
  heading error      Isaac's true body yaw minus the reference's (b1_d + 90 deg)
"""
import argparse
import glob
import json
import math
import os
import re
import sys

import numpy as np
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
SWEEP = os.path.abspath(os.path.join(HERE, ".."))
ARCH = os.path.abspath(os.path.join(SWEEP, ".."))
RUNS = os.environ.get("YW_RUNS") or os.path.join(ARCH, "valve_turning_20261015", "runs")
sys.path.insert(0, os.path.join(ARCH, "push_pull_20261003", "tools"))
import pl_metrics as PM   # noqa: E402

K_T, R_FIN, M_VIS = 120.0, 0.160, 0.07
Q2_STOP, Q3_STOP = -20.0, 50.0
SETTLE = 1.5
DBG = 2                                  # recorder column = debug index + 2


def marks_of(z):
    m = {}
    for t, n in zip(z["marks_t"], z["marks_name"]):
        m[str(n)] = float(t)            # the last occurrence wins
    return m


def args_of(tag):
    d = {}
    p = os.path.join(RUNS, tag + ".args")
    if os.path.isfile(p):
        for ln in open(p):
            if "=" in ln:
                k, v = ln.strip().split("=", 1)
                d[k.strip()] = v.strip()
    return d


def yaml_val(txt, key, cast=float):
    m = re.search(rf"^\s*{re.escape(key)}:\s*([^\s#]+)", txt, re.M)
    return cast(m.group(1)) if m else None


def yaw_q(q):                            # wxyz columns -> yaw [rad]
    w, x, y, zz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * zz + x * y), 1.0 - 2.0 * (y * y + zz * zz))


def tilt_q(q):
    x, y = q[:, 1], q[:, 2]
    return np.degrees(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))


def contact_sign(z, a, b):
    ct, claw = z["contact"], z["claw"]
    w = (ct[:, 0] > a) & (ct[:, 0] < b)
    if w.sum() < 10:
        return (1.0, 1.0), float("nan")
    F = ct[w, 1:4] + ct[w, 4:7]
    c = np.column_stack([np.interp(ct[w, 0], claw[:, 0], claw[:, k]) for k in (1, 2, 3)])
    v = np.gradient(c, ct[w, 0], axis=0)
    p = float(np.nanmean(np.sum(F * v, axis=1)))
    return ((1.0, 1.0) if p > 0 else (-1.0, -1.0)), p


def uniform(t_src, x_src, a, b, fs=100.0):
    t = np.arange(a, b, 1.0 / fs)
    if x_src.ndim == 1:
        return t, np.interp(t, t_src, x_src)
    return t, np.column_stack([np.interp(t, t_src, x_src[:, k]) for k in range(x_src.shape[1])])


def band_rms(t, x, lo=0.5, hi=2.5):
    fs = 1.0 / np.median(np.diff(t))
    b, a = butter(2, [lo / (0.5 * fs), hi / (0.5 * fs)], btype="band")
    return filtfilt(b, a, x)


def spec_peak(t, x, lo=0.5, hi=2.5):
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    n = len(x)
    w = np.hanning(n)
    X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(n, np.median(np.diff(t)))
    amp = 2.0 * np.abs(X) / np.sum(w)
    m = (f >= lo) & (f <= hi)
    if not m.any():
        return float("nan"), float("nan")
    k = np.argmax(amp[m])
    return float(f[m][k]), float(amp[m][k])


def failure_mode(z, row):
    m = _failure_mode(z, row)
    txt = " | ".join(str(e) for e in z["events"]) + " " + str(z["reason"])
    if m and "left DIRECT" in txt and "SAFETY handover" not in m:
        m += " + SAFETY handover"
    bt = z["base_truth"]
    if len(bt) and tilt_q(bt[-50:, 4:8]).max() > 60.0:
        m += " (ended inverted)"
    return m


def _failure_mode(z, row):
    ev = [str(e) for e in z["events"]]
    txt = " | ".join(ev) + " " + str(z["reason"])
    if "SAFETY GUARD" in txt:
        m = re.search(r"SAFETY GUARD: ([^)]*)\)?", txt)
        g = m.group(1) if m else ""
        if "tilt" in g:
            return "tilt guard"
        if "force" in g:
            return "force guard"
        if "q3" in g:
            return "elbow-singular branch (q3 guard)"
        return "guard: " + g[:60]
    if "left DIRECT" in txt or "SAFETY" in txt and "mode" in txt:
        return "SAFETY handover"
    if row.get("aborted"):
        return "driver abort: " + str(z["reason"])[:80]
    return ""


def score(tag):
    p = os.path.join(RUNS, tag + ".npz")
    z = np.load(p, allow_pickle=True)
    yt = open(os.path.join(RUNS, tag + ".yaml")).read()
    A = args_of(tag)
    mk = marks_of(z)
    row = dict(run=tag, aborted=bool(z["aborted"]), reason=str(z["reason"])[:200])
    Kpsi, Dpsi, Mpsi = (yaml_val(yt, k) for k in ("wb_ky_psi", "wb_dy_psi", "wb_my_psi"))
    tau_f = float(A.get("FRICTION", "nan"))
    T_turn = yaml_val(yt, "push_pull_push_time_s")
    turn_deg = yaml_val(yt, "push_pull_turn_deg")
    s = 1.0 if turn_deg >= 0 else -1.0
    row.update(K_psi=Kpsi, D_psi=Dpsi, M_psi=Mpsi, tau_f=tau_f, turn_cmd_deg=turn_deg, T_turn=T_turn,
               friction_model=A.get("FRICTION_MODEL", "tanh"))
    row["share_pred"] = Kpsi / (K_T * R_FIN ** 2 + Kpsi)
    row["lag_pred_deg"] = math.degrees(tau_f / (K_T * R_FIN ** 2 + Kpsi))
    row["M_pred"] = tau_f * row["share_pred"]
    tA = mk.get("push:start")
    if tA is None:
        row.update(outcome="no turn", mode=failure_mode(z, row), complete=False)
        return row, None
    t_end_rec = float(z["odom"][-1, 0])
    tE = mk.get("push:end", None)
    a = tA + SETTLE
    b = min(tE if tE is not None else t_end_rec, a + T_turn)
    tE_ = tE if tE is not None else b
    mid = (a + 0.2 * T_turn, a + 0.8 * T_turn)
    complete = (not row["aborted"]) and ("exit:end" in mk)
    od = z["odom"]
    direct_lost = bool(np.any(od[(od[:, 0] > mk.get("direct", 0)) & (od[:, 0] < mk.get("exit:end", t_end_rec)), 11] < 0.5))
    row["complete"] = complete and not direct_lost
    row["mode"] = "" if row["complete"] else failure_mode(z, row)
    if not row["complete"] and direct_lost and not row["mode"]:
        row["mode"] = "SAFETY handover"
    # ---- truth attitude -------------------------------------------------------
    bt = z["base_truth"]
    wst = (bt[:, 0] >= tA) & (bt[:, 0] <= tE_)
    row["tilt_turn_max"] = float(tilt_q(bt[wst, 4:8]).max())
    wc = (bt[:, 0] >= mk.get("ready:end", tA)) & (bt[:, 0] <= mk.get("exit:end", t_end_rec))
    row["tilt_contact_max"] = float(tilt_q(bt[wc, 4:8]).max())
    ref = z["ref"]
    yaw_ref = np.unwrap(np.arctan2(ref[:, 12], ref[:, 11]) + 0.5 * np.pi)
    yaw_tr = np.unwrap(yaw_q(bt[:, 4:8]))
    yr = np.interp(bt[wst, 0], ref[:, 0], yaw_ref)
    he = np.degrees(np.arctan2(np.sin(yaw_tr[wst] - yr), np.cos(yaw_tr[wst] - yr)))
    row["head_err_max_deg"] = float(np.abs(he).max())
    row["head_err_mean_deg"] = float(np.mean(he))
    # ---- the law's debug: yaw torque, motors, task force ------------------------
    dbg = z["dbg"].astype(float)
    wd = (dbg[:, 0] >= tA) & (dbg[:, 0] <= tE_)
    u1 = dbg[wd, 17 + DBG]
    tz = dbg[wd, 23 + DBG]
    mot = dbg[wd, 41 + DBG:45 + DBG]
    kf = yaml_val(yt, "alloc_thrust_coeff")
    km = abs(yaml_val(yt, "alloc_rotor0_km"))
    w_idle, w_max = yaml_val(yt, "alloc_omega_idle"), yaml_val(yt, "alloc_omega_max")
    f_h = float(np.mean(u1)) / 4.0
    tz_ceil = 4.0 * km * min(kf * w_max ** 2 - f_h, f_h - kf * w_idle ** 2)
    row["yaw_torque_peak"] = float(np.abs(tz).max())
    td = dbg[wd, 0]
    if len(td) > 20:
        bb, aa = butter(2, 5.0 / (0.5 / np.median(np.diff(td))))
        row["yaw_torque_peak_5hz"] = float(np.abs(filtfilt(bb, aa, tz)).max())
    else:
        row["yaw_torque_peak_5hz"] = float("nan")
    row["yaw_torque_mean"] = float(np.mean(tz))
    row["yaw_ceiling"] = tz_ceil
    row["yaw_torque_frac"] = row["yaw_torque_peak"] / tz_ceil
    row["motor_max"] = float(np.nanmax(mot))
    row["motor_min"] = float(np.nanmin(mot))
    row["n_sat_max"] = float(np.nanmax(dbg[wd, 51 + DBG]))
    row["motor_sat"] = bool(row["motor_max"] >= 0.995 or row["motor_min"] <= 0.005 or row["n_sat_max"] > 0)
    # the planner's force-guard reading
    Fg = dbg[:, 97 + DBG:100 + DBG]
    tg = dbg[:, 0]
    lp = np.zeros_like(Fg)
    for k in range(1, len(tg)):
        al = 1.0 - math.exp(-3.0 * max(tg[k] - tg[k - 1], 0.0))
        lp[k] = lp[k - 1] + al * (np.nan_to_num(Fg[k]) - lp[k - 1])
    row["task_force_peak"] = float(np.linalg.norm(lp[wd], axis=1).max())
    # ---- joints ------------------------------------------------------------------
    js = z["joints"]
    names = [str(n) for n in z["joint_names"]]
    order = [names.index(f"joint{i}") for i in range(1, 5)] if all(f"joint{i}" in names for i in range(1, 5)) else [0, 1, 2, 3]
    q = np.degrees(js[:, 1:5][:, order])
    wj = (js[:, 0] >= tA) & (js[:, 0] <= tE_)
    row["q3_min"] = float(q[wj, 2].min())
    row["q2_min"] = float(q[wj, 1].min())
    row["q3_max"] = float(q[wj, 2].max())
    row["on_stop_frac"] = float(np.mean((q[wj, 1] < Q2_STOP + 0.5) | (q[wj, 2] > Q3_STOP - 0.5)))
    tu, qu = uniform(js[:, 0], q, mk.get("ready:end", tA), mk.get("exit:start", min(b + 4.0, float(js[-1, 0]))))
    mt = (tu >= a) & (tu <= b)
    T3 = (b - a) / 3.0
    for j, nm in ((0, "q1"), (3, "q4")):
        row[f"{nm}_std"] = float(np.std(qu[mt, j]))
        bp = band_rms(tu, qu[:, j])
        first = (tu >= a) & (tu < a + T3)
        last = (tu >= b - T3) & (tu <= b)
        r1 = float(np.sqrt(np.mean(bp[first] ** 2)))
        r3 = float(np.sqrt(np.mean(bp[last] ** 2)))
        row[f"{nm}_swing_first"], row[f"{nm}_swing_last"] = r1, r3
        row[f"{nm}_growth"] = r3 / r1 if r1 > 1e-6 else float("nan")
        # diagnostic (2026-10-26): the swing is FORCED by the turn and starts from rest, so the plan's
        # last / first third grows by construction; last / middle third separates a growing swing
        # (> 1) from one that follows the turn's speed (< 1), and the hold after the turn shows the decay
        midw = (tu >= a + T3) & (tu < b - T3)
        r2 = float(np.sqrt(np.mean(bp[midw] ** 2)))
        row[f"{nm}_swing_mid"] = r2
        row[f"{nm}_growth_mid"] = r3 / r2 if r2 > 1e-6 else float("nan")
        hw = (tu >= b) & (tu <= (t_rel_ if (t_rel_ := mk.get("release:start")) else b + 3.0))
        row[f"{nm}_swing_hold"] = float(np.sqrt(np.mean(bp[hw] ** 2))) if hw.any() else float("nan")
        row[f"{nm}_peak_hz"], row[f"{nm}_peak_amp"] = spec_peak(tu[mt], qu[mt, j])
    # ---- the valve -------------------------------------------------------------
    box = z["box"]
    axis = box[0, 1:4]
    beam = np.degrees(np.unwrap(yaw_q(box[:, 4:8])))
    def beam_at(t):
        return float(np.interp(t, box[:, 0], beam))
    row["turned_deg"] = s * (beam_at(tE_) - beam_at(tA))
    t_rel = mk.get("release:start")
    if t_rel is not None:
        wa = box[:, 0] >= t_rel
        row["back_after_release_deg"] = float(max(0.0, np.max(s * (beam_at(t_rel) - beam[wa]))))
        row["beam_after_release_deg"] = float(s * (beam[wa][-1] - beam_at(t_rel)))
    else:
        row["back_after_release_deg"] = float("nan")
    # ---- the impedance (pl_metrics) ------------------------------------------------
    # the PhysX report's sign convention is the scene's, fixed: +1 (calibrated on yw_ref_f048 and
    # yw_k1_f048, whose mean power on the rotor P = +5.3 / +6.4 mW ~ tau_f * mean turn rate = 6.3 mW).
    # The per-run power rule (build_valve_turning.contact_sign) is reported, not used: under the ~1 Hz
    # swing the power average is dominated by the oscillation and flipped yw_k3_f048's sign.
    sign_cal, power = contact_sign(z, a + 2.0, b)
    sign = (1.0, 1.0)
    row["contact_sign"], row["contact_power_w"], row["contact_sign_per_run_rule"] = sign[0], power, sign_cal[0]
    o = PM.analyse(p, PM.yaml_gains(os.path.join(RUNS, tag + ".yaml")), 5.0, sign=sign)
    S = o["series"]
    t = S["t"]
    Mh = s * -S["F"][:, 3]             # the couple resisting the turn, +
    Rh = s * -S["rendered"][:, 3]
    Eh = S["e_imp"][:, 3]
    claw = z["claw"]
    c = np.column_stack([np.interp(t, claw[:, 0], claw[:, k]) for k in (1, 2, 3)])
    rv = c[:, :2] - axis[:2]
    rad = np.linalg.norm(rv, axis=1)
    that = np.column_stack([-rv[:, 1], rv[:, 0]]) / rad[:, None]     # CCW tangent
    Ft = -s * np.sum(S["F"][:, :2] * that, axis=1)                    # resisting the turn, +
    wm = (t >= mid[0]) & (t <= mid[1])
    wt = (t >= a) & (t <= b)
    row["M_bar"] = float(np.mean(Mh[wm]))
    row["M_sigma"] = float(np.std(Mh[wm]))
    row["R_bar"] = float(np.mean(Rh[wm]))
    row["Ft_bar"] = float(np.mean(Ft[wm]))
    row["r_bar"] = float(np.mean(rad[wm]))
    row["Ftr_bar"] = row["Ft_bar"] * row["r_bar"]
    row["e_imp_psi_rms_turn"] = float(np.sqrt(np.mean(Eh[wt] ** 2)))
    row["M_rms_turn"] = float(np.sqrt(np.mean(Mh[wt] ** 2)))
    row["corr_turn"] = float(np.corrcoef(Mh[wt], Rh[wt])[0, 1])
    row["corr_mid"] = float(np.corrcoef(Mh[wm], Rh[wm])[0, 1])
    row["axis_sum_mid"] = float(np.mean(Mh[wm] + Ft[wm] * rad[wm]))     # = the valve's load (~ tau_f)
    row["M_std_turn"] = float(np.std(Mh[wt]))
    row["Ftr_std_turn"] = float(np.std(Ft[wt] * rad[wt]))
    row["share_meas"] = row["M_bar"] / (row["M_bar"] + row["Ftr_bar"]) if (row["M_bar"] + row["Ftr_bar"]) != 0 else float("nan")
    row["mean_moment_err_frac"] = abs(row["M_bar"] - row["R_bar"]) / abs(row["M_bar"]) if row["M_bar"] != 0 else float("nan")
    # the beam's lag behind the plan (the EE reference's azimuth about the axis)
    r_ed = ref[:, 1:4]
    az_plan = np.degrees(np.unwrap(np.arctan2(r_ed[:, 1] - axis[1], r_ed[:, 0] - axis[0])))
    tb = np.arange(a, b, 0.01)
    be = np.interp(tb, box[:, 0], beam) - np.interp(a, box[:, 0], beam)
    pl = np.interp(tb, ref[:, 0], az_plan) - np.interp(a, ref[:, 0], az_plan)
    lag = s * (pl - be)
    wmb = (tb >= mid[0]) & (tb <= mid[1])
    row["lag_mid_deg"] = float(np.mean(lag[wmb])) if wmb.any() else float("nan")
    row["lag_max_deg"] = float(np.max(np.abs(lag))) if len(lag) else float("nan")
    # eps_UAV over the turn step
    me = (S["eps_t"] >= tA) & (S["eps_t"] <= tE_)
    row["eps_turn_max_mm"] = float(1e3 * S["eps"][me].max()) if me.any() else float("nan")
    series = dict(t=t, M=Mh, R=Rh, e_imp=Eh, Ft=Ft, r=rad, a=a, b=b, mid=mid, tA=tA, tE=tE_,
                  bt_t=bt[:, 0], tilt=tilt_q(bt[:, 4:8]), dbg_t=dbg[:, 0], tz=dbg[:, 23 + DBG],
                  js_t=js[:, 0], q=q, he_t=bt[wst, 0], he=he, release=t_rel,
                  imp_t=t, F_ext=S["F"], rendered=S["rendered"], e_y=S["e_y"], e_imp_all=S["e_imp"])
    return row, series


def checks(row, ref):
    """The plan's checks -> (safety_ok, visible, list of failed check names)."""
    f = []
    if not row.get("complete"):
        f.append("complete")
        return False, False, f
    if row["tilt_turn_max"] > 7.0:
        f.append("tilt>7")
    if row["motor_sat"]:
        f.append("motor sat")
    if row["yaw_torque_frac"] > 0.70:
        f.append("yaw torque>70%")
    if ref is not None and row["head_err_max_deg"] > 2.0 * ref["head_err_max_deg"]:
        f.append("heading err>2x ref")
    for nm in ("q1", "q4"):
        if ref is not None and row[f"{nm}_std"] > 1.3 * ref[f"{nm}_std"]:
            f.append(f"{nm} std>1.3x ref")
        # the plan's last / first third (row[nm_growth]) is reported; the verdict uses last / middle
        # third and the hold after the turn (the swing is forced by the turn and starts from rest)
        if row[f"{nm}_growth_mid"] > 1.5 or row[f"{nm}_swing_hold"] > 1.2 * row[f"{nm}_swing_last"]:
            f.append(f"{nm} growing")
    if row["on_stop_frac"] > 0.10:
        f.append("on stop>10%")
    if row["q3_min"] < 5.0:
        f.append("q3<5")
    if row["task_force_peak"] > 4.0:
        f.append("force>4N")
    if row["eps_turn_max_mm"] > 80.0:
        f.append("eps>80mm")
    if row["turned_deg"] < 85.0:
        f.append("turned<85")
    if row["back_after_release_deg"] > 5.0:
        f.append("back>5deg")
    safe = not f
    visible = row["M_bar"] >= M_VIS and row["M_bar"] >= 2.0 * row["M_sigma"]
    return safe, visible, f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="*")
    ap.add_argument("--ref", nargs="+", default=["yw_ref_f048", "yw_ref_f048_2"],
                    help="the reference run(s); relative checks use their mean")
    ap.add_argument("--json", default=os.path.join(SWEEP, "analysis", "scores.json"))
    a = ap.parse_args()
    tags = a.tags or sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(RUNS, "yw_*.npz")))
    ref = None
    refs = [score(r)[0] for r in a.ref if os.path.isfile(os.path.join(RUNS, r + ".npz"))]
    if refs:
        ref = {k: float(np.mean([r_[k] for r_ in refs])) for k in ("head_err_max_deg", "q1_std", "q4_std")}
        print(f"reference ({', '.join(a.ref)}): heading err max {ref['head_err_max_deg']:.2f} deg, q1 std "
              f"{ref['q1_std']:.2f}, q4 std {ref['q4_std']:.2f} deg (mean)")
    db = json.load(open(a.json)) if os.path.isfile(a.json) else {}
    for tag in tags:
        try:
            row, _ = score(tag)
        except Exception as e:                  # noqa: BLE001
            print(f"{tag}: cannot score ({type(e).__name__}: {e})")
            continue
        safe, vis, failed = checks(row, ref) if row.get("complete") else (False, False, ["complete"])
        row.update(safe=safe, visible=vis, failed=failed)
        db[tag] = row
        print(f"\n=== {tag}  K_psi {row['K_psi']:.4g} D_psi {row['D_psi']:.4g} tau_f {row['tau_f']:.3f} "
              f"[{row['friction_model']}]  {'COMPLETE' if row['complete'] else 'FAILED: ' + row['mode']}  "
              f"SAFE={safe} VISIBLE={vis}  {failed}")
        if "tilt_turn_max" not in row:
            continue
        print(f"  tilt turn {row['tilt_turn_max']:.2f} deg (contact {row['tilt_contact_max']:.2f}) | yaw torque peak "
              f"{row['yaw_torque_peak']:.3f} N.m = {100 * row['yaw_torque_frac']:.0f} % of {row['yaw_ceiling']:.3f} (5 Hz {row['yaw_torque_peak_5hz']:.3f}, "
              f"mean {row['yaw_torque_mean']:+.3f}) | "
              f"motors {row['motor_min']:.3f}..{row['motor_max']:.3f} | heading err max {row['head_err_max_deg']:.2f} deg")
        print(f"  q1 std {row['q1_std']:.2f} growth plan/mid {row['q1_growth']:.2f}/{row['q1_growth_mid']:.2f} peak {row['q1_peak_hz']:.2f} Hz "
              f"{row['q1_peak_amp']:.2f} deg | q4 std {row['q4_std']:.2f} growth plan/mid {row['q4_growth']:.2f}/{row['q4_growth_mid']:.2f} peak "
              f"{row['q4_peak_hz']:.2f} Hz {row['q4_peak_amp']:.2f} deg | swing rms first/mid/last/hold q1 "
              f"{row['q1_swing_first']:.1f}/{row['q1_swing_mid']:.1f}/{row['q1_swing_last']:.1f}/{row['q1_swing_hold']:.1f} q4 "
              f"{row['q4_swing_first']:.1f}/{row['q4_swing_mid']:.1f}/{row['q4_swing_last']:.1f}/{row['q4_swing_hold']:.1f} | stop {100 * row['on_stop_frac']:.1f} % "
              f"q3 min {row['q3_min']:.1f} q2 min {row['q2_min']:.1f}")
        print(f"  task force peak {row['task_force_peak']:.2f} N | eps turn max {row['eps_turn_max_mm']:.0f} mm | "
              f"turned {row['turned_deg']:.1f} deg, back after release {row['back_after_release_deg']:.2f} deg")
        print(f"  M_bar {row['M_bar']:+.4f} N.m (sigma {row['M_sigma']:.4f}; pred {row['M_pred']:.4f}) | rendered "
              f"{row['R_bar']:+.4f} (err {100 * row['mean_moment_err_frac']:.0f} %) | corr turn {row['corr_turn']:.2f} "
              f"mid {row['corr_mid']:.2f} | Ft*r {row['Ftr_bar']:+.4f} (Ft {row['Ft_bar']:+.3f} N, r {row['r_bar']:.3f}) | "
              f"axis sum {row['axis_sum_mid']:+.4f} (std M {row['M_std_turn']:.3f}, Ft*r {row['Ftr_std_turn']:.3f}) | "
              f"share {row['share_meas']:.3f} vs pred {row['share_pred']:.3f} | lag mid {row['lag_mid_deg']:.2f} deg vs "
              f"pred {row['lag_pred_deg']:.2f} | e_imp psi rms {row['e_imp_psi_rms_turn']:.4f} | sign {row['contact_sign']:+.0f} "
              f"P {1e3 * row['contact_power_w']:.1f} mW")
    os.makedirs(os.path.dirname(a.json), exist_ok=True)
    json.dump(db, open(a.json, "w"), indent=1, default=float)
    print(f"\nwrote {a.json} ({len(db)} runs)")


if __name__ == "__main__":
    main()
