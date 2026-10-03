#!/usr/bin/env python3
"""Sim-to-real comparison of one hardware flight against its Isaac replay.

    /usr/bin/python3 compare.py <real.npz> <sim.npz> <tag> [--rtf R]

Both files come from wb_l1_4d_flight_20260924/tools/extract_bag.py (same topic
set). The sim's wall clock is mapped to PLANT time with the real-time factor
measured from sensor_combined (PX4/Isaac stamps vs receive time), so a
half-speed simulation is compared at the pace the plant actually flew.

Phases come from the planner's status stream (EXECUTING -> HOLD), so the k-th
leg of the mission is compared against the k-th leg of the replay whatever
the operator's or the driver's hold lengths were. Every statistic uses the
definitions of wb_l1_4d_flight_20260924/tools/{phases,rmse_table}.py.

Writes analysis/compare_<tag>.json (stats + decimated traces) and prints the
table.
"""
import json
import os
import sys

import numpy as np

KT_COUNTS = np.array([162.4, 154.0, 150.5, 153.4])   # hardware joint_states.effort is current counts / (N.m)
D_Q, D_QD, D_TAU = slice(5, 9), slice(9, 13), slice(13, 17)
D_U1 = 17
D_EY, D_ER = slice(24, 28), slice(28, 31)
D_DHAT = slice(31, 41)
D_MOT = slice(41, 45)
D_XCD, D_XC = slice(45, 48), slice(48, 51)
D_NSAT = 51
D_FRAW = slice(97, 101)


def rtf_of(d):
    """Plant seconds per wall second, from sensor_combined's own stamps."""
    ts = d["sc__timestamp"].astype(np.float64) * 1e-6
    tr = d["sc__recv"]
    m = np.isfinite(ts) & (ts > 0)
    if m.sum() < 100:
        return 1.0
    A = np.column_stack([np.ones(m.sum()), tr[m] - tr[m][0]])
    c, *_ = np.linalg.lstsq(A, ts[m], rcond=None)
    return float(c[1])


def load(path, sim):
    d = np.load(path, allow_pickle=True)
    rtf = rtf_of(d) if sim else 1.0
    t0 = d["wb__recv"][0]
    T = lambda key: (d[f"{key}__recv"] - t0) * rtf  # noqa: E731
    r = {"rtf": rtf, "t": T("wb"), "D": d["wb__data"]}
    r["direct"] = r["D"][:, 0] > 0.5
    # planner status -> legs
    ts = T("pl_status")
    sv = [str(x) for x in d["pl_status__data"]]
    legs = []
    for i, s in enumerate(sv):
        if s.startswith("EXECUTING"):
            j = i + 1
            while j < len(sv) and not (sv[j].startswith("HOLD") or sv[j].startswith("IDLE")):
                j += 1
            T_plan = float(s.split("T=")[1].rstrip("s")) if "T=" in s else float("nan")
            legs.append((ts[i], ts[j] if j < len(sv) else ts[-1], T_plan))
    r["legs"] = legs
    tm = T("wbmode")
    mv = [str(x) for x in d["wbmode__data"]]
    ent = [tm[i] for i in range(len(mv)) if mv[i] == "DIRECT"]
    exi = [tm[i] for i in range(len(mv)) if mv[i] == "SAFETY" and ent and tm[i] > ent[0]]
    r["t_direct"] = (ent[0], exi[0] if exi else tm[-1]) if ent else (np.nan, np.nan)
    # odometry
    r["t_odom"] = T("odom")
    r["odom_p"] = np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"])
    r["odom_v"] = np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
    q = np.column_stack([d[f"odom__pose.pose.orientation.{k}"] for k in "wxyz"])
    r["tilt"] = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    w, x, y, z = q.T
    r["yaw"] = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    # arm (hardware convention j1..j4), applied torque in N.m
    names = list(d["js__name"][0])
    order = [names.index(f"joint{k}") for k in (1, 2, 3, 4)]
    r["t_js"] = T("js")
    eff = d["js__effort"][:, order]
    r["tau_applied"] = eff / KT_COUNTS if np.nanmax(np.abs(eff)) > 20 else eff
    r["qdot_js"] = d["js__velocity"][:, order]
    r["t_batt"] = T("batt")
    r["batt_v"] = d["batt__voltage_v"]
    r["t_ref"] = T("wbref")
    r["ref_v"] = np.column_stack([d[f"wbref__x_cd_dot.{k}"] for k in "xyz"])
    return r


def phases(r, hold_settle=2.0):
    """[(name, t_a, t_b)] in plant seconds: entry hold, then per leg the move
    and the settled hold that follows."""
    a, b = r["t_direct"]
    out = []
    legs = r["legs"]
    first = legs[0][0] if legs else b
    out.append(("hold0", a + 4.0, first - 0.5))
    for i, (ta, tb, Tp) in enumerate(legs):
        nxt = legs[i + 1][0] if i + 1 < len(legs) else b
        out.append((f"leg{i+1}_move", ta, tb))
        out.append((f"leg{i+1}_settle", tb + hold_settle, nxt - 0.5))
    return [(n, x, y) for n, x, y in out if y - x > 0.5]


def stats(r, ta, tb):
    t, D = r["t"], r["D"]
    m = (t >= ta) & (t <= tb) & r["direct"]
    if m.sum() < 10:
        return None
    Dm = D[m]
    ep = Dm[:, D_XC] - Dm[:, D_XCD]
    ey = Dm[:, D_EY][:, :3]
    hd = np.degrees(np.arcsin(np.clip(Dm[:, 27], -1, 1)))
    eR = Dm[:, D_ER]
    q = Dm[:, D_Q]
    qd = Dm[:, D_QD]
    mo = (r["t_odom"] >= ta) & (r["t_odom"] <= tb)
    ev = r["odom_v"][mo] - np.column_stack([np.interp(r["t_odom"][mo], r["t_ref"], r["ref_v"][:, k])
                                             for k in range(3)]) if mo.sum() else np.zeros((1, 3))
    mj = (r["t_js"] >= ta) & (r["t_js"] <= tb)
    rms = lambda v: float(np.sqrt(np.nanmean(v ** 2)))  # noqa: E731
    return {
        "n": int(m.sum()), "dur_s": float(tb - ta),
        "com_err_rms_mm": [rms(ep[:, k]) * 1e3 for k in range(3)],
        "com_err_norm_rms_mm": rms(np.linalg.norm(ep, axis=1)) * 1e3,
        "com_err_norm_peak_mm": float(np.linalg.norm(ep, axis=1).max()) * 1e3,
        "com_vel_err_rms_mms": [rms(ev[:, k]) * 1e3 for k in range(3)],
        "ee_task_err_rms_mm": [rms(ey[:, k]) * 1e3 for k in range(3)],
        "ee_task_norm_rms_mm": rms(np.linalg.norm(ey, axis=1)) * 1e3,
        "ee_task_norm_peak_mm": float(np.linalg.norm(ey, axis=1).max()) * 1e3,
        "heading_err_rms_deg": rms(hd), "heading_err_peak_deg": float(np.abs(hd).max()),
        "eR_norm_mean": float(np.linalg.norm(eR, axis=1).mean()),
        "eR_norm_peak": float(np.linalg.norm(eR, axis=1).max()),
        "tilt_mean_deg": float(r["tilt"][mo].mean()) if mo.sum() else float("nan"),
        "tilt_peak_deg": float(r["tilt"][mo].max()) if mo.sum() else float("nan"),
        "joint_err_rms_deg": [float(np.degrees(rms(q[:, k] - qd[:, k]))) for k in range(4)],
        "joint_err_peak_deg": [float(np.degrees(np.abs(q[:, k] - qd[:, k]).max())) for k in range(4)],
        "tau_cmd_rms_nm": [rms(Dm[:, 13 + k]) for k in range(4)],
        "tau_cmd_peak_nm": [float(np.abs(Dm[:, 13 + k]).max()) for k in range(4)],
        "tau_applied_rms_nm": [rms(r["tau_applied"][mj][:, k]) for k in range(4)] if mj.sum() else None,
        "u1_mean_n": float(Dm[:, D_U1].mean()), "u1_std_n": float(Dm[:, D_U1].std()),
        "dhat_t_mean_n": [float(v) for v in Dm[:, 31:34].mean(0)],
        "dhat_r_mean_nm": [float(v) for v in Dm[:, 34:37].mean(0)],
        "dhat_q_mean_nm": [float(v) for v in Dm[:, 37:41].mean(0)],
        "motors_mean": [float(v) for v in Dm[:, D_MOT].mean(0)],
        "motors_std": [float(v) for v in Dm[:, D_MOT].std(0)],
        "n_sat": int((Dm[:, D_NSAT] > 0).sum()),
        "n_clamp": int((np.abs(Dm[:, D_TAU]) > 2.99).sum()),
        "fraw_rms_n": [rms(Dm[:, 97 + k]) for k in range(3)] if Dm.shape[1] > 100 else None,
    }


def traces(r, ta, tb, hz=25.0):
    """Decimated traces on a phase-relative time base."""
    t, D = r["t"], r["D"]
    m = (t >= ta) & (t <= tb)
    if m.sum() < 5:
        return None
    tt = t[m]
    g = np.arange(0.0, tb - ta, 1.0 / hz)
    f = lambda col: np.interp(g, tt - ta, col[m]).round(5).tolist()  # noqa: E731
    ep = D[:, D_XC] - D[:, D_XCD]
    out = {"t": g.round(3).tolist(),
           "com_err_x": f(ep[:, 0]), "com_err_y": f(ep[:, 1]), "com_err_z": f(ep[:, 2]),
           "com_err_norm": f(np.linalg.norm(ep, axis=1)),
           "ee_err_norm": f(np.linalg.norm(D[:, 24:27], axis=1)),
           "heading_err_deg": f(np.degrees(np.arcsin(np.clip(D[:, 27], -1, 1)))),
           "eR_norm": f(np.linalg.norm(D[:, D_ER], axis=1)),
           "u1": f(D[:, D_U1]), "dhat_z": f(D[:, 33]), "dhat_x": f(D[:, 31]), "dhat_y": f(D[:, 32]),
           "dhat_rz": f(D[:, 36]),
           "mot_mean": f(D[:, D_MOT].mean(1)),
           "q2_deg": f(np.degrees(D[:, 6])), "q2d_deg": f(np.degrees(D[:, 10])),
           "q3_deg": f(np.degrees(D[:, 7])), "q3d_deg": f(np.degrees(D[:, 11])),
           "q1_deg": f(np.degrees(D[:, 5])), "q4_deg": f(np.degrees(D[:, 8])),
           "tau2": f(D[:, 14]), "tau3": f(D[:, 15])}
    mo = (r["t_odom"] >= ta) & (r["t_odom"] <= tb)
    if mo.sum() > 5:
        to = r["t_odom"][mo] - ta
        out["tilt_deg"] = np.interp(g, to, r["tilt"][mo]).round(4).tolist()
        for k, ax in enumerate("xyz"):
            out[f"pos_{ax}"] = np.interp(g, to, r["odom_p"][mo][:, k]).round(5).tolist()
            out[f"vel_{ax}"] = np.interp(g, to, r["odom_v"][mo][:, k]).round(5).tolist()
    return out


def main():
    real_p, sim_p, tag = sys.argv[1], sys.argv[2], sys.argv[3]
    R = load(real_p, sim=False)
    S = load(sim_p, sim=True)
    print(f"sim RTF (plant s / wall s): {S['rtf']:.3f}")
    print(f"real DIRECT {R['t_direct'][0]:.1f}..{R['t_direct'][1]:.1f} s, legs {[(round(a,1), round(b,1), T) for a, b, T in R['legs']]}")
    print(f"sim  DIRECT {S['t_direct'][0]:.1f}..{S['t_direct'][1]:.1f} s (plant), legs {[(round(a,1), round(b,1), T) for a, b, T in S['legs']]}")
    PR, PS = phases(R), phases(S)
    names = [n for n, _, _ in PR if n in [x for x, _, _ in PS]]
    res = {"tag": tag, "real": real_p, "sim": sim_p, "sim_rtf": S["rtf"], "phases": {}}
    hdr = f"{'phase':14} {'dur R/S':>10} {'CoM rms mm':>12} {'CoM pk mm':>11} {'EE rms mm':>11} {'head rms':>9} {'|eR| mean':>10} {'tilt pk':>8} {'q2/q3 err':>13} {'u1':>7} {'dhat_z':>7} {'tau2 pk':>8}"
    print(hdr)
    for n in names:
        _, ra, rb = next(x for x in PR if x[0] == n)
        _, sa, sb = next(x for x in PS if x[0] == n)
        sr, ss = stats(R, ra, rb), stats(S, sa, sb)
        if sr is None or ss is None:
            continue
        res["phases"][n] = {"real": sr, "sim": ss, "real_window": [ra, rb], "sim_window": [sa, sb],
                            "traces": {"real": traces(R, ra, rb), "sim": traces(S, sa, sb)}}
        for lab, st in (("R", sr), ("S", ss)):
            print(f"{(n if lab=='R' else ''):14} {st['dur_s']:5.1f}{lab:>5} {st['com_err_norm_rms_mm']:12.1f} {st['com_err_norm_peak_mm']:11.1f} "
                  f"{st['ee_task_norm_rms_mm']:11.1f} {st['heading_err_rms_deg']:9.2f} {st['eR_norm_mean']:10.4f} {st['tilt_peak_deg']:8.2f} "
                  f"{st['joint_err_rms_deg'][1]:6.2f}/{st['joint_err_rms_deg'][2]:<6.2f} {st['u1_mean_n']:7.2f} {st['dhat_t_mean_n'][2]:7.2f} {st['tau_cmd_peak_nm'][1]:8.2f}")
    # whole-DIRECT summary
    for lab, r in (("real", R), ("sim", S)):
        a, b = r["t_direct"]
        st = stats(r, a + 1.0, b - 0.5)
        res[f"direct_{lab}"] = st
        print(f"DIRECT {lab:4}: {b-a:6.1f} s | CoM rms {st['com_err_norm_rms_mm']:.1f} mm pk {st['com_err_norm_peak_mm']:.0f} | EE rms {st['ee_task_norm_rms_mm']:.1f} | "
              f"|eR| {st['eR_norm_mean']:.4f} pk {st['eR_norm_peak']:.3f} | tilt pk {st['tilt_peak_deg']:.1f} | sat {st['n_sat']} clamp {st['n_clamp']} | "
              f"motors {np.round(st['motors_mean'],3)} | dhat_t {np.round(st['dhat_t_mean_n'],2)} dhat_r {np.round(st['dhat_r_mean_nm'],3)}")
    adir = os.path.join(os.path.dirname(os.path.abspath(sim_p)), "..", "analysis")
    os.makedirs(adir, exist_ok=True)
    with open(os.path.join(adir, f"compare_{tag}.json"), "w") as fh:
        json.dump(res, fh)
    print("wrote", os.path.join(adir, f"compare_{tag}.json"))


if __name__ == "__main__":
    main()
