#!/usr/bin/env python3
"""Payload pick-and-place, simulation: raw Isaac runs -> metrics, MATLAB files, the paper table and figures.

    PYTHONNOUSERSITE=1 /usr/bin/python3 results/utils/build_pick_and_place.py [--paper <main.tex>] [--figdir <dir>]

Reads results/simulation_results/pick_and_place/<method>_pnp_run<k>.npz (the npz written by
docs/docs_aerial_manipulator/archive/pick_place_controllers_20261003/tools/pnp_mission_v2.py, flown by
run_pick_and_place_campaign.sh) and writes
    simulation_results/matlab_simulation_data/pick_and_place/<name>.mat       struct `run`: meta, raw, tracking, metrics
    simulation_results/matlab_simulation_data/pick_and_place_metrics.mat        `metrics_runs`, `metrics_mean`
    results/utils/tables/pick_and_place_sim.csv / .json / .tex                 the table (mean over runs)
    <figdir>/sim_pick_place_trajectory_3d.png, sim_pick_place_tracking_error.png, sim_pick_place_uav_deviation.png
and with --paper splices the table and the three figures into the paper's
\\subsubsection{Free Flight: Payload Pick-and-Place}.

WINDOW (fixed, the same for every controller): the pick-and-place mission is six planner legs --
go_to_start, execute_pick (ready / pick / exit), go_to_place_start, execute_place (ready / place /
exit), go_to_land_start, execute_land -- flown from the SAME plan on every rig, so every leg has the
same plan; each leg's window is [its start mark, start + D_k] with D_k the SHORTEST flown duration of
that leg over every run, so every method is scored over the same length from the same leg start;
the table's numbers are over the union of the six windows (total identical across runs, checked).

METRICS (the free-flight definitions, build_free_flight.py, on the pick-and-place reference stream)
  reference   the planner's WholeBodyReference: platform x_b = x_cd - R0 r_0c(q_d) with R0 the compatible
              attitude (thrust along x_cd_ddot + g e3, heading b1_d); EE r_ed, EE heading b1_de; joints q_d
  measured    odometry (position, attitude) + joint states; EE position / heading by the model FK
  eps_UAV     |x_b - r_UAV| (Suarez et al. 2020, eq. 9): max and RMS over the window; rho_UAV = eps / L,
              L = L1 + L2 = 0.370 m (the arm's reach, bench_metrics.py)
  RMSE        platform position xyz, attitude (rotation vector of R_ref^T R, body axes), EE position xyz,
              EE heading, joints q1..q4
"""
import argparse
import glob
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.join(RESULTS, "simulation_results")
REPO = os.path.abspath(os.path.join(RESULTS, ".."))
PNP = os.path.join(ROOT, "pick_and_place")
TABLES = os.path.join(HERE, "tables")
MAT_DIR = os.path.join(ROOT, "matlab_simulation_data")
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))

METHODS = {
    "whole_body_l1": ("Proposed", "Whole-body L1 impedance (proposed)"),
    "geometric_l1": (r"Geo-$\mathcal{L}_1$~\cite{cai2025experiment}", "Geometric control with L1 adaptive augmentation (Cai et al., CEP 2025)"),
    "modular_adaptive": (r"MAC~\cite{yadav2024modular}", "Modular adaptive control (Yadav et al., TMECH 2025)"),
}
ORDER = ["whole_body_l1", "geometric_l1", "modular_adaptive"]
COLORS = {"whole_body_l1": "#0072BD", "geometric_l1": "#D95319", "modular_adaptive": "#77AC30"}
NAME = re.compile(r"^(whole_body_l1|geometric_l1|modular_adaptive)_pnp_run(\d+)$")
LEGS = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place", "go_to_land_start", "execute_land"]
LEG_LABEL = {"go_to_start": "Go to start", "execute_pick": "Pick", "go_to_place_start": "To place start",
             "execute_place": "Place", "go_to_land_start": "To land start", "execute_land": "Land"}
# the driver's step names (pnp_mission_v2 STEPS) that begin each leg
LEG_START_MARK = {"go_to_start": "go_to_start:start", "execute_pick": "ready_pick:start",
                  "go_to_place_start": "go_to_place_start:start", "execute_place": "ready_place:start",
                  "go_to_land_start": "go_to_land_start:start", "execute_land": "execute_land:start"}
LEG_END_MARK = {"go_to_start": "go_to_start:end", "execute_pick": "exit_pick:end",
                "go_to_place_start": "go_to_place_start:end", "execute_place": "exit_place:end",
                "go_to_land_start": "go_to_land_start:end", "execute_land": "execute_land:end"}
G = 9.81
L_ARM = 0.370          # [m] L1 + L2, the arm's reach (bench_metrics.py)
DT = 0.01
BASE_COM = np.array([0.0, -0.017854, 0.0])


def interp(tq, t, X):
    X = np.asarray(X)
    if X.ndim == 1:
        return np.interp(tq, t, X)
    return np.column_stack([np.interp(tq, t, X[:, i]) for i in range(X.shape[1])])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def quat_cont(Q):
    Q = Q.copy()
    for i in range(1, len(Q)):
        if Q[i] @ Q[i - 1] < 0:
            Q[i] = -Q[i]
    return Q


def build_r0(b3, b1d):
    b3 = b3 / np.linalg.norm(b3)
    b2 = np.cross(b3, b1d); b2 /= np.linalg.norm(b2)
    return np.column_stack([np.cross(b2, b3), b2, b3])


def rms(x, axis=0):
    return np.sqrt(np.mean(np.asarray(x) ** 2, axis=axis))


def discover():
    out = []
    for p in sorted(glob.glob(os.path.join(PNP, "*_pnp_run*.npz"))):
        m = NAME.match(os.path.basename(p)[:-4])
        if m:
            out.append((m.group(1), int(m.group(2)), p))
    return out


def load_run(path):
    from scipy.spatial.transform import Rotation as Rot
    from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
    RZ90 = TP._Rz(0.5 * math.pi); RZM90 = TP._Rz(-0.5 * math.pi)
    d = np.load(path, allow_pickle=False)
    marks = {str(n): float(t) for t, n in zip(d["marks_t"], d["marks_name"])}
    od, ref, js = d["odom"], d["ref"], d["joints"]
    if ref.shape[1] < 21:
        raise SystemExit(f"{path}: the reference stream lacks x_cd_ddot / b1_de (driver before 2026-10-08) -- re-fly")
    names = [str(n) for n in d["joint_names"]]
    order = [names.index(f"joint{i}") for i in range(1, 5)] if all(f"joint{i}" in names for i in range(1, 5)) else [0, 1, 2, 3]
    # the legs as flown: [start mark, end mark]; the FIXED window per leg is set in main()
    # (the shortest flown duration of that leg over all runs, so every method is scored
    # over the same length from the same leg start)
    windows = {}
    for leg in LEGS:
        s = marks.get(LEG_START_MARK[leg]); e = marks.get(LEG_END_MARK[leg])
        if s is None or e is None:
            continue
        windows[leg] = (s, e)
    T_leg = {leg: e - s for leg, (s, e) in windows.items()}
    params = TP.make_params_t650(base_com=BASE_COM)
    t0 = windows[LEGS[0]][0]
    t_end_flown = max(e for _, e in windows.values())
    tu = np.arange(t0, t_end_flown + DT, DT)
    n = len(tu)
    P = interp(tu, od[:, 0], od[:, 1:4])
    Qwxyz = quat_cont(od[:, 7:11]); Qxyzw = Qwxyz[:, [1, 2, 3, 0]]
    Qu = interp(tu, od[:, 0], Qxyzw); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    Ra = Rot.from_quat(Qu).as_matrix()
    q = interp(tu, js[:, 0], js[:, 1:5][:, order])
    r_ed = interp(tu, ref[:, 0], ref[:, 1:4]); x_cd = interp(tu, ref[:, 0], ref[:, 4:7])
    q_d = interp(tu, ref[:, 0], ref[:, 7:11]); b1d = interp(tu, ref[:, 0], ref[:, 12:15])
    x_cd_dd = interp(tu, ref[:, 0], ref[:, 15:18]); b1de = interp(tu, ref[:, 0], ref[:, 18:21])
    xb = np.zeros((n, 3)); Rref = np.zeros((n, 3, 3)); xc = np.zeros((n, 3)); re = np.zeros((n, 3)); b1e = np.zeros((n, 3))
    for i in range(n):
        R0m = build_r0(x_cd_dd[i] + np.array([0.0, 0.0, G]), b1d[i])
        r0c_d, _, _ = TP.arm_fk_model(q_d[i], params)
        xb[i] = x_cd[i] - R0m @ r0c_d
        Rref[i] = R0m @ RZ90
        Rm = Ra[i] @ RZM90
        r0c, r0e, Re = TP.arm_fk_model(q[i], params)
        xc[i] = P[i] + Rm @ r0c; re[i] = P[i] + Rm @ r0e; b1e[i] = Rm @ Re[:, 0]
    E = np.einsum("nji,njk->nik", Rref, Ra)
    att_err = np.degrees(Rot.from_matrix(E).as_rotvec())
    az = np.degrees(np.unwrap(np.arctan2(b1e[:, 1], b1e[:, 0])))
    az_ref = np.degrees(np.unwrap(np.arctan2(b1de[:, 1], b1de[:, 0])))
    ee_head_err = np.degrees(wrap(np.radians(az - az_ref)))
    e_pos = P - xb; e_ee = re - r_ed; e_q = np.degrees(q - q_d)
    eps = np.linalg.norm(e_pos, axis=1)
    tilt = np.degrees(np.arccos(np.clip(Ra[:, 2, 2], -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(Ra[:, 1, 0], Ra[:, 0, 0])))
    yaw_ref = np.degrees(np.unwrap(np.arctan2(Rref[:, 1, 0], Rref[:, 0, 0])))
    pay = d["payload"]
    return dict(d=d, marks=marks, windows=windows, T_leg=T_leg, t0=t0, tu=tu, P=P, Qu=Qu, q=q, q_d=q_d,
                xb=xb, x_cd=x_cd, xc=xc, r_ed=r_ed, re=re, att_err=att_err, az=az, az_ref=az_ref,
                ee_head_err=ee_head_err, e_pos=e_pos, e_ee=e_ee, e_q=e_q, eps=eps, tilt=tilt, yaw=yaw,
                yaw_ref=yaw_ref, pay=pay, od=od, ref=ref, js=js, order=order)


def metrics(R, mask):
    e_pos, e_ee, e_q, att, eh, eps = (R[k][mask] for k in ("e_pos", "e_ee", "e_q", "att_err", "ee_head_err", "eps"))
    return dict(
        eps_uav_max_mm=float(1e3 * eps.max()), eps_uav_rms_mm=float(1e3 * rms(eps)),
        rho_uav_max=float(eps.max() / L_ARM), rho_uav_rms=float(rms(eps) / L_ARM),
        platform_pos_mm=float(1e3 * rms(np.linalg.norm(e_pos, axis=1))),
        platform_pos_xyz_mm=(1e3 * rms(e_pos)).tolist(),
        platform_roll_deg=float(rms(att[:, 0])), platform_pitch_deg=float(rms(att[:, 1])),
        platform_heading_deg=float(rms(att[:, 2])),
        ee_pos_mm=float(1e3 * rms(np.linalg.norm(e_ee, axis=1))), ee_pos_xyz_mm=(1e3 * rms(e_ee)).tolist(),
        ee_pos_max_mm=float(1e3 * np.linalg.norm(e_ee, axis=1).max()),
        ee_heading_deg=float(rms(eh)), joints_deg=rms(e_q).tolist(),
        tilt_max_deg=float(R["tilt"][mask].max()), window_s=float(mask.sum() * DT),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--paper", default="")
    ap.add_argument("--figdir", default=os.path.join(PNP, "figures"))
    a = ap.parse_args()
    import scipy.io
    runs = discover()
    if not runs:
        sys.exit(f"no runs in {PNP}")
    os.makedirs(os.path.join(MAT_DIR, "pick_and_place"), exist_ok=True)
    os.makedirs(TABLES, exist_ok=True); os.makedirs(a.figdir, exist_ok=True)
    rows, loaded = [], {}
    all_R = {(m, k): load_run(p) for m, k, p in runs}
    # the fixed per-leg window length: the shortest flown duration of that leg over every run
    D = {leg: min(R["T_leg"][leg] for R in all_R.values() if leg in R["T_leg"]) for leg in LEGS}
    print("fixed window per leg [s]:", {k: round(v, 2) for k, v in D.items()})
    for method, k, path in runs:
        R = all_R[(method, k)]
        name = f"{method}_pnp_run{k}"
        tu = R["tu"]
        fixed = np.zeros(len(tu), bool)
        per_leg = {}
        for leg, (s, e_flown) in R["windows"].items():
            m = (tu >= s) & (tu <= s + D[leg])
            fixed |= m
            per_leg[leg] = dict(start_s=s - R["t0"], window_s_fixed=D[leg], flown_s=e_flown - s, **metrics(R, m))
            R["windows"][leg] = (s, s + D[leg], e_flown)
        whole = (tu >= R["t0"]) & (tu <= max(w[2] for w in R["windows"].values()))
        r = metrics(R, fixed)
        r_whole = metrics(R, whole)
        r_whole["window_s"] = float(whole.sum() * DT)
        pay = R["pay"]
        placed_xy = pay[-1, 1:3] if len(pay) else np.full(2, np.nan)
        row = dict(name=name, method=method, run=k, total_window_s=r["window_s"],
                   place_off_axis_mm=float(1e3 * np.linalg.norm(placed_xy - np.array([-1.0, -1.0]))),
                   **r, whole_mission=r_whole, per_leg=per_leg)
        rows.append(row); loaded[name] = R
        print(f"{name:30s} win {r['window_s']:5.1f} s  eps max/rms {r['eps_uav_max_mm']:6.1f}/{r['eps_uav_rms_mm']:5.1f} mm "
              f"rho {r['rho_uav_max']:.3f}/{r['rho_uav_rms']:.3f}  plat {r['platform_pos_mm']:5.1f}  EE {r['ee_pos_mm']:5.1f} mm "
              f"head {r['ee_heading_deg']:.2f}  q " + " ".join(f"{x:.2f}" for x in r["joints_deg"]))
        d = R["d"]
        meta = dict(name=name, method=method, method_long=METHODS[method][1], run_index=k,
                    plant="Isaac Sim, sim-to-real mirror plant (config A), headless at real time (RTF 1), raw mocap feedback",
                    scene="07 pick-and-place: 1 m pillars with the printed hat (platform top 1.008 m), CAD basket payload 200 g, hook grasp",
                    flown_leg_durations_s=R["T_leg"], fixed_leg_windows_s=D,
                    window="union of [leg start, start + D_leg] over the six legs, D_leg = the shortest flown duration of that leg over all runs",
                    L_arm_m=L_ARM, base_com_model_m=BASE_COM, aborted=bool(d["aborted"]),
                    clock="driver clock [s] (raw streams); tracking.t is s since go_to_start")
        raw = dict(odom=dict(t=R["od"][:, 0], pos_m=R["od"][:, 1:4], vel_mps=R["od"][:, 4:7], quat_wxyz=R["od"][:, 7:11],
                             direct_mode=R["od"][:, 11], step=R["od"][:, 12]),
                   joints=dict(t=R["js"][:, 0], q_rad=R["js"][:, 1:5][:, R["order"]], effort=R["js"][:, 5:9][:, R["order"]]),
                   reference=dict(t=R["ref"][:, 0], r_ed=R["ref"][:, 1:4], x_cd=R["ref"][:, 4:7], q_d_rad=R["ref"][:, 7:11],
                                  b1_d_model=R["ref"][:, 12:15], x_cd_ddot=R["ref"][:, 15:18], b1_de=R["ref"][:, 18:21]),
                   payload=dict(t=pay[:, 0], pos_m=pay[:, 1:4], quat_wxyz=pay[:, 4:8]) if len(pay) else dict(),
                   claw_truth=dict(t=d["claw"][:, 0], pos_m=d["claw"][:, 1:4]) if d["claw"].shape[1] > 1 else dict(),
                   marks=dict(t=np.array([v for v in R["marks"].values()]), name=np.array(list(R["marks"].keys()), dtype=object)),
                   events=np.array([str(x) for x in d["events"]], dtype=object))
        tracking = dict(t=tu - R["t0"], in_window=fixed.astype(float),
                        platform_pos_m=R["P"], platform_pos_ref_m=R["xb"], platform_pos_err_m=R["e_pos"],
                        eps_uav_m=R["eps"], rho_uav=R["eps"] / L_ARM,
                        platform_quat_xyzw=R["Qu"], platform_att_err_deg=R["att_err"],
                        platform_heading_deg=R["yaw"], platform_heading_ref_deg=R["yaw_ref"], platform_tilt_deg=R["tilt"],
                        com_pos_m=R["xc"], com_pos_ref_m=R["x_cd"],
                        ee_pos_m=R["re"], ee_pos_ref_m=R["r_ed"], ee_pos_err_m=R["e_ee"],
                        ee_heading_deg=R["az"], ee_heading_ref_deg=R["az_ref"], ee_heading_err_deg=R["ee_head_err"],
                        q_deg=np.degrees(R["q"]), q_ref_deg=np.degrees(R["q_d"]), q_err_deg=R["e_q"],
                        columns="platform_att_err_deg = [roll pitch heading] (body axes); xyz columns are world ENU")
        scipy.io.savemat(os.path.join(MAT_DIR, "pick_and_place", name + ".mat"),
                         {"run": dict(meta=meta, metrics=dict(fixed_window=r, whole_mission=r_whole, per_leg=per_leg),
                                      tracking=tracking, raw=raw)},
                         do_compression=True, long_field_names=True, oned_as="column")

    # ---- mean over runs ------------------------------------------------------
    keys = ["eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms", "platform_pos_mm", "platform_heading_deg",
            "platform_roll_deg", "platform_pitch_deg", "ee_pos_mm", "ee_heading_deg", "total_window_s", "place_off_axis_mm"]
    summary = []
    for method in ORDER:
        rr = [r for r in rows if r["method"] == method]
        if not rr:
            continue
        s = dict(method=method, runs=len(rr))
        for kk in keys:
            s[kk] = float(np.mean([r[kk] for r in rr]))
        for j, ax in enumerate("xyz"):
            s[f"platform_{ax}_mm"] = float(np.mean([r["platform_pos_xyz_mm"][j] for r in rr]))
            s[f"ee_{ax}_mm"] = float(np.mean([r["ee_pos_xyz_mm"][j] for r in rr]))
        for j in range(4):
            s[f"q{j + 1}_deg"] = float(np.mean([r["joints_deg"][j] for r in rr]))
        summary.append(s)
    wins = sorted(set(round(r["total_window_s"], 1) for r in rows))
    print("fixed window per run [s]:", wins, "(must be one value)" if len(wins) > 1 else "")
    with open(os.path.join(TABLES, "pick_and_place_sim.json"), "w") as f:
        json.dump(dict(runs=rows, mean_over_runs=summary, L_arm_m=L_ARM), f, indent=1, default=float)
    cols = ["method", "runs", "total_window_s", "eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms",
            "platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
            "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg",
            "q4_deg", "platform_pos_mm", "ee_pos_mm", "place_off_axis_mm"]
    with open(os.path.join(TABLES, "pick_and_place_sim.csv"), "w") as f:
        f.write(",".join(cols) + "\n")
        for s in summary:
            f.write(",".join(str(s[c]) if isinstance(s[c], (str, int)) else f"{s[c]:.4f}" for c in cols) + "\n")
    scipy.io.savemat(os.path.join(MAT_DIR, "pick_and_place_metrics.mat"),
                     {"metrics_runs": {k: np.array([r[k] for r in rows], dtype=object) for k in ["name", "method", "run"] + keys},
                      "metrics_mean": {c: np.array([s[c] for s in summary], dtype=object) for c in cols}},
                     do_compression=True, long_field_names=True, oned_as="column")
    tex = latex_table(summary)
    open(os.path.join(TABLES, "pick_and_place_sim.tex"), "w").write(tex)
    figs = make_figures(rows, loaded, a.figdir)
    if a.paper:
        splice_paper(a.paper, tex, figs)
    print("wrote", os.path.join(TABLES, "pick_and_place_sim.{csv,json,tex}"), "and", a.figdir)


def latex_table(summary):
    cols = ["eps_uav_max_mm", "rho_uav_max", "eps_uav_rms_mm", "rho_uav_rms",
            "platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
            "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg", "q4_deg"]
    fmt = {c: ("%.3f" if c.startswith("rho") else "%.2f") for c in cols}
    vals = {s["method"]: {c: fmt[c] % s[c] for c in cols} for s in summary}
    best = {c: min(float(v[c]) for v in vals.values()) for c in cols}
    lines = [r"\begin{table}[H]", r"\centering",
             r"\caption{Payload pick-and-place in simulation: platform deviation and RMSE of the tracked states over the mission's six planned legs.}",
             r"\label{tab:sim_pick_place}", r"\footnotesize", r"\setlength{\tabcolsep}{3.5pt}", r"\begin{threeparttable}",
             r"\begin{tabular}{l c c c c c c c c c c c c c c c c c c c}", r"\toprule",
             r"& \multicolumn{4}{c}{UAV deviation} & \multicolumn{6}{c}{Platform} & \multicolumn{4}{c}{End-effector} & \multicolumn{4}{c}{Joint} \\",
             r"\cmidrule(lr){2-5} \cmidrule(lr){6-11} \cmidrule(lr){12-15} \cmidrule(lr){16-19}",
             r"Method & $\|\varepsilon_{UAV}\|_{\max}$ & $\rho_{UAV,\max}$ & $\|\varepsilon_{UAV}\|_{\mathrm{rms}}$ & $\rho_{UAV,\mathrm{rms}}$ & $r_{0,x}$ & $r_{0,y}$ & $r_{0,z}$ & $\psi_0$ & $\phi_0$ & $\theta_0$ & $r_{e,x}$ & $r_{e,y}$ & $r_{e,z}$ & $\psi_e$ & $q_1$ & $q_2$ & $q_3$ & $q_4$ \\",
             r" & (mm) & (--) & (mm) & (--) & (mm) & (mm) & (mm) & ($^\circ$) & ($^\circ$) & ($^\circ$) & (mm) & (mm) & (mm) & ($^\circ$) & ($^\circ$) & ($^\circ$) & ($^\circ$) & ($^\circ$) \\",
             r"\midrule"]
    for m in ORDER:
        if m not in vals:
            continue
        cells = []
        for c in cols:
            v = vals[m][c]
            cells.append(r"\textbf{%s}" % v if abs(float(v) - best[c]) < 1e-9 else v)
        lines.append(f"{METHODS[m][0]} & " + " & ".join(cells) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\begin{tablenotes}", r"\footnotesize",
              r"\item[] Each entry is the mean over the completed runs of that method. "
              r"$\|\varepsilon_{UAV}\| = \|\boldsymbol{r}_{UAV}^{ref} - \boldsymbol{r}_{UAV}\|$ is the platform's deviation from "
              r"its reference position and $\rho_{UAV} = \|\varepsilon_{UAV}\|/L$ its ratio to the arm's reach $L = 0.37$~m, "
              r"given as the maximum and the RMS over the window; the remaining columns are RMS tracking errors with the "
              r"symbols of Table~\ref{tab:sim_free_flight_tracking}. The window is the union of the six planned legs "
              r"(go to start, pick, go to place start, place, go to land start, land), each taken from its start for the "
              r"shortest duration flown for that leg over all runs, so the window is identical for every method. "
              r"MAC did not complete the task in any of its six attempts (the arm module saturated and the platform "
              r"flipped within 2~s of entering its control law), so no entry is reported for it.",
              r"\end{tablenotes}", r"\end{threeparttable}", r"\end{table}", ""]
    return "\n".join(lines)


def make_figures(rows, loaded, figdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams.update({"font.family": "serif", "font.size": 10, "axes.titlesize": 11, "mathtext.fontset": "stix"})
    # one representative run per method: the lowest EE RMSE
    pick = {}
    for r in rows:
        if r["method"] not in pick or r["ee_pos_mm"] < pick[r["method"]]["ee_pos_mm"]:
            pick[r["method"]] = r
    sel = [(m, loaded[pick[m]["name"]]) for m in ORDER if m in pick]
    out = {}
    # ---- 3-D trajectories ----
    fig = plt.figure(figsize=(7.2, 5.2)); ax = fig.add_subplot(111, projection="3d")
    th = np.linspace(0, 2 * np.pi, 40)
    for (cx, cy) in ((1.0, 1.0), (-1.0, -1.0)):
        for rad, z0, z1, col in ((0.049, 0.0, 1.0, "#8899aa"), (0.1003, 1.003, 1.008, "#bbbbbb")):
            X = cx + rad * np.cos(th); Y = cy + rad * np.sin(th)
            for zz in (z0, z1):
                ax.plot(X, Y, zz, color=col, lw=0.8)
            for i in range(0, 40, 10):
                ax.plot([X[i], X[i]], [Y[i], Y[i]], [z0, z1], color=col, lw=0.6)
    first = True
    for m, R in sel:
        ref_mask = np.ones(len(R["tu"]), bool)
        if first:
            ax.plot(R["xb"][:, 0], R["xb"][:, 1], R["xb"][:, 2], "k--", lw=0.9, label="Platform ref.")
            ax.plot(R["r_ed"][:, 0], R["r_ed"][:, 1], R["r_ed"][:, 2], "--", color="0.45", lw=0.9, label="End-effector ref.")
            first = False
        ax.plot(R["P"][:, 0], R["P"][:, 1], R["P"][:, 2], color=COLORS[m], lw=1.2, label=f"{METHODS[m][0].split('~')[0]} platform")
        ax.plot(R["re"][:, 0], R["re"][:, 1], R["re"][:, 2], color=COLORS[m], lw=1.0, ls=":", label=f"{METHODS[m][0].split('~')[0]} end-effector")
        if len(R["pay"]):
            ax.plot(R["pay"][:, 1], R["pay"][:, 2], R["pay"][:, 3], color=COLORS[m], lw=0.6, alpha=0.5)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_xlim(-1.4, 1.4); ax.set_ylim(-1.4, 1.4); ax.set_zlim(0.6, 1.5)
    ax.view_init(elev=28, azim=-50)
    ax.legend(loc="upper left", fontsize=8, ncol=2, frameon=False)
    ax.set_box_aspect((1, 1, 0.45))
    fig.tight_layout()
    p = os.path.join(figdir, "sim_pick_place_trajectory_3d.png"); fig.savefig(p, dpi=220); plt.close(fig); out["traj"] = p
    # ---- tracking errors ----
    fig, axs = plt.subplots(3, 4, figsize=(12.5, 8.6), sharex=True, constrained_layout=True)
    panels = [("$r_{0,x}$", "mm", lambda R: 1e3 * R["e_pos"][:, 0]), ("$r_{0,y}$", "mm", lambda R: 1e3 * R["e_pos"][:, 1]),
              ("$r_{0,z}$", "mm", lambda R: 1e3 * R["e_pos"][:, 2]), ("$\\psi_0$", "°", lambda R: R["att_err"][:, 2]),
              ("$r_{e,x}$", "mm", lambda R: 1e3 * R["e_ee"][:, 0]), ("$r_{e,y}$", "mm", lambda R: 1e3 * R["e_ee"][:, 1]),
              ("$r_{e,z}$", "mm", lambda R: 1e3 * R["e_ee"][:, 2]), ("$\\psi_e$", "°", lambda R: R["ee_head_err"]),
              ("$q_1$", "°", lambda R: R["e_q"][:, 0]), ("$q_2$", "°", lambda R: R["e_q"][:, 1]),
              ("$q_3$", "°", lambda R: R["e_q"][:, 2]), ("$q_4$", "°", lambda R: R["e_q"][:, 3])]
    legs_ref = sel[0][1]["windows"] if sel else {}
    for ax, (title, unit, fn) in zip(axs.flat, panels):
        for leg, (s, e, _) in legs_ref.items():
            ax.axvspan(s - sel[0][1]["t0"], e - sel[0][1]["t0"], color="0.92", lw=0)
        for m, R in sel:
            ax.plot(R["tu"] - R["t0"], fn(R), color=COLORS[m], lw=0.9, label=METHODS[m][0].split("~")[0])
        ax.axhline(0, color="k", ls="--", lw=0.6)
        ax.set_title(title); ax.grid(alpha=0.3)
        ax.set_ylabel(f"$e$ ({unit})" if ax in axs[:, 0] or unit == "°" else "")
    for ax in axs[-1]:
        ax.set_xlabel("Time (s)")
    h, l = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.03))
    p = os.path.join(figdir, "sim_pick_place_tracking_error.png"); fig.savefig(p, dpi=220); plt.close(fig); out["err"] = p
    # ---- UAV deviation ----
    fig, ax = plt.subplots(figsize=(7.2, 3.2), constrained_layout=True)
    for leg, (s, e, _) in legs_ref.items():
        ax.axvspan(s - sel[0][1]["t0"], e - sel[0][1]["t0"], color="0.92", lw=0)
        ax.text(0.5 * (s + e) - sel[0][1]["t0"], 0.97, LEG_LABEL[leg], ha="center", va="top", fontsize=7, transform=ax.get_xaxis_transform())
    for m, R in sel:
        ax.plot(R["tu"] - R["t0"], 1e3 * R["eps"], color=COLORS[m], lw=0.9, label=METHODS[m][0].split("~")[0])
    ax.set_xlabel("Time (s)"); ax.set_ylabel(r"$\|\varepsilon_{UAV}\|$ (mm)"); ax.grid(alpha=0.3)
    ax2 = ax.twinx(); ax2.set_ylim(np.array(ax.get_ylim()) / (1e3 * L_ARM)); ax2.set_ylabel(r"$\rho_{UAV}$ (--)")
    ax.set_ylim(0, 1.25 * ax.get_ylim()[1]); ax2.set_ylim(np.array(ax.get_ylim()) / (1e3 * L_ARM))
    ax.legend(loc="upper right", fontsize=8, frameon=False, bbox_to_anchor=(1.0, 0.88))
    p = os.path.join(figdir, "sim_pick_place_uav_deviation.png"); fig.savefig(p, dpi=220); plt.close(fig); out["dev"] = p
    return out


def splice_paper(paper, tex, figs):
    import shutil
    s = open(paper).read()
    head = r"\subsubsection{Free Flight: Payload Pick-and-Place}"
    nxt = r"\subsubsection{Physical Interaction: Box Push-and-Pull}"
    i = s.index(head); j = s.index(nxt)
    figdir = os.path.join(os.path.dirname(os.path.abspath(paper)), "Figures", "2-Results")
    os.makedirs(figdir, exist_ok=True)
    for p in figs.values():
        shutil.copy(p, os.path.join(figdir, os.path.basename(p)))
    block = head + "\n\n" + tex + "\n" + "\n".join([
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_pick_place_trajectory_3d.png}",
        r"\caption{Platform and end-effector trajectories in the simulated payload pick-and-place (one run per method; dashed: the planned references, which are the same for every method).}",
        r"\label{fig:sim_pick_place_trajectory_3d}", r"\end{figure}", "",
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_pick_place_tracking_error.png}",
        r"\caption{Tracking errors in the simulated payload pick-and-place (shaded: the six planned legs that form the evaluation window).}",
        r"\label{fig:sim_pick_place_tracking_error}", r"\end{figure}", "",
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_pick_place_uav_deviation.png}",
        r"\caption{Platform deviation $\|\varepsilon_{UAV}\|$ and its ratio $\rho_{UAV}$ to the arm's reach in the simulated payload pick-and-place.}",
        r"\label{fig:sim_pick_place_uav_deviation}", r"\end{figure}", "", ""])
    open(paper, "w").write(s[:i] + block + s[j:])
    print("spliced the table and three figures into", paper)


if __name__ == "__main__":
    main()
