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

WINDOW (the same for every controller, 2026-10-09): the mission is six CONTIGUOUS phases -- go to
start, pick (ready / pick / close / exit), to place start, place (ready / place / release / exit), to
land start, land -- each from its first step's start to the next phase's start. The driver flies a
fixed TIMETABLE (pnp_mission_v2.py --timetable): every step starts at the same mission time on every
run and every controller, and the short hovers between steps are constant slack inside the phase they
end. So the phase boundaries agree across runs (checked, spread printed) and the window is the whole
mission, t = 0 at the start of go to start to the end of land (common length = the shortest run).

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
# the six contiguous phases: (key, label, the driver mark that starts it, the mark that ends it)
PHASES = [("go_to_start", "Go to start", "go_to_start:start", "ready_pick:start"),
          ("pick", "Pick", "ready_pick:start", "go_to_place_start:start"),
          ("to_place_start", "To place start", "go_to_place_start:start", "ready_place:start"),
          ("place", "Place", "ready_place:start", "go_to_land_start:start"),
          ("to_land_start", "To land start", "go_to_land_start:start", "execute_land:start"),
          ("land", "Land", "execute_land:start", "execute_land:end")]
PHASE_KEYS = [p[0] for p in PHASES]
PHASE_LABEL = {p[0]: p[1] for p in PHASES}
PHASE_ALIGN_TOL_S = 0.25      # phase starts must agree across runs to this (the timetable's purpose)
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
    # the six contiguous phases as flown (driver clock)
    phases = {}
    for key, _, a_, b_ in PHASES:
        if a_ not in marks or b_ not in marks:
            raise SystemExit(f"{path}: no mark {a_} / {b_} -- not a complete mission")
        phases[key] = (marks[a_], marks[b_])
    if "timetable" not in d.files or d["timetable"].size == 0:
        raise SystemExit(f"{path}: flown without --timetable (before 2026-10-09) -- re-fly")
    late = [(str(n), float(x)) for n, x in zip(d["late_name"], d["late_s"])] if "late_name" in d.files else []
    params = TP.make_params_t650(base_com=BASE_COM)
    t0 = phases["go_to_start"][0]
    t_end_flown = phases["land"][1]
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
    return dict(d=d, marks=marks, phases=phases, timetable=np.asarray(d["timetable"], float), late=late, t0=t0, tu=tu, P=P, Qu=Qu, q=q, q_d=q_d,
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
    # the timetable: every run's phases start at the same mission time -- check it
    for (m, k), R in all_R.items():
        if R["late"]:
            print(f"WARNING {m} run {k}: LATE steps {R['late']} -- its phases are not on the timetable")
    rel = {key: np.array([R["phases"][key][0] - R["t0"] for R in all_R.values()]) for key in PHASE_KEYS}
    spread = {key: float(v.max() - v.min()) for key, v in rel.items()}
    print("phase start [s] mean / spread across runs:",
          {key: (round(float(rel[key].mean()), 2), round(spread[key], 3)) for key in PHASE_KEYS})
    if max(spread.values()) > PHASE_ALIGN_TOL_S:
        sys.exit(f"phase starts differ by up to {max(spread.values()):.2f} s across runs (> {PHASE_ALIGN_TOL_S}) -- "
                 "not one timetable; re-fly or check the slots")
    T_common = min(R["phases"]["land"][1] - R["t0"] for R in all_R.values())
    PH = {key: (float(rel[key].mean()), None) for key in PHASE_KEYS}        # the common phase table
    for i, key in enumerate(PHASE_KEYS):
        PH[key] = (PH[key][0], PH[PHASE_KEYS[i + 1]][0] if i + 1 < len(PHASE_KEYS) else T_common)
    print(f"window: the whole mission, 0 .. {T_common:.2f} s (the shortest run); phases",
          {k: (round(a_, 2), round(b_, 2)) for k, (a_, b_) in PH.items()})
    for method, k, path in runs:
        R = all_R[(method, k)]
        name = f"{method}_pnp_run{k}"
        tu = R["tu"]
        t_rel = tu - R["t0"]
        fixed = t_rel <= T_common + 1e-9
        phase_id = np.zeros(len(tu))
        per_leg = {}
        for i, key in enumerate(PHASE_KEYS):
            s0, e0 = R["phases"][key]
            m = (tu >= s0) & (tu < e0) & fixed
            phase_id[m] = i + 1
            per_leg[key] = dict(label=PHASE_LABEL[key], start_s=s0 - R["t0"], end_s=min(e0 - R["t0"], T_common),
                                **metrics(R, m))
        whole = np.ones(len(tu), bool)
        r = metrics(R, fixed)
        r_whole = metrics(R, whole)
        r_whole["window_s"] = float(whole.sum() * DT)
        R["phase_id"] = phase_id
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
                    phases=dict(key=np.array(PHASE_KEYS, dtype=object),
                                label=np.array([PHASE_LABEL[x] for x in PHASE_KEYS], dtype=object),
                                start_s=np.array([R["phases"][x][0] - R["t0"] for x in PHASE_KEYS]),
                                end_s=np.array([R["phases"][x][1] - R["t0"] for x in PHASE_KEYS])),
                    timetable_slots_s=dict(ready_pick_after_go_to_start=R["timetable"][0],
                                           exit_pick_after_ready_pick=R["timetable"][1],
                                           ready_place_after_exit_pick=R["timetable"][2],
                                           go_to_land_start_after_ready_place=R["timetable"][3],
                                           execute_land_after_go_to_land_start=R["timetable"][4]),
                    late_steps=np.array([f"{n} {x:.2f} s" for n, x in R["late"]], dtype=object),
                    window=f"the whole mission, t = 0 (start of go to start) to {T_common:.2f} s (end of land, shortest run)",
                    L_arm_m=L_ARM, base_com_model_m=BASE_COM, aborted=bool(d["aborted"]), t0_driver_s=R["t0"],
                    clock="driver clock [s] (raw streams); tracking.t is s since the start of go to start")
        raw = dict(odom=dict(t=R["od"][:, 0], pos_m=R["od"][:, 1:4], vel_mps=R["od"][:, 4:7], quat_wxyz=R["od"][:, 7:11],
                             direct_mode=R["od"][:, 11], step=R["od"][:, 12]),
                   joints=dict(t=R["js"][:, 0], q_rad=R["js"][:, 1:5][:, R["order"]], effort=R["js"][:, 5:9][:, R["order"]]),
                   reference=dict(t=R["ref"][:, 0], r_ed=R["ref"][:, 1:4], x_cd=R["ref"][:, 4:7], q_d_rad=R["ref"][:, 7:11],
                                  b1_d_model=R["ref"][:, 12:15], x_cd_ddot=R["ref"][:, 15:18], b1_de=R["ref"][:, 18:21]),
                   payload=dict(t=pay[:, 0], pos_m=pay[:, 1:4], quat_wxyz=pay[:, 4:8]) if len(pay) else dict(),
                   claw_truth=dict(t=d["claw"][:, 0], pos_m=d["claw"][:, 1:4]) if d["claw"].shape[1] > 1 else dict(),
                   marks=dict(t=np.array([v for v in R["marks"].values()]), name=np.array(list(R["marks"].keys()), dtype=object)),
                   events=np.array([str(x) for x in d["events"]], dtype=object))
        tracking = dict(t=tu - R["t0"], in_window=fixed.astype(float), phase_id=phase_id,
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
                         {"run": dict(meta=meta, metrics=dict(window=r, whole_recording=r_whole, per_phase=per_leg),
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
    print("window per run [s]:", wins, "(must be one value)" if len(wins) > 1 else "")
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
    ph_rows = [dict(name=r["name"], method=r["method"], run=r["run"], phase=key, label=PHASE_LABEL[key],
                    start_s=r["per_leg"][key]["start_s"], end_s=r["per_leg"][key]["end_s"],
                    eps_uav_max_mm=r["per_leg"][key]["eps_uav_max_mm"], eps_uav_rms_mm=r["per_leg"][key]["eps_uav_rms_mm"],
                    rho_uav_max=r["per_leg"][key]["rho_uav_max"], rho_uav_rms=r["per_leg"][key]["rho_uav_rms"],
                    ee_pos_mm=r["per_leg"][key]["ee_pos_mm"], platform_pos_mm=r["per_leg"][key]["platform_pos_mm"])
               for r in rows for key in PHASE_KEYS]
    flat = []
    for r in rows:
        f = {k: r[k] for k in ["name", "method", "run"] + keys}
        for j, ax in enumerate("xyz"):
            f[f"platform_{ax}_mm"] = r["platform_pos_xyz_mm"][j]; f[f"ee_{ax}_mm"] = r["ee_pos_xyz_mm"][j]
        for j in range(4):
            f[f"q{j + 1}_deg"] = r["joints_deg"][j]
        flat.append(f)
    scipy.io.savemat(os.path.join(MAT_DIR, "pick_and_place_metrics.mat"),
                     {"metrics_runs": colstruct(flat, list(flat[0].keys())),
                      "metrics_mean": colstruct(summary, cols),
                      "metrics_phase": colstruct(ph_rows, list(ph_rows[0].keys())),
                      "phases": dict(key=np.array(PHASE_KEYS, dtype=object),
                                     label=np.array([PHASE_LABEL[x] for x in PHASE_KEYS], dtype=object),
                                     start_s=np.array([PH[x][0] for x in PHASE_KEYS]),
                                     end_s=np.array([PH[x][1] for x in PHASE_KEYS])),
                      "window_s": T_common, "L_arm_m": L_ARM},
                     do_compression=True, long_field_names=True, oned_as="column")
    tex = latex_table(summary, T_common)
    open(os.path.join(TABLES, "pick_and_place_sim.tex"), "w").write(tex)
    figs = make_figures(rows, loaded, a.figdir, PH, T_common)
    write_readme(rows, summary, PH, T_common, all_R)
    if a.paper:
        splice_paper(a.paper, tex, figs)
    print("wrote", os.path.join(TABLES, "pick_and_place_sim.{csv,json,tex}"), "and", a.figdir)


def colstruct(rows, keys):
    """A MATLAB struct of columns: numbers as double arrays, strings as cell arrays."""
    out = {}
    for k in keys:
        vals = [r[k] for r in rows]
        out[k] = np.array(vals, dtype=object) if isinstance(vals[0], str) else np.array(vals, dtype=float)
    return out


AUTOPILOT_CONFIG = os.environ.get("FSC_AUTOPILOT_CONFIG", os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config"))
FLOWN_CONFIGS = {   # name in configs/ <- the yaml the stack flew (whole-body; the decoupled rig reads its planner block from it)
    "whole_body_l1_4d_sim_pick_and_place.yaml": "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_place.yaml",
    "geometric_l1_sim_pick_and_place.yaml": "params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim_pick_and_place.yaml",
}
POINTER = ("**Payload pick-and-place** (the paper's `tab:sim_pick_place`): `pick_and_place/`, "
           "`pick_and_place_metrics.mat` and `configs/*_sim_pick_and_place.yaml`. See `README_pick_and_place.md`.")


def write_readme(rows, summary, PH, T_common, all_R):
    """matlab_simulation_data/README_pick_and_place.md (generated) + the as-flown configs + a pointer in README.md."""
    import shutil
    sys.path.insert(0, HERE)
    from readme_text import _md_table
    cfg_dir = os.path.join(MAT_DIR, "configs")
    os.makedirs(cfg_dir, exist_ok=True)
    for dst, src in FLOWN_CONFIGS.items():
        shutil.copy2(os.path.join(AUTOPILOT_CONFIG, src), os.path.join(cfg_dir, dst))
    jl = os.path.join(PNP, "campaign.jsonl")
    att = [json.loads(l) for l in open(jl)] if os.path.isfile(jl) else []
    att_by = {}
    for x in att:
        att_by.setdefault(x["method"], []).append(x)
    first = min((x["time"] for x in att), default="?")[:16].replace("T", " ")
    last = max((x["time"] for x in att), default="?")[:16].replace("T", " ")
    names = {"whole_body_l1": "Proposed", "geometric_l1": "Geo-L1", "modular_adaptive": "MAC"}
    run_rows = [[names[r["method"]], f"`pick_and_place/{r['name']}.mat`", f"{r['eps_uav_max_mm']:.1f}", f"{r['rho_uav_max']:.3f}",
                 f"{r['eps_uav_rms_mm']:.1f}", f"{r['rho_uav_rms']:.3f}", f"{r['ee_pos_mm']:.1f}", f"{r['place_off_axis_mm']:.1f}"]
                for r in sorted(rows, key=lambda x: (ORDER.index(x["method"]), x["run"]))]
    mean_rows = [[names[x["method"]], str(x["runs"]), f"{x['eps_uav_max_mm']:.1f}", f"{x['rho_uav_max']:.3f}",
                  f"{x['eps_uav_rms_mm']:.1f}", f"{x['rho_uav_rms']:.3f}", f"{x['ee_pos_mm']:.1f}"] for x in summary]
    att_rows = [[names.get(m, m), str(len(v)), str(sum(1 for y in v if y["status"] == "completed")),
                 "; ".join(sorted(set(y["why"] for y in v if y["status"] != "completed" and y["why"]))) or "--"]
                for m, v in att_by.items()]
    ph_rows = []
    what = {"go_to_start": "fly from the hover to the start waypoint; hover",
            "pick": "approach behind the stem, hover 2 s (descent trim), slide in, close the gripper around the stem, hold, lift (exit)",
            "to_place_start": "carry the basket to the place-start waypoint; hover",
            "place": "approach above the place hat, hover 2 s, trim sideways + 2 s settle + descend, open, back out and climb (exit); hover",
            "to_land_start": "fly to the land-start waypoint; hover",
            "land": "the approach to the landing point (the mission's last planned leg)"}
    for key in PHASE_KEYS:
        a_, b_ = PH[key]
        ph_rows.append([str(PHASE_KEYS.index(key) + 1), PHASE_LABEL[key], f"{a_:.2f}", f"{b_:.2f}", f"{b_ - a_:.2f}", what[key]])
    slots = next(iter(all_R.values()))["timetable"]
    L = [
        "# Simulation results, MATLAB data (payload pick-and-place)", "",
        "This is the data behind the paper's pick-and-place table (`tab:sim_pick_place`) and its figures.",
        "It sits beside the free-flight data in this folder (see `README.md`) and uses the same plant, the same",
        "`run` struct layout and the same notation. These files plus MATLAB are all you need.", "",
        "## Files", "",
        "```",
        "pick_and_place/<method>_pnp_run<k>.mat   one run, struct `run`",
        "pick_and_place_metrics.mat               metrics_mean = the table (mean over runs), metrics_runs = every run,",
        "                                         metrics_phase = every run x phase, phases = the common phase table,",
        "                                         window_s = the evaluation window, L_arm_m = L",
        "configs/whole_body_l1_4d_sim_pick_and_place.yaml   the whole-body controller + planner + plant, as flown",
        "configs/geometric_l1_sim_pick_and_place.yaml       the decoupled (Geo-L1) controller + plant, as flown;",
        "                                                   it flew the planner block of the whole-body file",
        "```", "",
        "- **`<method>`:** `whole_body_l1` = Proposed, `geometric_l1` = Geo-L1 (see `README.md`).",
        "- **MAC (`modular_adaptive`):** no run file. It did not complete the task in any of its six attempts on",
        "  2026-10-08 (the arm module saturated and the platform flipped within 2 s of entering its control law),",
        "  so the table reports no entry for it.", "",
        "```matlab",
        "S = load('pick_and_place_metrics.mat');",
        "P = S.phases;                                    % the six phases, common to every run",
        "r = load('pick_and_place/whole_body_l1_pnp_run1.mat').run;",
        "g = load('pick_and_place/geometric_l1_pnp_run1.mat').run;",
        "figure; hold on",
        "for i = 1:numel(P.start_s)                       % phase bands, alternating shades",
        "    c = 0.90 + 0.07*mod(i+1, 2);",
        "    patch([P.start_s(i) P.end_s(i) P.end_s(i) P.start_s(i)], [0 0 400 400], c*[1 1 1], 'EdgeColor', 'none');",
        "    text(mean([P.start_s(i) P.end_s(i)]), 390, P.label{i}, 'HorizontalAlignment', 'center');",
        "end",
        "w = r.tracking.in_window > 0;  plot(r.tracking.t(w), 1e3 * r.tracking.eps_uav_m(w))   % ||eps_UAV|| [mm]",
        "w = g.tracking.in_window > 0;  plot(g.tracking.t(w), 1e3 * g.tracking.eps_uav_m(w))",
        "xlim([0 S.window_s]); xlabel('Time (s)'); ylabel('||\\epsilon_{UAV}|| (mm)')",
        "yyaxis right; ylim([0 400] / (1e3 * S.L_arm_m)); ylabel('\\rho_{UAV}')",
        "T = struct2table(S.metrics_mean);                % the table, one row per method",
        "",
        "% 3-D: references (dashed), measured platform and end effector, the basket (ground truth)",
        "w = r.tracking.in_window > 0;  tr = r.tracking;",
        "figure; hold on; grid on; axis equal; view(-50, 28)",
        "plot3(tr.platform_pos_ref_m(w,1), tr.platform_pos_ref_m(w,2), tr.platform_pos_ref_m(w,3), 'k--')",
        "plot3(tr.ee_pos_ref_m(w,1), tr.ee_pos_ref_m(w,2), tr.ee_pos_ref_m(w,3), '--', 'Color', [0.45 0.45 0.45])",
        "plot3(tr.platform_pos_m(w,1), tr.platform_pos_m(w,2), tr.platform_pos_m(w,3))",
        "plot3(tr.ee_pos_m(w,1), tr.ee_pos_m(w,2), tr.ee_pos_m(w,3), ':')",
        "tp = r.raw.payload.t - r.meta.t0_driver_s;  wp = tp >= 0 & tp <= S.window_s;   % raw streams: driver clock",
        "plot3(r.raw.payload.pos_m(wp,1), r.raw.payload.pos_m(wp,2), r.raw.payload.pos_m(wp,3))",
        "",
        "% the error grid: platform / end effector xyz in mm, headings and joints in deg",
        "E = {1e3*tr.platform_pos_err_m, tr.platform_att_err_deg(:,3), 1e3*tr.ee_pos_err_m, tr.ee_heading_err_deg, tr.q_err_deg};",
        "```", "",
        "## Timing: one timetable for every run", "",
        "The flight driver starts every step at the same mission time on every run and every controller",
        f"(`pnp_mission_v2.py --timetable`, slots {', '.join(f'{x:g}' for x in slots)} s). The phases are",
        "contiguous: each runs from its first step's start to the next phase's start, so the short hovers",
        "between steps belong to the phase they end. `t = 0` is the start of the first transit. The phase table",
        "below is the mean over the runs. Every run's phase starts agree with it to "
        f"{max(max(abs(R['phases'][k][0] - R['t0'] - PH[k][0]) for k in PHASE_KEYS) for R in all_R.values()):.2f} s.", "",
        _md_table(["#", "phase", "start [s]", "end [s]", "duration [s]", "what happens"], ph_rows), "",
        f"- **Window:** every metric covers the whole mission, 0 to {T_common:.2f} s. `run.tracking.in_window`",
        "  marks it; `run.tracking.phase_id` (1..6) says which phase each sample is in.",
        "- **Constant pauses:** about 2 s of hover after each transit, needed before the planner accepts the",
        "  next precision step, and 2 s above each target, where the planner measures the descent trim.",
        "  The place descent always trims sideways first (planner `pick_place_descent_trim_first_min: 0`), so",
        "  it lasts the same on every run.", "",
        "## Simulation index", "",
        f"Flown {first} to {last} (attempt end times), headless Isaac Sim at real time, raw mocap-emulator feedback, the same",
        "plant, planner, scene and task for every method.", "",
        _md_table(["method", "file", "eps max [mm]", "rho max", "eps rms [mm]", "rho rms", "EE RMSE [mm]",
                   "placed off axis [mm]"], run_rows), "",
        "The table is the mean over the runs:", "",
        _md_table(["method", "runs", "eps max [mm]", "rho max", "eps rms [mm]", "rho rms", "EE RMSE [mm]"], mean_rows), "",
        "Attempts (a run is kept only if it completed, placed the basket and kept every step on its slot):", "",
        _md_table(["method", "attempts", "kept", "why the others were not kept"], att_rows) if att_rows else "--", "",
        "## Scene and task", "",
        "- **Field:** two 1 m pillars, PICK at (1.0, 1.0) m and PLACE at (-1.0, -1.0) m, each with the printed",
        "  hat (platform top 1.008 m).",
        "- **Payload:** the CAD basket with a wire hanger, 200 g, starting on the PICK hat.",
        "- **Grasp:** a hook. The claw slides in level under the hanger's arch, the fingers close around its",
        "  3 mm stem, and the lift hangs the basket on them. At the place the claw descends with the jaws",
        "  closed, opens, and backs out.",
        "- **Arm poses:** pick and place [0, 32, 38, 0] deg, carry [12, 38, 42, 0] deg.",
        "- **Plant:** the free-flight mirror plant (see `README.md`, Plant). All `sim_*` keys are in the configs.", "",
        "## The struct `run`", "",
        _md_table(["field", "content"], [
            ["`meta`", "method, run index, scene, plant, `phases` (key, label, start_s, end_s of this run), "
                       "`timetable_slots_s`, `late_steps` (empty), `window`, `L_arm_m`, `base_com_model_m`, `t0_driver_s`"],
            ["`metrics`", "`window` = this run's table numbers over the window; `per_phase.<key>` = the same per phase; "
                          "`whole_recording` = over every sample"],
            ["`tracking`", "100 Hz, `t` in s since the start of the mission; `in_window`, `phase_id`; measured, "
                           "reference (`_ref_`) and error (`_err_`) for the platform position, `eps_uav_m`, `rho_uav`, "
                           "the platform attitude, the system CoM, the end-effector position and heading, and the joints"],
            ["`raw`", "every recorded stream on the driver's clock (subtract `meta.t0_driver_s` to get `tracking.t`): "
                      "`odom`, `joints`, `reference` (the planner's whole-body reference), `payload` (ground truth), "
                      "`claw_truth`, `marks` (every step's start/end), `events`"]]), "",
        "Units: positions in m (world ENU, z up) unless the field name says mm; angles in deg unless it says rad.",
        "Errors are measured − reference. `platform_att_err_deg` columns are `[roll pitch heading]`.", "",
        "## Paper notation", "",
        _md_table(["paper symbol", "meaning", "summary field (`metrics_mean`)", "run file"], [
            ["‖ε<sub>UAV</sub>‖", "‖r<sub>UAV</sub><sup>ref</sup> − r<sub>UAV</sub>‖, the platform's deviation from its "
             "reference position [mm]", "`eps_uav_max_mm`, `eps_uav_rms_mm`", "`tracking.eps_uav_m` [m]"],
            ["ρ<sub>UAV</sub>", "‖ε<sub>UAV</sub>‖ / L, L = 0.370 m (the arm's reach)", "`rho_uav_max`, `rho_uav_rms`",
             "`tracking.rho_uav`"],
            ["r<sub>0,x</sub>, r<sub>0,y</sub>, r<sub>0,z</sub>", "platform position [mm]",
             "`platform_x_mm`, `platform_y_mm`, `platform_z_mm`", "`tracking.platform_pos_err_m(:,1:3)` [m]"],
            ["ψ<sub>0</sub>, φ<sub>0</sub>, θ<sub>0</sub>", "platform heading, roll, pitch [deg]",
             "`platform_heading_deg`, `platform_roll_deg`, `platform_pitch_deg`", "`tracking.platform_att_err_deg(:,[3 1 2])`"],
            ["r<sub>e,x</sub>, r<sub>e,y</sub>, r<sub>e,z</sub>", "end-effector position [mm]",
             "`ee_x_mm`, `ee_y_mm`, `ee_z_mm`", "`tracking.ee_pos_err_m(:,1:3)` [m]"],
            ["ψ<sub>e</sub>", "end-effector heading [deg]", "`ee_heading_deg`", "`tracking.ee_heading_err_deg`"],
            ["q<sub>1</sub> … q<sub>4</sub>", "joint angles [deg]", "`q1_deg` … `q4_deg`", "`tracking.q_err_deg(:,1:4)`"]]), "",
        "- **Reference:** the planner's dynamically compatible whole-body reference, the same for every method.",
        "  The platform reference x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>) is "
        "r<sub>UAV</sub><sup>ref</sup>.",
        "- **max / rms:** over the window; the table's other columns are RMS errors.", ""]
    with open(os.path.join(MAT_DIR, "README_pick_and_place.md"), "w") as f:
        f.write("\n".join(L))
    # the pointer in README.md (regenerated by build_free_flight.py, whose template carries it too)
    rp = os.path.join(MAT_DIR, "README.md")
    if os.path.isfile(rp):
        t = open(rp).read()
        if POINTER not in t:
            anchor = "## Simulation index"
            t = t.replace(anchor, POINTER + "\n\n" + anchor, 1) if anchor in t else t + "\n" + POINTER + "\n"
            open(rp, "w").write(t)
    print("wrote", os.path.join(MAT_DIR, "README_pick_and_place.md"))


def latex_table(summary, T_common):
    cols = ["eps_uav_max_mm", "rho_uav_max", "eps_uav_rms_mm", "rho_uav_rms",
            "platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
            "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg", "q4_deg"]
    fmt = {c: ("%.3f" if c.startswith("rho") else "%.2f") for c in cols}
    vals = {s["method"]: {c: fmt[c] % s[c] for c in cols} for s in summary}
    best = {c: min(float(v[c]) for v in vals.values()) for c in cols}
    lines = [r"\begin{table}[H]", r"\centering",
             r"\caption{Payload pick-and-place in simulation: platform deviation and RMSE of the tracked states over the whole mission.}",
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
              r"symbols of Table~\ref{tab:sim_free_flight_tracking}. The window is the whole mission (%.1f~s), from the "
              r"start of the first transit to the end of the landing approach. Every run flies the same timetable, so each "
              r"step starts at the same time on every run and the window is identical for every method. " % T_common +
              r"MAC did not complete the task in any of its six attempts (the arm module saturated and the platform "
              r"flipped within 2~s of entering its control law), so no entry is reported for it.",
              r"\end{tablenotes}", r"\end{threeparttable}", r"\end{table}", ""]
    return "\n".join(lines)


def make_figures(rows, loaded, figdir, PH, T_common):
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
        ref_mask = (R["tu"] - R["t0"]) <= T_common + 1e-9
        if first:
            ax.plot(R["xb"][ref_mask, 0], R["xb"][ref_mask, 1], R["xb"][ref_mask, 2], "k--", lw=0.9, label="Platform ref.")
            ax.plot(R["r_ed"][ref_mask, 0], R["r_ed"][ref_mask, 1], R["r_ed"][ref_mask, 2], "--", color="0.45", lw=0.9, label="End-effector ref.")
            first = False
        ax.plot(R["P"][ref_mask, 0], R["P"][ref_mask, 1], R["P"][ref_mask, 2], color=COLORS[m], lw=1.2, label=f"{METHODS[m][0].split('~')[0]} platform")
        ax.plot(R["re"][ref_mask, 0], R["re"][ref_mask, 1], R["re"][ref_mask, 2], color=COLORS[m], lw=1.0, ls=":", label=f"{METHODS[m][0].split('~')[0]} end-effector")
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
    def bands(ax, labels=False):
        # the six contiguous phases (common to every run), alternating shades
        for i, key in enumerate(PHASE_KEYS):
            a_, b_ = PH[key]
            ax.axvspan(a_, b_, color="0.90" if i % 2 == 0 else "0.97", lw=0)
            if labels:
                lab = PHASE_LABEL[key].replace(" ", "\n", 1) if (b_ - a_) < 8.0 else PHASE_LABEL[key]
                ax.text(0.5 * (a_ + b_), 0.97, lab, ha="center", va="top", fontsize=7,
                        transform=ax.get_xaxis_transform())
        ax.set_xlim(0.0, T_common)

    def tw(R):
        m = (R["tu"] - R["t0"]) <= T_common + 1e-9
        return m, R["tu"][m] - R["t0"]
    for ax, (title, unit, fn) in zip(axs.flat, panels):
        bands(ax)
        for m, R in sel:
            mk, t = tw(R)
            ax.plot(t, fn(R)[mk], color=COLORS[m], lw=0.9, label=METHODS[m][0].split("~")[0])
        ax.axhline(0, color="k", ls="--", lw=0.6)
        ax.set_title(title); ax.grid(alpha=0.3)
        ax.set_ylabel(f"$e$ ({unit})" if ax in axs[:, 0] or unit == "°" else "")
    for ax in axs[-1]:
        ax.set_xlabel("Time (s)")
    h, l = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.03))
    p = os.path.join(figdir, "sim_pick_place_tracking_error.png"); fig.savefig(p, dpi=220, bbox_inches="tight"); plt.close(fig); out["err"] = p
    # ---- UAV deviation ----
    fig, ax = plt.subplots(figsize=(7.2, 3.2), constrained_layout=True)
    bands(ax, labels=True)
    for m, R in sel:
        mk, t = tw(R)
        ax.plot(t, 1e3 * R["eps"][mk], color=COLORS[m], lw=0.9, label=METHODS[m][0].split("~")[0])
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
        r"\caption{Tracking errors in the simulated payload pick-and-place (bands: the mission's six phases, which start at the same time on every run; the whole mission is the evaluation window).}",
        r"\label{fig:sim_pick_place_tracking_error}", r"\end{figure}", "",
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_pick_place_uav_deviation.png}",
        r"\caption{Platform deviation $\|\varepsilon_{UAV}\|$ and its ratio $\rho_{UAV}$ to the arm's reach in the simulated payload pick-and-place.}",
        r"\label{fig:sim_pick_place_uav_deviation}", r"\end{figure}", "", ""])
    open(paper, "w").write(s[:i] + block + s[j:])
    print("spliced the table and three figures into", paper)


if __name__ == "__main__":
    main()
