#!/usr/bin/env python3
"""Payload pick-and-DROP, simulation: raw Isaac runs -> metrics, MATLAB files, the table and figures (2026-10-26).

    PYTHONNOUSERSITE=1 /usr/bin/python3 results/utils/build_pick_and_drop.py [--figdir <dir>]

Reads results/simulation_results/Isaac_simulation_data/pick_and_drop/<method>_pnd_run<k>.npz (pnp_mission_v2.py --drop-place
--timetable, flown by run_pick_and_drop_campaign.sh) and writes
    simulation_results/matlab_simulation_data/pick_and_drop/<name>.mat           struct `run`: meta, raw, tracking, metrics
    simulation_results/matlab_simulation_data/pick_and_drop/pick_and_drop_metrics.mat
    simulation_results/matlab_simulation_data/pick_and_drop/README.md, configs/, reference_outputs/
    results/utils/tables/pick_and_drop_sim.csv / _std.csv / .json / .tex
    <figdir>/sim_pick_drop_trajectory_3d.png, sim_pick_drop_tracking_error.png, sim_pick_drop_uav_deviation.png,
             sim_pick_drop_outcome.png

The loading, the reference reconstruction and the tracking metrics are build_pick_and_place.py's (imported,
same definitions); this file adds the drop's own measures and the drop timetable's phase table.

WINDOW: the whole mission, six contiguous phases -- go to start, pick (ready / pick / close / exit), to drop
start (carry + hover), drop (Ready To Drop, release, exit), to land start, land. The DROP TIMETABLE lifts the
clamped payload at once, so the pick phase's END (the lift) varies by a few seconds with the descent trim and
the close gate; Ready To Drop is anchored to the mission start, so every later phase starts at the same time
on every run (checked). The window is t = 0 at the start of go to start to the end of land (common length =
the shortest run).

DROP MEASURES (drop_score.py's definitions): the claw's error from its drop target over the last 1.5 s before
the gripper opened (the planner's arrival error); the payload at release (bottom height over the tray rim);
the payload at rest -- its offset from the tray centre and its tilt; FLAT = on the tray floor, tilt < 5 deg.
"""
import argparse
import glob
import importlib.util
import json
import os
import re
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_pick_and_place as bpp  # noqa: E402

ROOT, TABLES, MAT_DIR, REPO = bpp.ROOT, bpp.TABLES, bpp.MAT_DIR, bpp.REPO
PND = os.path.join(ROOT, "pick_and_drop")
PND_MAT = os.path.join(MAT_DIR, "pick_and_drop")
METHODS, ORDER, COLORS, L_ARM, DT = bpp.METHODS, bpp.ORDER, bpp.COLORS, bpp.L_ARM, bpp.DT
NAME = re.compile(r"^(whole_body_l1|geometric_l1|modular_adaptive)_pnd_run(\d+)$")
SHORT = {"whole_body_l1": "Proposed", "geometric_l1": "Geo-L1", "modular_adaptive": "MAC"}
PHASES = [("go_to_start", "Go to start", "go_to_start:start", "ready_pick:start"),
          ("pick", "Pick", "ready_pick:start", "go_to_place_start:start"),
          ("to_drop_start", "To drop start", "go_to_place_start:start", "ready_place:start"),
          ("drop", "Drop", "ready_place:start", "go_to_land_start:start"),
          ("to_land_start", "To land start", "go_to_land_start:start", "execute_land:start"),
          ("land", "Land", "execute_land:start", "execute_land:end")]
PHASE_KEYS = [p[0] for p in PHASES]
PHASE_LABEL = {p[0]: p[1] for p in PHASES}
FREE_START = {"to_drop_start"}   # starts at the lift, which the drop timetable does not hold (clamped)
PHASE_ALIGN_TOL_S = 0.25
_spec = importlib.util.spec_from_file_location(
    "drop_score", os.path.join(REPO, "docs/docs_aerial_manipulator/archive/pick_place_drop_20261014/tools/drop_score.py"))
DS = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(DS)
AUTOPILOT_CONFIG = bpp.AUTOPILOT_CONFIG
FLOWN_CONFIGS = {
    "whole_body_l1_4d_sim_pick_and_drop.yaml": "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_drop.yaml",
    "geometric_l1_sim_pick_and_place.yaml": "params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim_pick_and_place.yaml",
    "modular_adaptive_sim_pick_and_drop.yaml": "params_single_aerial_manipulator_modular_adaptive_direct_actuation_t650_sim_pick_and_drop.yaml",
}


def discover():
    out = []
    for p in sorted(glob.glob(os.path.join(PND, "*_pnd_run*.npz"))):
        m = NAME.match(os.path.basename(p)[:-4])
        if m:
            out.append((m.group(1), int(m.group(2)), p))
    return out


def drop_measures(R):
    """drop_score.py's measures as numbers."""
    d = R["d"]
    pay, arr, od = d["payload"], d["arrival"], d["odom"]
    t_open = R["marks"]["gripper_open:start"]
    m = (arr[:, 1] == 3) & (arr[:, 0] > t_open - 1.5) & (arr[:, 0] <= t_open)
    e = arr[m, 4:7].mean(axis=0) if m.any() else np.full(3, np.nan)
    claw_err = float(arr[m, 3].mean()) if m.any() else float("nan")
    z_open = np.interp(t_open, pay[:, 0], pay[:, 3])
    after = (pay[:, 0] >= t_open) & (pay[:, 3] < z_open - 0.010)
    t_rel = float(pay[after, 0][0]) if after.any() else t_open
    p_rel = DS.at(pay[:, 0], pay[:, 1:8], t_rel)
    p = pay[-1, 1:8]
    off = p[:2] - DS.TRAY_C
    tilt = float(DS.tilt_deg(p[3:7])[0])
    inside = bool(np.all(np.abs(off) < DS.TRAY_HALF_IN) and p[2] < DS.RIM + DS.PAY_BOTTOM)
    flat = bool(abs(p[2] - DS.FLOOR_REST) < 0.005 and tilt < 5.0)
    o0 = DS.at(od[:, 0], od[:, 1:4], t_open)
    t_ex0 = R["marks"].get("exit_place:start", t_open + 4.0)
    w = (od[:, 0] >= t_open) & (od[:, 0] <= t_ex0)
    lurch = float(np.linalg.norm(od[w, 1:3] - o0[:2], axis=1).max()) if w.any() else float("nan")
    return dict(claw_err_at_release_mm=1e3 * claw_err, claw_err_at_release_xyz_mm=(1e3 * e).tolist(),
                release_drop_height_mm=float(1e3 * (p_rel[2] - DS.PAY_BOTTOM - DS.RIM)),
                release_off_centre_mm=float(1e3 * np.linalg.norm(p_rel[:2] - DS.TRAY_C)),
                rest_off_centre_mm=float(1e3 * np.linalg.norm(off)), rest_off_centre_xy_mm=(1e3 * off).tolist(),
                rest_tilt_deg=tilt, in_tray=inside, flat=flat, lurch_after_release_mm=1e3 * lurch,
                rest_pos_m=p[:3].tolist())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--figdir", default=os.path.join(PND, "figures"))
    a = ap.parse_args()
    import scipy.io
    runs = discover()
    if not runs:
        sys.exit(f"no runs in {PND}")
    os.makedirs(PND_MAT, exist_ok=True); os.makedirs(TABLES, exist_ok=True); os.makedirs(a.figdir, exist_ok=True)
    bpp.PHASES = PHASES                         # load_run builds the phases from these marks
    all_R = {(m, k): bpp.load_run(p) for m, k, p in runs}
    for (m, k), R in all_R.items():
        if R["late"]:
            print(f"WARNING {m} run {k}: LATE steps {R['late']} -- its phases are not on the timetable")
        if "gripper_open:start" not in R["marks"]:
            sys.exit(f"{m} run {k}: no release mark")
    rel = {key: np.array([R["phases"][key][0] - R["t0"] for R in all_R.values()]) for key in PHASE_KEYS}
    spread = {key: float(v.max() - v.min()) for key, v in rel.items()}
    print("phase start [s] mean / spread across runs:",
          {key: (round(float(rel[key].mean()), 2), round(spread[key], 3)) for key in PHASE_KEYS})
    bad = {k: v for k, v in spread.items() if k not in FREE_START and v > PHASE_ALIGN_TOL_S}
    if bad:
        sys.exit(f"phase starts differ across runs {bad} (> {PHASE_ALIGN_TOL_S} s) -- not one timetable")
    T_common = min(R["phases"]["land"][1] - R["t0"] for R in all_R.values())
    PH = {}
    for i, key in enumerate(PHASE_KEYS):
        PH[key] = (float(rel[key].mean()), float(rel[PHASE_KEYS[i + 1]].mean()) if i + 1 < len(PHASE_KEYS) else T_common)
    lift = rel["to_drop_start"]
    print(f"window 0 .. {T_common:.2f} s; the lift (pick -> to drop start) at {lift.min():.2f} .. {lift.max():.2f} s")
    rows, loaded = [], {}
    for method, k, path in runs:
        R = all_R[(method, k)]
        name = f"{method}_pnd_run{k}"
        tu = R["tu"]; t_rel = tu - R["t0"]
        fixed = t_rel <= T_common + 1e-9
        phase_id = np.zeros(len(tu)); per_leg = {}
        for i, key in enumerate(PHASE_KEYS):
            s0, e0 = R["phases"][key]
            mk = (tu >= s0) & (tu < e0) & fixed
            phase_id[mk] = i + 1
            per_leg[key] = dict(label=PHASE_LABEL[key], start_s=s0 - R["t0"], end_s=min(e0 - R["t0"], T_common),
                                **bpp.metrics(R, mk))
        r = bpp.metrics(R, fixed)
        r_whole = bpp.metrics(R, np.ones(len(tu), bool))
        dm = drop_measures(R)
        R["phase_id"] = phase_id; R["drop"] = dm
        row = dict(name=name, method=method, run=k, total_window_s=r["window_s"], **r, **dm,
                   whole_mission=r_whole, per_leg=per_leg)
        rows.append(row); loaded[name] = R
        print(f"{name:30s} eps max/rms {r['eps_uav_max_mm']:6.1f}/{r['eps_uav_rms_mm']:5.1f} mm  EE {r['ee_pos_mm']:5.1f} mm  "
              f"claw@release {dm['claw_err_at_release_mm']:5.1f} mm  rest {dm['rest_off_centre_mm']:5.1f} mm "
              f"tilt {dm['rest_tilt_deg']:4.1f}  {'FLAT' if dm['flat'] else ('in tray' if dm['in_tray'] else 'OUT')}")
        d = R["d"]
        tt = np.asarray(d["timetable_drop"], float) if "timetable_drop" in d.files else np.zeros(0)
        meta = dict(name=name, method=method, method_long=METHODS[method][1], run_index=k,
                    plant="Isaac Sim, sim-to-real mirror plant (config A), headless at real time (RTF 1), raw mocap feedback, "
                          "arm bridge drained every physics step (PEGASUS_ARM_SPIN_DRAIN=1)",
                    scene="07 pick-and-DROP: 0.77 m pillars with the printed hat, the Box_Assembly payload 200 g clamped by its "
                          "fin, dropped into the 35 mm tray (371 x 290 mm, turned 90 deg) on the PLACE hat",
                    phases=dict(key=np.array(PHASE_KEYS, dtype=object),
                                label=np.array([PHASE_LABEL[x] for x in PHASE_KEYS], dtype=object),
                                start_s=np.array([R["phases"][x][0] - R["t0"] for x in PHASE_KEYS]),
                                end_s=np.array([R["phases"][x][1] - R["t0"] for x in PHASE_KEYS])),
                    timetable_slots_s=dict(ready_pick_after_go_to_start=R["timetable"][0],
                                           ready_drop_after_go_to_start=tt[0] if tt.size else np.nan,
                                           go_to_land_start_after_ready_drop=tt[1] if tt.size else np.nan,
                                           execute_land_after_go_to_land_start=R["timetable"][4]),
                    late_steps=np.array([f"{n} {x:.2f} s" for n, x in R["late"]], dtype=object),
                    window=f"the whole mission, t = 0 (start of go to start) to {T_common:.2f} s (end of land, shortest run)",
                    L_arm_m=L_ARM, base_com_model_m=bpp.BASE_COM, aborted=bool(d["aborted"]), t0_driver_s=R["t0"],
                    t_release_s=R["marks"]["gripper_open:start"] - R["t0"],
                    tray=dict(centre_m=DS.TRAY_C, half_inner_m=DS.TRAY_HALF_IN, rim_m=DS.RIM),
                    clock="driver clock [s] (raw streams); tracking.t is s since the start of go to start")
        raw = dict(odom=dict(t=R["od"][:, 0], pos_m=R["od"][:, 1:4], vel_mps=R["od"][:, 4:7], quat_wxyz=R["od"][:, 7:11],
                             direct_mode=R["od"][:, 11], step=R["od"][:, 12]),
                   joints=dict(t=R["js"][:, 0], q_rad=R["js"][:, 1:5][:, R["order"]], effort=R["js"][:, 5:9][:, R["order"]]),
                   reference=dict(t=R["ref"][:, 0], r_ed=R["ref"][:, 1:4], x_cd=R["ref"][:, 4:7], q_d_rad=R["ref"][:, 7:11],
                                  b1_d_model=R["ref"][:, 12:15], x_cd_ddot=R["ref"][:, 15:18], b1_de=R["ref"][:, 18:21]),
                   payload=dict(t=R["pay"][:, 0], pos_m=R["pay"][:, 1:4], quat_wxyz=R["pay"][:, 4:8]),
                   claw_truth=dict(t=d["claw"][:, 0], pos_m=d["claw"][:, 1:4]) if d["claw"].shape[1] > 1 else dict(),
                   marks=dict(t=np.array(list(R["marks"].values())), name=np.array(list(R["marks"].keys()), dtype=object)),
                   events=np.array([str(x) for x in d["events"]], dtype=object))
        tracking = dict(t=t_rel, in_window=fixed.astype(float), phase_id=phase_id,
                        platform_pos_m=R["P"], platform_pos_ref_m=R["xb"], platform_pos_err_m=R["e_pos"],
                        eps_uav_m=R["eps"], rho_uav=R["eps"] / L_ARM,
                        platform_quat_xyzw=R["Qu"], platform_att_err_deg=R["att_err"],
                        platform_heading_deg=R["yaw"], platform_heading_ref_deg=R["yaw_ref"], platform_tilt_deg=R["tilt"],
                        com_pos_m=R["xc"], com_pos_ref_m=R["x_cd"],
                        ee_pos_m=R["re"], ee_pos_ref_m=R["r_ed"], ee_pos_err_m=R["e_ee"],
                        ee_heading_deg=R["az"], ee_heading_ref_deg=R["az_ref"], ee_heading_err_deg=R["ee_head_err"],
                        q_deg=np.degrees(R["q"]), q_ref_deg=np.degrees(R["q_d"]), q_err_deg=R["e_q"],
                        columns="platform_att_err_deg = [roll pitch heading] (body axes); xyz columns are world ENU")
        dm_mat = {k_: (np.array(v) if isinstance(v, list) else (float(v) if not isinstance(v, bool) else float(v)))
                  for k_, v in dm.items()}
        scipy.io.savemat(os.path.join(PND_MAT, name + ".mat"),
                         {"run": dict(meta=meta, metrics=dict(window=r, whole_recording=r_whole, per_phase=per_leg, drop=dm_mat),
                                      tracking=tracking, raw=raw)},
                         do_compression=True, long_field_names=True, oned_as="column")

    # ---- mean and sample std over the kept runs -----------------------------
    keys = ["eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms", "platform_pos_mm", "platform_heading_deg",
            "platform_roll_deg", "platform_pitch_deg", "ee_pos_mm", "ee_heading_deg", "tilt_max_deg", "total_window_s",
            "claw_err_at_release_mm", "release_drop_height_mm", "rest_off_centre_mm", "rest_tilt_deg", "lurch_after_release_mm"]
    jl = os.path.join(PND, "campaign.jsonl")
    att = [json.loads(x) for x in open(jl)] if os.path.isfile(jl) else []
    summary, summary_std = [], []
    for method in ORDER:
        rr = [r for r in rows if r["method"] == method]
        if not rr:
            continue
        aa = [x for x in att if x["method"] == method]
        s = dict(method=method, runs=len(rr), attempts=len(aa) or len(rr), flat=sum(r["flat"] for r in rr))
        sd = dict(method=method, runs=len(rr), attempts=s["attempts"], flat=s["flat"])
        def put(key, vals):
            s[key] = float(np.mean(vals)); sd[key] = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
        for kk in keys:
            put(kk, [r[kk] for r in rr])
        for j, ax in enumerate("xyz"):
            put(f"platform_{ax}_mm", [r["platform_pos_xyz_mm"][j] for r in rr])
            put(f"ee_{ax}_mm", [r["ee_pos_xyz_mm"][j] for r in rr])
        for j in range(4):
            put(f"q{j + 1}_deg", [r["joints_deg"][j] for r in rr])
        summary.append(s); summary_std.append(sd)
    cols = ["method", "attempts", "runs", "flat", "total_window_s", "eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max",
            "rho_uav_rms", "platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
            "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg", "q4_deg",
            "platform_pos_mm", "ee_pos_mm", "tilt_max_deg", "claw_err_at_release_mm", "release_drop_height_mm",
            "rest_off_centre_mm", "rest_tilt_deg", "lurch_after_release_mm"]
    with open(os.path.join(TABLES, "pick_and_drop_sim.json"), "w") as f:
        json.dump(dict(runs=rows, mean_over_runs=summary, std_over_runs=summary_std, attempts=att, L_arm_m=L_ARM,
                       window_s=T_common), f, indent=1, default=float)
    for fn, tab in (("pick_and_drop_sim.csv", summary), ("pick_and_drop_sim_std.csv", summary_std)):
        with open(os.path.join(TABLES, fn), "w") as f:
            f.write(",".join(cols) + "\n")
            for s in tab:
                f.write(",".join(str(s[c]) if isinstance(s[c], (str, int)) else f"{s[c]:.4f}" for c in cols) + "\n")
    ph_rows = [dict(name=r["name"], method=r["method"], run=r["run"], phase=key, label=PHASE_LABEL[key],
                    start_s=r["per_leg"][key]["start_s"], end_s=r["per_leg"][key]["end_s"],
                    eps_uav_max_mm=r["per_leg"][key]["eps_uav_max_mm"], eps_uav_rms_mm=r["per_leg"][key]["eps_uav_rms_mm"],
                    rho_uav_max=r["per_leg"][key]["rho_uav_max"], rho_uav_rms=r["per_leg"][key]["rho_uav_rms"],
                    ee_pos_mm=r["per_leg"][key]["ee_pos_mm"], platform_pos_mm=r["per_leg"][key]["platform_pos_mm"],
                    tilt_max_deg=r["per_leg"][key]["tilt_max_deg"])
               for r in rows for key in PHASE_KEYS]
    flat_rows = []
    for r in rows:
        f = {k_: r[k_] for k_ in ["name", "method", "run"] + keys}
        f["flat"] = float(r["flat"]); f["in_tray"] = float(r["in_tray"])
        for j, ax in enumerate("xyz"):
            f[f"platform_{ax}_mm"] = r["platform_pos_xyz_mm"][j]; f[f"ee_{ax}_mm"] = r["ee_pos_xyz_mm"][j]
        for j in range(4):
            f[f"q{j + 1}_deg"] = r["joints_deg"][j]
        f["rest_x_mm"], f["rest_y_mm"] = r["rest_off_centre_xy_mm"]
        flat_rows.append(f)
    att_mat = dict(method=np.array([x["method"] for x in att], dtype=object), run=np.array([x["run"] for x in att], float),
                   attempt=np.array([x["attempt"] for x in att], float),
                   status=np.array([x["status"] for x in att], dtype=object),
                   why=np.array([x.get("why", "") for x in att], dtype=object)) if att else dict()
    scipy.io.savemat(os.path.join(PND_MAT, "pick_and_drop_metrics.mat"),
                     {"metrics_runs": bpp.colstruct(flat_rows, list(flat_rows[0].keys())),
                      "metrics_mean": bpp.colstruct(summary, cols), "metrics_std": bpp.colstruct(summary_std, cols),
                      "metrics_phase": bpp.colstruct(ph_rows, list(ph_rows[0].keys())),
                      "phases": dict(key=np.array(PHASE_KEYS, dtype=object),
                                     label=np.array([PHASE_LABEL[x] for x in PHASE_KEYS], dtype=object),
                                     start_s=np.array([PH[x][0] for x in PHASE_KEYS]),
                                     end_s=np.array([PH[x][1] for x in PHASE_KEYS])),
                      "attempts": att_mat, "window_s": T_common, "L_arm_m": L_ARM,
                      "tray": dict(centre_m=DS.TRAY_C, half_inner_m=DS.TRAY_HALF_IN, rim_m=DS.RIM)},
                     do_compression=True, long_field_names=True, oned_as="column")
    tex = latex_table(summary, summary_std, T_common)
    open(os.path.join(TABLES, "pick_and_drop_sim.tex"), "w").write(tex)
    figs = make_figures(rows, loaded, a.figdir, PH, T_common)
    ref = os.path.join(PND_MAT, "reference_outputs"); os.makedirs(ref, exist_ok=True)
    for p in list(figs.values()) + [os.path.join(TABLES, x) for x in
                                    ("pick_and_drop_sim.csv", "pick_and_drop_sim_std.csv", "pick_and_drop_sim.tex")]:
        shutil.copy2(p, ref)
    write_readme(rows, summary, summary_std, PH, T_common, all_R, att)
    print("wrote", os.path.join(TABLES, "pick_and_drop_sim.{csv,json,tex}"), "and", a.figdir)


def latex_table(summary, summary_std, T_common):
    cols = ["eps_uav_max_mm", "rho_uav_max", "eps_uav_rms_mm", "rho_uav_rms", "platform_x_mm", "platform_y_mm",
            "platform_z_mm", "platform_heading_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg",
            "claw_err_at_release_mm", "rest_off_centre_mm", "rest_tilt_deg"]
    fmt = {c: ("%.3f" if c.startswith("rho") else "%.1f") for c in cols}
    sd_by = {x["method"]: x for x in summary_std}
    lines = [r"\begin{table}[H]", r"\centering",
             r"\caption{Payload pick-and-drop in simulation: success, platform deviation, tracking RMSE and the drop's accuracy.}",
             r"\label{tab:sim_pick_drop}", r"\footnotesize", r"\setlength{\tabcolsep}{2.5pt}", r"\begin{threeparttable}",
             r"\begin{tabular}{l c c c c c c c c c c c c c c c c}", r"\toprule",
             r"& & \multicolumn{4}{c}{UAV deviation} & \multicolumn{4}{c}{Platform} & \multicolumn{4}{c}{End-effector} & \multicolumn{3}{c}{Drop} \\",
             r"\cmidrule(lr){3-6} \cmidrule(lr){7-10} \cmidrule(lr){11-14} \cmidrule(lr){15-17}",
             r"Method & Success & $\|\varepsilon_{UAV}\|_{\max}$ & $\rho_{UAV,\max}$ & $\|\varepsilon_{UAV}\|_{\mathrm{rms}}$ & $\rho_{UAV,\mathrm{rms}}$ & $r_{0,x}$ & $r_{0,y}$ & $r_{0,z}$ & $\psi_0$ & $r_{e,x}$ & $r_{e,y}$ & $r_{e,z}$ & $\psi_e$ & $e_{rel}$ & $d_{rest}$ & $\theta_{rest}$ \\",
             r" & & (mm) & (--) & (mm) & (--) & (mm) & (mm) & (mm) & ($^\circ$) & (mm) & (mm) & (mm) & ($^\circ$) & (mm) & (mm) & ($^\circ$) \\",
             r"\midrule"]
    for m in ORDER:
        s = next((x for x in summary if x["method"] == m), None)
        if s is None:
            continue
        sd = sd_by[m]
        cells = [f"{s['runs']}/{s['attempts']}"] + [(fmt[c] % s[c]) + (r"$\pm$" + fmt[c] % sd[c] if s["runs"] > 1 else "")
                                                     for c in cols]
        lines.append(f"{METHODS[m][0]} & " + " & ".join(cells) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\begin{tablenotes}", r"\footnotesize",
              r"\item[] Success: runs that delivered the payload into the tray over the attempts flown. Each other entry is "
              r"the mean $\pm$ the sample standard deviation over the successful runs. "
              r"$\|\varepsilon_{UAV}\| = \|\boldsymbol{r}_{UAV}^{ref} - \boldsymbol{r}_{UAV}\|$ is the platform's deviation from "
              r"its reference position and $\rho_{UAV} = \|\varepsilon_{UAV}\|/L$ its ratio to the arm's reach $L = 0.37$~m "
              r"(maximum and RMS over the window); the platform and end-effector columns are RMS tracking errors with the "
              r"symbols of Table~\ref{tab:sim_free_flight_tracking}. $e_{rel}$: the claw's distance from its drop target when "
              r"the gripper opened; $d_{rest}$, $\theta_{rest}$: the payload's offset from the tray centre and its tilt at rest. "
              r"The window is the whole mission (%.1f~s); every run flies the same timetable." % T_common,
              r"\end{tablenotes}", r"\end{threeparttable}", r"\end{table}", ""]
    return "\n".join(lines)


def _scene(ax):
    th = np.linspace(0, 2 * np.pi, 40)
    for (cx, cy) in ((1.0, 1.0), (-1.0, -1.0)):
        for rad, z0, z1, col in ((0.049, 0.0, 0.77, "#8899aa"), (0.1003, 0.773, 0.778, "#bbbbbb")):
            X = cx + rad * np.cos(th); Y = cy + rad * np.sin(th)
            for zz in (z0, z1):
                ax.plot(X, Y, zz, color=col, lw=0.8)
            for i in range(0, 40, 10):
                ax.plot([X[i], X[i]], [Y[i], Y[i]], [z0, z1], color=col, lw=0.6)
    hx, hy = DS.TRAY_HALF_OUT
    bx = DS.TRAY_C[0] + np.array([-hx, hx, hx, -hx, -hx]); by = DS.TRAY_C[1] + np.array([-hy, -hy, hy, hy, -hy])
    for zz in (DS.RIM - 0.035, DS.RIM):
        ax.plot(bx, by, zz, color="#c0504d", lw=0.8)


def make_figures(rows, loaded, figdir, PH, T_common):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "font.size": 10, "axes.titlesize": 11, "mathtext.fontset": "stix"})
    pick = {}
    for r in rows:   # one representative run per method: the lowest EE RMSE
        if r["method"] not in pick or r["ee_pos_mm"] < pick[r["method"]]["ee_pos_mm"]:
            pick[r["method"]] = r
    sel = [(m, loaded[pick[m]["name"]]) for m in ORDER if m in pick]
    out = {}
    lab = lambda m: SHORT[m]  # noqa: E731
    # ---- 3-D ----
    fig = plt.figure(figsize=(7.2, 5.2)); ax = fig.add_subplot(111, projection="3d")
    _scene(ax)
    first = True
    for m, R in sel:
        mk = (R["tu"] - R["t0"]) <= T_common + 1e-9
        if first:
            ax.plot(R["xb"][mk, 0], R["xb"][mk, 1], R["xb"][mk, 2], "k--", lw=0.9, label="Platform ref.")
            ax.plot(R["r_ed"][mk, 0], R["r_ed"][mk, 1], R["r_ed"][mk, 2], "--", color="0.45", lw=0.9, label="End-effector ref.")
            first = False
        ax.plot(R["P"][mk, 0], R["P"][mk, 1], R["P"][mk, 2], color=COLORS[m], lw=1.2, label=f"{lab(m)} platform")
        ax.plot(R["re"][mk, 0], R["re"][mk, 1], R["re"][mk, 2], color=COLORS[m], lw=1.0, ls=":", label=f"{lab(m)} end-effector")
        ax.plot(R["pay"][:, 1], R["pay"][:, 2], R["pay"][:, 3], color=COLORS[m], lw=0.6, alpha=0.5)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_xlim(-1.4, 1.4); ax.set_ylim(-1.4, 1.4); ax.set_zlim(0.5, 1.5)
    ax.view_init(elev=28, azim=-50); ax.set_box_aspect((1, 1, 0.45))
    ax.legend(loc="upper left", fontsize=8, ncol=2, frameon=False)
    fig.tight_layout()
    p = os.path.join(figdir, "sim_pick_drop_trajectory_3d.png"); fig.savefig(p, dpi=220); plt.close(fig); out["traj"] = p
    # ---- tracking errors ----
    fig, axs = plt.subplots(3, 4, figsize=(12.5, 8.6), sharex=True, constrained_layout=True)
    panels = [("$r_{0,x}$", "mm", lambda R: 1e3 * R["e_pos"][:, 0]), ("$r_{0,y}$", "mm", lambda R: 1e3 * R["e_pos"][:, 1]),
              ("$r_{0,z}$", "mm", lambda R: 1e3 * R["e_pos"][:, 2]), ("$\\psi_0$", "°", lambda R: R["att_err"][:, 2]),
              ("$r_{e,x}$", "mm", lambda R: 1e3 * R["e_ee"][:, 0]), ("$r_{e,y}$", "mm", lambda R: 1e3 * R["e_ee"][:, 1]),
              ("$r_{e,z}$", "mm", lambda R: 1e3 * R["e_ee"][:, 2]), ("$\\psi_e$", "°", lambda R: R["ee_head_err"]),
              ("$q_1$", "°", lambda R: R["e_q"][:, 0]), ("$q_2$", "°", lambda R: R["e_q"][:, 1]),
              ("$q_3$", "°", lambda R: R["e_q"][:, 2]), ("$q_4$", "°", lambda R: R["e_q"][:, 3])]

    def bands(ax, labels=False):
        for i, key in enumerate(PHASE_KEYS):
            a_, b_ = PH[key]
            ax.axvspan(a_, b_, color="0.90" if i % 2 == 0 else "0.97", lw=0)
            if labels:
                t = PHASE_LABEL[key].replace(" ", "\n", 1) if (b_ - a_) < 8.0 else PHASE_LABEL[key]
                ax.text(0.5 * (a_ + b_), 0.97, t, ha="center", va="top", fontsize=7, transform=ax.get_xaxis_transform())
        ax.set_xlim(0.0, T_common)

    def tw(R):
        mk = (R["tu"] - R["t0"]) <= T_common + 1e-9
        return mk, R["tu"][mk] - R["t0"]
    for ax, (title, unit, fn) in zip(axs.flat, panels):
        bands(ax)
        for m, R in sel:
            mk, t = tw(R)
            ax.plot(t, fn(R)[mk], color=COLORS[m], lw=0.9, label=lab(m))
        ax.axhline(0, color="k", ls="--", lw=0.6)
        ax.set_title(title); ax.grid(alpha=0.3)
        ax.set_ylabel(f"$e$ ({unit})" if ax in axs[:, 0] or unit == "°" else "")
    for ax in axs[-1]:
        ax.set_xlabel("Time (s)")
    h, l = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.03))
    p = os.path.join(figdir, "sim_pick_drop_tracking_error.png"); fig.savefig(p, dpi=220, bbox_inches="tight"); plt.close(fig); out["err"] = p
    # ---- UAV deviation ----
    fig, ax = plt.subplots(figsize=(7.2, 3.2), constrained_layout=True)
    bands(ax, labels=True)
    for m, R in sel:
        mk, t = tw(R)
        ax.plot(t, 1e3 * R["eps"][mk], color=COLORS[m], lw=0.9, label=lab(m))
    ax.set_xlabel("Time (s)"); ax.set_ylabel(r"$\|\varepsilon_{UAV}\|$ (mm)"); ax.grid(alpha=0.3)
    ax.set_ylim(0, 1.25 * ax.get_ylim()[1])
    ax2 = ax.twinx(); ax2.set_ylim(np.array(ax.get_ylim()) / (1e3 * L_ARM)); ax2.set_ylabel(r"$\rho_{UAV}$ (--)")
    ax.legend(loc="upper right", fontsize=8, frameon=False, bbox_to_anchor=(1.0, 0.88))
    p = os.path.join(figdir, "sim_pick_drop_uav_deviation.png"); fig.savefig(p, dpi=220); plt.close(fig); out["dev"] = p
    # ---- the drop's outcome: every run's payload at rest, top view of the tray ----
    fig, ax = plt.subplots(figsize=(4.6, 5.0), constrained_layout=True)
    hx, hy = DS.TRAY_HALF_IN * 1e3
    ax.add_patch(plt.Rectangle((-hx, -hy), 2 * hx, 2 * hy, fill=False, color="#c0504d", lw=1.2))
    ax.plot(0, 0, "k+", ms=10)
    for m in ORDER:
        rr = [r for r in rows if r["method"] == m]
        for i, r in enumerate(rr):
            x, y = r["rest_off_centre_xy_mm"]
            ax.plot(x, y, "o" if r["flat"] else "^", color=COLORS[m], ms=8, mfc=COLORS[m] if r["flat"] else "none",
                    label=lab(m) if i == 0 else None)
    ax.set_aspect("equal"); ax.set_xlim(-hx - 30, hx + 30); ax.set_ylim(-hy - 30, hy + 30)
    ax.set_xlabel("x off the tray centre (mm)"); ax.set_ylabel("y off the tray centre (mm)"); ax.grid(alpha=0.3)
    ax.set_title("Payload at rest (filled: flat, open: tilted)", fontsize=9)
    ax.legend(loc="upper right", fontsize=8, frameon=False)
    p = os.path.join(figdir, "sim_pick_drop_outcome.png"); fig.savefig(p, dpi=220); plt.close(fig); out["outcome"] = p
    return out


def write_readme(rows, summary, summary_std, PH, T_common, all_R, att):
    from readme_text import _md_table
    cfg_dir = os.path.join(PND_MAT, "configs"); os.makedirs(cfg_dir, exist_ok=True)
    for dst, src in FLOWN_CONFIGS.items():
        shutil.copy2(os.path.join(AUTOPILOT_CONFIG, src), os.path.join(cfg_dir, dst))
    plant = []
    for line in open(os.path.join(AUTOPILOT_CONFIG, FLOWN_CONFIGS["whole_body_l1_4d_sim_pick_and_drop.yaml"])):
        mm = re.match(r"^\s*(sim_[a-z0-9_]+):\s*([^#\n]*)", line)
        if mm:
            plant.append(f"{mm.group(1)}: {mm.group(2).strip()}")
    sd_by = {x["method"]: x for x in summary_std}
    pm = lambda m, k, f="%.1f": (f % next(x for x in summary if x["method"] == m)[k]) + (  # noqa: E731
        (" ± " + f % sd_by[m][k]) if sd_by[m]["runs"] > 1 else "")
    run_rows = [[SHORT[r["method"]], f"`{r['name']}.mat`", f"{r['eps_uav_max_mm']:.1f}", f"{r['eps_uav_rms_mm']:.1f}",
                 f"{r['ee_pos_mm']:.1f}", f"{r['claw_err_at_release_mm']:.1f}", f"{r['rest_off_centre_mm']:.0f}",
                 f"{r['rest_tilt_deg']:.1f}", "flat" if r["flat"] else ("in, tilted" if r["in_tray"] else "out")]
                for r in sorted(rows, key=lambda x: (ORDER.index(x["method"]), x["run"]))]
    mean_rows = [[SHORT[x["method"]], f"{x['runs']} of {x['attempts']}", f"{x['flat']}", pm(x["method"], "eps_uav_max_mm"),
                  pm(x["method"], "eps_uav_rms_mm"), pm(x["method"], "ee_pos_mm"), pm(x["method"], "claw_err_at_release_mm"),
                  pm(x["method"], "rest_off_centre_mm", "%.0f"), pm(x["method"], "rest_tilt_deg")] for x in summary]
    att_by = {}
    for x in att:
        att_by.setdefault(x["method"], []).append(x)
    att_rows = [[SHORT.get(m, m), str(len(v)), str(sum(1 for y in v if y["status"] == "completed")),
                 "; ".join(f"attempt {y['run']}.{y['attempt']}: {y['why']}" for y in v if y["status"] != "completed") or "--"]
                for m, v in att_by.items()]
    first = min((x["time"] for x in att), default="?")[:16].replace("T", " ")
    last = max((x["time"] for x in att), default="?")[:16].replace("T", " ")
    what = {"go_to_start": "fly from the hover to the start waypoint; hover",
            "pick": "approach above the fin, hover 2 s (descent trim), descend, close the gripper on the fin, lift at once (exit)",
            "to_drop_start": "carry the payload to the drop-start waypoint; hover there until the slot (>= 6 s: the planner "
                             "measures the hanging payload and aims the drop at it)",
            "drop": "Ready To Drop to the release point above the tray, hover 2 s, open (the payload falls), back out and climb",
            "to_land_start": "fly to the land-start waypoint, the gripper closing and the arm folding; hover",
            "land": "the approach to the landing point"}
    ph_rows = [[str(i + 1), PHASE_LABEL[k], f"{PH[k][0]:.2f}", f"{PH[k][1]:.2f}", f"{PH[k][1] - PH[k][0]:.2f}", what[k]]
               for i, k in enumerate(PHASE_KEYS)]
    lift = [R["phases"]["to_drop_start"][0] - R["t0"] for R in all_R.values()]
    L = [
        "# Simulation results, MATLAB data (payload pick-and-drop)", "",
        "The data behind the pick-and-drop comparison (`tab:sim_pick_drop`). Self-contained: these files plus MATLAB.",
        "The same `run` struct layout and notation as `pick_and_place/` and `free_flight_tracking/` beside it,",
        "plus the drop's own measures under `run.metrics.drop`.", "",
        "## Files", "",
        "```",
        "<method>_pnd_run<k>.mat          one run, struct `run`",
        "pick_and_drop_metrics.mat        metrics_mean / metrics_std = the table (mean / sample std over the kept runs),",
        "                                 metrics_runs = every kept run, metrics_phase = every run x phase,",
        "                                 phases = the common phase table, attempts = every attempt flown,",
        "                                 window_s, L_arm_m, tray (centre, inner half-size, rim height)",
        "configs/                         the three controllers' yamls as flown (the decoupled one flew the",
        "                                 planner block of the whole-body file; the MAC file carries it verbatim)",
        "reference_outputs/               the Python table + figures, to check MATLAB output against",
        "```", "",
        "```matlab",
        "S = load('pick_and_drop_metrics.mat');  P = S.phases;",
        "r = load('whole_body_l1_pnd_run1.mat').run;",
        "w = r.tracking.in_window > 0;  plot(r.tracking.t(w), 1e3 * r.tracking.eps_uav_m(w))   % ||eps_UAV|| [mm]",
        "T = struct2table(S.metrics_mean);   D = r.metrics.drop;                                % the table; this run's drop",
        "```", "",
        "## Results", "",
        f"Flown {first} to {last} (attempt end times). Mean ± sample std over the kept runs:", "",
        _md_table(["method", "success", "flat", "eps max [mm]", "eps rms [mm]", "EE RMSE [mm]", "claw at release [mm]",
                   "rest off centre [mm]", "rest tilt [deg]"], mean_rows), "",
        "Every kept run:", "",
        _md_table(["method", "file", "eps max [mm]", "eps rms [mm]", "EE RMSE [mm]", "claw at release [mm]",
                   "rest off centre [mm]", "rest tilt [deg]", "payload"], run_rows), "",
        "Attempts (a run is kept if it completed, the payload came to rest inside the tray, every step was on its",
        "timetable slot, and the scene banner showed the 200 g payload and the arm-bridge drain):", "",
        _md_table(["method", "attempts", "kept", "why the others were not kept"], att_rows) if att_rows else "--", "",
        "## Timing", "",
        "The driver flies the DROP timetable (`pnp_mission_v2.py --drop-place --timetable`): the clamped payload is",
        "lifted as soon as the jaws close (a clamp on a payload still standing on its pillar is a closed chain that",
        "drifts), so the end of the pick phase varies "
        f"({min(lift):.1f} to {max(lift):.1f} s); Ready To Drop starts at a fixed mission time, so every later phase",
        "starts at the same time on every run. The phase table is the mean over the runs:", "",
        _md_table(["#", "phase", "start [s]", "end [s]", "duration [s]", "what happens"], ph_rows), "",
        f"- **Window:** every metric covers the whole mission, 0 to {T_common:.2f} s; `run.tracking.in_window` marks it,",
        "  `run.tracking.phase_id` (1..6) the phase.",
        "- **Release:** 2 s after the Ready To Drop leg ends, by the driver, on every run (`run.meta.t_release_s`).", "",
        "## Scene and task", "",
        "- **Field:** two 0.77 m pillars with the printed hat, PICK at (1, 1) m, PLACE at (-1, -1) m.",
        "- **Payload:** the experiment's Box_Assembly (basket, holder fin, mocap markers), **200 g**, on the PICK hat.",
        "- **Grasp:** a clamp on the holder's fin (the simulated jaws close on a 20 mm grip pad around it).",
        "- **Drop:** Ready To Drop straight to the release point (the payload's rest pose on the tray cover + the drop",
        "  margin), open, back out and climb. The tray: 371 x 290 mm, 35 mm walls, turned 90 deg, on the PLACE hat",
        "  (rim 0.813 m). Flat = resting on the tray floor with tilt < 5 deg.",
        "- **Arm pose:** pick = carry = drop [0, 32, 38, 0] deg.",
        "- **Controllers:** Proposed = whole-body 4-D L1 (`whole_body_l1_4d_sim_pick_and_drop.yaml`); Geo-L1 = the",
        "  decoupled geometric + L1 rig with a position-mode arm, its pick-and-place gains",
        "  (`geometric_l1_sim_pick_and_place.yaml`); MAC = modular adaptive (`modular_adaptive_sim_pick_and_drop.yaml`).",
        "- **Simulator:** headless Isaac at real time, raw mocap emulator, and the arm bridge DRAINED every physics",
        "  step (`PEGASUS_ARM_SPIN_DRAIN=1`, 2026-10-26: before it the applied arm torque could lag the command",
        "  ~40 ms, which made MAC unstable; `docs/docs_aerial_manipulator/archive/pick_place_mac_20261026`).",
        "- **Plant:** every `sim_*` key as flown:", "",
        "```", *plant, "```", "",
        "## The struct `run`", "",
        _md_table(["field", "content"], [
            ["`meta`", "method, run index, scene, plant, `phases`, `timetable_slots_s`, `late_steps`, `window`, `L_arm_m`, "
                       "`t0_driver_s`, `t_release_s`, `tray`"],
            ["`metrics`", "`window` = this run's table numbers; `per_phase.<key>`; `whole_recording`; `drop` = "
                          "claw_err_at_release_mm (+ xyz, target − claw), release_drop_height_mm (payload bottom over the rim), "
                          "release_off_centre_mm, rest_off_centre_mm (+ xy), rest_tilt_deg, in_tray, flat, lurch_after_release_mm, rest_pos_m"],
            ["`tracking`", "100 Hz, `t` in s since the start of the mission: measured, reference (`_ref_`) and error (`_err_`) "
                           "for the platform, `eps_uav_m`, `rho_uav`, attitude, CoM, end effector and joints"],
            ["`raw`", "every stream on the driver clock (subtract `meta.t0_driver_s`): `odom`, `joints`, `reference`, "
                      "`payload` (ground truth), `claw_truth`, `marks`, `events`"]]), "",
        "Notation and units as in `pick_and_place/README.md`: positions in m (world ENU) unless the name says mm,",
        "angles in deg unless it says rad; errors are measured − reference.", ""]
    with open(os.path.join(PND_MAT, "README.md"), "w") as f:
        f.write("\n".join(L))
    print("wrote", os.path.join(PND_MAT, "README.md"))


if __name__ == "__main__":
    main()
