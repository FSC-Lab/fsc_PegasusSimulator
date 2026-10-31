#!/usr/bin/env python3
"""Score the VALVE-TURNING simulation campaign and export it for MATLAB (2026-10-17).

    PYTHONNOUSERSITE=1 /usr/bin/python3 results/utils/build_valve_turning.py [--paper <main.tex>]

The box-pushing builder's twin (build_box_pushing.py: its helpers, phase machinery, metric
definitions, impedance scorer and table / figure / README layout are IMPORTED, not copied), for the
push-and-pull mode's TURN form: the whole-body 4-D L1 controller grasps the fin of the user's
bearing-mounted valve handle (Valve_Assembly, archive/valve_turning_20261015) and turns the beam one
full revolution, the claw world-held on its 0.160 m circle, the arm held, the law in CONTACT.
Reads results/simulation_results/Isaac_simulation_data/valve_turning/<method>_turn_run<k>.npz (+ .yaml + .args, from
run_valve_turning_campaign.sh) and writes
  results/utils/tables/valve_turning_sim.{csv,json,tex}
  results/simulation_results/Isaac_simulation_data/valve_turning/figures/sim_valve_turn_*.png
  results/simulation_results/matlab_simulation_data/valve_turning/<run>.mat, valve_turning_metrics.mat,
      README.md, configs/, reference_outputs/
What differs from the box push, and why:
  * THE BEAM replaces the box: the metric is the rotation turned during the push (mocap obj_0 = the
    rotor, its yaw = the beam's; the rotor's origin is the fixed bearing axis), the follow-through after
    the release, the claw's radius about the axis during the turn, and the arm's margin to its stops
    in the turn (the binding constraint of this task: the base leads its circle at the fast half and
    the world-held arm absorbs it).
  * THE CONTACT SIGN: pl_metrics calibrates the PhysX report's sign on the table's normal force on the
    box and the box's velocity; the rotor touches no table and its origin never moves, so the valve
    builder calibrates it itself -- the vehicle does positive work on a friction-loaded rotor, so the
    one sign (the scene reports both parts in one convention) is the one that makes the mean power of
    the contact force on the claw's velocity positive over the turn.
  * THE WINDOWS keep pl_metrics' names (SLIDE = the move after the 1.5 s settle) -- here the move is
    the 36 s turn; the README says so.
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
sys.path.insert(0, HERE)
import build_box_pushing as BB   # noqa: E402  (the shared machinery; its module code only sets constants)

VT = os.path.join(BB.ROOT, "valve_turning")
VT_MAT = os.path.join(BB.MAT_DIR, "valve_turning")
NAME = re.compile(r"^(whole_body_l1|coordinated_admittance)_turn_run(\d+)$")
CONFIG_NAME = {"whole_body_l1": "whole_body_l1_4d_sim_valve_turning.yaml",
               "coordinated_admittance": "coordinated_admittance_sim_valve_turning.yaml"}
PHASES = BB.PHASES          # the eight contiguous phases of the push-and-pull mode (push = the turn)
PHASE_KEYS, PHASE_LABEL, LAST = BB.PHASE_KEYS, BB.PHASE_LABEL, BB.LAST
PHASE_LABEL = dict(PHASE_LABEL); PHASE_LABEL["push"] = "Turn"
METHODS, ORDER, COLORS = BB.METHODS, BB.ORDER, BB.COLORS
IMP_WINDOWS, G, L_ARM, DT, BASE_COM, FC_HZ, SLOT_NAMES = (BB.IMP_WINDOWS, BB.G, BB.L_ARM, BB.DT, BB.BASE_COM,
                                                          BB.FC_HZ, BB.SLOT_NAMES)
interp, wrap, quat_cont, build_r0, rms, yaw_of, colstruct, metrics, imp_metrics = (
    BB.interp, BB.wrap, BB.quat_cont, BB.build_r0, BB.rms, BB.yaw_of, BB.colstruct, BB.metrics, BB.imp_metrics)
Q2_STOP, Q3_STOP = -20.0, 50.0       # [deg] the arm's stops the turn runs into (the real arm's q2 range, the asset's q3)
# The valve is reported for the proposed controller alone for now (user, 2026-10-18): False keeps the README's
# attempts / failed-attempts tables to the methods that have kept runs in the folder. The comparison rig's raw
# attempts stay in valve_turning/failed/ and campaign.jsonl; set True to list them again.
COMPARISON = False


def scene_text(T):
    return (f"07 valve turning: the HOOBRO console table (1.000 x 0.150 x 0.800 m) at x {T['table_xmin']:.3f}-{T['table_xmax']:.3f}, "
            f"y {T['table_ymin']:g}..{T['table_ymax']:g} m, top {T['table_top_z']:.2f} m; the valve mechanism (Valve_Assembly: a 608 "
            f"bearing, a 225 mm beam with a 5 x 40 x 130 mm fin at radius {T['fin_radius_m']:.3f} m) bolted to it, its axis at "
            f"({T['axis_xy'][0]:.2f}, {T['axis_xy'][1]:.2f}); the rotor {1e3 * T['rotor_mass_kg']:.0f} g, bearing Coulomb friction "
            f"{T['friction_nm']:g} N.m + viscous {T['damping_nms']:g} N.m.s/rad; T650 landing gear; the real arm's q2 range [-20, 45] deg")


def task_text(T):
    return (f"grasp the fin and turn the beam {T['turn_deg']:g} deg about the bearing axis in {T['push_time_s']:g} s "
            f"(after a {T['settle_s']:g} s settle), the arm held at [{', '.join(f'{v:g}' for v in T['pose_deg'])}] deg, "
            "the claw world-held on its circle, the law in CONTACT from the grip to the end of the turn")


def discover():
    out = []
    for p in sorted(glob.glob(os.path.join(VT, "*_turn_run*.npz"))):
        m = NAME.match(os.path.basename(p)[:-4])
        if m:
            out.append((m.group(1), int(m.group(2)), p))
    return out


def task_of(ypath, apath):
    """The scene / task numbers a run flew: the planner's push_pull_* block of its yaml and try_valve.sh's
    .args (FRICTION, DAMPING, ROTOR_MASS = the scene knobs; the overrides)."""
    T = BB.task_of(ypath, apath)
    pp = T["planner"]
    args = {}
    if os.path.isfile(apath):
        for line in open(apath):
            if "=" in line:
                k, v = line.strip().split("=", 1)
                args[k] = v
    obs = pp.get("obstacle", [1.0, 1.4, -0.2, 0.2, 0.854])
    ee = pp.get("ee_offset", [0.160, 0.0006, 0.179])
    T.update(turn_deg=float(pp.get("turn_deg", 360.0)), fin_radius_m=float(math.hypot(ee[0], ee[1])),
             grasp_height_m=float(ee[2]), axis_xy=[0.5 * (obs[0] + obs[1]), 0.5 * (obs[2] + obs[3])],
             beam_top_z=float(obs[4]), friction_nm=float(args.get("FRICTION", 0.048)),
             damping_nms=float(args.get("DAMPING", 0.0)), rotor_mass_kg=float(args.get("ROTOR_MASS", 0.204)),
             contact_phase=str(pp.get("contact", "true")).lower() != "false",
             wd_tilt_deg=next((float(m.group(1)) for m in (re.match(r"^\s*system_wd_max_tilt_deg:\s*([-0-9.eE+]+)", l)
                                                          for l in open(ypath)) if m), float("nan")),
             overrides=[l.strip() for l in open(apath) if "=" in l and not l.split("=")[0].strip().isupper()] if os.path.isfile(apath) else [])
    return T


def safe_metrics(R, mask):
    """metrics() over a phase that may hold no sample inside the window (the sub-second landing leg):
    NaN instead of a crash."""
    if mask.any():
        return metrics(R, mask)
    any_ = metrics(R, np.ones(len(R["tu"]), bool))
    return {k: (float("nan") if not isinstance(v, list) else [float("nan")] * len(v)) for k, v in any_.items()} | {"window_s": 0.0}


def contact_sign(d, marks):
    """One sign for the PhysX report's two parts: the one that makes the vehicle's mean power on the rotor
    POSITIVE over the turn (the claw drives a friction-loaded bearing)."""
    ct, claw = d["contact"], d["claw"]
    a_, b_ = marks["push:start"] + 1.5 + 2.0, marks["push:end"]
    w = (ct[:, 0] > a_) & (ct[:, 0] < b_)
    F = ct[w, 1:4] + ct[w, 4:7]
    v = np.gradient(interp(ct[w, 0], claw[:, 0], claw[:, 1:4]), ct[w, 0], axis=0)
    power = float(np.nanmean(np.sum(F * v, axis=1)))
    return (1.0, 1.0) if power > 0 else (-1.0, -1.0), power


def load_run(path):
    from scipy.spatial.transform import Rotation as Rot
    from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
    import pl_metrics as PM
    RZ90 = TP._Rz(0.5 * math.pi); RZM90 = TP._Rz(-0.5 * math.pi)
    d = np.load(path, allow_pickle=True)
    marks = {str(n): float(t) for t, n in zip(d["marks_t"], d["marks_name"])}
    od, ref, js = d["odom"], d["ref"], d["joints"]
    if ref.shape[1] < 20:
        raise SystemExit(f"{path}: the reference stream lacks x_cd_ddot / b1_de (driver before 2026-10-09) -- re-fly")
    if "timetable" not in d.files or d["timetable"].size == 0:
        raise SystemExit(f"{path}: flown without --timetable -- re-fly")
    names = [str(n) for n in d["joint_names"]]
    order = [names.index(f"joint{i}") for i in range(1, 5)] if all(f"joint{i}" in names for i in range(1, 5)) else [0, 1, 2, 3]
    phases = {}
    for key, _, a_, b_ in PHASES:
        if a_ not in marks or b_ not in marks:
            raise SystemExit(f"{path}: no mark {a_} / {b_} -- not a complete mission")
        phases[key] = (marks[a_], marks[b_])
    late = [(str(n), float(x)) for n, x in zip(d["late_name"], d["late_s"])] if "late_name" in d.files else []
    params = TP.make_params_t650(base_com=BASE_COM)
    t0 = phases["go_to_start"][0]
    tu = np.arange(t0, phases[LAST][1] + DT, DT)
    n = len(tu)
    P = interp(tu, od[:, 0], od[:, 1:4])
    Qwxyz = quat_cont(od[:, 7:11]); Qxyzw = Qwxyz[:, [1, 2, 3, 0]]
    Qu = interp(tu, od[:, 0], Qxyzw); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    Ra = Rot.from_quat(Qu).as_matrix()
    q = interp(tu, js[:, 0], js[:, 1:5][:, order])
    r_ed = interp(tu, ref[:, 0], ref[:, 1:4]); x_cd = interp(tu, ref[:, 0], ref[:, 4:7])
    q_d = interp(tu, ref[:, 0], ref[:, 7:11])
    b1d = np.column_stack([interp(tu, ref[:, 0], ref[:, 11:13]), np.zeros(n)])
    x_cd_dd = interp(tu, ref[:, 0], ref[:, 13:16]); b1de = interp(tu, ref[:, 0], ref[:, 16:19])
    cadm = d["cadm"] if "cadm" in d.files else np.zeros((0, 0))
    coordinated = cadm.size > 1 and cadm.shape[1] >= 33
    if coordinated:
        from fsc_aerial_manipulation.robotic_arm.utils_controller import coordinated_admittance as CA
        xb_cmd = interp(tu, cadm[:, 0], cadm[:, 26:29])
        q_d = interp(tu, cadm[:, 0], cadm[:, 29:33])
        eta_d = interp(tu, d["dbg"][:, 0], d["dbg"][:, 31:34])
    xb = np.zeros((n, 3)); Rref = np.zeros((n, 3, 3)); xc = np.zeros((n, 3)); re = np.zeros((n, 3)); b1e = np.zeros((n, 3))
    for i in range(n):
        if coordinated:
            xb[i] = xb_cmd[i]
            Rref[i] = CA.euler_to_r(eta_d[i])
        else:
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
    ypath = path[:-4] + ".yaml"
    if not os.path.isfile(ypath):
        raise SystemExit(f"{path}: no {os.path.basename(ypath)} beside it (the impedance gains)")
    gains = PM.yaml_gains(ypath)
    task = task_of(ypath, path[:-4] + ".args")
    sign, power = contact_sign(d, marks)
    imp = PM.analyse(path, gains, FC_HZ, sign=sign)
    # ---- the valve: the rotor's yaw = the beam's angle; its origin = the fixed axis ----
    box = d["box"]
    axis = box[0, 1:4]
    beam = np.degrees(np.unwrap(np.radians(yaw_of(box[:, 4:8]))))
    def beam_at(t):
        return float(np.interp(t, box[:, 0], beam))
    t_ps, t_pe = marks["push:start"], marks["push:end"]
    turned = beam_at(t_pe) - beam_at(t_ps)
    rate = np.gradient(beam, box[:, 0])
    wt = (box[:, 0] >= t_ps) & (box[:, 0] <= t_pe)
    # the planned beam angle: the EE reference's azimuth about the axis
    az_plan = np.degrees(np.unwrap(np.arctan2(r_ed[:, 1] - axis[1], r_ed[:, 0] - axis[0])))
    beam_u = interp(tu, box[:, 0], beam[:, None])[:, 0]
    k0 = int(np.searchsorted(tu, t_ps))
    beam_err = (beam_u - beam_u[k0]) - (az_plan - az_plan[k0])          # [deg] beam minus plan, from the push start
    claw = d["claw"]
    wc = (claw[:, 0] >= t_ps + 1.5) & (claw[:, 0] <= t_pe)
    radius = np.linalg.norm(claw[wc, 1:3] - axis[:2], axis=1)
    mturn = (tu >= t_ps) & (tu <= t_pe)
    qd = np.degrees(q[mturn])
    valvem = dict(turned_deg=float(turned), commanded_deg=float(task["turn_deg"]),
                  follow_through_deg=float(beam_at(marks["exit:end"]) - beam_at(t_pe)),
                  beam_rate_max_dps=float(np.abs(rate[wt]).max()) if wt.any() else float("nan"),
                  beam_plan_err_rms_deg=float(rms(beam_err[mturn])), beam_plan_err_max_deg=float(np.abs(beam_err[mturn]).max()),
                  claw_radius_mean_m=float(radius.mean()), claw_radius_std_mm=float(1e3 * radius.std()),
                  claw_radius_planned_m=float(task["fin_radius_m"]),
                  q2_min_turn_deg=float(qd[:, 1].min()), q3_max_turn_deg=float(qd[:, 2].max()),
                  q2_margin_turn_deg=float(qd[:, 1].min() - Q2_STOP), q3_margin_turn_deg=float(Q3_STOP - qd[:, 2].max()),
                  on_stop_fraction=float(np.mean((qd[:, 1] <= Q2_STOP + 0.5) | (qd[:, 2] >= Q3_STOP - 0.5))),
                  axis_m=axis.tolist(), axis_moved_mm=float(1e3 * np.linalg.norm(box[:, 1:4] - axis, axis=1).max()),
                  contact_sign=float(sign[0]), contact_power_w=float(power))
    return dict(d=d, marks=marks, phases=phases, timetable=np.asarray(d["timetable"], float), late=late, t0=t0, tu=tu, P=P, Qu=Qu, q=q, q_d=q_d,
                xb=xb, x_cd=x_cd, xc=xc, r_ed=r_ed, re=re, att_err=att_err, az=az, az_ref=az_ref,
                ee_head_err=ee_head_err, e_pos=e_pos, e_ee=e_ee, e_q=e_q, eps=eps, tilt=tilt, yaw=yaw,
                yaw_ref=yaw_ref, box=box, valvem=valvem, beam_u=beam_u, beam_err=beam_err, az_plan=az_plan,
                od=od, ref=ref, js=js, order=order, imp=imp, gains=gains, yaml=ypath, coordinated=coordinated, task=task)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--paper", default="")
    ap.add_argument("--figdir", default=os.path.join(VT, "figures"))
    a = ap.parse_args()
    import scipy.io
    runs = discover()
    if not runs:
        sys.exit(f"no runs in {VT}")
    os.makedirs(VT_MAT, exist_ok=True)
    os.makedirs(BB.TABLES, exist_ok=True); os.makedirs(a.figdir, exist_ok=True)
    rows, loaded = [], {}
    all_R = {(m, k): load_run(p) for m, k, p in runs}
    for (m, k), R in all_R.items():
        if R["late"]:
            print(f"WARNING {m} run {k}: LATE steps {R['late']} -- its phases are not on the timetable")
    by_method = {}
    for (m_, k_), R in all_R.items():
        by_method.setdefault(m_, []).append(R)
    ref_m = next(m_ for m_ in ORDER if m_ in by_method)
    ready_ref = float(np.mean([R["phases"]["ready"][0] - R["t0"] for R in by_method[ref_m]]))
    for m_, Rs in by_method.items():
        ready_m = float(np.mean([R["phases"]["ready"][0] - R["t0"] for R in Rs]))
        for R in Rs:
            R["t_shift"] = ready_m - ready_ref
            R["t0"] += R["t_shift"]
            R["t0_driver"] = R["t0"] - R["t_shift"]
    rel = {key: np.array([R["phases"][key][0] - R["t0"] for R in all_R.values()]) for key in PHASE_KEYS}
    spread = {key: float(v.max() - v.min()) for key, v in rel.items()}
    print("phase start [s] mean / spread across runs (methods aligned at Ready):",
          {key: (round(float(rel[key].mean()), 2), round(spread[key], 3)) for key in PHASE_KEYS})
    chk = {k_: v_ for k_, v_ in spread.items() if k_ != "go_to_start"}
    if max(chk.values()) > BB.PHASE_ALIGN_TOL_S:
        sys.exit(f"phase starts differ by up to {max(chk.values()):.2f} s across runs (> {BB.PHASE_ALIGN_TOL_S}) -- "
                 "not one timetable from Ready on; re-fly or check the slots")
    T_common = min(R["phases"][LAST][1] - R["t0"] for R in all_R.values())
    PH = {key: (float(rel[key].mean()), None) for key in PHASE_KEYS}
    for i, key in enumerate(PHASE_KEYS):
        PH[key] = (PH[key][0], PH[PHASE_KEYS[i + 1]][0] if i + 1 < len(PHASE_KEYS) else T_common)
    # the landing leg is under a second here and the shortest run can end before the mean land start:
    # a phase never ends before it starts (the window clips it)
    PH = {k: (min(a_, T_common), max(min(b_, T_common), min(a_, T_common))) for k, (a_, b_) in PH.items()}
    print(f"window: the whole mission, 0 .. {T_common:.2f} s (the shortest run); phases",
          {k: (round(a_, 2), round(b_, 2)) for k, (a_, b_) in PH.items()})
    for method, k, path in runs:
        R = all_R[(method, k)]
        name = f"{method}_turn_run{k}"
        tu = R["tu"]
        t_rel = tu - R["t0"]
        fixed = (t_rel >= -1e-9) & (t_rel <= T_common + 1e-9)
        phase_id = np.zeros(len(tu))
        per_leg = {}
        for i, key in enumerate(PHASE_KEYS):
            s0, e0 = R["phases"][key]
            m = (tu >= s0) & (tu < e0) & fixed
            phase_id[m] = i + 1
            per_leg[key] = dict(label=PHASE_LABEL[key], start_s=max(s0 - R["t0"], 0.0), end_s=min(e0 - R["t0"], T_common),
                                **safe_metrics(R, m))
        whole = np.ones(len(tu), bool)
        r = metrics(R, fixed)
        r_whole = metrics(R, whole)
        r_whole["window_s"] = float(whole.sum() * DT)
        R["phase_id"] = phase_id
        im = imp_metrics(R["imp"])
        V = R["valvem"]
        row = dict(name=name, method=method, run=k, total_window_s=r["window_s"], **r,
                   e_imp_slide_rms_n=im["SLIDE"]["e_imp_rms_n"], e_imp_slide_max_n=im["SLIDE"]["e_imp_max_n"],
                   e_imp_contact_rms_n=im["CONTACT"]["e_imp_rms_n"], e_imp_contact_max_n=im["CONTACT"]["e_imp_max_n"],
                   e_imp_hold_rms_n=im["HOLD"]["e_imp_rms_n"], e_imp_release_rms_n=im["RELEASE"]["e_imp_rms_n"],
                   e_imp_slide_psi_rms_nm=im["SLIDE"]["e_imp_psi_rms_nm"],
                   F_ext_slide_rms_n=im["SLIDE"]["F_ext_rms_n"], e_y_slide_rms_mm=im["SLIDE"]["e_y_rms_mm"],
                   beam_turned_deg=V["turned_deg"], beam_follow_through_deg=V["follow_through_deg"],
                   beam_plan_err_rms_deg=V["beam_plan_err_rms_deg"], beam_plan_err_max_deg=V["beam_plan_err_max_deg"],
                   claw_radius_mean_m=V["claw_radius_mean_m"], claw_radius_std_mm=V["claw_radius_std_mm"],
                   q2_margin_turn_deg=V["q2_margin_turn_deg"], q3_margin_turn_deg=V["q3_margin_turn_deg"],
                   on_stop_fraction=V["on_stop_fraction"],
                   **BB.release_margin(R),
                   whole_mission=r_whole, per_leg=per_leg, impedance=im)
        rows.append(row); loaded[name] = R
        print(f"{name:28s} win {r['window_s']:5.1f} s  eps max/rms {r['eps_uav_max_mm']:6.1f}/{r['eps_uav_rms_mm']:5.1f} mm "
              f"rho {r['rho_uav_max']:.3f}/{r['rho_uav_rms']:.3f}  EE {r['ee_pos_mm']:5.1f} mm head {r['ee_heading_deg']:.2f}  "
              f"e_imp slide {row['e_imp_slide_rms_n']:.3f} N (sign {V['contact_sign']:+.0f}, {V['contact_power_w'] * 1e3:.1f} mW)  "
              f"beam {V['turned_deg']:.1f} deg  stops q2 {V['q2_margin_turn_deg']:.1f} / q3 {V['q3_margin_turn_deg']:.1f} deg")
        d = R["d"]
        ser = R["imp"]["series"]
        T = R["task"]
        meta = dict(name=name, method=method, method_long=METHODS[method][1], run_index=k,
                    plant="Isaac Sim, sim-to-real mirror plant (config A), headless at real time (RTF 1), raw mocap feedback",
                    scene=scene_text(T), task=task_text(T),
                    task_values=dict(turn_deg=T["turn_deg"], turn_time_s=T["push_time_s"], settle_s=T["settle_s"],
                                     fin_radius_m=T["fin_radius_m"], grasp_height_above_table_m=T["grasp_height_m"],
                                     axis_xy_m=np.array(T["axis_xy"], float), beam_top_z_m=T["beam_top_z"],
                                     rotor_mass_kg=T["rotor_mass_kg"], bearing_friction_nm=T["friction_nm"],
                                     bearing_damping_nms=T["damping_nms"], contact_phase_flag=T["contact_phase"],
                                     turn_pose_deg=np.array(T["pose_deg"], float),
                                     table_x_m=np.array([T["table_xmin"], T["table_xmax"]]),
                                     table_y_m=np.array([T["table_ymin"], T["table_ymax"]]), table_top_z_m=T["table_top_z"],
                                     land_start_xyz_yawdeg=np.array(T["land_start"], float) if T["land_start"] else np.zeros(0),
                                     land_xyz_yawdeg=np.array(T["land"], float),
                                     overrides=np.array(T["overrides"], dtype=object)),
                    phases=dict(key=np.array(PHASE_KEYS, dtype=object),
                                label=np.array([PHASE_LABEL[x] for x in PHASE_KEYS], dtype=object),
                                start_s=np.array([R["phases"][x][0] - R["t0"] for x in PHASE_KEYS]),
                                end_s=np.array([R["phases"][x][1] - R["t0"] for x in PHASE_KEYS])),
                    timetable_slots_s={nm: float(x) for nm, x in zip(SLOT_NAMES, R["timetable"])},
                    late_steps=np.array([f"{n} {x:.2f} s" for n, x in R["late"]], dtype=object),
                    window=f"the whole mission, t = 0 (start of go to start) to {T_common:.2f} s (end of land, shortest run)",
                    impedance_windows=dict(CONTACT="ready end -> exit start (the law in contact)",
                                           HOLD="ready end -> turn start (the grip and the 1.5 s settle)",
                                           SLIDE=f"the {T['push_time_s']:g} s turn", RELEASE="turn end -> exit start"),
                    impedance_gains=dict(M_d=R["gains"]["my"], D_d=R["gains"]["dy"], K_d=R["gains"]["ky"],
                                         channels="x y z [N] heading [N.m]", lowpass_hz=FC_HZ),
                    contact_sign=dict(value=V["contact_sign"], mean_power_w=V["contact_power_w"],
                                      rule="the sign making the vehicle's mean power on the rotor positive over the turn"),
                    L_arm_m=L_ARM, base_com_model_m=BASE_COM, aborted=bool(d["aborted"]), t0_driver_s=R["t0"],
                    t_shift_s=float(R.get("t_shift", 0.0)),
                    clock="driver clock [s] (raw streams); tracking.t / impedance.t are s since the start of go to start")
        ct = d["contact"]
        raw = dict(odom=dict(t=R["od"][:, 0], pos_m=R["od"][:, 1:4], vel_mps=R["od"][:, 4:7], quat_wxyz=R["od"][:, 7:11],
                             direct_mode=R["od"][:, 11], step=R["od"][:, 12]),
                   joints=dict(t=R["js"][:, 0], q_rad=R["js"][:, 1:5][:, R["order"]], effort=R["js"][:, 5:9][:, R["order"]]),
                   reference=dict(t=R["ref"][:, 0], r_ed=R["ref"][:, 1:4], x_cd=R["ref"][:, 4:7], q_d_rad=R["ref"][:, 7:11],
                                  b1_d_model_xy=R["ref"][:, 11:13], x_cd_ddot=R["ref"][:, 13:16], b1_de=R["ref"][:, 16:19]),
                   reference_derivatives=dict(t=d["refd"][:, 0], r_ed_dot=d["refd"][:, 1:4], r_ed_ddot=d["refd"][:, 4:7]),
                   rotor=dict(t=R["box"][:, 0], axis_pos_m=R["box"][:, 1:4], quat_wxyz=R["box"][:, 4:8],
                              beam_angle_deg=np.degrees(np.unwrap(np.radians(yaw_of(R["box"][:, 4:8])))),
                              note="mocap obj_0 = the rotor: its origin the bearing axis on the table top (fixed), its yaw the beam's"),
                   claw_truth=dict(t=d["claw"][:, 0], pos_m=d["claw"][:, 1:4]),
                   base_truth=dict(t=d["base_truth"][:, 0], pos_m=d["base_truth"][:, 1:4], quat_wxyz=d["base_truth"][:, 4:8]),
                   contact_truth=dict(t=ct[:, 0], vehicle_normal_n=ct[:, 1:4], vehicle_friction_n=ct[:, 4:7],
                                      vehicle_moment_normal_nm=ct[:, 13:16], vehicle_moment_friction_nm=ct[:, 16:19],
                                      note="PhysX contact impulses on the rotor averaged over each publish interval, one sign "
                                           "convention for both parts; meta.contact_sign is the sign that makes them the force "
                                           "ON the rotor BY the vehicle"),
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
                        beam_angle_deg=R["beam_u"], beam_angle_plan_deg=R["az_plan"], beam_minus_plan_deg=R["beam_err"],
                        columns="platform_att_err_deg = [roll pitch heading] (body axes); xyz columns are world ENU; "
                                "beam_angle_plan_deg = the EE reference's azimuth about the axis (beam_minus_plan_deg is zeroed at the turn's start)")
        impedance = dict(t=ser["t"] - R["t0"], e_imp=ser["e_imp"], F_ext=ser["F"], rendered=ser["rendered"], e_y=ser["e_y"],
                         columns="e_imp / F_ext / rendered: [x y z heading] in [N N N N.m]; e_y: [m m m sin(heading error)]; "
                                 "every term low-passed (zero phase) at impedance_gains.lowpass_hz; t covers the CONTACT window",
                         windows=dict(name=np.array(list(im.keys()), dtype=object),
                                      start_s=np.array([R["marks"]["ready:end"] - R["t0"] if w in ("CONTACT", "HOLD") else
                                                        (R["marks"]["push:start"] + 1.5 - R["t0"] if w == "SLIDE" else
                                                         R["marks"]["push:end"] - R["t0"]) for w in im]),
                                      end_s=np.array([R["marks"]["exit:start"] - R["t0"] if w in ("CONTACT", "RELEASE") else
                                                      (R["marks"]["push:start"] + 1.5 - R["t0"] if w == "HOLD" else
                                                       R["marks"]["push:end"] - R["t0"]) for w in im])))
        scipy.io.savemat(os.path.join(VT_MAT, name + ".mat"),
                         {"run": dict(meta=meta, metrics=dict(window=r, whole_recording=r_whole, per_phase=per_leg,
                                                              impedance=im, valve=V),
                                      tracking=tracking, impedance=impedance, raw=raw)},
                         do_compression=True, long_field_names=True, oned_as="column")

    keys = ["eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms", "platform_pos_mm", "platform_heading_deg",
            "platform_roll_deg", "platform_pitch_deg", "ee_pos_mm", "ee_heading_deg", "total_window_s",
            "e_imp_slide_rms_n", "e_imp_slide_max_n", "e_imp_contact_rms_n", "e_imp_contact_max_n", "e_imp_hold_rms_n",
            "e_imp_release_rms_n", "e_imp_slide_psi_rms_nm", "F_ext_slide_rms_n", "e_y_slide_rms_mm",
            "beam_turned_deg", "beam_follow_through_deg", "beam_plan_err_rms_deg", "beam_plan_err_max_deg",
            "claw_radius_mean_m", "claw_radius_std_mm", "q2_margin_turn_deg", "q3_margin_turn_deg", "on_stop_fraction",
            "tilt_max_deg", "release_q2_max_deg", "release_q3_min_deg"]
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
            s[f"e_imp_slide_{ax}_rms_n"] = float(np.mean([r["impedance"]["SLIDE"]["e_imp_xyz_rms_n"][j] for r in rr]))
        for j in range(4):
            s[f"q{j + 1}_deg"] = float(np.mean([r["joints_deg"][j] for r in rr]))
        summary.append(s)
    wins = sorted(set(round(r["total_window_s"], 1) for r in rows))
    print("window per run [s]:", wins, "(must be one value)" if len(wins) > 1 else "")
    with open(os.path.join(BB.TABLES, "valve_turning_sim.json"), "w") as f:
        json.dump(dict(runs=rows, mean_over_runs=summary, L_arm_m=L_ARM), f, indent=1, default=float)
    cols = ["method", "runs", "total_window_s", "eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms",
            "platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
            "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg",
            "q4_deg", "platform_pos_mm", "ee_pos_mm", "e_imp_slide_rms_n", "e_imp_slide_max_n", "e_imp_slide_x_rms_n",
            "e_imp_slide_y_rms_n", "e_imp_slide_z_rms_n", "e_imp_slide_psi_rms_nm", "e_imp_contact_rms_n",
            "e_imp_contact_max_n", "e_imp_hold_rms_n", "e_imp_release_rms_n", "F_ext_slide_rms_n", "e_y_slide_rms_mm",
            "beam_turned_deg", "beam_follow_through_deg", "beam_plan_err_rms_deg", "beam_plan_err_max_deg",
            "claw_radius_mean_m", "claw_radius_std_mm", "q2_margin_turn_deg", "q3_margin_turn_deg", "on_stop_fraction",
            "tilt_max_deg", "release_q2_max_deg", "release_q3_min_deg"]
    with open(os.path.join(BB.TABLES, "valve_turning_sim.csv"), "w") as f:
        f.write(",".join(cols) + "\n")
        for s in summary:
            f.write(",".join(str(s[c]) if isinstance(s[c], (str, int)) else f"{s[c]:.4f}" for c in cols) + "\n")
    ph_rows = [dict(name=r["name"], method=r["method"], run=r["run"], phase=key, label=PHASE_LABEL[key],
                    start_s=r["per_leg"][key]["start_s"], end_s=r["per_leg"][key]["end_s"],
                    eps_uav_max_mm=r["per_leg"][key]["eps_uav_max_mm"], eps_uav_rms_mm=r["per_leg"][key]["eps_uav_rms_mm"],
                    rho_uav_max=r["per_leg"][key]["rho_uav_max"], rho_uav_rms=r["per_leg"][key]["rho_uav_rms"],
                    ee_pos_mm=r["per_leg"][key]["ee_pos_mm"], platform_pos_mm=r["per_leg"][key]["platform_pos_mm"],
                    tilt_max_deg=r["per_leg"][key]["tilt_max_deg"])
               for r in rows for key in PHASE_KEYS]
    imp_rows = [dict(name=r["name"], method=r["method"], run=r["run"], window=w, **{k: v for k, v in r["impedance"][w].items()
                                                                                     if not isinstance(v, list)},
                     e_imp_x_rms_n=r["impedance"][w]["e_imp_xyz_rms_n"][0], e_imp_y_rms_n=r["impedance"][w]["e_imp_xyz_rms_n"][1],
                     e_imp_z_rms_n=r["impedance"][w]["e_imp_xyz_rms_n"][2])
                for r in rows for w in IMP_WINDOWS if w in r["impedance"]]
    flat = []
    for r in rows:
        f = {k: r[k] for k in ["name", "method", "run"] + keys}
        for j, ax in enumerate("xyz"):
            f[f"platform_{ax}_mm"] = r["platform_pos_xyz_mm"][j]; f[f"ee_{ax}_mm"] = r["ee_pos_xyz_mm"][j]
            f[f"e_imp_slide_{ax}_rms_n"] = r["impedance"]["SLIDE"]["e_imp_xyz_rms_n"][j]
        for j in range(4):
            f[f"q{j + 1}_deg"] = r["joints_deg"][j]
        flat.append(f)
    scipy.io.savemat(os.path.join(VT_MAT, "valve_turning_metrics.mat"),
                     {"metrics_runs": colstruct(flat, list(flat[0].keys())),
                      "metrics_mean": colstruct(summary, cols),
                      "metrics_phase": colstruct(ph_rows, list(ph_rows[0].keys())),
                      "metrics_impedance": colstruct(imp_rows, list(imp_rows[0].keys())),
                      "phases": dict(key=np.array(PHASE_KEYS, dtype=object),
                                     label=np.array([PHASE_LABEL[x] for x in PHASE_KEYS], dtype=object),
                                     start_s=np.array([PH[x][0] for x in PHASE_KEYS]),
                                     end_s=np.array([PH[x][1] for x in PHASE_KEYS])),
                      "window_s": T_common, "L_arm_m": L_ARM},
                     do_compression=True, long_field_names=True, oned_as="column")
    tex = latex_table(summary, T_common, next(iter(all_R.values()))["task"])
    open(os.path.join(BB.TABLES, "valve_turning_sim.tex"), "w").write(tex)
    figs = make_figures(rows, loaded, a.figdir, PH, T_common)
    write_readme(rows, summary, PH, T_common, all_R)
    import shutil
    refdir = os.path.join(VT_MAT, "reference_outputs")
    os.makedirs(refdir, exist_ok=True)
    for f_ in [os.path.join(BB.TABLES, "valve_turning_sim." + e) for e in ("csv", "tex")] + list(figs.values()):
        shutil.copy2(f_, refdir)
    if a.paper:
        splice_paper(a.paper, tex, figs)
    print("wrote", os.path.join(BB.TABLES, "valve_turning_sim.{csv,json,tex}"), "and", a.figdir)


def failed_attempts():
    out = []
    for d_ in sorted(glob.glob(os.path.join(VT, "failed", "*_attempt*"))):
        npz = sorted(glob.glob(os.path.join(d_, "*.npz")))
        if not npz:
            out.append([f"`{os.path.basename(d_)}`", "no npz (never flew)", "--", "--"])
            continue
        z = np.load(npz[0], allow_pickle=True)
        mk = {str(n): float(t) for t, n in zip(z["marks_t"], z["marks_name"])}
        od = z["odom"]
        w, x, y, _ = od[:, 7:11].T
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
        reason = str(z["reason"]) if "reason" in z.files else ""
        direct = od[:, 11] > 0.5
        t_out = float(od[direct, 0].max()) if direct.any() else float("nan")
        ps = mk.get("push:start")
        when = f"{t_out - ps:+.1f} s into the Turn step" if ps is not None and np.isfinite(t_out) else "--"
        after = tilt[od[:, 0] >= t_out] if np.isfinite(t_out) else tilt
        tmax = float(after.max()) if after.size else float(tilt.max())
        outcome = ("**the vehicle flipped**" if tmax > 90 else "landed upright") + f" (tilt max {tmax:.0f} deg, final height {od[-1, 3]:.2f} m)"
        out.append([f"`{os.path.basename(d_)}`", reason.split(" -- ")[0][:120], when, outcome])
    return out


def write_readme(rows, summary, PH, T_common, all_R):
    import shutil
    from readme_text import _md_table
    cfg_dir = os.path.join(VT_MAT, "configs")
    os.makedirs(cfg_dir, exist_ok=True)
    first_R = all_R[min(all_R, key=lambda mk: (ORDER.index(mk[0]), mk[1]))]
    coord_R = next((all_R[mk] for mk in sorted(all_R) if mk[0] == "coordinated_admittance"), None)
    for mth in ORDER:
        Rm_ = next((R_ for (m_, _k), R_ in all_R.items() if m_ == mth), None)
        if Rm_ is not None:
            shutil.copy2(Rm_["yaml"], os.path.join(cfg_dir, CONFIG_NAME[mth]))
    plant, task = [], []
    for line in open(first_R["yaml"]):
        mm = re.match(r"^\s*(sim_[a-z0-9_]+):\s*([^#\n]*)", line)
        if mm:
            plant.append(f"{mm.group(1)}: {mm.group(2).strip()}")
        mm = re.match(r"^\s*(push_pull_[a-z0-9_]+):\s*([^#\n]*)", line)
        if mm:
            task.append(f"{mm.group(1)}: {mm.group(2).strip()}")
    jl = os.path.join(VT, "campaign.jsonl")
    att = [json.loads(l) for l in open(jl)] if os.path.isfile(jl) else []
    if not COMPARISON:
        att = [y for y in att if y["method"] in {m_ for m_, _k in all_R}]
    first =min((x["time"] for x in att), default="?")[:16].replace("T", " ")
    last = max((x["time"] for x in att), default="?")[:16].replace("T", " ")
    names = {m_: METHODS[m_][0] for m_ in ORDER}
    srt = sorted(rows, key=lambda x: (ORDER.index(x["method"]), x["run"]))
    run_rows = [[names[r["method"]], f"`{r['name']}.mat`", f"{r['eps_uav_max_mm']:.1f}", f"{r['rho_uav_max']:.3f}",
                 f"{r['eps_uav_rms_mm']:.1f}", f"{r['rho_uav_rms']:.3f}", f"{r['ee_pos_mm']:.1f}",
                 f"{r['e_imp_slide_rms_n']:.3f}", f"{r['e_imp_contact_rms_n']:.3f}", f"{r['beam_turned_deg']:.1f}"] for r in srt]
    mean_rows = [[names[x["method"]], str(x["runs"]), f"{x['eps_uav_max_mm']:.1f}", f"{x['rho_uav_max']:.3f}",
                  f"{x['eps_uav_rms_mm']:.1f}", f"{x['rho_uav_rms']:.3f}", f"{x['ee_pos_mm']:.1f}",
                  f"{x['e_imp_slide_rms_n']:.3f}", f"{x['e_imp_contact_rms_n']:.3f}", f"{x['beam_turned_deg']:.1f}"] for x in summary]
    att_rows = [[names.get(m, m), str(len([y for y in att if y["method"] == m])),
                 str(sum(1 for y in att if y["method"] == m and y["status"] == "completed")),
                 "; ".join(sorted(set(y["why"] for y in att if y["method"] == m and y["status"] != "completed" and y["why"]))) or "--"]
                for m in sorted(set(y["method"] for y in att))]
    T = first_R["task"]
    coord = coord_R is not None
    pose = ", ".join(f"{v:g}" for v in T["pose_deg"])
    what = {"go_to_start": f"fly from the hover to the start waypoint (the claw {T['approach_back_m']:.2f} m behind and "
                           f"{T['approach_dz_m']:.2f} m above the fin, the nose along the turn's tangent, the arm already in the turn pose); hover",
            "ready": "open the gripper, hover (CoM-anchored), descend diagonally onto the fin; the claw world-held from its arrival",
            "grasp": "wait for the claw within 2 cm / 3 mm across the fin, close the gripper on the fin (the jaws' stall)",
            "push": f"the law in CONTACT: {T['settle_s']:g} s settle, then the {T['turn_deg']:g} deg turn about the bearing axis over "
                    f"{T['push_time_s']:g} s (the claw on its {T['fin_radius_m']:.3f} m circle, the base and heading rotating with it, the arm held); hold on the fin",
            "release": ("Exit To Turn: the planner releases the claw (its reference back to the CoM anchor, the contact "
                        "phase off), then the gripper opens; the exit starts on its OPEN report" if T["release_at_open"] else
                        f"the gripper opens with the claw still world-held on the fin; Exit To Turn "
                        f"{T['planner'].get('valve_exit_wait_s', 1.0):g} s after the OPEN report (the arm GS's Valve Turning "
                        "sequence; the claw's reference returns to the CoM anchor only then)"),
            "exit": f"climb {T['exit_dz_m']:.2f} m with the jaws open, fold the arm home; hover",
            "go_to_land_start": "one planner transition to the landing hover"
                                + (f" ({T['land_start'][0]:g}, {T['land_start'][1]:g}, {T['land_start'][2]:g}) m" if len(T["land_start"]) >= 3 else "")
                                + ", heading turning clockwise as typed; hover",
            "land": "straight down to the land hover" + (f" at z {T['land'][2]:g} m" if len(T["land"]) >= 3 else "")
                    + " (the planner's last leg; the touchdown after it is not scored)"}
    ph_rows = [[str(i + 1), PHASE_LABEL[key], f"{PH[key][0]:.2f}", f"{PH[key][1]:.2f}", f"{PH[key][1] - PH[key][0]:.2f}", what[key]]
               for i, key in enumerate(PHASE_KEYS)]
    imp_rows = []
    for x in rows:
        for w in IMP_WINDOWS:
            v = x["impedance"][w]
            imp_rows.append([f"`{x['name']}`", w, f"{v['T_s']:.1f}", f"{v['e_imp_rms_n']:.3f}", f"{v['e_imp_max_n']:.3f}",
                             " / ".join(f"{c:.3f}" for c in v["e_imp_xyz_rms_n"]), f"{v['e_imp_psi_rms_nm']:.4f}",
                             f"{v['F_ext_rms_n']:.2f}", f"{v['e_y_rms_mm']:.1f}"])
    valve_rows = [[f"`{x['name']}`", f"{x['beam_turned_deg']:.1f}", f"{x['beam_follow_through_deg']:+.1f}",
                   f"{x['beam_plan_err_rms_deg']:.2f} / {x['beam_plan_err_max_deg']:.2f}",
                   f"{1e3 * x['claw_radius_mean_m']:.1f} ± {x['claw_radius_std_mm']:.1f}",
                   f"{x['q2_margin_turn_deg']:.1f} / {x['q3_margin_turn_deg']:.1f}", f"{100 * x['on_stop_fraction']:.1f}",
                   f"{x['tilt_max_deg']:.1f}", f"{x['release_q2_max_deg']:.1f} / {x['release_q3_min_deg']:.1f}"] for x in srt]
    g = first_R["gains"]
    slots = first_R["timetable"]
    fails = failed_attempts()
    if not COMPARISON:
        shown = {m_ for m_, _k in all_R}
        fails = [f for f in fails if any(f[0].strip("`").startswith(m_ + "_") for m_ in shown)]
    first_name = srt[0]["name"]
    if coord:
        method_lines = [
            "- **`<method>`:** `whole_body_l1` = Proposed (whole-body 4-D L1 impedance); `coordinated_admittance` =",
            "  Liu et al., CEP 146 (2024) 105898 (decoupled UAV PD + momentum wrench estimator + admittance on the",
            "  end-effector reference + force-adaptive kinematic MPC; position-mode arm), implemented as an independent",
            "  comparison rig -- its platform / arm references in `run` are that rig's own planner commands.", ""]
        align_lines = ["`t = 0` is the start of the Proposed rig's first transit; the methods are ALIGNED AT READY",
                       "(`meta.t_shift_s` is the shift applied)."]
    else:
        method_lines = ["- **`<method>`:** `whole_body_l1` = the proposed whole-body 4-D L1 impedance controller (the only method in",
                        "  this folder).", ""]
        align_lines = ["`t = 0` is the start of the first transit (go to start) of every run."]
    L = [
        "# Simulation results, MATLAB data (valve turning)", "",
        "This is the data behind the paper's valve-turning table (`tab:sim_valve_turn`) and its figures: the",
        "second physical-interaction task, flown in Isaac Sim by the proposed whole-body L1 impedance controller",
        f"({len(rows)} runs of one mission). It is one task folder of `matlab_simulation_data/`, self-contained, with the",
        "same plant, the same `run` struct layout and the same notation as the box-pushing folder beside it; where",
        "that folder has `box`, this one has `valve` (the beam's rotation) and `raw.rotor`.", "",
        "## Files", "",
        "```",
        "<method>_turn_run<k>.mat                 one run, struct `run`",
        "valve_turning_metrics.mat                metrics_mean = the table (mean over runs), metrics_runs = every run,",
        "                                         metrics_phase = every run x phase, metrics_impedance = every run x",
        "                                         contact window, phases = the common phase table, window_s, L_arm_m",
        "configs/<method>_sim_valve_turning.yaml  the controller + planner (push-and-pull block, turn form) + plant, as flown",
        "reference_outputs/                       the same table and figures drawn by the Python scorer",
        "                                         (valve_turning_sim.csv / .tex, sim_valve_turn_*.png)",
        "```", "",
        *method_lines,
        "## Quick start in MATLAB", "",
        "```matlab",
        "S = load('valve_turning_metrics.mat');",
        "T = struct2table(S.metrics_mean)                 % THE TABLE: one row per method, mean over the runs",
        "R = struct2table(S.metrics_runs);                % every run (the spread between runs)",
        "P = S.phases;                                    % the eight phases, common to every run",
        f"r = load('{first_name}.mat').run;",
        "tr = r.tracking;  w = tr.in_window > 0;          % 100 Hz, t = 0 at the start of go to start",
        "",
        "% (1) the beam against its plan, and the platform deviation",
        "figure; tiledlayout(2, 1)",
        "nexttile; plot(tr.t(w), tr.beam_angle_deg(w) - tr.beam_angle_deg(find(tr.phase_id == 4, 1)), tr.t(w), tr.beam_angle_plan_deg(w) - tr.beam_angle_plan_deg(find(tr.phase_id == 4, 1)), '--')",
        "ylabel('beam angle (deg)'); legend('measured', 'plan')",
        "nexttile; plot(tr.t(w), 1e3 * tr.eps_uav_m(w)); ylabel('||\\epsilon_{UAV}|| (mm)'); xlabel('Time (s)')",
        "",
        "% (2) the contact force and the impedance residual over the contact window",
        "im = r.impedance;  W = im.windows;",
        "figure; tiledlayout(3, 1)",
        "nexttile; plot(im.t, im.F_ext(:,1:3)); ylabel('F_{ext} (N)'); legend('x', 'y', 'z')",
        "nexttile; plot(im.t, im.e_imp(:,1:3)); ylabel('e_{imp} (N)')",
        "nexttile; plot(im.t, vecnorm(im.e_imp(:,1:3), 2, 2)); ylabel('||e_{imp}|| (N)'); xlabel('Time (s)')",
        "",
        "% (3) 3-D: references (dashed), measured platform and end effector, the claw's circle about the axis",
        "figure; hold on; grid on; axis equal; view(-50, 28)",
        "plot3(tr.platform_pos_ref_m(w,1), tr.platform_pos_ref_m(w,2), tr.platform_pos_ref_m(w,3), 'k--')",
        "plot3(tr.ee_pos_ref_m(w,1), tr.ee_pos_ref_m(w,2), tr.ee_pos_ref_m(w,3), '--', 'Color', [0.45 0.45 0.45])",
        "plot3(tr.platform_pos_m(w,1), tr.platform_pos_m(w,2), tr.platform_pos_m(w,3))",
        "plot3(tr.ee_pos_m(w,1), tr.ee_pos_m(w,2), tr.ee_pos_m(w,3), ':')",
        "tv = r.meta.task_values;  th = linspace(0, 2*pi, 90);",
        "plot3(tv.axis_xy_m(1) + tv.fin_radius_m*cos(th), tv.axis_xy_m(2) + tv.fin_radius_m*sin(th), (tv.table_top_z_m + tv.grasp_height_above_table_m)*ones(size(th)), 'Color', [0.64 0.08 0.18])",
        "",
        "% (4) one phase's numbers, e.g. the turn, for every run",
        "Ph = struct2table(S.metrics_phase);  Ph(strcmp(Ph.phase, 'push'), :)",
        "```", "",
        "## Timing: one timetable for every run", "",
        "The flight driver starts every step at the same mission time on every run",
        f"(`pl_mission.py --timetable`, slots {', '.join(f'{x:g}' for x in slots)} s, each from the previous step's",
        "start; `meta.timetable_slots_s` names them). The phases are contiguous. Only the GRASP phase's own length",
        "varies (the claw's arrival within 2 cm / 3 mm across the fin, then the jaws' close).",
        *align_lines,
        "The phase table below is the mean over the runs; every run's phase starts from Ready on agree with it to "
        f"{max(max(abs(R['phases'][k][0] - R['t0'] - PH[k][0]) for k in PHASE_KEYS if k != 'go_to_start') for R in all_R.values()):.2f} s.", "",
        _md_table(["#", "phase", "start [s]", "end [s]", "duration [s]", "what happens"], ph_rows), "",
        f"- **Window:** every metric covers the whole mission, 0 to {T_common:.2f} s. `run.tracking.in_window`",
        f"  marks it; `run.tracking.phase_id` (1..{len(PHASE_KEYS)}) says which phase each sample is in (4 = the turn).",
        "- **Contact windows** (`run.impedance.windows`, `run.metrics.impedance.<window>`; the names are the shared",
        "  scorer's): CONTACT = ready end to exit start (the law's contact phase, the claw world-held); HOLD = the grip",
        f"  and the {T['settle_s']:g} s settle; SLIDE = **the {T['push_time_s']:g} s turn**; RELEASE = turn end to exit start (the jaws open).", "",
        "## Simulation index", "",
        f"Flown {first} to {last} (attempt end times), headless Isaac Sim at real time, raw mocap-emulator feedback.", "",
        _md_table(["method", "file", "eps max [mm]", "rho max", "eps rms [mm]", "rho rms", "EE RMSE [mm]",
                   "e_imp turn rms [N]", "e_imp contact rms [N]", "beam turned [deg]"], run_rows), "",
        "The table is the mean over the runs:", "",
        _md_table(["method", "runs", "eps max [mm]", "rho max", "eps rms [mm]", "rho rms", "EE RMSE [mm]",
                   "e_imp turn rms [N]", "e_imp contact rms [N]", "beam turned [deg]"], mean_rows), "",
        "The impedance residual per contact window (`metrics_impedance`; SLIDE = the turn):", "",
        _md_table(["run", "window", "T [s]", "e_imp rms [N]", "e_imp max [N]", "x / y / z rms [N]", "heading rms [N.m]",
                   "F_ext rms [N]", "e_y rms [mm]"], imp_rows), "",
        "The valve and the arm (`metrics.valve`, `metrics.window`):", "",
        _md_table(["run", "beam turned in the turn step [deg]", "follow-through after release [deg]",
                   "beam − plan rms / max [deg]", "claw radius about the axis [mm]", "q2 / q3 margin to the stops in the turn [deg]",
                   "on a stop [% of the turn]", "vehicle tilt max [deg]", "q2 max / q3 min, release + exit [deg]"], valve_rows), "",
        "The stops are q2 = −20 deg (the real arm's range, enforced in the scene) and q3 = +50 deg (the asset's); the",
        "margin is the smallest distance to either during the turn step. With the claw world-held on its circle the",
        "base's lead over its moving reference is absorbed by the arm, so a margin of 0 means the arm sat on a stop.", "",
        "Attempts (a run is kept only if it completed, turned the beam >= 350 deg and kept every step on its slot):", "",
        _md_table(["method", "attempts", "kept", "why the others were not kept"], att_rows) if att_rows else "--", "",
        *(["The failed attempts (kept whole under `results/simulation_results/Isaac_simulation_data/valve_turning/failed/`):", "",
           _md_table(["attempt", "what stopped it", "when", "afterwards"], fails), ""] if fails else []),
        "## Scene and task", "",
        f"- **Table:** the HOOBRO console table (1.000 x 0.150 x 0.800 m), its length along world y: x "
        f"{T['table_xmin']:.3f}-{T['table_xmax']:.3f} m, y {T['table_ymin']:g}..{T['table_ymax']:g} m, top at {T['table_top_z']:.2f} m.",
        f"- **Valve:** the user's printed mechanism (Valve_Assembly.stl, 2026-10-15): a 150 x 50 x 20 mm plate with a 608",
        f"  bearing, bolted to the table with its axis at ({T['axis_xy'][0]:.2f}, {T['axis_xy'][1]:.2f}) m; on the bearing a 225 mm beam",
        f"  carrying a 5 x 40 x 130 mm fin handle at radius {T['fin_radius_m']:.3f} m, the fin top {1e3 * (T['grasp_height_m'] + 0.025):.0f} mm above the",
        f"  table. The rotor is flown at the printed part's **{1e3 * T['rotor_mass_kg']:.0f} g** (the pick-and-drop payload's density, same",
        f"  printer and PLA). **Bearing: Coulomb friction {T['friction_nm']:g} N.m** (= {T['friction_nm'] / T['fin_radius_m']:.2f} N at the fin) + viscous",
        f"  {T['damping_nms']:g} N.m.s/rad -- a CONSERVATIVE value: the real bearing, pulled by a 10 N spring gauge at the fin",
        "  (2026-10-17), never passed the gauge's first 0.2 N division (<= 0.032 N.m); the sim flies 1.5x that bound.",
        "  The sim claw cannot close below ~19 mm, so the fin's bare part carries an invisible 20 mm grip shim (the real",
        "  gripper's foam pads close on the 5 mm fin).",
        f"- **Task:** grasp the fin from behind and above (go to start, ready, grasp), turn the beam {T['turn_deg']:g} deg about the",
        f"  bearing axis in {T['push_time_s']:g} s (a smooth start-stop rotation after a {T['settle_s']:g} s settle: the claw on its circle,",
        "  the base on a larger one behind it, the heading turning with them, the arm held still), release, climb out,",
        "  fly to the landing hover, descend. The whole-body law's contact phase (chi) is "
        + ("CONTACT from the jaws' grip to the end of the turn." if T["contact_phase"] else "left FREE (push_pull_contact false)."),
        f"- **Arm pose in the turn:** [{pose}] deg (fold {T['pose_deg'][1] + T['pose_deg'][2]:g} deg; the fin is low -- {1e3 * T['grasp_height_m']:.0f} mm above the table",
        "  at the grasp -- so a more folded arm would put the T650 skids onto the beam); the real arm's q2 range [-20, 45] deg",
        "  is enforced as hard stops in the scene.",
        "- **Safety guards (on in every attempt; none tripped in the kept runs; "
        + (f"{len(fails)} failed attempt{'s' if len(fails) != 1 else ''} above" if fails else "no failed attempts") + "):**",
        f"  the planner aborts on a task force > {T['planner'].get('guard_force_n', 5.0):g} N or a vehicle tilt > "
        f"{T['planner'].get('guard_tilt_deg', 15.0):g} deg; the flight node's watchdog hands over to SAFETY at "
        f"{T['wd_tilt_deg']:g} deg of tilt.",
        *([f"  The planner's abort on the free handle: hold the {'measured' if T['planner'].get('abort_from_measured') == 'true' else 'planned'} "
           f"pose the turn was cut at, open the jaws with the claw world-held for "
           f"{T['planner'].get('abort_release_wait_s', 1.0):g} s, then climb straight up"
           + (f" after backing out {T['planner'].get('abort_back', 0.0):g} m" if float(T['planner'].get('abort_back', 0.0) or 0.0) > 0 else "")
           + "; the jaws also open if the node leaves DIRECT with the claw on the handle (the driver, as the arm GS does)."]
          if str(T['planner'].get('abort_release_world_held', 'false')).lower() == 'true' else []),
        f"- **Impedance, Proposed (the whole-body law's EE task):** M_d = diag({', '.join(f'{v:g}' for v in g['my'])}),",
        f"  D_d = diag({', '.join(f'{v:g}' for v in g['dy'])}), K_d = diag({', '.join(f'{v:g}' for v in g['ky'])})",
        "  on [x, y, z, heading] (kg, N.s/m, N/m; kg.m^2, N.m.s/rad, N.m/rad for the heading).",
        *([f"- **Admittance, Coordinated [Liu 2024] (eq 27, contact phase, claw frame):** M_d = diag({', '.join(f'{v:g}' for v in coord_R['gains']['my'][:3])}),",
           f"  D_d = diag({', '.join(f'{v:g}' for v in coord_R['gains']['dy'][:3])}), K_d = diag({', '.join(f'{v:g}' for v in coord_R['gains']['ky'][:3])}) (kg, N.s/m, N/m)."]
          if coord else []),
        "- **Plant:** the free-flight mirror plant -- motor delay, model mismatch, battery sag, a standing wrench",
        "  bias, imperfect joint actuation and emulated mocap feedback (the free-flight README explains each).",
        "  Every `sim_*` key as flown:", "",
        "```", *plant, "```", "",
        "- **The push-and-pull planner block as flown, turn form** (`configs/`):", "",
        "```", *task, "```", "",
        "## The struct `run`", "",
        _md_table(["field", "content"], [
            ["`meta`", "method, run index, `scene`, `task`, `task_values` (turn angle / time / settle, the fin's radius and grasp "
                       "height, the axis, the rotor's mass, the bearing's friction and damping, the contact-phase flag, the turn "
                       "pose, the table, Land Start, Land, the controller overrides), plant, `phases`, `timetable_slots_s`, "
                       "`late_steps` (empty), `window`, `impedance_windows`, `impedance_gains` (M_d, D_d, K_d), "
                       "`contact_sign` (the PhysX report's sign and the rule that set it), `L_arm_m`, `base_com_model_m`, `t0_driver_s`"],
            ["`metrics`", "`window` = this run's table numbers over the window; `per_phase.<key>` = the same per phase; "
                          "`whole_recording` = over every sample; `impedance.<window>` = the impedance residual per contact "
                          "window; `valve` = the beam's rotation in the turn step, its follow-through after the release, its "
                          "error against the plan, the claw's radius about the axis, the arm's margins to its stops"],
            ["`tracking`", "100 Hz, `t` in s since the start of the mission; `in_window`, `phase_id`; measured, reference "
                           "(`_ref_`) and error (`_err_`) for the platform position, `eps_uav_m`, `rho_uav`, the platform "
                           "attitude, the system CoM, the end-effector position and heading, the joints, and the BEAM angle "
                           "against its plan (`beam_angle_deg`, `beam_angle_plan_deg`, `beam_minus_plan_deg`)"],
            ["`impedance`", "the residual's time series over the CONTACT window (contact-report rate, ~62.5 Hz): `t`, `e_imp`, "
                            "`F_ext` (the true contact wrench on the claw), `rendered` (M_d edot_v + D_d e_v + K_d e_y), `e_y`, "
                            "and `windows` (name, start_s, end_s)"],
            ["`raw`", "every recorded stream on the driver's clock (subtract `meta.t0_driver_s` to get `tracking.t`): "
                      "`odom`, `joints`, `reference` (the planner's whole-body reference), `reference_derivatives`, `rotor` "
                      "(ground truth = the mocap's obj_0: the fixed axis and the beam's angle), `claw_truth`, `base_truth`, "
                      "`contact_truth` (the rotor's PhysX contact report), `marks` (every step's start/end), `events`"]]), "",
        "Units: positions in m (world ENU, z up) unless the field name says mm; angles in deg unless it says rad;",
        "forces in N, moments in N.m. Errors are measured − reference. `platform_att_err_deg` columns are",
        "`[roll pitch heading]`.", "",
        "## Paper notation", "",
        _md_table(["paper symbol", "meaning", "summary field (`metrics_mean`)", "run file"], [
            ["e<sub>imp</sub>", "F<sub>ext</sub> − (M<sub>d</sub> ė<sub>v</sub> + D<sub>d</sub> e<sub>v</sub> + K<sub>d</sub> e<sub>y</sub>), "
             "the impedance residual on the claw [N; heading N·m], rms over the turn / the whole contact",
             "`e_imp_slide_rms_n`, `e_imp_slide_max_n`, `e_imp_slide_{x,y,z}_rms_n`, `e_imp_slide_psi_rms_nm`, `e_imp_contact_rms_n`",
             "`impedance.e_imp` [N N N N·m], `metrics.impedance.<window>`"],
            ["F<sub>ext</sub>", "the true contact wrench on the claw (ground truth)", "`F_ext_slide_rms_n`", "`impedance.F_ext`"],
            ["‖ε<sub>UAV</sub>‖", "‖r<sub>UAV</sub><sup>ref</sup> − r<sub>UAV</sub>‖, the platform's deviation from its "
             "reference position [mm]", "`eps_uav_max_mm`, `eps_uav_rms_mm`", "`tracking.eps_uav_m` [m]"],
            ["ρ<sub>UAV</sub>", "‖ε<sub>UAV</sub>‖ / L, L = 0.370 m (the arm's reach)", "`rho_uav_max`, `rho_uav_rms`", "`tracking.rho_uav`"],
            ["r<sub>0,x</sub>, r<sub>0,y</sub>, r<sub>0,z</sub>", "platform position [mm]",
             "`platform_x_mm`, `platform_y_mm`, `platform_z_mm`", "`tracking.platform_pos_err_m(:,1:3)` [m]"],
            ["ψ<sub>0</sub>, φ<sub>0</sub>, θ<sub>0</sub>", "platform heading, roll, pitch [deg]",
             "`platform_heading_deg`, `platform_roll_deg`, `platform_pitch_deg`", "`tracking.platform_att_err_deg(:,[3 1 2])`"],
            ["r<sub>e,x</sub>, r<sub>e,y</sub>, r<sub>e,z</sub>", "end-effector position [mm]",
             "`ee_x_mm`, `ee_y_mm`, `ee_z_mm`", "`tracking.ee_pos_err_m(:,1:3)` [m]"],
            ["ψ<sub>e</sub>", "end-effector heading [deg]", "`ee_heading_deg`", "`tracking.ee_heading_err_deg`"],
            ["q<sub>1</sub> … q<sub>4</sub>", "joint angles [deg]", "`q1_deg` … `q4_deg`", "`tracking.q_err_deg(:,1:4)`"],
            ["θ<sub>v</sub>", f"the beam's rotation during the turn step [deg] ({T['turn_deg']:g} commanded)",
             "`beam_turned_deg`, `beam_plan_err_rms_deg`", "`metrics.valve.turned_deg`, `tracking.beam_angle_deg`"]]), "",
        "- **Reference:** the planner's dynamically compatible whole-body reference. The platform reference",
        "  x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>) is r<sub>UAV</sub><sup>ref</sup>.",
        "  During the turn the EE reference r<sub>e,d</sub> is the claw's planned circle about the axis (world-held).",
        "- **max / rms:** over the window; the table's other columns are RMS errors.",
        f"- **e<sub>imp</sub>:** every term through one zero-phase {FC_HZ:g} Hz low-pass; e<sub>y</sub> from the TRUE claw point",
        "  (ground truth) minus the streamed reference; e<sub>v</sub>, ė<sub>v</sub> its derivatives against the streamed",
        "  reference derivatives; F<sub>ext</sub> from the rotor's PhysX contact report, its sign the one that makes the",
        "  vehicle's mean power on the rotor positive over the turn (`meta.contact_sign`). A residual of 0 means the claw",
        "  moved exactly as the designed impedance under the measured force.", ""]
    with open(os.path.join(VT_MAT, "README.md"), "w") as f:
        f.write("\n".join(L))
    print("wrote", os.path.join(VT_MAT, "README.md"))


def latex_table(summary, T_common, task=None):
    T = task or dict(push_time_s=36.0, turn_deg=360.0)
    coord = any(x["method"] == "coordinated_admittance" for x in summary)
    cols = ["e_imp_slide_rms_n", "e_imp_slide_max_n", "e_imp_contact_rms_n", "eps_uav_max_mm", "rho_uav_max", "eps_uav_rms_mm",
            "rho_uav_rms", "platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
            "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg", "q4_deg"]
    fmt = {c: ("%.3f" if c.startswith("rho") or c.startswith("e_imp") else "%.2f") for c in cols}
    vals = {s["method"]: {c: fmt[c] % s[c] for c in cols} for s in summary}
    lines = [r"\begin{table}[H]", r"\centering",
             r"\caption{Valve turning in simulation: impedance residual, platform deviation and RMSE of the tracked states over the whole mission.}",
             r"\label{tab:sim_valve_turn}", r"\footnotesize", r"\setlength{\tabcolsep}{3pt}", r"\begin{threeparttable}",
             r"\begin{tabular}{l c c c c c c c c c c c c c c c c c c c c c c}", r"\toprule",
             r"& \multicolumn{3}{c}{Impedance residual} & \multicolumn{4}{c}{UAV deviation} & \multicolumn{6}{c}{Platform} & \multicolumn{4}{c}{End-effector} & \multicolumn{4}{c}{Joint} \\",
             r"\cmidrule(lr){2-4} \cmidrule(lr){5-8} \cmidrule(lr){9-14} \cmidrule(lr){15-18} \cmidrule(lr){19-22}",
             r"Method & $\|e_{imp}\|_{\mathrm{rms}}^{turn}$ & $\|e_{imp}\|_{\max}^{turn}$ & $\|e_{imp}\|_{\mathrm{rms}}^{contact}$ & $\|\varepsilon_{UAV}\|_{\max}$ & $\rho_{UAV,\max}$ & $\|\varepsilon_{UAV}\|_{\mathrm{rms}}$ & $\rho_{UAV,\mathrm{rms}}$ & $r_{0,x}$ & $r_{0,y}$ & $r_{0,z}$ & $\psi_0$ & $\phi_0$ & $\theta_0$ & $r_{e,x}$ & $r_{e,y}$ & $r_{e,z}$ & $\psi_e$ & $q_1$ & $q_2$ & $q_3$ & $q_4$ \\",
             r" & (N) & (N) & (N) & (mm) & (--) & (mm) & (--) & (mm) & (mm) & (mm) & ($^\circ$) & ($^\circ$) & ($^\circ$) & (mm) & (mm) & (mm) & ($^\circ$) & ($^\circ$) & ($^\circ$) & ($^\circ$) & ($^\circ$) \\",
             r"\midrule"]
    for m in ORDER:
        if m in vals:
            lines.append(f"{METHODS[m][0]} & " + " & ".join(vals[m][c] for c in cols) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\begin{tablenotes}", r"\footnotesize",
              r"\item[] Each entry is the mean over the completed runs. "
              r"$e_{imp} = \boldsymbol{F}_{ext} - (\boldsymbol{M}_d \dot{\boldsymbol{e}}_v + \boldsymbol{D}_d \boldsymbol{e}_v + \boldsymbol{K}_d \boldsymbol{e}_y)$ "
              r"is the impedance residual on the end effector (the measured contact force minus the force the designed impedance "
              r"would render for the measured deflection), given as the RMS and the maximum of its norm over the %g~s turn of the "
              r"handle through %g$^\circ$ and the RMS over the whole contact phase (grasp to release). " % (T["push_time_s"], T["turn_deg"]) +
              r"$\|\varepsilon_{UAV}\| = \|\boldsymbol{r}_{UAV}^{ref} - \boldsymbol{r}_{UAV}\|$ is the platform's deviation from "
              r"its reference position and $\rho_{UAV} = \|\varepsilon_{UAV}\|/L$ its ratio to the arm's reach $L = 0.37$~m, "
              r"given as the maximum and the RMS over the window; the remaining columns are RMS tracking errors with the "
              r"symbols of Table~\ref{tab:sim_free_flight_tracking}. The window is the whole mission (%.1f~s), from the "
              r"start of the first transit to the end of the landing leg; every run flies the same timetable" % T_common +
              (" from Ready on (the methods are aligned at Ready)." if coord else "."),
              r"\end{tablenotes}", r"\end{threeparttable}", r"\end{table}", ""]
    return "\n".join(lines)


def make_figures(rows, loaded, figdir, PH, T_common):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "font.size": 10, "axes.titlesize": 11, "mathtext.fontset": "stix"})
    pick = {}
    for r in rows:
        if r["method"] not in pick or r["e_imp_slide_rms_n"] < pick[r["method"]]["e_imp_slide_rms_n"]:
            pick[r["method"]] = r
    sel = [(m, loaded[pick[m]["name"]]) for m in ORDER if m in pick]
    out = {}
    Tk = sel[0][1]["task"]
    # ---- 3-D trajectories: the table, the claw's circle about the axis, the beam at start / end ----
    fig = plt.figure(figsize=(7.2, 5.2)); ax = fig.add_subplot(111, projection="3d")
    x0_, x1_, y0_, y1_, zt_ = Tk["table_xmin"], Tk["table_xmax"], Tk["table_ymin"], Tk["table_ymax"], Tk["table_top_z"]
    ax.plot([x0_, x1_, x1_, x0_, x0_], [y0_, y0_, y1_, y1_, y0_], [zt_] * 5, color="#8899aa", lw=0.9)
    for xx, yy in ((x0_, y0_), (x1_, y0_), (x1_, y1_), (x0_, y1_)):
        ax.plot([xx, xx], [yy, yy], [0.6, zt_], color="#8899aa", lw=0.6)
    axc = Tk["axis_xy"]; rf = Tk["fin_radius_m"]; zg = zt_ + Tk["grasp_height_m"]
    th = np.linspace(0, 2 * np.pi, 120)
    ax.plot(axc[0] + rf * np.cos(th), axc[1] + rf * np.sin(th), zg * np.ones_like(th), color="#A2142F", lw=0.9, ls="-.", label="Fin circle (r 0.16 m)")
    first = True
    for m, R in sel:
        mk = (R["tu"] - R["t0"]) <= T_common + 1e-9
        if first:
            ax.plot(R["xb"][mk, 0], R["xb"][mk, 1], R["xb"][mk, 2], "k--", lw=0.9, label="Platform ref.")
            ax.plot(R["r_ed"][mk, 0], R["r_ed"][mk, 1], R["r_ed"][mk, 2], "--", color="0.45", lw=0.9, label="End-effector ref.")
            first = False
        ax.plot(R["P"][mk, 0], R["P"][mk, 1], R["P"][mk, 2], color=COLORS[m], lw=1.2, label=f"{METHODS[m][0]} platform")
        ax.plot(R["re"][mk, 0], R["re"][mk, 1], R["re"][mk, 2], color=COLORS[m], lw=1.0, ls=":", label=f"{METHODS[m][0]} end-effector")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_xlim(-0.6, 1.6); ax.set_ylim(-1.1, 1.1); ax.set_zlim(0.6, 1.6)
    ax.view_init(elev=28, azim=-50)
    ax.legend(loc="upper left", fontsize=8, ncol=2, frameon=False)
    ax.set_box_aspect((1, 1, 0.45))
    fig.tight_layout()
    p = os.path.join(figdir, "sim_valve_turn_trajectory_3d.png"); fig.savefig(p, dpi=220); plt.close(fig); out["traj"] = p

    def bands(ax, labels=False):
        for i, key in enumerate(PHASE_KEYS):
            a_, b_ = PH[key]
            ax.axvspan(a_, b_, color="0.90" if i % 2 == 0 else "0.97", lw=0)
            if labels:
                lab = PHASE_LABEL[key].replace(" ", "\n", 1) if (b_ - a_) < 8.0 else PHASE_LABEL[key]
                ax.text(0.5 * (a_ + b_), 0.97 if (b_ - a_) > 2.5 else 0.86, lab, ha="center", va="top", fontsize=7,
                        transform=ax.get_xaxis_transform())
        ax.set_xlim(0.0, T_common)

    def tw(R):
        m = (R["tu"] - R["t0"]) <= T_common + 1e-9
        return m, R["tu"][m] - R["t0"]
    # ---- tracking errors ----
    fig, axs = plt.subplots(3, 4, figsize=(12.5, 8.6), sharex=True, constrained_layout=True)
    panels = [("$r_{0,x}$", "mm", lambda R: 1e3 * R["e_pos"][:, 0]), ("$r_{0,y}$", "mm", lambda R: 1e3 * R["e_pos"][:, 1]),
              ("$r_{0,z}$", "mm", lambda R: 1e3 * R["e_pos"][:, 2]), ("$\\psi_0$", "°", lambda R: R["att_err"][:, 2]),
              ("$r_{e,x}$", "mm", lambda R: 1e3 * R["e_ee"][:, 0]), ("$r_{e,y}$", "mm", lambda R: 1e3 * R["e_ee"][:, 1]),
              ("$r_{e,z}$", "mm", lambda R: 1e3 * R["e_ee"][:, 2]), ("$\\psi_e$", "°", lambda R: R["ee_head_err"]),
              ("$q_1$", "°", lambda R: R["e_q"][:, 0]), ("$q_2$", "°", lambda R: R["e_q"][:, 1]),
              ("$q_3$", "°", lambda R: R["e_q"][:, 2]), ("$q_4$", "°", lambda R: R["e_q"][:, 3])]
    for ax, (title, unit, fn) in zip(axs.flat, panels):
        bands(ax)
        for m, R in sel:
            mk, t = tw(R)
            ax.plot(t, fn(R)[mk], color=COLORS[m], lw=0.9, label=METHODS[m][0])
        ax.axhline(0, color="k", ls="--", lw=0.6)
        ax.set_title(title); ax.grid(alpha=0.3)
        ax.set_ylabel(f"$e$ ({unit})" if ax in axs[:, 0] or unit == "°" else "")
    for ax in axs[-1]:
        ax.set_xlabel("Time (s)")
    h, l = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.03))
    p = os.path.join(figdir, "sim_valve_turn_tracking_error.png"); fig.savefig(p, dpi=220, bbox_inches="tight"); plt.close(fig); out["err"] = p
    # ---- UAV deviation + the beam against its plan ----
    fig, axs = plt.subplots(2, 1, figsize=(7.2, 5.4), sharex=True, constrained_layout=True)
    bands(axs[0], labels=True); bands(axs[1])
    for m, R in sel:
        mk, t = tw(R)
        axs[0].plot(t, 1e3 * R["eps"][mk], color=COLORS[m], lw=0.9, label=METHODS[m][0])
        k0 = int(np.searchsorted(R["tu"], R["marks"]["push:start"]))
        axs[1].plot(t, R["beam_u"][mk] - R["beam_u"][k0], color=COLORS[m], lw=1.0, label=f"{METHODS[m][0]} beam")
    R0_ = sel[0][1]
    k0 = int(np.searchsorted(R0_["tu"], R0_["marks"]["push:start"]))
    mk, t = tw(R0_)
    turn = mk & (R0_["tu"] >= R0_["marks"]["push:start"]) & (R0_["tu"] <= R0_["marks"]["push:end"])
    axs[1].plot(R0_["tu"][turn] - R0_["t0"], R0_["az_plan"][turn] - R0_["az_plan"][k0], "k--", lw=0.8,
                label="Plan (the turn step)")
    axs[0].set_ylabel(r"$\|\varepsilon_{UAV}\|$ (mm)"); axs[0].grid(alpha=0.3)
    axs[0].set_ylim(0, 1.25 * axs[0].get_ylim()[1])
    ax2 = axs[0].twinx(); ax2.set_ylim(np.array(axs[0].get_ylim()) / (1e3 * L_ARM)); ax2.set_ylabel(r"$\rho_{UAV}$ (--)")
    axs[0].legend(loc="upper right", fontsize=8, frameon=False, bbox_to_anchor=(1.0, 0.80))
    axs[1].set_ylabel("Beam angle (°)"); axs[1].set_xlabel("Time (s)"); axs[1].grid(alpha=0.3)
    axs[1].legend(loc="upper left", fontsize=8, frameon=False)
    p = os.path.join(figdir, "sim_valve_turn_uav_deviation.png"); fig.savefig(p, dpi=220); plt.close(fig); out["dev"] = p
    # ---- the impedance residual and the contact force over the contact window ----
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 7.0), sharex=True, constrained_layout=True)
    styles = {m: ("-" if i == 0 else "--") for i, m in enumerate(ORDER)}
    for m, R in sel:
        s = R["imp"]["series"]; t = s["t"] - R["t0"]
        for i, (lab, col) in enumerate(zip("xyz", ("#0072BD", "#D95319", "#77AC30"))):
            axs[0].plot(t, s["F"][:, i], color=col, lw=0.9, ls=styles[m], label=f"{METHODS[m][0]} $F_{{ext,{lab}}}$")
            axs[1].plot(t, s["e_imp"][:, i], color=col, lw=0.9, ls=styles[m], label=f"{METHODS[m][0]} $e_{{imp,{lab}}}$")
        axs[2].plot(t, np.linalg.norm(s["e_imp"][:, :3], axis=1), color=COLORS[m], lw=0.9, label=f"{METHODS[m][0]} " + r"$\|e_{imp}\|$")
        for ax in axs:
            ax.axvspan(R["marks"]["push:start"] + 1.5 - R["t0"], R["marks"]["push:end"] - R["t0"], color="0.92", lw=0)
    axs[0].set_ylabel("$F_{ext}$ (N)"); axs[1].set_ylabel("$e_{imp}$ (N)"); axs[2].set_ylabel(r"$\|e_{imp}\|$ (N)")
    axs[2].set_xlabel("Time (s)")
    for ax in axs:
        ax.grid(alpha=0.3); ax.legend(fontsize=8, frameon=False, ncol=3, loc="upper right")
    axs[0].set_title("Contact phase (shaded: the %g s turn)" % Tk["push_time_s"], fontsize=10)
    p = os.path.join(figdir, "sim_valve_turn_impedance.png"); fig.savefig(p, dpi=220); plt.close(fig); out["imp"] = p
    return out


def splice_paper(paper, tex, figs):
    import shutil
    s = open(paper).read()
    head = r"\subsubsection{Physical Interaction: Valve Turning}"
    if head not in s:
        print(f"no '{head}' in {paper} -- the table and figures were written but not spliced")
        return
    i = s.index(head)
    j = s.find(r"\subsubsection", i + len(head))
    if j < 0:
        j = s.find(r"\subsection", i + len(head))
    if j < 0:
        j = len(s)
    figdir = os.path.join(os.path.dirname(os.path.abspath(paper)), "Figures", "2-Results")
    os.makedirs(figdir, exist_ok=True)
    for p in figs.values():
        shutil.copy(p, os.path.join(figdir, os.path.basename(p)))
    block = head + "\n\n" + tex + "\n" + "\n".join([
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_valve_turn_trajectory_3d.png}",
        r"\caption{Platform and end-effector trajectories in the simulated valve turning (dashed: the planned references; dash-dotted: the fin's circle about the bearing axis).}",
        r"\label{fig:sim_valve_turn_trajectory_3d}", r"\end{figure}", "",
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_valve_turn_impedance.png}",
        r"\caption{Contact force on the end effector and the impedance residual during the contact phase of the simulated valve turning (shaded: the turn).}",
        r"\label{fig:sim_valve_turn_impedance}", r"\end{figure}", "",
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_valve_turn_tracking_error.png}",
        r"\caption{Tracking errors in the simulated valve turning (bands: the mission's eight phases).}",
        r"\label{fig:sim_valve_turn_tracking_error}", r"\end{figure}", "",
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_valve_turn_uav_deviation.png}",
        r"\caption{Platform deviation $\|\varepsilon_{UAV}\|$, its ratio $\rho_{UAV}$ to the arm's reach, and the handle's angle against its plan in the simulated valve turning.}",
        r"\label{fig:sim_valve_turn_uav_deviation}", r"\end{figure}", "", ""])
    open(paper, "w").write(s[:i] + block + s[j:])
    print("spliced the table and four figures into", paper)


if __name__ == "__main__":
    main()
