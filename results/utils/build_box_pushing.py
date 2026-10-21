#!/usr/bin/env python3
"""Box pushing (physical interaction), simulation: raw Isaac runs -> metrics, MATLAB files, the paper table
and figures.

    PYTHONNOUSERSITE=1 /usr/bin/python3 results/utils/build_box_pushing.py [--paper <main.tex>] [--figdir <dir>]

Reads results/simulation_results/box_pushing/<method>_push_run<k>.npz (+ <name>.yaml, the controller config the
run flew -- the impedance gains come from it), written by
docs/docs_aerial_manipulator/archive/push_pull_20261003/tools/pl_mission.py --timetable and copied there by
run_box_pushing_campaign.sh, and writes
    simulation_results/matlab_simulation_data/box_pushing/<name>.mat        struct `run`: meta, metrics, tracking, impedance, raw
    simulation_results/matlab_simulation_data/box_pushing/box_pushing_metrics.mat   the table + per-phase metrics
    simulation_results/matlab_simulation_data/box_pushing/README.md, configs/     the folder's guide + the flown configs
    results/utils/tables/box_pushing_sim.csv / .json / .tex                    the table (mean over runs)
    <figdir>/sim_box_push_trajectory_3d.png, sim_box_push_tracking_error.png, sim_box_push_uav_deviation.png,
             sim_box_push_impedance.png
and with --paper splices the table and the figures into the paper's
\\subsubsection{Physical Interaction: Box Push-and-Pull}.

WINDOW: the mission is EIGHT CONTIGUOUS phases (2026-10-17; seven before 2026-10-16) -- go to start, ready
(gripper open, hover, descent onto the handle), grasp (the claw's arrival and the jaws' close), push (1.5 s
settle + the 24 s slide, the law in CONTACT, the claw world-held), release (gripper open), exit (climb, arm
home), go to land start (one planner transition off the table to the landing hover, 270 deg clockwise), land
(straight down to the land hover) -- each from its first step's start to the next phase's start. The driver
flies a fixed TIMETABLE (pl_mission.py --timetable): every step starts at the same mission time on every run;
only the grasp phase's own length varies (the claw's arrival on the fin, ~0.3-2 s), so the phases after it
shift by that much (checked, spread printed). The window is the whole mission, t = 0 at the start of go to
start to the end of land (the shortest run). The scene and task numbers in the README / meta (box mass,
table, push pose, distance, margins) are read from each run's .args / .yaml, not hard-coded.

METRICS
  1. The pick-and-place definitions (build_pick_and_place.py), on the push-and-pull reference stream:
     eps_UAV = |x_b - r_UAV| with x_b = x_cd - R0 r_0c(q_d) (max and RMS), rho_UAV = eps / L (L = 0.370 m),
     RMSE of the platform position xyz, attitude (rotation vector of R_ref^T R, body axes), EE position xyz
     (the model FK), EE heading, joints q1..q4.
  2. The IMPEDANCE RESIDUAL (the physical-interaction metric, user's definition):
         e_imp = F_ext - (M_d edot_v + D_d e_v + K_d e_y)
     per task channel (x, y, z [N], heading [N.m]); M_d / D_d / K_d = the law's wb_my_* / wb_dy_* / wb_ky_*
     from the run's yaml; F_ext = the TRUE contact wrench on the claw (the box's PhysX contact report, the
     scene's /push_pull_truth/contact), e_y = the true claw point minus the streamed EE reference r_ed (heading:
     the law's own e_y[3]); every term through one zero-phase 5 Hz low-pass (pl_metrics.py). Reported over the
     CONTACT window (the law in contact: ready end -> exit start) and inside it HOLD (grip + settle), SLIDE
     (the 24 s move) and RELEASE (push end -> exit start). e_imp = 0 is a perfect impedance.
  3. The box: distance pushed along the table (box truth, push start -> push end), its yaw change, its movement
     during the exit (the open claw catching the fin), and its final position.
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
BP = os.path.join(ROOT, "box_pushing")
TABLES = os.path.join(HERE, "tables")
MAT_DIR = os.path.join(ROOT, "matlab_simulation_data")
BP_MAT = os.path.join(MAT_DIR, "box_pushing")
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
sys.path.insert(0, os.path.join(REPO, "docs", "docs_aerial_manipulator", "archive", "push_pull_20261003", "tools"))

METHODS = {
    "whole_body_l1": ("Proposed", "Whole-body L1 impedance (proposed)"),
    # Liu et al., "A coordinated framework of aerial manipulator for safe and compliant physical
    # interaction", CEP 146 (2024): decoupled UAV PD + momentum wrench estimator + admittance on the
    # EE reference + force-adaptive kinematic MPC (archive/push_pull_coordinated_admittance_20261010)
    "coordinated_admittance": ("Coordinated [Liu 2024]", "Coordinated admittance + kinematic MPC (Liu et al., CEP 2024)"),
}
ORDER = ["whole_body_l1", "coordinated_admittance"]
COLORS = {"whole_body_l1": "#0072BD", "coordinated_admittance": "#D95319"}
NAME = re.compile(r"^(whole_body_l1|coordinated_admittance)_push_run(\d+)$")
# the as-flown config's name in matlab_simulation_data/box_pushing/configs
CONFIG_NAME = {"whole_body_l1": "whole_body_l1_4d_sim_box_pushing.yaml",
               "coordinated_admittance": "coordinated_admittance_sim_box_pushing.yaml"}
# the eight contiguous phases: (key, label, the driver mark that starts it, the mark that ends it).
# 2026-10-16 the planner's last leg became Go To Land Start (one transition off the table) + Land.
PHASES = [("go_to_start", "Go to start", "go_to_start:start", "ready:start"),
          ("ready", "Ready", "ready:start", "close:start"),
          ("grasp", "Grasp", "close:start", "push:start"),
          ("push", "Push", "push:start", "release:start"),
          ("release", "Release", "release:start", "exit:start"),
          ("exit", "Exit", "exit:start", "go_to_land_start:start"),
          ("go_to_land_start", "Go to land start", "go_to_land_start:start", "land:start"),
          ("land", "Land", "land:start", "land:end")]
LAST = PHASES[-1][0]
# the driver's timetable slots in the npz's order (pl_mission.py save())
SLOT_NAMES = ["ready_after_go_to_start", "descent_after_ready", "close_after_ready", "release_after_push",
              "exit_after_release", "go_to_land_start_after_exit", "land_after_go_to_land_start"]
PHASE_KEYS = [p[0] for p in PHASES]
PHASE_LABEL = {p[0]: p[1] for p in PHASES}
PHASE_ALIGN_TOL_S = 2.0       # the grasp phase's own length varies (the claw's arrival on the fin)
IMP_WINDOWS = ["CONTACT", "HOLD", "SLIDE", "RELEASE"]
G = 9.81
L_ARM = 0.370          # [m] L1 + L2, the arm's reach (bench_metrics.py)
DT = 0.01
BASE_COM = np.array([0.0, -0.017854, 0.0])
FC_HZ = 5.0            # the impedance residual's common zero-phase low-pass


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


def scene_text(T):
    """One line on the scene a run flew (task_of())."""
    return (f"07 box pushing: the HOOBRO console table (1.000 x 0.150 x 0.800 m) at x {T['table_xmin']:.3f}-{T['table_xmax']:.3f}, "
            f"y {T['table_ymin']:g}..{T['table_ymax']:g} m, top {T['table_top_z']:.2f} m; the CAD box (240 x 160 x 100 mm, "
            f"{1e3 * T['mass_kg']:.0f} g incl. its clamp bracket with a 5 mm fin handle), friction {T['mu_s']:g} static / "
            f"{T['mu_k']:g} sliding; T650 landing gear; the real arm's q2 range [-20, 45] deg")


def task_text(T):
    return (f"grasp the fin and push the box {T['distance_m']:.2f} m along the table (toward -y) in {T['push_time_s']:g} s "
            f"(after a {T['settle_s']:g} s settle), the arm held at [{', '.join(f'{v:g}' for v in T['pose_deg'])}] deg, "
            "the claw world-held, the law in CONTACT from the grip to the end of the push")


def yaw_of(qwxyz):
    w, x, y, z = qwxyz.T
    return np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def discover():
    out = []
    for p in sorted(glob.glob(os.path.join(BP, "*_push_run*.npz"))):
        m = NAME.match(os.path.basename(p)[:-4])
        if m:
            out.append((m.group(1), int(m.group(2)), p))
    return out


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
    t_end_flown = phases[LAST][1]
    tu = np.arange(t0, t_end_flown + DT, DT)
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
    # the coordinated-admittance rig (Liu et al. 2024): the platform reference is the planner node's
    # own base command (cadm record, +2 recorder columns: h1 [26..32]), the arm reference its joint
    # command, the attitude reference the flight law's eta_d (debug [29..31] + 2); the mission stream's
    # x_cd chain is never flown there (that split is the paper's eq 38, decided by its MPC)
    cadm = d["cadm"] if "cadm" in d.files else np.zeros((0, 0))
    coordinated = cadm.size > 1 and cadm.shape[1] >= 33
    if coordinated:
        from fsc_aerial_manipulation.robotic_arm.utils_controller import coordinated_admittance as CA
        xb_cmd = interp(tu, cadm[:, 0], cadm[:, 26:29])
        q_d = interp(tu, cadm[:, 0], cadm[:, 29:33])
        dbg = d["dbg"]
        eta_d = interp(tu, dbg[:, 0], dbg[:, 31:34])
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
    # the impedance residual (pl_metrics.py), with the gains of the yaml the run flew
    ypath = path[:-4] + ".yaml"
    if not os.path.isfile(ypath):
        raise SystemExit(f"{path}: no {os.path.basename(ypath)} beside it (the impedance gains)")
    gains = PM.yaml_gains(ypath)
    task = task_of(ypath, path[:-4] + ".args")
    imp = PM.analyse(path, gains, FC_HZ)
    # the box (ground truth = obj_0, what the mocap carries)
    box = d["box"]
    bs = box[box[:, 0] >= marks["push:start"]][0]; be = box[box[:, 0] >= marks["push:end"]][0]
    ex0 = box[box[:, 0] >= marks["exit:start"]][0]; ex1 = box[box[:, 0] >= marks["exit:end"]][0]
    boxm = dict(pushed_mm=float(1e3 * np.linalg.norm(be[1:3] - bs[1:3])),
                pushed_xyz_mm=(1e3 * (be[1:4] - bs[1:4])).tolist(),
                yaw_change_deg=float(yaw_of(be[None, 4:8])[0] - yaw_of(bs[None, 4:8])[0]),
                exit_move_mm=float(1e3 * np.linalg.norm(ex1[1:3] - ex0[1:3])),
                final_pos_m=box[-1, 1:4].tolist(), start_pos_m=box[0, 1:4].tolist(),
                tilt_max_deg=float(np.degrees(np.arccos(np.clip(1 - 2 * (box[:, 5] ** 2 + box[:, 6] ** 2), -1, 1))).max()))
    return dict(d=d, marks=marks, phases=phases, timetable=np.asarray(d["timetable"], float), late=late, t0=t0, tu=tu, P=P, Qu=Qu, q=q, q_d=q_d,
                xb=xb, x_cd=x_cd, xc=xc, r_ed=r_ed, re=re, att_err=att_err, az=az, az_ref=az_ref,
                ee_head_err=ee_head_err, e_pos=e_pos, e_ee=e_ee, e_q=e_q, eps=eps, tilt=tilt, yaw=yaw,
                yaw_ref=yaw_ref, box=box, boxm=boxm, od=od, ref=ref, js=js, order=order, imp=imp, gains=gains, yaml=ypath,
                coordinated=coordinated, task=task)


def task_of(ypath, apath):
    """The scene / task numbers a run flew: the planner's push_pull_* block of its yaml and the
    fly.sh settings (.args: MASS, DIST, MU_S, MU_K, BOX_Y0). Missing .args (the 2026-10-09/10 runs)
    -> the 07 scene's defaults of that time (400 g, mu 0.29)."""
    def num(s):
        s = s.strip()
        if s.startswith("["):
            return [float(x) for x in s.strip("[]").split(",") if x.strip()]
        try:
            return float(s)
        except ValueError:
            return s.strip("'\"")
    pp = {}
    for line in open(ypath):
        mm = re.match(r"^\s*push_pull_([a-z0-9_]+):\s*([^#\n]*)", line)
        if mm:
            pp[mm.group(1)] = num(mm.group(2))
    args = {}
    if os.path.isfile(apath):
        for line in open(apath):
            if "=" in line:
                k, v = line.strip().split("=", 1)
                args[k] = v
    tab = pp.get("table", [1.125, 1.275, -0.5, 0.5, 0.80])
    # THE PUSH SPEED (2026-10-21): a yaml with push_pull_push_mean_speed has no push time --
    # the planner's rule, max(|distance| / speed, floor); older runs carry push_pull_push_time_s
    dist = float(pp.get("push_distance", 0.50))
    if float(pp.get("push_mean_speed", 0.0) or 0.0) > 0.0:
        push_time = max(abs(dist) / float(pp["push_mean_speed"]), float(pp.get("push_min_time_s", 0.0)))
    else:
        push_time = float(pp.get("push_time_s", 24.0))
    return dict(mass_kg=float(args.get("MASS", 0.400)), mu_s=float(args.get("MU_S", 0.29)),
                box_y0_m=float(args["BOX_Y0"]) if "BOX_Y0" in args else float("nan"),
                release_at_open=args.get("RELEASE", "0") == "1",
                mu_k=float(args.get("MU_K", 0.29)), distance_m=dist,
                push_time_s=push_time, settle_s=float(pp.get("push_settle_s", 1.5)),
                ramp_s=float(pp.get("push_ramp_s", 0.0)),
                pose_deg=pp.get("push_pose_deg", [0.0, 26.0, 34.0, 0.0]),
                approach_back_m=float(pp.get("approach_back", 0.12)), approach_dz_m=float(pp.get("approach_dz", 0.10)),
                exit_dz_m=float(pp.get("exit_dz", 0.20)), ee_offset_m=pp.get("ee_offset", [0.0, 0.0, 0.183]),
                land_start=pp.get("land_start", []), land=pp.get("land", []),
                table_xmin=tab[0], table_xmax=tab[1], table_ymin=tab[2], table_ymax=tab[3], table_top_z=tab[4],
                planner=pp)


def release_margin(R):
    """The release window's joint margins (2026-10-17): from the jaws opening (release start) to the end
    of the exit, q2 rises toward its +45 deg stop and q3 falls toward 0 / the planner's 10 deg guard as the
    open claw (still world-held) absorbs the base's drift. One results attempt (run 3, attempt 1) buckled
    here and flipped; these are the kept runs' margins."""
    a_, b_ = R["phases"]["release"][0], R["phases"]["exit"][1]
    m = (R["tu"] >= a_) & (R["tu"] < b_)
    qd = np.degrees(R["q"][m])
    return dict(release_q2_max_deg=float(qd[:, 1].max()), release_q3_min_deg=float(qd[:, 2].min()))


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


def imp_metrics(imp):
    """The impedance residual per window, flat (MATLAB-friendly) fields."""
    out = {}
    for w in IMP_WINDOWS:
        v = imp["imp"].get(w)
        if v is None:
            continue
        out[w] = dict(T_s=float(v["T"]), e_imp_rms_n=float(v["e_xyz"][0]), e_imp_max_n=float(v["e_xyz"][1]),
                      e_imp_xyz_rms_n=[float(x) for x in v["e_comp"]],
                      e_imp_psi_rms_nm=float(v["e_psi"][0]), e_imp_psi_max_nm=float(v["e_psi"][1]),
                      F_ext_rms_n=float(v["F_xyz"][0]), F_ext_max_n=float(v["F_xyz"][1]),
                      M_ext_psi_rms_nm=float(v["F_psi"][0]),
                      e_y_rms_mm=float(v["ey_mm"][0]), e_y_max_mm=float(v["ey_mm"][1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--paper", default="")
    ap.add_argument("--figdir", default=os.path.join(BP, "figures"))
    a = ap.parse_args()
    import scipy.io
    runs = discover()
    if not runs:
        sys.exit(f"no runs in {BP}")
    os.makedirs(BP_MAT, exist_ok=True)
    os.makedirs(TABLES, exist_ok=True); os.makedirs(a.figdir, exist_ok=True)
    rows, loaded = [], {}
    all_R = {(m, k): load_run(p) for m, k, p in runs}
    for (m, k), R in all_R.items():
        if R["late"]:
            print(f"WARNING {m} run {k}: LATE steps {R['late']} -- its phases are not on the timetable")
    # The methods are ALIGNED AT READY (2026-10-10): from Ready on every rig flies the same slots,
    # but the first transit's length is each rig's own (the coordinated-admittance rig flies gentler
    # planner bounds and holds 15 s before Ready: --slot-ready 30 vs 11). t = 0 is the reference
    # method's (ORDER[0]) first-transit start; a method whose transit is longer starts before 0 and
    # the common window drops that part of it.
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
    if max(chk.values()) > PHASE_ALIGN_TOL_S:
        sys.exit(f"phase starts differ by up to {max(chk.values()):.2f} s across runs (> {PHASE_ALIGN_TOL_S}) -- "
                 "not one timetable from Ready on; re-fly or check the slots")
    for m_, Rs in by_method.items():
        g0 = [R["phases"]["go_to_start"][0] - R["t0"] for R in Rs]
        if max(g0) - min(g0) > PHASE_ALIGN_TOL_S:
            sys.exit(f"{m_}: its own first-transit starts differ by {max(g0) - min(g0):.2f} s -- re-fly")
    T_common = min(R["phases"][LAST][1] - R["t0"] for R in all_R.values())
    PH = {key: (float(rel[key].mean()), None) for key in PHASE_KEYS}
    for i, key in enumerate(PHASE_KEYS):
        PH[key] = (PH[key][0], PH[PHASE_KEYS[i + 1]][0] if i + 1 < len(PHASE_KEYS) else T_common)
    print(f"window: the whole mission, 0 .. {T_common:.2f} s (the shortest run); phases",
          {k: (round(a_, 2), round(b_, 2)) for k, (a_, b_) in PH.items()})
    for method, k, path in runs:
        R = all_R[(method, k)]
        name = f"{method}_push_run{k}"
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
                                **metrics(R, m))
        whole = np.ones(len(tu), bool)
        r = metrics(R, fixed)
        r_whole = metrics(R, whole)
        r_whole["window_s"] = float(whole.sum() * DT)
        R["phase_id"] = phase_id
        im = imp_metrics(R["imp"])
        row = dict(name=name, method=method, run=k, total_window_s=r["window_s"], **r,
                   e_imp_slide_rms_n=im["SLIDE"]["e_imp_rms_n"], e_imp_slide_max_n=im["SLIDE"]["e_imp_max_n"],
                   e_imp_contact_rms_n=im["CONTACT"]["e_imp_rms_n"], e_imp_contact_max_n=im["CONTACT"]["e_imp_max_n"],
                   e_imp_hold_rms_n=im["HOLD"]["e_imp_rms_n"], e_imp_release_rms_n=im["RELEASE"]["e_imp_rms_n"],
                   e_imp_slide_psi_rms_nm=im["SLIDE"]["e_imp_psi_rms_nm"],
                   F_ext_slide_rms_n=im["SLIDE"]["F_ext_rms_n"], e_y_slide_rms_mm=im["SLIDE"]["e_y_rms_mm"],
                   box_pushed_mm=R["boxm"]["pushed_mm"], box_yaw_change_deg=R["boxm"]["yaw_change_deg"],
                   box_exit_move_mm=R["boxm"]["exit_move_mm"], box_tilt_max_deg=R["boxm"]["tilt_max_deg"],
                   **release_margin(R),
                   whole_mission=r_whole, per_leg=per_leg, impedance=im)
        rows.append(row); loaded[name] = R
        print(f"{name:28s} win {r['window_s']:5.1f} s  eps max/rms {r['eps_uav_max_mm']:6.1f}/{r['eps_uav_rms_mm']:5.1f} mm "
              f"rho {r['rho_uav_max']:.3f}/{r['rho_uav_rms']:.3f}  plat {r['platform_pos_mm']:5.1f}  EE {r['ee_pos_mm']:5.1f} mm "
              f"head {r['ee_heading_deg']:.2f}  e_imp slide {row['e_imp_slide_rms_n']:.3f} N  box {row['box_pushed_mm']:.0f} mm")
        d = R["d"]
        ser = R["imp"]["series"]
        meta = dict(name=name, method=method, method_long=METHODS[method][1], run_index=k,
                    plant="Isaac Sim, sim-to-real mirror plant (config A), headless at real time (RTF 1), raw mocap feedback",
                    scene=scene_text(R["task"]), task=task_text(R["task"]),
                    task_values=dict(box_mass_kg=R["task"]["mass_kg"], friction_static=R["task"]["mu_s"],
                                     friction_kinetic=R["task"]["mu_k"], push_distance_m=R["task"]["distance_m"],
                                     push_time_s=R["task"]["push_time_s"], push_settle_s=R["task"]["settle_s"],
                                     push_pose_deg=np.array(R["task"]["pose_deg"], float),
                                     table_x_m=np.array([R["task"]["table_xmin"], R["task"]["table_xmax"]]),
                                     table_y_m=np.array([R["task"]["table_ymin"], R["task"]["table_ymax"]]),
                                     table_top_z_m=R["task"]["table_top_z"],
                                     land_start_xyz_yawdeg=np.array(R["task"]["land_start"], float),
                                     land_xyz_yawdeg=np.array(R["task"]["land"], float)),
                    phases=dict(key=np.array(PHASE_KEYS, dtype=object),
                                label=np.array([PHASE_LABEL[x] for x in PHASE_KEYS], dtype=object),
                                start_s=np.array([R["phases"][x][0] - R["t0"] for x in PHASE_KEYS]),
                                end_s=np.array([R["phases"][x][1] - R["t0"] for x in PHASE_KEYS])),
                    timetable_slots_s={nm: float(x) for nm, x in zip(SLOT_NAMES, R["timetable"])},
                    late_steps=np.array([f"{n} {x:.2f} s" for n, x in R["late"]], dtype=object),
                    window=f"the whole mission, t = 0 (start of go to start) to {T_common:.2f} s (end of land, shortest run)",
                    impedance_windows=dict(CONTACT="ready end -> exit start (the law in contact)",
                                           HOLD="ready end -> slide start (the grip and the 1.5 s settle)",
                                           SLIDE=f"the {R['task']['push_time_s']:g} s move", RELEASE="push end -> exit start"),
                    impedance_gains=dict(M_d=R["gains"]["my"], D_d=R["gains"]["dy"], K_d=R["gains"]["ky"],
                                         channels="x y z [N] heading [N.m]", lowpass_hz=FC_HZ),
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
                   box=dict(t=R["box"][:, 0], pos_m=R["box"][:, 1:4], quat_wxyz=R["box"][:, 4:8]),
                   claw_truth=dict(t=d["claw"][:, 0], pos_m=d["claw"][:, 1:4]),
                   base_truth=dict(t=d["base_truth"][:, 0], pos_m=d["base_truth"][:, 1:4], quat_wxyz=d["base_truth"][:, 4:8]),
                   contact_truth=dict(t=ct[:, 0], vehicle_normal_n=ct[:, 1:4], vehicle_friction_n=ct[:, 4:7],
                                      table_normal_n=ct[:, 7:10], table_friction_n=ct[:, 10:13],
                                      vehicle_moment_normal_nm=ct[:, 13:16], vehicle_moment_friction_nm=ct[:, 16:19],
                                      note="PhysX contact impulses on the box averaged over each publish interval, "
                                           "'the box is actor0' sign convention (pl_metrics.py calibrates it on the table)"),
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
        scipy.io.savemat(os.path.join(BP_MAT, name + ".mat"),
                         {"run": dict(meta=meta, metrics=dict(window=r, whole_recording=r_whole, per_phase=per_leg,
                                                              impedance=im, box=R["boxm"]),
                                      tracking=tracking, impedance=impedance, raw=raw)},
                         do_compression=True, long_field_names=True, oned_as="column")

    # ---- mean over runs ------------------------------------------------------
    keys = ["eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms", "platform_pos_mm", "platform_heading_deg",
            "platform_roll_deg", "platform_pitch_deg", "ee_pos_mm", "ee_heading_deg", "total_window_s",
            "e_imp_slide_rms_n", "e_imp_slide_max_n", "e_imp_contact_rms_n", "e_imp_contact_max_n", "e_imp_hold_rms_n",
            "e_imp_release_rms_n", "e_imp_slide_psi_rms_nm", "F_ext_slide_rms_n", "e_y_slide_rms_mm",
            "box_pushed_mm", "box_yaw_change_deg", "box_exit_move_mm", "box_tilt_max_deg", "tilt_max_deg",
            "release_q2_max_deg", "release_q3_min_deg"]
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
    with open(os.path.join(TABLES, "box_pushing_sim.json"), "w") as f:
        json.dump(dict(runs=rows, mean_over_runs=summary, L_arm_m=L_ARM), f, indent=1, default=float)
    cols = ["method", "runs", "total_window_s", "eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms",
            "platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
            "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg",
            "q4_deg", "platform_pos_mm", "ee_pos_mm", "e_imp_slide_rms_n", "e_imp_slide_max_n", "e_imp_slide_x_rms_n",
            "e_imp_slide_y_rms_n", "e_imp_slide_z_rms_n", "e_imp_slide_psi_rms_nm", "e_imp_contact_rms_n",
            "e_imp_contact_max_n", "e_imp_hold_rms_n", "e_imp_release_rms_n", "F_ext_slide_rms_n", "e_y_slide_rms_mm",
            "box_pushed_mm", "box_yaw_change_deg", "box_exit_move_mm", "box_tilt_max_deg", "tilt_max_deg",
            "release_q2_max_deg", "release_q3_min_deg"]
    with open(os.path.join(TABLES, "box_pushing_sim.csv"), "w") as f:
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
    scipy.io.savemat(os.path.join(BP_MAT, "box_pushing_metrics.mat"),
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
    open(os.path.join(TABLES, "box_pushing_sim.tex"), "w").write(tex)
    figs = make_figures(rows, loaded, a.figdir, PH, T_common)
    write_readme(rows, summary, PH, T_common, all_R)
    # the Python scorer's table and figures beside the MATLAB data, to check a MATLAB table / plot against
    import shutil
    refdir = os.path.join(BP_MAT, "reference_outputs")
    os.makedirs(refdir, exist_ok=True)
    for f_ in [os.path.join(TABLES, "box_pushing_sim." + e) for e in ("csv", "tex")] + list(figs.values()):
        shutil.copy2(f_, refdir)
    if a.paper:
        splice_paper(a.paper, tex, figs)
    print("wrote", os.path.join(TABLES, "box_pushing_sim.{csv,json,tex}"), "and", a.figdir)


def colstruct(rows, keys):
    """A MATLAB struct of columns: numbers as double arrays, strings as cell arrays."""
    out = {}
    for k in keys:
        vals = [r[k] for r in rows]
        out[k] = np.array(vals, dtype=object) if isinstance(vals[0], str) else np.array(vals, dtype=float)
    return out


def failed_attempts():
    """Every failed attempt under box_pushing/failed/ (the npz the driver saved): what stopped it, when
    relative to the jaws opening for the release, and what the vehicle did afterwards (peak tilt, final
    height) -- a planner ABORT that lands safely and one that ends upside down look the same in
    campaign.jsonl."""
    out = []
    for d_ in sorted(glob.glob(os.path.join(BP, "failed", "*_attempt*"))):
        npz = sorted(glob.glob(os.path.join(d_, "*.npz")))
        if not npz:
            out.append([f"`{os.path.basename(d_)}`", "no npz (never flew)", "--", "--", "--", "--"])
            continue
        z = np.load(npz[0], allow_pickle=True)
        mk = {str(n): float(t) for t, n in zip(z["marks_t"], z["marks_name"])}
        od = z["odom"]
        w, x, y, _ = od[:, 7:11].T
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
        reason = str(z["reason"]) if "reason" in z.files else ""
        mtrip = re.search(r"\[\s*([0-9.]+)s\] push_pull: ABORTING", open(os.path.join(d_, os.path.basename(npz[0])[:-4] + ".log")).read()) \
            if os.path.isfile(os.path.join(d_, os.path.basename(npz[0])[:-4] + ".log")) else None
        t_trip = float(mtrip.group(1)) if mtrip else float("nan")
        rel = mk.get("release:start")
        when = (f"{t_trip - rel:+.2f} s after the jaws opened" if rel is not None and np.isfinite(t_trip)
                else (f"at {t_trip:.1f} s" if np.isfinite(t_trip) else "--"))
        after = tilt[od[:, 0] >= t_trip] if np.isfinite(t_trip) else tilt
        tmax = float(after.max()) if after.size else float(tilt.max())
        outcome = ("**the vehicle flipped**" if tmax > 90 else "landed upright") + \
                  f" (tilt max {tmax:.0f} deg, final height {od[-1, 3]:.2f} m)"
        what_ = reason.split(" -- ")[0]
        what_ = ("planner abort " + what_.split("ABORTING ", 1)[1]) if "ABORTING " in what_ else what_
        out.append([f"`{os.path.basename(d_)}`", what_[:120],
                    when, outcome])
    return out


def write_readme(rows, summary, PH, T_common, all_R):
    """matlab_simulation_data/box_pushing/README.md (generated) + the as-flown configs."""
    import shutil
    sys.path.insert(0, HERE)
    from readme_text import _md_table
    cfg_dir = os.path.join(BP_MAT, "configs")
    os.makedirs(cfg_dir, exist_ok=True)
    # the scene / plant / timetable text comes from the PROPOSED rig's first run (ORDER[0]); the
    # comparison rig's own gains get their own line below (2026-10-17: the first run in dict order was
    # the coordinated rig's, whose claw-frame admittance was printed as "the law's EE task")
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
    jl = os.path.join(BP, "campaign.jsonl")
    att = [json.loads(l) for l in open(jl)] if os.path.isfile(jl) else []
    first = min((x["time"] for x in att), default="?")[:16].replace("T", " ")
    last = max((x["time"] for x in att), default="?")[:16].replace("T", " ")
    names = {m_: METHODS[m_][0] for m_ in ORDER}
    run_rows = [[names[r["method"]], f"`{r['name']}.mat`", f"{r['eps_uav_max_mm']:.1f}", f"{r['rho_uav_max']:.3f}",
                 f"{r['eps_uav_rms_mm']:.1f}", f"{r['rho_uav_rms']:.3f}", f"{r['ee_pos_mm']:.1f}",
                 f"{r['e_imp_slide_rms_n']:.3f}", f"{r['e_imp_contact_rms_n']:.3f}", f"{r['box_pushed_mm']:.0f}"]
                for r in sorted(rows, key=lambda x: (ORDER.index(x["method"]), x["run"]))]
    mean_rows = [[names[x["method"]], str(x["runs"]), f"{x['eps_uav_max_mm']:.1f}", f"{x['rho_uav_max']:.3f}",
                  f"{x['eps_uav_rms_mm']:.1f}", f"{x['rho_uav_rms']:.3f}", f"{x['ee_pos_mm']:.1f}",
                  f"{x['e_imp_slide_rms_n']:.3f}", f"{x['e_imp_contact_rms_n']:.3f}", f"{x['box_pushed_mm']:.0f}"] for x in summary]
    att_rows = [[names.get(m, m), str(len([y for y in att if y["method"] == m])),
                 str(sum(1 for y in att if y["method"] == m and y["status"] == "completed")),
                 "; ".join(sorted(set(y["why"] for y in att if y["method"] == m and y["status"] != "completed" and y["why"]))) or "--"]
                for m in sorted(set(y["method"] for y in att))]
    T = first_R["task"]
    present = [m_ for m_ in ORDER if any(r["method"] == m_ for r in rows)]
    coord = "coordinated_admittance" in present
    pose = ", ".join(f"{v:g}" for v in T["pose_deg"])
    what = {"go_to_start": f"fly from the hover to the start waypoint (the claw {T['approach_back_m']:.2f} m behind and "
                           f"{T['approach_dz_m']:.2f} m above the fin, the arm already in the push pose); hover",
            "ready": "open the gripper, hover (CoM-anchored), descend diagonally onto the fin; the claw world-held from its arrival",
            "grasp": "wait for the claw within 2 cm / 3 mm across the fin, close the gripper on the fin (the jaws' stall)",
            "push": f"the law in CONTACT: {T['settle_s']:g} s settle, then the {T['distance_m']:.2f} m slide over "
                    f"{T['push_time_s']:g} s, the arm held still; hold on the handle",
            "release": "Exit To Push: the planner releases the claw (its reference back to the CoM anchor, the contact "
                       "phase off), then the gripper opens; the exit starts on its OPEN report",
            "exit": f"climb {T['exit_dz_m']:.2f} m with the jaws open, fold the arm home; hover",
            "go_to_land_start": "one planner transition off the table to the landing hover "
                                + (f"({T['land_start'][0]:g}, {T['land_start'][1]:g}, {T['land_start'][2]:g}) m, "
                                   if len(T["land_start"]) >= 3 else "")
                                + "heading turning 270 deg clockwise; hover",
            "land": "straight down to the land hover"
                    + (f" at z {T['land'][2]:g} m" if len(T["land"]) >= 3 else "")
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
    box_rows = [[f"`{x['name']}`", f"{x['box_pushed_mm']:.0f}", f"{x['box_yaw_change_deg']:+.1f}",
                 f"{x['box_tilt_max_deg']:.2f}", f"{x['box_exit_move_mm']:.1f}", f"{x['tilt_max_deg']:.1f}",
                 f"{x['release_q2_max_deg']:.1f}", f"{x['release_q3_min_deg']:.1f}"]
                for x in sorted(rows, key=lambda x: (ORDER.index(x["method"]), x["run"]))]
    g = first_R["gains"]
    slots = first_R["timetable"]
    n_runs = len(rows)
    fails = failed_attempts()
    first_name = sorted(rows, key=lambda x: (ORDER.index(x["method"]), x["run"]))[0]["name"]
    if coord:
        method_lines = [
            "- **`<method>`:** `whole_body_l1` = Proposed (whole-body 4-D L1 impedance); `coordinated_admittance` =",
            "  Liu et al., CEP 146 (2024) 105898 (decoupled UAV PD + momentum wrench estimator + admittance on the",
            "  end-effector reference + force-adaptive kinematic MPC; position-mode arm), implemented as an independent",
            "  comparison rig -- its platform / arm references in `run` are that rig's own planner commands.", ""]
        align_lines = [
            "`t = 0` is the start of the Proposed rig's first transit; the methods are ALIGNED AT READY (from Ready on",
            "every rig flies the same slots; the coordinated rig's longer first transit starts before t = 0 and the",
            "window drops that part; `meta.t_shift_s` is the shift applied)."]
    else:
        method_lines = [
            "- **`<method>`:** `whole_body_l1` = the proposed whole-body 4-D L1 impedance controller (the only method in",
            "  this folder).", ""]
        align_lines = ["`t = 0` is the start of the first transit (go to start) of every run."]
    L = [
        "# Simulation results, MATLAB data (box pushing)", "",
        "This is the data behind the paper's box-pushing table (`tab:sim_box_push`) and its figures: the",
        "physical-interaction task, flown in Isaac Sim by the proposed whole-body L1 impedance controller",
        f"({n_runs} runs of one mission). It is one task folder of `matlab_simulation_data/`, self-contained: these",
        "files plus MATLAB are all you need. The free-flight and pick-and-place data are in the folders beside it,",
        "with the same plant, the same `run` struct layout and the same notation; this folder adds the IMPEDANCE",
        "RESIDUAL and the box.", "",
        "## Files", "",
        "```",
        "<method>_push_run<k>.mat                 one run, struct `run`",
        "box_pushing_metrics.mat                  metrics_mean = the table (mean over runs), metrics_runs = every run,",
        "                                         metrics_phase = every run x phase, metrics_impedance = every run x",
        "                                         contact window, phases = the common phase table, window_s, L_arm_m",
        "configs/<method>_sim_box_pushing.yaml    the controller + planner (push-and-pull block) + plant, as flown",
        "reference_outputs/                       the same table and figures drawn by the Python scorer",
        "                                         (box_pushing_sim.csv / .tex, sim_box_push_*.png): check a MATLAB",
        "                                         table or plot against them",
        "```", "",
        *method_lines,
        "## Quick start in MATLAB", "",
        "```matlab",
        "S = load('box_pushing_metrics.mat');",
        "T = struct2table(S.metrics_mean)                 % THE TABLE: one row per method, mean over the runs",
        "R = struct2table(S.metrics_runs);                % every run (the spread between runs)",
        "P = S.phases;                                    % the eight phases, common to every run",
        f"r = load('{first_name}.mat').run;",
        "tr = r.tracking;  w = tr.in_window > 0;          % 100 Hz, t = 0 at the start of go to start",
        "",
        "% (1) the platform deviation with the phase bands",
        "figure; hold on",
        "for i = 1:numel(P.start_s)                       % phase bands, alternating shades",
        "    c = 0.90 + 0.07*mod(i+1, 2);",
        "    patch([P.start_s(i) P.end_s(i) P.end_s(i) P.start_s(i)], [0 0 200 200], c*[1 1 1], 'EdgeColor', 'none');",
        "    text(mean([P.start_s(i) P.end_s(i)]), 195, P.label{i}, 'HorizontalAlignment', 'center');",
        "end",
        "plot(tr.t(w), 1e3 * tr.eps_uav_m(w))             % ||eps_UAV|| [mm]",
        "xlim([0 S.window_s]); xlabel('Time (s)'); ylabel('||\\epsilon_{UAV}|| (mm)')",
        "yyaxis right; ylim([0 200] / (1e3 * S.L_arm_m)); ylabel('\\rho_{UAV}')",
        "",
        "% (2) the contact force and the impedance residual over the contact window (x, y, z in N; col 4 = heading, N.m)",
        "im = r.impedance;  W = im.windows;",
        "figure; tiledlayout(3, 1)",
        "nexttile; plot(im.t, im.F_ext(:,1:3)); ylabel('F_{ext} (N)'); legend('x', 'y', 'z')",
        "nexttile; plot(im.t, im.e_imp(:,1:3)); ylabel('e_{imp} (N)')",
        "nexttile; plot(im.t, vecnorm(im.e_imp(:,1:3), 2, 2)); ylabel('||e_{imp}|| (N)'); xlabel('Time (s)')",
        "s = strcmp(W.name, 'SLIDE');  for k = 1:3, nexttile(k); xline([W.start_s(s) W.end_s(s)], ':'); end",
        "",
        "% (3) 3-D: references (dashed), measured platform and end effector, the box (ground truth), the table",
        "figure; hold on; grid on; axis equal; view(-50, 28)",
        "plot3(tr.platform_pos_ref_m(w,1), tr.platform_pos_ref_m(w,2), tr.platform_pos_ref_m(w,3), 'k--')",
        "plot3(tr.ee_pos_ref_m(w,1), tr.ee_pos_ref_m(w,2), tr.ee_pos_ref_m(w,3), '--', 'Color', [0.45 0.45 0.45])",
        "plot3(tr.platform_pos_m(w,1), tr.platform_pos_m(w,2), tr.platform_pos_m(w,3))",
        "plot3(tr.ee_pos_m(w,1), tr.ee_pos_m(w,2), tr.ee_pos_m(w,3), ':')",
        "tb = r.raw.box.t - r.meta.t0_driver_s;  wb = tb >= 0 & tb <= S.window_s;   % raw streams: driver clock",
        "plot3(r.raw.box.pos_m(wb,1), r.raw.box.pos_m(wb,2), r.raw.box.pos_m(wb,3))",
        "tv = r.meta.task_values;  tx = tv.table_x_m([1 2 2 1 1]);  ty = tv.table_y_m([1 1 2 2 1]);",
        "plot3(tx, ty, tv.table_top_z_m * ones(1, 5), 'Color', [0.53 0.6 0.67])     % the table top",
        "",
        "% (4) the error grid: platform / end effector xyz in mm, headings and joints in deg",
        "E = {1e3*tr.platform_pos_err_m, tr.platform_att_err_deg(:,3), 1e3*tr.ee_pos_err_m, tr.ee_heading_err_deg, tr.q_err_deg};",
        "",
        "% (5) one phase's numbers, e.g. the push, for every run",
        "Ph = struct2table(S.metrics_phase);  Ph(strcmp(Ph.phase, 'push'), :)",
        "```", "",
        "## Timing: one timetable for every run", "",
        "The flight driver starts every step at the same mission time on every run",
        f"(`pl_mission.py --timetable`, slots {', '.join(f'{x:g}' for x in slots)} s, each from the previous step's",
        "start; `meta.timetable_slots_s` names them). The phases are contiguous: each runs from its first step's start",
        "to the next phase's start. Only the GRASP phase's own length varies (the claw's arrival within 2 cm / 3 mm",
        "across the fin, then the jaws' close), because a gripped box must be pushed in CONTACT at once -- so the",
        "phases after it shift by that much between runs.",
        *align_lines,
        "The phase table below is the mean over the runs; every run's phase starts from Ready on agree with it to "
        f"{max(max(abs(R['phases'][k][0] - R['t0'] - PH[k][0]) for k in PHASE_KEYS if k != 'go_to_start') for R in all_R.values()):.2f} s.", "",
        _md_table(["#", "phase", "start [s]", "end [s]", "duration [s]", "what happens"], ph_rows), "",
        f"- **Window:** every metric covers the whole mission, 0 to {T_common:.2f} s. `run.tracking.in_window`",
        f"  marks it; `run.tracking.phase_id` (1..{len(PHASE_KEYS)}) says which phase each sample is in.",
        "- **Contact windows** (`run.impedance.windows`, `run.metrics.impedance.<window>`): CONTACT = ready end to",
        f"  exit start (the law's contact phase, the claw world-held); HOLD = the grip and the {T['settle_s']:g} s settle;",
        f"  SLIDE = the {T['push_time_s']:g} s move; RELEASE = push end to exit start (the jaws open).", "",
        "## Simulation index", "",
        f"Flown {first} to {last} (attempt end times), headless Isaac Sim at real time, raw mocap-emulator feedback.", "",
        _md_table(["method", "file", "eps max [mm]", "rho max", "eps rms [mm]", "rho rms", "EE RMSE [mm]",
                   "e_imp slide rms [N]", "e_imp contact rms [N]", "box pushed [mm]"], run_rows), "",
        "The table is the mean over the runs:", "",
        _md_table(["method", "runs", "eps max [mm]", "rho max", "eps rms [mm]", "rho rms", "EE RMSE [mm]",
                   "e_imp slide rms [N]", "e_imp contact rms [N]", "box pushed [mm]"], mean_rows), "",
        "The impedance residual per contact window (`metrics_impedance`):", "",
        _md_table(["run", "window", "T [s]", "e_imp rms [N]", "e_imp max [N]", "x / y / z rms [N]", "heading rms [N.m]",
                   "F_ext rms [N]", "e_y rms [mm]"], imp_rows), "",
        "The box and the vehicle (`metrics.box`, `metrics.window`):", "",
        _md_table(["run", "box pushed [mm]", "box yaw change [deg]", "box tilt max [deg]",
                   "box moved in the exit [mm]", "vehicle tilt max [deg]", "q2 max, release + exit [deg]",
                   "q3 min, release + exit [deg]"], box_rows), "",
        "The last two columns are the release window's joint margins: q2's stop is +45 deg, the planner's arm",
        "guard aborts below q3 = 10 deg (see the attempts below).", "",
        "Attempts (a run is kept only if it completed, pushed the box >= 0.45 m and kept every step on its slot):", "",
        _md_table(["method", "attempts", "kept", "why the others were not kept"], att_rows) if att_rows else "--", "",
        *(["The failed attempts (kept whole in the repository under `results/simulation_results/box_pushing/failed/`):", "",
           _md_table(["attempt", "what stopped it", "when", "afterwards"], fails), ""] if fails else []),
        "## Scene and task", "",
        f"- **Table:** the HOOBRO console table (1.000 x 0.150 x 0.800 m), its length along world y: x "
        f"{T['table_xmin']:.3f}-{T['table_xmax']:.3f} m, y {T['table_ymin']:g}..{T['table_ymax']:g} m, top at "
        f"{T['table_top_z']:.2f} m.",
        f"- **Box:** the CAD box (240 x 160 x 100 mm) with its clamp bracket and 5 mm fin handle, **{1e3 * T['mass_kg']:.0f} g**",
        f"  in total, friction {T['mu_s']:g} static / {T['mu_k']:g} sliding ("
        + ("the real box on the experiment table, spring-gauge bench test 2026-10-17" if abs(T['mu_k'] - 0.25) < 1e-9
           else "the measured experiment table") + f"), {T['mass_kg'] * G * T['mu_k']:.2f} N to slide. It starts",
        ("  on the table's centre line, its back face "
         f"{T['table_ymax'] - T['box_y0_m'] - 0.12:.2f} m from the table's +y end, and is pushed toward -y "
         f"(to {T['table_ymax'] - T['box_y0_m'] - 0.12 + T['distance_m']:.2f} m). The sim claw cannot close below ~19 mm,"
         if T['box_y0_m'] == T['box_y0_m'] else
         "  on the table's centre line on its +y half and is pushed toward -y. The sim claw cannot close below ~19 mm,"),
        "  so the fin's bare part carries an invisible 20 mm grip shim (the real gripper's foam pads close on the",
        "  5 mm fin).",
        f"- **Task:** grasp the fin from above (go to start, ready, grasp), push the box {T['distance_m']:.2f} m along the",
        f"  table in {T['push_time_s']:g} s (a min-snap move after a {T['settle_s']:g} s settle) with the arm held still and the",
        "  claw held in the world, release, climb out, fly off the table to the landing hover, descend. The whole-body law's",
        "  contact phase (chi) is CONTACT from the jaws' grip to the end of the push.",
        f"- **Arm pose in the push:** [{pose}] deg (fold {T['pose_deg'][1] + T['pose_deg'][2]:g} deg); airframe: the T650",
        "  landing gear; the real arm's q2 range [-20, 45] deg is enforced as hard stops in the scene.",
        f"- **Safety guards (on in every attempt, both rigs; none tripped in the kept runs; {len(fails)} failed "
        f"attempt{'s' if len(fails) != 1 else ''} above):** the planner refuses a push that would leave the box",
        "  within 3 cm of the table edge, and aborts on a box lagging > 40 mm behind its plan, a box tilt > 2 deg,",
        "  q3 < 10 deg, a lost box feed, a task force > 5 N or a vehicle tilt > 15 deg.",
        f"- **Impedance, Proposed (the whole-body law's EE task):** M_d = diag({', '.join(f'{v:g}' for v in g['my'])}),",
        f"  D_d = diag({', '.join(f'{v:g}' for v in g['dy'])}), K_d = diag({', '.join(f'{v:g}' for v in g['ky'])})",
        "  on [x, y, z, heading] (kg, N.s/m, N/m; kg.m^2, N.m.s/rad, N.m/rad for the heading).",
        *([f"- **Admittance, Coordinated [Liu 2024] (eq 27, contact phase, claw frame [along, lateral, vertical]):** "
           f"M_d = diag({', '.join(f'{v:g}' for v in coord_R['gains']['my'][:3])}),",
           f"  D_d = diag({', '.join(f'{v:g}' for v in coord_R['gains']['dy'][:3])}), "
           f"K_d = diag({', '.join(f'{v:g}' for v in coord_R['gains']['ky'][:3])}) (kg, N.s/m, N/m); its contact phase runs",
           "  from the jaws' stall to the release. That rig flies the planner at 0.20 m/s / 0.08 m/s^2 / 0.20 rad/s",
           "  (the whole-body rig 0.30 / 0.15 / 0.30) and hovers 15 s before Ready, so its first transit starts",
           "  before t = 0; from Ready on both rigs fly the same timetable. Its box-lag guard trips are its",
           "  decoupled base lagging the planned push (2026-10-17 campaign: the box 35-48 mm behind its plan at the",
           "  push's peak speed, the whole-body rig 15-22 mm), not a jammed box. After the release its arm closes the remaining lag",
           "  with the jaws open around the fin and slides the box along the push (`box moved in the exit`)."]
          if coord_R is not None else []),
        "- **Plant:** the free-flight mirror plant -- motor delay, model mismatch, battery sag, a standing wrench",
        "  bias, imperfect joint actuation and emulated mocap feedback (the free-flight README explains each).",
        "  Every `sim_*` key as flown:", "",
        "```", *plant, "```", "",
        "- **The push-and-pull planner block as flown** (`configs/`):", "",
        "```", *task, "```", "",
        "## The struct `run`", "",
        _md_table(["field", "content"], [
            ["`meta`", "method, run index, `scene`, `task`, `task_values` (box mass, friction, push distance / time / "
                       "settle, push pose, the table's x / y range and top, Land Start, Land), plant, `phases` (key, label, "
                       "start_s, end_s of this run), `timetable_slots_s`, `late_steps` (empty), `window`, `impedance_windows`, "
                       "`impedance_gains` (M_d, D_d, K_d), `L_arm_m`, `base_com_model_m`, `t0_driver_s`"],
            ["`metrics`", "`window` = this run's table numbers over the window; `per_phase.<key>` = the same per phase; "
                          "`whole_recording` = over every sample; `impedance.<window>` = the impedance residual per contact "
                          "window (rms / max, per channel, with |F_ext| and |e_y|); `box` = pushed distance, yaw change, "
                          "movement during the exit, final position"],
            ["`tracking`", "100 Hz, `t` in s since the start of the mission; `in_window`, `phase_id`; measured, "
                           "reference (`_ref_`) and error (`_err_`) for the platform position, `eps_uav_m`, `rho_uav`, "
                           "the platform attitude, the system CoM, the end-effector position and heading, and the joints"],
            ["`impedance`", "the residual's time series over the CONTACT window (contact-report rate, ~62.5 Hz): `t`, `e_imp`, "
                            "`F_ext` (the true contact wrench on the claw), `rendered` (M_d edot_v + D_d e_v + K_d e_y), `e_y`, "
                            "and `windows` (name, start_s, end_s)"],
            ["`raw`", "every recorded stream on the driver's clock (subtract `meta.t0_driver_s` to get `tracking.t`): "
                      "`odom`, `joints`, `reference` (the planner's whole-body reference), `reference_derivatives`, `box` "
                      "(ground truth = the mocap's obj_0), `claw_truth`, `base_truth`, `contact_truth` (the box's PhysX "
                      "contact report), `marks` (every step's start/end), `events`"]]), "",
        "Units: positions in m (world ENU, z up) unless the field name says mm; angles in deg unless it says rad;",
        "forces in N, moments in N.m. Errors are measured − reference. `platform_att_err_deg` columns are",
        "`[roll pitch heading]`.", "",
        "## Paper notation", "",
        _md_table(["paper symbol", "meaning", "summary field (`metrics_mean`)", "run file"], [
            ["e<sub>imp</sub>", "F<sub>ext</sub> − (M<sub>d</sub> ė<sub>v</sub> + D<sub>d</sub> e<sub>v</sub> + K<sub>d</sub> e<sub>y</sub>), "
             "the impedance residual on the claw [N; heading N·m], rms over the slide / the whole contact",
             "`e_imp_slide_rms_n`, `e_imp_slide_max_n`, `e_imp_slide_{x,y,z}_rms_n`, `e_imp_slide_psi_rms_nm`, `e_imp_contact_rms_n`",
             "`impedance.e_imp` [N N N N·m], `metrics.impedance.<window>`"],
            ["F<sub>ext</sub>", "the true contact wrench on the claw (ground truth)", "`F_ext_slide_rms_n`", "`impedance.F_ext`"],
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
            ["q<sub>1</sub> … q<sub>4</sub>", "joint angles [deg]", "`q1_deg` … `q4_deg`", "`tracking.q_err_deg(:,1:4)`"],
            ["d<sub>box</sub>", f"the box's travel along the table during the push [mm] ({T['distance_m']:.2f} m commanded)",
             "`box_pushed_mm`", "`metrics.box.pushed_mm`, `raw.box`"]]), "",
        "- **Reference:** the planner's dynamically compatible whole-body reference. The platform reference",
        "  x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>) is r<sub>UAV</sub><sup>ref</sup>.",
        "  During the push the EE reference r<sub>e,d</sub> is the claw's planned path along the table (world-held).",
        "- **max / rms:** over the window; the table's other columns are RMS errors.",
        f"- **e<sub>imp</sub>:** every term through one zero-phase {FC_HZ:g} Hz low-pass; e<sub>y</sub> from the TRUE claw point",
        "  (ground truth) minus the streamed reference; e<sub>v</sub>, ė<sub>v</sub> its derivatives against the streamed",
        "  reference derivatives; F<sub>ext</sub> from the box's PhysX contact report (sign calibrated on the table's",
        "  normal force). A residual of 0 means the claw moved exactly as the designed impedance under the measured force.", ""]
    with open(os.path.join(BP_MAT, "README.md"), "w") as f:
        f.write("\n".join(L))
    print("wrote", os.path.join(BP_MAT, "README.md"))


def latex_table(summary, T_common, task=None):
    T = task or dict(push_time_s=24.0)
    coord = any(x["method"] == "coordinated_admittance" for x in summary)
    cols = ["e_imp_slide_rms_n", "e_imp_slide_max_n", "e_imp_contact_rms_n", "eps_uav_max_mm", "rho_uav_max", "eps_uav_rms_mm",
            "rho_uav_rms", "platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
            "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg", "q4_deg"]
    fmt = {c: ("%.3f" if c.startswith("rho") or c.startswith("e_imp") else "%.2f") for c in cols}
    vals = {s["method"]: {c: fmt[c] % s[c] for c in cols} for s in summary}
    lines = [r"\begin{table}[H]", r"\centering",
             r"\caption{Box pushing in simulation: impedance residual, platform deviation and RMSE of the tracked states over the whole mission.}",
             r"\label{tab:sim_box_push}", r"\footnotesize", r"\setlength{\tabcolsep}{3pt}", r"\begin{threeparttable}",
             r"\begin{tabular}{l c c c c c c c c c c c c c c c c c c c c c c}", r"\toprule",
             r"& \multicolumn{3}{c}{Impedance residual} & \multicolumn{4}{c}{UAV deviation} & \multicolumn{6}{c}{Platform} & \multicolumn{4}{c}{End-effector} & \multicolumn{4}{c}{Joint} \\",
             r"\cmidrule(lr){2-4} \cmidrule(lr){5-8} \cmidrule(lr){9-14} \cmidrule(lr){15-18} \cmidrule(lr){19-22}",
             r"Method & $\|e_{imp}\|_{\mathrm{rms}}^{slide}$ & $\|e_{imp}\|_{\max}^{slide}$ & $\|e_{imp}\|_{\mathrm{rms}}^{contact}$ & $\|\varepsilon_{UAV}\|_{\max}$ & $\rho_{UAV,\max}$ & $\|\varepsilon_{UAV}\|_{\mathrm{rms}}$ & $\rho_{UAV,\mathrm{rms}}$ & $r_{0,x}$ & $r_{0,y}$ & $r_{0,z}$ & $\psi_0$ & $\phi_0$ & $\theta_0$ & $r_{e,x}$ & $r_{e,y}$ & $r_{e,z}$ & $\psi_e$ & $q_1$ & $q_2$ & $q_3$ & $q_4$ \\",
             r" & (N) & (N) & (N) & (mm) & (--) & (mm) & (--) & (mm) & (mm) & (mm) & ($^\circ$) & ($^\circ$) & ($^\circ$) & (mm) & (mm) & (mm) & ($^\circ$) & ($^\circ$) & ($^\circ$) & ($^\circ$) & ($^\circ$) \\",
             r"\midrule"]
    for m in ORDER:
        if m not in vals:
            continue
        lines.append(f"{METHODS[m][0]} & " + " & ".join(vals[m][c] for c in cols) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\begin{tablenotes}", r"\footnotesize",
              r"\item[] Each entry is the mean over the completed runs. "
              r"$e_{imp} = \boldsymbol{F}_{ext} - (\boldsymbol{M}_d \dot{\boldsymbol{e}}_v + \boldsymbol{D}_d \boldsymbol{e}_v + \boldsymbol{K}_d \boldsymbol{e}_y)$ "
              r"is the impedance residual on the end effector (the measured contact force minus the force the designed impedance "
              r"would render for the measured deflection), given as the RMS and the maximum of its norm over the %g~s slide and the " % T["push_time_s"] +
              r"RMS over the whole contact phase (grasp to release). "
              r"$\|\varepsilon_{UAV}\| = \|\boldsymbol{r}_{UAV}^{ref} - \boldsymbol{r}_{UAV}\|$ is the platform's deviation from "
              r"its reference position and $\rho_{UAV} = \|\varepsilon_{UAV}\|/L$ its ratio to the arm's reach $L = 0.37$~m, "
              r"given as the maximum and the RMS over the window; the remaining columns are RMS tracking errors with the " +
              (r"symbols of Table~\ref{tab:sim_free_flight_tracking}. The window is the whole mission (%.1f~s), from the "
               r"start of the Proposed rig's first transit to the end of the landing leg; every run flies the same "
               r"timetable from Ready on (the methods are aligned at Ready; the coordinated rig's longer first transit starts before the window)." % T_common
               if coord else
               r"symbols of Table~\ref{tab:sim_free_flight_tracking}. The window is the whole mission (%.1f~s), from the "
               r"start of the first transit to the end of the landing leg; every run flies the same timetable." % T_common),
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
    # ---- 3-D trajectories ----
    fig = plt.figure(figsize=(7.2, 5.2)); ax = fig.add_subplot(111, projection="3d")
    # the table top as flown (the run's push_pull_table: x 1.200..1.350 since 2026-10-16, y -0.5..0.5,
    # z 0.80) and the box at its start / end
    Tk = sel[0][1]["task"]
    x0_, x1_, y0_, y1_, zt_ = Tk["table_xmin"], Tk["table_xmax"], Tk["table_ymin"], Tk["table_ymax"], Tk["table_top_z"]
    tx = [x0_, x1_, x1_, x0_, x0_]; ty = [y0_, y0_, y1_, y1_, y0_]
    ax.plot(tx, ty, [zt_] * 5, color="#8899aa", lw=0.9)
    for xx, yy in ((x0_, y0_), (x1_, y0_), (x1_, y1_), (x0_, y1_)):
        ax.plot([xx, xx], [yy, yy], [0.6, zt_], color="#8899aa", lw=0.6)   # from the plot's floor (3-D lines are not clipped)
    first = True
    for m, R in sel:
        ref_mask = (R["tu"] - R["t0"]) <= T_common + 1e-9
        if first:
            ax.plot(R["xb"][ref_mask, 0], R["xb"][ref_mask, 1], R["xb"][ref_mask, 2], "k--", lw=0.9, label="Platform ref.")
            ax.plot(R["r_ed"][ref_mask, 0], R["r_ed"][ref_mask, 1], R["r_ed"][ref_mask, 2], "--", color="0.45", lw=0.9, label="End-effector ref.")
            first = False
        ax.plot(R["P"][ref_mask, 0], R["P"][ref_mask, 1], R["P"][ref_mask, 2], color=COLORS[m], lw=1.2, label=f"{METHODS[m][0]} platform")
        ax.plot(R["re"][ref_mask, 0], R["re"][ref_mask, 1], R["re"][ref_mask, 2], color=COLORS[m], lw=1.0, ls=":", label=f"{METHODS[m][0]} end-effector")
        b = R["box"]
        ax.plot(b[:, 1], b[:, 2], b[:, 3], color="#A2142F", lw=1.0, label="Box (ground truth)" if m == sel[0][0] else None)
        for row in (b[0], b[-1]):
            cx, cy, cz = row[1:4]; yw = np.radians(yaw_of(row[None, 4:8])[0])
            c, s = np.cos(yw), np.sin(yw)
            cs = np.array([[-0.08, -0.12], [0.08, -0.12], [0.08, 0.12], [-0.08, 0.12], [-0.08, -0.12]])
            X = cx + c * cs[:, 0] - s * cs[:, 1]; Y = cy + s * cs[:, 0] + c * cs[:, 1]
            for zz in (cz - 0.05, cz + 0.05):
                ax.plot(X, Y, zz, color="#A2142F", lw=0.6)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_xlim(-0.6, 1.6); ax.set_ylim(-1.1, 1.1); ax.set_zlim(0.6, 1.6)
    ax.view_init(elev=28, azim=-50)
    ax.legend(loc="upper left", fontsize=8, ncol=2, frameon=False)
    ax.set_box_aspect((1, 1, 0.45))
    fig.tight_layout()
    p = os.path.join(figdir, "sim_box_push_trajectory_3d.png"); fig.savefig(p, dpi=220); plt.close(fig); out["traj"] = p

    def bands(ax, labels=False):
        for i, key in enumerate(PHASE_KEYS):
            a_, b_ = PH[key]
            ax.axvspan(a_, b_, color="0.90" if i % 2 == 0 else "0.97", lw=0)
            if labels:
                lab = PHASE_LABEL[key].replace(" ", "\n", 1) if (b_ - a_) < 8.0 else PHASE_LABEL[key]
                # a phase too narrow for its label (the ~1 s grasp) gets it one line lower
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
    p = os.path.join(figdir, "sim_box_push_tracking_error.png"); fig.savefig(p, dpi=220, bbox_inches="tight"); plt.close(fig); out["err"] = p
    # ---- UAV deviation ----
    fig, ax = plt.subplots(figsize=(7.2, 3.2), constrained_layout=True)
    bands(ax, labels=True)
    for m, R in sel:
        mk, t = tw(R)
        ax.plot(t, 1e3 * R["eps"][mk], color=COLORS[m], lw=0.9, label=METHODS[m][0])
    ax.set_xlabel("Time (s)"); ax.set_ylabel(r"$\|\varepsilon_{UAV}\|$ (mm)"); ax.grid(alpha=0.3)
    ax.set_ylim(0, 1.25 * ax.get_ylim()[1])
    ax2 = ax.twinx(); ax2.set_ylim(np.array(ax.get_ylim()) / (1e3 * L_ARM)); ax2.set_ylabel(r"$\rho_{UAV}$ (--)")
    ax.legend(loc="upper right", fontsize=8, frameon=False, bbox_to_anchor=(1.0, 0.88))
    p = os.path.join(figdir, "sim_box_push_uav_deviation.png"); fig.savefig(p, dpi=220); plt.close(fig); out["dev"] = p
    # ---- the impedance residual and the contact force over the contact window ----
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 7.0), sharex=True, constrained_layout=True)
    styles = {m: ("-" if i == 0 else "--") for i, m in enumerate(ORDER)}   # one line style per method
    for m, R in sel:
        s = R["imp"]["series"]; t = s["t"] - R["t0"]
        for i, (lab, col) in enumerate(zip("xyz", ("#0072BD", "#D95319", "#77AC30"))):
            axs[0].plot(t, s["F"][:, i], color=col, lw=0.9, ls=styles[m], label=f"{METHODS[m][0]} $F_{{ext,{lab}}}$")
            axs[1].plot(t, s["e_imp"][:, i], color=col, lw=0.9, ls=styles[m], label=f"{METHODS[m][0]} $e_{{imp,{lab}}}$")
        axs[2].plot(t, np.linalg.norm(s["e_imp"][:, :3], axis=1), color=COLORS[m], lw=0.9, label=f"{METHODS[m][0]} " + r"$\|e_{imp}\|$")
        for ax in axs:
            for w, (a_, b_) in ((k, (R["marks"]["push:start"] + 1.5 - R["t0"], R["marks"]["push:end"] - R["t0"])) for k in ("SLIDE",)):
                ax.axvspan(a_, b_, color="0.92", lw=0)
    axs[0].set_ylabel("$F_{ext}$ (N)"); axs[1].set_ylabel("$e_{imp}$ (N)"); axs[2].set_ylabel(r"$\|e_{imp}\|$ (N)")
    axs[2].set_xlabel("Time (s)")
    for ax in axs:
        ax.grid(alpha=0.3); ax.legend(fontsize=8, frameon=False, ncol=3, loc="upper right")
    axs[0].set_title("Contact phase (shaded: the %g s slide)" % sel[0][1]["task"]["push_time_s"], fontsize=10)
    p = os.path.join(figdir, "sim_box_push_impedance.png"); fig.savefig(p, dpi=220); plt.close(fig); out["imp"] = p
    return out


def splice_paper(paper, tex, figs):
    import shutil
    s = open(paper).read()
    head = r"\subsubsection{Physical Interaction: Box Push-and-Pull}"
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
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_box_push_trajectory_3d.png}",
        r"\caption{Platform, end-effector and box trajectories in the simulated box pushing (dashed: the planned references).}",
        r"\label{fig:sim_box_push_trajectory_3d}", r"\end{figure}", "",
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_box_push_impedance.png}",
        r"\caption{Contact force on the end effector and the impedance residual during the contact phase of the simulated box pushing (shaded: the 24~s slide).}",
        r"\label{fig:sim_box_push_impedance}", r"\end{figure}", "",
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_box_push_tracking_error.png}",
        r"\caption{Tracking errors in the simulated box pushing (bands: the mission's eight phases).}",
        r"\label{fig:sim_box_push_tracking_error}", r"\end{figure}", "",
        r"\begin{figure}[H]", r"\centering",
        r"\includegraphics[width=\textwidth]{Figures/2-Results/sim_box_push_uav_deviation.png}",
        r"\caption{Platform deviation $\|\varepsilon_{UAV}\|$ and its ratio $\rho_{UAV}$ to the arm's reach in the simulated box pushing.}",
        r"\label{fig:sim_box_push_uav_deviation}", r"\end{figure}", "", ""])
    open(paper, "w").write(s[:i] + block + s[j:])
    print("spliced the table and four figures into", paper)


if __name__ == "__main__":
    main()
