#!/usr/bin/env python3
"""Payload pick-and-place, EXPERIMENT: hardware runs -> the simulation's pick-and-place `run` struct, for MATLAB.

    /usr/bin/python3 results/utils/experiment_pick_and_place.py

Reads each run's ros2 bag from results/experiment_results/pick_and_place/<name>/ (copied there unchanged) and writes,
under results/experiment_results/matlab_experiment_data/pick_and_place/:
  <name>.mat                      struct `run`: meta, metrics, tracking, raw -- the layout of
                                  simulation_results/matlab_simulation_data/pick_and_place/<method>_pnp_run<k>.mat
  pick_and_place_metrics.mat      metrics_runs, metrics_mean, metrics_phase, phases, window_s, L_arm_m, outcome
  configs/                        the controller and arm configs as flown (taken from git at the flown commit)
  README.md                       the folder's guide (self-contained, like the simulation's)
Extracted bags are cached in $AM_EXP_CACHE (default /tmp/am_experiment_cache), outside the repository.

The hardware task is a pick-and-DROP (AM_HW_PROFILE=pick_and_drop): the basket is picked from the pick hat by the
hook grasp, carried, and released from a hover into a tray, where the simulation's paper task sets it down on a
hat. The six phases are the same legs, in the same order (phase 4 is the drop). The operator presses each step on
the arm ground station, so the phases come from the planner's pick-and-place status in the bag, not from a
timetable.

METRICS: the simulation's definitions, imported from build_pick_and_place.py (build_r0, metrics, L_ARM, G, BASE_COM):
  reference   the planner's WholeBodyReference: platform x_b = x_cd - R0 r_0c(q_d), R0 = the compatible attitude
              (thrust along x_cd_ddot + g e3, heading b1_d); EE r_ed, EE heading b1_de; joints q_d
  measured    EKF2-fused odometry (position, attitude) + joint encoders; EE position / heading by the model FK
  eps_UAV     |x_b - r_UAV|, max and RMS; rho_UAV = eps / L, L = 0.370 m
  RMSE        platform position xyz, attitude (rotation vector of R_ref^T R, body axes), EE position xyz,
              EE heading, joints q1..q4
WINDOW: the whole mission, from the start of go_to_start to the end of the land leg (or the end of DIRECT, if the
pilot took over first), on a 100 Hz grid.

Interpreters: extraction needs rclpy + px4_msgs (ROS sourced, the user-site numpy 2); stage 2 needs scipy (apt,
numpy 1.21) and re-executes this file with PYTHONNOUSERSITE=1.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.abspath(os.path.join(HERE, ".."))
REPO = os.path.abspath(os.path.join(RESULTS, ".."))
EXP = os.path.join(RESULTS, "experiment_results")
BAG_ROOT = os.path.join(EXP, "pick_and_place")
MAT_DIR = os.path.join(EXP, "matlab_experiment_data", "pick_and_place")
ARCH = os.path.join(REPO, "docs", "docs_aerial_manipulator", "archive")
EXTRACT = os.path.join(ARCH, "wb_pick_drop_flight_20261016", "tools", "extract.py")   # 10-09 extractor + gripper
NP1 = os.path.join(ARCH, "wb_pick_place_flight_20261009", "tools", "np1_compat.py")
PP_TOOLS = os.path.join(ARCH, "wb_pick_place_flight_20261009", "tools")              # pp_common -> 0928 common
CACHE = os.environ.get("AM_EXP_CACHE", "/tmp/am_experiment_cache")
AUTOPILOT = os.environ.get("FSC_AUTOPILOT_REPO", os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2"))
ARM_REPO = os.environ.get("FSC_OM_ARM_REPO", os.path.expanduser("~/ros2_ws/src/fsc_open_manipulator"))

# The exported runs. name = <method>_pnp_<tag>, the simulation's naming with the flight tag for the run number.
RUNS = {
    "whole_body_l1_pnp_D-2": dict(
        tag="D-2", method="whole_body_l1", flown="2026-10-16 14:00",
        bag="flight_wb_l1_4d_pick_and_drop_20261016_140020",
        payload="the basket assembly (basket, holder fin, mocap markers; rigid body obj_0), hook grasp",
        # (name in configs/, repository, commit, path in the repository): the files the flight ran. Both commits were
        # pushed at 12:44 on 2026-10-16, 76 min before the flight; the bag shows their features (carry pose = pick
        # pose, the automatic drop's 2.0 s wait, the basket-aim check).
        configs=[("whole_body_l1_4d_hw_pick_and_drop.yaml", AUTOPILOT, "f6395dd",
                  "config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_drop.yaml"),
                 ("arm_external_torque_controller_hardware_aerial_pwm.yaml", ARM_REPO, "ed67281",
                  "open_manipulator_x_custom_controller/config/external_torque_controller_hardware_aerial_pwm.yaml")]),
}
# The session's other bags, listed in the README only (not exported).
SESSION = [
    ("E-1", "13:44", "flight_wb_l1_4d_pick_and_drop_empty_20261016_134438",
     "full mission without the basket (control flight)", "not exported"),
    ("D-1", "13:51", "flight_wb_l1_4d_pick_and_drop_20261016_135100",
     "full mission; the gripper closed wrongly on the fin (stalled at -0.0010 m); the basket came to rest 112 mm "
     "from the drop point, tilted 25 deg", "not exported"),
    ("D-2", "14:00", "flight_wb_l1_4d_pick_and_drop_20261016_140020",
     "full mission; the basket came to rest 30 mm from the drop point, tilted 5 deg; pilot took over at 0.37 m "
     "on the final descent", "**exported**"),
]
LEGS = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place", "go_to_land_start", "execute_land"]
PHASES = [("go_to_start", "Go to start", "fly from the hover to the start waypoint; hover"),
          ("pick", "Pick", "fly to a point 10 cm behind the fin, slide in level (the planner's 'descent' step), close the "
                           "gripper on the fin, lift 20 cm (exit)"),
          ("to_drop_start", "To drop start", "carry the basket to the drop-start waypoint (180 deg turn); hover"),
          ("drop", "Drop", "fly over the tray, climbing to the release height; open the gripper (the basket falls into "
                           "the tray); back out 20 cm and climb 20 cm (exit); hover"),
          ("to_land_start", "To land start", "fly to the land-start waypoint (180 deg turn), arm folding home; hover"),
          ("land", "Land", "descend toward the landing point (z 0.35 m)")]
PHASE_KEYS = [p[0] for p in PHASES]
FLIP_TILT_DEG = 60.0   # obj_0 samples tilted beyond this are the rigid-body solve's 180-deg flips (10-09 finding)
OUTLIER_HALF_WINDOW, OUTLIER_POS_M, OUTLIER_YAW_DEG = 7, 0.015, 30.0   # the rolling-median test on what is left
ANSI = re.compile(r"\x1b\[[0-9;]*m")
NAV_STATE = re.compile(r"nav_state (\d+)")
NAV_NAME = {15: "STAB, the pilot's mode switch", 18: "AUTO_LAND", 5: "AUTO_RTL", 4: "AUTO_LOITER"}  # px4_msgs VehicleStatus


# ---------------------------------------------------------------- stage 1 (ROS python, numpy 2)
def extract():
    os.makedirs(CACHE, exist_ok=True)
    for name, r in RUNS.items():
        out = os.path.join(CACHE, name + ".npz")
        if os.path.isfile(out):
            continue
        bag = os.path.join(BAG_ROOT, name)
        if not os.path.isdir(bag):
            raise SystemExit(f"{name}: no bag at {bag} -- copy {r['bag']} there first (files unchanged)")
        cmd = (f"source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && "
               f"/usr/bin/python3 '{EXTRACT}' '{bag}' '{out}' && /usr/bin/python3 '{NP1}' '{out}'")
        print("extracting", name, flush=True)
        subprocess.run(["bash", "-c", cmd], check=True, stdout=subprocess.DEVNULL)


# ---------------------------------------------------------------- stage 2 (numpy 1.21 + scipy)
def _yaml_list(text, key):
    m = re.search(rf"^\s*{key}:\s*\[([^\]]*)\]", text, re.M)
    return [float(x) for x in m.group(1).split(",")] if m else None


def _yaml_float(text, key):
    m = re.search(rf"^\s*{key}:\s*([-+0-9.eE]+)", text, re.M)
    return float(m.group(1)) if m else None


def basket_outliers(P, yaw, half=OUTLIER_HALF_WINDOW):
    """True where a basket sample departs from the rolling median of its neighbours (2*half+1 samples) by more than
    OUTLIER_POS_M in position or OUTLIER_YAW_DEG in yaw: the solve's 180-deg yaw flips and the jumps that pass the tilt
    test (near the claw, 30-150 mm off the track). Two passes, so a removed run does not bias the next median."""
    import numpy as np
    from numpy.lib.stride_tricks import sliding_window_view as win

    def rmed(x):
        xp = np.pad(x, [(half, half)] + [(0, 0)] * (x.ndim - 1), mode="edge")
        return np.median(win(xp, 2 * half + 1, axis=0), axis=-1)
    bad = np.zeros(len(P), bool)
    for _ in range(2):
        i = np.where(~bad)[0]
        dpos = np.linalg.norm(P[i] - rmed(P[i]), axis=1)
        ym = np.arctan2(rmed(np.sin(yaw[i])), rmed(np.cos(yaw[i])))
        dyaw = np.abs((yaw[i] - ym + np.pi) % (2 * np.pi) - np.pi)
        bad[i[(dpos > OUTLIER_POS_M) | (dyaw > np.radians(OUTLIER_YAW_DEG))]] = True
    return bad


def copy_configs(r):
    """configs/<name> <- `git show <commit>:<path>`; returns {name: text}. Keeps an existing copy if git is missing."""
    cfg = os.path.join(MAT_DIR, "configs")
    os.makedirs(cfg, exist_ok=True)
    out = {}
    for dst, repo, rev, path in r["configs"]:
        p = os.path.join(cfg, dst)
        try:
            text = subprocess.run(["git", "-C", repo, "show", f"{rev}:{path}"], check=True, capture_output=True,
                                  text=True).stdout
            hdr = f"# As flown: {os.path.basename(repo)} commit {rev}, {path}\n"
            open(p, "w").write(hdr + text)
        except (subprocess.CalledProcessError, FileNotFoundError):
            if not os.path.isfile(p):
                raise SystemExit(f"cannot read {rev}:{path} from {repo}, and configs/{dst} does not exist")
            print(f"WARNING: git unavailable for {dst}; keeping the existing copy", flush=True)
        out[dst] = open(p).read()
    return out


def convert(name, r):
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot
    os.environ["AM_NPZ"] = CACHE
    sys.path.insert(0, PP_TOOLS)
    sys.path.insert(0, HERE)
    import pp_common as PC
    import build_pick_and_place as B        # the simulation's definitions
    TP = PC.TP
    RZ90, RZM90 = TP._Rz(0.5 * math.pi), TP._Rz(-0.5 * math.pi)

    d, t0 = PC.load(name)
    cfg = copy_configs(r)
    wb_yaml = cfg[r["configs"][0][0]]
    ee_off = np.array(_yaml_list(wb_yaml, "pick_place_ee_offset"))
    drop_h = _yaml_float(wb_yaml, "pick_place_drop_height")

    # ---- phases: the planner's pick-and-place status (FLYING <leg> edges)
    st = PC.edges(d, t0, "pp_status")
    starts = {}
    for leg in LEGS:
        hit = [t for t, v in st if v.startswith(f"FLYING {leg}")]
        if not hit:
            raise SystemExit(f"{name}: the leg {leg} never flew -- not a complete mission")
        starts[leg] = hit[0]
    if any(v.startswith("ABORTING") for _, v in st):
        raise SystemExit(f"{name}: the mission was ABORTED")
    direct_a, direct_b = PC.direct_span(d, t0)
    land0 = starts["execute_land"]
    land_end = next((t for t, v in st if t > land0 and not v.startswith("FLYING execute_land")), direct_b)
    land_end = min(land_end, direct_b)
    to, Pod, Qod, Vod, Wod = PC.odom(d, t0)              # Q: w x y z, sign-continuous
    tj, qj, qdj = PC.joints_model(d, t0)                 # model convention, j1..j4 [rad]
    tr, ref = PC.wbref(d, t0)
    t_start = starts["go_to_start"]
    t_end = min(land_end, tr[-1], to[-1], tj[-1])
    bounds = [starts[k] for k in LEGS] + [t_end]
    phases = {key: (bounds[i], bounds[i + 1]) for i, key in enumerate(PHASE_KEYS)}

    # ---- the 100 Hz tracking grid, the simulation's computation (build_pick_and_place.load_run)
    tu = np.arange(t_start, t_end, B.DT)
    n = len(tu)
    P = B.interp(tu, to, Pod)
    Qu = B.interp(tu, to, Qod[:, [1, 2, 3, 0]]); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    Ra = Rot.from_quat(Qu).as_matrix()
    q = B.interp(tu, tj, qj)
    r_ed, x_cd, q_d = (B.interp(tu, tr, ref[k]) for k in ("r_ed", "x_cd", "q_d"))
    b1d, x_cd_dd, b1de = (B.interp(tu, tr, ref[k]) for k in ("b1_d", "x_cd_ddot", "b1_de"))
    params = TP.make_params_t650(base_com=B.BASE_COM)
    xb = np.zeros((n, 3)); Rref = np.zeros((n, 3, 3)); xc = np.zeros((n, 3)); re = np.zeros((n, 3)); b1e = np.zeros((n, 3))
    for i in range(n):
        R0m = B.build_r0(x_cd_dd[i] + np.array([0.0, 0.0, B.G]), b1d[i])
        r0c_d, _, _ = TP.arm_fk_model(q_d[i], params)
        xb[i] = x_cd[i] - R0m @ r0c_d
        Rref[i] = R0m @ RZ90
        Rm = Ra[i] @ RZM90
        r0c, r0e, Re = TP.arm_fk_model(q[i], params)
        xc[i] = P[i] + Rm @ r0c; re[i] = P[i] + Rm @ r0e; b1e[i] = Rm @ Re[:, 0]
    att_err = np.degrees(Rot.from_matrix(np.einsum("nji,njk->nik", Rref, Ra)).as_rotvec())
    az = np.degrees(np.unwrap(np.arctan2(b1e[:, 1], b1e[:, 0])))
    az_ref = np.degrees(np.unwrap(np.arctan2(b1de[:, 1], b1de[:, 0])))
    ee_head_err = np.degrees(B.wrap(np.radians(az - az_ref)))
    e_pos = P - xb; e_ee = re - r_ed; e_q = np.degrees(q - q_d)
    eps = np.linalg.norm(e_pos, axis=1)
    tilt = np.degrees(np.arccos(np.clip(Ra[:, 2, 2], -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(Ra[:, 1, 0], Ra[:, 0, 0])))
    yaw_ref = np.degrees(np.unwrap(np.arctan2(Rref[:, 1, 0], Rref[:, 0, 0])))
    R = dict(e_pos=e_pos, e_ee=e_ee, e_q=e_q, att_err=att_err, ee_head_err=ee_head_err, eps=eps, tilt=tilt)
    phase_id = np.zeros(n)
    per_phase = {}
    for i, key in enumerate(PHASE_KEYS):
        a_, b_ = phases[key]
        m = (tu >= a_) & (tu < b_)
        phase_id[m] = i + 1
        per_phase[key] = dict(label=PHASES[i][1], start_s=a_ - t_start, end_s=b_ - t_start, **B.metrics(R, m))
    window = B.metrics(R, np.ones(n, bool))

    # ---- the basket (obj_0, raw OptiTrack VRPN pose), mis-solved samples removed
    tp, Pp, Qp, _ = PC.mocap_pose(d, t0, "obj_vrpn")
    flip = PC.tilt_of(Qp) > FLIP_TILT_DEG
    tp, Pp, Qp = tp[~flip], Pp[~flip], Qp[~flip]
    out_ = basket_outliers(Pp, PC.yaw_of(Qp))
    tp, Pp, Qp = tp[~out_], Pp[~out_], Qp[~out_]
    near = np.abs(tp[np.clip(np.searchsorted(tp, tu), 0, len(tp) - 1)] - tu) <= 0.1
    pay_u = B.interp(tu, tp, Pp); pay_u[~near] = np.nan

    # ---- the task outcome (the simulation's place_off_axis_mm, for a drop)
    I = d["pp_info__data"]
    ok = np.where(np.isfinite(I[:, 14 + 7 * 3 + 4]))[0]
    goals = I[ok[-1], 14:56].reshape(6, 7)                  # per leg: base x y z, yaw [deg], claw x y z
    claw_drop = goals[3, 4:7]; drop_yaw = math.radians(goals[3, 3])
    Rz = np.array([[math.cos(drop_yaw), -math.sin(drop_yaw), 0], [math.sin(drop_yaw), math.cos(drop_yaw), 0], [0, 0, 1]])
    drop_pt = claw_drop - Rz @ ee_off - np.array([0.0, 0.0, drop_h])
    log = [(float(t), str(nm), ANSI.sub("", str(msg))) for t, nm, msg in
           zip(d["rosout__recv"] - t0, d["rosout__name"], d["rosout__msg"]) if "rosbag2_recorder" not in str(nm)]
    first = lambda s: next(t for t, nm, msg in log if s in msg)      # noqa: E731
    t_close, t_open = first("gripper CLOSE"), first("gripper OPEN")
    t_grasp = first("Exit To Pick at once")
    land_done = any(v.startswith(("DONE execute_land", "COMPLETE")) for t, v in st if t > land0)
    left = [msg for t, nm, msg in log if "PX4 LEFT OFFBOARD" in msg and abs(t - direct_b) < 0.5]
    nav = NAV_STATE.search(left[0]) if left else None
    direct_exit = ("the land leg completed" if land_done else
                   f"the land leg was cut short {direct_b - land0:.2f} s in: " +
                   (f"PX4 left OFFBOARD for nav_state {nav.group(1)} "
                    f"({NAV_NAME.get(int(nav.group(1)), 'see PX4 VehicleStatus')})"
                    if nav else "the controller left DIRECT"))
    at = lambda t: np.array([np.interp(t, tp, Pp[:, k]) for k in range(3)])   # noqa: E731
    pay0 = np.median(Pp[(tp >= t_start) & (tp < t_start + 2.0)], axis=0)
    m_carry = (tp >= phases["to_drop_start"][0]) & (tp < phases["to_drop_start"][1])
    rest_m = tp >= tp[-1] - 2.0
    rest = np.median(Pp[rest_m], axis=0)
    rel_open = at(t_open)
    claw_open = np.array([np.interp(t_open, tu, re[:, k]) for k in range(3)])
    outcome = dict(
        picked=bool(np.max(Pp[m_carry, 2]) > pay0[2] + 0.05),
        payload_start_m=pay0, drop_point_m=drop_pt, drop_claw_target_m=claw_drop, drop_yaw_deg=goals[3, 3],
        ee_offset_m=ee_off, drop_margin_m=drop_h,
        gripper_close_s=t_close - t_start, grasp_reported_s=t_grasp - t_start, gripper_open_s=t_open - t_start,
        claw_err_at_open_mm=1e3 * (claw_open - claw_drop),
        payload_at_open_m=rel_open, release_off_xy_mm=1e3 * (rel_open - drop_pt)[:2],
        release_off_mm=float(1e3 * np.linalg.norm((rel_open - drop_pt)[:2])),
        rest_m=rest, rest_off_xy_mm=1e3 * (rest - drop_pt)[:2],
        rest_off_mm=float(1e3 * np.linalg.norm((rest - drop_pt)[:2])),
        rest_tilt_deg=float(np.median(PC.tilt_of(Qp[rest_m]))), fall_m=float(rel_open[2] - rest[2]),
        definition="drop point = the drop leg's claw target - Rz(yaw) pick_place_ee_offset - [0 0 pick_place_drop_height] "
                   "(the Get-captured basket on the tray cover); release = the arm GS's gripper OPEN command; "
                   "rest = median over the last 2 s of the recording; xy offsets are world ENU")

    def edges_struct(k):
        e = PC.edges(d, t0, k)
        return dict(t=np.array([x[0] for x in e]), label=np.array([x[1] for x in e], dtype=object))

    js_pos = d["js__position"]
    cur = d["js__effort"][:, :4][:, PC.JS_IDX]                               # Present Current counts, j1..j4
    tw = d["wb__recv"] - t0
    phase_tab = dict(key=np.array(PHASE_KEYS, dtype=object), label=np.array([p[1] for p in PHASES], dtype=object),
                     start_s=np.array([phases[k][0] - t_start for k in PHASE_KEYS]),
                     end_s=np.array([phases[k][1] - t_start for k in PHASE_KEYS]))
    gap = lambda t: float(np.diff(t[(t >= t_start) & (t <= t_end)]).max())    # noqa: E731
    meta = dict(
        name=name, tag=r["tag"], method=r["method"], method_long="Whole-body L1 impedance (proposed), 4-D attribution",
        run_index=r["tag"], flown=r["flown"], bag=r["bag"],
        task="pick-and-drop: hook grasp from the pick hat, carry, release from a hover into a tray",
        scene="lab: the basket starts on the pick pillar's hat (resting at z = "
              f"{np.median(Pp[(tp >= t_start) & (tp < t_start + 2.0)], axis=0)[2]:.3f} m); the tray sits on the "
              "drop pillar; basket = OptiTrack rigid body obj_0",
        plant="T650 aerial manipulator (hardware), total mass 3.746 kg in the controller model",
        feedback="EKF2-fused OptiTrack odometry (~100 Hz); joint encoders; basket = OptiTrack rigid body obj_0",
        payload=r["payload"], phases=phase_tab,
        window=f"the whole mission, t = 0 (start of go to start) to {t_end - t_start:.2f} s (end of the land leg / DIRECT)",
        L_arm_m=B.L_ARM, base_com_model_m=B.BASE_COM, aborted=False,
        t0_s=t_start, t0_driver_s=t_start,
        clock="raw streams: s since the bag's first message; tracking.t: s since the start of go to start "
              "(= raw t - meta.t0_s). meta.t0_driver_s equals meta.t0_s, so scripts written for the simulation files run unchanged",
        direct_enter_s=direct_a, direct_exit_s=direct_b,
        direct_exit=direct_exit,
        max_gap_s=dict(odom=gap(to), joints=gap(tj), reference=gap(tr), payload=gap(tp)),
        payload_flips_removed=int(flip.sum()), payload_outliers_removed=int(out_.sum()),
        leg_goals=dict(leg=np.array(LEGS, dtype=object), base_m=goals[:, 0:3], yaw_deg=goals[:, 3], claw_m=goals[:, 4:7]),
        configs=np.array([c[0] for c in r["configs"]], dtype=object),
        config_source=np.array([f"{os.path.basename(c[1])} {c[2]}:{c[3]}" for c in r["configs"]], dtype=object))
    raw = dict(
        odom=dict(t=to, pos_m=Pod, vel_mps=Vod, quat_wxyz=Qod, angvel_rad_s=Wod),
        joints=dict(t=tj, q_rad=qj, qdot_rad_s=qdj, current_counts=cur[np.all(np.isfinite(js_pos[:, :4]), axis=1)],
                    effort=(cur / PC.KT * PC.HW_SIGN)[np.all(np.isfinite(js_pos[:, :4]), axis=1)]),
        reference=dict(t=tr, r_ed=ref["r_ed"], r_ed_dot=ref["r_ed_dot"], x_cd=ref["x_cd"], x_cd_dot=ref["x_cd_dot"],
                       x_cd_ddot=ref["x_cd_ddot"], q_d_rad=ref["q_d"], qdot_d_rad_s=ref["qdot_d"],
                       b1_d_model=ref["b1_d"], b1_de=ref["b1_de"]),
        payload=dict(t=tp, pos_m=Pp, quat_wxyz=Qp),
        gripper=dict(t=d["js__recv"] - t0, pos_m=js_pos[:, 4:6]),
        controller_debug=dict(t=tw, data=d["wb__data"].astype(float)),
        motors=dict(t=d["motors__recv"] - t0, control=d["motors__control"].astype(float)),
        battery=dict(t=d["batt__recv"] - t0, voltage_v=d["batt__voltage_v"].astype(float),
                     current_a=d["batt__current_a"].astype(float)),
        mocap_vrpn=dict(t=d["vrpn__recv"] - t0, pos_m=np.column_stack([d[f"vrpn__pose.position.{a}"] for a in "xyz"]),
                        quat_xyzw=np.column_stack([d[f"vrpn__pose.orientation.{a}"] for a in "xyzw"])),
        events=dict(mode=edges_struct("wbmode"), planner_status=edges_struct("pl_status"),
                    pick_place_status=edges_struct("pp_status"),
                    log=dict(t=np.array([x[0] for x in log]), node=np.array([x[1] for x in log], dtype=object),
                             msg=np.array([x[2] for x in log], dtype=object))))
    tracking = dict(
        t=tu - t_start, in_window=np.ones(n), phase_id=phase_id,
        platform_pos_m=P, platform_pos_ref_m=xb, platform_pos_err_m=e_pos, eps_uav_m=eps, rho_uav=eps / B.L_ARM,
        platform_quat_xyzw=Qu, platform_att_err_deg=att_err, platform_heading_deg=yaw, platform_heading_ref_deg=yaw_ref,
        platform_tilt_deg=tilt, com_pos_m=xc, com_pos_ref_m=x_cd, com_pos_err_m=xc - x_cd,
        ee_pos_m=re, ee_pos_ref_m=r_ed, ee_pos_err_m=e_ee,
        ee_heading_deg=az, ee_heading_ref_deg=az_ref, ee_heading_err_deg=ee_head_err,
        q_deg=np.degrees(q), q_ref_deg=np.degrees(q_d), q_err_deg=e_q, payload_pos_m=pay_u,
        columns="platform_att_err_deg = [roll pitch heading] (body axes); xyz columns are world ENU; "
                "payload_pos_m is NaN where no basket sample lies within 0.1 s")
    import scipy.io
    scipy.io.savemat(os.path.join(MAT_DIR, name + ".mat"),
                     {"run": dict(meta=meta, metrics=dict(window=window, per_phase=per_phase, outcome=outcome),
                                  tracking=tracking, raw=raw)},
                     do_compression=True, long_field_names=True, oned_as="column")
    print(f"wrote {os.path.relpath(os.path.join(MAT_DIR, name + '.mat'), RESULTS)}  window {window['window_s']:.2f} s  "
          f"eps max/rms {window['eps_uav_max_mm']:.1f}/{window['eps_uav_rms_mm']:.1f} mm  rho {window['rho_uav_max']:.3f}/"
          f"{window['rho_uav_rms']:.3f}  EE {window['ee_pos_mm']:.1f} mm  rest {outcome['rest_off_mm']:.0f} mm "
          f"tilt {outcome['rest_tilt_deg']:.1f} deg", flush=True)
    return dict(name=name, r=r, window=window, per_phase=per_phase, outcome=outcome, phases=phase_tab,
                t_end=t_end - t_start, meta=meta, z_end=float(P[-1, 2]),
                n_payload_raw=int(len(tp) + flip.sum() + out_.sum()),
                gains={k: _yaml_float(wb_yaml, k) for k in ("wb_k_r", "wb_k_w", "wb_ky_x", "wb_dy_x", "wb_ky_psi", "wb_dy_psi")})


def stage2():
    import numpy as np
    import scipy.io
    sys.path.insert(0, HERE)
    import build_pick_and_place as B
    os.makedirs(MAT_DIR, exist_ok=True)
    out = [convert(name, r) for name, r in RUNS.items()]
    flat, ph_rows = [], []
    for o in out:
        w = o["window"]
        f = dict(name=o["name"], method=o["r"]["method"], run=o["r"]["tag"], total_window_s=w["window_s"])
        for k in ("eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms", "platform_pos_mm",
                  "platform_heading_deg", "platform_roll_deg", "platform_pitch_deg", "ee_pos_mm", "ee_heading_deg",
                  "tilt_max_deg"):
            f[k] = w[k]
        for j, ax in enumerate("xyz"):
            f[f"platform_{ax}_mm"] = w["platform_pos_xyz_mm"][j]; f[f"ee_{ax}_mm"] = w["ee_pos_xyz_mm"][j]
        for j in range(4):
            f[f"q{j + 1}_deg"] = w["joints_deg"][j]
        f.update(drop_rest_off_mm=o["outcome"]["rest_off_mm"], drop_release_off_mm=o["outcome"]["release_off_mm"],
                 drop_rest_tilt_deg=o["outcome"]["rest_tilt_deg"])
        flat.append(f)
        for key in PHASE_KEYS:
            p = o["per_phase"][key]
            ph_rows.append(dict(name=o["name"], method=o["r"]["method"], run=o["r"]["tag"], phase=key, label=p["label"],
                                start_s=p["start_s"], end_s=p["end_s"], eps_uav_max_mm=p["eps_uav_max_mm"],
                                eps_uav_rms_mm=p["eps_uav_rms_mm"], rho_uav_max=p["rho_uav_max"],
                                rho_uav_rms=p["rho_uav_rms"], ee_pos_mm=p["ee_pos_mm"],
                                platform_pos_mm=p["platform_pos_mm"]))
    mean = [dict(method=m, runs=float(sum(1 for f in flat if f["method"] == m)),
                 **{k: float(np.mean([f[k] for f in flat if f["method"] == m]))
                    for k in flat[0] if k not in ("name", "method", "run")})
            for m in sorted({f["method"] for f in flat})]
    scipy.io.savemat(os.path.join(MAT_DIR, "pick_and_place_metrics.mat"),
                     {"metrics_runs": B.colstruct(flat, list(flat[0])), "metrics_mean": B.colstruct(mean, list(mean[0])),
                      "metrics_phase": B.colstruct(ph_rows, list(ph_rows[0])), "phases": out[0]["phases"],
                      "window_s": out[0]["t_end"], "L_arm_m": B.L_ARM},
                     do_compression=True, long_field_names=True, oned_as="column")
    print("wrote", os.path.relpath(os.path.join(MAT_DIR, "pick_and_place_metrics.mat"), RESULTS), flush=True)
    write_readme(out)
    json.dump([dict(name=o["name"], window=o["window"], outcome={k: (v.tolist() if hasattr(v, "tolist") else v)
                                                                 for k, v in o["outcome"].items()})
               for o in out], open(os.path.join(CACHE, "pick_and_place_scores.json"), "w"), indent=1, default=float)


def write_readme(out):
    sys.path.insert(0, HERE)
    from readme_text import _md_table
    o = out[0]
    name, w, oc, ph, meta = o["name"], o["window"], o["outcome"], o["phases"], o["meta"]
    f1 = lambda x: f"{x:.1f}"    # noqa: E731
    phase_rows = [[str(i + 1), PHASES[i][1], f"{ph['start_s'][i]:.2f}", f"{ph['end_s'][i]:.2f}",
                   f"{ph['end_s'][i] - ph['start_s'][i]:.2f}", PHASES[i][2]] for i in range(len(PHASES))]
    per = o["per_phase"]
    met_rows = [[PHASES[i][1], f1(per[k]["eps_uav_max_mm"]), f"{per[k]['rho_uav_max']:.3f}", f1(per[k]["eps_uav_rms_mm"]),
                 f"{per[k]['rho_uav_rms']:.3f}", f1(per[k]["ee_pos_mm"]), f"{per[k]['ee_heading_deg']:.2f}",
                 f1(per[k]["tilt_max_deg"])] for i, k in enumerate(PHASE_KEYS)]
    met_rows.append(["**whole mission**", f1(w["eps_uav_max_mm"]), f"{w['rho_uav_max']:.3f}", f1(w["eps_uav_rms_mm"]),
                     f"{w['rho_uav_rms']:.3f}", f1(w["ee_pos_mm"]), f"{w['ee_heading_deg']:.2f}", f1(w["tilt_max_deg"])])
    xyz = lambda v: "(" + ", ".join(f"{x:.0f}" for x in v) + ")"   # noqa: E731
    goals = meta["leg_goals"]
    goal_rows = [[LEGS[k], "(" + ", ".join(f"{x:.3f}" for x in goals["base_m"][k]) + ")", f"{goals['yaw_deg'][k]:.1f}",
                  "(" + ", ".join(f"{x:.3f}" for x in goals["claw_m"][k]) + ")"] for k in range(6)]
    L = [
        "# Experiment results, MATLAB data (payload pick-and-place)", "",
        "The hardware pick-and-place flight, converted from its ros2 bag. This folder is self-contained:",
        "these files plus MATLAB are all you need, with no ROS or custom message definitions. The `run` struct",
        "has the layout of the simulation's `matlab_simulation_data/pick_and_place/<method>_pnp_run<k>.mat`, so",
        "a script written for those files runs on this one.", "",
        "## Files", "",
        "```",
        f"{name}.mat               one run, struct `run`",
        "pick_and_place_metrics.mat               metrics_runs / metrics_mean = the run's numbers (one row),",
        "                                         metrics_phase = per phase, phases = the phase table,",
        "                                         window_s = the evaluation window, L_arm_m = L",
        "configs/whole_body_l1_4d_hw_pick_and_drop.yaml               the controller + planner, as flown",
        "configs/arm_external_torque_controller_hardware_aerial_pwm.yaml   the arm controller, as flown",
        "```", "",
        "- **Name:** `<method>_pnp_<run>`, the simulation's naming with the flight tag as the run.",
        "- **Method:** `whole_body_l1` = Proposed. Only the proposed controller flew this task on hardware.",
        "- **Configs:** taken from git at the commits the vehicle ran (first line of each file). Both were pushed",
        "  76 min before the flight, and the bag shows their features: carry pose = pick pose, the automatic drop's",
        "  2.0 s wait and the basket-aim check.", "",
        "```matlab",
        f"r  = load('{name}.mat').run;",
        "P  = r.meta.phases;                              % the six phases of this flight",
        "tr = r.tracking;",
        "figure; hold on",
        "for i = 1:numel(P.start_s)                       % phase bands, alternating shades",
        "    c = 0.90 + 0.07*mod(i+1, 2);",
        "    patch([P.start_s(i) P.end_s(i) P.end_s(i) P.start_s(i)], [0 0 400 400], c*[1 1 1], 'EdgeColor', 'none');",
        "    text(mean([P.start_s(i) P.end_s(i)]), 390, P.label{i}, 'HorizontalAlignment', 'center');",
        "end",
        "plot(tr.t, 1e3 * tr.eps_uav_m)                   % ||eps_UAV|| [mm]",
        "xlim([0 tr.t(end)]); xlabel('Time (s)'); ylabel('||\\epsilon_{UAV}|| (mm)')",
        "yyaxis right; ylim([0 400] / (1e3 * r.meta.L_arm_m)); ylabel('\\rho_{UAV}')",
        "",
        "% 3-D: references (dashed), measured platform and end effector, the basket (OptiTrack)",
        "figure; hold on; grid on; axis equal; view(-50, 28)",
        "plot3(tr.platform_pos_ref_m(:,1), tr.platform_pos_ref_m(:,2), tr.platform_pos_ref_m(:,3), 'k--')",
        "plot3(tr.ee_pos_ref_m(:,1), tr.ee_pos_ref_m(:,2), tr.ee_pos_ref_m(:,3), '--', 'Color', [0.45 0.45 0.45])",
        "plot3(tr.platform_pos_m(:,1), tr.platform_pos_m(:,2), tr.platform_pos_m(:,3))",
        "plot3(tr.ee_pos_m(:,1), tr.ee_pos_m(:,2), tr.ee_pos_m(:,3), ':')",
        "plot3(tr.payload_pos_m(:,1), tr.payload_pos_m(:,2), tr.payload_pos_m(:,3))   % or r.raw.payload (raw clock)",
        "",
        "% the error grid: platform / end effector xyz in mm, headings and joints in deg",
        "E = {1e3*tr.platform_pos_err_m, tr.platform_att_err_deg(:,3), 1e3*tr.ee_pos_err_m, tr.ee_heading_err_deg, tr.q_err_deg};",
        "```", "",
        "Raw streams are on the bag's clock: subtract `r.meta.t0_s` to put them on `tracking.t`",
        "(`r.meta.t0_driver_s` is the same number, so the simulation's snippet works unchanged).", "",
        "## The flight", "",
        _md_table(["run", "flown", "original ros2 bag", "what happened", "in this folder"],
                  [[t, f"2026-10-16 {hh}", f"`{b}`", wh, ex] for t, hh, b, wh, ex in SESSION]), "",
        "The bag of the exported run is kept unchanged in `results/experiment_results/pick_and_place/<name>/` on the",
        "lab desktop. The other two bags are in `docs/experimental_data_ros2_bag/1016 - T650-AM whole-body Pick&Drop-*/`.", "",
        "## How this differs from the simulation's pick-and-place", "",
        "- **The task is a pick-and-DROP.** The pick is the same hook grasp. At the place the basket is released",
        "  from a hover into a tray (`pick_place_place_drop`), where the simulation's paper task sets it down on a",
        "  hat. The six phases are the same legs in the same order; phase 4 is the drop.",
        "- **The operator paced the mission.** Each step was pressed on the arm ground station, so the phases come",
        "  from the planner's status in the bag. The simulation used a fixed timetable.",
        "- **One controller.** No Geo-L1 run exists for this task on hardware.",
        "- **The basket is measured by OptiTrack** (rigid body `obj_0`), not simulator ground truth.",
        f"  Mis-solved samples are removed: {meta['payload_flips_removed']} flips (tilt > {FLIP_TILT_DEG:.0f} deg) and "
        f"{meta['payload_outliers_removed']} jumps",
        f"  (more than {1e3 * OUTLIER_POS_M:.0f} mm or {OUTLIER_YAW_DEG:.0f} deg of yaw off the rolling median of "
        f"{2 * OUTLIER_HALF_WINDOW + 1} samples), {100 * (meta['payload_flips_removed'] + meta['payload_outliers_removed']) / o['n_payload_raw']:.1f} % of the stream.",
        "  They happen when the claw is near the basket and, at rest, in the tray.",
        f"- **The window ends with DIRECT.** {meta['direct_exit'][0].upper() + meta['direct_exit'][1:]}, with the "
        f"platform at {o['z_end']:.2f} m (land target {goals['base_m'][5][2]:.2f} m).",
        "- **Joint torque** `raw.joints.effort` [N.m, model convention] is the servos' Present Current",
        "  (`raw.joints.current_counts`) through the calibrated torque constants: the electromagnetic torque the",
        "  motors applied, before the gearbox's friction (the simulation's `effort` is the applied torque too).", "",
        "## Phases (s since the start of go to start)", "",
        _md_table(["#", "phase", "start [s]", "end [s]", "duration [s]", "what happens"], phase_rows), "",
        f"- **Window:** the whole mission, 0 to {o['t_end']:.2f} s. `tracking.phase_id` (1..6) says which phase",
        "  each sample is in; `tracking.in_window` is 1 everywhere (kept for the simulation scripts).",
        "- **Phase boundaries:** each phase runs from its leg's `FLYING` status to the next leg's, so the hovers",
        "  and the operator's waits between steps belong to the phase they end.", "",
        "## Results", "",
        _md_table(["phase", "eps max [mm]", "rho max", "eps rms [mm]", "rho rms", "EE RMSE [mm]", "EE heading RMSE [deg]",
                   "tilt max [deg]"], met_rows), "",
        "Drop (`run.metrics.outcome`):", "",
        f"- **Drop point** (the Get-captured basket on the tray cover): ({oc['drop_point_m'][0]:.3f}, "
        f"{oc['drop_point_m'][1]:.3f}, {oc['drop_point_m'][2]:.3f}) m; the claw released "
        f"{oc['drop_margin_m']:.2f} m above it.",
        f"- **Claw at the OPEN command,** relative to its target: {xyz(oc['claw_err_at_open_mm'])} mm (x, y, z).",
        f"- **Basket at the OPEN command:** {oc['release_off_mm']:.0f} mm from the drop point in xy; it fell "
        f"{oc['fall_m']:.3f} m.",
        f"- **Basket at rest:** {oc['rest_off_mm']:.0f} mm from the drop point in xy {xyz(oc['rest_off_xy_mm'])}, "
        f"tilted {oc['rest_tilt_deg']:.1f} deg.",
        f"- **Gripper:** CLOSE at {oc['gripper_close_s']:.2f} s, grasp reported at {oc['grasp_reported_s']:.2f} s, "
        f"OPEN at {oc['gripper_open_s']:.2f} s.", "",
        "## Conditions", "",
        "- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm in torque mode, 3.746 kg in the controller model.",
        "- **Feedback:** OptiTrack fused in PX4 EKF2 (~100 Hz); joint encoders.",
        "- **Controller:** whole-body L1, 4-D attribution, `AM_HW_PROFILE=pick_and_drop`",
        f"  (`configs/whole_body_l1_4d_hw_pick_and_drop.yaml`: k_R / k_w {o['gains']['wb_k_r']:g} / "
        f"{o['gains']['wb_k_w']:g}, K_y / D_y {o['gains']['wb_ky_x']:g} / {o['gains']['wb_dy_x']:g}, "
        f"K_psi / D_psi {o['gains']['wb_ky_psi']:g} / {o['gains']['wb_dy_psi']:g}).",
        "- **Arm poses:** pick = carry = drop [0, 32, 38, 0] deg, home [0, 40, 40, 0] deg.",
        f"- **EE offset** (the claw point in the basket's frame): ({oc['ee_offset_m'][0]:.2f}, {oc['ee_offset_m'][1]:.2f}, "
        f"{oc['ee_offset_m'][2]:.2f}) m.",
        "- **Leg goals** (`run.meta.leg_goals`, world ENU, planned from the Adjust-shifted waypoints):", "",
        _md_table(["leg", "base [m]", "yaw [deg]", "claw [m]"], goal_rows), "",
        "## The struct `run`", "",
        "| field | content |", "|---|---|",
        "| `meta` | name, run tag, method, flight date, original bag, phase table, window, clock, DIRECT entry/exit, largest "
        "gap per stream, flips removed, leg goals, configs |",
        "| `metrics` | `window` (whole mission), `per_phase.<phase>`, `outcome` (the drop) |",
        "| `tracking` | 100 Hz over the mission, `t` in s since the start of go to start. Measured, reference (`_ref_`) "
        "and error (`_err_`) for the platform position and attitude, `eps_uav_m` / `rho_uav`, the system CoM, the end-effector "
        "position and heading, the joints (`q_deg`), and `payload_pos_m` (the basket) |",
        "| `raw` | the bag's streams, time in s since the bag's first message: `odom`, `joints` (model convention), "
        "`reference` (the planner's whole-body reference), `payload` (obj_0), `gripper` (finger positions), "
        "`controller_debug` (`wb_control_debug`), `motors` (normalised), `battery`, `mocap_vrpn` (vehicle), `events.mode` / "
        "`events.planner_status` / `events.pick_place_status` (transitions), `events.log` (every node's /rosout line: the "
        "operator's steps and the planner's replies) |", "",
        "Units: positions in m (world ENU, z up) unless the name says mm; angles in deg unless it says rad. Errors are",
        "measured − reference. `platform_att_err_deg` columns are `[roll pitch heading]`.", "",
        "**Definitions** (the simulation's, `results/utils/build_pick_and_place.py`):", "",
        "- **Reference:** the planner's dynamically compatible whole-body reference. Platform",
        "  x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>); R<sub>0,d</sub> from the thrust",
        "  direction (CoM reference acceleration + g e<sub>3</sub>) and the reference heading; EE r<sub>e,d</sub>, heading",
        "  b<sub>1,e,d</sub>; joints q<sub>d</sub>.",
        "- **Measured EE:** the planner model's forward kinematics of the measured platform pose and joint angles.",
        f"- **ε<sub>UAV</sub>** = ‖x<sub>b</sub> − r<sub>UAV</sub>‖; ρ<sub>UAV</sub> = ε<sub>UAV</sub> / L, L = {meta['L_arm_m']:.3f} m.", "",
        "Regenerate (from the bag in `results/experiment_results/pick_and_place/`):", "",
        "```bash", "/usr/bin/python3 results/utils/experiment_pick_and_place.py", "```", ""]
    with open(os.path.join(MAT_DIR, "README.md"), "w") as f:
        f.write("\n".join(L))
    print("wrote", os.path.relpath(os.path.join(MAT_DIR, "README.md"), RESULTS), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage2", action="store_true")
    a = ap.parse_args()
    if a.stage2:
        stage2()
        return
    extract()
    env = dict(os.environ, PYTHONNOUSERSITE="1", AM_EXP_CACHE=CACHE)
    subprocess.run(["/usr/bin/python3", os.path.abspath(__file__), "--stage2"], env=env, check=True)


if __name__ == "__main__":
    main()
