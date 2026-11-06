#!/usr/bin/env python3
"""Mail delivery (mail box), EXPERIMENT: the 2026-11-05 hardware flights -> MATLAB.

    /usr/bin/python3 results/utils/experiment_mail_box.py

The task: pick a red envelope lying flat on the pick hat (wrist rolled 90 deg, the jaws close vertically), carry it,
and slide it through the mouth of the red mailbox along a line pitched 25 deg down; release it in the mouth, back
out, fly home, land. The planner's pick-and-place mode with the insert place (`AM_HW_PROFILE=mail_delivery`).

Reads every bag flight_wb_l1_4d_mail_box_<date>_<time>_success_<N> under docs/experimental_data_ros2_bag/<folder>/
<subfolder>/ (found by name), copies it unchanged to results/experiment_results/flight_test_data/mail_box/<name>/, and
writes, under results/experiment_results/matlab_experiment_data/mail_box/:
  <name>.mat           struct `run` (meta, metrics, tracking, raw), the layout of the pick-and-place / pick-and-drop
                       experiment files (../pick_and_drop/), name = whole_body_l1_mail_run<N>, N = the bag's success_N
  mail_box_metrics.mat metrics_runs (one row per run), metrics_mean, metrics_phase (per run and phase), bags, L_arm_m
  configs/, README.md
A bag whose .db3 is not on disk (the download is split into zip parts) is listed and skipped; re-run once it is.

Same metric definitions as experiment_pick_and_place.py (the simulation's, build_pick_and_place.py): the planner's
whole-body reference, eps_UAV, rho_UAV, RMSE per phase and over the whole mission (start of go_to_start to the end of
the land leg / DIRECT), 100 Hz. The envelope carries no mocap markers: the outcome is the claw at the release (FK
of the measured pose and joints) against the planner's release point, and the operator's verdict (the bag name).
obj_0 in these bags is the localization frame used by the Gets on the ground; it lay on the floor during the flight.

Interpreters as experiment_pick_and_place.py (stage 2 re-executes this file with PYTHONNOUSERSITE=1).
"""
import argparse
import glob
import math
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import experiment_pick_and_place as E   # noqa: E402

RESULTS, EXP, CACHE = E.RESULTS, E.EXP, E.CACHE
BAGS = os.path.join(E.REPO, "docs", "experimental_data_ros2_bag")
E.BAG_ROOT = BAG_ROOT = os.path.join(EXP, "flight_test_data", "mail_box")
E.MAT_DIR = MAT_DIR = os.path.join(EXP, "matlab_experiment_data", "mail_box")
NAME = re.compile(r"^flight_wb_l1_4d_mail_box_(\d{8})_(\d{6})_success_(\d+)$")
# The flown configs. The bags show the dev_robotic_arm build (150-value wb_control_debug, added there 2026-11-03)
# and vehicle_name AM-T650-WB-L1-4D-HW-MAIL = the mail_delivery yaml, last changed in d957c26 (2026-11-03 20:27);
# the branch head at the flights (5d8cbac, 2026-11-04) did not change it. The arm yaml: f812b48 on every branch.
CONFIGS = [("whole_body_l1_4d_hw_mail_delivery.yaml", E.AUTOPILOT, "d957c26",
            "config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_mail_delivery.yaml"),
           ("arm_external_torque_controller_hardware_aerial_pwm.yaml", E.ARM_REPO, "f812b48",
            "open_manipulator_x_custom_controller/config/external_torque_controller_hardware_aerial_pwm.yaml")]
PHASES = [("go_to_start", "Go to start", "fly to the start waypoint; hover"),
          ("pick", "Pick", "fly to a hold 10 cm behind the envelope's grasp point, slide in level (the planner's "
                           "'descent' step), close the gripper on the envelope, climb 20 cm (exit)"),
          ("to_mailbox_start", "To mailbox start", "carry the envelope to the mailbox-start waypoint; hover"),
          ("insert", "Insert", "Ready To Insert: the arm sweeps to the place pose and the claw holds 0.25 m back along "
                               "the insertion line (pitched 25 deg down); Insert: slide 0.25 m in through the mouth; "
                               "open the gripper (the release); back out 10 cm and climb 20 cm (exit); hover"),
          ("to_land_start", "To land start", "fly to the land-start waypoint, arm folding home; hover"),
          ("land", "Land", "descend toward the landing point (z 0.35 m)")]
PHASE_KEYS = [p[0] for p in PHASES]


def discover():
    seen = {}
    for d in glob.glob(os.path.join(BAGS, "*", "*", "flight_wb_l1_4d_mail_box_*_success_*")):
        m = NAME.match(os.path.basename(d))
        if not m or not os.path.isdir(d):
            continue
        db3 = sorted(glob.glob(os.path.join(glob.escape(d), "*.db3")))    # the folders carry "[Success]"
        n = int(m.group(3))
        r = dict(tag=f"S-{n}", n=n, method="whole_body_l1", flown=f"2026-{m.group(1)[4:6]}-{m.group(1)[6:8]} "
                 f"{m.group(2)[:2]}:{m.group(2)[2:4]}", bag=m.group(0), dir=d, present=bool(db3),
                 has_meta=os.path.isfile(os.path.join(d, "metadata.yaml")), configs=CONFIGS,
                 payload="the red envelope (printed core wrapped in paper; no mocap markers), gripped flat with the "
                         "wrist rolled 90 deg")
        old = seen.get(n)
        if old is None or (r["present"] and not old["present"]):
            seen[n] = r
    return {f"whole_body_l1_mail_run{n}": r for n, r in sorted(seen.items())}


def copy_bags(runs):
    for name, r in runs.items():
        if not r["present"]:
            continue
        dst = os.path.join(BAG_ROOT, name)
        src = sorted(os.listdir(r["dir"]))
        if os.path.isdir(dst) and sorted(os.listdir(dst)) == src and all(
                os.path.getsize(os.path.join(dst, x)) == os.path.getsize(os.path.join(r["dir"], x)) for x in src):
            continue
        if os.path.isdir(dst):
            shutil.rmtree(dst)
        shutil.copytree(r["dir"], dst)
        if not r["has_meta"]:
            subprocess.run(["bash", "-c", f"source /opt/ros/humble/setup.bash && ros2 bag reindex '{dst}' -s sqlite3"],
                           check=True, stdout=subprocess.DEVNULL)
            open(os.path.join(dst, "REINDEXED.txt"), "w").write(
                "metadata.yaml was not in the downloaded zip part that held this bag's .db3: rebuilt here with\n"
                "`ros2 bag reindex <this folder> -s sqlite3` (the .db3 is unchanged).\n")
        print("copied", r["bag"], "->", os.path.relpath(dst, RESULTS), flush=True)


# ---------------------------------------------------------------- stage 2 (numpy 1.21 + scipy)
def convert(name, r):
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot
    os.environ["AM_NPZ"] = CACHE
    sys.path.insert(0, E.PP_TOOLS)
    import pp_common as PC
    import build_pick_and_place as B        # the simulation's definitions
    TP = PC.TP
    RZ90, RZM90 = TP._Rz(0.5 * math.pi), TP._Rz(-0.5 * math.pi)

    d, t0 = PC.load(name)
    cfg = E.copy_configs(r)
    wb_yaml = cfg[r["configs"][0][0]]
    ee_off = np.array(E._yaml_list(wb_yaml, "pick_place_ee_offset"))
    pitch_deg = E._yaml_float(wb_yaml, "pick_place_place_insert_pitch_deg")

    # ---- phases: the planner's pick-and-place status (FLYING <leg> edges)
    st = PC.edges(d, t0, "pp_status")
    starts = {}
    for leg in E.LEGS:
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
    to, Pod, Qod, Vod, Wod = PC.odom(d, t0)
    tj, qj, qdj = PC.joints_model(d, t0)
    tr, ref = PC.wbref(d, t0)
    t_start = starts["go_to_start"]
    t_end = min(land_end, tr[-1], to[-1], tj[-1])
    bounds = [starts[k] for k in E.LEGS] + [t_end]
    phases = {key: (bounds[i], bounds[i + 1]) for i, key in enumerate(PHASE_KEYS)}

    # ---- the 100 Hz tracking grid (experiment_pick_and_place.convert's computation, unchanged)
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

    # ---- the events (status edges, the operator's steps and the planner's replies in /rosout)
    log = [(float(t), str(nm), E.ANSI.sub("", str(msg))) for t, nm, msg in
           zip(d["rosout__recv"] - t0, d["rosout__name"], d["rosout__msg"]) if "rosbag2_recorder" not in str(nm)]
    first_st = lambda prefix, after=-1e9: next(t for t, v in st if t > after and v.startswith(prefix))   # noqa: E731
    first_log = lambda s, after=-1e9: next((t, msg) for t, nm, msg in log if t > after and s in msg)      # noqa: E731
    pick_slide = first_st("FLYING execute_pick (descent)")
    t_close, _ = first_log("gripper CLOSE", pick_slide)
    t_exit_pick = first_st("FLYING exit_pick")
    slide0 = first_st("FLYING execute_place (descent)")
    slide1 = first_st("DONE execute_place", slide0)
    t_open, open_msg = first_log("gripper OPEN", slide0)
    t_exit_place = first_st("FLYING exit_place")
    auto = "(automatic)" in open_msg
    rel = [msg for t, nm, msg in log if "AUTO RELEASE" in msg and slide0 < t <= t_open + 0.5]
    last_st = [v for t, v in st if slide1 <= t <= t_open]
    planner_note = (rel[0].split("AUTO RELEASE -- ")[-1] if rel else
                    (last_st[-1].split("auto release: ")[-1] if last_st and "auto release: " in last_st[-1] else ""))
    land_done = any(v.startswith(("DONE execute_land", "COMPLETE")) for t, v in st if t > land0)
    left = [msg for t, nm, msg in log if "PX4 LEFT OFFBOARD" in msg and abs(t - direct_b) < 0.5]
    nav = E.NAV_STATE.search(left[0]) if left else None
    direct_exit = ("the land leg completed" if land_done else
                   f"the land leg was cut short {direct_b - land0:.2f} s in: " +
                   (f"PX4 left OFFBOARD for nav_state {nav.group(1)} "
                    f"({E.NAV_NAME.get(int(nav.group(1)), 'see PX4 VehicleStatus')})"
                    if nav else "the controller left DIRECT"))

    # ---- the outcome: the claw at the release against the planner's release point, along the insertion line
    I = d["pp_info__data"]
    ok = np.where(I[:, 4] == 1)[0]                          # [4] planned
    goals = I[ok[-1], 14:56].reshape(6, 7)                  # per leg: base x y z, yaw [deg], claw x y z
    claw_pick, claw_rel = goals[1, 4:7], goals[3, 4:7]
    at = lambda t, X: np.array([np.interp(t, tu, X[:, k]) for k in range(X.shape[1])])   # noqa: E731
    at_ref = lambda t: np.array([np.interp(t, tr, ref["r_ed"][:, k]) for k in range(3)])  # noqa: E731
    line = at_ref(slide1) - at_ref(slide0)                  # the insertion line, from the reference slide
    u = line / np.linalg.norm(line)
    e_open = at(t_open, re) - claw_rel
    along = float(e_open @ u)
    off = float(np.linalg.norm(e_open - along * u))
    A = d["pp_arr__data"]; ta = d["pp_arr__recv"] - t0
    k_open = int(np.clip(np.searchsorted(ta, t_open), 0, len(ta) - 1))
    q4 = at(t_open, np.degrees(q))[3]; q4_ref = at(t_open, np.degrees(q_d))[3]
    outcome = dict(
        envelope_tracked=False, delivered=True,
        verdict="delivered: the operator's verdict (the bag folder name success_N); the envelope has no mocap markers",
        release_mode="automatic (the planner's release gates)" if auto else "by hand (the arm GS)",
        planner_release_note=planner_note,
        pick_claw_target_m=claw_pick, release_point_m=claw_rel, insert_heading_deg=float(goals[3, 3]),
        insert_line_unit=u, insert_slide_m=float(np.linalg.norm(line)), insert_pitch_deg=pitch_deg,
        insert_pitch_measured_deg=float(math.degrees(math.asin(-u[2]))), ee_offset_m=ee_off,
        pick_slide_start_s=pick_slide - t_start, gripper_close_s=t_close - t_start, exit_pick_s=t_exit_pick - t_start,
        claw_err_at_close_mm=1e3 * (at(t_close, re) - claw_pick),
        insert_slide_start_s=slide0 - t_start, insert_slide_end_s=slide1 - t_start, gripper_open_s=t_open - t_start,
        exit_place_s=t_exit_place - t_start, release_after_slide_s=t_open - slide1,
        claw_err_at_open_mm=1e3 * e_open, claw_along_at_open_mm=1e3 * along, claw_off_axis_at_open_mm=1e3 * off,
        planner_claw_err_at_open_mm=1e3 * A[k_open, 3:6], q4_at_open_deg=float(q4), q4_ref_at_open_deg=float(q4_ref),
        definition="release point = the insert leg's claw target (planner); insertion line = the reference claw path of "
                   "the slide; errors are measured claw (model FK of the odometry and joints) - target, world ENU; "
                   "along > 0 = deeper than the release point; planner_claw_err_at_open_mm is the planner's own "
                   "target - claw (PX4 attitude) at the OPEN command")

    def edges_struct(k):
        e = PC.edges(d, t0, k)
        return dict(t=np.array([x[0] for x in e]), label=np.array([x[1] for x in e], dtype=object))

    js_pos = d["js__position"]
    cur = d["js__effort"][:, :4][:, PC.JS_IDX]
    tw = d["wb__recv"] - t0
    tf, Pf, Qf, _ = PC.mocap_pose(d, t0, "obj_vrpn")
    phase_tab = dict(key=np.array(PHASE_KEYS, dtype=object), label=np.array([p[1] for p in PHASES], dtype=object),
                     start_s=np.array([phases[k][0] - t_start for k in PHASE_KEYS]),
                     end_s=np.array([phases[k][1] - t_start for k in PHASE_KEYS]))
    gap = lambda t: float(np.diff(t[(t >= t_start) & (t <= t_end)]).max())    # noqa: E731
    frame_mid = np.median(Pf, axis=0)
    meta = dict(
        name=name, tag=r["tag"], method=r["method"], method_long="Whole-body L1 impedance (proposed), 4-D attribution",
        run_index=r["tag"], flown=r["flown"], bag=r["bag"],
        task="mail delivery: pick an envelope lying flat on the pick hat, insert it through the red mailbox's mouth "
             "along a line pitched 25 deg down, release it in the mouth",
        scene="lab: the envelope starts on the pick pillar's hat; the red mailbox stands on the console table; the "
              "localization frame (OptiTrack rigid body obj_0) is used by the Gets on the ground, then put aside (it "
              f"lay at ({frame_mid[0]:.2f}, {frame_mid[1]:.2f}, {frame_mid[2]:.2f}) m during the flight)",
        plant="T650 aerial manipulator (hardware), total mass 3.746 kg in the controller model",
        feedback="EKF2-fused OptiTrack odometry (~100 Hz); joint encoders; the envelope is not tracked",
        payload=r["payload"], phases=phase_tab,
        window=f"the whole mission, t = 0 (start of go to start) to {t_end - t_start:.2f} s (end of the land leg / DIRECT)",
        L_arm_m=B.L_ARM, base_com_model_m=B.BASE_COM, aborted=False,
        t0_s=t_start, t0_driver_s=t_start,
        clock="raw streams: s since the bag's first message; tracking.t: s since the start of go to start "
              "(= raw t - meta.t0_s). meta.t0_driver_s equals meta.t0_s, so scripts written for the simulation files run unchanged",
        direct_enter_s=direct_a, direct_exit_s=direct_b, direct_exit=direct_exit,
        max_gap_s=dict(odom=gap(to), joints=gap(tj), reference=gap(tr)),
        leg_goals=dict(leg=np.array(E.LEGS, dtype=object), base_m=goals[:, 0:3], yaw_deg=goals[:, 3], claw_m=goals[:, 4:7]),
        configs=np.array([c[0] for c in r["configs"]], dtype=object),
        config_source=np.array([f"{os.path.basename(c[1])} {c[2]}:{c[3]}" for c in r["configs"]], dtype=object))
    raw = dict(
        odom=dict(t=to, pos_m=Pod, vel_mps=Vod, quat_wxyz=Qod, angvel_rad_s=Wod),
        joints=dict(t=tj, q_rad=qj, qdot_rad_s=qdj, current_counts=cur[np.all(np.isfinite(js_pos[:, :4]), axis=1)],
                    effort=(cur / PC.KT * PC.HW_SIGN)[np.all(np.isfinite(js_pos[:, :4]), axis=1)]),
        reference=dict(t=tr, r_ed=ref["r_ed"], r_ed_dot=ref["r_ed_dot"], x_cd=ref["x_cd"], x_cd_dot=ref["x_cd_dot"],
                       x_cd_ddot=ref["x_cd_ddot"], q_d_rad=ref["q_d"], qdot_d_rad_s=ref["qdot_d"],
                       b1_d_model=ref["b1_d"], b1_de=ref["b1_de"]),
        localization_frame=dict(t=tf, pos_m=Pf, quat_wxyz=Qf),
        gripper=dict(t=d["js__recv"] - t0, pos_m=js_pos[:, 4:6]),
        planner_claw_error=dict(t=ta, data=A, columns="[0] leg, [1] 1 = claw target, [2] |error| [m], [3..5] target - "
                                                      "measured, world [m], [6] tolerance [m], [7] within, [8] settled"),
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
        q_deg=np.degrees(q), q_ref_deg=np.degrees(q_d), q_err_deg=e_q,
        columns="platform_att_err_deg = [roll pitch heading] (body axes); xyz columns are world ENU")
    import scipy.io
    scipy.io.savemat(os.path.join(MAT_DIR, name + ".mat"),
                     {"run": dict(meta=meta, metrics=dict(window=window, per_phase=per_phase, outcome=outcome),
                                  tracking=tracking, raw=raw)},
                     do_compression=True, long_field_names=True, oned_as="column")
    print(f"wrote {os.path.relpath(os.path.join(MAT_DIR, name + '.mat'), RESULTS)}  window {window['window_s']:.2f} s  "
          f"eps max/rms {window['eps_uav_max_mm']:.1f}/{window['eps_uav_rms_mm']:.1f} mm  EE {window['ee_pos_mm']:.1f} mm  "
          f"release {'auto' if auto else 'hand'}: along {1e3 * along:+.1f} / off {1e3 * off:.1f} mm, "
          f"{t_open - slide1:.2f} s after the slide", flush=True)
    return dict(name=name, r=r, window=window, per_phase=per_phase, outcome=outcome, phases=phase_tab,
                t_end=t_end - t_start, meta=meta, z_end=float(P[-1, 2]),
                gains={k: E._yaml_float(wb_yaml, k) for k in ("wb_k_r", "wb_k_w", "wb_ky_x", "wb_dy_x", "wb_ky_psi",
                                                               "wb_dy_psi")})


def stage2(runs):
    import json
    import numpy as np
    import scipy.io
    import build_pick_and_place as B
    os.makedirs(MAT_DIR, exist_ok=True)
    out = [convert(name, r) for name, r in runs.items() if r["present"]]
    flat, ph_rows = [], []
    for o in out:
        w, oc = o["window"], o["outcome"]
        f = dict(name=o["name"], method=o["r"]["method"], run=o["r"]["tag"], total_window_s=w["window_s"])
        for k in ("eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms", "platform_pos_mm",
                  "platform_heading_deg", "platform_roll_deg", "platform_pitch_deg", "ee_pos_mm", "ee_heading_deg",
                  "tilt_max_deg"):
            f[k] = w[k]
        for j, ax in enumerate("xyz"):
            f[f"platform_{ax}_mm"] = w["platform_pos_xyz_mm"][j]; f[f"ee_{ax}_mm"] = w["ee_pos_xyz_mm"][j]
        for j in range(4):
            f[f"q{j + 1}_deg"] = w["joints_deg"][j]
        f.update(release_along_mm=oc["claw_along_at_open_mm"], release_off_axis_mm=oc["claw_off_axis_at_open_mm"],
                 release_err_norm_mm=float(np.linalg.norm(oc["claw_err_at_open_mm"])),
                 pick_err_norm_mm=float(np.linalg.norm(oc["claw_err_at_close_mm"])),
                 release_after_slide_s=oc["release_after_slide_s"], release_automatic=float("automatic" in oc["release_mode"]))
        flat.append(f)
        for key in PHASE_KEYS:
            p = o["per_phase"][key]
            ph_rows.append(dict(name=o["name"], method=o["r"]["method"], run=o["r"]["tag"], phase=key, label=p["label"],
                                start_s=p["start_s"], end_s=p["end_s"], eps_uav_max_mm=p["eps_uav_max_mm"],
                                eps_uav_rms_mm=p["eps_uav_rms_mm"], rho_uav_max=p["rho_uav_max"],
                                rho_uav_rms=p["rho_uav_rms"], ee_pos_mm=p["ee_pos_mm"],
                                ee_heading_deg=p["ee_heading_deg"], platform_pos_mm=p["platform_pos_mm"],
                                tilt_max_deg=p["tilt_max_deg"]))
    bags = [dict(name=name, run=r["tag"], flown=r["flown"], bag=r["bag"],
                 status="exported" if r["present"] else "db3 not in the download") for name, r in runs.items()]
    res = {"bags": B.colstruct(bags, list(bags[0])), "L_arm_m": B.L_ARM}
    if flat:
        mean = [dict(method=m, runs=float(sum(1 for f in flat if f["method"] == m)),
                     **{k: float(np.mean([f[k] for f in flat if f["method"] == m]))
                        for k in flat[0] if k not in ("name", "method", "run")})
                for m in sorted({f["method"] for f in flat})]
        res.update(metrics_runs=B.colstruct(flat, list(flat[0])), metrics_mean=B.colstruct(mean, list(mean[0])),
                   metrics_phase=B.colstruct(ph_rows, list(ph_rows[0])))
    scipy.io.savemat(os.path.join(MAT_DIR, "mail_box_metrics.mat"), res, do_compression=True, long_field_names=True,
                     oned_as="column")
    print("wrote", os.path.relpath(os.path.join(MAT_DIR, "mail_box_metrics.mat"), RESULTS), flush=True)
    write_readme(runs, out, flat)
    json.dump(flat, open(os.path.join(CACHE, "mail_box_scores.json"), "w"), indent=1, default=float)


def write_readme(runs, out, flat):
    import numpy as np
    from readme_text import _md_table
    f1 = lambda x: f"{x:.1f}"    # noqa: E731
    missing = [r for r in runs.values() if not r["present"]]
    o0 = out[0] if out else None
    run_rows = []
    for name, r in runs.items():
        o = next((x for x in out if x["name"] == name), None)
        if o is None:
            run_rows.append([r["tag"], r["flown"], f"`{r['bag']}`", "**.db3 not in the download**", "--", "--", "--",
                             "--", "--", "--"])
            continue
        w, oc = o["window"], o["outcome"]
        run_rows.append([r["tag"], r["flown"], f"`{r['bag']}`", f"`{name}.mat`", f"{w['window_s']:.1f}",
                         f"{f1(w['eps_uav_max_mm'])} / {f1(w['eps_uav_rms_mm'])}", f1(w["ee_pos_mm"]),
                         "automatic" if "automatic" in oc["release_mode"] else "by hand",
                         f"{oc['claw_along_at_open_mm']:+.0f} / {oc['claw_off_axis_at_open_mm']:.0f}",
                         f"{oc['release_after_slide_s']:.2f}"])
    phase_rows = []
    if out:
        for i, k in enumerate(PHASE_KEYS):
            vals = [o["per_phase"][k] for o in out]
            phase_rows.append([PHASES[i][1], PHASES[i][2], f1(np.mean([v["end_s"] - v["start_s"] for v in vals])),
                               f1(np.mean([v["eps_uav_max_mm"] for v in vals])), f1(np.mean([v["eps_uav_rms_mm"] for v in vals])),
                               f1(np.mean([v["ee_pos_mm"] for v in vals])), f1(max(v["tilt_max_deg"] for v in vals))])
    L = ["# Experiment results, MATLAB data (mail delivery)", "",
         "The hardware mail-delivery flights of 2026-11-05, converted from their ros2 bags. The folder is",
         "self-contained: these files plus MATLAB are all you need, with no ROS or custom message definitions. The",
         "`run` struct has the layout of `../pick_and_drop/` and `../pick_and_place/` (the same six legs, the same",
         "metric definitions), so a script written for those files runs on these.", ""]
    if missing:
        L += ["> **Incomplete download.** The Google Drive download of the 11-05 bags is split into several zip parts",
              "> and only the first part is on disk. No .db3 for: " + ", ".join(f"`{r['bag']}`" for r in missing) + ".",
              "> Unpack the other parts into `docs/experimental_data_ros2_bag/1105 - .../` and re-run (below).", ""]
    L += ["## Files", "", "```",
          "whole_body_l1_mail_run<N>.mat     one flight, struct `run`; N = the bag's success_N",
          "mail_box_metrics.mat              metrics_runs (one row per flight), metrics_mean, metrics_phase, bags, L_arm_m",
          "configs/                          the controller + planner and the arm controller, as flown (from git)",
          "```", "",
          "```matlab",
          "r  = load('whole_body_l1_mail_run1.mat').run;",
          "P  = r.meta.phases;  tr = r.tracking;  oc = r.metrics.outcome;",
          "plot(tr.t, 1e3 * tr.eps_uav_m); xline(oc.gripper_open_s, '--', 'release')     % ||eps_UAV|| [mm]",
          "plot3(tr.ee_pos_m(:,1), tr.ee_pos_m(:,2), tr.ee_pos_m(:,3))                    % the claw",
          "S = load('mail_box_metrics.mat');  T = struct2table(S.metrics_runs);",
          "```", "",
          "Raw streams are on the bag's clock: subtract `r.meta.t0_s` to put them on `tracking.t`.", "",
          "## The flights", "",
          _md_table(["run", "flown", "original ros2 bag", "file", "window [s]", "‖ε<sub>UAV</sub>‖ max / RMS [mm]",
                     "EE RMSE [mm]", "release", "claw at release: along / off the line [mm]",
                     "release after the slide [s]"], run_rows), "",
          "- All five were delivered (the operator's verdict: the bag folders are named `success_N`). The envelope has",
          "  no mocap markers, so the outcome recorded here is the CLAW at the release against the planner's release",
          "  point: along the insertion line (> 0 = deeper) and off it. The planner's own automatic release fires inside",
          "  −15 / +20 mm along and 30 mm off, with the claw slower than 0.05 m/s for 1 s.",
          "- **release:** automatic = the planner's release gates opened the gripper (`pick_place/drop_now`); by hand =",
          "  the operator opened it from the arm ground station before the gates passed.",
          "- **End of the window** (`run.meta.direct_exit`): " +
          "; ".join(f"{o['r']['tag']}: {o['meta']['direct_exit']}" for o in out) + "." if out else "", ""]
    if out:
        L += ["## Phases (mean over the flights)", "",
              _md_table(["phase", "what happens", "duration [s]", "eps max [mm]", "eps rms [mm]", "EE RMSE [mm]",
                         "tilt max [deg]"], phase_rows), "",
              "Each phase runs from its leg's `FLYING` status to the next leg's (the operator's waits belong to the",
              "phase they end). The window is the whole mission, from the start of go to start to the end of the land",
              "leg or of DIRECT. Per-flight values: `metrics_phase`.", ""]
    if o0:
        oc, g = o0["outcome"], o0["gains"]
        L += ["## Conditions", "",
              "- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm in torque mode, 3.746 kg in the",
              "  controller model.",
              "- **Feedback:** OptiTrack fused in PX4 EKF2 (~100 Hz); joint encoders.",
              "- **Controller:** whole-body L1, 4-D attribution, `AM_HW_PROFILE=mail_delivery`",
              f"  (`configs/whole_body_l1_4d_hw_mail_delivery.yaml`: k_R / k_w {g['wb_k_r']:g} / {g['wb_k_w']:g},",
              f"  K_y / D_y {g['wb_ky_x']:g} / {g['wb_dy_x']:g}, K_psi / D_psi {g['wb_ky_psi']:g} / {g['wb_dy_psi']:g}).",
              "  The bags show the `dev_robotic_arm` build (vehicle `AM-T650-WB-L1-4D-HW-MAIL`); the yaml is that",
              "  branch's file at d957c26, unchanged at the branch head flown.",
              "- **Arm poses:** pick = carry [0, 36, 44, 90] deg (the jaws close vertically), place [0, 20, 45, 90] deg",
              "  (the claw pitched 25 deg down), home [0, 40, 40, 0] deg.",
              f"- **EE offset** (the claw point in the envelope's frame): ({oc['ee_offset_m'][0]:.2f}, "
              f"{oc['ee_offset_m'][1]:.2f}, {oc['ee_offset_m'][2]:.2f}) m.",
              f"- **Insert:** a {oc['insert_slide_m']:.2f} m slide along a line pitched {oc['insert_pitch_measured_deg']:.1f}"
              " deg down, heading", f"  {oc['insert_heading_deg']:.1f} deg; release point "
              f"({oc['release_point_m'][0]:.3f}, {oc['release_point_m'][1]:.3f}, {oc['release_point_m'][2]:.3f}) m in "
              "the first flight", "  (each flight's in `run.metrics.outcome`).",
              "- **Configs:** taken from git at the commits the vehicle ran (first line of each file).", "",
              "## The struct `run`", "",
              "| field | content |", "|---|---|",
              "| `meta` | name, run tag (S-N), method, flight date, original bag, phase table, window, clock, DIRECT entry/exit, "
              "largest gap per stream, leg goals, configs |",
              "| `metrics` | `window` (whole mission), `per_phase.<phase>`, `outcome` (the pick and the release, below) |",
              "| `tracking` | 100 Hz over the mission, `t` in s since the start of go to start. Measured, reference (`_ref_`) "
              "and error (`_err_`) for the platform position and attitude, `eps_uav_m` / `rho_uav`, the system CoM, the "
              "end-effector position and heading, the joints (`q_deg`). No payload column: the envelope is not tracked |",
              "| `raw` | the bag's streams, time in s since the bag's first message: `odom`, `joints` (model convention), "
              "`reference` (the planner's whole-body reference), `localization_frame` (obj_0; static, see above), "
              "`gripper` (finger positions), `planner_claw_error` (`pick_place/arrival_error`), `controller_debug` "
              "(`wb_control_debug`), `motors` (normalised), `battery`, `mocap_vrpn` (vehicle), `events.mode` / "
              "`events.planner_status` / `events.pick_place_status` (transitions), `events.log` (every node's /rosout line: "
              "the operator's steps and the planner's replies) |", "",
              "`run.metrics.outcome`:", "",
              "- `release_point_m`, `insert_line_unit`, `insert_slide_m`, `insert_pitch_deg` (yaml), "
              "`insert_pitch_measured_deg` (the reference slide), `insert_heading_deg`.",
              "- `claw_err_at_open_mm` (xyz, measured − release point), `claw_along_at_open_mm`, "
              "`claw_off_axis_at_open_mm`, `planner_claw_err_at_open_mm` (the planner's own, target − measured),",
              "  `q4_at_open_deg` / `q4_ref_at_open_deg`, `release_mode`, `planner_release_note` (the planner's text).",
              "- Pick: `pick_claw_target_m`, `claw_err_at_close_mm`. Times [s since go to start]: `pick_slide_start_s`,",
              "  `gripper_close_s`, `exit_pick_s`, `insert_slide_start_s`, `insert_slide_end_s`, `gripper_open_s`,",
              "  `exit_place_s`, `release_after_slide_s`.", "",
              "Units: positions in m (world ENU, z up) unless the name says mm; angles in deg unless it says rad. Errors",
              "are measured − reference. Definitions: `../pick_and_drop/README.md` (the simulation's, "
              "`results/utils/build_pick_and_place.py`); ρ<sub>UAV</sub> = ε<sub>UAV</sub> / L, L = 0.370 m.", ""]
    L += ["The bags are copied unchanged to `results/experiment_results/flight_test_data/mail_box/<name>/`; the",
          "originals are in `docs/experimental_data_ros2_bag/1105 - T650-AM Mail-Box, Robust Tracking, Box-Pushing-*/"
          "Mail-Box [Success]/`.", "",
          "Regenerate:", "", "```bash", "/usr/bin/python3 results/utils/experiment_mail_box.py", "```", ""]
    with open(os.path.join(MAT_DIR, "README.md"), "w") as fh:
        fh.write("\n".join(L))
    print("wrote", os.path.relpath(os.path.join(MAT_DIR, "README.md"), RESULTS), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage2", action="store_true")
    a = ap.parse_args()
    runs = discover()
    if a.stage2:
        sys.path.insert(0, HERE)
        stage2(runs)
        return
    for name, r in runs.items():
        print(f"{r['tag']:4s} {r['bag']:56s} {'db3' if r['present'] else 'NO .db3'}", flush=True)
    copy_bags(runs)
    E.RUNS = {name: r for name, r in runs.items() if r["present"]}
    E.extract()
    env = dict(os.environ, PYTHONNOUSERSITE="1", AM_EXP_CACHE=CACHE)
    subprocess.run(["/usr/bin/python3", os.path.abspath(__file__), "--stage2"], env=env, check=True)


if __name__ == "__main__":
    main()
