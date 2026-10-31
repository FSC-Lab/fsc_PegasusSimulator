#!/usr/bin/env python3
"""Box pushing, EXPERIMENT: hardware runs -> the simulation's box-pushing `run` struct, for MATLAB.

    /usr/bin/python3 results/utils/experiment_box_pushing.py

Reads each run's ros2 bag from results/experiment_results/flight_test_data/box_pushing/<name>/ (copied there unchanged) and
writes, under results/experiment_results/matlab_experiment_data/box_pushing/:
  <name>.mat                 struct `run`: meta, metrics, tracking, impedance, raw -- the layout of
                             simulation_results/matlab_simulation_data/box_pushing/<method>_push_run<k>.mat
  box_pushing_metrics.mat    metrics_runs, metrics_mean, metrics_std, metrics_phase, metrics_impedance, phases,
                             phases_runs, window_s, L_arm_m
  configs/                   the controller and arm configs as flown (taken from git at the flown commit)
  README.md                  the folder's guide (self-contained, like the simulation's)
Extracted bags are cached in $AM_EXP_CACHE (default /tmp/am_experiment_cache), outside the repository.

The hardware mission is the simulation's: go to start, ready, grasp the fin, push, release, exit, go to land start,
land, with the arm ground station running the steps (its automatic Ready / Push / Exit). The phases come from the
planner's push-and-pull status and the station's log in the bag, not from a timetable. Both 2026-10-27 runs ended in
a planner safety abort (the q3 guard), which replaced the normal exit; the phases say where.

METRICS: the simulation's definitions, imported from build_box_pushing.py (metrics, imp_metrics, build_r0, L_ARM, G,
BASE_COM, the eight phases, the impedance windows):
  reference   the planner's WholeBodyReference: platform x_b = x_cd - R0 r_0c(q_d), R0 = the compatible attitude
              (thrust along x_cd_ddot + g e3, heading b1_d); EE r_ed, EE heading b1_de; joints q_d
  measured    odometry (raw OptiTrack position + velocity, PX4 attitude) + joint encoders; EE position / heading by
              the model FK
  eps_UAV     |x_b - r_UAV|, max and RMS; rho_UAV = eps / L, L = 0.370 m
  impedance   e_imp = F_ext - (M_d e_a + D_d e_v + K_d e_y) on [x y z heading], every term low-passed at 5 Hz (zero
              phase), with the gains the run flew. THE ONE DIFFERENCE: the hardware has no force sensor, so F_ext is
              the controller's own contact-force ESTIMATE (the 4-D L1 attribution F_hat, wb_control_debug [97..100]),
              where the simulation uses the PhysX contact force. A box-model force along the push (mass x box
              acceleration + Coulomb friction) is exported beside it as an independent check.
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
BAG_ROOT = os.path.join(EXP, "flight_test_data", "box_pushing")
MAT_DIR = os.path.join(EXP, "matlab_experiment_data", "box_pushing")
ARCH = os.path.join(REPO, "docs", "docs_aerial_manipulator", "archive")
EXTRACT = os.path.join(ARCH, "wb_box_pushing_flight_20261021", "tools", "extract.py")   # 10-09 extractor + push-pull
NP1 = os.path.join(ARCH, "wb_pick_place_flight_20261009", "tools", "np1_compat.py")
PP_TOOLS = os.path.join(ARCH, "wb_pick_place_flight_20261009", "tools")                  # pp_common -> 0928 common
ANALYSIS = "docs/docs_aerial_manipulator/archive/wb_box_pushing_flight_20261027/README.md"
CACHE = os.environ.get("AM_EXP_CACHE", "/tmp/am_experiment_cache")
AUTOPILOT = os.environ.get("FSC_AUTOPILOT_REPO", os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2"))
ARM_REPO = os.environ.get("FSC_OM_ARM_REPO", os.path.expanduser("~/ros2_ws/src/fsc_open_manipulator"))

# (name in configs/, repository, commit, path in the repository): the files the 2026-10-27 flights ran.
# fsc_autopilot_ros2 a316625 (2026-10-25) is the only box-pushing config after 10-24 and the bag shows its features
# (push pose [0, 28, 32, 0] in the hold captures, the abort climbing straight up). The arm file last changed in
# f812b48; a0cb8b8 is the arm branch's head at the flights.
CONFIGS = [("whole_body_l1_4d_hw_box_pushing.yaml", AUTOPILOT, "a316625",
            "config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_box_pushing.yaml"),
           ("arm_external_torque_controller_hardware_aerial_pwm.yaml", ARM_REPO, "a0cb8b8",
            "open_manipulator_x_custom_controller/config/external_torque_controller_hardware_aerial_pwm.yaml")]
# The exported runs. name = <method>_push_<tag>, the simulation's naming with the flight tag for the run number.
RUNS = {
    "whole_body_l1_push_BP-1": dict(
        tag="BP-1", method="whole_body_l1", flown="2026-10-27 17:11",
        bag="flight_wb_l1_4d_box_pushing_300g_20261027_171114",
        what="0.40 m push COMPLETE (box 387 mm at the push end, 405 mm after the release); after the station "
             "released the claw and opened the jaws, the q3 rate guard aborted -- the abort's climb replaced the "
             "exit; box untouched, upright; normal way home and landing"),
    "whole_body_l1_push_BP-2": dict(
        tag="BP-2", method="whole_body_l1", flown="2026-10-27 17:22",
        bag="flight_wb_l1_4d_box_pushing_300g_20261027_172227",
        what="push stopped 1.5 s before its end (box 348 of 400 mm) by the q3 static guard; in the abort q3 went "
             "below 0 for 0.64 s (min -3.8 deg); box untouched, upright; normal way home and landing"),
}
BOX_MASS_KG = 0.280          # the real box with its handle (user, 2026-10-25; the bag names say 300 / 500 g)
BOX_MU = 0.25                # the 2026-10-17 spring-gauge bench test of the real box on the experiment table
BOX_SGN_DEADBAND = 0.005     # [m/s] the box-model force's friction sign: tanh(v / this)
PHASES_TEXT = {
    "go_to_start": "fly from the take-off hover to the point 0.12 m behind and 0.15 m above the fin, round the "
                   "table, the arm unfolding to the push pose; hover",
    "ready": "open the gripper, descend diagonally onto the fin; the claw held by the room once it has settled "
             "within 25 mm of the grasp point",
    "grasp": "the station closes the gripper on the fin; the push starts on the grasp",
    "push": "the law in CONTACT: 1.5 s settle, then the 0.40 m slide at 21 mm/s; then the hold on the handle "
            "until the release (BP-1: the station's 3 s calm wait)",
    "release": "the claw's reference back to the vehicle and the jaws open (BP-1: the station's exit; BP-2: the "
               "abort's 1.5 s release hold)",
    "exit": "the retreat: here the safety abort's straight climb of 0.30 m and the arm folding home (the normal "
            "exit climbs 0.25 m); hover",
    "go_to_land_start": "fly round the table's far end to the land-start hover",
    "land": "descend toward the landing hover (z 0.30 m); the pilot takes over before the leg ends",
}
FLIP_TILT_DEG = 60.0
ANSI = re.compile(r"\x1b\[[0-9;]*m")
NAV_STATE = re.compile(r"nav_state (\d+)")
NAV_NAME = {15: "STAB, the pilot's mode switch", 18: "AUTO_LAND", 5: "AUTO_RTL", 4: "AUTO_LOITER"}
FC_HZ = 5.0


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
def copy_configs():
    """configs/<name> <- `git show <commit>:<path>`; returns {name: text}. Keeps an existing copy if git is missing."""
    cfg = os.path.join(MAT_DIR, "configs")
    os.makedirs(cfg, exist_ok=True)
    out = {}
    for dst, repo, rev, path in CONFIGS:
        p = os.path.join(cfg, dst)
        try:
            text = subprocess.run(["git", "-C", repo, "show", f"{rev}:{path}"], check=True, capture_output=True,
                                  text=True).stdout
            open(p, "w").write(f"# As flown: {os.path.basename(repo)} commit {rev}, {path}\n" + text)
        except (subprocess.CalledProcessError, FileNotFoundError):
            if not os.path.isfile(p):
                raise SystemExit(f"cannot read {rev}:{path} from {repo}, and configs/{dst} does not exist")
            print(f"WARNING: git unavailable for {dst}; keeping the existing copy", flush=True)
        out[dst] = open(p).read()
    return out


def gains_of(text):
    import numpy as np
    return {k: np.array([float(re.search(rf"^\s*wb_{k}_{a}:\s*([-+0-9.eE]+)", text, re.M).group(1))
                         for a in ("x", "y", "z", "psi")]) for k in ("my", "dy", "ky")}


def lpf(x, fs, fc):
    from scipy.signal import butter, filtfilt
    b, a = butter(2, fc / (0.5 * fs))
    return filtfilt(b, a, x, axis=0)


def stats(v):
    import numpy as np
    v = np.asarray(v)
    return (float(np.sqrt(np.mean(v ** 2))), float(np.max(np.abs(v)))) if len(v) else (np.nan, np.nan)


def marks_of(name, PC, d, t0, direct_b):
    """The simulation's driver marks, from the planner's push-and-pull status and the arm station's log."""
    st = PC.edges(d, t0, "pl_pstatus")
    log = [(float(t), str(nm), ANSI.sub("", str(msg))) for t, nm, msg in
           zip(d["rosout__recv"] - t0, d["rosout__name"], d["rosout__msg"]) if "rosbag2_recorder" not in str(nm)]

    def first(prefix, after=-1e9):
        return next((t for t, v in st if v.startswith(prefix) and t > after), None)

    m = {"go_to_start:start": first("FLYING go_to_start"), "ready:start": first("FLYING ready"),
         "ready:end": first("DONE ready"), "push:start": first("FLYING push")}
    if None in m.values():
        raise SystemExit(f"{name}: no go_to_start / ready / push in the status -- not a box push")
    m["close:start"] = next(t for t, nm, msg in log if "gripper CLOSE" in msg and t > m["ready:end"] - 0.5)
    ab = first("ABORTING")
    abort_text = next((v for t, v in st if v.startswith("ABORTING")), "")
    done_push = first("DONE push", m["push:start"])
    if ab is not None and (done_push is None or done_push > ab):
        # the abort cut the push: its own release (contact off, CoM anchor, jaws open), then its climb
        m["push:end"] = m["release:start"] = ab
        plan = next((msg for t, nm, msg in log if "push-and-pull ABORT: [1] settle" in msg and t >= ab - 0.1), "")
        hold = re.search(r"\[1\] settle ([0-9.]+) s", plan)
        m["exit:start"] = ab + (float(hold.group(1)) if hold else 0.0)
    else:
        m["push:end"] = done_push
        # the release = the claw's reference handed back to the vehicle (the station's release call)
        rel = next((t for t, nm, msg in log if "EE reference anchor -> CoM" in msg and t > done_push
                    and (ab is None or t <= ab + 0.01)), None)
        m["release:start"] = rel if rel is not None else done_push
        m["exit:start"] = ab if ab is not None else first("FLYING exit", done_push)
    m["go_to_land_start:start"] = first("FLYING go_to_land_start", m["exit:start"])
    m["land:start"] = first("FLYING land", m["go_to_land_start:start"])
    land_done = first("DONE land", m["land:start"])
    m["land:end"] = min(land_done, direct_b) if land_done is not None else direct_b
    m["exit:end"] = m["go_to_land_start:start"]
    m["push:slide_start"] = m["push:start"] + 1.5
    m["push:slide_end"] = first("DONE push", m["push:start"]) if ab is None or (done_push and done_push < ab) else ab
    return m, st, log, ab, abort_text


def convert(name, r, cfg):
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot
    os.environ["AM_NPZ"] = CACHE
    sys.path.insert(0, PP_TOOLS)
    sys.path.insert(0, HERE)
    import pp_common as PC
    import build_box_pushing as B           # the simulation's definitions
    import experiment_pick_and_place as EPP  # the mocap-body outlier test
    TP = PC.TP
    RZ90, RZM90 = TP._Rz(0.5 * math.pi), TP._Rz(-0.5 * math.pi)

    d, t0 = PC.load(name)
    wb_yaml = cfg[CONFIGS[0][0]]
    gains = gains_of(wb_yaml)
    # DIRECT from the node's mode edges. (pp_common.direct_span also cuts at the first odometry gap > 0.5 s,
    # for the 10-09 PX4 link freeze; here a gap is the laptop-side recorder losing the Orin's topics over WiFi
    # while the vehicle kept flying in DIRECT -- kept in the window and flagged in tracking.recording_gap)
    md = PC.edges(d, t0, "wbmode")
    direct_a = next(t for t, v in md if v == "DIRECT")
    direct_b = next((t for t, v in md if v == "SAFETY" and t > direct_a), float(PC.wbdebug(d, t0)[0][-1]))
    mk, st, log, t_abort, abort_text = marks_of(name, PC, d, t0, direct_b)
    to, Pod, Qod, Vod, Wod = PC.odom(d, t0)
    tj, qj, qdj = PC.joints_model(d, t0)
    tr, ref = PC.wbref(d, t0)
    rdd_ref = np.column_stack([d[f"wbref__r_ed_ddot.{k}"] for k in "xyz"])
    t_start = mk["go_to_start:start"]
    t_end = min(mk["land:end"], tr[-1], to[-1], tj[-1])
    PHASE_KEYS = B.PHASE_KEYS
    phases = {key: (mk[a_], mk[b_] if b_ != "land:end" else t_end) for key, _, a_, b_ in B.PHASES}

    # ---- the 100 Hz tracking grid, the simulation's computation (build_box_pushing.load_run)
    tu = np.arange(t_start, t_end, B.DT)
    n = len(tu)
    P = B.interp(tu, to, Pod)
    Qu = B.interp(tu, to, Qod[:, [1, 2, 3, 0]]); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    Ra = Rot.from_quat(Qu).as_matrix()
    q = B.interp(tu, tj, qj)
    r_ed, x_cd, q_d = (B.interp(tu, tr, ref[k]) for k in ("r_ed", "x_cd", "q_d"))
    b1d, x_cd_dd, b1de = (B.interp(tu, tr, ref[k]) for k in ("b1_d", "x_cd_ddot", "b1_de"))
    params = TP.make_params_t650(base_com=B.BASE_COM)
    xb = np.zeros((n, 3)); Rref = np.zeros((n, 3, 3)); xc = np.zeros((n, 3)); re_ = np.zeros((n, 3)); b1e = np.zeros((n, 3))
    for i in range(n):
        R0m = B.build_r0(x_cd_dd[i] + np.array([0.0, 0.0, B.G]), b1d[i])
        r0c_d, _, _ = TP.arm_fk_model(q_d[i], params)
        xb[i] = x_cd[i] - R0m @ r0c_d
        Rref[i] = R0m @ RZ90
        Rm = Ra[i] @ RZM90
        r0c, r0e, Re = TP.arm_fk_model(q[i], params)
        xc[i] = P[i] + Rm @ r0c; re_[i] = P[i] + Rm @ r0e; b1e[i] = Rm @ Re[:, 0]
    att_err = np.degrees(Rot.from_matrix(np.einsum("nji,njk->nik", Rref, Ra)).as_rotvec())
    az = np.degrees(np.unwrap(np.arctan2(b1e[:, 1], b1e[:, 0])))
    az_ref = np.degrees(np.unwrap(np.arctan2(b1de[:, 1], b1de[:, 0])))
    ee_head_err = np.degrees(B.wrap(np.radians(az - az_ref)))
    e_pos = P - xb; e_ee = re_ - r_ed; e_q = np.degrees(q - q_d)
    eps = np.linalg.norm(e_pos, axis=1)
    tilt = np.degrees(np.arccos(np.clip(Ra[:, 2, 2], -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(Ra[:, 1, 0], Ra[:, 0, 0])))
    yaw_ref = np.degrees(np.unwrap(np.arctan2(Rref[:, 1, 0], Rref[:, 0, 0])))
    R = dict(e_pos=e_pos, e_ee=e_ee, e_q=e_q, att_err=att_err, ee_head_err=ee_head_err, eps=eps, tilt=tilt,
             phases=phases, tu=tu, q=q)
    phase_id = np.zeros(n)
    per_phase = {}
    for i, key in enumerate(PHASE_KEYS):
        a_, b_ = phases[key]
        m = (tu >= a_) & (tu < b_)
        phase_id[m] = i + 1
        per_phase[key] = dict(label=B.PHASE_LABEL[key], start_s=a_ - t_start, end_s=b_ - t_start, **B.metrics(R, m))
    window = B.metrics(R, np.ones(n, bool))

    # ---- the box (obj_0, raw OptiTrack VRPN pose), mis-solved samples removed
    tb, Pb, Qb, _ = PC.mocap_pose(d, t0, "obj_vrpn")
    flip = PC.tilt_of(Qb) > FLIP_TILT_DEG
    tb, Pb, Qb = tb[~flip], Pb[~flip], Qb[~flip]
    out_ = EPP.basket_outliers(Pb, PC.yaw_of(Qb))
    tb, Pb, Qb = tb[~out_], Pb[~out_], Qb[~out_]
    k_od = np.clip(np.searchsorted(to, tu), 1, len(to) - 1)
    rec_gap = (to[k_od] - to[k_od - 1] > 0.1) & (to[k_od] > tu) & (to[k_od - 1] <= tu)   # tu inside an odom gap
    near = np.abs(tb[np.clip(np.searchsorted(tb, tu), 0, len(tb) - 1)] - tu) <= 0.1
    box_u = B.interp(tu, tb, Pb); box_u[~near] = np.nan

    def box_at(t):
        k = min(np.searchsorted(tb, t), len(tb) - 1)
        return Pb[k], Qb[k]
    bs, qs = box_at(mk["push:start"]); be, qe = box_at(mk["push:end"])
    ex0, _ = box_at(mk["exit:start"]); ex1, _ = box_at(mk["exit:end"])
    boxm = dict(pushed_mm=float(1e3 * np.linalg.norm(be[:2] - bs[:2])), pushed_xyz_mm=(1e3 * (be - bs)).tolist(),
                yaw_change_deg=float(np.degrees(B.wrap(PC.yaw_of(qe[None])[0] - PC.yaw_of(qs[None])[0]))),
                exit_move_mm=float(1e3 * np.linalg.norm(ex1[:2] - ex0[:2])),
                final_pos_m=Pb[-1].tolist(), start_pos_m=Pb[0].tolist(),
                tilt_max_deg=float(PC.tilt_of(Qb[(tb >= t_start) & (tb <= t_end)]).max()))

    # ---- the impedance residual over CONTACT (ready end -> exit start), the simulation's terms on the 100 Hz grid
    tw, W = PC.wbdebug(d, t0)
    a_c, b_c = mk["ready:end"], mk["exit:start"]
    mc = (tu >= a_c) & (tu <= b_c)
    t = tu[mc]; fs = 1.0 / B.DT
    c = re_[mc]
    rd = B.interp(t, tr, ref["r_ed_dot"]); rdd = B.interp(t, tr, rdd_ref)
    hd = np.interp(t, tw, W[:, 27])                                   # e_y[3] = sin(heading error)
    e_y = np.column_stack([c - r_ed[mc], hd])
    e_y_f = lpf(e_y, fs, FC_HZ)
    ydot = np.gradient(e_y_f, t, axis=0)
    cdot = np.gradient(lpf(c, fs, FC_HZ), t, axis=0)
    cddot = np.gradient(cdot, t, axis=0)
    e_v = np.column_stack([cdot - lpf(rd, fs, FC_HZ), ydot[:, 3]])
    e_a = np.column_stack([cddot - lpf(rdd, fs, FC_HZ), np.gradient(ydot[:, 3], t)])
    Fhat = B.interp(t, tw, W[:, 97:101])                              # the 4-D attribution's F_hat, on the claw
    Fx_f = lpf(Fhat, fs, FC_HZ)
    rendered = gains["my"] * e_a + gains["dy"] * e_v + gains["ky"] * e_y_f
    e_imp = Fx_f - rendered
    delta = e_imp[:, :3] / gains["ky"][:3]
    # the box-model force on the claw along the push: -(m a + mu m g tanh(v / v0)), from the box's mocap
    I = d["pl_info__data"]
    ok = np.where(np.isfinite(I[:, 79]))[0]
    info = I[ok[-1]]
    yaw_t = math.radians(info[81]); u_push = np.array([math.cos(yaw_t), math.sin(yaw_t)])
    bx = lpf(B.interp(t, tb, Pb[:, :2]), fs, FC_HZ) @ u_push
    vb = np.gradient(bx, t); ab_ = np.gradient(vb, t)
    F_model = -(BOX_MASS_KG * ab_ + BOX_MU * BOX_MASS_KG * B.G * np.tanh(vb / BOX_SGN_DEADBAND))
    F_hat_along = Fx_f[:, :2] @ u_push
    wins = {"CONTACT": (a_c, b_c), "HOLD": (a_c, mk["push:slide_start"]),
            "SLIDE": (mk["push:slide_start"], mk["push:slide_end"]), "RELEASE": (mk["push:slide_end"], b_c)}
    imp = {}
    for k, (a_, b_) in wins.items():
        m = (t >= a_) & (t <= b_)
        if m.sum() < 5:
            continue
        imp[k] = dict(T=b_ - a_, e_xyz=stats(np.linalg.norm(e_imp[m, :3], axis=1)), e_psi=stats(e_imp[m, 3]),
                      F_xyz=stats(np.linalg.norm(Fx_f[m, :3], axis=1)), F_psi=stats(Fx_f[m, 3]),
                      ey_mm=stats(np.linalg.norm(e_y_f[m, :3], axis=1) * 1e3),
                      e_comp=np.sqrt(np.mean(e_imp[m, :3] ** 2, axis=0)),
                      e_mm=stats(np.linalg.norm(delta[m], axis=1) * 1e3))
    im = B.imp_metrics(dict(imp=imp))
    for k, (a_, b_) in wins.items():
        m = (t >= a_) & (t <= b_)
        if k in im and m.sum() >= 5:
            im[k].update(F_hat_along_push_rms_n=float(np.sqrt(np.mean(F_hat_along[m] ** 2))),
                         F_hat_along_push_mean_n=float(np.mean(F_hat_along[m])),
                         F_box_model_along_push_rms_n=float(np.sqrt(np.mean(F_model[m] ** 2))),
                         F_box_model_along_push_mean_n=float(np.mean(F_model[m])))

    # ---- events, task, meta
    def edges_struct(k):
        e = PC.edges(d, t0, k)
        return dict(t=np.array([x[0] for x in e]), label=np.array([x[1] for x in e], dtype=object))

    left = [msg for tt, nm, msg in log if "PX4 LEFT OFFBOARD" in msg and abs(tt - direct_b) < 0.5]
    nav = NAV_STATE.search(left[0]) if left else None
    land_done = any(v.startswith("DONE land") for tt, v in st if tt > mk["land:start"])
    direct_exit = ("the land leg completed" if land_done else
                   f"the land leg was cut short {direct_b - mk['land:start']:.2f} s in: " +
                   (f"PX4 left OFFBOARD for nav_state {nav.group(1)} ({NAV_NAME.get(int(nav.group(1)), 'see PX4 VehicleStatus')})"
                    if nav else "the controller left DIRECT"))
    plan_log = next((msg for tt, nm, msg in log if "slides" in msg and "in" in msg and "push-and-pull: push:" in msg), "")
    m_dist = re.search(r"slides ([0-9.]+) m in ([0-9.]+) s \(mean ([0-9.]+) mm/s\)", plan_log)
    dist, push_T, speed = (float(m_dist.group(1)), float(m_dist.group(2)), float(m_dist.group(3)) / 1e3) if m_dist \
        else (float(info[64]), float(info[72]), float(info[69]))
    goals = I[ok[-1], 19:61].reshape(6, 7)
    co, so = math.cos(yaw_t), math.sin(yaw_t)
    xr, yr = info[82:84], info[84:86]
    corners = np.array([[info[79] + co * x - so * y, info[80] + so * x + co * y]
                        for x, y in ((xr[0], yr[0]), (xr[1], yr[0]), (xr[1], yr[1]), (xr[0], yr[1]))])
    pose = [float(x) for x in re.search(r"^\s*push_pull_push_pose_deg:\s*\[([^\]]*)\]", wb_yaml, re.M).group(1).split(",")]
    ee_off = [float(x) for x in re.search(r"^\s*push_pull_ee_offset:\s*\[([^\]]*)\]", wb_yaml, re.M).group(1).split(",")]
    gap = lambda tt: float(np.diff(tt[(tt >= t_start) & (tt <= t_end)]).max())    # noqa: E731
    js_pos = d["js__position"]
    finite = np.all(np.isfinite(js_pos[:, :4]), axis=1)
    cur = d["js__effort"][:, :4][:, PC.JS_IDX]
    phase_tab = dict(key=np.array(PHASE_KEYS, dtype=object), label=np.array([B.PHASE_LABEL[k] for k in PHASE_KEYS], dtype=object),
                     start_s=np.array([phases[k][0] - t_start for k in PHASE_KEYS]),
                     end_s=np.array([phases[k][1] - t_start for k in PHASE_KEYS]))
    ab_rel = (t_abort - t_start) if t_abort is not None else float("nan")
    meta = dict(
        name=name, tag=r["tag"], method=r["method"], method_long="Whole-body L1 impedance (proposed), 4-D attribution",
        run_index=r["tag"], flown=r["flown"], bag=r["bag"], outcome=r["what"],
        task=f"box pushing: grasp the fin handle from above, push the box {dist:.2f} m along the table at "
             f"{1e3 * speed:.0f} mm/s, release, climb out, fly round the table, land",
        scene="lab: the real box (240 x 160 x 100 mm, clamp bracket, 5 mm fin; OptiTrack rigid body obj_0) on the "
              "console table; the table placed from the box (the box at its left end, back face 0.10 m in)",
        plant="T650 aerial manipulator (hardware), total mass 3.746 kg in the controller model",
        feedback="raw OptiTrack position + velocity (the raw-mocap guard on), PX4 attitude; joint encoders; "
                 "box = OptiTrack rigid body obj_0",
        task_values=dict(box_mass_kg=BOX_MASS_KG, friction_static=BOX_MU, friction_kinetic=BOX_MU,
                         push_distance_m=dist, push_time_s=push_T, push_mean_speed_mps=speed, push_settle_s=1.5,
                         push_pose_deg=np.array(pose), ee_offset_m=np.array(ee_off),
                         table_x_m=np.array([corners[:, 0].min(), corners[:, 0].max()]),
                         table_y_m=np.array([corners[:, 1].min(), corners[:, 1].max()]),
                         table_corners_xy_m=corners, table_yaw_deg=float(info[81]), table_top_z_m=float(info[86]),
                         box_captured_xyz_yawdeg=np.array(info[15:19]),
                         land_start_xyz_yawdeg=np.r_[goals[4, 0:3], goals[4, 3]],
                         land_xyz_yawdeg=np.r_[goals[5, 0:3], goals[5, 3]],
                         note="box mass from the user's scale; friction from the 2026-10-17 bench test; the table is "
                              "the planner's keep-out placed from the box (table_x_m / table_y_m are its world "
                              "bounding box, table_corners_xy_m the rectangle)"),
        phases=phase_tab,
        timetable_slots_s=dict(note="no timetable: the arm ground station ran the steps (automatic Ready / Push / Exit)"),
        late_steps=np.array([], dtype=object),
        window=f"the whole mission, t = 0 (start of go to start) to {t_end - t_start:.2f} s (end of the land leg / DIRECT)",
        impedance_windows=dict(CONTACT="ready end -> exit start", HOLD="ready end -> slide start (the grip and the 1.5 s settle)",
                               SLIDE="slide start -> the push's end (BP-2: the abort)", RELEASE="push end -> exit start"),
        impedance_gains=dict(M_d=gains["my"], D_d=gains["dy"], K_d=gains["ky"], channels="x y z [N] heading [N.m]",
                             lowpass_hz=FC_HZ, frame="world axes (M_d / D_d / K_d are world-axis diagonals; y = the push)"),
        force_source="F_ext = the controller's own contact-force estimate F_hat (4-D L1 attribution, wb_control_debug "
                     "[97..100], on the claw, world axes); no force sensor on the hardware",
        L_arm_m=B.L_ARM, base_com_model_m=B.BASE_COM, aborted=t_abort is not None, abort_s=ab_rel,
        abort_reason=abort_text, t0_s=t_start, t0_driver_s=t_start, t_shift_s=0.0,
        clock="raw streams: s since the bag's first message; tracking.t / impedance.t: s since the start of go to "
              "start (= raw t - meta.t0_s). meta.t0_driver_s equals meta.t0_s, so scripts written for the "
              "simulation files run unchanged",
        direct_enter_s=direct_a, direct_exit_s=direct_b, direct_exit=direct_exit,
        max_gap_s=dict(odom=gap(to), joints=gap(tj), reference=gap(tr), box=gap(tb)),
        box_flips_removed=int(flip.sum()), box_outliers_removed=int(out_.sum()),
        leg_goals=dict(leg=np.array(B.STEPS if hasattr(B, "STEPS") else
                                    ["go_to_start", "ready", "push", "exit", "go_to_land_start", "land"], dtype=object),
                       base_m=goals[:, 0:3], yaw_deg=goals[:, 3], claw_m=goals[:, 4:7]),
        configs=np.array([c_[0] for c_ in CONFIGS], dtype=object),
        config_source=np.array([f"{os.path.basename(c_[1])} {c_[2]}:{c_[3]}" for c_ in CONFIGS], dtype=object))
    A = d["pl_arr__data"]
    raw = dict(
        odom=dict(t=to, pos_m=Pod, vel_mps=Vod, quat_wxyz=Qod, angvel_rad_s=Wod),
        joints=dict(t=tj, q_rad=qj, qdot_rad_s=qdj, current_counts=cur[finite], effort=(cur / PC.KT * PC.HW_SIGN)[finite]),
        reference=dict(t=tr, r_ed=ref["r_ed"], r_ed_dot=ref["r_ed_dot"], x_cd=ref["x_cd"], x_cd_dot=ref["x_cd_dot"],
                       x_cd_ddot=ref["x_cd_ddot"], q_d_rad=ref["q_d"], qdot_d_rad_s=ref["qdot_d"],
                       b1_d_model=ref["b1_d"], b1_de=ref["b1_de"]),
        reference_derivatives=dict(t=tr, r_ed_dot=ref["r_ed_dot"], r_ed_ddot=rdd_ref),
        box=dict(t=tb, pos_m=Pb, quat_wxyz=Qb),
        mocap_vrpn=dict(t=d["vrpn__recv"] - t0, pos_m=np.column_stack([d[f"vrpn__pose.position.{a}"] for a in "xyz"]),
                        quat_xyzw=np.column_stack([d[f"vrpn__pose.orientation.{a}"] for a in "xyzw"])),
        gripper=dict(t=d["js__recv"] - t0, pos_m=js_pos[:, 4:6]),
        controller_debug=dict(t=d["wb__recv"] - t0, data=d["wb__data"].astype(float)),
        push_monitor=dict(t=d["pl_arr__recv"] - t0, data=A.astype(float),
                          note="the planner's push_pull/arrival_error: [10] the force guard's reading [N], [11] the box "
                               "lag [m], [12] the box tilt [deg], [21..29] the push monitor (whole_body_trajectory_planner)"),
        motors=dict(t=d["motors__recv"] - t0, control=d["motors__control"].astype(float)),
        battery=dict(t=d["batt__recv"] - t0, voltage_v=d["batt__voltage_v"].astype(float),
                     current_a=d["batt__current_a"].astype(float)),
        marks=dict(t=np.array(list(mk.values())), name=np.array(list(mk.keys()), dtype=object)),
        events=dict(mode=edges_struct("wbmode"), planner_status=edges_struct("pl_status"),
                    push_pull_status=edges_struct("pl_pstatus"),
                    log=dict(t=np.array([x[0] for x in log]), node=np.array([x[1] for x in log], dtype=object),
                             msg=np.array([x[2] for x in log], dtype=object))))
    tracking = dict(
        t=tu - t_start, in_window=np.ones(n), phase_id=phase_id,
        platform_pos_m=P, platform_pos_ref_m=xb, platform_pos_err_m=e_pos, eps_uav_m=eps, rho_uav=eps / B.L_ARM,
        platform_quat_xyzw=Qu, platform_att_err_deg=att_err, platform_heading_deg=yaw, platform_heading_ref_deg=yaw_ref,
        platform_tilt_deg=tilt, com_pos_m=xc, com_pos_ref_m=x_cd,
        ee_pos_m=re_, ee_pos_ref_m=r_ed, ee_pos_err_m=e_ee,
        ee_heading_deg=az, ee_heading_ref_deg=az_ref, ee_heading_err_deg=ee_head_err,
        q_deg=np.degrees(q), q_ref_deg=np.degrees(q_d), q_err_deg=e_q, box_pos_m=box_u,
        recording_gap=rec_gap.astype(float),
        columns="platform_att_err_deg = [roll pitch heading] (body axes); xyz columns are world ENU; "
                "box_pos_m is NaN where no box sample lies within 0.1 s; recording_gap = 1 where the odometry has a "
                "gap > 0.1 s in the bag (linearly interpolated across)")
    impedance = dict(t=t - t_start, e_imp=e_imp, e_imp_mm=1e3 * delta, F_ext=Fx_f, rendered=rendered, e_y=e_y_f,
                     F_hat_along_push_n=F_hat_along, F_box_model_along_push_n=F_model,
                     columns="e_imp / F_ext / rendered: [x y z heading] in [N N N N.m]; F_ext = the controller's "
                             "ESTIMATE F_hat (no force sensor); e_imp_mm: the residual's displacement equivalent "
                             "(K_d^-1 e_imp), [x y z] mm; e_y: [m m m sin(heading error)]; *_along_push_n: the force on "
                             "the claw along the push, F_hat's and the box model's -(m a + mu m g tanh(v / 5 mm/s)); "
                             "every term low-passed (zero phase) at impedance_gains.lowpass_hz; t covers the CONTACT window",
                     windows=dict(name=np.array(list(im.keys()), dtype=object),
                                  start_s=np.array([wins[w][0] - t_start for w in im]),
                                  end_s=np.array([wins[w][1] - t_start for w in im])))
    rel = dict(phases=phases, tu=tu, q=q)
    margin = B.release_margin(rel)
    import scipy.io
    scipy.io.savemat(os.path.join(MAT_DIR, name + ".mat"),
                     {"run": dict(meta=meta, metrics=dict(window=window, whole_recording=window, per_phase=per_phase,
                                                          impedance=im, box=boxm, release=margin),
                                  tracking=tracking, impedance=impedance, raw=raw)},
                     do_compression=True, long_field_names=True, oned_as="column")
    print(f"wrote {os.path.relpath(os.path.join(MAT_DIR, name + '.mat'), RESULTS)}  window {window['window_s']:.2f} s  "
          f"eps max/rms {window['eps_uav_max_mm']:.1f}/{window['eps_uav_rms_mm']:.1f} mm  EE {window['ee_pos_mm']:.1f} mm  "
          f"box {boxm['pushed_mm']:.0f} mm  e_imp slide {im['SLIDE']['e_imp_rms_n']:.2f} N (F_hat)", flush=True)
    gaps = [(float(to[k] - t_start), float(to[k + 1] - to[k])) for k in np.where(np.diff(to) > 0.3)[0]
            if t_start <= to[k] <= t_end]
    return dict(name=name, r=r, window=window, per_phase=per_phase, im=im, boxm=boxm, margin=margin, phases=phase_tab,
                gaps=gaps,
                t_end=t_end - t_start, meta=meta, gains=gains, z_end=float(P[-1, 2]), marks=mk, t_start=t_start)


def stage2():
    import numpy as np
    import scipy.io
    sys.path.insert(0, HERE)
    import build_box_pushing as B
    os.makedirs(MAT_DIR, exist_ok=True)
    cfg = copy_configs()
    out = [convert(name, r, cfg) for name, r in RUNS.items()]
    keys = ["eps_uav_max_mm", "eps_uav_rms_mm", "rho_uav_max", "rho_uav_rms", "platform_pos_mm", "platform_heading_deg",
            "platform_roll_deg", "platform_pitch_deg", "ee_pos_mm", "ee_heading_deg", "tilt_max_deg"]
    flat, ph_rows, imp_rows = [], [], []
    for o in out:
        w, im = o["window"], o["im"]
        f = dict(name=o["name"], method=o["r"]["method"], run=o["r"]["tag"], total_window_s=w["window_s"],
                 **{k: w[k] for k in keys})
        for j, ax in enumerate("xyz"):
            f[f"platform_{ax}_mm"] = w["platform_pos_xyz_mm"][j]; f[f"ee_{ax}_mm"] = w["ee_pos_xyz_mm"][j]
            f[f"e_imp_slide_{ax}_rms_n"] = im["SLIDE"]["e_imp_xyz_rms_n"][j]
        for j in range(4):
            f[f"q{j + 1}_deg"] = w["joints_deg"][j]
        f.update(e_imp_slide_rms_n=im["SLIDE"]["e_imp_rms_n"], e_imp_slide_max_n=im["SLIDE"]["e_imp_max_n"],
                 e_imp_contact_rms_n=im["CONTACT"]["e_imp_rms_n"], e_imp_contact_max_n=im["CONTACT"]["e_imp_max_n"],
                 e_imp_hold_rms_n=im["HOLD"]["e_imp_rms_n"],
                 e_imp_release_rms_n=im["RELEASE"]["e_imp_rms_n"] if "RELEASE" in im else float("nan"),
                 e_imp_slide_psi_rms_nm=im["SLIDE"]["e_imp_psi_rms_nm"], e_imp_slide_mm_rms=im["SLIDE"]["e_imp_mm_rms"],
                 e_imp_contact_mm_rms=im["CONTACT"]["e_imp_mm_rms"], F_ext_slide_rms_n=im["SLIDE"]["F_ext_rms_n"],
                 e_y_slide_rms_mm=im["SLIDE"]["e_y_rms_mm"],
                 F_hat_slide_along_push_mean_n=im["SLIDE"]["F_hat_along_push_mean_n"],
                 F_box_model_slide_along_push_mean_n=im["SLIDE"]["F_box_model_along_push_mean_n"],
                 box_pushed_mm=o["boxm"]["pushed_mm"], box_yaw_change_deg=o["boxm"]["yaw_change_deg"],
                 box_exit_move_mm=o["boxm"]["exit_move_mm"], box_tilt_max_deg=o["boxm"]["tilt_max_deg"],
                 release_q2_max_deg=o["margin"]["release_q2_max_deg"], release_q3_min_deg=o["margin"]["release_q3_min_deg"],
                 aborted=float(o["meta"]["aborted"]), abort_s=o["meta"]["abort_s"])
        flat.append(f)
        for key in B.PHASE_KEYS:
            p = o["per_phase"][key]
            ph_rows.append(dict(name=o["name"], method=o["r"]["method"], run=o["r"]["tag"], phase=key, label=p["label"],
                                start_s=p["start_s"], end_s=p["end_s"], eps_uav_max_mm=p["eps_uav_max_mm"],
                                eps_uav_rms_mm=p["eps_uav_rms_mm"], rho_uav_max=p["rho_uav_max"],
                                rho_uav_rms=p["rho_uav_rms"], ee_pos_mm=p["ee_pos_mm"],
                                platform_pos_mm=p["platform_pos_mm"], tilt_max_deg=p["tilt_max_deg"]))
        for wn, v in o["im"].items():
            imp_rows.append(dict(name=o["name"], method=o["r"]["method"], run=o["r"]["tag"], window=wn,
                                 **{k: x for k, x in v.items() if not isinstance(x, list)},
                                 e_imp_x_rms_n=v["e_imp_xyz_rms_n"][0], e_imp_y_rms_n=v["e_imp_xyz_rms_n"][1],
                                 e_imp_z_rms_n=v["e_imp_xyz_rms_n"][2]))
    num = [k for k in flat[0] if k not in ("name", "method", "run")]
    mean = [dict(method="whole_body_l1", runs=float(len(flat)), **{k: float(np.mean([f[k] for f in flat])) for k in num})]
    std = [dict(method="whole_body_l1", runs=float(len(flat)),
                **{k: float(np.std([f[k] for f in flat], ddof=1)) if len(flat) > 1 else 0.0 for k in num})]
    imp_keys = sorted(set().union(*[r_.keys() for r_ in imp_rows]))
    imp_rows = [{k: r_.get(k, float("nan")) for k in imp_keys} for r_ in imp_rows]
    phases_runs = {o["r"]["tag"].replace("-", "_"): o["phases"] for o in out}
    scipy.io.savemat(os.path.join(MAT_DIR, "box_pushing_metrics.mat"),
                     {"metrics_runs": B.colstruct(flat, list(flat[0])), "metrics_mean": B.colstruct(mean, list(mean[0])),
                      "metrics_std": B.colstruct(std, list(std[0])),
                      "metrics_phase": B.colstruct(ph_rows, list(ph_rows[0])),
                      "metrics_impedance": B.colstruct(imp_rows, imp_keys),
                      "phases": out[0]["phases"], "phases_runs": phases_runs,
                      "window_s": out[0]["t_end"], "L_arm_m": B.L_ARM},
                     do_compression=True, long_field_names=True, oned_as="column")
    print("wrote", os.path.relpath(os.path.join(MAT_DIR, "box_pushing_metrics.mat"), RESULTS), flush=True)
    write_readme(out)
    json.dump([dict(name=o["name"], window=o["window"], box=o["boxm"], margin=o["margin"],
                    impedance={k: {kk: (vv.tolist() if hasattr(vv, "tolist") else vv) for kk, vv in v.items()}
                               for k, v in o["im"].items()}) for o in out],
              open(os.path.join(CACHE, "box_pushing_scores.json"), "w"), indent=1, default=float)


def write_readme(out):
    sys.path.insert(0, HERE)
    from readme_text import _md_table
    f1 = lambda x: f"{x:.1f}"    # noqa: E731
    o0 = out[0]
    tv = o0["meta"]["task_values"]
    g = o0["gains"]
    names = [o["name"] for o in out]
    run_rows = [[o["r"]["tag"], o["r"]["flown"], f"`{o['r']['bag']}`", o["r"]["what"], f"`{o['name']}.mat`"] for o in out]
    ph_rows = []
    for i, key in enumerate(o0["phases"]["key"]):
        cells = [str(i + 1), o0["phases"]["label"][i]]
        for o in out:
            cells.append(f"{o['phases']['start_s'][i]:.2f} - {o['phases']['end_s'][i]:.2f}")
        cells.append(PHASES_TEXT[key])
        ph_rows.append(cells)
    res = []
    for o in out:
        w, im, b = o["window"], o["im"], o["boxm"]
        res.append([o["r"]["tag"], f1(w["eps_uav_max_mm"]), f"{w['rho_uav_max']:.3f}", f1(w["eps_uav_rms_mm"]),
                    f"{w['rho_uav_rms']:.3f}", f1(w["ee_pos_mm"]), f"{w['ee_heading_deg']:.2f}", f1(w["tilt_max_deg"]),
                    f"{b['pushed_mm']:.0f}", f"{b['tilt_max_deg']:.2f}"])
    per = []
    for o in out:
        for key in o0["phases"]["key"]:
            p = o["per_phase"][key]
            per.append([o["r"]["tag"], p["label"], f1(p["eps_uav_max_mm"]), f1(p["eps_uav_rms_mm"]), f1(p["ee_pos_mm"]),
                        f"{p['ee_heading_deg']:.2f}", f1(p["tilt_max_deg"])])
    imp = []
    for o in out:
        for wn, v in o["im"].items():
            imp.append([o["r"]["tag"], wn, f"{v['T_s']:.1f}", f"{v['e_imp_rms_n']:.2f}", f"{v['F_ext_rms_n']:.2f}",
                        f"{v['F_hat_along_push_mean_n']:+.2f}", f"{v['F_box_model_along_push_mean_n']:+.2f}",
                        f"{v['e_y_rms_mm']:.1f}", f"{v['e_imp_mm_rms']:.1f}"])
    box_rows = [[o["r"]["tag"], f"{o['boxm']['pushed_mm']:.0f}", f"{o['boxm']['yaw_change_deg']:+.1f}",
                 f"{o['boxm']['tilt_max_deg']:.2f}", f"{o['boxm']['exit_move_mm']:.1f}",
                 f"{o['margin']['release_q2_max_deg']:.1f}", f"{o['margin']['release_q3_min_deg']:.1f}",
                 f"{o['meta']['abort_s']:.2f}", o["meta"]["abort_reason"].split(" -- ")[0].replace("ABORTING ", "")]
                for o in out]
    L = [
        "# Experiment results, MATLAB data (box pushing)", "",
        "The two hardware box-pushing flights of 2026-10-27, converted from their ros2 bags. This folder is",
        "self-contained: these files plus MATLAB are all you need, with no ROS or custom message definitions.",
        "The `run` struct has the layout of the simulation's",
        "`matlab_simulation_data/box_pushing/<method>_push_run<k>.mat`, so a script written for those files runs on",
        "these. **These flights show feasibility; the accuracy comparison is the simulation's.**", "",
        "## Files", "",
        "```",
        *[f"{nm}.mat          one run, struct `run`" for nm in names],
        "box_pushing_metrics.mat                metrics_runs = every run, metrics_mean / metrics_std over the two,",
        "                                       metrics_phase = every run x phase, metrics_impedance = every run x",
        "                                       contact window, phases = BP-1's phase table, phases_runs = both,",
        "                                       window_s = BP-1's window, L_arm_m = L",
        "configs/whole_body_l1_4d_hw_box_pushing.yaml                  the controller + planner, as flown",
        "configs/arm_external_torque_controller_hardware_aerial_pwm.yaml   the arm controller, as flown",
        "```", "",
        "- **Name:** `<method>_push_<run>`, the simulation's naming with the flight tag as the run.",
        "- **Method:** `whole_body_l1` = Proposed. Only the proposed controller flew this task on hardware.",
        "- **Configs:** taken from git at the commits the vehicle ran (first line of each file). The bag shows",
        "  their features: the push pose [0, 28, 32, 0] deg in the planner's hold captures, the abort climbing",
        "  straight up.", "",
        "```matlab",
        "S  = load('box_pushing_metrics.mat');",
        "R  = struct2table(S.metrics_runs)                % the two runs",
        f"r  = load('{names[0]}.mat').run;",
        "P  = r.meta.phases;  tr = r.tracking;           % this run's eight phases; 100 Hz, t = 0 at go to start",
        "figure; hold on",
        "for i = 1:numel(P.start_s)                       % phase bands, alternating shades",
        "    c = 0.90 + 0.07*mod(i+1, 2);",
        "    patch([P.start_s(i) P.end_s(i) P.end_s(i) P.start_s(i)], [0 0 200 200], c*[1 1 1], 'EdgeColor', 'none');",
        "    text(mean([P.start_s(i) P.end_s(i)]), 195, P.label{i}, 'HorizontalAlignment', 'center');",
        "end",
        "plot(tr.t, 1e3 * tr.eps_uav_m)                   % ||eps_UAV|| [mm]",
        "xlim([0 tr.t(end)]); xlabel('Time (s)'); ylabel('||\\epsilon_{UAV}|| (mm)')",
        "",
        "% the box along the push against the plan (the simulation's README plots the same from raw.box)",
        "tv = r.meta.task_values;  u = [cosd(tv.table_yaw_deg) sind(tv.table_yaw_deg)];",
        "w  = ~isnan(tr.box_pos_m(:,1));  s = tr.box_pos_m(:,1:2) * u';  s0 = s(find(w & tr.phase_id == 4, 1));",
        "figure; plot(tr.t(w), 1e3*(s(w) - s0)); xlabel('Time (s)'); ylabel('box travel along the push (mm)')",
        "",
        "% the force the controller estimated on the claw, and the box model's, along the push",
        "im = r.impedance;  figure; plot(im.t, im.F_hat_along_push_n, im.t, im.F_box_model_along_push_n)",
        "legend('F\\_hat (controller estimate)', 'box model m a + \\mu m g'); ylabel('N'); xlabel('Time (s)')",
        "```", "",
        "Raw streams are on the bag's clock: subtract `r.meta.t0_s` to put them on `tracking.t`",
        "(`r.meta.t0_driver_s` is the same number, so the simulation's snippets work unchanged).", "",
        "## The flights", "",
        _md_table(["run", "flown", "original ros2 bag", "what happened", "file"], run_rows), "",
        "The bags are kept unchanged in `results/experiment_results/flight_test_data/box_pushing/<name>/` on the lab",
        f"desktop, and in `docs/experimental_data_ros2_bag/1027 - T650-AM whole-body Box-Pushing & Tracking [Success]-*/`.",
        f"The folder name says 300 g and the data file 500 g; the box weighs {1e3 * BOX_MASS_KG:.0f} g. Flight analysis:",
        f"`{ANALYSIS}`.", "",
        "## How this differs from the simulation's box pushing", "",
        "- **Both runs ended in a planner safety abort** (the q3 posture guard), which replaced the normal exit:",
        "  `run.meta.aborted`, `abort_s`, `abort_reason`. The phases keep the simulation's eight names; in these",
        "  runs `exit` is the abort's straight climb and arm fold, and `release` is the station's release (BP-1)",
        "  or the abort's 1.5 s release hold (BP-2).",
        "- **F_ext is an estimate.** The hardware has no force sensor, so the impedance residual uses the",
        "  controller's own contact-force estimate F_hat (`run.meta.force_source`); the simulation uses the PhysX",
        "  contact force. `impedance.F_box_model_along_push_n` is an independent physical check (box mass x its",
        f"  measured acceleration + mu m g, mu = {BOX_MU}). It reads ~0 while the box is stuck (static friction is",
        "  not observable from the box's motion), so its mean is below mu m g when the box stick-slips.",
        "- **Lighter box, shorter push, newer arm settings:** "
        f"{1e3 * BOX_MASS_KG:.0f} g, {tv['push_distance_m']:.2f} m in {tv['push_time_s']:.1f} s, push pose "
        f"[{', '.join(f'{x:g}' for x in tv['push_pose_deg'])}] deg, push axis at natural inertia "
        f"(M_d / D_d / K_d along y = {g['my'][1]:g} / {g['dy'][1]:g} / {g['ky'][1]:g}). The simulation flew 500 g,",
        "  0.50 m in 24 s, [0, 32, 28, 0] deg, M_d / D_d / K_d = 1 / 20.2 / 120 on every axis.",
        "- **The operator's station paced the mission** (its automatic Ready / Push / Exit); the phases come from",
        "  the planner's status and the station's log, so the two runs' phase times differ (`phases_runs`).",
        "- **The box is measured by OptiTrack** (rigid body `obj_0`), not simulator ground truth; mis-solved",
        f"  samples are removed (tilt > {FLIP_TILT_DEG:.0f} deg; jumps off the rolling median): "
        + ", ".join(f"{o['r']['tag']} {o['meta']['box_flips_removed']} + {o['meta']['box_outliers_removed']}" for o in out)
        + ".",
        "- **The measured claw** is the model's forward kinematics of the measured platform pose and joints (the",
        "  simulation's `raw.claw_truth`, `raw.base_truth` and `raw.contact_truth` have no hardware counterpart).",
        "- **Recording gaps:** the recorder ran on the ground-station laptop, so the vehicle computer's topics reach",
        "  it over WiFi; "
        + "; ".join(f"{o['r']['tag']}: " + (", ".join(f"{g_:.2f} s ({dg:.2f} s long)" for g_, dg in o["gaps"])
                                            if o["gaps"] else "none > 0.3 s") for o in out)
        + " (s since go to start). The vehicle kept flying in DIRECT; `tracking` interpolates across and marks",
        "  the samples in `tracking.recording_gap`.",
        "- **The window ends with DIRECT:** " + "; ".join(f"{o['r']['tag']}: {o['meta']['direct_exit']}" for o in out) + ".",
        "- **Joint torque** `raw.joints.effort` [N.m, model convention] is the servos' Present Current through the",
        "  calibrated torque constants (`raw.joints.current_counts`).", "",
        "## Phases (s since the start of go to start)", "",
        _md_table(["#", "phase"] + [f"{o['r']['tag']} start - end [s]" for o in out] + ["what happens"], ph_rows), "",
        "`tracking.phase_id` (1..8) says which phase each sample is in; `tracking.in_window` is 1 everywhere.", "",
        "## Results", "",
        _md_table(["run", "eps max [mm]", "rho max", "eps rms [mm]", "rho rms", "EE RMSE [mm]", "EE heading RMSE [deg]",
                   "tilt max [deg]", "box pushed [mm]", "box tilt max [deg]"], res), "",
        "Per phase (`metrics_phase`):", "",
        _md_table(["run", "phase", "eps max [mm]", "eps rms [mm]", "EE RMSE [mm]", "EE heading RMSE [deg]", "tilt max [deg]"],
                  per), "",
        "The impedance residual per contact window (`metrics_impedance`; F_ext = the controller's estimate):", "",
        _md_table(["run", "window", "T [s]", "e_imp rms [N]", "F_ext rms [N]", "F_hat along push, mean [N]",
                   "box model along push, mean [N]", "e_y rms [mm]", "e_imp as displacement rms [mm]"], imp), "",
        "Along the push, a NEGATIVE force on the claw is the box resisting the push. The box model needs mu m g = "
        f"{BOX_MU * BOX_MASS_KG * 9.81:.2f} N to slide.", "",
        "The box, the release window and the abort (`metrics.box`, `metrics.release`, `meta`):", "",
        _md_table(["run", "box pushed [mm]", "box yaw change [deg]", "box tilt max [deg]", "box moved in the exit [mm]",
                   "q2 max, release + exit [deg]", "q3 min, release + exit [deg]", "abort at [s]", "abort reason"], box_rows), "",
        "## Conditions", "",
        "- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm in torque mode, 3.746 kg in the controller model.",
        "- **Feedback:** raw OptiTrack position and velocity with the raw-mocap guard, PX4 attitude; joint encoders.",
        "- **Controller:** whole-body L1, 4-D attribution, `AM_HW_PROFILE=box_pushing`",
        "  (`configs/whole_body_l1_4d_hw_box_pushing.yaml`): M_d = diag(" + ", ".join(f"{x:g}" for x in g["my"]) +
        "), D_d = diag(" + ", ".join(f"{x:g}" for x in g["dy"]) + "), K_d = diag(" + ", ".join(f"{x:g}" for x in g["ky"]) +
        ") on [x, y, z, heading], world axes.",
        f"- **Task:** push {tv['push_distance_m']:.2f} m at {1e3 * tv['push_mean_speed_mps']:.0f} mm/s "
        f"({tv['push_time_s']:.2f} s after a 1.5 s settle), push pose "
        f"[{', '.join(f'{x:g}' for x in tv['push_pose_deg'])}] deg, EE offset "
        f"({', '.join(f'{x:g}' for x in tv['ee_offset_m'])}) m above the box's origin.",
        f"- **Table** (the planner's keep-out, placed from the box): top at {tv['table_top_z_m']:.2f} m, yaw "
        f"{tv['table_yaw_deg']:.1f} deg (`meta.task_values.table_corners_xy_m`).",
        "- **Safety guards:** box tilt > 2 deg, box lag > 40 mm, force reading > 5 N, q3 < 10 deg (static and",
        "  rate-aware, q2-wall aware), vehicle tilt > 10 deg (planner) / 15 deg (node), the raw-mocap guard.", "",
        "## The struct `run`", "",
        "| field | content |", "|---|---|",
        "| `meta` | name, run tag, method, flight date, original bag, outcome, task values, phase table, window, clock, "
        "DIRECT entry/exit, abort time and reason, the impedance windows and gains, the force source, largest gap per "
        "stream, box samples removed, leg goals, configs |",
        "| `metrics` | `window` (whole mission; `whole_recording` = the same), `per_phase.<phase>`, `impedance.<window>`, "
        "`box`, `release` (the release window's joint margins) |",
        "| `tracking` | 100 Hz over the mission, `t` in s since the start of go to start: platform position and attitude "
        "(measured, `_ref_`, `_err_`), `eps_uav_m` / `rho_uav`, the system CoM, the end-effector position and heading, the "
        "joints (`q_deg`), and `box_pos_m` |",
        "| `impedance` | over the CONTACT window: `e_imp`, `e_imp_mm`, `F_ext` (the estimate), `rendered`, `e_y`, the two "
        "along-push forces, `windows` |",
        "| `raw` | the bag's streams, time in s since the bag's first message: `odom`, `joints` (model convention), "
        "`reference` + `reference_derivatives` (the planner's whole-body reference), `box` (obj_0), `mocap_vrpn` "
        "(vehicle), `gripper` (finger positions), `controller_debug` (`wb_control_debug`), `push_monitor` (the planner's "
        "guard readings), `motors` (normalised), `battery`, `marks` (the phase marks), `events.mode` / "
        "`events.planner_status` / `events.push_pull_status` (transitions), `events.log` (every node's /rosout line) |", "",
        "Units: positions in m (world ENU, z up) unless the name says mm; angles in deg unless it says rad. Errors are",
        "measured − reference. `platform_att_err_deg` columns are `[roll pitch heading]`.", "",
        "**Definitions** (the simulation's, `results/utils/build_box_pushing.py`):", "",
        "- **Reference:** the planner's dynamically compatible whole-body reference. Platform",
        "  x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>); R<sub>0,d</sub> from the thrust",
        "  direction (CoM reference acceleration + g e<sub>3</sub>) and the reference heading; EE r<sub>e,d</sub>, heading",
        "  b<sub>1,e,d</sub>; joints q<sub>d</sub>.",
        "- **ε<sub>UAV</sub>** = ‖x<sub>b</sub> − r<sub>UAV</sub>‖; ρ<sub>UAV</sub> = ε<sub>UAV</sub> / L, L = "
        f"{o0['meta']['L_arm_m']:.3f} m.",
        "- **Impedance residual:** e<sub>imp</sub> = F<sub>ext</sub> − (M<sub>d</sub> ë + D<sub>d</sub> ė + K<sub>d</sub> e)",
        "  on the claw's world error chains, every term low-passed at 5 Hz (zero phase).", "",
        "Regenerate (from the bags in `results/experiment_results/flight_test_data/box_pushing/`):", "",
        "```bash", "/usr/bin/python3 results/utils/experiment_box_pushing.py", "```", ""]
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
