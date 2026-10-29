#!/usr/bin/env python3
"""Export chosen runs of the heading-channel sweep (2026-10-26) for MATLAB, in the valve-turning
`run` struct layout (results/simulation_results/matlab_simulation_data/valve_turning/README.md):

    PYTHONNOUSERSITE=1 /usr/bin/python3 export_matlab.py yw_k3_f010 [...] [--out ../matlab]

results/utils/build_valve_turning.py's load_run() and struct layout, with two changes for this sweep:
  * the runs END after Exit (pl_mission.py --stop-after-exit): the phase table stops at "exit"
    (go_to_start, ready, grasp, push = the 90 deg turn, release, exit);
  * meta.sweep carries the sweep's knobs (K_psi, D_psi, M_psi, tau_f, the friction model) and the
    scorer's row (tools/yaw_score.py: every check, pass / fail); `heading` adds the sweep's own series
    over the contact window: the couple M (resisting the turn +), the rendered moment, F_t r, the
    commanded body yaw torque, q1 / q4, the tilt.
impedance.F_ext(:,4), impedance.rendered(:,4), impedance.e_imp(:,4) and impedance.e_y(:,4) are the
heading channel, exactly as the builder writes them (N.m; e_y(:,4) = sin of the heading error).
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SWEEP = os.path.abspath(os.path.join(HERE, ".."))
REPO = os.path.abspath(os.path.join(SWEEP, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "results", "utils"))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
sys.path.insert(0, HERE)
import build_valve_turning as BV   # noqa: E402
import yaw_score as YS             # noqa: E402

# the runs end after Exit
PHASES = [p for p in BV.PHASES if p[0] not in ("go_to_land_start", "land")]
PHASES = [p if p[0] != "exit" else ("exit", "Exit", "exit:start", "exit:end") for p in PHASES]
BV.PHASES = PHASES
BV.PHASE_KEYS = [p[0] for p in PHASES]
BV.LAST = "exit"
PHASE_KEYS, PHASE_LABEL = BV.PHASE_KEYS, BV.PHASE_LABEL


def export(tag, out, scores):
    import scipy.io
    path = os.path.join(YS.RUNS, tag + ".npz")
    R = BV.load_run(path)
    tu = R["tu"]
    T_end = R["phases"]["exit"][1] - R["t0"]
    fixed = (tu - R["t0"] >= -1e-9) & (tu - R["t0"] <= T_end + 1e-9)
    phase_id = np.zeros(len(tu))
    per_leg = {}
    for i, key in enumerate(PHASE_KEYS):
        s0, e0 = R["phases"][key]
        m = (tu >= s0) & (tu < e0) & fixed
        phase_id[m] = i + 1
        per_leg[key] = dict(label=PHASE_LABEL[key], start_s=s0 - R["t0"], end_s=e0 - R["t0"], **BV.safe_metrics(R, m))
    r = BV.metrics(R, fixed)
    im = BV.imp_metrics(R["imp"])
    V = R["valvem"]
    d = R["d"]
    ser = R["imp"]["series"]
    T = R["task"]
    row, S = YS.score(tag)
    srow = scores.get(tag, row)
    meta = dict(name=tag, method="whole_body_l1", method_long=BV.METHODS["whole_body_l1"][1],
                plant="Isaac Sim, sim-to-real mirror plant (config A), headless at real time (RTF 1), raw mocap feedback",
                scene=BV.scene_text(T), task=BV.task_text(T),
                task_values=dict(turn_deg=T["turn_deg"], turn_time_s=T["push_time_s"], settle_s=T["settle_s"],
                                 fin_radius_m=T["fin_radius_m"], grasp_height_above_table_m=T["grasp_height_m"],
                                 axis_xy_m=np.array(T["axis_xy"], float), beam_top_z_m=T["beam_top_z"],
                                 rotor_mass_kg=T["rotor_mass_kg"], bearing_friction_nm=T["friction_nm"],
                                 bearing_damping_nms=T["damping_nms"], contact_phase_flag=T["contact_phase"],
                                 turn_pose_deg=np.array(T["pose_deg"], float),
                                 table_x_m=np.array([T["table_xmin"], T["table_xmax"]]),
                                 table_y_m=np.array([T["table_ymin"], T["table_ymax"]]), table_top_z_m=T["table_top_z"],
                                 overrides=np.array(T["overrides"], dtype=object)),
                sweep=dict(K_psi=srow["K_psi"], D_psi=srow["D_psi"], M_psi=srow["M_psi"], tau_f_nm=srow["tau_f"],
                           friction_model=srow["friction_model"],
                           friction_model_note="drive = stick-slip Coulomb friction in the bearing joint's drive "
                                               "(force limit tau_f); see tools/friction_check.py",
                           checks={k: (v if not isinstance(v, (list, tuple)) else np.array(v, dtype=object))
                                   for k, v in srow.items() if k not in ("run",)}),
                phases=dict(key=np.array(PHASE_KEYS, dtype=object),
                            label=np.array([PHASE_LABEL[x] for x in PHASE_KEYS], dtype=object),
                            start_s=np.array([R["phases"][x][0] - R["t0"] for x in PHASE_KEYS]),
                            end_s=np.array([R["phases"][x][1] - R["t0"] for x in PHASE_KEYS])),
                timetable_slots_s={nm: float(x) for nm, x in zip(BV.SLOT_NAMES, R["timetable"])},
                window=f"t = 0 (start of go to start) to {T_end:.2f} s (end of Exit; the run stops there)",
                impedance_windows=dict(CONTACT="ready end -> exit start (the law in contact)",
                                       HOLD="ready end -> turn start (the grip and the 1.5 s settle)",
                                       SLIDE=f"the {T['push_time_s']:g} s turn", RELEASE="turn end -> exit start"),
                impedance_gains=dict(M_d=R["gains"]["my"], D_d=R["gains"]["dy"], K_d=R["gains"]["ky"],
                                     channels="x y z [N] heading [N.m]", lowpass_hz=BV.FC_HZ),
                contact_sign=dict(value=V["contact_sign"], mean_power_w=V["contact_power_w"],
                                  rule="the sign making the vehicle's mean power on the rotor positive over the turn"),
                L_arm_m=BV.L_ARM, base_com_model_m=BV.BASE_COM, aborted=bool(d["aborted"]), t0_driver_s=R["t0"],
                clock="driver clock [s] (raw streams); tracking.t / impedance.t / heading.t are s since the start of go to start")
    ct = d["contact"]
    raw = dict(odom=dict(t=R["od"][:, 0], pos_m=R["od"][:, 1:4], vel_mps=R["od"][:, 4:7], quat_wxyz=R["od"][:, 7:11],
                         direct_mode=R["od"][:, 11], step=R["od"][:, 12]),
               joints=dict(t=R["js"][:, 0], q_rad=R["js"][:, 1:5][:, R["order"]], effort=R["js"][:, 5:9][:, R["order"]]),
               reference=dict(t=R["ref"][:, 0], r_ed=R["ref"][:, 1:4], x_cd=R["ref"][:, 4:7], q_d_rad=R["ref"][:, 7:11],
                              b1_d_model_xy=R["ref"][:, 11:13], x_cd_ddot=R["ref"][:, 13:16], b1_de=R["ref"][:, 16:19]),
               reference_derivatives=dict(t=d["refd"][:, 0], r_ed_dot=d["refd"][:, 1:4], r_ed_ddot=d["refd"][:, 4:7]),
               rotor=dict(t=R["box"][:, 0], axis_pos_m=R["box"][:, 1:4], quat_wxyz=R["box"][:, 4:8],
                          beam_angle_deg=np.degrees(np.unwrap(np.radians(BV.yaw_of(R["box"][:, 4:8])))),
                          note="mocap obj_0 = the rotor: its origin the bearing axis on the table top (fixed), its yaw the beam's"),
               claw_truth=dict(t=d["claw"][:, 0], pos_m=d["claw"][:, 1:4]),
               base_truth=dict(t=d["base_truth"][:, 0], pos_m=d["base_truth"][:, 1:4], quat_wxyz=d["base_truth"][:, 4:8]),
               contact_truth=dict(t=ct[:, 0], vehicle_normal_n=ct[:, 1:4], vehicle_friction_n=ct[:, 4:7],
                                  vehicle_moment_normal_nm=ct[:, 13:16], vehicle_moment_friction_nm=ct[:, 16:19]),
               law_debug=dict(t=d["dbg"][:, 0], u1_n=d["dbg"][:, 17 + 2], tau_body_flu_nm=d["dbg"][:, 21 + 2:24 + 2],
                              motor_cmd=d["dbg"][:, 41 + 2:45 + 2], e_y=d["dbg"][:, 24 + 2:28 + 2],
                              F_hat_raw_n=d["dbg"][:, 97 + 2:100 + 2],
                              note="wb_control_debug in DIRECT: u1, the body torque (actual FLU), the four motor "
                                   "commands, the law's task error e_y, the task-force reading the planner's guard filters"),
               marks=dict(t=np.array(list(R["marks"].values())), name=np.array(list(R["marks"].keys()), dtype=object)),
               events=np.array([str(x) for x in d["events"]], dtype=object))
    tracking = dict(t=tu - R["t0"], in_window=fixed.astype(float), phase_id=phase_id,
                    platform_pos_m=R["P"], platform_pos_ref_m=R["xb"], platform_pos_err_m=R["e_pos"],
                    eps_uav_m=R["eps"], rho_uav=R["eps"] / BV.L_ARM,
                    platform_quat_xyzw=R["Qu"], platform_att_err_deg=R["att_err"],
                    platform_heading_deg=R["yaw"], platform_heading_ref_deg=R["yaw_ref"], platform_tilt_deg=R["tilt"],
                    com_pos_m=R["xc"], com_pos_ref_m=R["x_cd"],
                    ee_pos_m=R["re"], ee_pos_ref_m=R["r_ed"], ee_pos_err_m=R["e_ee"],
                    ee_heading_deg=R["az"], ee_heading_ref_deg=R["az_ref"], ee_heading_err_deg=R["ee_head_err"],
                    q_deg=np.degrees(R["q"]), q_ref_deg=np.degrees(R["q_d"]), q_err_deg=R["e_q"],
                    beam_angle_deg=R["beam_u"], beam_angle_plan_deg=R["az_plan"], beam_minus_plan_deg=R["beam_err"])
    impedance = dict(t=ser["t"] - R["t0"], e_imp=ser["e_imp"], F_ext=ser["F"], rendered=ser["rendered"], e_y=ser["e_y"],
                     columns="e_imp / F_ext / rendered: [x y z heading] in [N N N N.m]; e_y: [m m m sin(heading error)]; "
                             "every term low-passed (zero phase) at impedance_gains.lowpass_hz; t covers the CONTACT window")
    heading = dict(t=S["t"] - R["t0"], couple_resisting_nm=S["M"], rendered_resisting_nm=S["R"],
                   Ft_r_resisting_nm=S["Ft"] * S["r"], e_imp_psi_nm=S["e_imp"],
                   turn_window_s=np.array([S["a"], S["b"]]) - R["t0"], mid_window_s=np.array(S["mid"]) - R["t0"],
                   yaw_torque_t=S["dbg_t"] - R["t0"], yaw_torque_cmd_nm=S["tz"],
                   joints_t=S["js_t"] - R["t0"], q_deg=S["q"], tilt_t=S["bt_t"] - R["t0"], tilt_deg=S["tilt"],
                   note="resisting = the moment / force opposing the turn, positive; couple = the true contact moment "
                        "about the vertical at the claw point; rendered = M_psi e'' + D_psi e' + K_psi e on the law's e_y(4)")
    os.makedirs(out, exist_ok=True)
    scipy.io.savemat(os.path.join(out, tag + ".mat"),
                     {"run": dict(meta=meta, metrics=dict(window=r, per_phase=per_leg, impedance=im, valve=V),
                                  tracking=tracking, impedance=impedance, heading=heading, raw=raw)},
                     do_compression=True, long_field_names=True, oned_as="column")
    print(f"wrote {os.path.join(out, tag + '.mat')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="+")
    ap.add_argument("--out", default=os.path.join(SWEEP, "matlab"))
    a = ap.parse_args()
    sj = os.path.join(SWEEP, "analysis", "scores.json")
    scores = json.load(open(sj)) if os.path.isfile(sj) else {}
    for t in a.tags:
        export(t, a.out, scores)


if __name__ == "__main__":
    main()
