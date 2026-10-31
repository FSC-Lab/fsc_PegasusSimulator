#!/usr/bin/env python3
"""Payload pick-and-drop, EXPERIMENT: the 2026-10-28 hardware flight -> the simulation's pick-and-drop `run` struct, for MATLAB.

    /usr/bin/python3 results/utils/experiment_pick_and_drop.py

Reads the run's ros2 bag from results/experiment_results/flight_test_data/pick_and_drop/<name>/ (copied there unchanged) and
writes, under results/experiment_results/matlab_experiment_data/pick_and_drop/:
  <name>.mat                      struct `run`: meta, metrics, tracking, raw -- the layout of
                                  simulation_results/matlab_simulation_data/pick_and_drop/<method>_pnd_run<k>.mat
  pick_and_drop_metrics.mat       metrics_runs, metrics_mean, metrics_phase, phases, window_s, L_arm_m
  configs/                        the controller and arm configs as flown (taken from git at the flown commit)
  README.md                       the folder's guide (self-contained, like the simulation's)

The conversion, the metrics and the struct are experiment_pick_and_place.py's (the 2026-10-16 D-2 export, also a
pick-and-drop): this file only names a different run, folder and README. Extracted bags are cached in
$AM_EXP_CACHE (default /tmp/am_experiment_cache), outside the repository.

The run: the last of four flights on 2026-10-28 that lowered the Drop Margin 12 -> 8 -> 6 -> 5 cm (the bag names).
The margin was typed on the arm GS; the bag does not record it (no parameter events; the GS reports it on screen
only), so the run carries drop_height = 0.05 from the bag name. The four-flight comparison that chose it:
docs/docs_aerial_manipulator/archive/wb_pick_drop_flight_20261028/ (tools/compare_margins.py).
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import experiment_pick_and_place as E   # noqa: E402

EXP = E.EXP
E.BAG_ROOT = os.path.join(EXP, "flight_test_data", "pick_and_drop")
E.MAT_DIR = MAT_DIR = os.path.join(EXP, "matlab_experiment_data", "pick_and_drop")
E.METRICS_NAME = "pick_and_drop_metrics.mat"
E.SCORES_NAME = "pick_and_drop_scores.json"
ANALYSIS = "docs/docs_aerial_manipulator/archive/wb_pick_drop_flight_20261028"
BAGS = "docs/experimental_data_ros2_bag/1028 - T650-AM whole-body Tracking, Pick&Drop, Valve-Turning-*/"

E.RUNS = {
    "whole_body_l1_pnd_M5": dict(
        tag="M5", method="whole_body_l1", flown="2026-10-28 16:56",
        bag="flight_wb_l1_4d_pick_and_drop_5offset_20261028_165603",
        payload="the basket assembly (basket, holder fin, mocap markers; rigid body obj_0), hook grasp",
        drop_height=0.05,   # the Drop Margin typed on the arm GS (the bag name); the yaml's value is 0.10
        # (name in configs/, repository, commit, path): the files the flight ran. a12e454 (2026-10-28 09:48) is the last
        # change to the pick-and-drop yaml before the flight, and the bag shows it: Drop Start z 1.21. f812b48
        # (2026-10-17) is the last change to the arm controller yaml.
        configs=[("whole_body_l1_4d_hw_pick_and_drop.yaml", E.AUTOPILOT, "a12e454",
                  "config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_drop.yaml"),
                 ("arm_external_torque_controller_hardware_aerial_pwm.yaml", E.ARM_REPO, "f812b48",
                  "open_manipulator_x_custom_controller/config/external_torque_controller_hardware_aerial_pwm.yaml")]),
}
# The session's four bags (tools/compare_margins.py). Release height = the basket centre at the gripper's OPEN
# command above its flat rest on the tray floor (z = 0.931 m, measured: M6 0.932, M5 0.930).
SESSION = [
    ("M12", "16:16", "flight_wb_l1_4d_pick_and_drop_12offset_20261028_161640", "12 cm",
     "released 138 mm up, hit at 1.05 m/s, bounced off the tray and fell to the floor", "83.5", "209.9 / 58.1", "59.7", "not exported"),
    ("M8", "16:27", "flight_wb_l1_4d_pick_and_drop_8offset_20261028_162711", "8 cm (*)",
     "released 120 mm up, hit at 1.03 m/s; in the tray, 71 mm from the drop point, tilted 13 deg", "84.9", "168.8 / 53.7", "56.1",
     "not exported"),
    ("M6", "16:48", "flight_wb_l1_4d_pick_and_drop_6offset_20261028_164816", "6 cm",
     "released 81 mm up, hit at 0.81 m/s, slid 66 mm; in the tray, 129 mm from the drop point, tilted 5 deg", "85.4",
     "165.4 / 50.5", "53.4", "not exported"),
    ("M5", "16:56", "flight_wb_l1_4d_pick_and_drop_5offset_20261028_165603", "5 cm",
     "released 75 mm up, hit at 0.73 m/s, no bounce, slid 16 mm; in the tray, 58 mm from the drop point, tilted 3 deg",
     "79.0", "158.9 / 47.2", "50.2", "**exported**"),
]


def write_readme(out):
    from readme_text import _md_table
    PHASES, PHASE_KEYS, LEGS = E.PHASES, E.PHASE_KEYS, E.LEGS
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
    fall = 1e3 * oc["fall_m"]                     # = payload_at_open_m z - rest_m z
    L = [
        "# Experiment results, MATLAB data (payload pick-and-drop)", "",
        "The hardware pick-and-drop flight, converted from its ros2 bag. This folder is self-contained:",
        "these files plus MATLAB are all you need, with no ROS or custom message definitions. The `run` struct",
        "has the layout of the simulation's `matlab_simulation_data/pick_and_drop/<method>_pnd_run<k>.mat`, so",
        "a script written for those files runs on this one.", "",
        "## Files", "",
        "```",
        f"{name}.mat                one run, struct `run`",
        "pick_and_drop_metrics.mat                metrics_runs / metrics_mean = the run's numbers (one row),",
        "                                         metrics_phase = per phase, phases = the phase table,",
        "                                         window_s = the evaluation window, L_arm_m = L",
        "configs/whole_body_l1_4d_hw_pick_and_drop.yaml               the controller + planner, as flown",
        "configs/arm_external_torque_controller_hardware_aerial_pwm.yaml   the arm controller, as flown",
        "```", "",
        "- **Name:** `<method>_pnd_<run>`, the simulation's naming with the flight tag as the run. The tag is the",
        "  Drop Margin: `M5` = 5 cm.",
        "- **Method:** `whole_body_l1` = Proposed. Only the proposed controller flew this task on hardware.",
        "- **Configs:** taken from git at the commits the vehicle ran (first line of each file). The Drop Margin is",
        "  NOT the yaml's `pick_place_drop_height` (0.10 m): it was typed on the arm ground station, 0.05 m.", "",
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
        "## The session: four flights, the Drop Margin lowered each time", "",
        "Same mission, controller and payload; only the Drop Margin (how far above the Drop point the gripper opens)",
        "changed. All four completed the six legs; in each the pilot took over in the land leg. Released ... up = the",
        "basket centre at the gripper's OPEN command above its flat rest on the tray floor (z = 0.931 m); hit = the",
        "peak fall speed. Tracking over the whole mission.", "",
        _md_table(["run", "flown", "original ros2 bag", "Drop Margin", "the drop", "window [s]", "‖ε<sub>UAV</sub>‖ max / RMS [mm]",
                   "EE RMSE [mm]", "in this folder"],
                  [[t, f"2026-10-28 {hh}", f"`{b}`", m, wh, win, eps, ee, ex] for t, hh, b, m, wh, win, eps, ee, ex in SESSION]),
        "",
        "(*) The bag records the drop leg's claw target, not the margin: 1.241 / 1.219 / 1.178 / 1.168 m. The steps",
        "(22 / 41 / 10 mm) match the names except M8, whose claw sat about 2 cm higher than an 8 cm margin gives.",
        "The heights in the table are measured on the basket, so they do not depend on the names.", "",
        "**M5 is exported: the lowest release gave the best drop on every measure** (lowest release, slowest impact,",
        "no bounce, least slide, flattest rest, closest to the drop point), and the best tracking of the four.",
        f"The comparison: `{ANALYSIS}/` (`tools/compare_margins.py`).", "",
        "The bag of the exported run is kept unchanged in `results/experiment_results/flight_test_data/pick_and_drop/<name>/`",
        f"on the lab desktop. The other bags are in `{BAGS}`. The earlier pick-and-drop flight D-2",
        "(2026-10-16, Drop Margin 0.20 m) is exported in `../pick_and_place/`.", "",
        "## How this differs from the simulation's pick-and-drop", "",
        "- **The operator paced the mission.** Each step was pressed on the arm ground station, so the phases come",
        "  from the planner's status in the bag. The simulation used a fixed timetable.",
        "- **The drop was pressed by hand,** 0.51 s after Ready To Drop ended (the arm GS's Drop button), not by the",
        "  planner's automatic drop, and Ready To Drop was pressed 0.6 s after arriving at Drop Start, so the planner",
        "  did not aim the drop at the measured basket (it needs a 4 s hover there). The simulation's drop margin is",
        "  0.20 m, flown on the same timetable for all three controllers.",
        "- **One controller.** No Geo-L1 run exists for this task on hardware.",
        "- **The basket is measured by OptiTrack** (rigid body `obj_0`), not simulator ground truth.",
        f"  Mis-solved samples are removed: {meta['payload_flips_removed']} flips (tilt > {E.FLIP_TILT_DEG:.0f} deg) and "
        f"{meta['payload_outliers_removed']} jumps",
        f"  (more than {1e3 * E.OUTLIER_POS_M:.0f} mm or {E.OUTLIER_YAW_DEG:.0f} deg of yaw off the rolling median of "
        f"{2 * E.OUTLIER_HALF_WINDOW + 1} samples), {100 * (meta['payload_flips_removed'] + meta['payload_outliers_removed']) / o['n_payload_raw']:.1f} % of the stream.",
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
        f"{oc['drop_margin_m']:.2f} m above it. Reconstructed from the drop leg's claw target (the bag does not hold",
        "  the Drop point): its xy is exact, its z assumes the 5 cm margin.",
        f"- **Claw at the OPEN command,** relative to its target: {xyz(oc['claw_err_at_open_mm'])} mm (x, y, z).",
        f"- **Basket at the OPEN command:** {oc['release_off_mm']:.0f} mm from the drop point in xy; its centre was "
        f"{fall:.0f} mm above where it came to rest.",
        f"- **Basket at rest,** in the tray: {oc['rest_off_mm']:.0f} mm from the drop point in xy {xyz(oc['rest_off_xy_mm'])}, "
        f"tilted {oc['rest_tilt_deg']:.1f} deg.",
        f"- **Gripper:** CLOSE at {oc['gripper_close_s']:.2f} s, grasp reported at {oc['grasp_reported_s']:.2f} s, "
        f"OPEN at {oc['gripper_open_s']:.2f} s.",
        "- The claw error here is the model FK of the measured pose and joints on the 100 Hz grid; `compare_margins.py`",
        "  reads the planner's own `current_ee` instead, a few mm different ((-53, 6, 3) mm here).", "",
        "## Conditions", "",
        "- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm in torque mode, 3.746 kg in the controller model.",
        "- **Feedback:** OptiTrack fused in PX4 EKF2 (~100 Hz); joint encoders.",
        "- **Controller:** whole-body L1, 4-D attribution, `AM_HW_PROFILE=pick_and_drop`",
        f"  (`configs/whole_body_l1_4d_hw_pick_and_drop.yaml`: k_R / k_w {o['gains']['wb_k_r']:g} / "
        f"{o['gains']['wb_k_w']:g}, K_y / D_y {o['gains']['wb_ky_x']:g} / {o['gains']['wb_dy_x']:g}, "
        f"K_psi / D_psi {o['gains']['wb_ky_psi']:g} / {o['gains']['wb_dy_psi']:g}, the wrist-roll stuck-joint hold on).",
        "- **Arm poses:** pick = carry = drop [0, 32, 38, 0] deg, home [0, 40, 40, 0] deg.",
        f"- **EE offset** (the claw point in the basket's frame): ({oc['ee_offset_m'][0]:.2f}, {oc['ee_offset_m'][1]:.2f}, "
        f"{oc['ee_offset_m'][2]:.2f}) m. Side Margin 0.10 m, Vertical Margin 0.20 m, Back Margin 0.20 m,",
        f"  Drop Margin {oc['drop_margin_m']:.2f} m, Drop Start z {goals['base_m'][2][2]:.2f} m.",
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
        "Regenerate (from the bag in `results/experiment_results/flight_test_data/pick_and_drop/`):", "",
        "```bash", "/usr/bin/python3 results/utils/experiment_pick_and_drop.py", "```", ""]
    with open(os.path.join(MAT_DIR, "README.md"), "w") as f:
        f.write("\n".join(L))
    print("wrote", os.path.relpath(os.path.join(MAT_DIR, "README.md"), E.RESULTS), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage2", action="store_true")
    a = ap.parse_args()
    if a.stage2:
        E.stage2(readme=write_readme)
        return
    E.extract()
    env = dict(os.environ, PYTHONNOUSERSITE="1", AM_EXP_CACHE=E.CACHE)
    subprocess.run(["/usr/bin/python3", os.path.abspath(__file__), "--stage2"], env=env, check=True)


if __name__ == "__main__":
    main()
