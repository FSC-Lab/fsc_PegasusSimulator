#!/usr/bin/env python3
"""Free-flight trajectory tracking, EXPERIMENT: score the formal hardware runs from their ros2 bags, average
the final campaign's runs of each setting, and write the paper table and the bag index.

    /usr/bin/python3 results/utils/experiment_tracking.py [--paper <main.tex>] [--copy-bags]

Writes, under results/experiment_results/:
  flight_test_index.md   every aerial-manipulator flight-test bag, the scored runs, which runs fill each entry
  matlab_experiment_data/free_flight_tracking/            self-contained, laid out like the simulation's folder:
    <shape>/v<speed>/<method>_<shape>_v<speed>_run<k>.mat every AVERAGED run, the same struct `run` as the simulation
    free_flight_tracking_rmse.mat                         rmse_mean (the table), rmse_runs, rmse_all
    configs/, README.md                                   the flown configs (from git), the folder's guide
  matlab_experiment_data/README.md                        index of the task folders
  flight_test_data/free_flight_tracking/<shape>/v<speed>/<method>_<shape>_v<speed>_<runs of the bag>/
                         (--copy-bags) each bag holding an averaged run, files unchanged
and, with --paper, the table labelled tab:exp_free_flight_tracking in main.tex. Extracted bags are cached
in $AM_EXP_CACHE (default /tmp/am_experiment_cache), outside the repository.

Metrics: the hardware report's definitions (docs/docs_aerial_manipulator/archive/wb_vs_decoupled_flight_
20260928/tools/metrics.py), which the simulation table uses too. Each run is scored over its whole planner
run ('EXECUTING T=...' with T >= 10 s, the n-th of the bag, from its start to the next planner status;
the go-to-start and go-home transitions plan 3-7 s, the shortest EE run, the 0.20 m/s circle, 19.7 s),
trimmed 0.1 s at each end, 100 Hz grid, against the planner's streamed whole-body reference; the measured
EE is the planner model's FK of the EKF2-fused odometry and the joint encoders.

Selection (the simulation's rule, adapted): each table entry is the MEAN, column by column, of two repeated
one-lap tests of its (trajectory, speed, method): both laps of the setting's latest eligible flight (each bag
flies two laps back to back, each its own planner run from rest to rest). The rule never looks at the error;
only one setting (Proposed, circle 0.13 m/s) has more than one eligible flight. Eligible = flown in the final
campaign (2026-10-27 to 10-29: the whole-body
controller with the 2026-10-16 wrist-roll fix, Geo-L1 on its 2026-10-01 tune, both on the same hardware
yamls throughout), the planned task (q2 = 25 +- 15 deg, four cycles per lap, fold 55 deg), and a run that
reached the end of its plan. Earlier runs stay scored and listed, but are not averaged.

Interpreters: extraction needs rclpy + px4_msgs (ROS sourced, the user-site numpy 2); scoring needs
scipy (apt, numpy 1.21) -> stage 2 re-executes this file with PYTHONNOUSERSITE=1.
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.abspath(os.path.join(HERE, ".."))
REPO = os.path.abspath(os.path.join(RESULTS, ".."))
BAGS = os.path.join(REPO, "docs", "experimental_data_ros2_bag")
ARCH = os.path.join(REPO, "docs", "docs_aerial_manipulator", "archive")
TOOLS = os.path.join(ARCH, "wb_vs_decoupled_flight_20260928", "tools")
EXP = os.path.join(RESULTS, "experiment_results")
OUT_MD = os.path.join(EXP, "flight_test_index.md")
FLIGHT = os.path.join(EXP, "flight_test_data")           # the copied bags, one folder per task
BAG_COPIES = os.path.join(FLIGHT, "free_flight_tracking")
MAT_DIR = os.path.join(EXP, "matlab_experiment_data")
FF_MAT = os.path.join(MAT_DIR, "free_flight_tracking")    # self-contained, like the simulation's task folders
AUTOPILOT = os.environ.get("FSC_AUTOPILOT_REPO", os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2"))
ARM_REPO = os.environ.get("FSC_OM_ARM_REPO", os.path.expanduser("~/ros2_ws/src/fsc_open_manipulator"))
# (name in configs/, repository, path): the hardware configs of the final campaign
CONFIG_FILES = [
    ("whole_body_l1_4d_hw.yaml", AUTOPILOT, "config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml"),
    ("geometric_l1_hw.yaml", AUTOPILOT, "config/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650.yaml"),
    ("arm_external_torque_controller_hardware_aerial_pwm.yaml", ARM_REPO,
     "open_manipulator_x_custom_controller/config/external_torque_controller_hardware_aerial_pwm.yaml"),
    ("arm_position_controller_hardware_aerial.yaml", ARM_REPO,
     "open_manipulator_x_custom_controller/config/position_controller_hardware_aerial.yaml"),
]
METHOD_LONG = {"whole_body_l1": "Whole-body L1 impedance (proposed)",
               "geometric_l1": "Geometric control with L1 adaptive augmentation (Cai et al., CEP 2025), position-mode arm"}
CACHE = os.environ.get("AM_EXP_CACHE", "/tmp/am_experiment_cache")

# The controller configurations the scored runs flew (fsc_autopilot_ros2 hardware yamls
# params_single_aerial_manipulator_{whole_body_l1_4d,geometric_l1}_direct_actuation_t650.yaml).
CONFIGS = {
    "wb_0927": "whole-body L1 4-D, the 2026-09-27 tune (K_psi / D_psi 0.2484 / 0.2903)",
    "wb_1016": "whole-body L1 4-D, the 2026-09-27 tune + the 2026-10-16 wrist-roll fix (K_psi / D_psi 2.5 / 0.93, "
               "j4 stuck-joint hold; fsc_autopilot_ros2 a316625)",
    "geo_prev": "Geo-L1, the gain set flown since 2026-08-21",
    "geo_1001": "Geo-L1, the 2026-10-01 tune",
}
CONFIG_SHORT = {"wb_0927": "09-27 tune", "wb_1016": "09-27 tune + 10-16 wrist-roll fix",
                "geo_prev": "previous gains", "geo_1001": "10-01 tune"}

# The final campaign: every setting re-flown 2026-10-27..29 on one configuration per method. Runs before it are
# scored and listed but not averaged into the table.
FINAL_FROM, FINAL_TO = "10-27", "10-29"
EARLIER = "before the final campaign (2026-10-27 to 10-29)"
RULE = "both laps of the setting's latest flight in the final campaign"

# The formal free-flight comparison runs. key: (tag, date time, method, shape, speed, bag, n-th planner run,
#                                               eligible, note, CONFIGS key)
RUNS = {
    "w1": ("WB-1", "09-28 17:11", "whole_body_l1", "circle", 0.13, "flight_wb_l1_4d_circle_20260928_171154", 0, True, "", "wb_0927"),
    "w2": ("WB-2", "09-28 17:22", "whole_body_l1", "circle", 0.13, "flight_wb_l1_4d_circle_20260928_172242", 0, True, "", "wb_0927"),
    "d1": ("DEC-1", "09-28 18:28", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_20260928_182835", 0, False,
           "previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune", "geo_prev"),
    "d2": ("DEC-2", "09-28 18:36", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_20260928_183608", 0, False,
           "previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune", "geo_prev"),
    "r1": ("DEC-3", "10-02 13:42", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_20261002_134247", 0, True, "", "geo_1001"),
    "r2": ("DEC-4", "10-02 13:42", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_20261002_134247", 1, False,
           "incomplete: the operator returned to SAFETY 0.8 s before the plan ended; flown through a "
           "ground-station clock step that reset the L1 estimate", "geo_1001"),
    "w3": ("WB-3", "10-05 16:24", "whole_body_l1", "figure8", 0.10, "flight_wb_l1_4d_figure8_vel010_20261005_162406", 0, True, "", "wb_0927"),
    "w4": ("WB-4", "10-05 16:24", "whole_body_l1", "figure8", 0.10, "flight_wb_l1_4d_figure8_vel010_20261005_162406", 1, True, "", "wb_0927"),
    "r5": ("DEC-5", "10-05 16:37", "geometric_l1", "figure8", 0.10, "flight_decoupled_l1_figure8_vel010_20261005_163734", 0, True, "", "geo_1001"),
    "r6": ("DEC-6", "10-05 16:54", "geometric_l1", "figure8", 0.13, "flight_decoupled_l1_figure8_vel013_20261005_165431", 0, True, "", "geo_1001"),
    "w5": ("WB-5", "10-05 17:01", "whole_body_l1", "figure8", 0.13, "flight_wb_l1_4d_figure8_vel013_20261005_170129", 0, True, "", "wb_0927"),
    "w6": ("WB-6", "10-05 17:01", "whole_body_l1", "figure8", 0.13, "flight_wb_l1_4d_figure8_vel013_20261005_170129", 1, True, "", "wb_0927"),
    "w7": ("WB-7", "10-27 17:50", "whole_body_l1", "circle", 0.20, "flight_wb_l1_4d_circle_020_20261027_175015", 0, True, "", "wb_1016"),
    "w8": ("WB-8", "10-27 17:50", "whole_body_l1", "circle", 0.20, "flight_wb_l1_4d_circle_020_20261027_175015", 1, True, "", "wb_1016"),
    "w9": ("WB-9", "10-27 17:58", "whole_body_l1", "figure8", 0.20, "flight_wb_l1_4d_figure8_020_20261027_175849", 0, True, "", "wb_1016"),
    "w10": ("WB-10", "10-27 17:58", "whole_body_l1", "figure8", 0.20, "flight_wb_l1_4d_figure8_020_20261027_175849", 1, True, "", "wb_1016"),
    "w11": ("WB-11", "10-27 18:06", "whole_body_l1", "circle", 0.10, "flight_wb_l1_4d_circle_010_20261027_180659", 0, True, "", "wb_1016"),
    "w12": ("WB-12", "10-27 18:06", "whole_body_l1", "circle", 0.10, "flight_wb_l1_4d_circle_010_20261027_180659", 1, True, "", "wb_1016"),
    "r7": ("DEC-7", "10-27 18:13", "geometric_l1", "circle", 0.10, "flight_decoupled_l1_circle_010_20261027_181331", 0, True, "", "geo_1001"),
    "r8": ("DEC-8", "10-27 18:13", "geometric_l1", "circle", 0.10, "flight_decoupled_l1_circle_010_20261027_181331", 1, True, "", "geo_1001"),
    "w13": ("WB-13", "10-28 14:46", "whole_body_l1", "circle", 0.13, "flight_wb_l1_4d_circle_013_20261028_144602", 0, True, "", "wb_1016"),
    "w14": ("WB-14", "10-28 14:46", "whole_body_l1", "circle", 0.13, "flight_wb_l1_4d_circle_013_20261028_144602", 1, True, "", "wb_1016"),
    "w15": ("WB-15", "10-28 15:08", "whole_body_l1", "circle", 0.13, "flight_wb_l1_4d_circle_013_20261028_150821", 0, True, "", "wb_1016"),
    "w16": ("WB-16", "10-28 15:08", "whole_body_l1", "circle", 0.13, "flight_wb_l1_4d_circle_013_20261028_150821", 1, True, "", "wb_1016"),
    "r9": ("DEC-9", "10-28 15:16", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_013_20261028_151600", 0, True, "", "geo_1001"),
    "r10": ("DEC-10", "10-28 15:16", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_013_20261028_151600", 1, True, "", "geo_1001"),
    "w17": ("WB-17", "10-28 15:22", "whole_body_l1", "figure8", 0.13, "flight_wb_l1_4d_figure8_013_20261028_152202", 0, True, "", "wb_1016"),
    "w18": ("WB-18", "10-28 15:22", "whole_body_l1", "figure8", 0.13, "flight_wb_l1_4d_figure8_013_20261028_152202", 1, True, "", "wb_1016"),
    "r11": ("DEC-11", "10-28 15:28", "geometric_l1", "figure8", 0.13, "flight_decoupled_l1_figure8_013_20261028_152811", 0, True, "", "geo_1001"),
    "r12": ("DEC-12", "10-28 15:28", "geometric_l1", "figure8", 0.13, "flight_decoupled_l1_figure8_013_20261028_152811", 1, True, "", "geo_1001"),
    "w19": ("WB-19", "10-29 15:06", "whole_body_l1", "figure8", 0.10, "flight_wb_l1_4d_figure8_010_20261029_150625", 0, True, "", "wb_1016"),
    "w20": ("WB-20", "10-29 15:06", "whole_body_l1", "figure8", 0.10, "flight_wb_l1_4d_figure8_010_20261029_150625", 1, True, "", "wb_1016"),
    "r13": ("DEC-13", "10-29 15:25", "geometric_l1", "figure8", 0.10, "flight_decoupled_l1_figure8_010_20261029_152543", 0, True, "", "geo_1001"),
    "r14": ("DEC-14", "10-29 15:25", "geometric_l1", "figure8", 0.10, "flight_decoupled_l1_figure8_010_20261029_152543", 1, True, "", "geo_1001"),
}

# Every aerial-manipulator flight-test bag (the index). (date, bag, controller, what was flown, runs scored here)
INDEX = [
    ("09-12", "flight_wb_l1_20260912_190224", "whole-body L1 (6-D observer)", "first hardware flight of the whole-body law, short", "--"),
    ("09-12", "flight_wb_l1_20260912_201430", "whole-body L1 (6-D observer)", "hover and arm moves in DIRECT (development)", "--"),
    ("09-18", "flight_wb_l1_4d_20260918_144828", "whole-body L1 4-D", "first 4-D flight: planner legs (EE excursions, go-home, 0.61 m base step)", "--"),
    ("09-18", "flight_wb_l1_4d_20260918_152112", "whole-body L1 4-D", "metadata only, no data file", "--"),
    ("09-21", "flight_wb_l1_4d_circle_20260921_112328", "whole-body L1 4-D", "circle attempt (development)", "--"),
    ("09-21", "flight_wb_l1_4d_circle_20260921_115207", "whole-body L1 4-D", "circle attempt (development)", "--"),
    ("09-21", "flight_wb_l1_4d_circle_20260921_123637", "whole-body L1 4-D", "circle attempt: go-to-start only, mocap flips", "--"),
    ("09-24", "flight_wb_l1_4d_circle_20260924_120228", "whole-body L1 4-D", "circle, earlier tune, arm q2 30 +- 10 deg / 48 s; mocap freeze mid-run", "--"),
    ("09-24", "flight_wb_l1_4d_circle_20260924_120546", "whole-body L1 4-D", "circle, earlier tune, arm q2 30 +- 10 deg / 48 s", "--"),
    ("09-24", "flight_wb_l1_4d_circle_20260924_165654", "whole-body L1 4-D", "circle, earlier tune, arm q2 25 +- 15 deg / 12 s", "--"),
    ("09-24", "flight_wb_l1_4d_circle_20260924_172101", "whole-body L1 4-D", "circle, earlier tune, arm q2 25 +- 15 deg / 6 s", "--"),
    ("09-28", "flight_wb_l1_4d_circle_20260928_171154", "whole-body L1 4-D", "circle 0.13 m/s", "WB-1"),
    ("09-28", "flight_wb_l1_4d_circle_20260928_172242", "whole-body L1 4-D", "circle 0.13 m/s", "WB-2"),
    ("09-28", "flight_decoupled_l1_circle_20260928_182835", "Geo-L1 (previous gains)", "circle 0.13 m/s", "DEC-1"),
    ("09-28", "flight_decoupled_l1_circle_20260928_183608", "Geo-L1 (previous gains)", "circle 0.13 m/s", "DEC-2"),
    ("09-29", "flight_wb_l1_4d_ps4_20260929_160027", "whole-body L1 4-D", "PS4 teleoperation", "--"),
    ("10-02", "flight_decoupled_l1_circle_20261002_134247", "Geo-L1 (2026-10-01 tune)", "circle 0.13 m/s, two runs", "DEC-3, DEC-4"),
    ("10-02", "flight_decoupled_l1_circle_ps420261002_134830", "Geo-L1 (2026-10-01 tune)", "PS4 teleoperation", "--"),
    ("10-05", "flight_wb_l1_4d_figure8_vel010_20261005_162406", "whole-body L1 4-D", "figure-8 0.10 m/s, two runs", "WB-3, WB-4"),
    ("10-05", "flight_decoupled_l1_figure8_vel010_20261005_163734", "Geo-L1 (2026-10-01 tune)", "figure-8 0.10 m/s", "DEC-5"),
    ("10-05", "flight_decoupled_l1_figure8_vel013_20261005_165431", "Geo-L1 (2026-10-01 tune)", "figure-8 0.13 m/s", "DEC-6"),
    ("10-05", "flight_wb_l1_4d_figure8_vel013_20261005_170129", "whole-body L1 4-D", "figure-8 0.13 m/s, two runs", "WB-5, WB-6"),
    ("10-09", "flight_wb_l1_4d_pick_and_place_20261009_172557", "whole-body L1 4-D", "pick-and-place PP-1: the six-leg mission without a payload, completed", "--"),
    ("10-09", "flight_wb_l1_4d_pick_and_place_20261009_180228", "whole-body L1 4-D", "pick-and-place PP-2: basket picked, placed 166 mm short; PX4 link froze in flight", "--"),
    ("10-09", "flight_wb_l1_4d_pick_and_place_20261009_181817", "whole-body L1 4-D", "pick-and-place PP-3: gripper closed above the arch, nothing lifted", "--"),
    ("10-10", "flight_wb_l1_4d_pick_and_place_20261010_190719", "whole-body L1 4-D", "pick-and-place PP-4: set-down pulled the basket off the hat", "--"),
    ("10-14", "flight_wb_l1_4d_pick_and_drop_20261014_172845", "whole-body L1 4-D", "pick-and-drop PD-1, no payload, over the full bin; ABORT at the drop hold", "--"),
    ("10-14", "flight_wb_l1_4d_pick_and_drop_20261014_183914", "whole-body L1 4-D", "pick-and-drop PD-2, no payload, over the cut tray; operator revert before the drop", "--"),
    ("10-15", "flight_wb_l1_4d_pick_and_drop_empty_20261015_172318", "whole-body L1 4-D", "pick-and-drop E-1, no payload, completed", "--"),
    ("10-15", "flight_wb_l1_4d_pick_and_drop_20261015_175059", "whole-body L1 4-D", "pick-and-drop D-1, basket dropped, completed", "--"),
    ("10-15", "flight_wb_l1_4d_pick_and_drop_20261015_180850", "whole-body L1 4-D", "pick-and-drop D-2, basket pulled out of the jaws in the pick climb", "--"),
    ("10-16", "flight_wb_l1_4d_pick_and_drop_empty_20261016_134438", "whole-body L1 4-D", "pick-and-drop E-1, no payload, completed", "--"),
    ("10-16", "flight_wb_l1_4d_pick_and_drop_20261016_135100", "whole-body L1 4-D", "pick-and-drop D-1, completed; wrong grip, basket rest 112 mm off", "--"),
    ("10-16", "flight_wb_l1_4d_pick_and_drop_20261016_140020", "whole-body L1 4-D", "pick-and-drop D-2, completed, basket flat", "pick-and-place D-2"),
    ("10-19", "flight_wb_l1_4d_pick_and_drop_20261019_170849", "whole-body L1 4-D", "pick-and-drop: PX4 link cut in the pick's lift, vehicle flipped", "--"),
    ("10-19", "flight_wb_l1_4d_pick_and_drop_20261019_172008", "whole-body L1 4-D", "hover (link check)", "--"),
    ("10-20", "flight_wb_l1_4d_pick_and_drop_20261020_182234", "whole-body L1 4-D", "pick-and-drop, completed, no link loss", "--"),
    ("10-21", "stale/flight_wb_l1_4d_box_pushing_20261021_161745", "whole-body L1 4-D", "box pushing: Plan refused (box off the configured table)", "--"),
    ("10-21", "stale/flight_wb_l1_4d_box_pushing_20261021_162604", "whole-body L1 4-D", "box pushing: Plan refused (no metadata.yaml)", "--"),
    ("10-21", "free_flight/flight_wb_l1_4d_box_pushing_20261021_164338", "whole-body L1 4-D", "box pushing: go to start and hover over a typed box point, land", "--"),
    ("10-21", "stale/flight_wb_l1_4d_box_pushing_20261021_165311", "whole-body L1 4-D", "box pushing: Plan refused (box off the configured table)", "--"),
    ("10-21", "box_pushing/flight_wb_l1_4d_box_pushing_20261021_170123", "whole-body L1 4-D", "first box push, 280 g, 0.10 m: pushed; q3-guard abort at the exit", "--"),
    ("10-23", "flight_wb_l1_4d_box_pushing_20261023_172853", "whole-body L1 4-D", "box pushing: q3 rate guard aborted at the grasp", "--"),
    ("10-24", "flight_wb_l1_4d_box_pushing_240g_20261024_150423", "whole-body L1 4-D", "box pushing #1 (280 g): pushed ~70 mm, box-tilt abort", "--"),
    ("10-24", "flight_wb_l1_4d_box_pushing_240g_20261024_151159", "whole-body L1 4-D", "box pushing #2 (280 g): missed the fin, q3 rate guard", "--"),
    ("10-24", "flight_wb_l1_4d_box_pushing_240g_20261024_151634", "whole-body L1 4-D", "box pushing #3 (280 g): box blown off the table in go to start", "--"),
    ("10-24", "flight_wb_l1_4d_box_pushing_240g_20261024_162850", "whole-body L1 4-D", "box pushing A (280 g): q3 rate guard in the release", "--"),
    ("10-24", "flight_wb_l1_4d_box_pushing_240g_20261024_163413", "whole-body L1 4-D", "box pushing B (280 g): q3 rate guard at the grasp", "--"),
    ("10-24", "flight_wb_l1_4d_box_pushing_240g_20261024_164419", "whole-body L1 4-D", "box pushing C (280 g): q3 rate guard at the grasp", "--"),
    ("10-24", "flight_wb_l1_4d_box_pushing_280g_20261024_194310", "whole-body L1 4-D", "box pushing 280 g, 0.50 m: box at its plan, box-tilt abort", "--"),
    ("10-24", "flight_wb_l1_4d_box_pushing_500g_20261024_195657", "whole-body L1 4-D", "box pushing 510 g: arm buckled at the grasp, watchdog to SAFETY", "--"),
    ("10-27", "flight_wb_l1_4d_box_pushing_300g_20261027_171114", "whole-body L1 4-D", "box pushing 280 g, 0.40 m: push completed, q3-guard abort after the release", "box-pushing BP-1"),
    ("10-27", "flight_wb_l1_4d_box_pushing_300g_20261027_172227", "whole-body L1 4-D", "box pushing 280 g, 0.40 m: q3 guard aborted at 348 mm", "box-pushing BP-2"),
    ("10-27", "flight_wb_l1_4d_circle_020_20261027_175015", "whole-body L1 4-D (+ 10-16 wrist-roll fix)", "circle 0.20 m/s, two runs", "WB-7, WB-8"),
    ("10-27", "flight_wb_l1_4d_figure8_020_20261027_175849", "whole-body L1 4-D (+ 10-16 wrist-roll fix)", "figure-8 0.20 m/s, two runs", "WB-9, WB-10"),
    ("10-27", "flight_wb_l1_4d_circle_010_20261027_180659", "whole-body L1 4-D (+ 10-16 wrist-roll fix)", "circle 0.10 m/s, two runs", "WB-11, WB-12"),
    ("10-27", "flight_decoupled_l1_circle_010_20261027_181331", "Geo-L1 (2026-10-01 tune)", "circle 0.10 m/s, two runs", "DEC-7, DEC-8"),
    ("10-28", "flight_wb_l1_4d_circle_013_20261028_144602", "whole-body L1 4-D (+ 10-16 wrist-roll fix)", "circle 0.13 m/s, two runs", "WB-13, WB-14"),
    ("10-28", "flight_wb_l1_4d_circle_013_20261028_150821", "whole-body L1 4-D (+ 10-16 wrist-roll fix)", "circle 0.13 m/s, two runs", "WB-15, WB-16"),
    ("10-28", "flight_decoupled_l1_circle_013_20261028_151600", "Geo-L1 (2026-10-01 tune)", "circle 0.13 m/s, two runs", "DEC-9, DEC-10"),
    ("10-28", "flight_wb_l1_4d_figure8_013_20261028_152202", "whole-body L1 4-D (+ 10-16 wrist-roll fix)", "figure-8 0.13 m/s, two runs", "WB-17, WB-18"),
    ("10-28", "flight_decoupled_l1_figure8_013_20261028_152811", "Geo-L1 (2026-10-01 tune)", "figure-8 0.13 m/s, two runs", "DEC-11, DEC-12"),
    ("10-28", "flight_wb_l1_4d_pick_and_drop_12offset_20261028_161640", "whole-body L1 4-D", "pick-and-drop M12, Drop Margin 12 cm: completed; the basket bounced off the tray onto the floor", "--"),
    ("10-28", "flight_wb_l1_4d_pick_and_drop_8offset_20261028_162711", "whole-body L1 4-D", "pick-and-drop M8, Drop Margin 8 cm: completed; basket in the tray, tilted 13 deg", "--"),
    ("10-28", "flight_wb_l1_4d_pick_and_drop_6offset_20261028_164816", "whole-body L1 4-D", "pick-and-drop M6, Drop Margin 6 cm: completed; basket in the tray, slid 66 mm", "--"),
    ("10-28", "flight_wb_l1_4d_pick_and_drop_5offset_20261028_165603", "whole-body L1 4-D", "pick-and-drop M5, Drop Margin 5 cm: completed; basket flat in the tray", "pick-and-drop M5"),
    ("10-28", "rosbag2_2026_10_28-18_06_52_Valve_Turning", "whole-body L1 4-D", "valve turning (not analysed here)", "--"),
    ("10-28", "rosbag2_2026_10_28-18_18_33_Valve_Turning", "whole-body L1 4-D", "valve turning (not analysed here)", "--"),
    ("10-29", "flight_wb_l1_4d_valve_turning_20261029_145128", "whole-body L1 4-D", "valve turning (not analysed here)", "--"),
    ("10-29", "flight_wb_l1_4d_valve_turning_20261029_145526", "whole-body L1 4-D", "valve turning (not analysed here)", "--"),
    ("10-29", "flight_wb_l1_4d_figure8_010_20261029_150625", "whole-body L1 4-D (+ 10-16 wrist-roll fix)", "figure-8 0.10 m/s, two runs", "WB-19, WB-20"),
    ("10-29", "flight_decoupled_l1_figure8_010_20261029_152543", "Geo-L1 (2026-10-01 tune)", "figure-8 0.10 m/s, two runs", "DEC-13, DEC-14"),
]

KEYMAP = {"platform_x_mm": ("base_pos", "rms_xyz", 0), "platform_y_mm": ("base_pos", "rms_xyz", 1),
          "platform_z_mm": ("base_pos", "rms_xyz", 2), "platform_roll_deg": ("att", "rms", 0),
          "platform_pitch_deg": ("att", "rms", 1), "platform_heading_deg": ("att", "rms", 2),
          "ee_x_mm": ("ee_pos", "rms_xyz", 0), "ee_y_mm": ("ee_pos", "rms_xyz", 1), "ee_z_mm": ("ee_pos", "rms_xyz", 2),
          "q1_deg": ("joint", "rms", 0), "q2_deg": ("joint", "rms", 1), "q3_deg": ("joint", "rms", 2),
          "q4_deg": ("joint", "rms", 3)}


def bag_dir(name):
    hits = glob.glob(os.path.join(BAGS, "*", "*", name))
    if len(hits) != 1:
        raise SystemExit(f"bag {name}: found {hits}")
    return hits[0]


# ---------------------------------------------------------------- stage 1 (ROS python, numpy 2)
def extract():
    os.makedirs(CACHE, exist_ok=True)
    for bag in sorted({r[5] for r in RUNS.values()}):
        out = os.path.join(CACHE, bag + ".npz")
        if os.path.isfile(out):
            continue
        cmd = (f"source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && "
               f"/usr/bin/python3 extract_bag.py '{bag_dir(bag)}' '{out}' && /usr/bin/python3 np1_compat.py '{out}'")
        print("extracting", bag, flush=True)
        subprocess.run(["bash", "-c", cmd], cwd=TOOLS, check=True, stdout=subprocess.DEVNULL)
    for key, r in RUNS.items():
        link = os.path.join(CACHE, key + ".npz")
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(r[5] + ".npz", link)


# ---------------------------------------------------------------- stage 2 (numpy 1.21 + scipy)
def score():
    os.environ["AM_NPZ"] = CACHE
    sys.path.insert(0, TOOLS)
    import common as C
    import metrics as M
    C.FLIGHTS.update({k: f"{v[0]} {v[1]}" for k, v in RUNS.items()})

    def windows(nm):
        d, t0 = C.load(nm)
        mk = "wbmode" if "wbmode__recv" in d.files else "gmode"
        md = C.edges(d, t0, mk)
        st = C.edges(d, t0, "pl_status")
        runs = [(t, st[i + 1][0] if i + 1 < len(st) else float(d[f"{mk}__recv"][-1] - t0))
                for i, (t, v) in enumerate(st) if v.startswith("EXECUTING T=") and float(v.split("=")[1].rstrip("s")) >= 10.0]
        c0, c1 = runs[RUNS[nm][6]]
        a = max(t for t, v in md if v == "DIRECT" and t < c0)
        b = next((t for t, v in md if v == "SAFETY" and t > c0), float(d[f"{mk}__recv"][-1] - t0))
        return a, b, c0, c1

    C.windows = windows
    M.windows = windows
    out = {}
    for key in RUNS:
        o, _ = M.analyse(key)
        m = o["metrics"]
        row = {c: float(m[g][f][i]) for c, (g, f, i) in KEYMAP.items()}
        row.update(ee_heading_deg=float(m["ee_head"]["rms"]), ee_pos_mm=float(m["ee_pos"]["rms_norm"]),
                   platform_pos_mm=float(m["base_pos"]["rms_norm"]), window=list(o["window"]),
                   tilt_max_deg=float(m["tilt_max_deg"]))
        out[key] = row
        print(f"{RUNS[key][0]:6s} window {o['window'][1] - o['window'][0]:6.2f} s  EE {row['ee_pos_mm']:6.1f} mm  "
              f"platform {row['platform_pos_mm']:6.1f} mm", flush=True)
    json.dump(out, open(os.path.join(CACHE, "scores.json"), "w"), indent=1)
    chosen = choose(out)
    for cell, keys in sorted(chosen.items()):
        for k, key in enumerate(keys, 1):
            export_mat(C, key, out[key], k, keys)
    export_summary(chosen, out)


SELECTION = (f"one of the two repeated one-lap tests averaged in its table entry: {RULE} "
             f"(2026-{FINAL_FROM} to {FINAL_TO})")
TABLE_COLS = ["platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
              "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg",
              "q4_deg", "platform_pos_mm", "ee_pos_mm"]


def eligible(key):
    """(eligible, reason): a completed run of the compared configuration and task, flown in the final campaign."""
    r = RUNS[key]
    if not r[7]:
        return False, r[8]
    if not FINAL_FROM <= r[1][:5] <= FINAL_TO:
        return False, EARLIER
    return True, ""


def in_flight_order(keys):
    return sorted(keys, key=lambda k: (RUNS[k][1], RUNS[k][6]))


def choose(sc):
    """(shape, 'v.vv', method) -> the runs averaged in that table entry, in flight order: every lap of the
    setting's latest eligible flight (bag). Chosen by flight time only, never by the error."""
    cells = {}
    for key, r in RUNS.items():
        if eligible(key)[0]:
            cells.setdefault((r[3], f"{r[4]:.2f}", r[2]), []).append(key)
    out = {}
    for c, ks in cells.items():
        latest = max(RUNS[k][1] for k in ks)                 # 'MM-DD HH:MM', one value per bag
        out[c] = in_flight_order([k for k in ks if RUNS[k][1] == latest])
    return out


def cell_mean(sc, keys):
    """The table entry: every column averaged over the runs, as the simulation table does."""
    return {c: sum(sc[k][c] for k in keys) / len(keys) for c in TABLE_COLS + ["tilt_max_deg"]}


def status(key, sc, chosen):
    ok, why = eligible(key)
    if not ok:
        return f"not eligible: {why}"
    r = RUNS[key]
    return ("**averaged**" if key in chosen.get((r[3], f"{r[4]:.2f}", r[2]), [])
            else "not averaged: an earlier flight of the same setting (the table uses the latest)")


def base_name(key):
    r = RUNS[key]
    return f"{r[2]}_{r[3]}_v{r[4]:.2f}".replace(".", "p")


def mat_name(key, k):
    return f"{base_name(key)}_run{k}"


def cell_dir(root, key):
    r = RUNS[key]
    return os.path.join(root, "free_flight_tracking", r[3], f"v{r[4]:.2f}".replace(".", "p"))


def _rows(a):
    """A numeric stream as a float matrix; rows that np1_compat joined into strings are split and NaN-padded."""
    import numpy as np
    a = np.asarray(a)
    if a.dtype.kind in "fiub":
        return a.astype(float)
    rows = [[float(x) for x in str(s).split(",") if x != ""] for s in a]
    out = np.full((len(rows), max((len(r) for r in rows), default=0)), np.nan)
    for i, r in enumerate(rows):
        out[i, :len(r)] = r
    return out


def export_mat(C, key, score_row, k, cell_keys):
    """One averaged run -> the simulation's `run` struct (meta, rmse, tracking, raw), same definitions; saved as
    run<k> of its setting (k = flight order among the runs averaged)."""
    import math
    import numpy as np
    import scipy.io
    from scipy.spatial.transform import Rotation as Rot
    r = RUNS[key]
    d, t0 = C.load(key)
    a, b, c0, c1 = C.windows(key)
    tu = np.arange(c0 + 0.1, c1 - 0.1, 0.01)
    to, P, Q, V, W = C.odom(d, t0)                     # Q: w x y z, sign-continuous
    tj, qj, qdj = C.joints_model(d, t0)                # model convention, j1..j4
    tr, ref = C.wbref(d, t0)
    Pu = C.interp(tu, to, P)
    Qu = C.quat_cont(C.interp(tu, to, Q)); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    qu = C.interp(tu, tj, qj)
    R = {k: C.interp(tu, tr, v) for k, v in ref.items()}
    xb, R0, _ = C.ref_base(R)
    xc, re, b1e, _ = C.fk_world(Pu, Qu, qu)
    Ra = Rot.from_quat(Qu[:, [1, 2, 3, 0]]).as_matrix()
    Rref = np.einsum("nij,jk->nik", R0, C.TP._Rz(0.5 * math.pi))
    att = np.degrees(Rot.from_matrix(np.einsum("nji,njk->nik", Rref, Ra)).as_rotvec())
    yaw = np.degrees(np.unwrap(np.arctan2(Ra[:, 1, 0], Ra[:, 0, 0])))
    yaw_ref = np.degrees(np.unwrap(np.arctan2(Rref[:, 1, 0], Rref[:, 0, 0])))
    az = np.degrees(np.unwrap(np.arctan2(b1e[:, 1], b1e[:, 0])))
    az_ref = np.degrees(np.unwrap(np.arctan2(R["b1_de"][:, 1], R["b1_de"][:, 0])))
    head_err = np.degrees(C.wrap(np.radians(az - az_ref)))
    e_pos, e_ee, e_com, e_q = Pu - xb, re - R["r_ed"], xc - R["x_cd"], np.degrees(qu - R["q_d"])
    tilt = np.degrees(np.arccos(np.clip(Ra[:, 2, 2], -1, 1)))
    rms = lambda x: np.sqrt(np.mean(np.asarray(x) ** 2, axis=0))   # noqa: E731
    rmse = dict(platform_pos_mm=float(1e3 * rms(np.linalg.norm(e_pos, axis=1))),
                platform_pos_xyz_mm=(1e3 * rms(e_pos)).tolist(),
                platform_roll_deg=float(rms(att[:, 0])), platform_pitch_deg=float(rms(att[:, 1])),
                platform_heading_deg=float(rms(att[:, 2])),
                ee_pos_mm=float(1e3 * rms(np.linalg.norm(e_ee, axis=1))), ee_pos_xyz_mm=(1e3 * rms(e_ee)).tolist(),
                ee_heading_deg=float(rms(head_err)), joints_deg=rms(e_q).tolist(),
                com_pos_mm=float(1e3 * rms(np.linalg.norm(e_com, axis=1))),
                ee_pos_max_mm=float(1e3 * np.linalg.norm(e_ee, axis=1).max()), tilt_max_deg=float(tilt.max()))
    # the export must reproduce the table's numbers exactly (same helpers, same grid)
    for c in ("platform_pos_mm", "ee_pos_mm", "ee_heading_deg", "platform_heading_deg"):
        if abs(rmse[c] - score_row[c]) > 1e-6:
            raise SystemExit(f"{r[0]}: exported {c} {rmse[c]} != scored {score_row[c]}")

    def edges_struct(k):
        e = C.edges(d, t0, k)
        return dict(t=np.array([x[0] for x in e]), label=np.array([x[1] for x in e], dtype=object))

    dbg_key = "wb" if r[2] == "whole_body_l1" else "l1"
    mot_key = "motors" if r[2] == "whole_body_l1" else "gmotors"
    mode_key = "wbmode" if r[2] == "whole_body_l1" else "gmode"
    raw = dict(
        odom=dict(t=to, pos_m=P, vel_mps=V, quat_xyzw=Q[:, [1, 2, 3, 0]], angvel_rad_s=W),
        joints=dict(t=tj, q_rad=qj, qdot_rad_s=qdj),
        reference=dict(t=tr, x_cd=ref["x_cd"], x_cd_dot=ref["x_cd_dot"], x_cd_ddot=ref["x_cd_ddot"],
                       b1_d_model=ref["b1_d"], b1_d_dot_model=ref["b1_d_dot"], r_ed=ref["r_ed"],
                       r_ed_dot=ref["r_ed_dot"], b1_de=ref["b1_de"], q_d_rad=ref["q_d"], qdot_d_rad_s=ref["qdot_d"]),
        events=dict(mode=edges_struct(mode_key), planner_status=edges_struct("pl_status")))
    if f"{dbg_key}__data" in d.files:
        raw["controller_debug"] = dict(t=d[f"{dbg_key}__recv"] - t0, data=_rows(d[f"{dbg_key}__data"]))
    if f"{mot_key}__control" in d.files:
        raw["motors"] = dict(t=d[f"{mot_key}__recv"] - t0, control=_rows(d[f"{mot_key}__control"]))
    if "batt__voltage_v" in d.files:
        raw["battery"] = dict(t=d["batt__recv"] - t0, voltage_v=d["batt__voltage_v"].astype(float),
                              current_a=d["batt__current_a"].astype(float))
    if "vrpn__recv" in d.files:
        raw["mocap_vrpn"] = dict(t=d["vrpn__recv"] - t0,
                                 pos_m=np.column_stack([d[f"vrpn__pose.position.{a}"] for a in "xyz"]),
                                 quat_xyzw=np.column_stack([d[f"vrpn__pose.orientation.{a}"] for a in "xyzw"]))
    meta = dict(name=mat_name(key, k), tag=r[0], run=k, runs_in_mean=len(cell_keys),
                averaged_with=", ".join(RUNS[x][0] for x in cell_keys), method=r[2], method_long=METHOD_LONG[r[2]], shape=r[3],
                mean_speed_nominal_mps=r[4], flown=f"2026-{r[1]}", bag=r[5], run_in_bag=r[6] + 1,
                run_start_s=c0, run_end_s=c1, direct_enter_s=a,
                vehicle="T650 aerial manipulator, total mass 3.746 kg", feedback="EKF2-fused OptiTrack odometry",
                trajectory=("circle, radius 0.50 m" if r[3] == "circle" else "figure-8, 1.40 x 0.70 m, long axis on world x")
                + ", q2 = 25 +- 15 deg four cycles per lap, fold 55 deg, one lap",
                clock="raw streams: s since the bag's first message; tracking.t: s since the run started",
                controller_config=CONFIGS[r[9]],
                selection=SELECTION)
    tracking = dict(
        t=tu - c0, platform_pos_m=Pu, platform_pos_ref_m=xb, platform_pos_err_m=e_pos,
        platform_quat_xyzw=Qu[:, [1, 2, 3, 0]], platform_att_err_deg=att,
        platform_heading_deg=yaw, platform_heading_ref_deg=yaw_ref, platform_tilt_deg=tilt,
        com_pos_m=xc, com_pos_ref_m=R["x_cd"], com_pos_err_m=e_com,
        ee_pos_m=re, ee_pos_ref_m=R["r_ed"], ee_pos_err_m=e_ee,
        ee_heading_deg=az, ee_heading_ref_deg=az_ref, ee_heading_err_deg=head_err,
        q_deg=np.degrees(qu), q_ref_deg=np.degrees(R["q_d"]), q_err_deg=e_q,
        columns="platform_att_err_deg = [roll pitch heading] (body axes); xyz columns are world ENU")
    path = os.path.join(cell_dir(MAT_DIR, key), mat_name(key, k) + ".mat")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    scipy.io.savemat(path, {"run": dict(meta=meta, rmse=rmse, tracking=tracking, raw=raw)},
                     do_compression=True, long_field_names=True, oned_as="column")
    print("wrote", os.path.relpath(path, RESULTS), flush=True)


def export_summary(chosen, sc):
    """free_flight_tracking/free_flight_tracking_rmse.mat, the simulation's layout: `rmse_mean` = the table (one row
    per setting and method, mean over its runs), `rmse_runs` = every averaged run; plus `rmse_all` = every scored run
    of the comparison with its status (struct2table in MATLAB)."""
    import numpy as np
    import scipy.io

    def colstruct(rows):
        out = {}
        for c in rows[0]:
            v = [x[c] for x in rows]
            out[c] = np.array(v, dtype=object) if isinstance(v[0], str) else np.array(v, dtype=float)
        return out

    def run_row(key):
        r = RUNS[key]
        row = dict(tag=r[0], flown=f"2026-{r[1]}", shape=r[3], mean_speed_mps=r[4], method=r[2], bag=r[5],
                   run_in_bag=r[6] + 1)
        row.update({c: sc[key][c] for c in TABLE_COLS + ["tilt_max_deg"]})
        return row

    runs, means = [], []
    for cell in sorted(chosen):
        keys = chosen[cell]
        for k, key in enumerate(keys, 1):
            runs.append(dict(name=mat_name(key, k), run=k, **run_row(key)))
        m = dict(shape=cell[0], mean_speed_mps=float(cell[1]), method=cell[2], runs=len(keys),
                 tags=", ".join(RUNS[x][0] for x in keys))
        m.update(cell_mean(sc, keys))
        means.append(m)
    every = [dict(status=status(key, sc, chosen).strip("*"), **run_row(key)) for key in RUNS]
    path = os.path.join(FF_MAT, "free_flight_tracking_rmse.mat")
    os.makedirs(FF_MAT, exist_ok=True)
    scipy.io.savemat(path, {"rmse_mean": colstruct(means), "rmse_runs": colstruct(runs), "rmse_all": colstruct(every)},
                     oned_as="column")
    print("wrote", os.path.relpath(path, RESULTS), flush=True)


# ---------------------------------------------------------------- outputs
def wb_config_text(keys):
    """The averaged whole-body runs grouped by configuration, as text: (with the 10-16 fix, without)."""
    out = []
    for cfg in ("wb_1016", "wb_0927"):
        ks = in_flight_order([k for k in keys if RUNS[k][2] == "whole_body_l1" and RUNS[k][9] == cfg])
        out.append(", ".join(RUNS[k][0] for k in ks) or "none")
    return out[0], out[1]


def bag_groups(keys):
    """original bag -> (copy folder, the runs of that bag that are averaged)."""
    groups = {}
    for key in in_flight_order(keys):
        r = RUNS[key]
        if r[5] not in groups:
            tags = "+".join(RUNS[x][0] for x in in_flight_order([x for x in RUNS if RUNS[x][5] == r[5]]))
            groups[r[5]] = [os.path.join(cell_dir(FLIGHT, key), f"{base_name(key)}_{tags}"), []]
        groups[r[5]][1].append(key)
    return groups


def copy_bags(keys):
    for bag, (dst, _) in sorted(bag_groups(keys).items()):
        src = bag_dir(bag)
        if os.path.isdir(dst) and sorted(os.listdir(dst)) == sorted(os.listdir(src)) and all(
                os.path.getsize(os.path.join(dst, f)) == os.path.getsize(os.path.join(src, f)) for f in os.listdir(src)):
            continue
        if os.path.isdir(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
        print("copied", bag, "->", os.path.relpath(dst, RESULTS), flush=True)


def prune(chosen, keys):
    """Remove run files and bag copies under the two free_flight_tracking trees that no longer belong to the table
    (an earlier selection's). The original bags in docs/experimental_data_ros2_bag are never touched."""
    keep_mat = {os.path.join(cell_dir(MAT_DIR, key), mat_name(key, k) + ".mat")
                for cell in chosen for k, key in enumerate(chosen[cell], 1)}
    keep_bag = {dst for dst, _ in bag_groups(keys).values()}
    for f in glob.glob(os.path.join(FF_MAT, "*", "*", "*.mat")):
        if f not in keep_mat:
            os.remove(f)
            print("removed", os.path.relpath(f, RESULTS), flush=True)
    for d in glob.glob(os.path.join(BAG_COPIES, "*", "*", "*")):
        if os.path.isdir(d) and d not in keep_bag:
            shutil.rmtree(d)
            print("removed", os.path.relpath(d, RESULTS), flush=True)
    stale = os.path.join(MAT_DIR, "free_flight_tracking_rmse.mat")     # the summary's location before 2026-10-29
    if os.path.isfile(stale):
        os.remove(stale)
        print("removed", os.path.relpath(stale, RESULTS), flush=True)


def copy_configs():
    """free_flight_tracking/configs/ <- each config at the last commit that changed it (the one flown in the final
    campaign); returns [(name, source)]."""
    os.makedirs(os.path.join(FF_MAT, "configs"), exist_ok=True)
    out = []
    for dst, repo, path in CONFIG_FILES:
        rev = subprocess.run(["git", "-C", repo, "log", "-1", "--format=%h %ad", "--date=short", "--", path],
                             check=True, capture_output=True, text=True).stdout.split()
        text = subprocess.run(["git", "-C", repo, "show", f"{rev[0]}:{path}"], check=True, capture_output=True,
                              text=True).stdout
        with open(os.path.join(FF_MAT, "configs", dst), "w") as f:
            f.write(f"# {os.path.basename(repo)} {rev[0]} ({rev[1]}): {path}, unchanged through the final campaign\n"
                    + text)
        out.append((dst, f"{os.path.basename(repo)} {rev[0]} ({rev[1]})"))
    return out


def write_outputs(paper, copy=False):
    sys.path.insert(0, HERE)
    import make_latex_table as T
    import readme_text as RT
    sc = json.load(open(os.path.join(CACHE, "scores.json")))
    chosen = choose(sc)
    keys = [k for ks in chosen.values() for k in ks]
    means = {cell: cell_mean(sc, ks) for cell, ks in chosen.items()}
    tex = T.exp_table(means)
    os.makedirs(T.TABLES, exist_ok=True)
    open(T.EXP_TEX, "w").write(tex)
    os.makedirs(FF_MAT, exist_ok=True)
    open(os.path.join(FF_MAT, os.path.basename(T.EXP_TEX)), "w").write(tex)   # travels with the MATLAB data
    if paper:
        T.splice(paper, tex, T.EXP_LABEL, anchor="\\subsection{Experiment}\n\\subsubsection{Free Flight: Trajectory Tracking}")
        print("updated", paper)
    if copy:
        copy_bags(keys)
    prune(chosen, keys)
    configs = copy_configs()

    name = {"whole_body_l1": "Proposed (whole-body L1 4-D)", "geometric_l1": "Geo-L1"}
    shape = {"circle": "circle", "figure8": "figure-8"}
    cells_rows, trend = [], {}
    for sh in ("circle", "figure8"):
        for v in (0.10, 0.13, 0.20):
            row = []
            for m in ("whole_body_l1", "geometric_l1"):
                ks = chosen.get((sh, f"{v:.2f}", m))
                row.append(f"{means[(sh, f'{v:.2f}', m)]['ee_pos_mm']:.1f} ({', '.join(RUNS[k][0] for k in ks)})"
                           if ks else "not flown")
                if ks:
                    trend.setdefault((sh, m), []).append((v, means[(sh, f"{v:.2f}", m)]["ee_pos_mm"]))
            cells_rows.append([shape[sh], f"{v:.2f}"] + row)
    not_flown = [f"{['Proposed', 'Geo-L1'][i]} {r[0]} {r[1]} m/s" for r in cells_rows for i in (0, 1)
                 if r[2 + i] == "not flown"]
    trend_rows = []
    for (sh, m), pts in sorted(trend.items()):
        vals = [e for _, e in sorted(pts)]
        trend_rows.append([shape[sh], RT.METHOD_NAMES[m][0], " < ".join(f"{e:.1f}" for e in vals) if all(
            a < b for a, b in zip(vals, vals[1:])) else " / ".join(f"{e:.1f}" for e in vals),
            "yes" if all(a < b for a, b in zip(vals, vals[1:])) else "**no**"])
    scored = [[RUNS[k][0], f"2026-{RUNS[k][1]}", RT.METHOD_NAMES[RUNS[k][2]][0], shape[RUNS[k][3]], f"{RUNS[k][4]:.2f}",
               f"{sc[k]['ee_pos_mm']:.2f}", f"{sc[k]['platform_pos_mm']:.2f}", f"{sc[k]['ee_heading_deg']:.2f}",
               CONFIG_SHORT[RUNS[k][9]], status(k, sc, chosen)] for k in RUNS]
    files = []
    groups = bag_groups(keys)
    for cell in sorted(chosen, key=lambda c: (c[0], c[1], c[2])):
        for k, key in enumerate(chosen[cell], 1):
            r = RUNS[key]
            n = sum(1 for x in RUNS.values() if x[5] == r[5])
            vt = f"v{r[4]:.2f}".replace(".", "p")
            files.append([f"`{r[3]}/{vt}/{mat_name(key, k)}.mat`", r[0], f"2026-{r[1]}", f"`{r[5]}`",
                          f"{['first', 'second', 'third', 'fourth'][r[6]]} of {n}" if n > 1 else "only run",
                          f"`{os.path.basename(groups[r[5]][0])}/`", f"{sc[key]['ee_pos_mm']:.2f}"])
    index_rows = [[i, d, f"`{b}`", c, w, rr] for i, (d, b, c, w, rr) in enumerate(INDEX, 1)]
    fix_runs, nofix_runs = wb_config_text(keys)
    ctx = dict(files=files, cells=cells_rows, trend=trend_rows, scored=scored, not_flown=not_flown,
               fix_runs=fix_runs, nofix_runs=nofix_runs, configs=configs, final=(FINAL_FROM, FINAL_TO),
               example=next(f[0].strip("`") for f in files if "whole_body_l1_circle" in f[0]))
    os.makedirs(FF_MAT, exist_ok=True)
    with open(os.path.join(FF_MAT, "README.md"), "w") as f:
        f.write(RT.exp_readme(ctx))
    # MAT_DIR/README.md (the task index + the results summary for the manuscript) is written by hand: not touched here
    print("wrote", os.path.join(FF_MAT, "README.md"))

    L = ["# Aerial-manipulator flight tests (ros2 bags)", "",
         "Every aerial-manipulator flight-test bag in `docs/experimental_data_ros2_bag/`, and which runs fill the paper's",
         "experimental free-flight trajectory-tracking table (`tab:exp_free_flight_tracking`).",
         "Not listed: the robotic-arm calibration bench bags (09-09, 09-11), the bare-T650 recordings (08-06, 08-07), the",
         "valve free-spin mocap recordings (`mocap_valve_spin_20261023_174910`, `mocap_valve_spin_20261027_183950`) and the",
         "analysis folder `flight_wb_l1_4d_pick_and_drop_20261014_172845_analysis`.", "",
         "Regenerate (re-scores from the bags, updates this file, the MATLAB data and the table):", "",
         "```bash", "/usr/bin/python3 results/utils/experiment_tracking.py --copy-bags [--paper <paper>/main.tex]", "```", "",
         "## 1. All flight-test bags", "",
         "| # | date | bag | controller | flown | scored runs |", "|---|---|---|---|---|---|"]
    for i, (date, bag, ctrl, what, runs) in enumerate(INDEX, 1):
        L.append(f"| {i} | {date} | `{bag}` | {ctrl} | {what} | {runs} |")
    L += ["", "Bag folders: `<date> - T650-AM .../<date> - T650-AM .../<bag>/` under `docs/experimental_data_ros2_bag/`; the",
          "10-21 bags sit one level deeper, in the sub-folder written before the bag name. The 10-24 and 10-27 box-pushing",
          "bag names carry the mass typed at the time (240 / 300 g); the box weighed 280 g in all of them except the 510 g",
          "flight (`..._500g_20261024_195657`). Last column: the run's tag in section 2, or the task folder it is exported to",
          "(`flight_test_data/<task>/`, `matlab_experiment_data/<task>/`).", "",
          "## 2. Scored runs of the formal free-flight comparison", "",
          "Each run is scored over its whole planned trajectory (speed-up and slow-down included), with the same",
          f"definitions as the simulation table. **Rule:** each table entry is the mean of two repeated one-lap tests,",
          f"{RULE} (2026-{FINAL_FROM} to {FINAL_TO}); each lap is its own planner run from rest to rest.",
          "The rule never looks at the error. **averaged** = in the table.", "",
          "| run | date | method | trajectory | speed [m/s] | EE position RMSE [mm] | platform position RMSE [mm] | EE heading RMSE [deg] | configuration | status |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for row in scored:
        L.append("| " + " | ".join(row[:4] + [row[4], row[5], row[6], row[7], row[8], row[9]]) + " |")
    L += ["", "Configurations (fsc_autopilot_ros2 hardware yamls; the 10-16 fix sets the EE-heading gains and freezes the",
          "observer rows of a stuck wrist-roll joint, nothing else in the law changed):", ""]
    L += [f"- **{CONFIG_SHORT[k]}:** {v}." for k, v in CONFIGS.items()]
    L += ["", "## 3. The experimental table: the runs averaged in each entry (EE position RMSE, mean, mm)", "",
          "| trajectory | speed [m/s] | Proposed | Geo-L1 |", "|---|---|---|---|"]
    L += [f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} |" for r in cells_rows]
    L += ["", "Does the mean EE position RMSE grow with the mean speed?", "",
          "| trajectory | method | 0.10 / 0.13 / 0.20 m/s [mm] | grows |", "|---|---|---|---|"]
    L += [f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} |" for r in trend_rows]
    L += ["", "## 4. The data in results/experiment_results", "",
          "- `matlab_experiment_data/free_flight_tracking/`: self-contained for MATLAB (its own README.md): one `.mat`",
          "  per averaged run, `<method>_<trajectory>_v<speed>_run<k>.mat` as in the simulation, the summary",
          "  `free_flight_tracking_rmse.mat` and the flown configs.",
          "- `flight_test_data/free_flight_tracking/<trajectory>/v<speed>/<method>_<trajectory>_v<speed>_<runs of the bag>/`:",
          "  each bag that holds an averaged run, copied unchanged (`ros2 bag info` works on the folder).", "",
          "| file | run | flown | original bag | run in the bag | bag copy | EE RMSE [mm] |", "|---|---|---|---|---|---|---|"]
    L += ["| " + " | ".join(r) + " |" for r in files]
    L += [""]
    open(OUT_MD, "w").write("\n".join(L))
    print("wrote", OUT_MD)
    for cell in sorted(chosen):
        print(cell, "->", ", ".join(RUNS[k][0] for k in chosen[cell]), f"mean EE {means[cell]['ee_pos_mm']:.2f} mm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--paper", default=None)
    ap.add_argument("--stage2", action="store_true")
    ap.add_argument("--copy-bags", action="store_true", help="copy the selected runs' bags into experiment_results/flight_test_data/free_flight_tracking")
    a = ap.parse_args()
    if a.stage2:
        score()
        return
    extract()
    env = dict(os.environ, PYTHONNOUSERSITE="1", AM_EXP_CACHE=CACHE)
    subprocess.run(["/usr/bin/python3", os.path.abspath(__file__), "--stage2"], env=env, check=True)
    write_outputs(a.paper, a.copy_bags)


if __name__ == "__main__":
    main()
