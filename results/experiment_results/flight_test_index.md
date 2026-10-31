# Aerial-manipulator flight tests (ros2 bags)

Every aerial-manipulator flight-test bag in `docs/experimental_data_ros2_bag/`, and which runs fill the paper's
experimental free-flight trajectory-tracking table (`tab:exp_free_flight_tracking`).
Not listed: the robotic-arm calibration bench bags (09-09, 09-11), the bare-T650 recordings (08-06, 08-07), the
valve free-spin mocap recordings (`mocap_valve_spin_20261023_174910`, `mocap_valve_spin_20261027_183950`) and the
analysis folder `flight_wb_l1_4d_pick_and_drop_20261014_172845_analysis`.

Regenerate (re-scores from the bags, updates this file, the MATLAB data and the table):

```bash
/usr/bin/python3 results/utils/experiment_tracking.py --copy-bags [--paper <paper>/main.tex]
```

## 1. All flight-test bags

| # | date | bag | controller | flown | scored runs |
|---|---|---|---|---|---|
| 1 | 09-12 | `flight_wb_l1_20260912_190224` | whole-body L1 (6-D observer) | first hardware flight of the whole-body law, short | -- |
| 2 | 09-12 | `flight_wb_l1_20260912_201430` | whole-body L1 (6-D observer) | hover and arm moves in DIRECT (development) | -- |
| 3 | 09-18 | `flight_wb_l1_4d_20260918_144828` | whole-body L1 4-D | first 4-D flight: planner legs (EE excursions, go-home, 0.61 m base step) | -- |
| 4 | 09-18 | `flight_wb_l1_4d_20260918_152112` | whole-body L1 4-D | metadata only, no data file | -- |
| 5 | 09-21 | `flight_wb_l1_4d_circle_20260921_112328` | whole-body L1 4-D | circle attempt (development) | -- |
| 6 | 09-21 | `flight_wb_l1_4d_circle_20260921_115207` | whole-body L1 4-D | circle attempt (development) | -- |
| 7 | 09-21 | `flight_wb_l1_4d_circle_20260921_123637` | whole-body L1 4-D | circle attempt: go-to-start only, mocap flips | -- |
| 8 | 09-24 | `flight_wb_l1_4d_circle_20260924_120228` | whole-body L1 4-D | circle, earlier tune, arm q2 30 +- 10 deg / 48 s; mocap freeze mid-run | -- |
| 9 | 09-24 | `flight_wb_l1_4d_circle_20260924_120546` | whole-body L1 4-D | circle, earlier tune, arm q2 30 +- 10 deg / 48 s | -- |
| 10 | 09-24 | `flight_wb_l1_4d_circle_20260924_165654` | whole-body L1 4-D | circle, earlier tune, arm q2 25 +- 15 deg / 12 s | -- |
| 11 | 09-24 | `flight_wb_l1_4d_circle_20260924_172101` | whole-body L1 4-D | circle, earlier tune, arm q2 25 +- 15 deg / 6 s | -- |
| 12 | 09-28 | `flight_wb_l1_4d_circle_20260928_171154` | whole-body L1 4-D | circle 0.13 m/s | WB-1 |
| 13 | 09-28 | `flight_wb_l1_4d_circle_20260928_172242` | whole-body L1 4-D | circle 0.13 m/s | WB-2 |
| 14 | 09-28 | `flight_decoupled_l1_circle_20260928_182835` | Geo-L1 (previous gains) | circle 0.13 m/s | DEC-1 |
| 15 | 09-28 | `flight_decoupled_l1_circle_20260928_183608` | Geo-L1 (previous gains) | circle 0.13 m/s | DEC-2 |
| 16 | 09-29 | `flight_wb_l1_4d_ps4_20260929_160027` | whole-body L1 4-D | PS4 teleoperation | -- |
| 17 | 10-02 | `flight_decoupled_l1_circle_20261002_134247` | Geo-L1 (2026-10-01 tune) | circle 0.13 m/s, two runs | DEC-3, DEC-4 |
| 18 | 10-02 | `flight_decoupled_l1_circle_ps420261002_134830` | Geo-L1 (2026-10-01 tune) | PS4 teleoperation | -- |
| 19 | 10-05 | `flight_wb_l1_4d_figure8_vel010_20261005_162406` | whole-body L1 4-D | figure-8 0.10 m/s, two runs | WB-3, WB-4 |
| 20 | 10-05 | `flight_decoupled_l1_figure8_vel010_20261005_163734` | Geo-L1 (2026-10-01 tune) | figure-8 0.10 m/s | DEC-5 |
| 21 | 10-05 | `flight_decoupled_l1_figure8_vel013_20261005_165431` | Geo-L1 (2026-10-01 tune) | figure-8 0.13 m/s | DEC-6 |
| 22 | 10-05 | `flight_wb_l1_4d_figure8_vel013_20261005_170129` | whole-body L1 4-D | figure-8 0.13 m/s, two runs | WB-5, WB-6 |
| 23 | 10-09 | `flight_wb_l1_4d_pick_and_place_20261009_172557` | whole-body L1 4-D | pick-and-place PP-1: the six-leg mission without a payload, completed | -- |
| 24 | 10-09 | `flight_wb_l1_4d_pick_and_place_20261009_180228` | whole-body L1 4-D | pick-and-place PP-2: basket picked, placed 166 mm short; PX4 link froze in flight | -- |
| 25 | 10-09 | `flight_wb_l1_4d_pick_and_place_20261009_181817` | whole-body L1 4-D | pick-and-place PP-3: gripper closed above the arch, nothing lifted | -- |
| 26 | 10-10 | `flight_wb_l1_4d_pick_and_place_20261010_190719` | whole-body L1 4-D | pick-and-place PP-4: set-down pulled the basket off the hat | -- |
| 27 | 10-14 | `flight_wb_l1_4d_pick_and_drop_20261014_172845` | whole-body L1 4-D | pick-and-drop PD-1, no payload, over the full bin; ABORT at the drop hold | -- |
| 28 | 10-14 | `flight_wb_l1_4d_pick_and_drop_20261014_183914` | whole-body L1 4-D | pick-and-drop PD-2, no payload, over the cut tray; operator revert before the drop | -- |
| 29 | 10-15 | `flight_wb_l1_4d_pick_and_drop_empty_20261015_172318` | whole-body L1 4-D | pick-and-drop E-1, no payload, completed | -- |
| 30 | 10-15 | `flight_wb_l1_4d_pick_and_drop_20261015_175059` | whole-body L1 4-D | pick-and-drop D-1, basket dropped, completed | -- |
| 31 | 10-15 | `flight_wb_l1_4d_pick_and_drop_20261015_180850` | whole-body L1 4-D | pick-and-drop D-2, basket pulled out of the jaws in the pick climb | -- |
| 32 | 10-16 | `flight_wb_l1_4d_pick_and_drop_empty_20261016_134438` | whole-body L1 4-D | pick-and-drop E-1, no payload, completed | -- |
| 33 | 10-16 | `flight_wb_l1_4d_pick_and_drop_20261016_135100` | whole-body L1 4-D | pick-and-drop D-1, completed; wrong grip, basket rest 112 mm off | -- |
| 34 | 10-16 | `flight_wb_l1_4d_pick_and_drop_20261016_140020` | whole-body L1 4-D | pick-and-drop D-2, completed, basket flat | pick-and-place D-2 |
| 35 | 10-19 | `flight_wb_l1_4d_pick_and_drop_20261019_170849` | whole-body L1 4-D | pick-and-drop: PX4 link cut in the pick's lift, vehicle flipped | -- |
| 36 | 10-19 | `flight_wb_l1_4d_pick_and_drop_20261019_172008` | whole-body L1 4-D | hover (link check) | -- |
| 37 | 10-20 | `flight_wb_l1_4d_pick_and_drop_20261020_182234` | whole-body L1 4-D | pick-and-drop, completed, no link loss | -- |
| 38 | 10-21 | `stale/flight_wb_l1_4d_box_pushing_20261021_161745` | whole-body L1 4-D | box pushing: Plan refused (box off the configured table) | -- |
| 39 | 10-21 | `stale/flight_wb_l1_4d_box_pushing_20261021_162604` | whole-body L1 4-D | box pushing: Plan refused (no metadata.yaml) | -- |
| 40 | 10-21 | `free_flight/flight_wb_l1_4d_box_pushing_20261021_164338` | whole-body L1 4-D | box pushing: go to start and hover over a typed box point, land | -- |
| 41 | 10-21 | `stale/flight_wb_l1_4d_box_pushing_20261021_165311` | whole-body L1 4-D | box pushing: Plan refused (box off the configured table) | -- |
| 42 | 10-21 | `box_pushing/flight_wb_l1_4d_box_pushing_20261021_170123` | whole-body L1 4-D | first box push, 280 g, 0.10 m: pushed; q3-guard abort at the exit | -- |
| 43 | 10-23 | `flight_wb_l1_4d_box_pushing_20261023_172853` | whole-body L1 4-D | box pushing: q3 rate guard aborted at the grasp | -- |
| 44 | 10-24 | `flight_wb_l1_4d_box_pushing_240g_20261024_150423` | whole-body L1 4-D | box pushing #1 (280 g): pushed ~70 mm, box-tilt abort | -- |
| 45 | 10-24 | `flight_wb_l1_4d_box_pushing_240g_20261024_151159` | whole-body L1 4-D | box pushing #2 (280 g): missed the fin, q3 rate guard | -- |
| 46 | 10-24 | `flight_wb_l1_4d_box_pushing_240g_20261024_151634` | whole-body L1 4-D | box pushing #3 (280 g): box blown off the table in go to start | -- |
| 47 | 10-24 | `flight_wb_l1_4d_box_pushing_240g_20261024_162850` | whole-body L1 4-D | box pushing A (280 g): q3 rate guard in the release | -- |
| 48 | 10-24 | `flight_wb_l1_4d_box_pushing_240g_20261024_163413` | whole-body L1 4-D | box pushing B (280 g): q3 rate guard at the grasp | -- |
| 49 | 10-24 | `flight_wb_l1_4d_box_pushing_240g_20261024_164419` | whole-body L1 4-D | box pushing C (280 g): q3 rate guard at the grasp | -- |
| 50 | 10-24 | `flight_wb_l1_4d_box_pushing_280g_20261024_194310` | whole-body L1 4-D | box pushing 280 g, 0.50 m: box at its plan, box-tilt abort | -- |
| 51 | 10-24 | `flight_wb_l1_4d_box_pushing_500g_20261024_195657` | whole-body L1 4-D | box pushing 510 g: arm buckled at the grasp, watchdog to SAFETY | -- |
| 52 | 10-27 | `flight_wb_l1_4d_box_pushing_300g_20261027_171114` | whole-body L1 4-D | box pushing 280 g, 0.40 m: push completed, q3-guard abort after the release | box-pushing BP-1 |
| 53 | 10-27 | `flight_wb_l1_4d_box_pushing_300g_20261027_172227` | whole-body L1 4-D | box pushing 280 g, 0.40 m: q3 guard aborted at 348 mm | box-pushing BP-2 |
| 54 | 10-27 | `flight_wb_l1_4d_circle_020_20261027_175015` | whole-body L1 4-D (+ 10-16 wrist-roll fix) | circle 0.20 m/s, two runs | WB-7, WB-8 |
| 55 | 10-27 | `flight_wb_l1_4d_figure8_020_20261027_175849` | whole-body L1 4-D (+ 10-16 wrist-roll fix) | figure-8 0.20 m/s, two runs | WB-9, WB-10 |
| 56 | 10-27 | `flight_wb_l1_4d_circle_010_20261027_180659` | whole-body L1 4-D (+ 10-16 wrist-roll fix) | circle 0.10 m/s, two runs | WB-11, WB-12 |
| 57 | 10-27 | `flight_decoupled_l1_circle_010_20261027_181331` | Geo-L1 (2026-10-01 tune) | circle 0.10 m/s, two runs | DEC-7, DEC-8 |
| 58 | 10-28 | `flight_wb_l1_4d_circle_013_20261028_144602` | whole-body L1 4-D (+ 10-16 wrist-roll fix) | circle 0.13 m/s, two runs | WB-13, WB-14 |
| 59 | 10-28 | `flight_wb_l1_4d_circle_013_20261028_150821` | whole-body L1 4-D (+ 10-16 wrist-roll fix) | circle 0.13 m/s, two runs | WB-15, WB-16 |
| 60 | 10-28 | `flight_decoupled_l1_circle_013_20261028_151600` | Geo-L1 (2026-10-01 tune) | circle 0.13 m/s, two runs | DEC-9, DEC-10 |
| 61 | 10-28 | `flight_wb_l1_4d_figure8_013_20261028_152202` | whole-body L1 4-D (+ 10-16 wrist-roll fix) | figure-8 0.13 m/s, two runs | WB-17, WB-18 |
| 62 | 10-28 | `flight_decoupled_l1_figure8_013_20261028_152811` | Geo-L1 (2026-10-01 tune) | figure-8 0.13 m/s, two runs | DEC-11, DEC-12 |
| 63 | 10-28 | `flight_wb_l1_4d_pick_and_drop_12offset_20261028_161640` | whole-body L1 4-D | pick-and-drop M12, Drop Margin 12 cm: completed; the basket bounced off the tray onto the floor | -- |
| 64 | 10-28 | `flight_wb_l1_4d_pick_and_drop_8offset_20261028_162711` | whole-body L1 4-D | pick-and-drop M8, Drop Margin 8 cm: completed; basket in the tray, tilted 13 deg | -- |
| 65 | 10-28 | `flight_wb_l1_4d_pick_and_drop_6offset_20261028_164816` | whole-body L1 4-D | pick-and-drop M6, Drop Margin 6 cm: completed; basket in the tray, slid 66 mm | -- |
| 66 | 10-28 | `flight_wb_l1_4d_pick_and_drop_5offset_20261028_165603` | whole-body L1 4-D | pick-and-drop M5, Drop Margin 5 cm: completed; basket flat in the tray | pick-and-drop M5 |
| 67 | 10-28 | `rosbag2_2026_10_28-18_06_52_Valve_Turning` | whole-body L1 4-D | valve turning (not analysed here) | -- |
| 68 | 10-28 | `rosbag2_2026_10_28-18_18_33_Valve_Turning` | whole-body L1 4-D | valve turning (not analysed here) | -- |
| 69 | 10-29 | `flight_wb_l1_4d_valve_turning_20261029_145128` | whole-body L1 4-D | valve turning (not analysed here) | -- |
| 70 | 10-29 | `flight_wb_l1_4d_valve_turning_20261029_145526` | whole-body L1 4-D | valve turning (not analysed here) | -- |
| 71 | 10-29 | `flight_wb_l1_4d_figure8_010_20261029_150625` | whole-body L1 4-D (+ 10-16 wrist-roll fix) | figure-8 0.10 m/s, two runs | WB-19, WB-20 |
| 72 | 10-29 | `flight_decoupled_l1_figure8_010_20261029_152543` | Geo-L1 (2026-10-01 tune) | figure-8 0.10 m/s, two runs | DEC-13, DEC-14 |

Bag folders: `<date> - T650-AM .../<date> - T650-AM .../<bag>/` under `docs/experimental_data_ros2_bag/`; the
10-21 bags sit one level deeper, in the sub-folder written before the bag name. The 10-24 and 10-27 box-pushing
bag names carry the mass typed at the time (240 / 300 g); the box weighed 280 g in all of them except the 510 g
flight (`..._500g_20261024_195657`). Last column: the run's tag in section 2, or the task folder it is exported to
(`flight_test_data/<task>/`, `matlab_experiment_data/<task>/`).

## 2. Scored runs of the formal free-flight comparison

Each run is scored over its whole planned trajectory (speed-up and slow-down included), with the same
definitions as the simulation table. **Rule:** each table entry is the mean of two repeated one-lap tests,
both laps of the setting's latest flight in the final campaign (2026-10-27 to 10-29); each lap is its own planner run from rest to rest.
The rule never looks at the error. **averaged** = in the table.

| run | date | method | trajectory | speed [m/s] | EE position RMSE [mm] | platform position RMSE [mm] | EE heading RMSE [deg] | configuration | status |
|---|---|---|---|---|---|---|---|---|---|
| WB-1 | 2026-09-28 17:11 | Proposed | circle | 0.13 | 26.25 | 25.35 | 0.48 | 09-27 tune | not eligible: before the final campaign (2026-10-27 to 10-29) |
| WB-2 | 2026-09-28 17:22 | Proposed | circle | 0.13 | 24.37 | 23.43 | 0.44 | 09-27 tune | not eligible: before the final campaign (2026-10-27 to 10-29) |
| DEC-1 | 2026-09-28 18:28 | Geo-L1 | circle | 0.13 | 192.05 | 151.29 | 10.31 | previous gains | not eligible: previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune |
| DEC-2 | 2026-09-28 18:36 | Geo-L1 | circle | 0.13 | 218.44 | 176.84 | 10.31 | previous gains | not eligible: previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune |
| DEC-3 | 2026-10-02 13:42 | Geo-L1 | circle | 0.13 | 77.45 | 66.19 | 3.48 | 10-01 tune | not eligible: before the final campaign (2026-10-27 to 10-29) |
| DEC-4 | 2026-10-02 13:42 | Geo-L1 | circle | 0.13 | 72.34 | 58.34 | 3.82 | 10-01 tune | not eligible: incomplete: the operator returned to SAFETY 0.8 s before the plan ended; flown through a ground-station clock step that reset the L1 estimate |
| WB-3 | 2026-10-05 16:24 | Proposed | figure-8 | 0.10 | 37.27 | 36.76 | 0.46 | 09-27 tune | not eligible: before the final campaign (2026-10-27 to 10-29) |
| WB-4 | 2026-10-05 16:24 | Proposed | figure-8 | 0.10 | 43.74 | 43.61 | 0.48 | 09-27 tune | not eligible: before the final campaign (2026-10-27 to 10-29) |
| DEC-5 | 2026-10-05 16:37 | Geo-L1 | figure-8 | 0.10 | 72.59 | 66.56 | 3.58 | 10-01 tune | not eligible: before the final campaign (2026-10-27 to 10-29) |
| DEC-6 | 2026-10-05 16:54 | Geo-L1 | figure-8 | 0.13 | 76.86 | 68.73 | 4.69 | 10-01 tune | not eligible: before the final campaign (2026-10-27 to 10-29) |
| WB-5 | 2026-10-05 17:01 | Proposed | figure-8 | 0.13 | 42.66 | 42.11 | 0.54 | 09-27 tune | not eligible: before the final campaign (2026-10-27 to 10-29) |
| WB-6 | 2026-10-05 17:01 | Proposed | figure-8 | 0.13 | 58.19 | 58.31 | 0.48 | 09-27 tune | not eligible: before the final campaign (2026-10-27 to 10-29) |
| WB-7 | 2026-10-27 17:50 | Proposed | circle | 0.20 | 35.78 | 34.48 | 0.35 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| WB-8 | 2026-10-27 17:50 | Proposed | circle | 0.20 | 37.17 | 35.95 | 0.30 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| WB-9 | 2026-10-27 17:58 | Proposed | figure-8 | 0.20 | 40.13 | 39.77 | 0.56 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| WB-10 | 2026-10-27 17:58 | Proposed | figure-8 | 0.20 | 47.54 | 46.79 | 1.17 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| WB-11 | 2026-10-27 18:06 | Proposed | circle | 0.10 | 29.46 | 27.91 | 0.18 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| WB-12 | 2026-10-27 18:06 | Proposed | circle | 0.10 | 29.15 | 27.91 | 0.27 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| DEC-7 | 2026-10-27 18:13 | Geo-L1 | circle | 0.10 | 80.50 | 71.06 | 3.08 | 10-01 tune | **averaged** |
| DEC-8 | 2026-10-27 18:13 | Geo-L1 | circle | 0.10 | 84.53 | 73.81 | 3.21 | 10-01 tune | **averaged** |
| WB-13 | 2026-10-28 14:46 | Proposed | circle | 0.13 | 26.89 | 25.43 | 0.19 | 09-27 tune + 10-16 wrist-roll fix | not averaged: an earlier flight of the same setting (the table uses the latest) |
| WB-14 | 2026-10-28 14:46 | Proposed | circle | 0.13 | 45.44 | 44.42 | 1.19 | 09-27 tune + 10-16 wrist-roll fix | not averaged: an earlier flight of the same setting (the table uses the latest) |
| WB-15 | 2026-10-28 15:08 | Proposed | circle | 0.13 | 31.05 | 29.24 | 0.17 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| WB-16 | 2026-10-28 15:08 | Proposed | circle | 0.13 | 39.57 | 38.37 | 0.81 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| DEC-9 | 2026-10-28 15:16 | Geo-L1 | circle | 0.13 | 89.97 | 80.33 | 3.63 | 10-01 tune | **averaged** |
| DEC-10 | 2026-10-28 15:16 | Geo-L1 | circle | 0.13 | 90.14 | 79.61 | 3.77 | 10-01 tune | **averaged** |
| WB-17 | 2026-10-28 15:22 | Proposed | figure-8 | 0.13 | 38.47 | 37.72 | 0.41 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| WB-18 | 2026-10-28 15:22 | Proposed | figure-8 | 0.13 | 33.76 | 32.43 | 0.50 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| DEC-11 | 2026-10-28 15:28 | Geo-L1 | figure-8 | 0.13 | 84.94 | 78.11 | 4.71 | 10-01 tune | **averaged** |
| DEC-12 | 2026-10-28 15:28 | Geo-L1 | figure-8 | 0.13 | 85.38 | 77.67 | 4.76 | 10-01 tune | **averaged** |
| WB-19 | 2026-10-29 15:06 | Proposed | figure-8 | 0.10 | 38.10 | 36.76 | 0.95 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| WB-20 | 2026-10-29 15:06 | Proposed | figure-8 | 0.10 | 45.07 | 43.70 | 1.44 | 09-27 tune + 10-16 wrist-roll fix | **averaged** |
| DEC-13 | 2026-10-29 15:25 | Geo-L1 | figure-8 | 0.10 | 83.03 | 77.09 | 3.59 | 10-01 tune | **averaged** |
| DEC-14 | 2026-10-29 15:25 | Geo-L1 | figure-8 | 0.10 | 86.10 | 79.79 | 3.62 | 10-01 tune | **averaged** |

Configurations (fsc_autopilot_ros2 hardware yamls; the 10-16 fix sets the EE-heading gains and freezes the
observer rows of a stuck wrist-roll joint, nothing else in the law changed):

- **09-27 tune:** whole-body L1 4-D, the 2026-09-27 tune (K_psi / D_psi 0.2484 / 0.2903).
- **09-27 tune + 10-16 wrist-roll fix:** whole-body L1 4-D, the 2026-09-27 tune + the 2026-10-16 wrist-roll fix (K_psi / D_psi 2.5 / 0.93, j4 stuck-joint hold; fsc_autopilot_ros2 a316625).
- **previous gains:** Geo-L1, the gain set flown since 2026-08-21.
- **10-01 tune:** Geo-L1, the 2026-10-01 tune.

## 3. The experimental table: the runs averaged in each entry (EE position RMSE, mean, mm)

| trajectory | speed [m/s] | Proposed | Geo-L1 |
|---|---|---|---|
| circle | 0.10 | 29.3 (WB-11, WB-12) | 82.5 (DEC-7, DEC-8) |
| circle | 0.13 | 35.3 (WB-15, WB-16) | 90.1 (DEC-9, DEC-10) |
| circle | 0.20 | 36.5 (WB-7, WB-8) | not flown |
| figure-8 | 0.10 | 41.6 (WB-19, WB-20) | 84.6 (DEC-13, DEC-14) |
| figure-8 | 0.13 | 36.1 (WB-17, WB-18) | 85.2 (DEC-11, DEC-12) |
| figure-8 | 0.20 | 43.8 (WB-9, WB-10) | not flown |

Does the mean EE position RMSE grow with the mean speed?

| trajectory | method | 0.10 / 0.13 / 0.20 m/s [mm] | grows |
|---|---|---|---|
| circle | Geo-L1 | 82.5 < 90.1 | yes |
| circle | Proposed | 29.3 < 35.3 < 36.5 | yes |
| figure-8 | Geo-L1 | 84.6 < 85.2 | yes |
| figure-8 | Proposed | 41.6 / 36.1 / 43.8 | **no** |

## 4. The data in results/experiment_results

- `matlab_experiment_data/free_flight_tracking/`: self-contained for MATLAB (its own README.md): one `.mat`
  per averaged run, `<method>_<trajectory>_v<speed>_run<k>.mat` as in the simulation, the summary
  `free_flight_tracking_rmse.mat` and the flown configs.
- `flight_test_data/free_flight_tracking/<trajectory>/v<speed>/<method>_<trajectory>_v<speed>_<runs of the bag>/`:
  each bag that holds an averaged run, copied unchanged (`ros2 bag info` works on the folder).

| file | run | flown | original bag | run in the bag | bag copy | EE RMSE [mm] |
|---|---|---|---|---|---|---|
| `circle/v0p10/geometric_l1_circle_v0p10_run1.mat` | DEC-7 | 2026-10-27 18:13 | `flight_decoupled_l1_circle_010_20261027_181331` | first of 2 | `geometric_l1_circle_v0p10_DEC-7+DEC-8/` | 80.50 |
| `circle/v0p10/geometric_l1_circle_v0p10_run2.mat` | DEC-8 | 2026-10-27 18:13 | `flight_decoupled_l1_circle_010_20261027_181331` | second of 2 | `geometric_l1_circle_v0p10_DEC-7+DEC-8/` | 84.53 |
| `circle/v0p10/whole_body_l1_circle_v0p10_run1.mat` | WB-11 | 2026-10-27 18:06 | `flight_wb_l1_4d_circle_010_20261027_180659` | first of 2 | `whole_body_l1_circle_v0p10_WB-11+WB-12/` | 29.46 |
| `circle/v0p10/whole_body_l1_circle_v0p10_run2.mat` | WB-12 | 2026-10-27 18:06 | `flight_wb_l1_4d_circle_010_20261027_180659` | second of 2 | `whole_body_l1_circle_v0p10_WB-11+WB-12/` | 29.15 |
| `circle/v0p13/geometric_l1_circle_v0p13_run1.mat` | DEC-9 | 2026-10-28 15:16 | `flight_decoupled_l1_circle_013_20261028_151600` | first of 2 | `geometric_l1_circle_v0p13_DEC-9+DEC-10/` | 89.97 |
| `circle/v0p13/geometric_l1_circle_v0p13_run2.mat` | DEC-10 | 2026-10-28 15:16 | `flight_decoupled_l1_circle_013_20261028_151600` | second of 2 | `geometric_l1_circle_v0p13_DEC-9+DEC-10/` | 90.14 |
| `circle/v0p13/whole_body_l1_circle_v0p13_run1.mat` | WB-15 | 2026-10-28 15:08 | `flight_wb_l1_4d_circle_013_20261028_150821` | first of 2 | `whole_body_l1_circle_v0p13_WB-15+WB-16/` | 31.05 |
| `circle/v0p13/whole_body_l1_circle_v0p13_run2.mat` | WB-16 | 2026-10-28 15:08 | `flight_wb_l1_4d_circle_013_20261028_150821` | second of 2 | `whole_body_l1_circle_v0p13_WB-15+WB-16/` | 39.57 |
| `circle/v0p20/whole_body_l1_circle_v0p20_run1.mat` | WB-7 | 2026-10-27 17:50 | `flight_wb_l1_4d_circle_020_20261027_175015` | first of 2 | `whole_body_l1_circle_v0p20_WB-7+WB-8/` | 35.78 |
| `circle/v0p20/whole_body_l1_circle_v0p20_run2.mat` | WB-8 | 2026-10-27 17:50 | `flight_wb_l1_4d_circle_020_20261027_175015` | second of 2 | `whole_body_l1_circle_v0p20_WB-7+WB-8/` | 37.17 |
| `figure8/v0p10/geometric_l1_figure8_v0p10_run1.mat` | DEC-13 | 2026-10-29 15:25 | `flight_decoupled_l1_figure8_010_20261029_152543` | first of 2 | `geometric_l1_figure8_v0p10_DEC-13+DEC-14/` | 83.03 |
| `figure8/v0p10/geometric_l1_figure8_v0p10_run2.mat` | DEC-14 | 2026-10-29 15:25 | `flight_decoupled_l1_figure8_010_20261029_152543` | second of 2 | `geometric_l1_figure8_v0p10_DEC-13+DEC-14/` | 86.10 |
| `figure8/v0p10/whole_body_l1_figure8_v0p10_run1.mat` | WB-19 | 2026-10-29 15:06 | `flight_wb_l1_4d_figure8_010_20261029_150625` | first of 2 | `whole_body_l1_figure8_v0p10_WB-19+WB-20/` | 38.10 |
| `figure8/v0p10/whole_body_l1_figure8_v0p10_run2.mat` | WB-20 | 2026-10-29 15:06 | `flight_wb_l1_4d_figure8_010_20261029_150625` | second of 2 | `whole_body_l1_figure8_v0p10_WB-19+WB-20/` | 45.07 |
| `figure8/v0p13/geometric_l1_figure8_v0p13_run1.mat` | DEC-11 | 2026-10-28 15:28 | `flight_decoupled_l1_figure8_013_20261028_152811` | first of 2 | `geometric_l1_figure8_v0p13_DEC-11+DEC-12/` | 84.94 |
| `figure8/v0p13/geometric_l1_figure8_v0p13_run2.mat` | DEC-12 | 2026-10-28 15:28 | `flight_decoupled_l1_figure8_013_20261028_152811` | second of 2 | `geometric_l1_figure8_v0p13_DEC-11+DEC-12/` | 85.38 |
| `figure8/v0p13/whole_body_l1_figure8_v0p13_run1.mat` | WB-17 | 2026-10-28 15:22 | `flight_wb_l1_4d_figure8_013_20261028_152202` | first of 2 | `whole_body_l1_figure8_v0p13_WB-17+WB-18/` | 38.47 |
| `figure8/v0p13/whole_body_l1_figure8_v0p13_run2.mat` | WB-18 | 2026-10-28 15:22 | `flight_wb_l1_4d_figure8_013_20261028_152202` | second of 2 | `whole_body_l1_figure8_v0p13_WB-17+WB-18/` | 33.76 |
| `figure8/v0p20/whole_body_l1_figure8_v0p20_run1.mat` | WB-9 | 2026-10-27 17:58 | `flight_wb_l1_4d_figure8_020_20261027_175849` | first of 2 | `whole_body_l1_figure8_v0p20_WB-9+WB-10/` | 40.13 |
| `figure8/v0p20/whole_body_l1_figure8_v0p20_run2.mat` | WB-10 | 2026-10-27 17:58 | `flight_wb_l1_4d_figure8_020_20261027_175849` | second of 2 | `whole_body_l1_figure8_v0p20_WB-9+WB-10/` | 47.54 |
