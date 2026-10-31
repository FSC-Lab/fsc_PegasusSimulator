# Experiment results, MATLAB data (free-flight trajectory tracking)

This is the data behind the paper's experimental table (`tab:exp_free_flight_tracking`), converted from the
hardware ros2 bags. The folder is self-contained and laid out like the simulation's
`matlab_simulation_data/free_flight_tracking/`: these files plus MATLAB are all you need, with no ROS or custom
message definitions.

## Files

```
<shape>/v<speed>/<method>_<shape>_v<speed>_run<k>.mat   one run, struct `run`
free_flight_tracking_rmse.mat     rmse_mean = the table, rmse_runs = its runs, rmse_all = every scored run
free_flight_tracking_exp.tex      the table in LaTeX (tab:exp_free_flight_tracking), from rmse_mean
configs/                          the hardware configs of the final campaign (taken from git)
```

- **`<shape>` / `v<speed>`:** as in `matlab_simulation_data` (`v0p13` = 0.13 m/s).
- **`<method>`:** `whole_body_l1` (Proposed) or `geometric_l1` (Geo-L1, position-mode arm).
- **`run<k>`:** the k-th of the runs averaged in that table entry, in flight order. `run.meta.tag` is the flight
  tag (WB-n, DEC-n) used in the lab's reports and in the index below.

```matlab
r = load('circle/v0p10/whole_body_l1_circle_v0p10_run1.mat').run;
plot(r.tracking.t, vecnorm(r.tracking.ee_pos_err_m, 2, 2) * 1e3)        % EE position error [mm]
S = load('free_flight_tracking_rmse.mat');
T = struct2table(S.rmse_mean);     % the table: one row per (shape, speed, method)
R = struct2table(S.rmse_runs);     % every run averaged in it
A = struct2table(S.rmse_all);      % every scored run, with its status
```

## How the table is made

The simulation's rule: **each table entry is the mean, column by column, of the per-run RMSE of its runs.**
- **Runs:** the completed runs of that setting flown in the final campaign, 2026-10-27 to 10-29, with one
  configuration per method throughout (below). Earlier flights are scored and listed but not averaged.
- **Two repeated tests per entry:** each flight (bag) flies two one-lap runs back to back, each its own
  planner run from rest to rest, and each lap counts as one test. An entry averages both laps of the
  setting's latest flight. The rule never looks at the error. Only one setting (Proposed, circle 0.13 m/s)
  was flown twice; its earlier flight is scored and listed but not averaged.

### The runs in each entry (mean EE position RMSE, mm)

| trajectory | speed [m/s] | Proposed | Geo-L1 |
|---|---|---|---|
| circle | 0.10 | 29.3 (WB-11, WB-12) | 82.5 (DEC-7, DEC-8) |
| circle | 0.13 | 35.3 (WB-15, WB-16) | 90.1 (DEC-9, DEC-10) |
| circle | 0.20 | 36.5 (WB-7, WB-8) | not flown |
| figure-8 | 0.10 | 41.6 (WB-19, WB-20) | 84.6 (DEC-13, DEC-14) |
| figure-8 | 0.13 | 36.1 (WB-17, WB-18) | 85.2 (DEC-11, DEC-12) |
| figure-8 | 0.20 | 43.8 (WB-9, WB-10) | not flown |

Not flown on hardware ("--" in the table): Geo-L1 circle 0.20 m/s, Geo-L1 figure-8 0.20 m/s.

### Does the error grow with the mean speed?

| trajectory | method | mean EE RMSE at 0.10 / 0.13 / 0.20 m/s [mm] | grows |
|---|---|---|---|
| circle | Geo-L1 | 82.5 < 90.1 | yes |
| circle | Proposed | 29.3 < 35.3 < 36.5 | yes |
| figure-8 | Geo-L1 | 84.6 < 85.2 | yes |
| figure-8 | Proposed | 41.6 / 36.1 / 43.8 | **no** |

### The files

| file | run | flown | original ros2 bag | run in the bag | bag copy | EE RMSE [mm] |
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

The bag copies are in `results/experiment_results/flight_test_data/free_flight_tracking/<shape>/v<speed>/` on the
lab machine; a bag holds both runs of its flight.

### Every scored run of the formal comparison

Same metric definitions as the simulation table.

| run | flown | method | trajectory | speed [m/s] | EE RMSE [mm] | platform RMSE [mm] | EE heading RMSE [deg] | configuration | status |
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

### Conditions of the final campaign

- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm, total mass 3.746 kg.
- **Feedback:** OptiTrack fused in PX4 EKF2, about 100 Hz; joint encoders.
- **Proposed:**
  - the 2026-09-27 tune: k_x / k_v 50.03 / 12.58, k_R / k_ω 2.134 / 1.567, K_y / D_y 211.9 / 26.82;
  - with the 2026-10-16 wrist-roll fix: K_ψ / D_ψ 2.5 / 0.93 and the j4 stuck-joint hold
    (WB-7, WB-8, WB-9, WB-10, WB-11, WB-12, WB-15, WB-16, WB-17, WB-18, WB-19, WB-20);
  - L1 bandwidth ω_c 2.927 / 0.7428 / 0.8479 rad/s (translation / rotation / arm);
  - the arm in torque mode with friction feed-forward and the velocity observer.
- **Geo-L1:**
  - the 2026-10-01 tune: K_p 20.11 (x, y) / 13.5 (z), K_v 11.05 / 10.82, k_R 3.337 / 1.737,
    k_ω 0.9505 / 0.4105;
  - L1: A_s 4.334 / 2.791, ω_c 1.0 rad/s;
  - the arm in position mode.
- **Allocator thrust coefficient:** 4.260431e-05 for both.
- **Configs:** `configs/whole_body_l1_4d_hw.yaml` (fsc_autopilot_ros2 05c9cdb (2026-10-19)); `configs/geometric_l1_hw.yaml` (fsc_autopilot_ros2 d85be8a (2026-10-05)); `configs/arm_external_torque_controller_hardware_aerial_pwm.yaml` (fsc_open_manipulator f812b48 (2026-10-17)); `configs/arm_position_controller_hardware_aerial.yaml` (fsc_open_manipulator 6c5ef48 (2026-10-09)).

## Trajectories (hardware runs: 0.10, 0.13 and 0.20 m/s)

The planner generates one end-effector trajectory per setting, and every method receives the same reference.
- **Circle:** radius 0.50 m.
- **Figure-8:** a Gerono lemniscate, half-length 0.70 m and half-width 0.35 m (1.40 × 0.70 m), long axis on
  world x, path length 4.268 m per lap.
- **Timing:** one lap with a 4 s minimum-snap speed-up and slow-down. Lap time = path length / mean EE speed.
- **Heading:** the gripper heading follows the path tangent.
- **Arm sweep:** q2 = 25° ± 15°, four cycles per lap, fold q2 + q3 = 55°, q1 = 0. The airframe follows the
  compatible whole-body reference.

| mean EE speed [m/s] | circle lap [s] | figure-8 lap [s] |
|---|---|---|
| 0.10 | 31.42 | 42.68 |
| 0.13 | 24.17 | 32.83 |
| 0.20 | 15.71 | 21.34 |

## The struct `run`

Every run file holds one struct, `run`:

| field | content |
|---|---|
| `meta` | run tag, run index k (`run`), the runs averaged with it (`averaged_with`, `runs_in_mean`), method, shape, nominal mean speed, original bag, which run in the bag, flight date, run window (s since the bag's first message), vehicle, feedback, trajectory, controller configuration, selection rule |
| `rmse` | this run's numbers: `platform_pos_xyz_mm` (x y z), `platform_pos_mm` (3-D norm), `platform_roll_deg`, `platform_pitch_deg`, `platform_heading_deg`, `ee_pos_xyz_mm`, `ee_pos_mm`, `ee_heading_deg`, `joints_deg` (q1..q4), plus `com_pos_mm`, `ee_pos_max_mm`, `tilt_max_deg` |
| `tracking` | 100 Hz over the trajectory run, `t` in s since the run started. Measured, reference (`_ref_`) and error (`_err_`) for the platform position (`platform_pos_*`) and attitude (`platform_quat_xyzw`, `platform_att_err_deg`, `platform_heading_*`, `platform_tilt_deg`), the system CoM (`com_pos_*`), the end-effector position (`ee_pos_*`) and heading (`ee_heading_*`), and the joints (`q_deg`, `q_ref_deg`, `q_err_deg`) |
| `raw` | the bag's streams, time in s since the bag's first message: `odom` (EKF2-fused odometry: position, velocity, attitude, body rates), `joints` (q and qdot, model convention), `reference` (the planner's whole-body reference), `controller_debug` (`wb_control_debug` for the proposed method, `l1_control_debug` for Geo-L1), `motors` (PX4 motor commands, normalised), `battery` (voltage, current), `mocap_vrpn` (raw OptiTrack pose), `events.mode` / `events.planner_status` (transition times and labels) |

Units: positions in m (world ENU, z up) unless the field name says mm; angles in deg unless the name says
rad. Errors are measured − reference. `platform_att_err_deg` columns are `[roll pitch heading]`.

## Paper notation

How the table's symbols map to the fields here. The RMSE appears in two places: the summary file
`free_flight_tracking_rmse.mat`, and each run file's `run.rmse`. The time series behind each RMSE is in
`run.tracking`, as measured − reference.

| paper symbol | meaning | summary file field | run file: `run.rmse.` | run file: `run.tracking.` |
|---|---|---|---|---|
| r<sub>0,x</sub>, r<sub>0,y</sub>, r<sub>0,z</sub> | platform position (body origin O<sub>0</sub>) in {I} [mm] | `platform_x_mm`, `platform_y_mm`, `platform_z_mm` | `platform_pos_xyz_mm(1:3)` | `platform_pos_err_m(:,1:3)` [m] |
| ψ<sub>0</sub>, φ<sub>0</sub>, θ<sub>0</sub> | platform heading, roll, pitch: body-axis components of the rotation vector of R<sub>0,d</sub><sup>T</sup> R<sub>0</sub> [deg] | `platform_heading_deg`, `platform_roll_deg`, `platform_pitch_deg` | same names | `platform_att_err_deg(:,3)`, `(:,1)`, `(:,2)` |
| r<sub>e,x</sub>, r<sub>e,y</sub>, r<sub>e,z</sub> | end-effector position in {I} [mm] | `ee_x_mm`, `ee_y_mm`, `ee_z_mm` | `ee_pos_xyz_mm(1:3)` | `ee_pos_err_m(:,1:3)` [m] |
| ψ<sub>e</sub> | end-effector heading: azimuth of b<sub>1,e</sub> [deg] | `ee_heading_deg` | `ee_heading_deg` | `ee_heading_err_deg` |
| q<sub>1</sub> … q<sub>4</sub> | joint angles, model convention [deg] | `q1_deg` … `q4_deg` | `joints_deg(1:4)` | `q_err_deg(:,1:4)` |

- **Reference:** every error is measured against the planner's dynamically compatible whole-body reference
  trajectory, which is the same for every method.
  - Platform position: x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>).
  - Platform attitude R<sub>0,d</sub>: thrust along the CoM reference acceleration + g e<sub>3</sub>, heading
    from the reference. It is not the controller's internal attitude command.
  - End effector: position r<sub>e,d</sub>, heading b<sub>1,e,d</sub>. Joints: q<sub>d</sub>.
- **Measured end effector:** the planner model's forward kinematics of the measured platform pose and joint
  angles.
- **Window:** each RMSE covers the whole planned trajectory, including the 4 s speed-up and slow-down, with
  0.1 s trimmed at each end, on a 100 Hz grid.
- **Norms:** `platform_pos_mm` and `ee_pos_mm` are the RMSE of the 3-D error norm. They are not in the paper
  table but are useful for plots.
