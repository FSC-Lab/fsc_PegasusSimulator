# Experiment results, MATLAB data (free-flight trajectory tracking with an unmodelled end-effector mass)

Circle tracking at 0.13 m/s with a mass on the end effector that the controller's model does not include, flown on 2026-11-05, converted from the hardware ros2
bags. Same task, configuration and metric definitions as `../free_flight_tracking/` (the undisturbed
circle 0.13 m/s entry is the reference row below), so a script written for those files runs on these.
This folder is self-contained: these files plus MATLAB are all you need.

## Files

```
<setting>/<method>_circle_v0p13_<setting>_run<k>.mat       one LAP, struct `run`
free_flight_model_uncertainty_rmse.mat                     rmse_mean, rmse_runs, rmse_flights, rmse_nominal, bags
configs/                                                   the hardware configs as flown (taken from git)
```

- **`<setting>`:** `ee_mass_50g` = a 50 g mass on the end effector, not in the controller model, `ee_mass_100g` = a 100 g mass on the end effector, not in the controller model.
- **`<method>`:** `whole_body_l1` (Proposed) or `geometric_l1` (Geo-L1, position-mode arm).
- **`run<k>`:** the k-th lap of that setting and method, in flight order (two laps per flight).
  `run.meta.tag` is the lap's tag (e.g. `WB-50g-3`), `run.meta.lap` says which lap of its flight it is.

Summary file (`struct2table` each):

- `rmse_mean`: one row per setting and method, the mean over its laps (the entry to report).
- `rmse_runs`: every lap in it.
- `rmse_flights`: each flight's two-lap run scored as a whole (for reference).
- `rmse_nominal`: the undisturbed circle 0.13 m/s entry of `free_flight_tracking` (2026-10-28, WB-15/16,
  DEC-9/10).
- `bags`: every bag of the study and what the download holds.

```matlab
r = load('ee_mass_100g/whole_body_l1_circle_v0p13_ee100g_run1.mat').run;
plot(r.tracking.t, vecnorm(r.tracking.ee_pos_err_m, 2, 2) * 1e3)        % EE position error [mm]
S = load('free_flight_model_uncertainty_rmse.mat');
T = struct2table(S.rmse_mean);    N = struct2table(S.rmse_nominal);
```

## How the entries are made

- **Runs = laps.** Each flight flies ONE planner run of two laps back to back (T = 52.33 s, lap 24.17 s),
  not the nominal campaign's two separate one-lap runs. The laps are split where the planner says the
  first lap ends (`ee_trajectory/info[15]`, 26.17 s into the run). So lap 1 holds the 2 s speed-up and
  lap 2 the 2 s slow-down; a nominal run holds both ramps in its 28.17 s.
- **Each lap is scored like a nominal run:** the hardware report's definitions, 0.1 s trimmed at each end,
  100 Hz grid, against the planner's streamed whole-body reference.
- **Entry = the mean over all laps of the setting:** 2 flights × 2 laps = 4. The rule never looks at the error.

## Results (mean over the laps)

| method | disturbance | laps | EE RMSE [mm] | EE heading RMSE [deg] | platform RMSE [mm] | platform heading RMSE [deg] | max tilt [deg] |
|---|---|---|---|---|---|---|---|
| Proposed | none (nominal, 2026-10-28) | 2 (WB-15, WB-16) | 35.3 | 0.49 | 33.8 | 0.22 | 3.5 |
| Proposed | EE +50 g | 4 of 4 (WB-50g-1, WB-50g-2, WB-50g-3, WB-50g-4) | 42.9 | 0.34 | 42.0 | 0.28 | 4.1 |
| Proposed | EE +100 g | 4 of 4 (WB-100g-1, WB-100g-2, WB-100g-3, WB-100g-4) | 41.4 | 0.32 | 39.9 | 0.33 | 4.1 |
| Geo-L1 | none (nominal, 2026-10-28) | 2 (DEC-9, DEC-10) | 90.1 | 3.70 | 80.0 | 3.90 | 3.8 |
| Geo-L1 | EE +50 g | 4 of 4 (DEC-50g-1, DEC-50g-2, DEC-50g-3, DEC-50g-4) | 74.9 | 4.12 | 64.4 | 4.20 | 5.4 |
| Geo-L1 | EE +100 g | 4 of 4 (DEC-100g-1, DEC-100g-2, DEC-100g-3, DEC-100g-4) | 68.1 | 4.15 | 61.3 | 4.23 | 6.8 |

The nominal rows are `../free_flight_tracking/`'s circle 0.13 m/s entries (one-lap runs, flown 10-28).

### Every lap

| file | lap tag | method | disturbance | flown | lap of the flight | window [s] | EE RMSE [mm] | EE heading RMSE [deg] | platform RMSE [mm] |
|---|---|---|---|---|---|---|---|---|---|
| `ee_mass_100g/geometric_l1_circle_v0p13_ee100g_run1.mat` | DEC-100g-1 | Geo-L1 | EE +100 g | 2026-11-05 17:27 | 1 | 26.17 | 61.0 | 4.23 | 57.4 |
| `ee_mass_100g/geometric_l1_circle_v0p13_ee100g_run2.mat` | DEC-100g-2 | Geo-L1 | EE +100 g | 2026-11-05 17:27 | 2 | 26.17 | 69.0 | 4.28 | 63.2 |
| `ee_mass_100g/geometric_l1_circle_v0p13_ee100g_run3.mat` | DEC-100g-3 | Geo-L1 | EE +100 g | 2026-11-05 17:34 | 1 | 26.17 | 65.0 | 4.04 | 56.6 |
| `ee_mass_100g/geometric_l1_circle_v0p13_ee100g_run4.mat` | DEC-100g-4 | Geo-L1 | EE +100 g | 2026-11-05 17:34 | 2 | 26.17 | 77.3 | 4.05 | 68.2 |
| `ee_mass_50g/geometric_l1_circle_v0p13_ee50g_run1.mat` | DEC-50g-1 | Geo-L1 | EE +50 g | 2026-11-05 17:58 | 1 | 26.17 | 70.0 | 3.98 | 60.3 |
| `ee_mass_50g/geometric_l1_circle_v0p13_ee50g_run2.mat` | DEC-50g-2 | Geo-L1 | EE +50 g | 2026-11-05 17:58 | 2 | 26.17 | 81.2 | 4.03 | 70.8 |
| `ee_mass_50g/geometric_l1_circle_v0p13_ee50g_run3.mat` | DEC-50g-3 | Geo-L1 | EE +50 g | 2026-11-05 18:03 | 1 | 26.17 | 74.8 | 4.21 | 63.8 |
| `ee_mass_50g/geometric_l1_circle_v0p13_ee50g_run4.mat` | DEC-50g-4 | Geo-L1 | EE +50 g | 2026-11-05 18:03 | 2 | 26.17 | 73.7 | 4.24 | 62.7 |
| `ee_mass_100g/whole_body_l1_circle_v0p13_ee100g_run1.mat` | WB-100g-1 | Proposed | EE +100 g | 2026-11-05 17:44 | 1 | 26.17 | 38.8 | 0.33 | 37.3 |
| `ee_mass_100g/whole_body_l1_circle_v0p13_ee100g_run2.mat` | WB-100g-2 | Proposed | EE +100 g | 2026-11-05 17:44 | 2 | 26.17 | 44.4 | 0.36 | 42.5 |
| `ee_mass_100g/whole_body_l1_circle_v0p13_ee100g_run3.mat` | WB-100g-3 | Proposed | EE +100 g | 2026-11-05 17:49 | 1 | 26.17 | 37.4 | 0.27 | 35.8 |
| `ee_mass_100g/whole_body_l1_circle_v0p13_ee100g_run4.mat` | WB-100g-4 | Proposed | EE +100 g | 2026-11-05 17:49 | 2 | 26.17 | 45.1 | 0.30 | 44.1 |
| `ee_mass_50g/whole_body_l1_circle_v0p13_ee50g_run1.mat` | WB-50g-1 | Proposed | EE +50 g | 2026-11-05 18:10 | 1 | 26.17 | 40.5 | 0.28 | 39.3 |
| `ee_mass_50g/whole_body_l1_circle_v0p13_ee50g_run2.mat` | WB-50g-2 | Proposed | EE +50 g | 2026-11-05 18:10 | 2 | 26.17 | 39.9 | 0.36 | 38.8 |
| `ee_mass_50g/whole_body_l1_circle_v0p13_ee50g_run3.mat` | WB-50g-3 | Proposed | EE +50 g | 2026-11-05 18:18 | 1 | 26.17 | 46.5 | 0.36 | 45.4 |
| `ee_mass_50g/whole_body_l1_circle_v0p13_ee50g_run4.mat` | WB-50g-4 | Proposed | EE +50 g | 2026-11-05 18:18 | 2 | 26.17 | 44.9 | 0.35 | 44.4 |

### The bags

| flown | original ros2 bag | method | disturbance | laps | status | bag copy (`flight_test_data/free_flight_model_uncertainty/`) | both laps, EE RMSE [mm] |
|---|---|---|---|---|---|---|---|
| 2026-11-05 17:27 | `flight_decoupled_l1_circle_013_100g_20261105_172752` | Geo-L1 | EE +100 g | DEC-100g-1, DEC-100g-2 | scored | `ee_mass_100g/geometric_l1_circle_v0p13_ee100g_DEC-100g-1+DEC-100g-2/` | 65.1 |
| 2026-11-05 17:34 | `flight_decoupled_l1_circle_013_100g_20261105_173408` | Geo-L1 | EE +100 g | DEC-100g-3, DEC-100g-4 | scored | `ee_mass_100g/geometric_l1_circle_v0p13_ee100g_DEC-100g-3+DEC-100g-4/` | 71.4 |
| 2026-11-05 17:44 | `flight_wb_l1_4d_circle_013_100g_20261105_174407` | Proposed | EE +100 g | WB-100g-1, WB-100g-2 | scored | `ee_mass_100g/whole_body_l1_circle_v0p13_ee100g_WB-100g-1+WB-100g-2/` | 41.9 |
| 2026-11-05 17:49 | `flight_wb_l1_4d_circle_013_100g_20261105_174953` | Proposed | EE +100 g | WB-100g-3, WB-100g-4 | scored | `ee_mass_100g/whole_body_l1_circle_v0p13_ee100g_WB-100g-3+WB-100g-4/` | 41.3 |
| 2026-11-05 17:58 | `flight_decoupled_l1_circle_013_50g_20261105_175803` | Geo-L1 | EE +50 g | DEC-50g-1, DEC-50g-2 | scored | `ee_mass_50g/geometric_l1_circle_v0p13_ee50g_DEC-50g-1+DEC-50g-2/` | 75.7 |
| 2026-11-05 18:03 | `flight_decoupled_l1_circle_013_50g_20261105_180344` | Geo-L1 | EE +50 g | DEC-50g-3, DEC-50g-4 | scored | `ee_mass_50g/geometric_l1_circle_v0p13_ee50g_DEC-50g-3+DEC-50g-4/` | 74.2 |
| 2026-11-05 18:10 | `flight_wb_l1_4d_circle_013_50g_20261105_181041` | Proposed | EE +50 g | WB-50g-1, WB-50g-2 | scored | `ee_mass_50g/whole_body_l1_circle_v0p13_ee50g_WB-50g-1+WB-50g-2/` | 40.2 |
| 2026-11-05 18:18 | `flight_wb_l1_4d_circle_013_50g_20261105_181833` | Proposed | EE +50 g | WB-50g-3, WB-50g-4 | scored | `ee_mass_50g/whole_body_l1_circle_v0p13_ee50g_WB-50g-3+WB-50g-4/` | 45.6 |

The originals are in `docs/experimental_data_ros2_bag/1105 - T650-AM Mail-Box, Robust Tracking, Box-Pushing-*/Robust Tracking [Success]/`.

## Conditions

- **Disturbance:** EE +50 g: a 50 g mass on the end effector, not in the controller model; EE +100 g: a 100 g mass on the end effector, not in the controller model.
- **Trajectory:** the nominal circle: radius 0.50 m, 0.13 m/s (lap 24.17 s), q2 = 25 ± 15 deg four
  cycles per lap, fold 55 deg, EE heading along the tangent; two laps per flight.
- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm, total mass 3.746 kg in the controller
  model (the disturbance mass is not in it).
- **Feedback:** OptiTrack fused in PX4 EKF2, about 100 Hz; joint encoders.
- **Controllers:** the final campaign's hardware yamls, unchanged (`git diff` between the final campaign's
  commit and the branch flown on 11-05 is empty for both); the nodes were built from the `dev_robotic_arm`
  branch, whose later features the free-flight yaml leaves off.
  - Proposed: whole-body L1 4-D, the final campaign's hardware yaml unchanged (2026-09-27 tune + the 2026-10-16 wrist-roll fix; fsc_autopilot_ros2 05c9cdb); node built from the dev_robotic_arm branch.
  - Geo-L1: Geo-L1, the 2026-10-01 tune unchanged (fsc_autopilot_ros2 d85be8a); node built from the dev_robotic_arm branch.
- **Configs:** `configs/whole_body_l1_4d_hw.yaml` (fsc_autopilot_ros2 05c9cdb (2026-10-19)); `configs/geometric_l1_hw.yaml` (fsc_autopilot_ros2 d85be8a (2026-10-05)); `configs/arm_external_torque_controller_hardware_aerial_pwm.yaml` (fsc_open_manipulator f812b48 (2026-10-17)); `configs/arm_position_controller_hardware_aerial.yaml` (fsc_open_manipulator 6c5ef48 (2026-10-09)).

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

Extra `meta` fields here: `setting`, `setting_label`, `disturbance` (and `ee_mass_kg` or `wind_speed_mps` /
`fan_distance_m`), `lap`, `laps_in_planner_run`, `lap_start_s` / `lap_end_s`, `planner_run_start_s` /
`planner_run_end_s`, `planner_run_duration_s`, `lap_time_s`, `ramp_time_s` (all s since the bag's first
message, except the durations). `raw` holds the whole bag, so both laps of a flight carry the same `raw`.

Regenerate (from the bags in `docs/experimental_data_ros2_bag/`):

```bash
/usr/bin/python3 results/utils/experiment_robust_tracking.py --copy-bags
```
