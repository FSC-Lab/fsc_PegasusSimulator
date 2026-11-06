# Experiment results, MATLAB data (free-flight trajectory tracking in wind)

Circle tracking at 0.13 m/s with wind from a fan, 5 m/s, placed 4 m from the circle's centre, flown on 2026-11-05, converted from the hardware ros2
bags. Same task, configuration and metric definitions as `../free_flight_tracking/` (the undisturbed
circle 0.13 m/s entry is the reference row below), so a script written for those files runs on these.
This folder is self-contained: these files plus MATLAB are all you need.

## Files

```
<method>_circle_v0p13_<setting>_run<k>.mat                 one LAP, struct `run`
free_flight_wind_disturbance_rmse.mat                      rmse_mean, rmse_runs, rmse_flights, rmse_nominal, bags
configs/                                                   the hardware configs as flown (taken from git)
```

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
r = load('whole_body_l1_circle_v0p13_wind_run1.mat').run;
plot(r.tracking.t, vecnorm(r.tracking.ee_pos_err_m, 2, 2) * 1e3)        % EE position error [mm]
S = load('free_flight_wind_disturbance_rmse.mat');
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
| Proposed | wind 5 m/s | 4 of 4 (WB-wind-1, WB-wind-2, WB-wind-3, WB-wind-4) | 38.7 | 0.32 | 37.7 | 0.30 | 3.7 |
| Geo-L1 | none (nominal, 2026-10-28) | 2 (DEC-9, DEC-10) | 90.1 | 3.70 | 80.0 | 3.90 | 3.8 |
| Geo-L1 | wind 5 m/s | 4 of 4 (DEC-wind-1, DEC-wind-2, DEC-wind-3, DEC-wind-4) | 79.3 | 4.08 | 69.1 | 4.16 | 9.8 |

The nominal rows are `../free_flight_tracking/`'s circle 0.13 m/s entries (one-lap runs, flown 10-28).

### Every lap

| file | lap tag | method | disturbance | flown | lap of the flight | window [s] | EE RMSE [mm] | EE heading RMSE [deg] | platform RMSE [mm] |
|---|---|---|---|---|---|---|---|---|---|
| `geometric_l1_circle_v0p13_wind_run1.mat` | DEC-wind-1 | Geo-L1 | wind 5 m/s | 2026-11-05 18:52 | 1 | 26.17 | 83.8 | 3.90 | 73.6 |
| `geometric_l1_circle_v0p13_wind_run2.mat` | DEC-wind-2 | Geo-L1 | wind 5 m/s | 2026-11-05 18:52 | 2 | 26.17 | 82.8 | 3.92 | 72.9 |
| `geometric_l1_circle_v0p13_wind_run3.mat` | DEC-wind-3 | Geo-L1 | wind 5 m/s | 2026-11-05 19:02 | 1 | 26.17 | 70.2 | 4.28 | 60.1 |
| `geometric_l1_circle_v0p13_wind_run4.mat` | DEC-wind-4 | Geo-L1 | wind 5 m/s | 2026-11-05 19:02 | 2 | 26.17 | 80.4 | 4.21 | 69.9 |
| `whole_body_l1_circle_v0p13_wind_run1.mat` | WB-wind-1 | Proposed | wind 5 m/s | 2026-11-05 19:07 | 1 | 26.17 | 35.5 | 0.25 | 34.3 |
| `whole_body_l1_circle_v0p13_wind_run2.mat` | WB-wind-2 | Proposed | wind 5 m/s | 2026-11-05 19:07 | 2 | 26.17 | 44.2 | 0.34 | 43.7 |
| `whole_body_l1_circle_v0p13_wind_run3.mat` | WB-wind-3 | Proposed | wind 5 m/s | 2026-11-05 19:13 | 1 | 26.17 | 32.1 | 0.27 | 31.0 |
| `whole_body_l1_circle_v0p13_wind_run4.mat` | WB-wind-4 | Proposed | wind 5 m/s | 2026-11-05 19:13 | 2 | 26.18 | 43.1 | 0.43 | 42.0 |

### The bags

| flown | original ros2 bag | method | disturbance | laps | status | bag copy (`flight_test_data/free_flight_wind_disturbance/`) | both laps, EE RMSE [mm] |
|---|---|---|---|---|---|---|---|
| 2026-11-05 18:52 | `flight_decoupled_l1_circle_013_wind_20261105_185232` | Geo-L1 | wind 5 m/s | DEC-wind-1, DEC-wind-2 | scored | `geometric_l1_circle_v0p13_wind_DEC-wind-1+DEC-wind-2/` | 83.6 |
| 2026-11-05 19:02 | `flight_decoupled_l1_circle_013_wind_20261105_190235` | Geo-L1 | wind 5 m/s | DEC-wind-3, DEC-wind-4 | scored | `geometric_l1_circle_v0p13_wind_DEC-wind-3+DEC-wind-4/` | 75.5 |
| 2026-11-05 19:07 | `flight_wb_l1_4d_circle_013_wind_20261105_190722` | Proposed | wind 5 m/s | WB-wind-1, WB-wind-2 | scored | `whole_body_l1_circle_v0p13_wind_WB-wind-1+WB-wind-2/` | 40.0 |
| 2026-11-05 19:13 | `flight_wb_l1_4d_circle_013_wind_20261105_191334` | Proposed | wind 5 m/s | WB-wind-3, WB-wind-4 | scored | `whole_body_l1_circle_v0p13_wind_WB-wind-3+WB-wind-4/` | 38.0 |

The originals are in `docs/experimental_data_ros2_bag/1105 - T650-AM Mail-Box, Robust Tracking, Box-Pushing-*/Robust Tracking [Success]/`.

## Conditions

- **Disturbance:** wind 5 m/s: 5 m/s wind from a fan placed 4 m from the circle's centre.
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
