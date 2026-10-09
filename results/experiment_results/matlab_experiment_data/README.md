# Experiment results, MATLAB data (free-flight trajectory tracking)

This is the data behind the paper's experimental table (Table III, `tab:exp_free_flight_tracking`): the
selected hardware run of each filled setting, converted from its ros2 bag. The folder is self-contained:
these files plus MATLAB are all you need, with no ROS or custom message definitions.

## Files

```
free_flight_tracking/<shape>/v<speed>/<method>_<shape>_v<speed>_<run>.mat   one run, struct `run`
free_flight_tracking_rmse.mat     struct `rmse` = Table III (one row per selected run)
```

- **`<shape>` / `v<speed>`:** as in `matlab_simulation_data` (`v0p13` = 0.13 m/s).
- **`<method>`:** `whole_body_l1` (Proposed) or `geometric_l1` (Geo-L1, position-mode arm).
- **`<run>`:** the flight tag (WB-n, DEC-n) used in the lab's comparison reports.

```matlab
r = load('free_flight_tracking/circle/v0p13/whole_body_l1_circle_v0p13_WB-2.mat').run;
plot(r.tracking.t, vecnorm(r.tracking.ee_pos_err_m, 2, 2) * 1e3)        % EE position error [mm]
T = struct2table(load('free_flight_tracking_rmse.mat').rmse);           % Table III
```

## Experiment index

### The runs in this folder (one per filled table cell)

**Selection rule:** among the repeated flights of a setting, the completed run with the lowest
end-effector position RMSE, flown with the controller version the paper compares.

| file | run | trajectory | speed [m/s] | method | flown | original ros2 bag | run in the bag | EE RMSE [mm] |
|---|---|---|---|---|---|---|---|---|
| `free_flight_tracking/circle/v0p13/geometric_l1_circle_v0p13_DEC-3.mat` | DEC-3 | circle | 0.13 | Geo-L1 | 2026-10-02 13:42 | `flight_decoupled_l1_circle_20261002_134247` | first of 2 | 77.45 |
| `free_flight_tracking/circle/v0p13/whole_body_l1_circle_v0p13_WB-2.mat` | WB-2 | circle | 0.13 | Proposed | 2026-09-28 17:22 | `flight_wb_l1_4d_circle_20260928_172242` | only run | 24.37 |
| `free_flight_tracking/figure8/v0p10/geometric_l1_figure8_v0p10_DEC-5.mat` | DEC-5 | figure-8 | 0.10 | Geo-L1 | 2026-10-05 16:37 | `flight_decoupled_l1_figure8_vel010_20261005_163734` | only run | 72.59 |
| `free_flight_tracking/figure8/v0p10/whole_body_l1_figure8_v0p10_WB-3.mat` | WB-3 | figure-8 | 0.10 | Proposed | 2026-10-05 16:24 | `flight_wb_l1_4d_figure8_vel010_20261005_162406` | first of 2 | 37.27 |
| `free_flight_tracking/figure8/v0p13/geometric_l1_figure8_v0p13_DEC-6.mat` | DEC-6 | figure-8 | 0.13 | Geo-L1 | 2026-10-05 16:54 | `flight_decoupled_l1_figure8_vel013_20261005_165431` | only run | 76.86 |
| `free_flight_tracking/figure8/v0p13/whole_body_l1_figure8_v0p13_WB-5.mat` | WB-5 | figure-8 | 0.13 | Proposed | 2026-10-05 17:01 | `flight_wb_l1_4d_figure8_vel013_20261005_170129` | first of 2 | 42.66 |

The bags themselves are kept in `results/experiment_results/free_flight_tracking/<shape>/v<speed>/<same name>/`
on the lab machine; the `.mat` files here hold everything the table and the plots need.

### Which run fills each table cell

| trajectory | speed [m/s] | Proposed | Geo-L1 |
|---|---|---|---|
| circle | 0.10 | not flown | not flown |
| circle | 0.13 | WB-2 | DEC-3 |
| circle | 0.20 | not flown | not flown |
| figure-8 | 0.10 | WB-3 | DEC-5 |
| figure-8 | 0.13 | WB-5 | DEC-6 |
| figure-8 | 0.20 | not flown | not flown |

Not flown on hardware ("--" in the table): circle 0.10 m/s, circle 0.20 m/s, figure-8 0.20 m/s.

### Every scored run of the formal comparison

Same metric definitions as the simulation table.

| run | flown | method | trajectory | speed [m/s] | EE RMSE [mm] | platform RMSE [mm] | EE heading RMSE [deg] | status |
|---|---|---|---|---|---|---|---|---|
| WB-1 | 2026-09-28 17:11 | Proposed | circle | 0.13 | 26.25 | 25.35 | 0.48 | candidate |
| WB-2 | 2026-09-28 17:22 | Proposed | circle | 0.13 | 24.37 | 23.43 | 0.44 | **selected** |
| DEC-1 | 2026-09-28 18:28 | Geo-L1 | circle | 0.13 | 192.05 | 151.29 | 10.31 | not eligible: previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune |
| DEC-2 | 2026-09-28 18:36 | Geo-L1 | circle | 0.13 | 218.44 | 176.84 | 10.31 | not eligible: previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune |
| DEC-3 | 2026-10-02 13:42 | Geo-L1 | circle | 0.13 | 77.45 | 66.19 | 3.48 | **selected** |
| DEC-4 | 2026-10-02 13:42 | Geo-L1 | circle | 0.13 | 72.34 | 58.34 | 3.82 | not eligible: incomplete: the operator returned to SAFETY 0.8 s before the plan ended; flown through a ground-station clock step that reset the L1 estimate |
| WB-3 | 2026-10-05 16:24 | Proposed | figure-8 | 0.10 | 37.27 | 36.76 | 0.46 | **selected** |
| WB-4 | 2026-10-05 16:24 | Proposed | figure-8 | 0.10 | 43.74 | 43.61 | 0.48 | candidate |
| DEC-5 | 2026-10-05 16:37 | Geo-L1 | figure-8 | 0.10 | 72.59 | 66.56 | 3.58 | **selected** |
| DEC-6 | 2026-10-05 16:54 | Geo-L1 | figure-8 | 0.13 | 76.86 | 68.73 | 4.69 | **selected** |
| WB-5 | 2026-10-05 17:01 | Proposed | figure-8 | 0.13 | 42.66 | 42.11 | 0.54 | **selected** |
| WB-6 | 2026-10-05 17:01 | Proposed | figure-8 | 0.13 | 58.19 | 58.31 | 0.48 | candidate |

### Conditions of the selected runs

- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm, total mass 3.746 kg.
- **Feedback:** OptiTrack fused in PX4 EKF2, about 100 Hz; joint encoders.
- **Proposed:**
  - the 2026-09-27 tune: k_x / k_v 50.03 / 12.58, k_R / k_ω 2.134 / 1.567, K_y / D_y 211.9 / 26.82,
    K_ψ / D_ψ 0.2484 / 0.2903;
  - L1 bandwidth ω_c 2.927 / 0.7428 / 0.8479 rad/s (translation / rotation / arm);
  - the arm in torque mode with friction feed-forward and the velocity observer.
- **Geo-L1:**
  - the 2026-10-01 tune: K_p 20.11 (x, y) / 13.5 (z), K_v 11.05 / 10.82, k_R 3.337 / 1.737,
    k_ω 0.9505 / 0.4105;
  - L1: A_s 4.334 / 2.791, ω_c 1.0 rad/s;
  - the arm in position mode.
- **Allocator thrust coefficient:** 4.260431e-05 for both.

### All aerial-manipulator flight-test bags (for reference)

Arm-calibration bench bags and bare-T650 recordings are not listed.

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

## Trajectories (hardware runs: 0.10 and 0.13 m/s only)

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
| `meta` | run tag, method, shape, nominal mean speed, original bag, which run in the bag, flight date, run window (s since the bag's first message), vehicle, feedback, trajectory, selection rule |
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
