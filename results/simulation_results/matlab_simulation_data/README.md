# Simulation results, MATLAB data (free-flight trajectory tracking)

This is the data behind the paper's simulation table (Table II, `tab:sim_free_flight_tracking`).
The folder is self-contained: these files plus MATLAB are all you need.

## Files

```
free_flight_tracking/<shape>/v<speed>/<method>_<shape>_v<speed>_run<k>.mat   one run, struct `run`
free_flight_tracking_rmse.mat     rmse_mean = Table II (mean over runs), rmse_runs = every run
configs/                          the exact controller + plant config each method flew
```

- **`<shape>`:** `circle` or `figure8`.
- **`v<speed>`:** the mean end-effector speed, e.g. `v0p13` = 0.13 m/s.
- **`run<k>`:** the repeat, 1..3.
- **`<method>`:** see the table below.

| method key | table name | method |
|---|---|---|
| `whole_body_l1` | Proposed | whole-body L1 impedance control (this paper) |
| `geometric_l1` | Geo-L1 | geometric control with L1 adaptive augmentation, Cai et al., Control Eng. Pract. 164 (2025) 106418; arm in position mode |
| `modular_adaptive` | MAC | modular adaptive control, Yadav et al., IEEE/ASME Trans. Mechatronics 30(4) (2025) |

```matlab
r = load('free_flight_tracking/circle/v0p13/whole_body_l1_circle_v0p13_run1.mat').run;
plot(r.tracking.t, vecnorm(r.tracking.ee_pos_err_m, 2, 2) * 1e3)        % EE position error [mm]
plot3(r.tracking.ee_pos_ref_m(:,1), r.tracking.ee_pos_ref_m(:,2), r.tracking.ee_pos_ref_m(:,3), '--')
S = load('free_flight_tracking_rmse.mat');
T = struct2table(S.rmse_mean);     % Table II: one row per (shape, speed, method)
R = struct2table(S.rmse_runs);     % every run
```

**Payload pick-and-place** (the paper's `tab:sim_pick_place`): `pick_and_place/`, `pick_and_place_metrics.mat` and `configs/*_sim_pick_and_place.yaml`. See `README_pick_and_place.md`.

## Simulation index

Flown 2026-10-07 21:52:15 to 2026-10-08 01:02:07, in one campaign with one launch condition:
- **Simulator:** Isaac Sim, headless, at real time (RTF 1.000 during every trajectory).
- **Stack:** the same PX4 SITL, planner and arm stack for every method; raw mocap-emulator feedback.
- **Repeats:** 3 runs per setting per method.
- **Gains:** each method used ONE fixed config for all six settings (`configs/`); only the control law
  differs between methods. Geo-L1's arm runs a position-mode servo stack, as in its paper.

Each table cell is the mean over its 3 runs of the per-run RMSE.

| trajectory | speed [m/s] | method | folder | files | EE RMSE per run [mm] | mean (table) [mm] | attempts |
|---|---|---|---|---|---|---|---|
| circle | 0.10 | Proposed | `free_flight_tracking/circle/v0p10/` | `whole_body_l1_circle_v0p10_run1.mat`, `whole_body_l1_circle_v0p10_run2.mat`, `whole_body_l1_circle_v0p10_run3.mat` | 12.3 / 12.0 / 12.5 | 12.24 | 3 (0 hover trips, 0 failed in run) |
| circle | 0.10 | Geo-L1 | `free_flight_tracking/circle/v0p10/` | `geometric_l1_circle_v0p10_run1.mat`, `geometric_l1_circle_v0p10_run2.mat`, `geometric_l1_circle_v0p10_run3.mat` | 42.4 / 44.4 / 45.2 | 44.01 | 3 (0 hover trips, 0 failed in run) |
| circle | 0.10 | MAC | `free_flight_tracking/circle/v0p10/` | `modular_adaptive_circle_v0p10_run1.mat`, `modular_adaptive_circle_v0p10_run2.mat`, `modular_adaptive_circle_v0p10_run3.mat` | 77.8 / 76.4 / 76.9 | 77.04 | 3 (0 hover trips, 0 failed in run) |
| circle | 0.13 | Proposed | `free_flight_tracking/circle/v0p13/` | `whole_body_l1_circle_v0p13_run1.mat`, `whole_body_l1_circle_v0p13_run2.mat`, `whole_body_l1_circle_v0p13_run3.mat` | 14.5 / 15.9 / 14.8 | 15.06 | 3 (0 hover trips, 0 failed in run) |
| circle | 0.13 | Geo-L1 | `free_flight_tracking/circle/v0p13/` | `geometric_l1_circle_v0p13_run1.mat`, `geometric_l1_circle_v0p13_run2.mat`, `geometric_l1_circle_v0p13_run3.mat` | 46.6 / 46.4 / 47.5 | 46.85 | 3 (0 hover trips, 0 failed in run) |
| circle | 0.13 | MAC | `free_flight_tracking/circle/v0p13/` | `modular_adaptive_circle_v0p13_run1.mat`, `modular_adaptive_circle_v0p13_run2.mat`, `modular_adaptive_circle_v0p13_run3.mat` | 74.9 / 75.0 / 75.7 | 75.20 | 5 (2 hover trips, 0 failed in run) |
| circle | 0.20 | Proposed | `free_flight_tracking/circle/v0p20/` | `whole_body_l1_circle_v0p20_run1.mat`, `whole_body_l1_circle_v0p20_run2.mat`, `whole_body_l1_circle_v0p20_run3.mat` | 19.9 / 19.9 / 20.0 | 19.93 | 6 (3 hover trips, 0 failed in run) |
| circle | 0.20 | Geo-L1 | `free_flight_tracking/circle/v0p20/` | `geometric_l1_circle_v0p20_run1.mat`, `geometric_l1_circle_v0p20_run2.mat`, `geometric_l1_circle_v0p20_run3.mat` | 52.1 / 51.9 / 53.4 | 52.48 | 3 (0 hover trips, 0 failed in run) |
| circle | 0.20 | MAC | `free_flight_tracking/circle/v0p20/` | `modular_adaptive_circle_v0p20_run1.mat`, `modular_adaptive_circle_v0p20_run2.mat`, `modular_adaptive_circle_v0p20_run3.mat` | 74.5 / 76.9 / 75.7 | 75.69 | 5 (2 hover trips, 0 failed in run) |
| figure-8 | 0.10 | Proposed | `free_flight_tracking/figure8/v0p10/` | `whole_body_l1_figure8_v0p10_run1.mat`, `whole_body_l1_figure8_v0p10_run2.mat`, `whole_body_l1_figure8_v0p10_run3.mat` | 15.5 / 15.4 / 15.2 | 15.37 | 5 (2 hover trips, 0 failed in run) |
| figure-8 | 0.10 | Geo-L1 | `free_flight_tracking/figure8/v0p10/` | `geometric_l1_figure8_v0p10_run1.mat`, `geometric_l1_figure8_v0p10_run2.mat`, `geometric_l1_figure8_v0p10_run3.mat` | 42.1 / 42.0 / 44.4 | 42.86 | 3 (0 hover trips, 0 failed in run) |
| figure-8 | 0.10 | MAC | `free_flight_tracking/figure8/v0p10/` | `modular_adaptive_figure8_v0p10_run1.mat`, `modular_adaptive_figure8_v0p10_run2.mat`, `modular_adaptive_figure8_v0p10_run3.mat` | 77.7 / 77.7 / 77.5 | 77.66 | 4 (1 hover trip, 0 failed in run) |
| figure-8 | 0.13 | Proposed | `free_flight_tracking/figure8/v0p13/` | `whole_body_l1_figure8_v0p13_run1.mat`, `whole_body_l1_figure8_v0p13_run2.mat`, `whole_body_l1_figure8_v0p13_run3.mat` | 19.3 / 18.7 / 18.1 | 18.70 | 5 (2 hover trips, 0 failed in run) |
| figure-8 | 0.13 | Geo-L1 | `free_flight_tracking/figure8/v0p13/` | `geometric_l1_figure8_v0p13_run1.mat`, `geometric_l1_figure8_v0p13_run2.mat`, `geometric_l1_figure8_v0p13_run3.mat` | 44.9 / 45.4 / 45.6 | 45.29 | 3 (0 hover trips, 0 failed in run) |
| figure-8 | 0.13 | MAC | `free_flight_tracking/figure8/v0p13/` | `modular_adaptive_figure8_v0p13_run1.mat`, `modular_adaptive_figure8_v0p13_run2.mat`, `modular_adaptive_figure8_v0p13_run3.mat` | 76.2 / 76.9 / 75.5 | 76.18 | 3 (0 hover trips, 0 failed in run) |
| figure-8 | 0.20 | Proposed | `free_flight_tracking/figure8/v0p20/` | `whole_body_l1_figure8_v0p20_run1.mat`, `whole_body_l1_figure8_v0p20_run2.mat`, `whole_body_l1_figure8_v0p20_run3.mat` | 25.4 / 24.7 / 25.6 | 25.26 | 3 (0 hover trips, 0 failed in run) |
| figure-8 | 0.20 | Geo-L1 | `free_flight_tracking/figure8/v0p20/` | `geometric_l1_figure8_v0p20_run1.mat`, `geometric_l1_figure8_v0p20_run2.mat`, `geometric_l1_figure8_v0p20_run3.mat` | 59.8 / 60.2 / 61.1 | 60.40 | 3 (0 hover trips, 0 failed in run) |
| figure-8 | 0.20 | MAC | `free_flight_tracking/figure8/v0p20/` | `modular_adaptive_figure8_v0p20_run1.mat`, `modular_adaptive_figure8_v0p20_run2.mat`, `modular_adaptive_figure8_v0p20_run3.mat` | 74.0 / 74.4 / 73.1 | 73.84 | 4 (1 hover trip, 0 failed in run) |

Some attempts failed while hovering in DIRECT, before the trajectory started; they were re-flown. No
trajectory failed after starting. Totals: Proposed 7 of 25 attempts; Geo-L1 0 of 18 attempts; MAC 6 of 24 attempts.
- **Proposed:** a growing roll oscillation at about 1.55 Hz, a simulator-only mode near the rotor-lag pole.
- **MAC:** a roll and pitch divergence at about 2 Hz shortly after entering DIRECT.

## Trajectories

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

## Plant (identical for every run and every method)

The simulated plant mirrors the uncertainties identified from the hardware flights:
- **Motor delay:** first-order rotor lag, λ = 10.03 s⁻¹.
- **Model mismatch:** thrust coefficient ×1.037 and yaw-torque coefficient scaled at the plant; the
  airframe CoM shifted 17.85 mm.
- **Battery sag:** the thrust coefficient falls 3.6 %/min from lift-off.
- **Standing wrench bias:** force (0.55, −0.50, 0) N and torque (0, 0, −0.095) N·m in the body frame.
- **Imperfect joint actuation:**
  - gearbox friction, ×(1.0, 0.70, 0.65, 1.5) per joint;
  - current-loop residual, (22, 15, 24, 23) mN·m rms;
  - reported joint velocity lagged 48 ms and quantised to 0.024 rad/s;
  - joint torque limit 3 N·m.
- **Feedback:** emulated motion capture at 60 Hz with noise.

All plant keys, as flown (`sim_*` in `configs/whole_body_l1_4d_mirror_sim.yaml`; the same in all three):

```
sim_plant_kf_scale: 1.037
sim_plant_kf_sag_per_min: 0.036
sim_plant_km_scale: 0.2333
sim_plant_rotor_lambda: 10.0265
sim_wall_clock_compensation: true
sim_plant_mass_scale: 1.0
sim_plant_inertia_scale: 1.0
sim_plant_com_shift_x: -0.017854
sim_plant_com_shift_y: 0.0
sim_plant_com_shift_z: 0.0
sim_plant_force_bias_x: 0.55
sim_plant_force_bias_y: -0.50
sim_plant_force_bias_z: 0.0
sim_plant_torque_bias_x: 0.0
sim_plant_torque_bias_y: 0.0
sim_plant_torque_bias_z: -0.095
sim_arm_current_noise_enable: true
sim_arm_current_noise_a_j1: 0.0096
sim_arm_current_noise_a_j2: 0.0062
sim_arm_current_noise_a_j3: 0.0096
sim_arm_current_noise_a_j4: 0.0096
sim_arm_current_noise_bw_hz: 5.0
sim_arm_current_noise_seed: 0
sim_arm_friction_scale_j1: 1.0
sim_arm_friction_scale_j2: 0.70
sim_arm_friction_scale_j3: 0.65
sim_arm_friction_scale_j4: 1.5
sim_arm_friction_width: 0.015
sim_arm_mass_scale: 1.0
sim_arm_vel_lag_s: 0.048
sim_arm_vel_quant_rad_s: 0.024
sim_ee_marker_cube: false
sim_ee_marker_cube_mass_kg: 0.03
sim_feedback_mocap_rate_hz: 60.0
sim_feedback_pos_noise_m: 0.0005
sim_feedback_vel_noise_mps: 0.022
sim_feedback_noise_seed: 0
```

Configs here: `configs/whole_body_l1_4d_mirror_sim.yaml`, `configs/geometric_l1_mirror_sim.yaml`, `configs/modular_adaptive_mirror_sim.yaml`.

## The struct `run`

Every run file holds one struct, `run`:

| field | content |
|---|---|
| `meta` | method, shape, nominal mean speed, lap time, laps, circle radius / figure-8 half-axes, arm sweep (q2 centre, amplitude, period, fold), plant description, controller yaml, driver arguments, flight time, minimum RTF, run window |
| `rmse` | this run's numbers: `platform_pos_xyz_mm` (x y z), `platform_pos_mm` (3-D norm), `platform_roll_deg`, `platform_pitch_deg`, `platform_heading_deg`, `ee_pos_xyz_mm`, `ee_pos_mm`, `ee_heading_deg`, `joints_deg` (q1..q4), plus `com_pos_mm`, `ee_pos_max_mm`, `tilt_max_deg` |
| `tracking` | 100 Hz over the trajectory run, `t` in s since the run started. Measured, reference (`_ref_`) and error (`_err_`) for the platform position (`platform_pos_*`) and attitude (`platform_quat_xyzw`, `platform_att_err_deg`, `platform_heading_*`, `platform_tilt_deg`), the system CoM (`com_pos_*`), the end-effector position (`ee_pos_*`) and heading (`ee_heading_*`), and the joints (`q_deg`, `q_ref_deg`, `q_err_deg`) |
| `raw` | every recorded stream, time in s on the flight driver's clock: `odom` (position, velocity, attitude), `joints` (q, model convention), `reference` (the planner's whole-body reference: CoM chain, model-frame headings, EE position and heading, joint reference, airframe reference `x_b`), `planner_ee` (the planner's own current / reference EE), `arm_reference`, `controller_debug` (the method's debug array), `events` (the driver's event log) |

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
