# Simulation results, MATLAB data (payload pick-and-place)

This is the data behind the paper's pick-and-place table (`tab:sim_pick_place`) and its figures.
It is one task folder of `matlab_simulation_data/`, self-contained: these files plus MATLAB are all
you need. The free-flight data are in the separate `free_flight_tracking/` folder beside it, with
the same plant, the same `run` struct layout and the same notation.

## Files

```
<method>_pnp_run<k>.mat                  one run, struct `run`
pick_and_place_metrics.mat               metrics_mean = the table (mean over runs), metrics_runs = every run,
                                         metrics_phase = every run x phase, phases = the common phase table,
                                         window_s = the evaluation window, L_arm_m = L
configs/whole_body_l1_4d_sim_pick_and_place.yaml   the whole-body controller + planner + plant, as flown
configs/geometric_l1_sim_pick_and_place.yaml       the decoupled (Geo-L1) controller + plant, as flown;
                                                   it flew the planner block of the whole-body file
```

- **`<method>`:** `whole_body_l1` = Proposed, `geometric_l1` = Geo-L1 (see `README.md`).
- **MAC (`modular_adaptive`):** no run file. It did not complete the task in any of its six attempts on
  2026-10-08 (the arm module saturated and the platform flipped within 2 s of entering its control law),
  so the table reports no entry for it.

```matlab
S = load('pick_and_place_metrics.mat');
P = S.phases;                                    % the six phases, common to every run
r = load('whole_body_l1_pnp_run1.mat').run;
g = load('geometric_l1_pnp_run1.mat').run;
figure; hold on
for i = 1:numel(P.start_s)                       % phase bands, alternating shades
    c = 0.90 + 0.07*mod(i+1, 2);
    patch([P.start_s(i) P.end_s(i) P.end_s(i) P.start_s(i)], [0 0 400 400], c*[1 1 1], 'EdgeColor', 'none');
    text(mean([P.start_s(i) P.end_s(i)]), 390, P.label{i}, 'HorizontalAlignment', 'center');
end
w = r.tracking.in_window > 0;  plot(r.tracking.t(w), 1e3 * r.tracking.eps_uav_m(w))   % ||eps_UAV|| [mm]
w = g.tracking.in_window > 0;  plot(g.tracking.t(w), 1e3 * g.tracking.eps_uav_m(w))
xlim([0 S.window_s]); xlabel('Time (s)'); ylabel('||\epsilon_{UAV}|| (mm)')
yyaxis right; ylim([0 400] / (1e3 * S.L_arm_m)); ylabel('\rho_{UAV}')
T = struct2table(S.metrics_mean);                % the table, one row per method

% 3-D: references (dashed), measured platform and end effector, the basket (ground truth)
w = r.tracking.in_window > 0;  tr = r.tracking;
figure; hold on; grid on; axis equal; view(-50, 28)
plot3(tr.platform_pos_ref_m(w,1), tr.platform_pos_ref_m(w,2), tr.platform_pos_ref_m(w,3), 'k--')
plot3(tr.ee_pos_ref_m(w,1), tr.ee_pos_ref_m(w,2), tr.ee_pos_ref_m(w,3), '--', 'Color', [0.45 0.45 0.45])
plot3(tr.platform_pos_m(w,1), tr.platform_pos_m(w,2), tr.platform_pos_m(w,3))
plot3(tr.ee_pos_m(w,1), tr.ee_pos_m(w,2), tr.ee_pos_m(w,3), ':')
tp = r.raw.payload.t - r.meta.t0_driver_s;  wp = tp >= 0 & tp <= S.window_s;   % raw streams: driver clock
plot3(r.raw.payload.pos_m(wp,1), r.raw.payload.pos_m(wp,2), r.raw.payload.pos_m(wp,3))

% the error grid: platform / end effector xyz in mm, headings and joints in deg
E = {1e3*tr.platform_pos_err_m, tr.platform_att_err_deg(:,3), 1e3*tr.ee_pos_err_m, tr.ee_heading_err_deg, tr.q_err_deg};
```

## Timing: one timetable for every run

The flight driver starts every step at the same mission time on every run and every controller
(`pnp_mission_v2.py --timetable`, slots 5.3, 18.3, 22.3, 25, 18.3 s). The phases are
contiguous: each runs from its first step's start to the next phase's start, so the short hovers
between steps belong to the phase they end. `t = 0` is the start of the first transit. The phase table
below is the mean over the runs. Every run's phase starts agree with it to 0.01 s.

| # | phase | start [s] | end [s] | duration [s] | what happens |
|---|---|---|---|---|---|
| 1 | Go to start | 0.00 | 5.31 | 5.31 | fly from the hover to the start waypoint; hover |
| 2 | Pick | 5.31 | 26.87 | 21.56 | approach behind the stem, hover 2 s (descent trim), slide in, close the gripper around the stem, hold, lift (exit) |
| 3 | To place start | 26.87 | 45.92 | 19.05 | carry the basket to the place-start waypoint; hover |
| 4 | Place | 45.92 | 70.92 | 25.01 | approach above the place hat, hover 2 s, trim sideways + 2 s settle + descend, open, back out and climb (exit); hover |
| 5 | To land start | 70.92 | 89.23 | 18.31 | fly to the land-start waypoint; hover |
| 6 | Land | 89.23 | 92.48 | 3.25 | the approach to the landing point (the mission's last planned leg) |

- **Window:** every metric covers the whole mission, 0 to 92.48 s. `run.tracking.in_window`
  marks it; `run.tracking.phase_id` (1..6) says which phase each sample is in.
- **Constant pauses:** about 2 s of hover after each transit, needed before the planner accepts the
  next precision step, and 2 s above each target, where the planner measures the descent trim.
  The place descent always trims sideways first (planner `pick_place_descent_trim_first_min: 0`), so
  it lasts the same on every run.

## Simulation index

Flown 2026-10-09 10:01 to 2026-10-09 10:19 (attempt end times), headless Isaac Sim at real time, raw mocap-emulator feedback, the same
plant, planner, scene and task for every method.

| method | file | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | placed off axis [mm] |
|---|---|---|---|---|---|---|---|
| Proposed | `whole_body_l1_pnp_run1.mat` | 140.9 | 0.381 | 25.6 | 0.069 | 27.5 | 25.3 |
| Proposed | `whole_body_l1_pnp_run2.mat` | 151.9 | 0.411 | 26.7 | 0.072 | 28.7 | 23.7 |
| Geo-L1 | `geometric_l1_pnp_run1.mat` | 314.5 | 0.850 | 66.7 | 0.180 | 67.6 | 27.1 |
| Geo-L1 | `geometric_l1_pnp_run2.mat` | 310.2 | 0.838 | 65.7 | 0.178 | 67.0 | 30.2 |

The table is the mean over the runs:

| method | runs | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] |
|---|---|---|---|---|---|---|
| Proposed | 2 | 146.4 | 0.396 | 26.1 | 0.071 | 28.1 |
| Geo-L1 | 2 | 312.3 | 0.844 | 66.2 | 0.179 | 67.3 |

Attempts (a run is kept only if it completed, placed the basket and kept every step on its slot):

| method | attempts | kept | why the others were not kept |
|---|---|---|---|
| Proposed | 3 | 2 | the basket slid off the hook (fingers) mid-carry, 35 s into the mission, and fell to the floor (placed False) |
| Geo-L1 | 2 | 2 | -- |

## Scene and task

- **Field:** two 1 m pillars, PICK at (1.0, 1.0) m and PLACE at (-1.0, -1.0) m, each with the printed
  hat (platform top 1.008 m).
- **Payload:** the CAD basket with a wire hanger, 200 g, starting on the PICK hat.
- **Grasp:** a hook. The claw slides in level under the hanger's arch, the fingers close around its
  3 mm stem, and the lift hangs the basket on them. At the place the claw descends with the jaws
  closed, opens, and backs out.
- **Arm poses:** pick and place [0, 32, 38, 0] deg, carry [12, 38, 42, 0] deg.
- **Plant:** the free-flight mirror plant -- motor delay, model mismatch, battery sag, a standing
  wrench bias, imperfect joint actuation and emulated mocap feedback (the free-flight README explains
  each). Every `sim_*` key as flown (`configs/whole_body_l1_4d_sim_pick_and_place.yaml`; the decoupled
  file carries the same):

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

## The struct `run`

| field | content |
|---|---|
| `meta` | method, run index, scene, plant, `phases` (key, label, start_s, end_s of this run), `timetable_slots_s`, `late_steps` (empty), `window`, `L_arm_m`, `base_com_model_m`, `t0_driver_s` |
| `metrics` | `window` = this run's table numbers over the window; `per_phase.<key>` = the same per phase; `whole_recording` = over every sample |
| `tracking` | 100 Hz, `t` in s since the start of the mission; `in_window`, `phase_id`; measured, reference (`_ref_`) and error (`_err_`) for the platform position, `eps_uav_m`, `rho_uav`, the platform attitude, the system CoM, the end-effector position and heading, and the joints |
| `raw` | every recorded stream on the driver's clock (subtract `meta.t0_driver_s` to get `tracking.t`): `odom`, `joints`, `reference` (the planner's whole-body reference), `payload` (ground truth), `claw_truth`, `marks` (every step's start/end), `events` |

Units: positions in m (world ENU, z up) unless the field name says mm; angles in deg unless it says rad.
Errors are measured − reference. `platform_att_err_deg` columns are `[roll pitch heading]`.

## Paper notation

| paper symbol | meaning | summary field (`metrics_mean`) | run file |
|---|---|---|---|
| ‖ε<sub>UAV</sub>‖ | ‖r<sub>UAV</sub><sup>ref</sup> − r<sub>UAV</sub>‖, the platform's deviation from its reference position [mm] | `eps_uav_max_mm`, `eps_uav_rms_mm` | `tracking.eps_uav_m` [m] |
| ρ<sub>UAV</sub> | ‖ε<sub>UAV</sub>‖ / L, L = 0.370 m (the arm's reach) | `rho_uav_max`, `rho_uav_rms` | `tracking.rho_uav` |
| r<sub>0,x</sub>, r<sub>0,y</sub>, r<sub>0,z</sub> | platform position [mm] | `platform_x_mm`, `platform_y_mm`, `platform_z_mm` | `tracking.platform_pos_err_m(:,1:3)` [m] |
| ψ<sub>0</sub>, φ<sub>0</sub>, θ<sub>0</sub> | platform heading, roll, pitch [deg] | `platform_heading_deg`, `platform_roll_deg`, `platform_pitch_deg` | `tracking.platform_att_err_deg(:,[3 1 2])` |
| r<sub>e,x</sub>, r<sub>e,y</sub>, r<sub>e,z</sub> | end-effector position [mm] | `ee_x_mm`, `ee_y_mm`, `ee_z_mm` | `tracking.ee_pos_err_m(:,1:3)` [m] |
| ψ<sub>e</sub> | end-effector heading [deg] | `ee_heading_deg` | `tracking.ee_heading_err_deg` |
| q<sub>1</sub> … q<sub>4</sub> | joint angles [deg] | `q1_deg` … `q4_deg` | `tracking.q_err_deg(:,1:4)` |

- **Reference:** the planner's dynamically compatible whole-body reference, the same for every method.
  The platform reference x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>) is r<sub>UAV</sub><sup>ref</sup>.
- **max / rms:** over the window; the table's other columns are RMS errors.
