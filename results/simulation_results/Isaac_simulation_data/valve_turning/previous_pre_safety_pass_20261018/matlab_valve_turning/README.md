# Simulation results, MATLAB data (valve turning)

This is the data behind the paper's valve-turning table (`tab:sim_valve_turn`) and its figures: the
second physical-interaction task, flown in Isaac Sim by the proposed whole-body L1 impedance controller
(5 runs of one mission). It is one task folder of `matlab_simulation_data/`, self-contained, with the
same plant, the same `run` struct layout and the same notation as the box-pushing folder beside it; where
that folder has `box`, this one has `valve` (the beam's rotation) and `raw.rotor`.

## Files

```
<method>_turn_run<k>.mat                 one run, struct `run`
valve_turning_metrics.mat                metrics_mean = the table (mean over runs), metrics_runs = every run,
                                         metrics_phase = every run x phase, metrics_impedance = every run x
                                         contact window, phases = the common phase table, window_s, L_arm_m
configs/<method>_sim_valve_turning.yaml  the controller + planner (push-and-pull block, turn form) + plant, as flown
reference_outputs/                       the same table and figures drawn by the Python scorer
                                         (valve_turning_sim.csv / .tex, sim_valve_turn_*.png)
```

- **`<method>`:** `whole_body_l1` = Proposed (whole-body 4-D L1 impedance); `coordinated_admittance` =
  Liu et al., CEP 146 (2024) 105898 (decoupled UAV PD + momentum wrench estimator + admittance on the
  end-effector reference + force-adaptive kinematic MPC; position-mode arm), implemented as an independent
  comparison rig -- its platform / arm references in `run` are that rig's own planner commands.

## Quick start in MATLAB

```matlab
S = load('valve_turning_metrics.mat');
T = struct2table(S.metrics_mean)                 % THE TABLE: one row per method, mean over the runs
R = struct2table(S.metrics_runs);                % every run (the spread between runs)
P = S.phases;                                    % the eight phases, common to every run
r = load('whole_body_l1_turn_run1.mat').run;
tr = r.tracking;  w = tr.in_window > 0;          % 100 Hz, t = 0 at the start of go to start

% (1) the beam against its plan, and the platform deviation
figure; tiledlayout(2, 1)
nexttile; plot(tr.t(w), tr.beam_angle_deg(w) - tr.beam_angle_deg(find(tr.phase_id == 4, 1)), tr.t(w), tr.beam_angle_plan_deg(w) - tr.beam_angle_plan_deg(find(tr.phase_id == 4, 1)), '--')
ylabel('beam angle (deg)'); legend('measured', 'plan')
nexttile; plot(tr.t(w), 1e3 * tr.eps_uav_m(w)); ylabel('||\epsilon_{UAV}|| (mm)'); xlabel('Time (s)')

% (2) the contact force and the impedance residual over the contact window
im = r.impedance;  W = im.windows;
figure; tiledlayout(3, 1)
nexttile; plot(im.t, im.F_ext(:,1:3)); ylabel('F_{ext} (N)'); legend('x', 'y', 'z')
nexttile; plot(im.t, im.e_imp(:,1:3)); ylabel('e_{imp} (N)')
nexttile; plot(im.t, vecnorm(im.e_imp(:,1:3), 2, 2)); ylabel('||e_{imp}|| (N)'); xlabel('Time (s)')

% (3) 3-D: references (dashed), measured platform and end effector, the claw's circle about the axis
figure; hold on; grid on; axis equal; view(-50, 28)
plot3(tr.platform_pos_ref_m(w,1), tr.platform_pos_ref_m(w,2), tr.platform_pos_ref_m(w,3), 'k--')
plot3(tr.ee_pos_ref_m(w,1), tr.ee_pos_ref_m(w,2), tr.ee_pos_ref_m(w,3), '--', 'Color', [0.45 0.45 0.45])
plot3(tr.platform_pos_m(w,1), tr.platform_pos_m(w,2), tr.platform_pos_m(w,3))
plot3(tr.ee_pos_m(w,1), tr.ee_pos_m(w,2), tr.ee_pos_m(w,3), ':')
tv = r.meta.task_values;  th = linspace(0, 2*pi, 90);
plot3(tv.axis_xy_m(1) + tv.fin_radius_m*cos(th), tv.axis_xy_m(2) + tv.fin_radius_m*sin(th), (tv.table_top_z_m + tv.grasp_height_above_table_m)*ones(size(th)), 'Color', [0.64 0.08 0.18])

% (4) one phase's numbers, e.g. the turn, for every run
Ph = struct2table(S.metrics_phase);  Ph(strcmp(Ph.phase, 'push'), :)
```

## Timing: one timetable for every run

The flight driver starts every step at the same mission time on every run
(`pl_mission.py --timetable`, slots 11, 2.5, 7, 39, 3, 8.5, 40 s, each from the previous step's
start; `meta.timetable_slots_s` names them). The phases are contiguous. Only the GRASP phase's own length
varies (the claw's arrival within 2 cm / 3 mm across the fin, then the jaws' close).
`t = 0` is the start of the Proposed rig's first transit; the methods are ALIGNED AT READY
(`meta.t_shift_s` is the shift applied).
The phase table below is the mean over the runs; every run's phase starts from Ready on agree with it to 0.72 s.

| # | phase | start [s] | end [s] | duration [s] | what happens |
|---|---|---|---|---|---|
| 1 | Go to start | -7.60 | 11.00 | 18.60 | fly from the hover to the start waypoint (the claw 0.12 m behind and 0.10 m above the fin, the nose along the turn's tangent, the arm already in the turn pose); hover |
| 2 | Ready | 11.00 | 18.01 | 7.00 | open the gripper, hover (CoM-anchored), descend diagonally onto the fin; the claw world-held from its arrival |
| 3 | Grasp | 18.01 | 19.10 | 1.10 | wait for the claw within 2 cm / 3 mm across the fin, close the gripper on the fin (the jaws' stall) |
| 4 | Turn | 19.10 | 58.11 | 39.01 | the law in CONTACT: 1.5 s settle, then the 360 deg turn about the bearing axis over 36 s (the claw on its 0.160 m circle, the base and heading rotating with it, the arm held); hold on the fin |
| 5 | Release | 58.11 | 61.12 | 3.01 | Exit To Turn: the planner releases the claw (its reference back to the CoM anchor, the contact phase off), then the gripper opens; the exit starts on its OPEN report |
| 6 | Exit | 61.12 | 69.62 | 8.51 | climb 0.20 m with the jaws open, fold the arm home; hover |
| 7 | Go to land start | 69.62 | 109.31 | 39.69 | one planner transition to the landing hover, heading turning clockwise as typed; hover |
| 8 | Land | 109.31 | 109.31 | 0.00 | straight down to the land hover at z 0.8 m (the planner's last leg; the touchdown after it is not scored) |

- **Window:** every metric covers the whole mission, 0 to 109.31 s. `run.tracking.in_window`
  marks it; `run.tracking.phase_id` (1..8) says which phase each sample is in (4 = the turn).
- **Contact windows** (`run.impedance.windows`, `run.metrics.impedance.<window>`; the names are the shared
  scorer's): CONTACT = ready end to exit start (the law's contact phase, the claw world-held); HOLD = the grip
  and the 1.5 s settle; SLIDE = **the 36 s turn**; RELEASE = turn end to exit start (the jaws open).

## Simulation index

Flown 2026-10-18 02:36 to 2026-10-18 03:24 (attempt end times), headless Isaac Sim at real time, raw mocap-emulator feedback.

| method | file | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | e_imp turn rms [N] | e_imp contact rms [N] | beam turned [deg] |
|---|---|---|---|---|---|---|---|---|---|
| Proposed | `whole_body_l1_turn_run1.mat` | 49.8 | 0.135 | 19.3 | 0.052 | 13.3 | 0.917 | 0.923 | 360.4 |
| Proposed | `whole_body_l1_turn_run2.mat` | 59.0 | 0.159 | 21.3 | 0.057 | 13.6 | 1.019 | 1.017 | 363.5 |
| Proposed | `whole_body_l1_turn_run3.mat` | 57.9 | 0.156 | 20.9 | 0.057 | 14.1 | 0.992 | 0.976 | 362.6 |
| Coordinated [Liu 2024] | `coordinated_admittance_turn_run1.mat` | 164.1 | 0.443 | 55.6 | 0.150 | 46.6 | 9.611 | 9.110 | 355.4 |
| Coordinated [Liu 2024] | `coordinated_admittance_turn_run3.mat` | 177.4 | 0.479 | 55.1 | 0.149 | 55.5 | 11.502 | 10.781 | 479.2 |

The table is the mean over the runs:

| method | runs | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | e_imp turn rms [N] | e_imp contact rms [N] | beam turned [deg] |
|---|---|---|---|---|---|---|---|---|---|
| Proposed | 3 | 55.6 | 0.150 | 20.5 | 0.055 | 13.7 | 0.976 | 0.972 | 362.2 |
| Coordinated [Liu 2024] | 2 | 170.7 | 0.461 | 55.3 | 0.149 | 51.0 | 10.557 | 9.945 | 417.3 |

The impedance residual per contact window (`metrics_impedance`; SLIDE = the turn):

| run | window | T [s] | e_imp rms [N] | e_imp max [N] | x / y / z rms [N] | heading rms [N.m] | F_ext rms [N] | e_y rms [mm] |
|---|---|---|---|---|---|---|---|---|
| `coordinated_admittance_turn_run1` | CONTACT | 43.2 | 9.110 | 23.820 | 4.171 / 8.064 / 0.746 | 0.2314 | 2.02 | 56.1 |
| `coordinated_admittance_turn_run1` | HOLD | 2.7 | 3.538 | 5.262 | 0.231 / 3.524 / 0.201 | 0.0453 | 0.48 | 24.3 |
| `coordinated_admittance_turn_run1` | SLIDE | 36.0 | 9.611 | 23.820 | 4.562 / 8.434 / 0.645 | 0.2517 | 2.17 | 58.7 |
| `coordinated_admittance_turn_run1` | RELEASE | 4.5 | 7.113 | 14.407 | 0.954 / 6.910 / 1.391 | 0.0860 | 1.27 | 48.2 |
| `coordinated_admittance_turn_run3` | CONTACT | 43.4 | 10.781 | 46.665 | 6.077 / 8.824 / 1.195 | 0.1642 | 1.99 | 79.4 |
| `coordinated_admittance_turn_run3` | HOLD | 2.9 | 2.926 | 4.544 | 0.388 / 2.891 / 0.232 | 0.0526 | 0.58 | 21.6 |
| `coordinated_admittance_turn_run3` | SLIDE | 36.0 | 11.502 | 46.665 | 6.663 / 9.291 / 1.260 | 0.1800 | 2.18 | 81.4 |
| `coordinated_admittance_turn_run3` | RELEASE | 4.5 | 7.712 | 16.199 | 1.437 / 7.506 / 1.035 | 0.0000 | 0.00 | 86.8 |
| `whole_body_l1_turn_run1` | CONTACT | 44.6 | 0.923 | 2.995 | 0.629 / 0.561 / 0.376 | 0.0756 | 0.87 | 5.9 |
| `whole_body_l1_turn_run1` | HOLD | 4.1 | 0.961 | 2.995 | 0.363 / 0.466 / 0.758 | 0.0552 | 0.42 | 5.9 |
| `whole_body_l1_turn_run1` | SLIDE | 36.0 | 0.917 | 2.026 | 0.658 / 0.573 / 0.281 | 0.0719 | 0.93 | 5.9 |
| `whole_body_l1_turn_run1` | RELEASE | 4.5 | 0.932 | 1.754 | 0.581 / 0.538 / 0.491 | 0.1122 | 0.60 | 5.2 |
| `whole_body_l1_turn_run2` | CONTACT | 45.0 | 1.017 | 3.351 | 0.599 / 0.665 / 0.484 | 0.0889 | 1.15 | 6.6 |
| `whole_body_l1_turn_run2` | HOLD | 4.4 | 0.739 | 2.612 | 0.235 / 0.439 / 0.547 | 0.0474 | 0.30 | 4.1 |
| `whole_body_l1_turn_run2` | SLIDE | 36.0 | 1.019 | 2.359 | 0.644 / 0.689 / 0.384 | 0.0911 | 1.27 | 6.8 |
| `whole_body_l1_turn_run2` | RELEASE | 4.5 | 1.224 | 3.351 | 0.454 / 0.649 / 0.933 | 0.1015 | 0.64 | 7.4 |
| `whole_body_l1_turn_run3` | CONTACT | 44.8 | 0.976 | 2.807 | 0.635 / 0.636 / 0.379 | 0.0729 | 0.97 | 5.5 |
| `whole_body_l1_turn_run3` | HOLD | 4.2 | 0.753 | 1.757 | 0.276 / 0.501 / 0.489 | 0.0747 | 0.47 | 5.3 |
| `whole_body_l1_turn_run3` | SLIDE | 36.0 | 0.992 | 2.039 | 0.682 / 0.650 / 0.312 | 0.0714 | 1.05 | 5.4 |
| `whole_body_l1_turn_run3` | RELEASE | 4.5 | 1.030 | 2.807 | 0.472 / 0.643 / 0.651 | 0.0826 | 0.51 | 6.4 |

The valve and the arm (`metrics.valve`, `metrics.window`):

| run | beam turned in the turn step [deg] | follow-through after release [deg] | beam − plan rms / max [deg] | claw radius about the axis [mm] | q2 / q3 margin to the stops in the turn [deg] | on a stop [% of the turn] | vehicle tilt max [deg] | q2 max / q3 min, release + exit [deg] |
|---|---|---|---|---|---|---|---|---|
| `whole_body_l1_turn_run1` | 360.4 | +2.4 | 1.76 / 4.72 | 160.1 ± 0.7 | 4.2 / 5.2 | 0.0 | 6.4 | 39.0 / 15.0 |
| `whole_body_l1_turn_run2` | 363.5 | +2.2 | 2.12 / 5.21 | 160.4 ± 0.4 | 0.0 / -0.0 | 6.0 | 5.6 | 39.3 / 15.7 |
| `whole_body_l1_turn_run3` | 362.6 | +3.2 | 3.06 / 6.90 | 160.2 ± 0.6 | 0.0 / -0.0 | 4.5 | 5.4 | 39.6 / 15.3 |
| `coordinated_admittance_turn_run1` | 355.4 | +36.2 | 22.61 / 57.20 | 159.1 ± 2.1 | 21.9 / 24.2 | 0.0 | 5.9 | 41.3 / 17.2 |
| `coordinated_admittance_turn_run3` | 479.2 | +0.0 | 30.22 / 125.30 | 161.7 ± 22.3 | 19.4 / 22.8 | 0.0 | 13.5 | 41.3 / 19.2 |

The stops are q2 = −20 deg (the real arm's range, enforced in the scene) and q3 = +50 deg (the asset's); the
margin is the smallest distance to either during the turn step. With the claw world-held on its circle the
base's lead over its moving reference is absorbed by the arm, so a margin of 0 means the arm sat on a stop.

Attempts (a run is kept only if it completed, turned the beam >= 350 deg and kept every step on its slot):

| method | attempts | kept | why the others were not kept |
|---|---|---|---|
| Coordinated [Liu 2024] | 7 | 2 | [  96.62s] ABORT: left DIRECT while push complete; [  96.97s] ABORT: left DIRECT while push complete; [ 111.17s] ABORT: left DIRECT while release; beam turned 273.6 deg; beam turned 337.2 deg |
| Proposed | 4 | 3 | [  67.81s] ABORT: left DIRECT while push complete |

The failed attempts (kept whole under `results/simulation_results/Isaac_simulation_data/valve_turning/failed/`):

| attempt | what stopped it | when | afterwards |
|---|---|---|---|
| `coordinated_admittance_turn_run2_attempt1` | left DIRECT while push complete | +24.3 s into the Turn step | **the vehicle flipped** (tilt max 180 deg, final height 0.13 m) |
| `coordinated_admittance_turn_run2_attempt2` |  | +95.7 s into the Turn step | landed upright (tilt max 5 deg, final height 0.24 m) |
| `coordinated_admittance_turn_run2_attempt3` | left DIRECT while push complete | +24.2 s into the Turn step | landed upright (tilt max 30 deg, final height 0.24 m) |
| `coordinated_admittance_turn_run3_attempt1` |  | +95.7 s into the Turn step | landed upright (tilt max 5 deg, final height 0.24 m) |
| `coordinated_admittance_turn_run3_attempt2` | left DIRECT while release | +38.4 s into the Turn step | landed upright (tilt max 42 deg, final height 0.25 m) |
| `whole_body_l1_turn_run1_attempt1` | left DIRECT while push complete | +13.1 s into the Turn step | landed upright (tilt max 30 deg, final height 0.24 m) |

## Scene and task

- **Table:** the HOOBRO console table (1.000 x 0.150 x 0.800 m), its length along world y: x 1.125-1.275 m, y -0.5..0.5 m, top at 0.80 m.
- **Valve:** the user's printed mechanism (Valve_Assembly.stl, 2026-10-15): a 150 x 50 x 20 mm plate with a 608
  bearing, bolted to the table with its axis at (1.20, 0.00) m; on the bearing a 225 mm beam
  carrying a 5 x 40 x 130 mm fin handle at radius 0.160 m, the fin top 204 mm above the
  table. The rotor is flown at the printed part's **204 g** (the pick-and-drop payload's density, same
  printer and PLA). **Bearing: Coulomb friction 0.048 N.m** (= 0.30 N at the fin) + viscous
  0 N.m.s/rad -- a CONSERVATIVE value: the real bearing, pulled by a 10 N spring gauge at the fin
  (2026-10-17), never passed the gauge's first 0.2 N division (<= 0.032 N.m); the sim flies 1.5x that bound.
  The sim claw cannot close below ~19 mm, so the fin's bare part carries an invisible 20 mm grip shim (the real
  gripper's foam pads close on the 5 mm fin).
- **Task:** grasp the fin from behind and above (go to start, ready, grasp), turn the beam 360 deg about the
  bearing axis in 36 s (a smooth start-stop rotation after a 1.5 s settle: the claw on its circle,
  the base on a larger one behind it, the heading turning with them, the arm held still), release, climb out,
  fly to the landing hover, descend. The whole-body law's contact phase (chi) is CONTACT from the jaws' grip to the end of the turn.
- **Arm pose in the turn:** [0, 10, 20, 0] deg (fold 30 deg; the fin is low -- 179 mm above the table
  at the grasp -- so a more folded arm would put the T650 skids onto the beam); the real arm's q2 range [-20, 45] deg
  is enforced as hard stops in the scene.
- **Safety guards (on in every attempt; none tripped in the kept runs; 6 failed attempts above):**
  the planner aborts on a task force > 5 N or a vehicle tilt > 15 deg; the flight node's watchdog hands over to
  SAFETY at 15 deg of tilt.
- **Impedance, Proposed (the whole-body law's EE task):** M_d = diag(1, 1, 1, 0.05),
  D_d = diag(20.2, 20.2, 20.2, 0.2903), K_d = diag(120, 120, 120, 0.2484)
  on [x, y, z, heading] (kg, N.s/m, N/m; kg.m^2, N.m.s/rad, N.m/rad for the heading).
- **Admittance, Coordinated [Liu 2024] (eq 27, contact phase, claw frame):** M_d = diag(0.5, 0.5, 0.5),
  D_d = diag(17.3, 20, 20), K_d = diag(150, 20, 20) (kg, N.s/m, N/m).
- **Plant:** the free-flight mirror plant -- motor delay, model mismatch, battery sag, a standing wrench
  bias, imperfect joint actuation and emulated mocap feedback (the free-flight README explains each).
  Every `sim_*` key as flown:

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

- **The push-and-pull planner block as flown, turn form** (`configs/`):

```
push_pull_ee_offset: [0.1600, 0.0006, 0.1790]
push_pull_yaw_from_box_deg: 90.0
push_pull_turn_deg: 360.0
push_pull_push_distance: 0.50
push_pull_push_time_s: 36.0
push_pull_push_settle_s: 1.5
push_pull_exit_time_s: 3.0
push_pull_push_pose_deg: [0.0, 10.0, 20.0, 0.0]
push_pull_approach_dz: 0.10
push_pull_approach_back: 0.12
push_pull_exit_dz: 0.20
push_pull_contact: true
push_pull_contact_at_ready: false
push_pull_contact_off_at_push_end: true
push_pull_unload_time_s: 0.0
push_pull_world_anchor_push: true
push_pull_world_anchor_ready: false
push_pull_world_anchor_on_handle: true
push_pull_descent_trim: false
push_pull_land: [-0.5, 0.0, 0.8, -360.0]
push_pull_start_mark: [0.0, 0.0, 1.0, 0.0]
push_pull_table: [1.125, 1.275, -0.5, 0.5, 0.80]
push_pull_obstacle: [1.000, 1.400, -0.200, 0.200, 0.8540]
push_pull_table_clearance: 0.05
push_pull_gear_depth: 0.2425
push_pull_fence_max_z: 2.2
push_pull_guard_tilt_deg: 15.0
push_pull_guard_force_n: 5.0
```

## The struct `run`

| field | content |
|---|---|
| `meta` | method, run index, `scene`, `task`, `task_values` (turn angle / time / settle, the fin's radius and grasp height, the axis, the rotor's mass, the bearing's friction and damping, the contact-phase flag, the turn pose, the table, Land Start, Land, the controller overrides), plant, `phases`, `timetable_slots_s`, `late_steps` (empty), `window`, `impedance_windows`, `impedance_gains` (M_d, D_d, K_d), `contact_sign` (the PhysX report's sign and the rule that set it), `L_arm_m`, `base_com_model_m`, `t0_driver_s` |
| `metrics` | `window` = this run's table numbers over the window; `per_phase.<key>` = the same per phase; `whole_recording` = over every sample; `impedance.<window>` = the impedance residual per contact window; `valve` = the beam's rotation in the turn step, its follow-through after the release, its error against the plan, the claw's radius about the axis, the arm's margins to its stops |
| `tracking` | 100 Hz, `t` in s since the start of the mission; `in_window`, `phase_id`; measured, reference (`_ref_`) and error (`_err_`) for the platform position, `eps_uav_m`, `rho_uav`, the platform attitude, the system CoM, the end-effector position and heading, the joints, and the BEAM angle against its plan (`beam_angle_deg`, `beam_angle_plan_deg`, `beam_minus_plan_deg`) |
| `impedance` | the residual's time series over the CONTACT window (contact-report rate, ~62.5 Hz): `t`, `e_imp`, `F_ext` (the true contact wrench on the claw), `rendered` (M_d edot_v + D_d e_v + K_d e_y), `e_y`, and `windows` (name, start_s, end_s) |
| `raw` | every recorded stream on the driver's clock (subtract `meta.t0_driver_s` to get `tracking.t`): `odom`, `joints`, `reference` (the planner's whole-body reference), `reference_derivatives`, `rotor` (ground truth = the mocap's obj_0: the fixed axis and the beam's angle), `claw_truth`, `base_truth`, `contact_truth` (the rotor's PhysX contact report), `marks` (every step's start/end), `events` |

Units: positions in m (world ENU, z up) unless the field name says mm; angles in deg unless it says rad;
forces in N, moments in N.m. Errors are measured − reference. `platform_att_err_deg` columns are
`[roll pitch heading]`.

## Paper notation

| paper symbol | meaning | summary field (`metrics_mean`) | run file |
|---|---|---|---|
| e<sub>imp</sub> | F<sub>ext</sub> − (M<sub>d</sub> ė<sub>v</sub> + D<sub>d</sub> e<sub>v</sub> + K<sub>d</sub> e<sub>y</sub>), the impedance residual on the claw [N; heading N·m], rms over the turn / the whole contact | `e_imp_slide_rms_n`, `e_imp_slide_max_n`, `e_imp_slide_{x,y,z}_rms_n`, `e_imp_slide_psi_rms_nm`, `e_imp_contact_rms_n` | `impedance.e_imp` [N N N N·m], `metrics.impedance.<window>` |
| F<sub>ext</sub> | the true contact wrench on the claw (ground truth) | `F_ext_slide_rms_n` | `impedance.F_ext` |
| ‖ε<sub>UAV</sub>‖ | ‖r<sub>UAV</sub><sup>ref</sup> − r<sub>UAV</sub>‖, the platform's deviation from its reference position [mm] | `eps_uav_max_mm`, `eps_uav_rms_mm` | `tracking.eps_uav_m` [m] |
| ρ<sub>UAV</sub> | ‖ε<sub>UAV</sub>‖ / L, L = 0.370 m (the arm's reach) | `rho_uav_max`, `rho_uav_rms` | `tracking.rho_uav` |
| r<sub>0,x</sub>, r<sub>0,y</sub>, r<sub>0,z</sub> | platform position [mm] | `platform_x_mm`, `platform_y_mm`, `platform_z_mm` | `tracking.platform_pos_err_m(:,1:3)` [m] |
| ψ<sub>0</sub>, φ<sub>0</sub>, θ<sub>0</sub> | platform heading, roll, pitch [deg] | `platform_heading_deg`, `platform_roll_deg`, `platform_pitch_deg` | `tracking.platform_att_err_deg(:,[3 1 2])` |
| r<sub>e,x</sub>, r<sub>e,y</sub>, r<sub>e,z</sub> | end-effector position [mm] | `ee_x_mm`, `ee_y_mm`, `ee_z_mm` | `tracking.ee_pos_err_m(:,1:3)` [m] |
| ψ<sub>e</sub> | end-effector heading [deg] | `ee_heading_deg` | `tracking.ee_heading_err_deg` |
| q<sub>1</sub> … q<sub>4</sub> | joint angles [deg] | `q1_deg` … `q4_deg` | `tracking.q_err_deg(:,1:4)` |
| θ<sub>v</sub> | the beam's rotation during the turn step [deg] (360 commanded) | `beam_turned_deg`, `beam_plan_err_rms_deg` | `metrics.valve.turned_deg`, `tracking.beam_angle_deg` |

- **Reference:** the planner's dynamically compatible whole-body reference. The platform reference
  x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>) is r<sub>UAV</sub><sup>ref</sup>.
  During the turn the EE reference r<sub>e,d</sub> is the claw's planned circle about the axis (world-held).
- **max / rms:** over the window; the table's other columns are RMS errors.
- **e<sub>imp</sub>:** every term through one zero-phase 5 Hz low-pass; e<sub>y</sub> from the TRUE claw point
  (ground truth) minus the streamed reference; e<sub>v</sub>, ė<sub>v</sub> its derivatives against the streamed
  reference derivatives; F<sub>ext</sub> from the rotor's PhysX contact report, its sign the one that makes the
  vehicle's mean power on the rotor positive over the turn (`meta.contact_sign`). A residual of 0 means the claw
  moved exactly as the designed impedance under the measured force.
