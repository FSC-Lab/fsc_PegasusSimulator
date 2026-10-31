# Simulation results, MATLAB data (payload pick-and-drop)

The data behind the pick-and-drop comparison (`tab:sim_pick_drop`). Self-contained: these files plus MATLAB.
The same `run` struct layout and notation as `pick_and_place/` and `free_flight_tracking/` beside it,
plus the drop's own measures under `run.metrics.drop`.

## Files

```
<method>_pnd_run<k>.mat          one run, struct `run`
pick_and_drop_metrics.mat        metrics_mean / metrics_std = the table (mean / sample std over the kept runs),
                                 metrics_runs = every kept run, metrics_phase = every run x phase,
                                 phases = the common phase table, attempts = every attempt flown,
                                 window_s, L_arm_m, tray (centre, inner half-size, rim height)
configs/                         the three controllers' yamls as flown (the decoupled one flew the
                                 planner block of the whole-body file; the MAC file carries it verbatim)
reference_outputs/               the Python table + figures, to check MATLAB output against
```

```matlab
S = load('pick_and_drop_metrics.mat');  P = S.phases;
r = load('whole_body_l1_pnd_run1.mat').run;
w = r.tracking.in_window > 0;  plot(r.tracking.t(w), 1e3 * r.tracking.eps_uav_m(w))   % ||eps_UAV|| [mm]
T = struct2table(S.metrics_mean);   D = r.metrics.drop;                                % the table; this run's drop
```

## Results

Flown 2026-10-26 16:13 to 2026-10-26 16:58 (attempt end times). Mean ± sample std over the kept runs:

| method | success | flat | eps max [mm] | eps rms [mm] | EE RMSE [mm] | claw at release [mm] | rest off centre [mm] | rest tilt [deg] |
|---|---|---|---|---|---|---|---|---|
| Proposed | 3 of 4 | 3 | 132.8 ± 6.8 | 21.9 ± 1.4 | 24.8 ± 0.9 | 7.3 ± 0.5 | 15 ± 4 | 0.0 ± 0.0 |
| Geo-L1 | 3 of 4 | 2 | 331.2 ± 6.8 | 65.3 ± 1.3 | 68.1 ± 1.9 | 40.4 ± 15.4 | 31 ± 17 | 3.9 ± 6.7 |
| MAC | 3 of 3 | 1 | 139.1 ± 11.4 | 93.0 ± 7.1 | 119.1 ± 6.8 | 168.7 ± 9.6 | 58 ± 26 | 9.1 ± 9.2 |

Every kept run:

| method | file | eps max [mm] | eps rms [mm] | EE RMSE [mm] | claw at release [mm] | rest off centre [mm] | rest tilt [deg] | payload |
|---|---|---|---|---|---|---|---|---|
| Proposed | `whole_body_l1_pnd_run1.mat` | 138.8 | 22.8 | 25.8 | 7.5 | 14 | 0.0 | flat |
| Proposed | `whole_body_l1_pnd_run2.mat` | 125.4 | 20.3 | 23.9 | 6.7 | 20 | 0.0 | flat |
| Proposed | `whole_body_l1_pnd_run3.mat` | 134.1 | 22.5 | 24.7 | 7.7 | 13 | 0.0 | flat |
| Geo-L1 | `geometric_l1_pnd_run1.mat` | 338.1 | 63.8 | 66.3 | 40.9 | 27 | 0.0 | flat |
| Geo-L1 | `geometric_l1_pnd_run2.mat` | 331.1 | 66.3 | 70.1 | 55.5 | 50 | 11.6 | in, tilted |
| Geo-L1 | `geometric_l1_pnd_run3.mat` | 324.4 | 65.8 | 67.8 | 24.7 | 17 | 0.0 | flat |
| MAC | `modular_adaptive_pnd_run1.mat` | 151.6 | 101.2 | 127.0 | 178.9 | 86 | 18.4 | in, tilted |
| MAC | `modular_adaptive_pnd_run2.mat` | 136.4 | 89.0 | 115.1 | 167.7 | 51 | 0.0 | flat |
| MAC | `modular_adaptive_pnd_run3.mat` | 129.2 | 88.9 | 115.3 | 159.6 | 36 | 8.8 | in, tilted |

Attempts (a run is kept if it completed, the payload came to rest inside the tray, every step was on its
timetable slot, and the scene banner showed the 200 g payload and the arm-bridge drain):

| method | attempts | kept | why the others were not kept |
|---|---|---|---|
| Proposed | 4 | 3 | attempt 3.1: [  43.40s] ABORT: left DIRECT while execute_pick holding above |
| Geo-L1 | 4 | 3 | attempt 1.1: [  66.60s] TIMETABLE LATE: go_to_place_start accepted 0.60 s after its mark |
| MAC | 3 | 3 | -- |

## Timing

The driver flies the DROP timetable (`pnp_mission_v2.py --drop-place --timetable`): the clamped payload is
lifted as soon as the jaws close (a clamp on a payload still standing on its pillar is a closed chain that
drifts), so the end of the pick phase varies (24.6 to 30.4 s); Ready To Drop starts at a fixed mission time, so every later phase
starts at the same time on every run. The phase table is the mean over the runs:

| # | phase | start [s] | end [s] | duration [s] | what happens |
|---|---|---|---|---|---|
| 1 | Go to start | 0.00 | 5.31 | 5.31 | fly from the hover to the start waypoint; hover |
| 2 | Pick | 5.31 | 25.83 | 20.52 | approach above the fin, hover 2 s (descent trim), descend, close the gripper on the fin, lift at once (exit) |
| 3 | To drop start | 25.83 | 54.01 | 28.18 | carry the payload to the drop-start waypoint; hover there until the slot (>= 6 s: the planner measures the hanging payload and aims the drop at it) |
| 4 | Drop | 54.01 | 80.01 | 26.01 | Ready To Drop to the release point above the tray, hover 2 s, open (the payload falls), back out and climb |
| 5 | To land start | 80.01 | 98.32 | 18.30 | fly to the land-start waypoint, the gripper closing and the arm folding; hover |
| 6 | Land | 98.32 | 104.17 | 5.86 | the approach to the landing point |

- **Window:** every metric covers the whole mission, 0 to 104.17 s; `run.tracking.in_window` marks it,
  `run.tracking.phase_id` (1..6) the phase.
- **Release:** 2 s after the Ready To Drop leg ends, by the driver, on every run (`run.meta.t_release_s`).

## Scene and task

- **Field:** two 0.77 m pillars with the printed hat, PICK at (1, 1) m, PLACE at (-1, -1) m.
- **Payload:** the experiment's Box_Assembly (basket, holder fin, mocap markers), **200 g**, on the PICK hat.
- **Grasp:** a clamp on the holder's fin (the simulated jaws close on a 20 mm grip pad around it).
- **Drop:** Ready To Drop straight to the release point (the payload's rest pose on the tray cover + the drop
  margin), open, back out and climb. The tray: 371 x 290 mm, 35 mm walls, turned 90 deg, on the PLACE hat
  (rim 0.813 m). Flat = resting on the tray floor with tilt < 5 deg.
- **Arm pose:** pick = carry = drop [0, 32, 38, 0] deg.
- **Controllers:** Proposed = whole-body 4-D L1 (`whole_body_l1_4d_sim_pick_and_drop.yaml`); Geo-L1 = the
  decoupled geometric + L1 rig with a position-mode arm, its pick-and-place gains
  (`geometric_l1_sim_pick_and_place.yaml`); MAC = modular adaptive (`modular_adaptive_sim_pick_and_drop.yaml`).
- **Simulator:** headless Isaac at real time, raw mocap emulator, and the arm bridge DRAINED every physics
  step (`PEGASUS_ARM_SPIN_DRAIN=1`, 2026-10-26: before it the applied arm torque could lag the command
  ~40 ms, which made MAC unstable; `docs/docs_aerial_manipulator/archive/pick_place_mac_20261026`).
- **Plant:** every `sim_*` key as flown:

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
sim_feedback_mocap_rate_hz: 120.0
sim_feedback_pos_noise_m: 0.0005
sim_feedback_vel_noise_mps: 0.022
sim_feedback_noise_seed: 0
```

## The struct `run`

| field | content |
|---|---|
| `meta` | method, run index, scene, plant, `phases`, `timetable_slots_s`, `late_steps`, `window`, `L_arm_m`, `t0_driver_s`, `t_release_s`, `tray` |
| `metrics` | `window` = this run's table numbers; `per_phase.<key>`; `whole_recording`; `drop` = claw_err_at_release_mm (+ xyz, target − claw), release_drop_height_mm (payload bottom over the rim), release_off_centre_mm, rest_off_centre_mm (+ xy), rest_tilt_deg, in_tray, flat, lurch_after_release_mm, rest_pos_m |
| `tracking` | 100 Hz, `t` in s since the start of the mission: measured, reference (`_ref_`) and error (`_err_`) for the platform, `eps_uav_m`, `rho_uav`, attitude, CoM, end effector and joints |
| `raw` | every stream on the driver clock (subtract `meta.t0_driver_s`): `odom`, `joints`, `reference`, `payload` (ground truth), `claw_truth`, `marks`, `events` |

Notation and units as in `pick_and_place/README.md`: positions in m (world ENU) unless the name says mm,
angles in deg unless it says rad; errors are measured − reference.
