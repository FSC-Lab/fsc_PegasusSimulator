# Experiment results, MATLAB data (mail delivery)

The hardware mail-delivery flights of 2026-11-05, converted from their ros2 bags. The folder is
self-contained: these files plus MATLAB are all you need, with no ROS or custom message definitions. The
`run` struct has the layout of `../pick_and_drop/` and `../pick_and_place/` (the same six legs, the same
metric definitions), so a script written for those files runs on these.

## Files

```
whole_body_l1_mail_run<N>.mat     one flight, struct `run`; N = the bag's success_N
mail_box_metrics.mat              metrics_runs (one row per flight), metrics_mean, metrics_phase, bags, L_arm_m
configs/                          the controller + planner and the arm controller, as flown (from git)
```

```matlab
r  = load('whole_body_l1_mail_run1.mat').run;
P  = r.meta.phases;  tr = r.tracking;  oc = r.metrics.outcome;
plot(tr.t, 1e3 * tr.eps_uav_m); xline(oc.gripper_open_s, '--', 'release')     % ||eps_UAV|| [mm]
plot3(tr.ee_pos_m(:,1), tr.ee_pos_m(:,2), tr.ee_pos_m(:,3))                    % the claw
S = load('mail_box_metrics.mat');  T = struct2table(S.metrics_runs);
```

Raw streams are on the bag's clock: subtract `r.meta.t0_s` to put them on `tracking.t`.

## The flights

| run | flown | original ros2 bag | file | window [s] | ‖ε<sub>UAV</sub>‖ max / RMS [mm] | EE RMSE [mm] | release | claw at release: along / off the line [mm] | release after the slide [s] |
|---|---|---|---|---|---|---|---|---|---|
| S-1 | 2026-11-05 16:03 | `flight_wb_l1_4d_mail_box_20261105_160336_success_1` | `whole_body_l1_mail_run1.mat` | 87.0 | 90.4 / 30.9 | 31.4 | automatic | +13 / 7 | 2.38 |
| S-2 | 2026-11-05 16:13 | `flight_wb_l1_4d_mail_box_20261105_161320_success_2` | `whole_body_l1_mail_run2.mat` | 82.6 | 81.0 / 26.0 | 27.0 | by hand | -22 / 14 | 2.24 |
| S-3 | 2026-11-05 16:27 | `flight_wb_l1_4d_mail_box_20261105_162728_success_3` | `whole_body_l1_mail_run3.mat` | 82.3 | 94.5 / 35.3 | 35.5 | by hand | +48 / 33 | 1.95 |
| S-4 | 2026-11-05 16:33 | `flight_wb_l1_4d_mail_box_20261105_163355_success_4` | `whole_body_l1_mail_run4.mat` | 84.0 | 81.5 / 31.9 | 32.4 | by hand | +48 / 19 | 1.51 |
| S-5 | 2026-11-05 16:49 | `flight_wb_l1_4d_mail_box_20261105_164904_success_5` | `whole_body_l1_mail_run5.mat` | 82.2 | 100.2 / 32.2 | 32.8 | by hand | +24 / 30 | 1.68 |

- All five were delivered (the operator's verdict: the bag folders are named `success_N`). The envelope has
  no mocap markers, so the outcome recorded here is the CLAW at the release against the planner's release
  point: along the insertion line (> 0 = deeper) and off it. The planner's own automatic release fires inside
  −15 / +20 mm along and 30 mm off, with the claw slower than 0.05 m/s for 1 s.
- **release:** automatic = the planner's release gates opened the gripper (`pick_place/drop_now`); by hand =
  the operator opened it from the arm ground station before the gates passed.
- **End of the window** (`run.meta.direct_exit`): S-1: the land leg was cut short 4.90 s in: PX4 left OFFBOARD for nav_state 15 (STAB, the pilot's mode switch); S-2: the land leg was cut short 5.22 s in: PX4 left OFFBOARD for nav_state 15 (STAB, the pilot's mode switch); S-3: the land leg was cut short 5.69 s in: PX4 left OFFBOARD for nav_state 15 (STAB, the pilot's mode switch); S-4: the land leg was cut short 5.51 s in: PX4 left OFFBOARD for nav_state 15 (STAB, the pilot's mode switch); S-5: the land leg was cut short 5.03 s in: PX4 left OFFBOARD for nav_state 15 (STAB, the pilot's mode switch).

## Phases (mean over the flights)

| phase | what happens | duration [s] | eps max [mm] | eps rms [mm] | EE RMSE [mm] | tilt max [deg] |
|---|---|---|---|---|---|---|
| Go to start | fly to the start waypoint; hover | 3.4 | 37.2 | 23.3 | 25.2 | 3.0 |
| Pick | fly to a hold 10 cm behind the envelope's grasp point, slide in level (the planner's 'descent' step), close the gripper on the envelope, climb 20 cm (exit) | 26.5 | 72.2 | 24.8 | 25.7 | 4.5 |
| To mailbox start | carry the envelope to the mailbox-start waypoint; hover | 10.5 | 52.6 | 29.5 | 30.1 | 3.6 |
| Insert | Ready To Insert: the arm sweeps to the place pose and the claw holds 0.25 m back along the insertion line (pitched 25 deg down); Insert: slide 0.25 m in through the mouth; open the gripper (the release); back out 10 cm and climb 20 cm (exit); hover | 22.7 | 81.5 | 37.8 | 38.3 | 4.2 |
| To land start | fly to the land-start waypoint, arm folding home; hover | 15.1 | 58.8 | 30.0 | 30.2 | 3.3 |
| Land | descend toward the landing point (z 0.35 m) | 5.3 | 52.2 | 32.7 | 33.2 | 3.2 |

Each phase runs from its leg's `FLYING` status to the next leg's (the operator's waits belong to the
phase they end). The window is the whole mission, from the start of go to start to the end of the land
leg or of DIRECT. Per-flight values: `metrics_phase`.

## Conditions

- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm in torque mode, 3.746 kg in the
  controller model.
- **Feedback:** OptiTrack fused in PX4 EKF2 (~100 Hz); joint encoders.
- **Controller:** whole-body L1, 4-D attribution, `AM_HW_PROFILE=mail_delivery`
  (`configs/whole_body_l1_4d_hw_mail_delivery.yaml`: k_R / k_w 2.134 / 1.567,
  K_y / D_y 211.9 / 26.82, K_psi / D_psi 2.5 / 0.93).
  The bags show the `dev_robotic_arm` build (vehicle `AM-T650-WB-L1-4D-HW-MAIL`); the yaml is that
  branch's file at d957c26, unchanged at the branch head flown.
- **Arm poses:** pick = carry [0, 36, 44, 90] deg (the jaws close vertically), place [0, 20, 45, 90] deg
  (the claw pitched 25 deg down), home [0, 40, 40, 0] deg.
- **EE offset** (the claw point in the envelope's frame): (-0.06, 0.00, 0.00) m.
- **Insert:** a 0.25 m slide along a line pitched 25.0 deg down, heading
  -0.3 deg; release point (0.390, 0.100, 1.080) m in the first flight
  (each flight's in `run.metrics.outcome`).
- **Configs:** taken from git at the commits the vehicle ran (first line of each file).

## The struct `run`

| field | content |
|---|---|
| `meta` | name, run tag (S-N), method, flight date, original bag, phase table, window, clock, DIRECT entry/exit, largest gap per stream, leg goals, configs |
| `metrics` | `window` (whole mission), `per_phase.<phase>`, `outcome` (the pick and the release, below) |
| `tracking` | 100 Hz over the mission, `t` in s since the start of go to start. Measured, reference (`_ref_`) and error (`_err_`) for the platform position and attitude, `eps_uav_m` / `rho_uav`, the system CoM, the end-effector position and heading, the joints (`q_deg`). No payload column: the envelope is not tracked |
| `raw` | the bag's streams, time in s since the bag's first message: `odom`, `joints` (model convention), `reference` (the planner's whole-body reference), `localization_frame` (obj_0; static, see above), `gripper` (finger positions), `planner_claw_error` (`pick_place/arrival_error`), `controller_debug` (`wb_control_debug`), `motors` (normalised), `battery`, `mocap_vrpn` (vehicle), `events.mode` / `events.planner_status` / `events.pick_place_status` (transitions), `events.log` (every node's /rosout line: the operator's steps and the planner's replies) |

`run.metrics.outcome`:

- `release_point_m`, `insert_line_unit`, `insert_slide_m`, `insert_pitch_deg` (yaml), `insert_pitch_measured_deg` (the reference slide), `insert_heading_deg`.
- `claw_err_at_open_mm` (xyz, measured − release point), `claw_along_at_open_mm`, `claw_off_axis_at_open_mm`, `planner_claw_err_at_open_mm` (the planner's own, target − measured),
  `q4_at_open_deg` / `q4_ref_at_open_deg`, `release_mode`, `planner_release_note` (the planner's text).
- Pick: `pick_claw_target_m`, `claw_err_at_close_mm`. Times [s since go to start]: `pick_slide_start_s`,
  `gripper_close_s`, `exit_pick_s`, `insert_slide_start_s`, `insert_slide_end_s`, `gripper_open_s`,
  `exit_place_s`, `release_after_slide_s`.

Units: positions in m (world ENU, z up) unless the name says mm; angles in deg unless it says rad. Errors
are measured − reference. Definitions: `../pick_and_drop/README.md` (the simulation's, `results/utils/build_pick_and_place.py`); ρ<sub>UAV</sub> = ε<sub>UAV</sub> / L, L = 0.370 m.

The bags are copied unchanged to `results/experiment_results/flight_test_data/mail_box/<name>/`; the
originals are in `docs/experimental_data_ros2_bag/1105 - T650-AM Mail-Box, Robust Tracking, Box-Pushing-*/Mail-Box [Success]/`.

Regenerate:

```bash
/usr/bin/python3 results/utils/experiment_mail_box.py
```
