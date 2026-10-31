# Experiment results, MATLAB data (payload pick-and-drop)

The hardware pick-and-drop flight, converted from its ros2 bag. This folder is self-contained:
these files plus MATLAB are all you need, with no ROS or custom message definitions. The `run` struct
has the layout of the simulation's `matlab_simulation_data/pick_and_drop/<method>_pnd_run<k>.mat`, so
a script written for those files runs on this one.

## Files

```
whole_body_l1_pnd_M5.mat                one run, struct `run`
pick_and_drop_metrics.mat                metrics_runs / metrics_mean = the run's numbers (one row),
                                         metrics_phase = per phase, phases = the phase table,
                                         window_s = the evaluation window, L_arm_m = L
configs/whole_body_l1_4d_hw_pick_and_drop.yaml               the controller + planner, as flown
configs/arm_external_torque_controller_hardware_aerial_pwm.yaml   the arm controller, as flown
```

- **Name:** `<method>_pnd_<run>`, the simulation's naming with the flight tag as the run. The tag is the
  Drop Margin: `M5` = 5 cm.
- **Method:** `whole_body_l1` = Proposed. Only the proposed controller flew this task on hardware.
- **Configs:** taken from git at the commits the vehicle ran (first line of each file). The Drop Margin is
  NOT the yaml's `pick_place_drop_height` (0.10 m): it was typed on the arm ground station, 0.05 m.

```matlab
r  = load('whole_body_l1_pnd_M5.mat').run;
P  = r.meta.phases;                              % the six phases of this flight
tr = r.tracking;
figure; hold on
for i = 1:numel(P.start_s)                       % phase bands, alternating shades
    c = 0.90 + 0.07*mod(i+1, 2);
    patch([P.start_s(i) P.end_s(i) P.end_s(i) P.start_s(i)], [0 0 400 400], c*[1 1 1], 'EdgeColor', 'none');
    text(mean([P.start_s(i) P.end_s(i)]), 390, P.label{i}, 'HorizontalAlignment', 'center');
end
plot(tr.t, 1e3 * tr.eps_uav_m)                   % ||eps_UAV|| [mm]
xlim([0 tr.t(end)]); xlabel('Time (s)'); ylabel('||\epsilon_{UAV}|| (mm)')
yyaxis right; ylim([0 400] / (1e3 * r.meta.L_arm_m)); ylabel('\rho_{UAV}')

% 3-D: references (dashed), measured platform and end effector, the basket (OptiTrack)
figure; hold on; grid on; axis equal; view(-50, 28)
plot3(tr.platform_pos_ref_m(:,1), tr.platform_pos_ref_m(:,2), tr.platform_pos_ref_m(:,3), 'k--')
plot3(tr.ee_pos_ref_m(:,1), tr.ee_pos_ref_m(:,2), tr.ee_pos_ref_m(:,3), '--', 'Color', [0.45 0.45 0.45])
plot3(tr.platform_pos_m(:,1), tr.platform_pos_m(:,2), tr.platform_pos_m(:,3))
plot3(tr.ee_pos_m(:,1), tr.ee_pos_m(:,2), tr.ee_pos_m(:,3), ':')
plot3(tr.payload_pos_m(:,1), tr.payload_pos_m(:,2), tr.payload_pos_m(:,3))   % or r.raw.payload (raw clock)

% the error grid: platform / end effector xyz in mm, headings and joints in deg
E = {1e3*tr.platform_pos_err_m, tr.platform_att_err_deg(:,3), 1e3*tr.ee_pos_err_m, tr.ee_heading_err_deg, tr.q_err_deg};
```

Raw streams are on the bag's clock: subtract `r.meta.t0_s` to put them on `tracking.t`
(`r.meta.t0_driver_s` is the same number, so the simulation's snippet works unchanged).

## The session: four flights, the Drop Margin lowered each time

Same mission, controller and payload; only the Drop Margin (how far above the Drop point the gripper opens)
changed. All four completed the six legs; in each the pilot took over in the land leg. Released ... up = the
basket centre at the gripper's OPEN command above its flat rest on the tray floor (z = 0.931 m); hit = the
peak fall speed. Tracking over the whole mission.

| run | flown | original ros2 bag | Drop Margin | the drop | window [s] | ‖ε<sub>UAV</sub>‖ max / RMS [mm] | EE RMSE [mm] | in this folder |
|---|---|---|---|---|---|---|---|---|
| M12 | 2026-10-28 16:16 | `flight_wb_l1_4d_pick_and_drop_12offset_20261028_161640` | 12 cm | released 138 mm up, hit at 1.05 m/s, bounced off the tray and fell to the floor | 83.5 | 209.9 / 58.1 | 59.7 | not exported |
| M8 | 2026-10-28 16:27 | `flight_wb_l1_4d_pick_and_drop_8offset_20261028_162711` | 8 cm (*) | released 120 mm up, hit at 1.03 m/s; in the tray, 71 mm from the drop point, tilted 13 deg | 84.9 | 168.8 / 53.7 | 56.1 | not exported |
| M6 | 2026-10-28 16:48 | `flight_wb_l1_4d_pick_and_drop_6offset_20261028_164816` | 6 cm | released 81 mm up, hit at 0.81 m/s, slid 66 mm; in the tray, 129 mm from the drop point, tilted 5 deg | 85.4 | 165.4 / 50.5 | 53.4 | not exported |
| M5 | 2026-10-28 16:56 | `flight_wb_l1_4d_pick_and_drop_5offset_20261028_165603` | 5 cm | released 75 mm up, hit at 0.73 m/s, no bounce, slid 16 mm; in the tray, 58 mm from the drop point, tilted 3 deg | 79.0 | 158.9 / 47.2 | 50.2 | **exported** |

(*) The bag records the drop leg's claw target, not the margin: 1.241 / 1.219 / 1.178 / 1.168 m. The steps
(22 / 41 / 10 mm) match the names except M8, whose claw sat about 2 cm higher than an 8 cm margin gives.
The heights in the table are measured on the basket, so they do not depend on the names.

**M5 is exported: the lowest release gave the best drop on every measure** (lowest release, slowest impact,
no bounce, least slide, flattest rest, closest to the drop point), and the best tracking of the four.
The comparison: `docs/docs_aerial_manipulator/archive/wb_pick_drop_flight_20261028/` (`tools/compare_margins.py`).

The bag of the exported run is kept unchanged in `results/experiment_results/flight_test_data/pick_and_drop/<name>/`
on the lab desktop. The other bags are in `docs/experimental_data_ros2_bag/1028 - T650-AM whole-body Tracking, Pick&Drop, Valve-Turning-*/`. The earlier pick-and-drop flight D-2
(2026-10-16, Drop Margin 0.20 m) is exported in `../pick_and_place/`.

## How this differs from the simulation's pick-and-drop

- **The operator paced the mission.** Each step was pressed on the arm ground station, so the phases come
  from the planner's status in the bag. The simulation used a fixed timetable.
- **The drop was pressed by hand,** 0.51 s after Ready To Drop ended (the arm GS's Drop button), not by the
  planner's automatic drop, and Ready To Drop was pressed 0.6 s after arriving at Drop Start, so the planner
  did not aim the drop at the measured basket (it needs a 4 s hover there). The simulation's drop margin is
  0.20 m, flown on the same timetable for all three controllers.
- **One controller.** No Geo-L1 run exists for this task on hardware.
- **The basket is measured by OptiTrack** (rigid body `obj_0`), not simulator ground truth.
  Mis-solved samples are removed: 906 flips (tilt > 60 deg) and 48 jumps
  (more than 15 mm or 30 deg of yaw off the rolling median of 15 samples), 8.5 % of the stream.
- **The window ends with DIRECT.** The land leg was cut short 5.15 s in: PX4 left OFFBOARD for nav_state 15 (STAB, the pilot's mode switch), with the platform at 0.36 m (land target 0.35 m).
- **Joint torque** `raw.joints.effort` [N.m, model convention] is the servos' Present Current
  (`raw.joints.current_counts`) through the calibrated torque constants: the electromagnetic torque the
  motors applied, before the gearbox's friction (the simulation's `effort` is the applied torque too).

## Phases (s since the start of go to start)

| # | phase | start [s] | end [s] | duration [s] | what happens |
|---|---|---|---|---|---|
| 1 | Go to start | 0.00 | 3.46 | 3.46 | fly from the hover to the start waypoint; hover |
| 2 | Pick | 3.46 | 21.10 | 17.64 | fly to a point 10 cm behind the fin, slide in level (the planner's 'descent' step), close the gripper on the fin, lift 20 cm (exit) |
| 3 | To drop start | 21.10 | 37.78 | 16.68 | carry the basket to the drop-start waypoint (180 deg turn); hover |
| 4 | Drop | 37.78 | 57.37 | 19.59 | fly over the tray, climbing to the release height; open the gripper (the basket falls into the tray); back out 20 cm and climb 20 cm (exit); hover |
| 5 | To land start | 57.37 | 73.80 | 16.43 | fly to the land-start waypoint (180 deg turn), arm folding home; hover |
| 6 | Land | 73.80 | 78.95 | 5.15 | descend toward the landing point (z 0.35 m) |

- **Window:** the whole mission, 0 to 78.95 s. `tracking.phase_id` (1..6) says which phase
  each sample is in; `tracking.in_window` is 1 everywhere (kept for the simulation scripts).
- **Phase boundaries:** each phase runs from its leg's `FLYING` status to the next leg's, so the hovers
  and the operator's waits between steps belong to the phase they end.

## Results

| phase | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | EE heading RMSE [deg] | tilt max [deg] |
|---|---|---|---|---|---|---|---|
| Go to start | 46.3 | 0.125 | 34.8 | 0.094 | 36.3 | 0.72 | 2.5 |
| Pick | 158.9 | 0.429 | 43.5 | 0.118 | 46.2 | 0.32 | 5.8 |
| To drop start | 77.2 | 0.209 | 50.1 | 0.136 | 53.8 | 0.61 | 4.2 |
| Drop | 155.5 | 0.420 | 55.3 | 0.150 | 59.8 | 0.70 | 4.9 |
| To land start | 96.9 | 0.262 | 45.6 | 0.123 | 46.6 | 0.68 | 3.3 |
| Land | 36.6 | 0.099 | 22.9 | 0.062 | 23.6 | 0.24 | 3.2 |
| **whole mission** | 158.9 | 0.429 | 47.2 | 0.128 | 50.2 | 0.59 | 5.8 |

Drop (`run.metrics.outcome`):

- **Drop point** (the Get-captured basket on the tray cover): (-0.810, -1.030, 0.958) m; the claw released 0.05 m above it. Reconstructed from the drop leg's claw target (the bag does not hold
  the Drop point): its xy is exact, its z assumes the 5 cm margin.
- **Claw at the OPEN command,** relative to its target: (-55, 10, 3) mm (x, y, z).
- **Basket at the OPEN command:** 80 mm from the drop point in xy; its centre was 75 mm above where it came to rest.
- **Basket at rest,** in the tray: 58 mm from the drop point in xy (-55, 20), tilted 2.9 deg.
- **Gripper:** CLOSE at 16.57 s, grasp reported at 17.20 s, OPEN at 49.81 s.
- The claw error here is the model FK of the measured pose and joints on the 100 Hz grid; `compare_margins.py`
  reads the planner's own `current_ee` instead, a few mm different ((-53, 6, 3) mm here).

## Conditions

- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm in torque mode, 3.746 kg in the controller model.
- **Feedback:** OptiTrack fused in PX4 EKF2 (~100 Hz); joint encoders.
- **Controller:** whole-body L1, 4-D attribution, `AM_HW_PROFILE=pick_and_drop`
  (`configs/whole_body_l1_4d_hw_pick_and_drop.yaml`: k_R / k_w 2.134 / 1.567, K_y / D_y 211.9 / 26.82, K_psi / D_psi 2.5 / 0.93, the wrist-roll stuck-joint hold on).
- **Arm poses:** pick = carry = drop [0, 32, 38, 0] deg, home [0, 40, 40, 0] deg.
- **EE offset** (the claw point in the basket's frame): (-0.02, 0.00, 0.16) m. Side Margin 0.10 m, Vertical Margin 0.20 m, Back Margin 0.20 m,
  Drop Margin 0.05 m, Drop Start z 1.21 m.
- **Leg goals** (`run.meta.leg_goals`, world ENU, planned from the Adjust-shifted waypoints):

| leg | base [m] | yaw [deg] | claw [m] |
|---|---|---|---|
| go_to_start | (0.042, -0.060, 1.000) | 0.0 | (0.295, -0.061, 0.939) |
| execute_pick | (0.834, 0.895, 1.101) | 1.3 | (1.090, 0.900, 0.999) |
| go_to_place_start | (0.842, -1.060, 1.210) | -180.0 | (0.586, -1.058, 1.108) |
| execute_place | (-0.534, -1.032, 1.269) | -180.0 | (-0.790, -1.030, 1.168) |
| go_to_land_start | (-0.458, 0.740, 1.000) | -360.0 | (-0.205, 0.739, 0.939) |
| execute_land | (-0.458, 0.740, 0.350) | -360.0 | (-0.205, 0.739, 0.289) |

## The struct `run`

| field | content |
|---|---|
| `meta` | name, run tag, method, flight date, original bag, phase table, window, clock, DIRECT entry/exit, largest gap per stream, flips removed, leg goals, configs |
| `metrics` | `window` (whole mission), `per_phase.<phase>`, `outcome` (the drop) |
| `tracking` | 100 Hz over the mission, `t` in s since the start of go to start. Measured, reference (`_ref_`) and error (`_err_`) for the platform position and attitude, `eps_uav_m` / `rho_uav`, the system CoM, the end-effector position and heading, the joints (`q_deg`), and `payload_pos_m` (the basket) |
| `raw` | the bag's streams, time in s since the bag's first message: `odom`, `joints` (model convention), `reference` (the planner's whole-body reference), `payload` (obj_0), `gripper` (finger positions), `controller_debug` (`wb_control_debug`), `motors` (normalised), `battery`, `mocap_vrpn` (vehicle), `events.mode` / `events.planner_status` / `events.pick_place_status` (transitions), `events.log` (every node's /rosout line: the operator's steps and the planner's replies) |

Units: positions in m (world ENU, z up) unless the name says mm; angles in deg unless it says rad. Errors are
measured − reference. `platform_att_err_deg` columns are `[roll pitch heading]`.

**Definitions** (the simulation's, `results/utils/build_pick_and_place.py`):

- **Reference:** the planner's dynamically compatible whole-body reference. Platform
  x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>); R<sub>0,d</sub> from the thrust
  direction (CoM reference acceleration + g e<sub>3</sub>) and the reference heading; EE r<sub>e,d</sub>, heading
  b<sub>1,e,d</sub>; joints q<sub>d</sub>.
- **Measured EE:** the planner model's forward kinematics of the measured platform pose and joint angles.
- **ε<sub>UAV</sub>** = ‖x<sub>b</sub> − r<sub>UAV</sub>‖; ρ<sub>UAV</sub> = ε<sub>UAV</sub> / L, L = 0.370 m.

Regenerate (from the bag in `results/experiment_results/flight_test_data/pick_and_drop/`):

```bash
/usr/bin/python3 results/utils/experiment_pick_and_drop.py
```
