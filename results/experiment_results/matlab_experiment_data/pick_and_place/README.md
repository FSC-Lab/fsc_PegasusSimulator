# Experiment results, MATLAB data (payload pick-and-place)

The hardware pick-and-place flight, converted from its ros2 bag. This folder is self-contained:
these files plus MATLAB are all you need, with no ROS or custom message definitions. The `run` struct
has the layout of the simulation's `matlab_simulation_data/pick_and_place/<method>_pnp_run<k>.mat`, so
a script written for those files runs on this one.

## Files

```
whole_body_l1_pnp_D-2.mat               one run, struct `run`
pick_and_place_metrics.mat               metrics_runs / metrics_mean = the run's numbers (one row),
                                         metrics_phase = per phase, phases = the phase table,
                                         window_s = the evaluation window, L_arm_m = L
configs/whole_body_l1_4d_hw_pick_and_drop.yaml               the controller + planner, as flown
configs/arm_external_torque_controller_hardware_aerial_pwm.yaml   the arm controller, as flown
```

- **Name:** `<method>_pnp_<run>`, the simulation's naming with the flight tag as the run.
- **Method:** `whole_body_l1` = Proposed. Only the proposed controller flew this task on hardware.
- **Configs:** taken from git at the commits the vehicle ran (first line of each file). Both were pushed
  76 min before the flight, and the bag shows their features: carry pose = pick pose, the automatic drop's
  2.0 s wait and the basket-aim check.

```matlab
r  = load('whole_body_l1_pnp_D-2.mat').run;
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

## The flight

| run | flown | original ros2 bag | what happened | in this folder |
|---|---|---|---|---|
| E-1 | 2026-10-16 13:44 | `flight_wb_l1_4d_pick_and_drop_empty_20261016_134438` | full mission without the basket (control flight) | not exported |
| D-1 | 2026-10-16 13:51 | `flight_wb_l1_4d_pick_and_drop_20261016_135100` | full mission; the gripper closed wrongly on the fin (stalled at -0.0010 m); the basket came to rest 112 mm from the drop point, tilted 25 deg | not exported |
| D-2 | 2026-10-16 14:00 | `flight_wb_l1_4d_pick_and_drop_20261016_140020` | full mission; the basket came to rest 30 mm from the drop point, tilted 5 deg; pilot took over at 0.37 m on the final descent | **exported** |

The bag of the exported run is kept unchanged in `results/experiment_results/pick_and_place/<name>/` on the
lab desktop. The other two bags are in `docs/experimental_data_ros2_bag/1016 - T650-AM whole-body Pick&Drop-*/`.

## How this differs from the simulation's pick-and-place

- **The task is a pick-and-DROP.** The pick is the same hook grasp. At the place the basket is released
  from a hover into a tray (`pick_place_place_drop`), where the simulation's paper task sets it down on a
  hat. The six phases are the same legs in the same order; phase 4 is the drop.
- **The operator paced the mission.** Each step was pressed on the arm ground station, so the phases come
  from the planner's status in the bag. The simulation used a fixed timetable.
- **One controller.** No Geo-L1 run exists for this task on hardware.
- **The basket is measured by OptiTrack** (rigid body `obj_0`), not simulator ground truth.
  Mis-solved samples are removed: 242 flips (tilt > 60 deg) and 18 jumps
  (more than 15 mm or 30 deg of yaw off the rolling median of 15 samples), 2.0 % of the stream.
  They happen when the claw is near the basket and, at rest, in the tray.
- **The window ends with DIRECT.** The land leg was cut short 4.94 s in: PX4 left OFFBOARD for nav_state 15 (STAB, the pilot's mode switch), with the platform at 0.37 m (land target 0.35 m).
- **Joint torque** `raw.joints.effort` [N.m, model convention] is the servos' Present Current
  (`raw.joints.current_counts`) through the calibrated torque constants: the electromagnetic torque the
  motors applied, before the gearbox's friction (the simulation's `effort` is the applied torque too).

## Phases (s since the start of go to start)

| # | phase | start [s] | end [s] | duration [s] | what happens |
|---|---|---|---|---|---|
| 1 | Go to start | 0.00 | 4.36 | 4.36 | fly from the hover to the start waypoint; hover |
| 2 | Pick | 4.36 | 21.84 | 17.48 | fly to a point 10 cm behind the fin, slide in level (the planner's 'descent' step), close the gripper on the fin, lift 20 cm (exit) |
| 3 | To drop start | 21.84 | 38.73 | 16.89 | carry the basket to the drop-start waypoint (180 deg turn); hover |
| 4 | Drop | 38.73 | 63.04 | 24.31 | fly over the tray, climbing to the release height; open the gripper (the basket falls into the tray); back out 20 cm and climb 20 cm (exit); hover |
| 5 | To land start | 63.04 | 79.48 | 16.44 | fly to the land-start waypoint (180 deg turn), arm folding home; hover |
| 6 | Land | 79.48 | 84.41 | 4.93 | descend toward the landing point (z 0.35 m) |

- **Window:** the whole mission, 0 to 84.41 s. `tracking.phase_id` (1..6) says which phase
  each sample is in; `tracking.in_window` is 1 everywhere (kept for the simulation scripts).
- **Phase boundaries:** each phase runs from its leg's `FLYING` status to the next leg's, so the hovers
  and the operator's waits between steps belong to the phase they end.

## Results

| phase | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | EE heading RMSE [deg] | tilt max [deg] |
|---|---|---|---|---|---|---|---|
| Go to start | 34.6 | 0.094 | 19.6 | 0.053 | 21.3 | 0.28 | 3.1 |
| Pick | 156.6 | 0.423 | 47.7 | 0.129 | 51.1 | 1.07 | 5.0 |
| To drop start | 88.6 | 0.239 | 47.6 | 0.129 | 52.3 | 1.74 | 3.6 |
| Drop | 175.8 | 0.475 | 67.4 | 0.182 | 72.3 | 1.77 | 6.0 |
| To land start | 67.2 | 0.182 | 32.9 | 0.089 | 33.2 | 1.52 | 3.3 |
| Land | 43.1 | 0.116 | 19.0 | 0.051 | 20.2 | 0.97 | 4.3 |
| **whole mission** | 175.8 | 0.475 | 49.8 | 0.135 | 53.4 | 1.50 | 6.0 |

Drop (`run.metrics.outcome`):

- **Drop point** (the Get-captured basket on the tray cover): (-1.220, -1.177, 0.926) m; the claw released 0.20 m above it.
- **Claw at the OPEN command,** relative to its target: (-59, -27, 6) mm (x, y, z).
- **Basket at the OPEN command:** 52 mm from the drop point in xy; it fell 0.224 m.
- **Basket at rest:** 30 mm from the drop point in xy (-30, -1), tilted 5.1 deg.
- **Gripper:** CLOSE at 17.17 s, grasp reported at 17.79 s, OPEN at 53.67 s.

## Conditions

- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm in torque mode, 3.746 kg in the controller model.
- **Feedback:** OptiTrack fused in PX4 EKF2 (~100 Hz); joint encoders.
- **Controller:** whole-body L1, 4-D attribution, `AM_HW_PROFILE=pick_and_drop`
  (`configs/whole_body_l1_4d_hw_pick_and_drop.yaml`: k_R / k_w 2.134 / 1.567, K_y / D_y 211.9 / 26.82, K_psi / D_psi 0.2484 / 0.2903).
- **Arm poses:** pick = carry = drop [0, 32, 38, 0] deg, home [0, 40, 40, 0] deg.
- **EE offset** (the claw point in the basket's frame): (-0.02, 0.00, 0.16) m.
- **Leg goals** (`run.meta.leg_goals`, world ENU, planned from the Adjust-shifted waypoints):

| leg | base [m] | yaw [deg] | claw [m] |
|---|---|---|---|
| go_to_start | (-0.103, -0.203, 1.000) | 0.0 | (0.150, -0.205, 0.939) |
| execute_pick | (0.624, 0.779, 1.149) | 3.7 | (0.880, 0.794, 1.047) |
| go_to_place_start | (0.697, -1.203, 1.000) | -180.0 | (0.441, -1.202, 0.898) |
| execute_place | (-0.944, -1.179, 1.387) | -180.0 | (-1.200, -1.177, 1.286) |
| go_to_land_start | (-0.603, 0.597, 1.000) | -360.0 | (-0.350, 0.595, 0.939) |
| execute_land | (-0.603, 0.597, 0.350) | -360.0 | (-0.350, 0.595, 0.289) |

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

Regenerate (from the bag in `results/experiment_results/pick_and_place/`):

```bash
/usr/bin/python3 results/utils/experiment_pick_and_place.py
```
