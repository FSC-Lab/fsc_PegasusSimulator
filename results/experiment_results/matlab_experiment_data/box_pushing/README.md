# Experiment results, MATLAB data (box pushing)

The two hardware box-pushing flights of 2026-10-27, converted from their ros2 bags. This folder is
self-contained: these files plus MATLAB are all you need, with no ROS or custom message definitions.
The `run` struct has the layout of the simulation's
`matlab_simulation_data/box_pushing/<method>_push_run<k>.mat`, so a script written for those files runs on
these. **These flights show feasibility; the accuracy comparison is the simulation's.**

## Files

```
whole_body_l1_push_BP-1.mat          one run, struct `run`
whole_body_l1_push_BP-2.mat          one run, struct `run`
box_pushing_metrics.mat                metrics_runs = every run, metrics_mean / metrics_std over the two,
                                       metrics_phase = every run x phase, metrics_impedance = every run x
                                       contact window, phases = BP-1's phase table, phases_runs = both,
                                       window_s = BP-1's window, L_arm_m = L
configs/whole_body_l1_4d_hw_box_pushing.yaml                  the controller + planner, as flown
configs/arm_external_torque_controller_hardware_aerial_pwm.yaml   the arm controller, as flown
```

- **Name:** `<method>_push_<run>`, the simulation's naming with the flight tag as the run.
- **Method:** `whole_body_l1` = Proposed. Only the proposed controller flew this task on hardware.
- **Configs:** taken from git at the commits the vehicle ran (first line of each file). The bag shows
  their features: the push pose [0, 28, 32, 0] deg in the planner's hold captures, the abort climbing
  straight up.

```matlab
S  = load('box_pushing_metrics.mat');
R  = struct2table(S.metrics_runs)                % the two runs
r  = load('whole_body_l1_push_BP-1.mat').run;
P  = r.meta.phases;  tr = r.tracking;           % this run's eight phases; 100 Hz, t = 0 at go to start
figure; hold on
for i = 1:numel(P.start_s)                       % phase bands, alternating shades
    c = 0.90 + 0.07*mod(i+1, 2);
    patch([P.start_s(i) P.end_s(i) P.end_s(i) P.start_s(i)], [0 0 200 200], c*[1 1 1], 'EdgeColor', 'none');
    text(mean([P.start_s(i) P.end_s(i)]), 195, P.label{i}, 'HorizontalAlignment', 'center');
end
plot(tr.t, 1e3 * tr.eps_uav_m)                   % ||eps_UAV|| [mm]
xlim([0 tr.t(end)]); xlabel('Time (s)'); ylabel('||\epsilon_{UAV}|| (mm)')

% the box along the push against the plan (the simulation's README plots the same from raw.box)
tv = r.meta.task_values;  u = [cosd(tv.table_yaw_deg) sind(tv.table_yaw_deg)];
w  = ~isnan(tr.box_pos_m(:,1));  s = tr.box_pos_m(:,1:2) * u';  s0 = s(find(w & tr.phase_id == 4, 1));
figure; plot(tr.t(w), 1e3*(s(w) - s0)); xlabel('Time (s)'); ylabel('box travel along the push (mm)')

% the force the controller estimated on the claw, and the box model's, along the push
im = r.impedance;  figure; plot(im.t, im.F_hat_along_push_n, im.t, im.F_box_model_along_push_n)
legend('F\_hat (controller estimate)', 'box model m a + \mu m g'); ylabel('N'); xlabel('Time (s)')
```

Raw streams are on the bag's clock: subtract `r.meta.t0_s` to put them on `tracking.t`
(`r.meta.t0_driver_s` is the same number, so the simulation's snippets work unchanged).

## The flights

| run | flown | original ros2 bag | what happened | file |
|---|---|---|---|---|
| BP-1 | 2026-10-27 17:11 | `flight_wb_l1_4d_box_pushing_300g_20261027_171114` | 0.40 m push COMPLETE (box 387 mm at the push end, 405 mm after the release); after the station released the claw and opened the jaws, the q3 rate guard aborted -- the abort's climb replaced the exit; box untouched, upright; normal way home and landing | `whole_body_l1_push_BP-1.mat` |
| BP-2 | 2026-10-27 17:22 | `flight_wb_l1_4d_box_pushing_300g_20261027_172227` | push stopped 1.5 s before its end (box 348 of 400 mm) by the q3 static guard; in the abort q3 went below 0 for 0.64 s (min -3.8 deg); box untouched, upright; normal way home and landing | `whole_body_l1_push_BP-2.mat` |

The bags are kept unchanged in `results/experiment_results/flight_test_data/box_pushing/<name>/` on the lab
desktop, and in `docs/experimental_data_ros2_bag/1027 - T650-AM whole-body Box-Pushing & Tracking [Success]-*/`.
The folder name says 300 g and the data file 500 g; the box weighs 280 g. Flight analysis:
`docs/docs_aerial_manipulator/archive/wb_box_pushing_flight_20261027/README.md`.

## How this differs from the simulation's box pushing

- **Both runs ended in a planner safety abort** (the q3 posture guard), which replaced the normal exit:
  `run.meta.aborted`, `abort_s`, `abort_reason`. The phases keep the simulation's eight names; in these
  runs `exit` is the abort's straight climb and arm fold, and `release` is the station's release (BP-1)
  or the abort's 1.5 s release hold (BP-2).
- **F_ext is an estimate.** The hardware has no force sensor, so the impedance residual uses the
  controller's own contact-force estimate F_hat (`run.meta.force_source`); the simulation uses the PhysX
  contact force. `impedance.F_box_model_along_push_n` is an independent physical check (box mass x its
  measured acceleration + mu m g, mu = 0.25). It reads ~0 while the box is stuck (static friction is
  not observable from the box's motion), so its mean is below mu m g when the box stick-slips.
- **Lighter box, shorter push, newer arm settings:** 280 g, 0.40 m in 19.0 s, push pose [0, 28, 32, 0] deg, push axis at natural inertia (M_d / D_d / K_d along y = 1.64 / 33.128 / 196.8). The simulation flew 500 g,
  0.50 m in 24 s, [0, 32, 28, 0] deg, M_d / D_d / K_d = 1 / 20.2 / 120 on every axis.
- **The operator's station paced the mission** (its automatic Ready / Push / Exit); the phases come from
  the planner's status and the station's log, so the two runs' phase times differ (`phases_runs`).
- **The box is measured by OptiTrack** (rigid body `obj_0`), not simulator ground truth; mis-solved
  samples are removed (tilt > 60 deg; jumps off the rolling median): BP-1 0 + 0, BP-2 0 + 0.
- **The measured claw** is the model's forward kinematics of the measured platform pose and joints (the
  simulation's `raw.claw_truth`, `raw.base_truth` and `raw.contact_truth` have no hardware counterpart).
- **Recording gaps:** the recorder ran on the ground-station laptop, so the vehicle computer's topics reach
  it over WiFi; BP-1: 61.63 s (0.60 s long); BP-2: none > 0.3 s (s since go to start). The vehicle kept flying in DIRECT; `tracking` interpolates across and marks
  the samples in `tracking.recording_gap`.
- **The window ends with DIRECT:** BP-1: the land leg was cut short 5.82 s in: PX4 left OFFBOARD for nav_state 15 (STAB, the pilot's mode switch); BP-2: the land leg was cut short 5.77 s in: PX4 left OFFBOARD for nav_state 15 (STAB, the pilot's mode switch).
- **Joint torque** `raw.joints.effort` [N.m, model convention] is the servos' Present Current through the
  calibrated torque constants (`raw.joints.current_counts`).

## Phases (s since the start of go to start)

| # | phase | BP-1 start - end [s] | BP-2 start - end [s] | what happens |
|---|---|---|---|---|
| 1 | Go to start | 0.00 - 27.33 | 0.00 - 26.77 | fly from the take-off hover to the point 0.12 m behind and 0.15 m above the fin, round the table, the arm unfolding to the push pose; hover |
| 2 | Ready | 27.33 - 30.93 | 26.77 - 30.43 | open the gripper, descend diagonally onto the fin; the claw held by the room once it has settled within 25 mm of the grasp point |
| 3 | Grasp | 30.93 - 31.58 | 30.43 - 31.07 | the station closes the gripper on the fin; the push starts on the grasp |
| 4 | Push | 31.58 - 55.45 | 31.07 - 50.08 | the law in CONTACT: 1.5 s settle, then the 0.40 m slide at 21 mm/s; then the hold on the handle until the release (BP-1: the station's 3 s calm wait) |
| 5 | Release | 55.45 - 55.89 | 50.08 - 51.58 | the claw's reference back to the vehicle and the jaws open (BP-1: the station's exit; BP-2: the abort's 1.5 s release hold) |
| 6 | Exit | 55.89 - 64.91 | 51.58 - 59.70 | the retreat: here the safety abort's straight climb of 0.30 m and the arm folding home (the normal exit climbs 0.25 m); hover |
| 7 | Go to land start | 64.91 - 94.66 | 59.70 - 89.41 | fly round the table's far end to the land-start hover |
| 8 | Land | 94.66 - 100.46 | 89.41 - 95.18 | descend toward the landing hover (z 0.30 m); the pilot takes over before the leg ends |

`tracking.phase_id` (1..8) says which phase each sample is in; `tracking.in_window` is 1 everywhere.

## Results

| run | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | EE heading RMSE [deg] | tilt max [deg] | box pushed [mm] | box tilt max [deg] |
|---|---|---|---|---|---|---|---|---|---|
| BP-1 | 83.9 | 0.227 | 34.6 | 0.094 | 33.0 | 2.77 | 4.9 | 390 | 1.54 |
| BP-2 | 129.2 | 0.349 | 34.4 | 0.093 | 34.3 | 2.07 | 3.9 | 352 | 3.06 |

Per phase (`metrics_phase`):

| run | phase | eps max [mm] | eps rms [mm] | EE RMSE [mm] | EE heading RMSE [deg] | tilt max [deg] |
|---|---|---|---|---|---|---|
| BP-1 | Go to start | 41.1 | 19.1 | 23.7 | 0.63 | 1.8 |
| BP-1 | Ready | 44.6 | 24.7 | 28.2 | 0.47 | 1.4 |
| BP-1 | Grasp | 18.4 | 14.5 | 19.9 | 0.79 | 1.3 |
| BP-1 | Push | 73.6 | 38.0 | 24.7 | 2.88 | 4.9 |
| BP-1 | Release | 49.3 | 26.4 | 13.6 | 2.30 | 4.6 |
| BP-1 | Exit | 58.0 | 37.3 | 39.7 | 1.98 | 4.1 |
| BP-1 | Go to land start | 83.9 | 42.7 | 42.8 | 3.59 | 2.2 |
| BP-1 | Land | 65.0 | 33.8 | 36.3 | 4.89 | 1.7 |
| BP-2 | Go to start | 54.4 | 22.2 | 26.8 | 0.66 | 1.8 |
| BP-2 | Ready | 31.1 | 15.9 | 20.2 | 0.34 | 1.5 |
| BP-2 | Grasp | 34.9 | 21.8 | 19.4 | 2.06 | 2.2 |
| BP-2 | Push | 104.5 | 39.9 | 27.3 | 2.70 | 3.9 |
| BP-2 | Release | 129.2 | 73.5 | 76.0 | 2.16 | 3.9 |
| BP-2 | Exit | 128.7 | 54.4 | 57.4 | 1.80 | 3.7 |
| BP-2 | Go to land start | 61.1 | 33.1 | 34.4 | 2.66 | 1.9 |
| BP-2 | Land | 50.3 | 24.5 | 33.1 | 1.49 | 1.8 |

The impedance residual per contact window (`metrics_impedance`; F_ext = the controller's estimate):

| run | window | T [s] | e_imp rms [N] | F_ext rms [N] | F_hat along push, mean [N] | box model along push, mean [N] | e_y rms [mm] | e_imp as displacement rms [mm] |
|---|---|---|---|---|---|---|---|---|
| BP-1 | CONTACT | 25.4 | 4.70 | 2.08 | -1.09 | -0.12 | 24.3 | 25.4 |
| BP-1 | HOLD | 2.5 | 2.42 | 1.19 | +0.28 | +0.25 | 18.2 | 15.5 |
| BP-1 | SLIDE | 19.1 | 4.86 | 2.22 | -1.32 | -0.22 | 25.1 | 26.3 |
| BP-1 | RELEASE | 3.7 | 5.01 | 1.80 | -0.87 | +0.10 | 23.8 | 26.0 |
| BP-2 | CONTACT | 21.6 | 6.06 | 2.84 | -0.88 | -0.18 | 32.8 | 32.5 |
| BP-2 | HOLD | 2.6 | 6.01 | 1.94 | -0.21 | -0.04 | 30.2 | 32.3 |
| BP-2 | SLIDE | 17.5 | 4.96 | 1.89 | -0.45 | -0.21 | 26.3 | 26.5 |
| BP-2 | RELEASE | 1.5 | 13.40 | 8.22 | -7.07 | -0.03 | 76.1 | 72.0 |

Along the push, a NEGATIVE force on the claw is the box resisting the push. The box model needs mu m g = 0.69 N to slide.

The box, the release window and the abort (`metrics.box`, `metrics.release`, `meta`):

| run | box pushed [mm] | box yaw change [deg] | box tilt max [deg] | box moved in the exit [mm] | q2 max, release + exit [deg] | q3 min, release + exit [deg] | abort at [s] | abort reason |
|---|---|---|---|---|---|---|---|---|
| BP-1 | 390 | +0.2 | 1.54 | 0.3 | 45.7 | 2.1 | 55.89 | (SAFETY GUARD: q3 13.0 deg falling at 100.9 deg/s -> 6.3 deg in 0.20 s (13.5 deg of the fall stopped by q2's 45.0 deg wall: q2 43.2 deg rising at 76.3 deg/s) < 10.0 deg (the arm folding toward its elbow-singular branch)) |
| BP-2 | 352 | +3.1 | 3.06 | 0.2 | 45.2 | -3.8 | 50.08 | (SAFETY GUARD: q3 10.5 deg falling at 21.2 deg/s -> 6.3 deg in 0.20 s (q2 44.8 deg) < 10.0 deg (the arm folding toward its elbow-singular branch)) |

## Conditions

- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm in torque mode, 3.746 kg in the controller model.
- **Feedback:** raw OptiTrack position and velocity with the raw-mocap guard, PX4 attitude; joint encoders.
- **Controller:** whole-body L1, 4-D attribution, `AM_HW_PROFILE=box_pushing`
  (`configs/whole_body_l1_4d_hw_box_pushing.yaml`): M_d = diag(1, 1.64, 1, 0.05), D_d = diag(20.2, 33.128, 20.2, 0.2903), K_d = diag(120, 196.8, 120, 0.2484) on [x, y, z, heading], world axes.
- **Task:** push 0.40 m at 21 mm/s (19.00 s after a 1.5 s settle), push pose [0, 28, 32, 0] deg, EE offset (0, 0, 0.183) m above the box's origin.
- **Table** (the planner's keep-out, placed from the box): top at 0.80 m, yaw -90.0 deg (`meta.task_values.table_corners_xy_m`).
- **Safety guards:** box tilt > 2 deg, box lag > 40 mm, force reading > 5 N, q3 < 10 deg (static and
  rate-aware, q2-wall aware), vehicle tilt > 10 deg (planner) / 15 deg (node), the raw-mocap guard.

## The struct `run`

| field | content |
|---|---|
| `meta` | name, run tag, method, flight date, original bag, outcome, task values, phase table, window, clock, DIRECT entry/exit, abort time and reason, the impedance windows and gains, the force source, largest gap per stream, box samples removed, leg goals, configs |
| `metrics` | `window` (whole mission; `whole_recording` = the same), `per_phase.<phase>`, `impedance.<window>`, `box`, `release` (the release window's joint margins) |
| `tracking` | 100 Hz over the mission, `t` in s since the start of go to start: platform position and attitude (measured, `_ref_`, `_err_`), `eps_uav_m` / `rho_uav`, the system CoM, the end-effector position and heading, the joints (`q_deg`), and `box_pos_m` |
| `impedance` | over the CONTACT window: `e_imp`, `e_imp_mm`, `F_ext` (the estimate), `rendered`, `e_y`, the two along-push forces, `windows` |
| `raw` | the bag's streams, time in s since the bag's first message: `odom`, `joints` (model convention), `reference` + `reference_derivatives` (the planner's whole-body reference), `box` (obj_0), `mocap_vrpn` (vehicle), `gripper` (finger positions), `controller_debug` (`wb_control_debug`), `push_monitor` (the planner's guard readings), `motors` (normalised), `battery`, `marks` (the phase marks), `events.mode` / `events.planner_status` / `events.push_pull_status` (transitions), `events.log` (every node's /rosout line) |

Units: positions in m (world ENU, z up) unless the name says mm; angles in deg unless it says rad. Errors are
measured − reference. `platform_att_err_deg` columns are `[roll pitch heading]`.

**Definitions** (the simulation's, `results/utils/build_box_pushing.py`):

- **Reference:** the planner's dynamically compatible whole-body reference. Platform
  x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>); R<sub>0,d</sub> from the thrust
  direction (CoM reference acceleration + g e<sub>3</sub>) and the reference heading; EE r<sub>e,d</sub>, heading
  b<sub>1,e,d</sub>; joints q<sub>d</sub>.
- **ε<sub>UAV</sub>** = ‖x<sub>b</sub> − r<sub>UAV</sub>‖; ρ<sub>UAV</sub> = ε<sub>UAV</sub> / L, L = 0.370 m.
- **Impedance residual:** e<sub>imp</sub> = F<sub>ext</sub> − (M<sub>d</sub> ë + D<sub>d</sub> ė + K<sub>d</sub> e)
  on the claw's world error chains, every term low-passed at 5 Hz (zero phase).

Regenerate (from the bags in `results/experiment_results/flight_test_data/box_pushing/`):

```bash
/usr/bin/python3 results/utils/experiment_box_pushing.py
```
