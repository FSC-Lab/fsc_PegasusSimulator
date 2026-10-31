# Simulation results, MATLAB data (box pushing)

This is the data behind the paper's box-pushing table (`tab:sim_box_push`) and its figures: the
physical-interaction task, flown in Isaac Sim by the proposed whole-body L1 impedance controller
(8 runs of one mission). It is one task folder of `matlab_simulation_data/`, self-contained: these
files plus MATLAB are all you need. The free-flight and pick-and-place data are in the folders beside it,
with the same plant, the same `run` struct layout and the same notation; this folder adds the IMPEDANCE
RESIDUAL and the box.

## Files

```
<method>_push_run<k>.mat                 one run, struct `run`
box_pushing_metrics.mat                  metrics_mean = the table (mean over runs), metrics_runs = every run,
                                         metrics_phase = every run x phase, metrics_impedance = every run x
                                         contact window, phases = the common phase table, window_s, L_arm_m
configs/<method>_sim_box_pushing.yaml    the controller + planner (push-and-pull block) + plant, as flown
reference_outputs/                       the same table and figures drawn by the Python scorer
                                         (box_pushing_sim.csv / .tex, sim_box_push_*.png): check a MATLAB
                                         table or plot against them
```

- **`<method>`:** `whole_body_l1` = Proposed (whole-body 4-D L1 impedance).
  `coordinated_admittance` = Liu et al., CEP 146 (2024) 105898 (decoupled UAV PD + momentum wrench
  estimator + admittance on the end-effector reference + force-adaptive kinematic MPC; position-mode arm),
  an independent comparison rig -- its platform / arm references in `run` are that rig's own planner commands.
  `arbitrary_time` = Chen et al., IEEE TIE 72(12) (2025) 13693 (ACTNDO joint-torque observer + TSKFE fuzzy
  estimator + arbitrary-convergence-time quadrotor law + joint-space arm admittance; torque-mode arm), an
  independent comparison rig -- its platform / attitude references in `run` are that law's own commands
  (the planner's stream converted to a base reference, plus its contact re-seat), the arm reference the
  stream's q_d.

## Quick start in MATLAB

```matlab
S = load('box_pushing_metrics.mat');
T = struct2table(S.metrics_mean)                 % THE TABLE: one row per method, mean over the runs
R = struct2table(S.metrics_runs);                % every run (the spread between runs)
P = S.phases;                                    % the eight phases, common to every run
r = load('whole_body_l1_push_run1.mat').run;
tr = r.tracking;  w = tr.in_window > 0;          % 100 Hz, t = 0 at the start of go to start

% (1) the platform deviation with the phase bands
figure; hold on
for i = 1:numel(P.start_s)                       % phase bands, alternating shades
    c = 0.90 + 0.07*mod(i+1, 2);
    patch([P.start_s(i) P.end_s(i) P.end_s(i) P.start_s(i)], [0 0 200 200], c*[1 1 1], 'EdgeColor', 'none');
    text(mean([P.start_s(i) P.end_s(i)]), 195, P.label{i}, 'HorizontalAlignment', 'center');
end
plot(tr.t(w), 1e3 * tr.eps_uav_m(w))             % ||eps_UAV|| [mm]
xlim([0 S.window_s]); xlabel('Time (s)'); ylabel('||\epsilon_{UAV}|| (mm)')
yyaxis right; ylim([0 200] / (1e3 * S.L_arm_m)); ylabel('\rho_{UAV}')

% (2) the contact force and the impedance residual over the contact window (x, y, z in N; col 4 = heading, N.m)
im = r.impedance;  W = im.windows;
figure; tiledlayout(3, 1)
nexttile; plot(im.t, im.F_ext(:,1:3)); ylabel('F_{ext} (N)'); legend('x', 'y', 'z')
nexttile; plot(im.t, im.e_imp(:,1:3)); ylabel('e_{imp} (N)')
nexttile; plot(im.t, vecnorm(im.e_imp(:,1:3), 2, 2)); ylabel('||e_{imp}|| (N)'); xlabel('Time (s)')
s = strcmp(W.name, 'SLIDE');  for k = 1:3, nexttile(k); xline([W.start_s(s) W.end_s(s)], ':'); end

% (3) 3-D: references (dashed), measured platform and end effector, the box (ground truth), the table
figure; hold on; grid on; axis equal; view(-50, 28)
plot3(tr.platform_pos_ref_m(w,1), tr.platform_pos_ref_m(w,2), tr.platform_pos_ref_m(w,3), 'k--')
plot3(tr.ee_pos_ref_m(w,1), tr.ee_pos_ref_m(w,2), tr.ee_pos_ref_m(w,3), '--', 'Color', [0.45 0.45 0.45])
plot3(tr.platform_pos_m(w,1), tr.platform_pos_m(w,2), tr.platform_pos_m(w,3))
plot3(tr.ee_pos_m(w,1), tr.ee_pos_m(w,2), tr.ee_pos_m(w,3), ':')
tb = r.raw.box.t - r.meta.t0_driver_s;  wb = tb >= 0 & tb <= S.window_s;   % raw streams: driver clock
plot3(r.raw.box.pos_m(wb,1), r.raw.box.pos_m(wb,2), r.raw.box.pos_m(wb,3))
tv = r.meta.task_values;  tx = tv.table_x_m([1 2 2 1 1]);  ty = tv.table_y_m([1 1 2 2 1]);
plot3(tx, ty, tv.table_top_z_m * ones(1, 5), 'Color', [0.53 0.6 0.67])     % the table top

% (4) the error grid: platform / end effector xyz in mm, headings and joints in deg
E = {1e3*tr.platform_pos_err_m, tr.platform_att_err_deg(:,3), 1e3*tr.ee_pos_err_m, tr.ee_heading_err_deg, tr.q_err_deg};

% (5) one phase's numbers, e.g. the push, for every run
Ph = struct2table(S.metrics_phase);  Ph(strcmp(Ph.phase, 'push'), :)
```

## Timing: one timetable for every run

The flight driver starts every step at the same mission time on every run
(`pl_mission.py --timetable`, slots 11, 2.5, 7, 27, 3, 8.5, 40 s, each from the previous step's
start; `meta.timetable_slots_s` names them). The phases are contiguous: each runs from its first step's start
to the next phase's start. Only the GRASP phase's own length varies (the claw's arrival within 2 cm / 3 mm
across the fin, then the jaws' close), because a gripped box must be pushed in CONTACT at once -- so the
phases after it shift by that much between runs.
`t = 0` is the start of the Proposed rig's first transit; the methods are ALIGNED AT READY (from Ready on
every rig flies the same slots; a comparison rig's longer first transit starts before t = 0 and the
window drops that part; `meta.t_shift_s` is the shift applied).
The phase table below is the mean over the runs; every run's phase starts from Ready on agree with it to 0.60 s.

| # | phase | start [s] | end [s] | duration [s] | what happens |
|---|---|---|---|---|---|
| 1 | Go to start | -11.87 | 11.01 | 22.88 | fly from the hover to the start waypoint (the claw 0.12 m behind and 0.15 m above the fin, the arm already in the push pose); hover |
| 2 | Ready | 11.01 | 18.01 | 7.01 | open the gripper, hover (CoM-anchored), descend diagonally onto the fin; the claw world-held from its arrival |
| 3 | Grasp | 18.01 | 18.98 | 0.97 | wait for the claw within 2 cm / 3 mm across the fin, close the gripper on the fin (the jaws' stall) |
| 4 | Push | 18.98 | 45.98 | 27.00 | the law in CONTACT: 1.5 s settle, then the 0.50 m slide over 24 s, the arm held still; hold on the handle |
| 5 | Release | 45.98 | 46.46 | 0.47 | Exit To Push: the planner releases the claw (its reference back to the CoM anchor, the contact phase off), then the gripper opens; the exit starts on its OPEN report |
| 6 | Exit | 46.46 | 54.96 | 8.51 | climb 0.20 m with the jaws open, fold the arm home; hover |
| 7 | Go to land start | 54.96 | 94.97 | 40.00 | one planner transition off the table to the landing hover (-0.5, 0, 1.3) m, heading turning 270 deg clockwise; hover |
| 8 | Land | 94.97 | 99.84 | 4.87 | straight down to the land hover at z 0.8 m (the planner's last leg; the touchdown after it is not scored) |

- **Window:** every metric covers the whole mission, 0 to 99.84 s. `run.tracking.in_window`
  marks it; `run.tracking.phase_id` (1..8) says which phase each sample is in.
- **Contact windows** (`run.impedance.windows`, `run.metrics.impedance.<window>`): CONTACT = ready end to
  exit start (the law's contact phase, the claw world-held); HOLD = the grip and the 1.5 s settle;
  SLIDE = the 24 s move; RELEASE = push end to exit start (the jaws open).

## Simulation index

Flown 2026-10-17 21:24 to 2026-10-25 12:46 (attempt end times), headless Isaac Sim at real time, raw mocap-emulator feedback.

| method | file | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | e_imp slide rms [N] | e_imp contact rms [N] | box pushed [mm] |
|---|---|---|---|---|---|---|---|---|---|
| Proposed | `whole_body_l1_push_run1.mat` | 55.0 | 0.149 | 16.2 | 0.044 | 17.7 | 0.610 | 0.620 | 498 |
| Proposed | `whole_body_l1_push_run2.mat` | 39.0 | 0.105 | 15.3 | 0.041 | 16.9 | 0.635 | 0.640 | 489 |
| Proposed | `whole_body_l1_push_run3.mat` | 40.1 | 0.108 | 15.9 | 0.043 | 16.7 | 0.797 | 0.775 | 492 |
| Coordinated [Liu 2024] | `coordinated_admittance_push_run1.mat` | 122.2 | 0.330 | 30.3 | 0.082 | 33.4 | 2.220 | 2.148 | 511 |
| Coordinated [Liu 2024] | `coordinated_admittance_push_run3.mat` | 106.4 | 0.288 | 30.0 | 0.081 | 32.4 | 2.152 | 2.027 | 506 |
| Arbitrary-time [Chen 2025] | `arbitrary_time_push_run1.mat` | 44.7 | 0.121 | 10.6 | 0.029 | 11.6 | 9.289 | 8.838 | 497 |
| Arbitrary-time [Chen 2025] | `arbitrary_time_push_run2.mat` | 74.1 | 0.200 | 17.1 | 0.046 | 18.5 | 17.831 | 20.126 | 522 |
| Arbitrary-time [Chen 2025] | `arbitrary_time_push_run3.mat` | 56.6 | 0.153 | 15.0 | 0.040 | 15.5 | 16.561 | 15.382 | 533 |

The table is the mean ± sample standard deviation (ddof 1) over each method's kept runs
(`metrics_mean` / `metrics_std` in `box_pushing_metrics.mat`):

| method | runs | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | e_imp slide rms [N] | e_imp contact rms [N] | box pushed [mm] |
|---|---|---|---|---|---|---|---|---|---|
| Proposed | 3 | 44.7 ± 9.0 | 0.121 ± 0.024 | 15.8 ± 0.4 | 0.043 ± 0.001 | 17.1 ± 0.5 | 0.681 ± 0.102 | 0.678 ± 0.084 | 493 ± 4 |
| Coordinated [Liu 2024] | 2 | 114.3 ± 11.2 | 0.309 ± 0.030 | 30.1 ± 0.2 | 0.081 ± 0.000 | 32.9 ± 0.7 | 2.186 ± 0.048 | 2.087 ± 0.086 | 508 ± 4 |
| Arbitrary-time [Chen 2025] | 3 | 58.5 ± 14.8 | 0.158 ± 0.040 | 14.2 ± 3.3 | 0.038 ± 0.009 | 15.2 ± 3.5 | 14.560 ± 4.609 | 14.782 ± 5.668 | 517 ± 18 |

The impedance residual per contact window (`metrics_impedance`):

| run | window | T [s] | e_imp rms [N] | e_imp max [N] | x / y / z rms [N] | heading rms [N.m] | F_ext rms [N] | e_y rms [mm] | e_imp as displacement rms [mm] |
|---|---|---|---|---|---|---|---|---|---|
| `arbitrary_time_push_run1` | CONTACT | 29.9 | 8.838 | 40.092 | 1.367 / 8.724 / 0.375 | 0.3011 | 1.06 | 12.8 | 15.49 |
| `arbitrary_time_push_run1` | HOLD | 3.9 | 4.393 | 11.975 | 0.922 / 4.291 / 0.182 | 0.1771 | 0.24 | 6.9 | 13.51 |
| `arbitrary_time_push_run1` | SLIDE | 23.9 | 9.289 | 40.092 | 1.325 / 9.185 / 0.394 | 0.2861 | 1.14 | 13.9 | 15.71 |
| `arbitrary_time_push_run1` | RELEASE | 2.1 | 9.695 | 30.031 | 2.236 / 9.424 / 0.416 | 0.5440 | 0.97 | 7.7 | 16.40 |
| `arbitrary_time_push_run2` | CONTACT | 30.3 | 20.126 | 153.315 | 2.116 / 19.992 / 0.937 | 0.4620 | 0.97 | 25.0 | 32.68 |
| `arbitrary_time_push_run2` | HOLD | 4.1 | 7.402 | 22.812 | 1.365 / 7.272 / 0.220 | 0.3265 | 0.31 | 9.3 | 14.89 |
| `arbitrary_time_push_run2` | SLIDE | 23.9 | 17.831 | 50.559 | 1.498 / 17.747 / 0.850 | 0.3361 | 1.04 | 24.1 | 27.73 |
| `arbitrary_time_push_run2` | RELEASE | 2.4 | 43.561 | 153.315 | 5.620 / 43.152 / 1.976 | 1.1892 | 0.95 | 45.2 | 74.63 |
| `arbitrary_time_push_run3` | CONTACT | 30.1 | 15.382 | 51.198 | 1.814 / 15.256 / 0.754 | 0.4437 | 1.00 | 20.9 | 25.54 |
| `arbitrary_time_push_run3` | HOLD | 4.0 | 5.939 | 20.512 | 1.753 / 5.671 / 0.182 | 0.4095 | 0.38 | 7.2 | 14.32 |
| `arbitrary_time_push_run3` | SLIDE | 23.9 | 16.561 | 51.198 | 1.118 / 16.505 / 0.767 | 0.2520 | 1.08 | 22.6 | 26.53 |
| `arbitrary_time_push_run3` | RELEASE | 2.3 | 14.036 | 27.405 | 5.015 / 13.059 / 1.142 | 1.2853 | 0.94 | 19.5 | 30.03 |
| `coordinated_admittance_push_run1` | CONTACT | 28.0 | 2.148 | 4.913 | 0.116 / 2.096 / 0.456 | 0.0197 | 1.08 | 24.5 | 27.34 |
| `coordinated_admittance_push_run1` | HOLD | 2.4 | 0.608 | 0.938 | 0.100 / 0.511 / 0.314 | 0.0224 | 0.72 | 18.0 | 16.81 |
| `coordinated_admittance_push_run1` | SLIDE | 24.0 | 2.220 | 4.913 | 0.110 / 2.171 / 0.450 | 0.0188 | 1.12 | 25.4 | 27.31 |
| `coordinated_admittance_push_run1` | RELEASE | 1.6 | 2.467 | 2.842 | 0.197 / 2.369 / 0.659 | 0.0267 | 0.93 | 20.2 | 37.85 |
| `coordinated_admittance_push_run3` | CONTACT | 28.2 | 2.027 | 4.763 | 0.139 / 1.962 / 0.489 | 0.0293 | 1.10 | 25.0 | 28.61 |
| `coordinated_admittance_push_run3` | HOLD | 2.4 | 0.600 | 0.956 | 0.130 / 0.442 / 0.384 | 0.0256 | 0.78 | 19.7 | 20.50 |
| `coordinated_admittance_push_run3` | SLIDE | 24.1 | 2.152 | 4.763 | 0.142 / 2.092 / 0.484 | 0.0258 | 1.13 | 25.9 | 28.83 |
| `coordinated_admittance_push_run3` | RELEASE | 1.8 | 1.433 | 1.994 | 0.100 / 1.270 / 0.656 | 0.0612 | 1.01 | 18.1 | 34.25 |
| `whole_body_l1_push_run1` | CONTACT | 30.0 | 0.620 | 1.502 | 0.340 / 0.413 / 0.315 | 0.0516 | 1.19 | 9.4 | 5.17 |
| `whole_body_l1_push_run1` | HOLD | 3.9 | 0.562 | 1.121 | 0.157 / 0.439 / 0.314 | 0.0453 | 0.46 | 3.1 | 4.68 |
| `whole_body_l1_push_run1` | SLIDE | 24.1 | 0.610 | 1.452 | 0.353 / 0.391 / 0.308 | 0.0534 | 1.29 | 10.3 | 5.08 |
| `whole_body_l1_push_run1` | RELEASE | 2.0 | 0.817 | 1.502 | 0.426 / 0.577 / 0.391 | 0.0410 | 0.88 | 6.0 | 6.81 |
| `whole_body_l1_push_run2` | CONTACT | 29.6 | 0.640 | 1.698 | 0.278 / 0.439 / 0.374 | 0.0507 | 1.12 | 9.8 | 5.34 |
| `whole_body_l1_push_run2` | HOLD | 3.7 | 0.581 | 1.389 | 0.192 / 0.438 / 0.330 | 0.0469 | 0.49 | 3.1 | 4.84 |
| `whole_body_l1_push_run2` | SLIDE | 24.0 | 0.635 | 1.355 | 0.291 / 0.427 / 0.369 | 0.0515 | 1.19 | 10.8 | 5.29 |
| `whole_body_l1_push_run2` | RELEASE | 1.9 | 0.805 | 1.698 | 0.263 / 0.573 / 0.500 | 0.0462 | 1.03 | 4.4 | 6.71 |
| `whole_body_l1_push_run3` | CONTACT | 29.4 | 0.775 | 2.407 | 0.412 / 0.489 / 0.438 | 0.0520 | 0.99 | 9.9 | 6.46 |
| `whole_body_l1_push_run3` | HOLD | 3.5 | 0.676 | 1.994 | 0.250 / 0.426 / 0.463 | 0.0250 | 0.49 | 4.3 | 5.64 |
| `whole_body_l1_push_run3` | SLIDE | 24.1 | 0.797 | 2.407 | 0.441 / 0.508 / 0.428 | 0.0547 | 1.06 | 10.7 | 6.64 |
| `whole_body_l1_push_run3` | RELEASE | 1.9 | 0.648 | 1.097 | 0.240 / 0.328 / 0.505 | 0.0538 | 0.70 | 4.3 | 5.40 |

A force residual scales with the stiffness a rig declares, so its DISPLACEMENT equivalent is given too: the
declared compliance applied to e_imp -- how far the claw is from where its own declared impedance puts it
under the measured force (`impedance.e_imp_mm`, `metrics.impedance.<window>.e_imp_mm_rms`).

The box and the vehicle (`metrics.box`, `metrics.window`):

| run | box pushed [mm] | box yaw change [deg] | box tilt max [deg] | box moved in the exit [mm] | vehicle tilt max [deg] | q2 max, release + exit [deg] | q3 min, release + exit [deg] |
|---|---|---|---|---|---|---|---|
| `whole_body_l1_push_run1` | 498 | -3.3 | 0.36 | 1.3 | 3.9 | 39.0 | 27.2 |
| `whole_body_l1_push_run2` | 489 | -4.2 | 0.22 | 0.4 | 3.7 | 38.4 | 27.8 |
| `whole_body_l1_push_run3` | 492 | -5.9 | 2.22 | 0.6 | 3.3 | 38.8 | 27.4 |
| `coordinated_admittance_push_run1` | 511 | -0.8 | 0.25 | 55.0 | 4.9 | 41.7 | 26.3 |
| `coordinated_admittance_push_run3` | 506 | -4.1 | 0.37 | 62.9 | 4.0 | 41.4 | 25.9 |
| `arbitrary_time_push_run1` | 497 | -1.7 | 0.12 | 3.1 | 3.0 | 39.3 | 28.3 |
| `arbitrary_time_push_run2` | 522 | -5.0 | 0.29 | 2.9 | 4.0 | 39.3 | 25.8 |
| `arbitrary_time_push_run3` | 533 | -5.0 | 0.29 | 15.1 | 3.3 | 39.3 | 27.6 |

The last two columns are the release window's joint margins: q2's stop is +45 deg, the planner's arm
guard aborts below q3 = 10 deg (see the attempts below).

Attempts (a run is kept only if it completed, pushed the box >= 0.45 m and kept every step on its slot):

| method | attempts | kept | why the others were not kept |
|---|---|---|---|
| Arbitrary-time [Chen 2025] | 5 | 3 | [  88.87s] push_pull: ABORTING (SAFETY GUARD: the box was dragged 52 mm sideways (> 45 mm: toward the table's edge)) -- gripper open, contact off, climbing 0.30; [ 102.35s] push_pull: ABORTING (SAFETY GUARD: the box was dragged 57 mm sideways (> 45 mm: toward the table's edge)) -- gripper open, contact off, climbing 0.30 |
| Coordinated [Liu 2024] | 6 | 2 | [  82.45s] push_pull: ABORTING (SAFETY GUARD: the box is 47 mm BEHIND its plan (> 40 mm: jammed or too heavy)) -- gripper open, contact off, climbing 0.30 m, th; [  84.34s] push_pull: ABORTING (SAFETY GUARD: the box is 45 mm BEHIND its plan (> 40 mm: jammed or too heavy)) -- gripper open, contact off, climbing 0.30 m, th; [  84.95s] push_pull: ABORTING (SAFETY GUARD: the box is 44 mm BEHIND its plan (> 40 mm: jammed or too heavy)) -- gripper open, contact off, climbing 0.30 m, th; [  85.23s] push_pull: ABORTING (SAFETY GUARD: the box is 47 mm BEHIND its plan (> 40 mm: jammed or too heavy)) -- gripper open, contact off, climbing 0.30 m, th |
| Proposed | 3 | 3 | -- |

The failed attempts (kept whole in the repository under `results/simulation_results/Isaac_simulation_data/box_pushing/failed/`):

| attempt | what stopped it | when | afterwards |
|---|---|---|---|
| `arbitrary_time_push_run1_attempt1` | planner abort (SAFETY GUARD: the box was dragged 52 mm sideways (> 45 mm: toward the table's edge)) | at 88.9 s | landed upright (tilt max 13 deg, final height 0.24 m) |
| `arbitrary_time_push_run3_attempt1` | planner abort (SAFETY GUARD: the box was dragged 57 mm sideways (> 45 mm: toward the table's edge)) | +1.19 s after the jaws opened | landed upright (tilt max 22 deg, final height 0.24 m) |
| `coordinated_admittance_push_run2_attempt1` | planner abort (SAFETY GUARD: the box is 44 mm BEHIND its plan (> 40 mm: jammed or too heavy)) | at 85.0 s | landed upright (tilt max 4 deg, final height 0.24 m) |
| `coordinated_admittance_push_run2_attempt2` | planner abort (SAFETY GUARD: the box is 45 mm BEHIND its plan (> 40 mm: jammed or too heavy)) | at 84.3 s | landed upright (tilt max 5 deg, final height 0.24 m) |
| `coordinated_admittance_push_run2_attempt3` | planner abort (SAFETY GUARD: the box is 47 mm BEHIND its plan (> 40 mm: jammed or too heavy)) | at 82.5 s | landed upright (tilt max 4 deg, final height 0.24 m) |
| `coordinated_admittance_push_run3_attempt1` | planner abort (SAFETY GUARD: the box is 47 mm BEHIND its plan (> 40 mm: jammed or too heavy)) | at 85.2 s | landed upright (tilt max 4 deg, final height 0.24 m) |

## Scene and task

- **Table:** the HOOBRO console table (1.000 x 0.150 x 0.800 m), its length along world y: x 1.200-1.350 m, y -0.5..0.5 m, top at 0.80 m.
- **Box:** the CAD box (240 x 160 x 100 mm) with its clamp bracket and 5 mm fin handle, **500 g**
  in total, friction 0.25 static / 0.25 sliding (the real box on the experiment table, spring-gauge bench test 2026-10-17), 1.23 N to slide. It starts
  on the table's centre line, its back face 0.10 m from the table's +y end, and is pushed toward -y (to 0.60 m). The sim claw cannot close below ~19 mm,
  so the fin's bare part carries an invisible 20 mm grip shim (the real gripper's foam pads close on the
  5 mm fin).
- **Task:** grasp the fin from above (go to start, ready, grasp), push the box 0.50 m along the
  table in 24 s (a min-snap move after a 1.5 s settle) with the arm held still and the
  claw held in the world, release, climb out, fly off the table to the landing hover, descend. The whole-body law's
  contact phase (chi) is CONTACT from the jaws' grip to the end of the push.
- **Arm pose in the push:** [0, 32, 28, 0] deg (fold 60 deg); airframe: the T650
  landing gear; the real arm's q2 range [-20, 45] deg is enforced as hard stops in the scene.
- **Safety guards (on in every attempt, every rig; none tripped in the kept runs; 6 failed attempts above):** the planner refuses a push that would leave the box
  within 3 cm of the table edge, and aborts on a box lagging > 40 mm behind its plan, a box tilt > 2 deg,
  q3 < 10 deg, a lost box feed, a task force > 5 N or a vehicle tilt > 15 deg.
- **Impedance, Proposed (the whole-body law's EE task):** M_d = diag(1, 1, 1, 0.05),
  D_d = diag(20.2, 20.2, 20.2, 0.2903), K_d = diag(120, 120, 120, 0.2484)
  on [x, y, z, heading] (kg, N.s/m, N/m; kg.m^2, N.m.s/rad, N.m/rad for the heading).
- **Admittance, Coordinated [Liu 2024] (eq 27, contact phase, claw frame [along, lateral, vertical]):** M_d = diag(0.5, 0.5, 0.5),
  D_d = diag(17.3, 20, 20), K_d = diag(150, 20, 20) (kg, N.s/m, N/m); its contact phase runs
  from the jaws' stall to the release. That rig flies the planner at 0.20 m/s / 0.08 m/s^2 / 0.20 rad/s
  (the whole-body rig 0.30 / 0.15 / 0.30) and hovers 15 s before Ready, so its first transit starts
  before t = 0; from Ready on every rig flies the same timetable. Its box-lag guard trips are its
  decoupled base lagging the planned push (2026-10-17 campaign: the box 35-48 mm behind its plan at the
  push's peak speed, the whole-body rig 15-22 mm), not a jammed box. After the release its arm closes the remaining lag
  with the jaws open around the fin and slides the box along the push (`box moved in the exit`).
- **Admittance, Arbitrary-time [Chen 2025] (eq 34, JOINT space, the arm relative to the base):** H_d = diag(0.3333, 0.3333, 0.3333, 0.3333), D_d = diag(1.34, 1.34, 1.34, 1.34),
  K_d = diag(3, 3, 3, 3) (kg.m^2, N.m.s/rad, N.m/rad, as flown). At the push
  pose this declares 686 N/m along the arm (= the push) and 210 N/m across it (J^-T K_d J^-1, `meta.impedance_gains`);
  its residual is scored against that task-space image exactly as the other rigs' (the claw's world error
  chains, the true contact wrench). Its OWN safety guards (the other rigs' untouched): box lag 100 mm, contact force 3.5 N on its own estimate, and one guard the
  other rigs do not carry: the box dragged sideways more than 45 mm (`push_pull_guard_box_cross`) --
  its base-relative arm drags the clamped box with the base's sideways swings (up to 35 mm in completed
  pushes; twice past the table's side edge in tuning, the whole-body and coordinated rigs 2-5 mm).
  That rig flies the planner of 2026-10-24 (Go To Start
  round the table, ~19 s; the other rigs' runs flew the straight 10.2 s transition of 2026-10-17) and hovers
  15 s before Ready, so its first transit starts before t = 0; from Ready on every rig flies the same timetable.
  Declared deviations from the paper (archive/box_pushing_arbitrary_time_20261024/README.md): the floored
  time-to-go, the observer gain and arm window softened for the arm's torque delay, the arm stack's friction
  feed-forward off, a bumpless contact transfer (the base reference re-seated at the jaws' stall), and the
  admittance at one third of the paper's stiffness with the paper's dynamics kept (H/D/K 1/4/9 -> 1/3, 1.34,
  3: omega 3 rad/s, zeta 0.67; tuned 2026-10-25, K 9 completed 1 push in 8).
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

- **The push-and-pull planner block as flown** (`configs/`):

```
push_pull_ee_offset: [0.0, 0.0, 0.183]
push_pull_yaw_from_box_deg: -90.0
push_pull_push_pose_deg: [0.0, 32.0, 28.0, 0.0]
push_pull_approach_dz: 0.15
push_pull_approach_back: 0.12
push_pull_exit_dz: 0.20
push_pull_contact_at_ready: false
push_pull_contact_off_at_push_end: true
push_pull_push_distance: 0.50
push_pull_push_time_s: 24.00
push_pull_push_settle_s: 1.5
push_pull_push_ramp_s: 0.0
push_pull_exit_time_s: 3.0
push_pull_unload_time_s: 0.0
push_pull_world_anchor_push: true
push_pull_world_anchor_ready: false
push_pull_world_anchor_on_handle: true
push_pull_descent_trim: false
push_pull_land: [-0.5, 0.0, 0.8, -360.0]
push_pull_land_start: [-0.5, 0.0, 1.3, -360.0]
push_pull_start_mark: [0.0, 0.0, 1.0, 0.0]
push_pull_table: [1.200, 1.350, -0.5, 0.5, 0.80]
push_pull_table_clearance: 0.05
push_pull_gear_depth: 0.2425
push_pull_fence_max_z: 2.2
push_pull_guard_tilt_deg: 15.0
push_pull_guard_force_n: 5.0
push_pull_abort_from_measured: true
push_pull_abort_back: 0.03
push_pull_abort_release_wait_s: 1.0
push_pull_box_half_length: 0.12
push_pull_box_edge_margin: 0.03
push_pull_guard_box_lag: 0.040
push_pull_guard_box_tilt_deg: 2.0
push_pull_guard_box_persist_s: 0.20
push_pull_guard_box_stale_s: 0.50
push_pull_guard_q3_min_deg: 10.0
```

## The struct `run`

| field | content |
|---|---|
| `meta` | method, run index, `scene`, `task`, `task_values` (box mass, friction, push distance / time / settle, push pose, the table's x / y range and top, Land Start, Land), plant, `phases` (key, label, start_s, end_s of this run), `timetable_slots_s`, `late_steps` (empty), `window`, `impedance_windows`, `impedance_gains` (M_d, D_d, K_d), `L_arm_m`, `base_com_model_m`, `t0_driver_s` |
| `metrics` | `window` = this run's table numbers over the window; `per_phase.<key>` = the same per phase; `whole_recording` = over every sample; `impedance.<window>` = the impedance residual per contact window (rms / max, per channel, with |F_ext| and |e_y|); `box` = pushed distance, yaw change, movement during the exit, final position |
| `tracking` | 100 Hz, `t` in s since the start of the mission; `in_window`, `phase_id`; measured, reference (`_ref_`) and error (`_err_`) for the platform position, `eps_uav_m`, `rho_uav`, the platform attitude, the system CoM, the end-effector position and heading, and the joints |
| `impedance` | the residual's time series over the CONTACT window (contact-report rate, ~62.5 Hz): `t`, `e_imp`, `F_ext` (the true contact wrench on the claw), `rendered` (M_d edot_v + D_d e_v + K_d e_y), `e_y`, and `windows` (name, start_s, end_s) |
| `raw` | every recorded stream on the driver's clock (subtract `meta.t0_driver_s` to get `tracking.t`): `odom`, `joints`, `reference` (the planner's whole-body reference), `reference_derivatives`, `box` (ground truth = the mocap's obj_0), `claw_truth`, `base_truth`, `contact_truth` (the box's PhysX contact report), `marks` (every step's start/end), `events` |

Units: positions in m (world ENU, z up) unless the field name says mm; angles in deg unless it says rad;
forces in N, moments in N.m. Errors are measured − reference. `platform_att_err_deg` columns are
`[roll pitch heading]`.

## Paper notation

| paper symbol | meaning | summary field (`metrics_mean`) | run file |
|---|---|---|---|
| e<sub>imp</sub> | F<sub>ext</sub> − (M<sub>d</sub> ė<sub>v</sub> + D<sub>d</sub> e<sub>v</sub> + K<sub>d</sub> e<sub>y</sub>), the impedance residual on the claw [N; heading N·m], rms over the slide / the whole contact | `e_imp_slide_rms_n`, `e_imp_slide_max_n`, `e_imp_slide_{x,y,z}_rms_n`, `e_imp_slide_psi_rms_nm`, `e_imp_contact_rms_n` | `impedance.e_imp` [N N N N·m], `metrics.impedance.<window>` |
| F<sub>ext</sub> | the true contact wrench on the claw (ground truth) | `F_ext_slide_rms_n` | `impedance.F_ext` |
| ‖ε<sub>UAV</sub>‖ | ‖r<sub>UAV</sub><sup>ref</sup> − r<sub>UAV</sub>‖, the platform's deviation from its reference position [mm] | `eps_uav_max_mm`, `eps_uav_rms_mm` | `tracking.eps_uav_m` [m] |
| ρ<sub>UAV</sub> | ‖ε<sub>UAV</sub>‖ / L, L = 0.370 m (the arm's reach) | `rho_uav_max`, `rho_uav_rms` | `tracking.rho_uav` |
| r<sub>0,x</sub>, r<sub>0,y</sub>, r<sub>0,z</sub> | platform position [mm] | `platform_x_mm`, `platform_y_mm`, `platform_z_mm` | `tracking.platform_pos_err_m(:,1:3)` [m] |
| ψ<sub>0</sub>, φ<sub>0</sub>, θ<sub>0</sub> | platform heading, roll, pitch [deg] | `platform_heading_deg`, `platform_roll_deg`, `platform_pitch_deg` | `tracking.platform_att_err_deg(:,[3 1 2])` |
| r<sub>e,x</sub>, r<sub>e,y</sub>, r<sub>e,z</sub> | end-effector position [mm] | `ee_x_mm`, `ee_y_mm`, `ee_z_mm` | `tracking.ee_pos_err_m(:,1:3)` [m] |
| ψ<sub>e</sub> | end-effector heading [deg] | `ee_heading_deg` | `tracking.ee_heading_err_deg` |
| q<sub>1</sub> … q<sub>4</sub> | joint angles [deg] | `q1_deg` … `q4_deg` | `tracking.q_err_deg(:,1:4)` |
| d<sub>box</sub> | the box's travel along the table during the push [mm] (0.50 m commanded) | `box_pushed_mm` | `metrics.box.pushed_mm`, `raw.box` |

- **Reference:** the planner's dynamically compatible whole-body reference. The platform reference
  x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>) is r<sub>UAV</sub><sup>ref</sup>.
  During the push the EE reference r<sub>e,d</sub> is the claw's planned path along the table (world-held).
- **max / rms:** over the window; the table's other columns are RMS errors.
- **e<sub>imp</sub>:** every term through one zero-phase 5 Hz low-pass; e<sub>y</sub> from the TRUE claw point
  (ground truth) minus the streamed reference; e<sub>v</sub>, ė<sub>v</sub> its derivatives against the streamed
  reference derivatives; F<sub>ext</sub> from the box's PhysX contact report (sign calibrated on the table's
  normal force). A residual of 0 means the claw moved exactly as the designed impedance under the measured force.
