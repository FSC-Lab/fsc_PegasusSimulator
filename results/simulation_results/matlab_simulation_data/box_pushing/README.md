# Simulation results, MATLAB data (box pushing)

This is the data behind the paper's box-pushing table (`tab:sim_box_push`) and its figures: the
physical-interaction task, flown in Isaac Sim by the proposed whole-body L1 impedance controller
(15 runs of one mission). It is one task folder of `matlab_simulation_data/`, self-contained: these
files plus MATLAB are all you need. The free-flight and pick-and-place data are in the folders beside it,
with the same plant, the same `run` struct layout and the same notation; this folder adds the IMPEDANCE
RESIDUAL and the box.

## Files

```
<method>_push_run<k>.mat                 one run, struct `run`
box_pushing_metrics.mat                  metrics_mean = the table (mean over runs), metrics_std = its sample std,
                                         metrics_best = each method's best run, metrics_runs = every run,
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
(`pl_mission.py --timetable`, slots 35, 2.5, 8, 22.05, 3, 8.5, 40 s, each from the previous step's
start; `meta.timetable_slots_s` names them). The phases are contiguous: each runs from its first step's start
to the next phase's start. Only the GRASP phase's own length varies (the claw's arrival within 2 cm / 3 mm
across the fin, then the jaws' close), because a gripped box must be pushed in CONTACT at once -- so the
phases after it shift by that much between runs.
`t = 0` is the start of the first transit (go to start); every rig starts it at the same mission time and
flies the same slots from there (the methods' Ready starts agree to 0.00 s; `meta.t_shift_s`).
The phase table below is the mean over the runs; every run's phase starts from Ready on agree with it to 0.65 s.

| # | phase | start [s] | end [s] | duration [s] | what happens |
|---|---|---|---|---|---|
| 1 | Go to start | 0.00 | 35.01 | 35.01 | fly from the hover round the table's box end (>= 0.70 m from the table seen from above) to the start waypoint (the claw 0.12 m behind and 0.15 m above the fin, the arm unfolding into the push pose); hover |
| 2 | Ready | 35.01 | 43.01 | 8.01 | open the gripper, hover (CoM-anchored), descend diagonally onto the fin; the claw stays CoM-anchored until it has settled near the grasp point (25 mm, < 30 mm/s), then is held in the room |
| 3 | Grasp | 43.01 | 43.98 | 0.97 | wait for the claw on the fin (the whole-body rig 2 cm / 3 mm across, the comparison rigs 2.5 cm / 4 mm against the live box), close the gripper on the fin (the jaws' stall) |
| 4 | Push | 43.98 | 66.04 | 22.06 | the law in CONTACT: 1.5 s settle, then the 0.40 m slide over 19.0 s, the arm held still; hold on the handle |
| 5 | Release | 66.04 | 66.39 | 0.35 | Exit To Push: the planner releases the claw (its reference back to the CoM anchor, the contact phase off), then the gripper opens; the exit starts on its OPEN report |
| 6 | Exit | 66.39 | 74.89 | 8.51 | climb 0.25 m with the jaws open, fold the arm home; hover |
| 7 | Go to land start | 74.89 | 114.90 | 40.01 | Go To Start's mirror image: on along the push line at the exit height over the pushed box, round the table's far end and back along its front to the landing hover (-1, 0, 1.3) m in the plan's frame, heading turning 90 deg clockwise; hover |
| 8 | Land | 114.90 | 122.31 | 7.42 | straight down to the land hover at z 0.3 m (the planner's last leg; the touchdown after it is not scored) |

- **Window:** every metric covers the whole mission, 0 to 122.31 s. `run.tracking.in_window`
  marks it; `run.tracking.phase_id` (1..8) says which phase each sample is in.
- **Contact windows** (`run.impedance.windows`, `run.metrics.impedance.<window>`): CONTACT = ready end to
  exit start (the law's contact phase, the claw world-held); HOLD = the grip and the 1.5 s settle;
  SLIDE = the 19.0 s move; RELEASE = push end to exit start (the jaws open).

## Simulation index

Flown 2026-10-28 20:42 to 2026-10-29 13:49 (attempt end times), headless Isaac Sim at real time, raw mocap-emulator feedback.

| method | file | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | e_imp slide rms [N] | e_imp contact rms [N] | box pushed [mm] |
|---|---|---|---|---|---|---|---|---|---|
| Proposed | `whole_body_l1_push_run1.mat` | 29.8 | 0.080 | 10.5 | 0.028 | 10.9 | 1.050 | 1.028 | 395 |
| Proposed | `whole_body_l1_push_run2.mat` | 27.7 | 0.075 | 9.5 | 0.026 | 9.5 | 1.075 | 1.037 | 407 |
| Proposed | `whole_body_l1_push_run3.mat` **(best)** | 27.3 | 0.074 | 8.9 | 0.024 | 9.3 | 0.776 | 0.705 | 399 |
| Proposed | `whole_body_l1_push_run4.mat` | 24.8 | 0.067 | 8.9 | 0.024 | 9.3 | 1.035 | 0.979 | 405 |
| Proposed | `whole_body_l1_push_run5.mat` | 25.2 | 0.068 | 9.4 | 0.025 | 9.8 | 1.031 | 0.968 | 399 |
| Coordinated [Liu 2024] | `coordinated_admittance_push_run1.mat` **(best)** | 111.8 | 0.302 | 37.7 | 0.102 | 34.4 | 1.929 | 1.931 | 406 |
| Coordinated [Liu 2024] | `coordinated_admittance_push_run2.mat` | 87.0 | 0.235 | 37.7 | 0.102 | 34.2 | 2.402 | 2.298 | 418 |
| Coordinated [Liu 2024] | `coordinated_admittance_push_run3.mat` | 81.8 | 0.221 | 38.6 | 0.104 | 34.6 | 3.220 | 3.157 | 421 |
| Coordinated [Liu 2024] | `coordinated_admittance_push_run4.mat` | 101.1 | 0.273 | 38.5 | 0.104 | 34.3 | 2.581 | 2.468 | 401 |
| Coordinated [Liu 2024] | `coordinated_admittance_push_run5.mat` | 96.9 | 0.262 | 38.7 | 0.104 | 35.0 | 2.765 | 2.685 | 404 |
| Arbitrary-time [Chen 2025] | `arbitrary_time_push_run1.mat` | 61.2 | 0.165 | 14.7 | 0.040 | 14.9 | 12.913 | 12.291 | 395 |
| Arbitrary-time [Chen 2025] | `arbitrary_time_push_run2.mat` | 65.2 | 0.176 | 14.3 | 0.039 | 14.5 | 11.058 | 10.028 | 405 |
| Arbitrary-time [Chen 2025] | `arbitrary_time_push_run3.mat` **(best)** | 51.7 | 0.140 | 11.5 | 0.031 | 11.7 | 8.072 | 7.538 | 405 |
| Arbitrary-time [Chen 2025] | `arbitrary_time_push_run4.mat` | 55.0 | 0.149 | 13.4 | 0.036 | 14.2 | 12.291 | 11.354 | 413 |
| Arbitrary-time [Chen 2025] | `arbitrary_time_push_run5.mat` | 56.1 | 0.152 | 14.0 | 0.038 | 14.4 | 13.024 | 11.843 | 410 |

The table is the mean ± sample standard deviation (ddof 1) over each method's kept runs
(`metrics_mean` / `metrics_std` in `box_pushing_metrics.mat`):

| method | runs | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | e_imp slide rms [N] | e_imp contact rms [N] | box pushed [mm] |
|---|---|---|---|---|---|---|---|---|---|
| Proposed | 5 | 26.9 ± 2.0 | 0.073 ± 0.005 | 9.5 ± 0.6 | 0.026 ± 0.002 | 9.8 ± 0.7 | 0.994 ± 0.123 | 0.944 ± 0.137 | 401 ± 5 |
| Coordinated [Liu 2024] | 5 | 95.7 ± 11.8 | 0.259 ± 0.032 | 38.2 ± 0.5 | 0.103 ± 0.001 | 34.5 ± 0.3 | 2.580 ± 0.474 | 2.508 ± 0.456 | 410 ± 9 |
| Arbitrary-time [Chen 2025] | 5 | 57.8 ± 5.3 | 0.156 ± 0.014 | 13.6 ± 1.3 | 0.037 ± 0.003 | 13.9 ± 1.3 | 11.472 ± 2.055 | 10.611 ± 1.916 | 406 ± 7 |

Each method's BEST run (`metrics_best` in `box_pushing_metrics.mat`, `reference_outputs/box_pushing_sim_best.*`):
the kept run with the lowest slide impedance residual, the physical-interaction metric; the figures plot
these runs.

| method | best run | eps max [mm] | rho max | eps rms [mm] | rho rms | EE RMSE [mm] | EE heading [deg] | e_imp slide rms [N] | as displacement [mm] | e_imp contact rms [N] | box pushed [mm] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Proposed | `whole_body_l1_push_run3` | 27.3 | 0.074 | 8.9 | 0.024 | 9.3 | 1.35 | 0.776 | 4.3 | 0.705 | 399 |
| Coordinated [Liu 2024] | `coordinated_admittance_push_run1` | 111.8 | 0.302 | 37.7 | 0.102 | 34.4 | 1.05 | 1.929 | 24.0 | 1.931 | 406 |
| Arbitrary-time [Chen 2025] | `arbitrary_time_push_run3` | 51.7 | 0.140 | 11.5 | 0.031 | 11.7 | 1.00 | 8.072 | 16.3 | 7.538 | 405 |

The impedance residual per contact window (`metrics_impedance`):

| run | window | T [s] | e_imp rms [N] | e_imp max [N] | x / y / z rms [N] | heading rms [N.m] | F_ext rms [N] | e_y rms [mm] | e_imp as displacement rms [mm] |
|---|---|---|---|---|---|---|---|---|---|
| `arbitrary_time_push_run1` | CONTACT | 26.1 | 12.291 | 42.882 | 3.155 / 11.858 / 0.717 | 0.6503 | 0.57 | 21.6 | 27.00 |
| `arbitrary_time_push_run1` | HOLD | 5.1 | 11.533 | 39.896 | 0.843 / 11.479 / 0.725 | 0.1646 | 0.31 | 9.8 | 21.01 |
| `arbitrary_time_push_run1` | SLIDE | 19.1 | 12.913 | 42.882 | 3.544 / 12.396 / 0.731 | 0.7285 | 0.63 | 24.5 | 28.60 |
| `arbitrary_time_push_run1` | RELEASE | 1.9 | 6.526 | 18.198 | 2.958 / 5.792 / 0.537 | 0.6406 | 0.51 | 10.5 | 24.70 |
| `arbitrary_time_push_run2` | CONTACT | 25.7 | 10.028 | 38.799 | 1.025 / 9.954 / 0.664 | 0.2129 | 0.58 | 15.7 | 22.71 |
| `arbitrary_time_push_run2` | HOLD | 4.7 | 5.508 | 16.723 | 1.055 / 5.398 / 0.291 | 0.1879 | 0.26 | 8.2 | 17.05 |
| `arbitrary_time_push_run2` | SLIDE | 19.1 | 11.058 | 38.799 | 1.057 / 10.982 / 0.742 | 0.2262 | 0.64 | 17.4 | 23.57 |
| `arbitrary_time_push_run2` | RELEASE | 1.9 | 7.568 | 19.966 | 0.475 / 7.538 / 0.477 | 0.1062 | 0.44 | 11.1 | 26.01 |
| `arbitrary_time_push_run3` | CONTACT | 26.0 | 7.538 | 33.149 | 1.954 / 7.268 / 0.418 | 0.4260 | 0.55 | 11.1 | 16.85 |
| `arbitrary_time_push_run3` | HOLD | 5.0 | 4.472 | 14.102 | 1.633 / 4.160 / 0.162 | 0.3393 | 0.21 | 7.3 | 15.26 |
| `arbitrary_time_push_run3` | SLIDE | 19.1 | 8.072 | 33.149 | 1.572 / 7.903 / 0.469 | 0.3232 | 0.62 | 12.1 | 16.25 |
| `arbitrary_time_push_run3` | RELEASE | 2.0 | 8.278 | 21.042 | 4.463 / 6.965 / 0.329 | 1.0504 | 0.38 | 8.6 | 24.67 |
| `arbitrary_time_push_run4` | CONTACT | 25.9 | 11.354 | 36.514 | 2.342 / 11.090 / 0.669 | 0.4752 | 0.57 | 19.8 | 25.19 |
| `arbitrary_time_push_run4` | HOLD | 5.0 | 8.141 | 29.300 | 1.417 / 8.000 / 0.515 | 0.2760 | 0.31 | 9.5 | 18.99 |
| `arbitrary_time_push_run4` | SLIDE | 19.1 | 12.291 | 36.514 | 2.416 / 12.031 / 0.701 | 0.4915 | 0.62 | 21.9 | 25.93 |
| `arbitrary_time_push_run4` | RELEASE | 1.9 | 8.371 | 21.396 | 3.329 / 7.649 / 0.700 | 0.6786 | 0.61 | 18.6 | 31.20 |
| `arbitrary_time_push_run5` | CONTACT | 25.7 | 11.843 | 46.308 | 2.252 / 11.606 / 0.697 | 0.4691 | 0.55 | 19.0 | 26.52 |
| `arbitrary_time_push_run5` | HOLD | 4.7 | 5.825 | 19.361 | 1.279 / 5.673 / 0.347 | 0.2307 | 0.25 | 7.5 | 18.54 |
| `arbitrary_time_push_run5` | SLIDE | 19.1 | 13.024 | 46.308 | 2.254 / 12.805 / 0.755 | 0.4681 | 0.61 | 21.1 | 27.52 |
| `arbitrary_time_push_run5` | RELEASE | 1.9 | 10.454 | 30.731 | 3.688 / 9.754 / 0.734 | 0.8043 | 0.52 | 16.9 | 32.51 |
| `coordinated_admittance_push_run1` | CONTACT | 24.1 | 1.931 | 4.446 | 0.223 / 1.889 / 0.331 | 0.0155 | 0.57 | 21.0 | 23.61 |
| `coordinated_admittance_push_run1` | HOLD | 3.5 | 1.556 | 2.212 | 0.154 / 1.534 / 0.206 | 0.0141 | 0.52 | 16.7 | 16.43 |
| `coordinated_admittance_push_run1` | SLIDE | 19.1 | 1.929 | 4.446 | 0.237 / 1.886 / 0.333 | 0.0152 | 0.59 | 21.6 | 23.98 |
| `coordinated_admittance_push_run1` | RELEASE | 1.6 | 2.591 | 2.800 | 0.167 / 2.537 / 0.500 | 0.0203 | 0.53 | 22.2 | 31.31 |
| `coordinated_admittance_push_run2` | CONTACT | 24.4 | 2.298 | 4.634 | 0.245 / 2.256 / 0.363 | 0.0270 | 0.57 | 22.1 | 26.55 |
| `coordinated_admittance_push_run2` | HOLD | 3.6 | 1.072 | 1.914 | 0.086 / 1.041 / 0.243 | 0.0126 | 0.54 | 15.8 | 14.63 |
| `coordinated_admittance_push_run2` | SLIDE | 19.1 | 2.402 | 4.634 | 0.272 / 2.360 / 0.358 | 0.0223 | 0.58 | 23.0 | 27.42 |
| `coordinated_admittance_push_run2` | RELEASE | 1.7 | 2.876 | 3.218 | 0.115 / 2.818 / 0.567 | 0.0667 | 0.41 | 23.1 | 34.49 |
| `coordinated_admittance_push_run3` | CONTACT | 23.6 | 3.157 | 5.990 | 0.193 / 3.125 / 0.408 | 0.0229 | 0.55 | 26.1 | 30.69 |
| `coordinated_admittance_push_run3` | HOLD | 2.8 | 1.349 | 2.500 | 0.105 / 1.310 / 0.302 | 0.0294 | 0.47 | 16.6 | 18.23 |
| `coordinated_admittance_push_run3` | SLIDE | 19.1 | 3.220 | 5.990 | 0.198 / 3.190 / 0.392 | 0.0224 | 0.58 | 26.5 | 30.58 |
| `coordinated_admittance_push_run3` | RELEASE | 1.7 | 4.295 | 4.724 | 0.232 / 4.239 / 0.648 | 0.0154 | 0.39 | 32.6 | 44.55 |
| `coordinated_admittance_push_run4` | CONTACT | 24.0 | 2.468 | 6.515 | 0.207 / 2.433 / 0.360 | 0.0216 | 0.56 | 22.8 | 26.33 |
| `coordinated_admittance_push_run4` | HOLD | 3.3 | 1.685 | 2.441 | 0.139 / 1.649 / 0.314 | 0.0113 | 0.39 | 19.5 | 20.40 |
| `coordinated_admittance_push_run4` | SLIDE | 19.1 | 2.581 | 6.515 | 0.202 / 2.550 / 0.345 | 0.0216 | 0.58 | 23.5 | 26.22 |
| `coordinated_admittance_push_run4` | RELEASE | 1.6 | 2.445 | 3.007 | 0.338 / 2.354 / 0.567 | 0.0345 | 0.65 | 19.8 | 36.53 |
| `coordinated_admittance_push_run5` | CONTACT | 23.9 | 2.685 | 5.671 | 0.241 / 2.650 / 0.358 | 0.0160 | 0.61 | 23.9 | 27.89 |
| `coordinated_admittance_push_run5` | HOLD | 3.3 | 0.970 | 1.654 | 0.083 / 0.929 / 0.266 | 0.0150 | 0.41 | 15.0 | 15.27 |
| `coordinated_admittance_push_run5` | SLIDE | 19.1 | 2.765 | 5.671 | 0.230 / 2.733 / 0.354 | 0.0161 | 0.63 | 24.7 | 27.87 |
| `coordinated_admittance_push_run5` | RELEASE | 1.5 | 3.892 | 4.866 | 0.486 / 3.824 / 0.537 | 0.0171 | 0.64 | 29.8 | 44.27 |
| `whole_body_l1_push_run1` | CONTACT | 25.7 | 1.028 | 2.479 | 0.339 / 0.946 / 0.220 | 0.0540 | 0.54 | 5.8 | 5.87 |
| `whole_body_l1_push_run1` | HOLD | 4.7 | 1.040 | 2.479 | 0.302 / 0.902 / 0.420 | 0.0567 | 0.28 | 5.1 | 6.29 |
| `whole_body_l1_push_run1` | SLIDE | 19.1 | 1.050 | 2.273 | 0.339 / 0.984 / 0.140 | 0.0528 | 0.60 | 6.0 | 5.86 |
| `whole_body_l1_push_run1` | RELEASE | 1.9 | 0.719 | 1.513 | 0.416 / 0.569 / 0.144 | 0.0593 | 0.42 | 4.9 | 4.67 |
| `whole_body_l1_push_run2` | CONTACT | 25.8 | 1.037 | 2.766 | 0.438 / 0.903 / 0.259 | 0.0373 | 0.56 | 6.1 | 6.25 |
| `whole_body_l1_push_run2` | HOLD | 4.8 | 0.999 | 1.857 | 0.213 / 0.930 / 0.294 | 0.0469 | 0.24 | 5.3 | 5.61 |
| `whole_body_l1_push_run2` | SLIDE | 19.1 | 1.075 | 2.766 | 0.482 / 0.926 / 0.259 | 0.0319 | 0.64 | 6.3 | 6.55 |
| `whole_body_l1_push_run2` | RELEASE | 1.9 | 0.685 | 0.901 | 0.403 / 0.539 / 0.127 | 0.0555 | 0.24 | 6.1 | 4.46 |
| `whole_body_l1_push_run3` | CONTACT | 25.6 | 0.705 | 2.201 | 0.238 / 0.647 / 0.150 | 0.0398 | 0.53 | 4.6 | 4.03 |
| `whole_body_l1_push_run3` | HOLD | 4.6 | 0.378 | 0.757 | 0.191 / 0.272 / 0.181 | 0.0246 | 0.09 | 2.0 | 2.59 |
| `whole_body_l1_push_run3` | SLIDE | 19.1 | 0.776 | 2.201 | 0.240 / 0.726 / 0.132 | 0.0407 | 0.60 | 5.2 | 4.34 |
| `whole_body_l1_push_run3` | RELEASE | 1.9 | 0.557 | 1.532 | 0.305 / 0.414 / 0.214 | 0.0571 | 0.31 | 1.9 | 3.75 |
| `whole_body_l1_push_run4` | CONTACT | 25.8 | 0.979 | 2.546 | 0.330 / 0.910 / 0.150 | 0.0430 | 0.52 | 5.6 | 5.52 |
| `whole_body_l1_push_run4` | HOLD | 4.8 | 0.828 | 2.161 | 0.156 / 0.803 / 0.129 | 0.0258 | 0.27 | 3.4 | 4.41 |
| `whole_body_l1_push_run4` | SLIDE | 19.1 | 1.035 | 2.546 | 0.357 / 0.964 / 0.126 | 0.0467 | 0.57 | 6.2 | 5.82 |
| `whole_body_l1_push_run4` | RELEASE | 1.9 | 0.719 | 1.686 | 0.379 / 0.521 / 0.319 | 0.0388 | 0.42 | 4.3 | 4.90 |
| `whole_body_l1_push_run5` | CONTACT | 25.7 | 0.968 | 2.345 | 0.313 / 0.904 / 0.151 | 0.0449 | 0.60 | 6.2 | 5.43 |
| `whole_body_l1_push_run5` | HOLD | 4.7 | 0.819 | 1.560 | 0.226 / 0.769 / 0.168 | 0.0347 | 0.26 | 3.4 | 4.56 |
| `whole_body_l1_push_run5` | SLIDE | 19.1 | 1.031 | 2.345 | 0.330 / 0.966 / 0.147 | 0.0476 | 0.67 | 6.9 | 5.76 |
| `whole_body_l1_push_run5` | RELEASE | 1.9 | 0.582 | 0.982 | 0.327 / 0.461 / 0.139 | 0.0394 | 0.41 | 3.7 | 3.77 |

A force residual scales with the stiffness a rig declares, so its DISPLACEMENT equivalent is given too: the
declared compliance applied to e_imp -- how far the claw is from where its own declared impedance puts it
under the measured force (`impedance.e_imp_mm`, `metrics.impedance.<window>.e_imp_mm_rms`).

The box and the vehicle (`metrics.box`, `metrics.window`):

| run | box pushed [mm] | box yaw change [deg] | box tilt max [deg] | box moved in the exit [mm] | vehicle tilt max [deg] | q2 max, release + exit [deg] | q3 min, release + exit [deg] |
|---|---|---|---|---|---|---|---|
| `whole_body_l1_push_run1` | 395 | -4.7 | 0.46 | 0.6 | 2.8 | 39.0 | 30.9 |
| `whole_body_l1_push_run2` | 407 | -0.1 | 0.69 | 0.2 | 2.7 | 39.5 | 31.6 |
| `whole_body_l1_push_run3` | 399 | +4.2 | 0.41 | 0.1 | 2.2 | 39.1 | 31.1 |
| `whole_body_l1_push_run4` | 405 | +1.0 | 0.39 | 0.0 | 2.2 | 39.3 | 30.4 |
| `whole_body_l1_push_run5` | 399 | -3.8 | 0.41 | 0.2 | 2.4 | 38.5 | 31.3 |
| `coordinated_admittance_push_run1` | 406 | -1.1 | 0.23 | 56.8 | 3.6 | 41.7 | 29.4 |
| `coordinated_admittance_push_run2` | 418 | -4.0 | 0.43 | 18.3 | 2.4 | 41.5 | 30.5 |
| `coordinated_admittance_push_run3` | 421 | -3.9 | 0.33 | 3.2 | 2.4 | 41.2 | 30.2 |
| `coordinated_admittance_push_run4` | 401 | +0.4 | 0.33 | 38.3 | 3.4 | 41.8 | 30.7 |
| `coordinated_admittance_push_run5` | 404 | -0.4 | 0.27 | 6.5 | 3.1 | 41.4 | 30.5 |
| `arbitrary_time_push_run1` | 395 | -1.0 | 0.23 | 14.9 | 2.7 | 39.2 | 31.8 |
| `arbitrary_time_push_run2` | 405 | -1.4 | 0.22 | 0.0 | 2.5 | 39.3 | 31.9 |
| `arbitrary_time_push_run3` | 405 | -0.0 | 0.57 | 32.5 | 3.2 | 39.2 | 31.8 |
| `arbitrary_time_push_run4` | 413 | -0.0 | 0.44 | 19.9 | 3.4 | 39.3 | 31.6 |
| `arbitrary_time_push_run5` | 410 | -1.6 | 0.48 | 26.4 | 3.5 | 39.2 | 31.6 |

The last two columns are the release window's joint margins: q2's stop is +45 deg, the planner's arm
guard aborts below q3 = 10 deg (see the attempts below).

Attempts (a run is kept only if it completed, pushed the box >= 0.36 m (90 % of the commanded 0.40 m) and kept every step on its slot):

| method | attempts | kept | why the others were not kept |
|---|---|---|---|
| Arbitrary-time [Chen 2025] | 8 | 5 | [  89.30s] push_pull: ABORTING (SAFETY GUARD: the box was dragged 50 mm sideways (> 45 mm: toward the table's edge)) -- gripper open, contact off, climbing 0.30; [  97.97s] push_pull: ABORTING (SAFETY GUARD: the box is 106 mm AHEAD OF its plan (> 100 mm: running away)) -- gripper open, contact off, climbing 0.30 m, then ; [  99.18s] push_pull: ABORTING (SAFETY GUARD: the box is 114 mm BEHIND its plan (> 100 mm: jammed or too heavy)) -- gripper open, contact off, climbing 0.30 m,  |
| Coordinated [Liu 2024] | 6 | 5 | [  89.52s] push_pull: ABORTING (SAFETY GUARD: the box is 60 mm BEHIND its plan (> 60 mm: jammed or too heavy)) -- gripper open, contact off, climbing 0.30 m, th |
| Proposed | 5 | 5 | -- |

The failed attempts (kept whole in the repository under `results/simulation_results/Isaac_simulation_data/box_pushing/failed/`):

| attempt | what stopped it | when | afterwards |
|---|---|---|---|
| `arbitrary_time_push_run1_attempt1` | planner abort (SAFETY GUARD: the box is 114 mm BEHIND its plan (> 100 mm: jammed or too heavy)) | at 99.2 s | landed upright (tilt max 17 deg, final height 0.24 m) |
| `arbitrary_time_push_run3_attempt1` | planner abort (SAFETY GUARD: the box was dragged 50 mm sideways (> 45 mm: toward the table's edge)) | at 89.3 s | landed upright (tilt max 18 deg, final height 0.24 m) |
| `arbitrary_time_push_run4_attempt1` | planner abort (SAFETY GUARD: the box is 106 mm AHEAD OF its plan (> 100 mm: running away)) | at 98.0 s | landed upright (tilt max 25 deg, final height 0.24 m) |
| `coordinated_admittance_push_run3_attempt1` | planner abort (SAFETY GUARD: the box is 60 mm BEHIND its plan (> 60 mm: jammed or too heavy)) | at 89.5 s | landed upright (tilt max 16 deg, final height 0.24 m) |

## Scene and task

- **Table:** the HOOBRO console table (1.000 x 0.150 x 0.800 m), its length along world y: x 0.600-0.750 m, y -0.5..0.5 m, top at 0.80 m.
- **Box:** the CAD box (240 x 160 x 100 mm) with its clamp bracket and 5 mm fin handle, **280 g**
  in total, friction 0.25 static / 0.25 sliding (the real box on the experiment table, spring-gauge bench test 2026-10-17), 0.69 N to slide. It starts
  on the table's centre line, its back face 0.10 m from the table's +y end, and is pushed toward -y (to 0.50 m). The sim claw cannot close below ~19 mm,
  so the fin's bare part carries an invisible 20 mm grip shim (the real gripper's foam pads close on the
  5 mm fin).
- **Task:** grasp the fin from above (go to start, ready, grasp), push the box 0.40 m along the
  table in 19.0 s (a min-snap move after a 1.5 s settle) with the arm held still and the
  claw held in the world, release, climb out, fly off the table to the landing hover, descend. The whole-body law's
  contact phase (chi) is CONTACT from the jaws' grip to the end of the push.
- **Arm pose in the push:** [0, 28, 32, 0] deg (fold 60 deg); airframe: the T650
  landing gear; the real arm's q2 range [-20, 45] deg is enforced as hard stops in the scene.
- **Safety guards (on in every attempt, every rig; none tripped in the kept runs; 4 failed attempts above):** the planner refuses a push that would leave the box
  within 3 cm of the table edge, and aborts on a box lagging > 40 mm behind its plan, a box tilt > 2 deg,
  q3 < 10 deg (static, and q3 predicted 0.2 s ahead from its rate, with credit for q2 resting on its +45 deg
  wall), a lost box feed, a task force > 5 N or a vehicle tilt > 15 deg. The arbitrary-time rig carries its
  own box-lag / force values and one guard more, the coordinated rig its own box-lag value (both below);
  every other guard is the whole-body rig's.
- **Impedance, Proposed (the whole-body law's EE task):** M_d = diag(1, 1.64, 1, 0.05),
  D_d = diag(20.2, 33.128, 20.2, 0.2903), K_d = diag(120, 196.8, 120, 0.2484)
  on [x, y, z, heading] (kg, N.s/m, N/m; kg.m^2, N.m.s/rad, N.m/rad for the heading).
- **Admittance, Coordinated [Liu 2024] (eq 27, contact phase, claw frame [along, lateral, vertical]):** M_d = diag(0.5, 0.5, 0.5),
  D_d = diag(17.3, 20, 20), K_d = diag(150, 20, 20) (kg, N.s/m, N/m); its contact phase runs
  from the jaws' stall to the release. That rig flies the planner's transits at 0.20 m/s / 0.08 m/s^2 /
  0.20 rad/s (the whole-body rig 0.30 / 0.15 / 0.30) -- the push itself is the same move on every rig
  (distance / mean speed) -- so its transits take longer (they fit the same timetable slots); from Ready on
  every rig flies the same timetable. Its OWN box-lag guard: 60 mm (2026-10-28, user decision; every
  other guard shared): its decoupled base lags the planned push, so at the shared 40 mm the box ran
  41-42 mm behind its plan at the push's peak speed and 2 of 2 attempts aborted, the box not jammed.
- **Admittance, Arbitrary-time [Chen 2025] (eq 34, JOINT space, the arm relative to the base):** H_d = diag(0.3333, 0.3333, 0.3333, 0.3333), D_d = diag(1.34, 1.34, 1.34, 1.34),
  K_d = diag(3, 3, 3, 3) (kg.m^2, N.m.s/rad, N.m/rad, as flown). At the push
  pose this declares 618 N/m along the arm (= the push) and 222 N/m across it (J^-T K_d J^-1, `meta.impedance_gains`);
  its residual is scored against that task-space image exactly as the other rigs' (the claw's world error
  chains, the true contact wrench). Its OWN safety guards (the other rigs' untouched): box lag 100 mm, contact force 3.5 N on its own estimate, and one guard the
  other rigs do not carry: the box dragged sideways more than 45 mm (`push_pull_guard_box_cross`) --
  its base-relative arm drags the clamped box with the base's sideways swings (up to 35 mm in completed
  pushes; twice past the table's side edge in tuning, the whole-body and coordinated rigs 2-5 mm).
  From Ready on it flies the same timetable as the other rigs.
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
sim_feedback_mocap_rate_hz: 120.0
sim_feedback_pos_noise_m: 0.0005
sim_feedback_vel_noise_mps: 0.022
sim_feedback_noise_seed: 0
```

- **The push-and-pull planner block as flown** (`configs/`):

```
push_pull_ee_offset: [0.0, 0.0, 0.183]
push_pull_yaw_from_box_deg: -90.0
push_pull_push_pose_deg: [0.0, 28.0, 32.0, 0.0]
push_pull_approach_dz: 0.15
push_pull_approach_back: 0.12
push_pull_exit_dz: 0.25
push_pull_contact_at_ready: false
push_pull_contact_off_at_push_end: true
push_pull_push_distance: 0.40
push_pull_push_mean_speed: 0.021
push_pull_push_min_time_s: 0.0
push_pull_push_distance_ok: [0.10, 0.50]
push_pull_push_speed_ok: [0.008, 0.030]
push_pull_push_settle_s: 1.5
push_pull_push_ramp_s: 0.0
push_pull_exit_time_s: 3.0
push_pull_unload_time_s: 0.0
push_pull_world_anchor_push: true
push_pull_world_anchor_ready: false
push_pull_world_anchor_on_handle: true
push_pull_handle_settle_tol: 0.025
push_pull_handle_settle_axis_tol: 0.015
push_pull_handle_settle_max_speed: 0.030
push_pull_handle_settle_dwell_s: 0.3
push_pull_ready_max_speed: 0.030
push_pull_ready_settle_s: 0.5
push_pull_descent_trim: false
push_pull_land: [-1.0, 0.0, 0.3, -180.0]
push_pull_land_start: [-1.0, 0.0, 1.3, -180.0]
push_pull_start_mark: [-1.0, 0.0, 1.0, 0.0]
push_pull_table: [0.600, 0.750, -0.5, 0.5, 0.80]
push_pull_table_clearance: 0.05
push_pull_table_from_box: true
push_pull_table_size: [1.0, 0.15]
push_pull_table_box_gap: 0.10
push_pull_box_half_height: 0.05
push_pull_start_middle_tol: 0.30
push_pull_go_to_start_bypass: true
push_pull_bypass_clearance: 0.70
push_pull_handle_top_above_grasp: 0.025
push_pull_gear_depth: 0.2425
push_pull_fence_max_z: 2.2
push_pull_fence_min_z: 0.25
push_pull_guard_tilt_deg: 15.0
push_pull_guard_force_n: 5.0
push_pull_abort_from_measured: true
push_pull_abort_back: 0.0
push_pull_abort_release_wait_s: 1.0
push_pull_abort_unbuckle: true
push_pull_box_half_length: 0.12
push_pull_box_edge_margin: 0.03
push_pull_guard_box_lag: 0.040
push_pull_guard_box_tilt_deg: 2.0
push_pull_guard_box_persist_s: 0.20
push_pull_guard_box_stale_s: 0.50
push_pull_guard_q3_min_deg: 10.0
push_pull_guard_q3_lookahead_s: 0.2
push_pull_guard_q3_rate_min_deg_s: 5.0
push_pull_guard_q3_rate_persist_s: 0.05
push_pull_guard_q3_rate_filter_s: 0.02
push_pull_guard_q2_max_deg: 45.0
push_pull_box_gate_jump_m: 0.020
push_pull_box_gate_jump_deg: 10.0
push_pull_mocap_guard_topic: "state_estimator/mocap_guard"
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
| d<sub>box</sub> | the box's travel along the table during the push [mm] (0.40 m commanded) | `box_pushed_mm` | `metrics.box.pushed_mm`, `raw.box` |

- **Reference:** the planner's dynamically compatible whole-body reference. The platform reference
  x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>) is r<sub>UAV</sub><sup>ref</sup>.
  During the push the EE reference r<sub>e,d</sub> is the claw's planned path along the table (world-held).
- **max / rms:** over the window; the table's other columns are RMS errors.
- **e<sub>imp</sub>:** every term through one zero-phase 5 Hz low-pass; e<sub>y</sub> from the TRUE claw point
  (ground truth) minus the streamed reference; e<sub>v</sub>, ė<sub>v</sub> its derivatives against the streamed
  reference derivatives; F<sub>ext</sub> from the box's PhysX contact report (sign calibrated on the table's
  normal force). A residual of 0 means the claw moved exactly as the designed impedance under the measured force.
