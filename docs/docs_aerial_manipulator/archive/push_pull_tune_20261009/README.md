# Push or pull, then the push tune (2026-10-09)

User request: the physical-interaction task for the paper. (1) Metrics = the pick-and-place
set (UAV deviation, tracking errors) plus the IMPEDANCE RESIDUAL
`e_imp = F_ext - (M_d edot_v + D_d e_v + K_d e_y)`; (2) a quick test to pick the simpler and
safer of box PUSHING and box PULLING -- only that one goes into the final simulation and the
experiment; (3) a systematic gain tune for the chosen task; (4) the results campaign ->
`results/simulation_results/Isaac_simulation_data/box_pushing/` + the MATLAB export.

Scene and box: the user's CAD box (`Box_Push.stl` = `Box_Assembly.stl`, byte-identical,
checked: the real experiment box), 400 g, mu 0.29, 5 mm fin + the sim's 20 mm grip shim, the
T650 gear; everything else as `../push_pull_cad_box_20261007/`. Plant: the mirror (config A).
Raw mocap feedback. Headless, RTF 1.

Tools (this folder): `tools/fly.sh <tag> [key=val ...]` (one flight on the driver's
TIMETABLE, `PULL=1` for a pull), `tools/summarize.py`, `tools/chain_ab.sh`,
`tools/chain_tune.sh`. Driver changes (`../push_pull_20261003/tools/pl_mission.py`):
`--timetable` (slots 11 / 2.5 / 7 / 27 / 3 / 8.5 s from the previous step's start; the
jaws' stall is never delayed) and the reference stream now carries `x_cd_ddot` + `b1_de`
(cols 13..18). Results pipeline: `results/utils/run_box_pushing_campaign.sh` +
`build_box_pushing.py`.

## 1. Metrics

- `e_imp` per task channel (x, y, z N; heading N.m), the true contact wrench (the box's
  PhysX report) against the claw truth and the streamed reference, one zero-phase 5 Hz
  low-pass on every term -- `pl_metrics.py`, over CONTACT / HOLD / SLIDE / RELEASE.
- `eps_UAV`, `rho_UAV` (L = 0.370 m) and the RMSE set (platform xyz / attitude, EE xyz /
  heading, joints) on the pick-and-place definitions, over the whole mission (seven
  contiguous phases: go to start, ready, grasp, push, release, exit, go to land).
- Safety: completion, tilt max, joint margins (q2 <= 45, 0 < q3 <= 50), q3 at the release,
  the box's movement during the exit, the box pushed.

## 2. Push vs pull (A/B, the shipped push yaml, 3 + 3) -- PUSH

`tools/chain_ab.sh`, the shipped 10-07 push yaml (60 deg pose, omega_c_t 1.0, 24 s), the box
at y = +0.25 pushed to -0.25 or at -0.25 pulled to +0.25. All six completed. Scored with
`summarize.py` / `pl_metrics.py` (e_imp = the norm's rms over the window, N):

| run | box (mm) | e_imp slide | e_imp contact | e_imp release (rms / peak) | rho slide / exit | tilt max | q2 range | q3 range | q3 at release | box moved in exit |
|---|---|---|---|---|---|---|---|---|---|---|
| push 1 | 497 | 0.66 | 0.85 | 1.34 / 4.8 | 0.059 / 0.070 | 2.6 | 10..36 | 22..50 | 50 | 0 |
| push 2 | 502 | 1.22 | 1.21 | 1.29 / 3.1 | 0.070 / 0.065 | 3.2 | 11..37 | 21..50 | 41 | 0 |
| push 3 | 497 | 0.71 | 0.85 | 1.31 / 5.0 | 0.053 / 0.098 | 3.3 | 19..35 | 23..44 | 48 | 1 |
| pull 1 | 496 | 0.76 | 1.05 | 1.98 / 6.5 | 0.065 / 0.154 | 5.1 | 16..38 | 17..39 | 50 | 6 |
| pull 2 | 492 | 1.29 | 2.12 | **4.79 / 15.8** | 0.077 / 0.178 | 7.2 | 15..42 | 15..39 | 50 | 3 |
| pull 3 | 495 | 1.15 | 1.36 | 2.32 / 6.5 | 0.081 / 0.170 | 6.6 | 15..**45.0** | 9..41 | 50 | 5 |

The push is the simpler and safer task on every axis that matters:

- **Impedance**: over the whole contact the push's residual is 0.85-1.21 N against the pull's
  1.05-2.12; the pull's RELEASE is where it breaks (2-4.8 N rms, peaks to 15.8 N) -- a pull ends
  with the drone still pulling and pressing the box down (10-07 section 4.4), so letting go is a
  jolt; a push ends nearly unloaded.
- **Platform**: tilt 2.6-3.3 deg pushing vs 5.1-7.2 pulling; the platform deviation after the
  release (EXIT) 24-36 mm vs 57-66 mm (rho 0.15-0.18).
- **Joint margins**: pulling, q2 touched its 45 deg stop (pull 3) and q3 went down to 9 deg;
  pushing stays >= 8 deg from both.
- **The box**: the pull's exit drags it back 3-6 mm every time (the open claw catching the fin
  from the loaded release); the push 0-1 mm.
- The pull's pulling force also has a large vertical part (F angle 38-48 deg above the table in
  two of three runs), i.e. the claw presses the box into the table while dragging it.

Both tasks still fold the arm onto q3's +50 stop at the release in most runs (the stored-load
problem of 10-07, untouched here) -- on the push the exit then moves the box <= 1 mm.

Decision: **box PUSHING** for the simulation and the experiment. The tune below is for the push.

## 3. The push tune

### 3.1 The plan

What can move a contact task's numbers on this law, one change per flight against the shipped
push yaml (baseline = the three A/B pushes), `tools/chain_tune.sh`, scored with
`tools/tune_table.py` (`screening.csv`):

| group | shipped | candidates | why |
|---|---|---|---|
| A. EE impedance K_y / D_y (x, y, z) | 211.9 / 26.82 (zeta 0.92) | 120 / 20.2, 300 / 31.9 (same zeta) | the impedance the law renders -- e_imp is its fidelity, and the task's stiffness against the base's ~1 Hz swing |
| B. translational observer omega_c_t | 1.0 | 0.5, 1.5 | how fast the stuck box's reaction is learned / unlearned (the 10-07 lesson: 2.927 fails) |
| C. rotational observer omega_c_r | 0.7428 | 0.5, 1.0 | the box's moment on the airframe |
| D. contact reading omega_x | 2.0 | 1.0, 4.0 | the filter on the attributed contact force the impedance renders |
| E. u3 feed-forward omega_x_t/r/q | 0.2072 | 0.1, 0.4 | the internal-disturbance feed-forward into the arm |
| F. attitude k_R / k_w | 1.6 / 1.2 | 1.3 / 1.0, 2.134 / 1.567 (H1b) | the airframe's lean under the contact force |
| G. position k_x / k_v | 50.03 / 12.58 | 32 / 10, 64 / 14 | the base's hold against the reaction |
| H. push time | 24 s | 18, 12 | speed (the pull failed on speed at 12 s) |

### 3.2 Screening (16 flights)

e_imp = the norm's rms (N) over SLIDE / CONTACT / HOLD / RELEASE (+ the release peak), F = the
true contact force in the slide (rms), rho = eps_UAV max / L over the slide / the contact /
the exit, joint ranges in the push, q3 at the release, the box's movement in the exit (mm):

| run | ok | slide | contact | hold | release / pk | F | e_y mm | rho slide / contact / exit | tilt | q2 | q3 | q3 rel | exit box |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline ab_push_1 | y | 0.66 | 0.85 | 1.12 | 1.34 / 4.8 | 1.14 | 5.7 | .059 / .071 / .070 | 2.6 | 10..36 | 22..50 | 50 | 0 |
| baseline ab_push_2 | y | 1.22 | 1.21 | 1.04 | 1.29 / 3.1 | 0.91 | 6.9 | .070 / .070 / .065 | 3.2 | 11..37 | 21..50 | 41 | 0 |
| baseline ab_push_3 | y | 0.71 | 0.85 | 0.99 | 1.31 / 5.0 | 1.16 | 6.1 | .053 / .066 / .098 | 3.3 | 19..35 | 23..44 | 48 | 1 |
| **A K_y 120 / D_y 20.2** | y | **0.53** | **0.52** | **0.53** | **0.49 / 0.8** | 1.05 | 10.5 | .060 / .060 / **.038** | 3.0 | 15..32 | 26..48 | **43** | 0 |
| A K_y 300 / D_y 31.9 | **N** | | | | | | | | | | | | left DIRECT in the grasp (watchdog) |
| B omega_c_t 0.5 | y | 0.85 | 1.00 | 1.09 | 1.54 / 3.1 | 1.32 | 6.1 | .096 / .103 / .172 | 2.8 | 19..40 | 19..42 | 50 | **21** |
| B omega_c_t 1.5 | y | 1.00 | 1.10 | 0.93 | 1.61 / 4.4 | 1.02 | 6.3 | .058 / .074 / .054 | 3.0 | 10..37 | 21..50 | 45 | 0 |
| C omega_c_r 0.5 | y | 0.79 | 0.89 | 1.17 | 1.04 / 2.8 | 1.31 | 6.2 | .090 / .090 / .066 | 2.8 | 16..44 | 12..48 | 49 | 1 |
| C omega_c_r 1.0 | y | 0.91 | 1.04 | 0.95 | 1.60 / 4.7 | 1.33 | 7.0 | .052 / .072 / .080 | 3.1 | 16..41 | 16..45 | 50 | 5 |
| D omega_x 1.0 | y | 1.36 | **2.15** | 1.06 | **4.78 / 15.9** | 1.74 | 7.9 | .082 / .130 / .180 | 3.0 | 16..43 | 14..46 | 50 | 1 |
| D omega_x 4.0 | y | 1.27 | **2.43** | 1.18 | **5.77 / 20.1** | 1.87 | 7.9 | .091 / .143 / .174 | 3.0 | 18..44 | 14..44 | 50 | 0 |
| **E omega_x_t/r/q 0.1** | y | 0.73 | **0.77** | 0.74 | **0.96 / 2.3** | 1.14 | 6.2 | .073 / .073 / .062 | 3.7 | 16..36 | 22..45 | **44** | 0 |
| E omega_x_t/r/q 0.4 | y | 0.86 | 0.90 | 1.02 | 1.03 / 3.2 | 1.04 | 5.7 | .076 / .076 / .064 | 3.8 | 10..40 | 19..50 | 48 | 1 |
| F k_R / k_w 1.3 / 1.0 | y | 0.82 | 0.95 | 1.00 | 1.45 / 3.8 | 1.39 | 6.3 | **.108** / .108 / .091 | **5.4** | 13..**45** | **7**..47 | 50 | 0 |
| F k_R / k_w 2.134 / 1.567 | **N** | | | | | | | | | | | | left DIRECT in the hover before the mission |
| G k_x / k_v 32 / 10 | **N** | | | | | | | | | | | | force guard 24 N in the push (a stick-slip lurch) |
| G k_x / k_v 64 / 14 | **N** | | | | | | | | | | | | left DIRECT in the hover before the mission |
| H push 18 s | y | 0.77 | 1.06 | 0.93 | 1.47 / 5.7 | 1.63 | 6.5 | .068 / .089 / .140 | 2.7 | 19..42 | 14..42 | 50 | 3 |
| H push 12 s | y | 0.60 | 1.00 | 0.72 | 1.28 / 6.5 | 1.21 | 6.3 | .050 / .081 / .117 | 2.9 | 18..35 | 24..43 | 50 | 1 |

What it says:

- **The EE impedance is the lever.** Softer (K_y 120, D_y 20.2) halves the residual on EVERY
  window (slide 0.53 vs 0.66-1.22, contact 0.52 vs 0.85-1.21, release 0.49 / peak 0.8 vs
  1.3 / 3-5), keeps the arm off q3's +50 stop at the release (43 deg) and gives the smallest
  exit deviation (rho 0.038). The price is the deflection under the 1 N force: |e_y| 10.5 mm
  rms against 6 (F / K). Stiffer (300) is unstable in the grasp -- every attempt to stiffen the
  task on this plant has been (10-07's `wb_dy` 40, the 09-27 K_y 270).
- **The u3 feed-forward filters at 0.1** improve the contact / release windows (0.77 / 0.96 N,
  release peak 2.3) and keep the arm off the stop; 0.4 does nothing.
- **The contact reading must stay at 2.0**: both 1.0 and 4.0 double the contact residual and
  make the release a 16-20 N jolt.
- The observers (B, C), the attitude pair and the position pair are at their limits already:
  B/C are inside the baseline's scatter or cost margin (omega_c_t 0.5: the exit drags the box
  21 mm; omega_c_r 0.5: q2 to 44); softer attitude leans 5.4 deg and puts q2 on its stop and
  q3 at 7; H1b's stiffer attitude and BOTH position pairs are unstable (two hover aborts and a
  force-guard trip).
- **Speed**: 12 s and 18 s pushes complete, but the release peak grows (5.7-6.5 N) and the exit
  deviation with it (rho 0.12-0.14 vs 0.07-0.10); 24 s stays.
- The baseline's own scatter is wide (slide 0.66-1.22): a single run proves nothing, which is
  why the winners are repeat-tested below.

### 3.3 Confirmation (8 flights, `tools/chain_confirm.sh`)

The winners repeat-tested (the 2026-08-10 lesson: one run proves nothing on this rig), alone and
combined, plus a softer bound on K_y. Same columns as 3.2:

| run | slide | contact | hold | release / pk | F | e_y mm | rho slide / contact / exit | tilt | q2 | q3 | q3 rel | exit box |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| K_y 120 #1 (t_ky120) | 0.53 | 0.52 | 0.53 | 0.49 / 0.8 | 1.05 | 10.5 | .060 / .060 / .038 | 3.0 | 15..32 | 26..48 | 43 | 0 |
| K_y 120 #2 | 0.82 | 0.79 | 0.47 | 0.87 / 1.8 | 1.00 | 11.4 | .053 / .053 / .041 | 3.2 | 14..29 | 29..50 | 47 | 0 |
| K_y 120 #3 | 0.66 | 0.69 | 0.71 | 0.82 / 1.6 | 0.93 | 9.0 | .054 / .063 / .051 | 2.6 | 14..38 | 20..48 | 44 | 0 |
| **K_y 120 + FF 0.1 #1** | 0.63 | 0.62 | 0.68 | 0.55 / 1.3 | 0.97 | 10.0 | .068 / .068 / .042 | 3.5 | 13..31 | 28..48 | 44 | 0 |
| **K_y 120 + FF 0.1 #2** | 0.57 | 0.57 | 0.62 | 0.51 / 1.5 | 0.97 | 9.7 | .055 / .055 / .037 | 3.1 | 15..32 | 28..46 | 44 | 0 |
| **K_y 120 + FF 0.1 #3** | 0.81 | 0.77 | 0.43 | 0.78 / 1.2 | 1.01 | 11.0 | .064 / .064 / .050 | 2.7 | 14..33 | 30..49 | 42 | 0 |
| K_y 80 / D_y 16.5 #1 | 0.48 | 0.50 | 0.48 | 0.62 / 1.1 | 1.04 | 13.7 | .062 / .062 / .057 | 3.1 | 14..32 | 25..**50** | 48 | 0 |
| K_y 80 / D_y 16.5 #2 | 0.68 | 0.67 | 0.48 | 0.73 / 1.4 | 0.96 | 15.5 | .065 / .065 / .061 | 2.8 | 12..29 | 28..**50** | 48 | 0 |
| FF 0.1 alone #2 | 1.32 | 1.34 | 1.23 | 1.52 / 4.5 | 0.94 | 7.3 | .067 / .075 / .072 | 3.7 | 10..35 | 24..50 | 41 | 0 |

- All eight completed, 488-502 mm pushed, 0 mm moved in the exit, tilt <= 3.5 deg.
- **K_y 120 alone and K_y 120 + FF 0.1 are indistinguishable on the slide** (0.53-0.82 vs
  0.57-0.81 N rms, means 0.67 / 0.67) and both keep the release at 0.5-0.9 N (baseline 1.3)
  with peaks of 0.8-1.8 N (baseline 3-5) and the arm off q3's stop at the release (42-47 vs
  41-50). The combination is the more CONSISTENT set: release peaks 1.2-1.5 N (vs 0.8-1.8),
  q3 at the release 42-44 (vs 43-47), and q3 never reached 50 in the push (48.3 / 45.9 / 49.4
  vs one 50.0).
- **The feed-forward filters alone do NOT reproduce their screening gain**: the repeat is a
  baseline run (slide 1.32, release 1.52 / peak 4.5). So the improvement is K_y's; FF 0.1 is
  kept in the shipped set only because the combination was the most consistent 3/3 and it
  costs nothing measurable.
- **K_y 80** buys a residual inside K_y 120's scatter (0.48 / 0.68) for 4-5 mm more claw
  deflection (13.7-15.5 mm rms) and q3 ON its +50 stop during the push in both runs: softer
  still is not better.

**SHIPPED (make_push_pull_yaml.py OVERRIDES): `wb_ky_x/y/z` 120, `wb_dy_x/y/z` 20.2,
`wb_l1_omega_x_t/r/q` 0.1**, everything else as the 10-07 push yaml (k_R/k_w 1.6/1.2,
omega_c_t 1.0, omega_x 2.0, the 60 deg pose, 24 s, the CoM-anchored descent with the claw
world-held from the handle). The hardware files are NOT touched (no hardware push-and-pull
block exists yet).

## 4. The results campaign (`results/utils/run_box_pushing_campaign.sh 3`)

Three flights on the shipped push yaml (`fly.sh final_r<k>_a1`, the driver's timetable),
3/3 completed at the first attempt, the box pushed 498 / 492 / 497 mm, 0.0-0.5 mm moved in
the exit, tilt <= 3.1 deg, every step on its slot (phase starts agree across runs to 0.09 s --
the grasp's own length). Copied to `results/simulation_results/Isaac_simulation_data/box_pushing/` and built by
`results/utils/build_box_pushing.py` into `matlab_simulation_data/box_pushing/` (README there).

| run | e_imp slide rms / max | contact | hold | release rms / max | e_y slide mm | eps max / rms mm | rho max / rms | platform | EE | EE heading | q1..q4 deg |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.511 / 1.28 | 0.531 | 0.576 | 0.593 / 1.43 | 9.6 | 36.2 / 16.5 | 0.098 / 0.045 | 16.5 | 17.8 | 2.07 | 1.2 / 3.3 / 4.3 / 3.6 |
| 2 | 0.920 / 1.70 | 0.946 | 0.562 | 1.297 / 2.94 | 12.0 | 37.1 / 18.7 | 0.100 / 0.051 | 18.7 | 18.1 | 2.10 | 1.2 / 4.3 / 5.0 / 2.8 |
| 3 | 0.547 / 1.19 | 0.589 | 0.591 | 0.776 / 1.68 | 9.8 | 37.1 / 17.3 | 0.100 / 0.047 | 17.3 | 18.6 | 2.11 | 1.2 / 3.5 / 4.4 / 4.3 |
| mean | 0.659 / 1.39 | 0.689 | 0.576 | 0.889 / 2.02 | 10.5 | 36.8 / 17.5 | 0.100 / 0.047 | 17.5 | 18.2 | 2.09 | 1.2 / 3.7 / 4.6 / 3.6 |

- The platform deviation's maximum (36-37 mm) is NOT in the contact: it is the 270 deg
  clockwise turn of the leg to the landing hover; in the push it peaks at 23-37 mm (ρ ≤ 0.10).
- Run 2 is the campaign's scatter case (slide 0.92, release 1.30 / 2.9 N) -- the same spread the
  tune saw (K_y 120: 0.53 / 0.82 / 0.66), with the arm still off the stop and the box untouched.
- The EE RMSE over the mission (18 mm) is dominated by the free-flight legs (the ~0.1 Hz airframe
  wander of the go-to-start / go-to-land transits), not the contact: in the slide the claw's task
  error is 9.6-12 mm rms, which is the designed deflection F / K_y under the 1.0-1.2 N force.
