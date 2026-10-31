# Arbitrary-convergence-time control (Chen et al., IEEE TIE 2025) as a box-pushing comparison rig

2026-10-24, user request: implement Chen, Liu, Cai, Shang, Chang, Chao, Wang, Shen,
"Arbitrary Convergence Time Control of Unmanned Aerial Manipulator for Active
Interaction and Dynamic Tracking", IEEE Trans. Industrial Electronics 72(12), 2025,
13693-13702 (`docs/docs_aerial_manipulator/references/Arbitrary_Convergence_Time_...pdf`)
for the box-pushing Isaac simulation, **independent of the whole-body controller**
(no shared code: own fork, own namespace, own yaml keys).

## 1. What was built

| piece | where |
|---|---|
| Python reference (source of truth, self-test) | `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_controller/arbitrary_time_control.py` |
| C++ law (the four modules + arm model + stream conversion) | fsc_autopilot_ros2 `fsc_autopilot_ros2_node/single_aerial_manipulator_arbitrary_time_direct_actuation/client_lib/src/arbitrary_time_law.cpp` |
| flight node (SAFETY baseline + DIRECT law + arm torque I/O) | same fork, `single_aerial_manipulator_arbitrary_time_direct_actuation_client.cpp`, executable `autopilot_arbitrary_time_direct_actuation_node`, namespace `fsc_autopilot_ros2/arbitrary_time_direct_actuation` |
| parity fixture + gtest (1e-8) | `application/robotic_arm/utils/generate_arbitrary_time_truth.py` -> `client_lib/tests/data/arbitrary_time_truth_t650.json`; `fsc_autopilot_lib/single_vehicle_baseline/tests/src/test_arbitrary_time_parity.cpp` (`ArbitraryTimeParityTest`, 6 tests) |
| yaml (GENERATED) | `tools/make_arbitrary_time_yaml.py` -> fsc_autopilot_ros2 `config/params_single_aerial_manipulator_arbitrary_time_direct_actuation_t650_sim_box_pushing.yaml` |
| arm config (one line changed) | fsc_open_manipulator `open_manipulator_x_isaac_bridge/config/torque_controller_isaac_aerial_atc.yaml` (arm_planner follows this rig's mode topic) |
| stacks | fsc_autopilot_ros2 `scripts/isaacsim/start_arbitrary_time_direct_actuation_t650_aerial_manipulator_stack.sh`; Pegasus `scripts/indoor_sim/start_t650_aerial_manipulator_arbitrary_time_direct_actuation_sitl.sh` + box wrapper `start_t650_aerial_manipulator_arbitrary_time_box_pushing_sitl.sh` |
| one-flight harness | `tools/run_pl_atc.sh <tag>` (after `scripts/kill_stale_sim_processes.sh -y` in a separate call) |
| offline bench | `tools/bench_atc.py` (the Python law on a rigid body with the rotor lag, feedback delay, mocap noise, the mirror plant's disturbances) |

Shared with the other rigs, by design: the plant (section 1 of the yaml, byte copy of
the whole-body box-pushing yaml's), the SAFETY loop + UDE, the guards, the wrench
allocator (the geometric+L1 fork's source, as the coordinated-admittance rig does),
the trajectory planner (same `fsc_trajectory_planner`, same box-pushing block,
pointed at this rig on its command line) and the arm stack (torque mode).

## 2. The paper, and where it leaves a choice

(Full list in the Python file's header; the node prints it as the cyan LAW CHECK line.)

- **ACTNDO (eqs 6-11)** on the arm's own fixed-base dynamics: stage 1 is a
  generalized-momentum observer whose `tau1_hat + w1` reconstructs tau_ext once its
  residual is zero; stage 2 (`tau_hat = rho + k H`) filters it. Implemented with the
  sign of `w_o2` the proof needs (printed, it makes `V_o2'` positive).
- **TSKFE (eqs 19-24, 29)**: the paper's 5 x 5 triangles, k-FRI with k = 3, eq 29
  adaptation. A rule matches when it fires with `W > l_max` (strictly -- with l_max = 0
  the non-strict test matches everything and no rule would ever be generated).
- **Quadrotor law (eqs 25-28) + decoupling module (eq 3)** and **arm admittance + law
  (eqs 34-38)**: per-channel backstepping with the windowed gains.
- **THE ONE STRUCTURAL ADDITION: the time-to-go floor.** The paper's gains grow as
  `1/(t_e - t)` and are zero after the window. Neither runs: the first is unbounded at
  `t_e`, the second leaves each controller with the single term `M r1` -- an undamped
  unit oscillator. `t_go = max(t_e - t, floor)`: the paper's law until the floor, then the
  same law as a constant-gain backstepping, small-signal `r1'' + (a+b) r1' + (1+ab) r1 = 0`.
- eta per channel (position vs attitude bandwidths a factor ~3 apart on this plant); the
  observer's windows restart at DIRECT entry and at each interaction edge (the planner's
  contact flag = the paper's `t_s`), the controllers' at DIRECT entry only;
  `Phi_d' = 0`; world-frame position rows; `J_m^-T` as damped least squares for the
  4-joint arm; the arm's gravity in the measured base frame.

## 3. Results (the final configuration, 2026-10-25)

Configuration: the tuned admittance (H/D/K 1/3, 1.34, 3 -- section 5) and this rig's own guards (box lag
100 mm, force 3.5 N on its estimate, the box dragged sideways 45 mm; box tilt / q3 / vehicle tilt
shared), the shared timetable, scene and plant. Campaign: `METHOD=arbitrary_time MAX_ATTEMPTS=4
TAG_PREFIX=final results/utils/run_box_pushing_campaign.sh 3` (driven by `tools/final_sequence.sh`).

**Completion: 3 of 5 attempts** (boxes 497 / 520 / 533 mm). Both failures are the new sideways guard
(52 mm, 57 mm), each leaving the box on the table and upright, its centre of mass 32-33 mm inside the
side edge at rest (closest 17-20 mm while it moved); both vehicles aborted, flew off the table and
landed.

Task-space scoring, mean +- sample std over the completed runs (`build_box_pushing.py`; methods aligned
at Ready, the whole 99.8 s mission):

| | Proposed (3) | Coordinated [Liu 2024] (2) | Arbitrary-time [Chen 2025] (3) |
|---|---|---|---|
| e_imp slide rms | 0.68 +- 0.10 N | 2.19 +- 0.05 N | 14.6 +- 4.6 N |
| ... as displacement (K_t^-1 e_imp) | 5.7 +- 0.8 mm | 28.1 +- 1.1 mm | 23.3 +- 6.6 mm |
| e_imp slide max | 1.7 +- 0.6 N | 4.8 +- 0.1 N | 47.3 +- 6.2 N |
| declared stiffness along the push | 120 N/m | 150 N/m | 686 N/m (task image of K_d 3) |
| claw error in the slide (e_y rms) | 10.6 +- 0.2 mm | 25.6 +- 0.4 mm | 20.2 +- 5.5 mm |
| eps_UAV max / rms | 44.7 +- 9.0 / 15.8 +- 0.4 mm | 114.3 +- 11.2 / 30.1 +- 0.2 mm | 58.5 +- 14.8 / 14.2 +- 3.3 mm |
| EE position RMSE (mission) | 17.1 +- 0.5 mm | 32.9 +- 0.7 mm | 15.2 +- 3.5 mm |
| box pushed / moved in the exit | 493 +- 4 / 0.8 mm | 508 +- 4 / 59 mm | 517 +- 18 / 7.0 mm |
| completion (attempts) | 3 of 3 (2026-10-17) | 2 of 6 (2026-10-17) | 3 of 5 |

Reading it: over the whole mission (mostly free flight) this rig tracks at the proposed rig's level
(eps rms 14 vs 16 mm, EE 15 vs 17 mm). In contact it does not render its declared impedance: the box
stick-slips ahead of and behind its plan on the soft base (section 5), the claw's world error in the
slide is 2x the proposed rig's, and that error, through the stiff declared impedance, is a 21x larger
force residual -- 4x as a displacement. The run-to-run spread is large (std 30 % of the mean): the
stick-slip cycle is not repeatable. The box overruns the 500 mm push by up to 33 mm.

History (all scored the same way now, data kept in this folder):

- `campaign_k9_20261024/` -- the paper's admittance (K_d 9), the shared 40 mm / 5 N guards: 1 of 8.
- `campaign_k3_pre_cross_20261025/` -- the tuned admittance and guards, before the sideways guard
  existed: 3 of 4; the failure dragged the box over the side edge and it fell off the table (section 5),
  which is why the guard was added and the campaign re-flown.

## 4. Flown: what the paper needed on this plant

Six development flights (`runs/atc_0*`), headless Isaac at RTF 1, raw mocap, the mirror plant,
the 2026-10-17 bench box (500 g, mu 0.25, 0.50 m in 23.8 s), the box guards on. Scored with
`tools/atc_score.py` (per 5 s: errors, torques, dominant frequencies) and `tools/push_timeline.py`
(the push second by second: the law's force terms, the observer against the true contact force,
the arm's admittance offset, the box against its plan).

| flight | change (cumulative) | outcome |
|---|---|---|
| atc_01 | the paper's values | left DIRECT in Ready: a 3 Hz joint-1 / yaw oscillation |
| atc_02 | ACTNDO off (diagnostic) | completed Ready; box-lag guard 44 mm -- without the J_E feed-forward the base cannot push |
| atc_03 | ACTNDO on, `obs_k` 8.2 -> 4.0, arm window / floor 1.5 / 0.6 -> 2.0 / 1.0 s | 3 Hz mode gone; j2/j3 limit cycle at 0.8 Hz; box-lag guard 45 mm |
| atc_04 | the arm stack's friction feed-forward OFF | 0.8 Hz limit cycle gone; box-lag guard 42 mm |
| atc_05 | `quad_ff_gain` 1.0 -> 0.5 (diagnostic) | box-lag guard 46 mm |
| atc_06 | ff back to 1.0, **contact re-seat ON** | **completed**, box pushed 509 mm, lag <= 20 mm |

Each change, and why it is not a retune of the paper:

- **The arm's torque path is ~36 ms long** (the node -> the arm stack's pass-through -> Isaac's
  effort command; measured on the debug stream). The paper's observer gain `k` = 8.2 and the arm
  window 1.5 s / floor 0.6 s close the arm loop too fast for it: a 3 Hz joint-1 mode that the yaw
  channel picks up through the arm reaction (atc_01). `obs_k` 4.0 and window / floor 2.0 / 1.0 s
  (offline: `tools/bench_arm.py`) keep the paper's structure and halve the arm loop's bandwidth.
- **Arm-side friction feed-forward OFF.** The arm stack adds a measured-velocity friction relay to
  any streamed torque when the plant carries gearbox friction (the whole-body rigs' arrangement).
  The paper's ACTNDO already estimates the joint friction as part of `tau_ext` -- the relay on top is
  a second, discontinuous compensation, and j2/j3 limit-cycled at 0.8 Hz with it (atc_03). Now the
  launcher's default for this rig; `ATC_ARM_FRICTION_FF=true` turns it back on.
- **Bumpless contact transfer, addition (q)** (`atc_contact_reseat`, default false = the paper;
  true in the generated yaml). At the jaws' stall the base hovers a few mm off its reference
  (normal hover wander, +-8 mm in these flights). Returning to it through the stiff joint
  admittance (2.1 kN/m along the arm at the push pose) pulls the clamped box back: 19 mm in atc_05,
  before the push even starts, half of the box-lag guard's 40 mm. `tools/bench_push.py` (the push
  axis: base + this law's position row + the arm as a spring + the box on Coulomb friction)
  ranks it: zero preload keeps the lag at 30 mm (no trip) where a 1 N preload trips at 52-59 mm,
  whatever the feed-forward gain or a softer admittance (softer makes it WORSE -- the base pushes
  through a softer spring). The node therefore shifts the stream's base reference by the base's
  tracking error at the interaction's start edge (the planner's contact flag, sent at the Push press
  = the stall, the same timing the coordinated rig gets), holds it through the contact, and blends it
  out (minimum jerk, 1 s) after the contact ends. The paper does not address a base error at contact
  onset; the whole-body law's free-axis anchor absorbs the same error by construction.
- **What still fails: the J_E feed-forward against a stick-slip box.** The paper compensates the
  estimated contact wrench on the base (`-J_E tau_hat`, eq 26/28). While the clamped box is held by
  static friction the force on the claw is whatever the arm applies, so the feed-forward cancels the
  box's restoring pull on the base: the base drifts on its own soft position loop and drags the box
  (results attempt r1_a1, +3.5..+6.5 s into the push: base 20 mm backward, box 14 mm back). At the
  break-away the ACTNDO estimate lags the true force, and for ~1 s the feed-forward keeps pushing the
  base forward with 1.4 N against a box that now slides freely: base 32 mm ahead of its reference, box
  44 mm ahead of its plan, the box-lag guard aborts ("running away"). atc_04 showed the same coupling
  ending as a lag. The paper's experiments contact a rigid surface, not a friction-held object; a
  feed-forward gain < 1 trades it for the base's own lag (atc_05). The 2026-10-25 tuning (section 5)
  softens the admittance, which keeps the feed-forward steadier, and gives the rig its own guards; the
  stick-slip cycle itself remains (the base's position stiffness is the floored law's, ~22 N/m).
- **The force the feed-forward carries is ~40 % of the true push force.** atc_06, steady slide
  (+8..+20 s), model frame: the true claw force along the push -1.52 N; the node's estimate -0.58 N.
  The damped `J_m^-T` (lambda 0.05, addition (g)) alone gives 62 % on a perfect joint torque (the
  push direction is the arm's weakest, singular value 0.066 m); the rest is the ACTNDO's attribution
  -- the contact moment (the fin is gripped 25 mm under its top) and the arm's own friction / gravity
  mismatch fold into `tau_hat` (j2: +0.088 N.m over the true joint torque). The TSKFE integral supplies
  the difference while sliding (`D_hat_y` -1.1..-1.5 N), but it is slow, so the base carries the
  friction transitions on its soft position loop. lambda 0.02 would give 91 % on a perfect torque and
  56 % on the observer's -- not changed mid-campaign.
- **Not changed:** the position rows' eta (1.04). 1.5 is marginal in free flight already
  (`tools/bench_atc.py`: hold 45 mm, a 1 m excursion after the yaw step) -- the `Phi_d' = 0` cascade
  caps the position bandwidth on this plant.

### How the impedance residual is scored for this rig (task space, 2026-10-25)

User decision, 2026-10-25: every rig's residual is scored in the SAME task space, the claw's world
motion against its streamed reference and the true contact wrench on the claw:

    e_imp = F_ext - (M_t e_v' + D_t e_v + K_t e_y)

The whole-body rig declares `M_t / D_t / K_t` directly (its task impedance), the coordinated rig its
admittance; this rig declares a JOINT-space, arm-relative admittance `H_d e'' + D_d e' + K_d e =
tau_ext`, so `pl_metrics.py` (rig `atc`) uses its task-space image at the push pose,
`M_t = J^-T H_d J^-1` (likewise D_t, K_t), `J` = the claw's [position; heading] Jacobian in the model
frame, rotated by the streamed heading. At the tuned admittance (K_d 3 N.m/rad) that declares 686 N/m
along the arm (= the push: the arm's weakest Jacobian direction, singular value 0.066 m) and 210 N/m
across it. Because the paper's admittance is relative to the BASE, the base's tracking error enters
the claw's world error in full, and through 686 N/m every millimetre of it reads as ~0.7 N of
residual -- the score is the rig's declared impedance in the frame the task is defined in, which is
what the user asked for.

A force residual scales with the declared stiffness (whole-body 120 N/m, coordinated 150 N/m, this
rig 686 N/m along the push), so `pl_metrics.py` also reports its DISPLACEMENT equivalent `K_t^-1
e_imp` for every rig.

(Until 2026-10-25 this rig was scored against its own joint-space target, `J^-T(tau_ext - (H_d e_q''
+ D_d e_q' + K_d e_q))`: atc_06's slide read 3.6 N rms = 10.2 mm there; in task space the same run
reads 24.4 N = 15.7 mm. That scoring is retired.)

## 5. Tuning and this rig's own guards (2026-10-25)

User request: tune the paper's parameters and its safety guards (the whole-body rig's guards
untouched), score in task space, at least three successful runs. Flights `runs/t*`, each a clean
slate + `tools/run_pl_atc.sh` with the results campaign's driver flags (`tools/atc_chain.sh`, one
`<tag>:<yaml>[:ENV=v]` job per flight); variants in `variants/`.

| flight | change from the K 9 campaign config | outcome | box lag peaks (behind / ahead) | slide e_imp rms (task) |
|---|---|---|---|---|
| t0_guard | guards 70 mm / 3.5 N | box tilted 3.6 deg > 2 deg | +59 / -51 mm | -- |
| t1_lam002 | `atc_ie_lambda` 0.05 -> 0.02 | box lag 75 mm behind | +73 / -30 mm | -- |
| t2_adm3 | admittance H/D/K 1/4/9 -> 0.333/1.34/3 | **completed** | +25 / -14 mm | 7.3 N = 14 mm |
| t3_adm3_rep | repeat | **completed** | +59 / -69 mm | 21.0 N = 34 mm |
| t4_adm15 | admittance K 1.5 (H 0.167, D 0.67) | box 78 mm ahead | +53 / -78 mm | -- |
| t5_adm10 | admittance K 1.0 (H 0.111, D 0.45) | box lag 72 mm behind | +74 / -64 mm | -- |
| t6_adm3_g100 | K 3, box-lag guard 100 mm | **completed** | +63 / -49 mm | 19.6 N = 37 mm |
| t7_adm3_d2 | K 3, D 2.0 (zeta 1), guard 100 mm | **completed** | +42 / -32 mm | 17.6 N = 30 mm |

(Two first attempts at t6/t7 are in `runs/invalid_concurrent/`: two copies of the chain ran at once
and their clean slates killed each other's stacks -- not results.)

- **The admittance is the lever, and K 3 keeps the paper's dynamics.** H/D/K 1/3, 1.34, 3 is the
  paper's admittance with one third of its stiffness: omega = sqrt(K/H) = 3 rad/s and zeta =
  D / (2 sqrt(KH)) = 0.67, both exactly the paper's. With the arm three times more compliant the
  ACTNDO estimate the base feed-forward carries is steadier (~70 % of the true push force at K 3);
  softer still (K 1.5, 1.0) the estimate collapses as the arm yields (planner-filtered |F_hat| peaks
  0.65 / 0.88 N against 1.1-1.5 N at K 3) and the base carries the friction on its own soft loop
  again. K 3 completed 4 of 4 tuning pushes. zeta 1 (t7) completed once with lower lag peaks but a
  100 mm base excursion in the exit; on one flight it is inside K 3's scatter, so the paper's zeta
  stays.
- **What K 3 buys is reliability, not a smaller residual.** The two K 9 pushes that got through
  (atc_06, the K 9 campaign's run 3) read 24.4 / 31.3 N in the slide = **15.7 / 22.3 mm** as
  displacement, claw world error 12.0 / 17.6 mm rms; the four K 3 pushes 7.3-21.0 N = **14-37 mm**,
  claw error 9-32 mm rms. The force residual is lower at K 3 mostly because the declared stiffness is
  one third (686 vs 2057 N/m along the push); in displacement terms K 3 is no better, because the
  softer arm yields more to the stick-slip.
- **The A/B at the final guards** (the K 9 campaign's failures had tripped the old 40 mm lag guard at
  41-58 mm, which the new 100 mm guard would not catch, so its count alone does not rank the two):
  K 9 flown twice with exactly the final guards (`variants/t_v7_k9_final_guards.yaml`, flights
  k9a/k9b_final_guards): **0 of 2** -- box tilted 3.1 deg (17 s into the push), then the box dragged
  58 mm sideways. K 3 at the same guards: 3 of 5 in the campaign. The box-tilt reading separates them
  cleanly: K 9 tips the clamped box 2.6-2.8 deg in its pushes (t0, k9a), K 3 never above 0.31 deg --
  the stiffer arm transmits the stick-slip's force steps to the fin as a moment. Counting every
  flight at the otherwise final law (development flight atc_06 onwards), K 9 completed 2 of 12 (the
  K 9 campaign 1 of 8, atc_06, t0, the A/B 0 of 2) and K 3 10 of 13 (tuning 4 of 4, the pre-guard
  campaign 3 of 4, the final campaign 3 of 5).
- **lambda 0.02 on the damped `J_m^-T`** makes the feed-forward larger (91 % of a perfect torque) and
  the stick-slip worse: it over-pushes at each break-away (t1).
- **What remains: the stick-slip cycle.** The base's position stiffness is the floored law's,
  `m (1 + eta^2) / t_floor^2` ~ 22 N/m (its bandwidth capped by the attitude loop and the rotor lag,
  section 4); against the box's 1.23 N of friction the box swings ahead of and behind its plan in a
  3-4 s cycle (`tools/push_cycle.py`). Every completed push rides it; it is what the task-space
  residual sees.

### The guards (`ATC_GUARDS` in `tools/make_arbitrary_time_yaml.py`, planner section only)

Read off the four completed K 3 pushes with `tools/guard_readings.py` (contact phase = end of Ready
to the start of Exit):

| reading | t2 | t3 | t6 | t7 | guard | margin |
|---|---|---|---|---|---|---|
| box lag peak | 25 mm | 69 mm | 63 mm | 42 mm | **100 mm** (whole-body rig 40) | 1.45x the worst |
| force guard input (low-passed 3 rad/s) | 1.12 N | 1.36 N | 1.46 N | 1.10 N | **3.5 N** (whole-body rig 5.0) | 2.4x |
| box tilt | 0.10 deg | 0.31 deg | 0.16 deg | 0.08 deg | 2 deg (shared) | 6x |
| q3 min | 27.9 deg | 25.4 deg | 25.9 deg | 26.1 deg | 10 deg (shared) | |
| box dragged sideways (max) | 3 mm | 15 mm | 35 mm | 15 mm | **45 mm** (added, off on the other rigs) | 1.3x |

The force guard reads THIS node's estimate (`atc_control_debug` [97..99]), which under-reads the true
contact force along the push, so its value is lower than the whole-body rig's. The whole-body yamls
are untouched (40 mm / 5.0 N, no sideways guard).

**The sideways guard is new, and it exists because the box fell off the table twice.** In the first
K 3 results campaign (flown before it existed, `campaign_k3_pre_cross_20261025/`: 3 of 4 completed)
attempt r3_a1 dragged the box ~90 mm across the 150 mm-deep table in the push's last seconds; the
box-tilt guard caught it tipping (3.5 deg) and released, but its centre of mass was already over the
side edge (a 160 mm box: over after 75 mm), and it fell to the floor when the claw climbed away. The
K 9 tuning flight t0 ended the same way (450 mm sideways, box on the floor). Mechanism: the true
sideways contact force (0.2-0.4 N rms, the box yawing 5-8 deg while it stick-slips) pushes the soft
base sideways (the base's lateral error correlates 0.6-0.85 with it; 0.3 N on ~22 N/m is ~14 mm), and
the base-relative arm admittance carries the clamped box with the base. The whole-body and coordinated
rigs drag the box 2-5 mm (their claws are held in the room). No existing guard measures it: the box
lag is along the push, and the tilt comes only once the box is already over the edge.

`push_pull_guard_box_cross` (fsc_trajectory_planner, 2026-10-25, default 0 = off): the box's horizontal
displacement across the push since the push step started, above the limit for the box guards' 0.2 s.
45 mm. Offline replay over 21 recorded pushes (`tools/cross_guard_replay.py`): no trip in any completed
one; in r3_a1 and t0 it trips with the box's CoM 8-14 mm inside the edge -- and in r3_a1 the base's
swing carries the box ~20 mm further in the 0.6 s the jaws take to open, so it **reduces** the side-edge
risk, it does not remove it (40 mm would trip 19 mm inside and still end 5 mm over). Loopback case
[9l] (the box dragged 30 mm passes, 60 mm aborts naming it); the whole-body loopback still ALL PASS.

### Jam tests at the tuned configuration

(Flown before the sideways guard was added; their sideways drag peaked at 6 mm (j1) and 25 mm (j2),
so it would not have changed either.)

| test | what tripped | when | box | vehicle |
|---|---|---|---|---|
| j1_stopper (`PEGASUS_PUSH_STOPPER_Y=-0.04`: the box hits a stop after 0.20 m) | box tilt 4.7 deg > 2 deg | 2.0 s after the box hit the stop (box lag 87 mm, the lag guard not yet) | tipped to ~15 deg against the stop, fell back upright | tilt <= 9.7 deg (at the release), q3 >= 26.6 deg; climbed, flew off the table, landed |
| j2_heavy (1 kg box, mu 0.6: 5.9 N of friction) | force 4.1 N > 3.5 N | 9.4 s into the push, the box not yet moved (true force 7 N) | moved 18 mm, upright | released cleanly, then **left DIRECT 6 s into the abort's climb at 15.4 deg tilt** (the node's watchdog); SAFETY recovered, landed |

The j2 excursion is the paper's TSKFE, not the guard: through the abort's climb the fuzzy estimator
kept adapting on the base's lag behind the rising reference (`D_hat_z` 1.6 -> 4.9 N in 5 s, thrust
37 -> 45 N), then its output jumped back (`D_hat_z` 4.9 -> 1.4 N in 0.5 s) as the firing rules changed,
and the vehicle tilted. One flight; not chased further (it is the law's response after the contact
has ended, which the comparison does not change).
