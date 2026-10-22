# Box pushing: systematic stability tuning of the contact phase (Ready To Push → Push → Exit To Push)

**Plan written 2026-10-21 (Fable) for execution by the next session (Opus). Nothing in this
plan has been run. The tree was re-verified at hand-over (23:50) after the other thread finished its
development for the night — see "Tree at hand-over" at the end of section 0.** Scope requested by the user: a systematic gain tuning for the latest box-pushing
task, judged on STABILITY during the contact phase, Ready To Push and Exit To Push included, with the
10-21 hardware flight as the reference. The task is the one the hardware flies: whole-body 4-D L1 on
raw mocap, CAD box with the 5 mm fin, push pose [0, 32, 28, 0], 21 mm/s, box guards + mocap guard,
free-axis anchor 0.75, station automation.

Read first: `../wb_box_pushing_flight_20261021/README.md` (the flight), `../box_pushing_gain_tune_20261021/
README.md` (why gains alone do not fix the swing; the anchor; the Isaac A/B), the report
https://claude.ai/artifact/X1wtEcXhiuCYxKdiNdUE3Z, Command.md section 17.

---

## 0. Where we stand (facts the plan is built on)

| | hardware 10-21 `_170123` (anchor 0, driver-paced steps) | Isaac v21_1..3 (same config) | Isaac a75_1..3 + v75_n1 (SHIPPED: anchor 0.75, station automation) | bench, hardware noise, anchor 0.75 |
|---|---|---|---|---|
| open-jaw room hold before the grasp | 3.05 s (2.4 s waiting for the GS gate) | 2.3 s | 0.4–1.0 s | — |
| arm swing (q2−q3)/2 rms, world-held | **10.5°** @ 0.54–0.62 Hz | 5.3–5.9° @ 0.6–0.7 Hz | **1.9–2.8°** | 2.6–2.7° |
| q3 min (guard 10, elbow branch < 0) | **−7.3°** | 13.2–18.1 | 19.4–25.3 | 18.9 |
| q2 max (hardware limit 45) | **45.3°** | 39.6–42.4 | 38.1–39.0 | 39.7 |
| CoM error along the arm, rms | 14–15 mm | 6–8 mm | 8.7–14.9 (whole push step) | — |
| box pushed BACK while clamped (settle) | 10.5 mm | 0–5.5 | 0–19.1 | (no box) |
| box stuck in the slide / peak speed | 20 % / 103 mm/s | 0–11 % / 58–75 | 0–20 % / 68–87 | — |
| box travel at the push end (plan 100) | 107 (90 at the abort) | 98–101 | 91–108 | — |
| box-lag guard reading (limit 40) | 24 (37.5 re-estimated with the anchor) | 10–15 | 7.8 (bare 12.2) | — |
| vehicle tilt | 5.7° | 2.6–3.3 | 1.4–2.5 | — |
| outcome | q3-guard ABORT 0.78 s after the push | normal exit 3/3 | normal exit 4/4 | 3/3 seeds inside the limits |

Established, not to be re-derived:

1. **The instability of the contact phase is the room-held arm taking up the vehicle's low-frequency
   wander**, not a lightly damped mode: 25 gain candidates (attitude, position, task impedance,
   heading, M_r_d, observer bandwidths) left the bench at 0/2–0/3 seeds; only the observer bandwidth
   moved it at all (`sweep_r1_hw.txt`, `sweep_r2_hw.txt`, `ablate.txt`). The free-axis anchor
   (`wb_ee_anchor_free_axis_weight` 0.75) is the structural fix and is already in both box yamls.
2. **Hardware swings 2× Isaac** because the hardware's raw-mocap velocity carries 3–6 cm/s of noise
   in a ~5 Hz band (the bench reproduces the flight with 4.5 cm/s AR(1) at 5 Hz) while Isaac's
   emulator injects 2.2 cm/s WHITE per sample (`isaacsim_optitrack_ros2_emulator_node.cpp:504`).
   The same 4.5 cm/s spread to 50 Hz gives only 2.7–3.0° on the bench vs 7.6–7.8° band-limited.
   The swing scales with the LOW-FREQUENCY feedback error.
3. **The anchor does NOT act during Ready To Push.** The client latches the free axis only when
   `l1.contact && !ee_anchor_target_` (`...client.cpp` `stepEeFreeAxis`), and the planner sets the
   contact flag at the Push press, after the jaws grip. From the claw's arrival on the handle to the
   grasp the claw is room-held at weight 0: exactly the 10-21 configuration (6.2° rms after 1.6 s on
   hardware). Station automation shrank that window to 0.4–1.0 s in Isaac; on hardware add the real
   jaws' ~0.5–0.7 s close. The bench's 3/3 at 0.75 was run with the weight on throughout, so it
   describes the CONTACT phase, not Ready.
4. **The release is a 1.0 s blend under a clamped claw.** Exit = `push_pull/release` (EE anchor back
   to the CoM, contact off, free axis unlatched; all faded over `wb_ee_anchor_blend_s`, node DEFAULT
   1.0 s — the key is NOT in either box yaml) → gripper open on the release reply → exit climb on the
   OPEN report (`push_pull_exit_dz` 0.25 over `push_pull_exit_time_s` 3.0). During the blend the
   reference walks toward the CoM anchor while the box still holds the claw (box moved ≤ 1.6 mm in
   6/6, `box_pushing_release_20261017`).
5. **The settle (`push_pull_push_settle_s` 1.5) overlaps the free-axis fade-in** (contact flag at the
   grasp → the anchor blends in over 1.0 s → the slide starts at 1.5 s). The clamped box is pushed
   back during this window (10.5 mm hardware, up to 19 mm Isaac a75_1).
6. **Hardware-only effects Isaac cannot show**: the arm's u3 feed-forward learned the joints'
   STICTION under the CoM anchor and sagged the claw 8–10 mm at the world-hold switch
   (`claw_offset.txt`; `wb_l1_omega_x_q` 0.1 → τ 10 s); `obj_0` mis-solves (now gated,
   `push_pull_box_gate_jump_m/deg` 0.020/10, `../box_mocap_gate_20261021`); the processor's velocity
   spikes after a mocap pause. The abort now unbuckles the arm (`push_pull_abort_unbuckle: true`,
   `../box_abort_unbuckle_20261021`). Both are in the yamls and in the planner binary built 23:43.

**Tree at hand-over (verified 2026-10-21 23:50).** Every binary is newer than its sources: planner
`~/ros2_ws/build/fsc_trajectory_planner/whole_body_trajectory_planner` 23:43 (contains
`push_pull_box_gate_jump_m`, `push_pull_abort_unbuckle`, `push_pull_push_mean_speed`,
`law free-axis shift`); node `~/ros2_ws/build/fsc_autopilot_ros2/.../autopilot_whole_body_l1_direct_actuation_node`
12:59 (`ee_anchor_free_axis_weight`, `PX4 LEFT OFFBOARD`); arm GS
`~/ros2_ws/build/utils_custom_ground_station/joint_plot_inverted` 21:03 (= `push_pull_panel.cpp`'s
time); estimator `indoor_mocap_feedback_node` 23:45. Both box yamls regenerated 23:32,
`make_push_pull_yaml.py --check` UP TO DATE. Uncommitted in five repos (do not commit for them):
Pegasus (generator, `pl_mission.py` — new flags `--gs-auto` / `--gs-exit`, `run_pl.sh`, the planner
check lib, Command.md, CLAUDE.md), fsc_autopilot_ros2 (the three box yamls, the two hardware stack
scripts), fsc_trajectory_planner (unbuckle + box gate + T_max refusal, tests), fsc_open_manipulator
(`push_pull_panel.{hpp,cpp}`), fsc_drone_state_estimator_ros2 (`mocap_odom_bridge` second-fault fix).
No tmux server, no Isaac / PX4 / agent process. If any of these files changes before phase 1, redo
0.1 and re-fly the control.

So the tune's job is not to find the gain that stops the swing. It is: (a) make the simulation
hardware-like enough that a verdict transfers; (b) tune the TRANSITIONS (anchor blend, settle, the
Ready window, release) and the residual damping on top of the anchor; (c) measure margins and
robustness of the result; (d) ship with the hardware ladder's acceptance gate updated.

---

## 1. Objective and acceptance criteria

"Stable" is defined per sub-phase, scored from the driver's marks in each run's npz
(`ready:end` = arrival on the handle / world hold on; `gripper_close:start/end`; `push:start`;
`push:end`; `release:start`; `gripper_release:start/end`; `exit:start/end`). Thresholds are set
from the shipped Isaac baseline (a75_1..3, v75_n1) with margin to the guards; a candidate must meet
them on EVERY repeat, nominal AND hardware-like noise (section 3).

| phase | metric | pass (Isaac nominal) | pass (Isaac hardware-like) | note |
|---|---|---|---|---|
| **Ready** (arrival → jaws closed) | R1 open-jaw hold time | ≤ 1.0 s | ≤ 1.0 s | automation; hardware adds the real close ~0.6 s |
| | R2 swing (q2−q3)/2 std | ≤ 3.0° | ≤ 5.0° | the anchor is OFF here (fact 3) |
| | R3 q3 min / q2 max | ≥ 20° / ≤ 40° | ≥ 15° / ≤ 42° | guard 10 / limit 45 |
| | R4 claw error at the close (e_y xy) | ≤ 5 mm | ≤ 8 mm | the GS gate is 20 / 3 mm |
| **Push** (grasp → push end; settle and slide scored apart) | P1 swing std | ≤ 3.0° | ≤ 5.0° | shipped 1.9–2.8 |
| | P2 q3 min / q2 max | ≥ 18° / ≤ 40° | ≥ 15° / ≤ 42° | |
| | P3 vehicle tilt max | ≤ 3.5° | ≤ 5.0° | hardware guard 10 |
| | P4 box pushed back in the settle | ≤ 5 mm | ≤ 8 mm | shipped up to 19 (one run) |
| | P5 box stuck share / peak speed | ≤ 15 % / ≤ 2× plan (95 mm/s) | same | plan peak 47 mm/s |
| | P6 box travel at the push end | 100 ± 10 mm | 100 ± 15 mm | the anchor's accepted cost |
| | P7 box-lag guard reading (compensated) | ≤ 20 mm | ≤ 25 mm | limit 40; `[law free-axis shift]` counted |
| | P8 impedance residual e_imp, slide | ≤ 1.0 N rms | ≤ 1.3 N | `pl_metrics.py`; shipped 0.66–0.8 at 500 g |
| | P9 force guard reading (filtered) | ≤ 2.5 N | ≤ 3.0 N | limit 5 |
| **Exit** (push end → exit end) | E1 q3 min / q2 max, push end → jaws open + 1 s | ≥ 20° / ≤ 40° | ≥ 15° / ≤ 42° | the 10-17 crash window; shipped ≥ 25.8 |
| | E2 box moved after the push end | ≤ 3 mm | ≤ 5 mm | shipped ≤ 1.6 |
| | E3 vehicle tilt max | ≤ 3.5° | ≤ 5.0° | |
| | E4 push end → jaws open | ≤ 1.0 s | ≤ 1.2 s | automation 0.2 s + release reply + open |
| **global** | mission complete, no guard trip, 0 saturation, 0 joint clamp | 3/3 | 3/3 | one abort = disqualified |
| | bench delay margin (first delay that diverges) | ≥ the shipped config's | — | ordering only (section 3) |

Primary ranking metrics: P2/E1 (joint margin), R2/P1 (swing), P4/E2 (box disturbed by the arm).
Secondary (must not regress beyond scatter): P6, P7, P8, P3. The scatter is measured, not assumed:
the shipped config's own spread over its repeats (3 flights + the 4 existing) is the floor; a
difference counts only if the means differ by more than 1.5× the pooled std.

---

## 2. Instruments, and what each can and cannot decide

| instrument | what it models | can decide | cannot decide |
|---|---|---|---|
| **bench** `../box_pushing_gain_tune_20261021/tools/wh_bench.py` (+ `sweep.py`): exact law + L1 observer (controller.py / l1_observer.py = the C++'s source of truth) on the mirror plant, rotor lag, 16 ms transport, arm friction + stops, raw-mocap noise 4.5 cm/s AR(1) 5 Hz, holding the 10-21 flight's own reference | the room-held claw, jaws open (= Ready) or, with `free_w`, the contact-phase geometry WITHOUT a box | gain ordering at HARDWARE disturbance level; delay margin; the stiction sag (stick-slip friction is in the plant) | anything with the box (travel, stick-slip, release); the anchor blend transient (its weight is static); planner timing |
| **Isaac nominal** (`sim_feedback_vel_noise_mps` 0.022 white; what every box campaign flew) | the full chain: planner, GS automation, box, guards, contact | transitions, box effects, guard margins, repeatability; relative ordering at HALF the hardware disturbance | hardware-level swing amplitude (fact 2) |
| **Isaac hardware-like** (new, section 5 phase 1) | Isaac with the mocap velocity noise calibrated to the 10-21 bags | the above at the hardware's disturbance level | unmodelled hardware effects (stiction, mis-solves, velocity spikes, real jaw timing) |

Rule: a hardware-level claim needs the bench AND hardware-like Isaac to agree; Isaac nominal alone
ranks transitions and box effects; the bench alone ranks gains and margins. Never quote the bench's
absolute stability verdict (its 2026-08-23 header: ordering only).

---

## 3. The knobs, by what they act on

Shipped values are in Appendix A. "Where" says which instrument ranks the knob.

### Tier 1: transitions (the Ready window and the release) — Isaac, the main lever left

| id | knob | shipped | candidates | mechanism, expected direction | where |
|---|---|---|---|---|---|
| T1 | `wb_ee_anchor_blend_s` (node default; ADD to the box yaml via the generator's ADDITIONS so it is explicit) | 1.0 | 0.5, 1.0, 2.0 | the one time constant of three hand-overs: CoM→world at arrival, the free-axis fade-in at the grasp, world→CoM at the release under a clamped claw. Shorter = less time half-anchored and clamped under a moving reference, but a steeper step into the arm; longer = gentler, longer exposure. Constraint: the settle (T2) must stay ≥ the blend. | Isaac |
| T2 | `push_pull_push_settle_s` | 1.5 | 1.0, 1.5, 2.5 | the clamped pause before the slide, where the box is pushed BACK (P4). Shorter cuts exposure but overlaps the fade-in; longer lets the base settle after the jaws take the fin. Pair each with T1 (settle ≥ blend). | Isaac |
| T3 | arm GS `push_pull_auto_ready_delay_s` (+ `push_pull_grip_dwell_in` 0.3 in the driver / GS gate) | 1.0 | 0.5, 1.0 | the open-jaw room-hold window R1; the gate itself (20 / 3 mm) is a SAFETY setting, do not widen it | Isaac |
| T4 | **free-axis anchor active from the world hold, not from the grasp** | off (fact 3) | A: node option (new, default off) that latches the free axis when the EE anchor goes world; B: planner `push_pull_contact_at_ready: true` (exists; turns the contact flag on at arrival — with open jaws the 4-D attribution then renders the raw F̂ reading as a task force, which is why it was turned off 10-06) | removes the unprotected Ready window entirely; the bench's 3/3 at 0.75 IS this configuration. A is the clean one (~20 lines in `stepEeFreeAxis`, parity untouched: the law already has the term); B is a no-code A/B with a known side effect. | bench (already: `sweep_r3_hw`), Isaac hardware-like for the grasp accuracy R4 and P6 |
| T5 | `push_pull_world_anchor_on_handle` false (world hold at the grasp instead of at arrival) | true | one A/B pair only | removes the Ready swing but the claw follows the base's drift while the GS waits for its 3 mm across-fin gate (10-07: "never gripped" with the driver's slower gate; automation closes at first entry) | Isaac, low priority |
| T6 | `push_pull_exit_time_s` / `push_pull_exit_dz` | 3.0 / 0.25 | 2.0 / 0.25 | the climb after the jaws open; shorter = less time with the open claw beside the fin | Isaac, only if E2 shows re-contact |

### Tier 2: the anchor weight itself — Isaac hardware-like with the box

| id | knob | shipped | candidates | trade |
|---|---|---|---|---|
| A1 | `wb_ee_anchor_free_axis_weight` | 0.75 | 0.5, 0.75, 1.0 | swing / joint margin (bench 4.3–4.8° → 2.6 → 0.9) vs push accuracy (P6) and later jam detection (P7: a stuck box's lag is excused in proportion). 1.0 ran away once on the valve with a fast translational observer (ω_c_t 2.927); the box flies 1.0. Already measured in Isaac NOMINAL (a75) and the bench; the open question is hardware-like Isaac WITH the box. |

### Tier 3: residual damping — bench first (hardware noise, `free_w` 0.75 AND 0), then ≤ 4 finalists in Isaac

The 10-21 sweep ran at weight 0. Re-screen a reduced set at the shipped 0.75 (the contact phase) and
at 0 (the Ready phase, unless T4 ships), 3 seeds each, common random numbers, scored on swing rms,
q3 min, q2 max, CoM rms, τ peak, and the delay margin at 16 → 24 → 32 ms.

| id | knob (bench key) | shipped | candidates | why |
|---|---|---|---|---|
| D1 | `wb_l1_omega_c_t` (`omega_c_t`) | 1.0 | 0.7, 1.5 | the only group that moved the swing at weight 0 (×0.5 → −10 %); 0.5 tripled the valve's lead; 2.0 is the valve's; 1.5 was "inside scatter" on the 10-09 push |
| D2 | `wb_l1_omega_c_q` (`omega_c_q`) | 0.8479 | 0.6, 1.2 | the arm rows' observer bandwidth; sets how fast the stiction-learned sag unlearns too |
| D3 | `wb_l1_omega_c_r` (`omega_c_r`) | 0.7428 | 0.5, 1.0 | rotational channel; 1.25 diverged on the pick-and-drop rig, 1.0 safe there |
| D4 | `wb_dy_x/y/z` (`dy`) at fixed `ky` 120 | 20.2 (ζ 0.92) | 25, 30 | task damping; 40 left DIRECT in free flight (do not go there) |
| D5 | `wb_ky_x/y/z` + `wb_dy` at ζ 0.92 (`ky`, `dy`) | 120 / 20.2 | 90 / 17.5, 150 / 22.6 | stiffness: softer = more claw deflection under the 1.2 N push (F/K_y), stiffer = more load into the swing; 300 unstable in the grasp |
| D6 | `wb_l1_omega_x_q` (`omega_x_blk` per block) | 0.1 | 0.05, 0.2 | the u3 feed-forward's arm rows: faster unlearns the stiction sag sooner, noisier; 0.4 "does nothing", 1.0/4.0 make the release a jolt |
| D7 | `wb_k_x` (`k_x`) at `k_v` 12.58 | 50.03 | 40, 32 | 32 delayed the bench divergence 7 → 15–17 s at weight 0; 32/10 tripped the force guard, 64/14 aborted in hover |
| D8 | `wb_k_r` / `wb_k_w` (`k_R`, `k_w`) | 1.6 / 1.2 | 1.6 / 1.5 | more attitude damping only; 2.134/1.567 aborted in hover on the push, 1.3/1.0 tilted 5.4° |

Bench-only (hardware effects Isaac does not have): D2 and D6 also scored on the claw's static z
offset over the first 3 s of the world hold (the stiction sag, `claw_offset.py`'s prediction: −9 mm
shipped).

### Not to be re-tried (measured, each with a record)

K_y 300 (unstable in the grasp); D_y 40 and k_v 20 / 25 (left DIRECT in free flight); `wb_l1_omega_x`
1.0 / 4.0 (release jolts 16–20 N); k_x/k_v 32/10 (force guard 24 N) and 64/14 (hover abort);
`wb_ky_psi`/`dy_psi` 2.5/0.93 on the push (fights the fin, q3 margin given back);
`wb_l1_contact_hold_translation` (stretched the arm to the wrist singularity);
`wb_l1_stuck_hold_j2/j3` (freezes the stiction the slow fold already taught); the cruise push
(`push_pull_push_ramp_s` > 0); speeds 8 and 30 mm/s; a bare-plan box-lag guard at 55 mm.

---

## 4. Campaign structure

Budget: an Isaac mission of the 10-21 task is ~6–7 min wall incl. the clean slate (≈ 9/hour). The
whole plan is ~55–65 flights ≈ 7 h of Isaac plus ~2 h of bench, over 1–2 days. Phases are ordered so
the plan can be cut after any phase and still ship a justified answer.

### Phase 0: freeze, instrument, baseline (no Isaac; may run while the other thread tests)

0.1 Record the tree in the campaign README: `git log -1` + `git status --short` of Pegasus,
    fsc_autopilot_ros2, fsc_trajectory_planner, fsc_open_manipulator, isaacsim_optitrack_ros2_emulator,
    fsc_drone_state_estimator_ros2; the sim and hardware box yamls copied into `configs/`.
    Re-run the binary check of "Tree at hand-over" (section 0): `strings <binary> | grep -c <key>` for
    the four planner keys and the two node strings, and `find <src> -newer <binary>` for the planner,
    node, arm GS and estimator. The stack's own check refuses a planner without the keys, but a
    mismatch found that way costs a hung flight (~10 min). Any source newer than its binary = rebuild
    that package first (`colcon build --packages-select ...` from `~/ros2_ws`, with no sim running).
0.2 Tools, COPIED into `tools/` here (never edit the live ones: `pl_mission.py`, `run_pl.sh`,
    `fly.sh`, `wb_entry_sim.py`, `push_pull_panel.*`):
    - `fly.sh` from `../box_pushing_gain_tune_20261021/tools/` (has `FREE_W`), extended so that
      `sim_feedback_*` and `wb_ee_anchor_blend_s` keys go through the same sed path as `push_pull_*`
      (the stack reads the yaml's `sim_feedback_*` at start; `wb_l1_set_gains.py` does not know
      either key). Every flight records `.args` + the as-flown `.yaml`.
    - `phase_metrics.py`: `ab_metrics.py` extended to the table of section 1 (per sub-phase windows
      from the marks, the settle split from the slide, the release window E1 = `push:end` →
      `gripper_release:end` + 1 s, the compensated box-lag reading from the planner's
      `arrival_error`, the force-guard reading low-passed as the planner does it, saturation /
      clamp counts) plus `pl_metrics.py`'s e_imp. One row per run, a pooled row per configuration
      with mean [min..max] and std.
    - `chain.sh` template: interleaved A/B/control order, `touch PAUSE` honoured before every clean
      slate, PID captured with `$!`, `CHAIN DONE` at the end, Isaac closed afterwards.
    - `bench_sweep.py`: `sweep.py` with the Tier-3 rounds at `free_w` 0.75 and 0, 3 seeds, and a
      delay-margin column (re-run the passing cases at 24 and 32 ms; the margin = the first delay
      that diverges).
0.3 Re-score the existing runs with `phase_metrics.py` to get the Isaac-nominal baseline
    distribution for free: `../box_pushing_gain_tune_20261021/runs/{a75_1..3,v75_n1}` (shipped),
    `{a0_1..3}` and `../box_pushing_speed_20261021/runs/v21_*` (anchor 0), `../box_pushing_release_delay_20261021/runs/d02_*`.
    Put the 10-21 hardware numbers beside them (from `../wb_box_pushing_flight_20261021/{phases,push_metrics,spectra}.txt`).
0.4 Bench baseline: the shipped config at `free_w` 0.75 and 0, 3 seeds, delay 16 / 24 / 32 / 40 ms →
    the shipped delay margin, the reference for Tier 3.

### Phase 1: make Isaac hardware-like (3–6 flights, after the other thread's sanity check)

Goal: an Isaac condition whose anchor-0, driver-paced push reproduces the 10-21 swing (8–12° rms),
q3 excursion and hover CoM error (13–23 mm rms) within ~30 %. Preferred route A: give the emulator a
band-limited velocity-noise option, `velocity_noise_bw_hz` (0 = white, bit-identical to today),
exact-discretization AR(1) as `servo_model.py` does it (`a = exp(−2π f_c dt)`, so the std does not
depend on the rate), a launch argument, the stack script's `sim_feedback_vel_noise_bw_hz` pass-through
beside the three existing keys, the key in the mirror `_sim.yaml` plant section at 0.0 (so every
regenerated yaml is unchanged), and the campaign overriding 0.045 / 5.0 per flight. Calibrate with 2
flights at anchor 0 (`FREE_W=0`, driver steps, no automation = the 10-21 flow); accept if the swing
and CoM error land in the band; if not, try 0.035 and 0.055 once each. Route B (no code, fallback):
white noise raised until the anchor-0 swing matches — expect ~3× the std, with unrealistic
high-frequency content (rotor and torque chatter); usable for ranking only, state it. If neither
matches, the bench stays the hardware-level instrument and Isaac stays nominal.

Deliverable: the "HW-like" `fly.sh` environment (`NOISE=0.045 NOISE_BW=5`), and the shipped config
flown 3× under it (the control for phases 2–4).

### Phase 2: transitions in Isaac (T1–T3, then T4; ~24 flights)

Task: 240 g, 0.10 m, 21 mm/s, hardware guard values (`push_pull_guard_tilt_deg=10.0
push_pull_abort_release_wait_s=1.5`), station automation (`ARM_GS_AUTO_STEPS=true
ARM_GS_AUTO_EXIT_DELAY=0.2 PL_DRIVER_ARGS="--gs-auto"`), HW-like noise if phase 1 delivered it,
else nominal. Interleave with the shipped control; 3 repeats each.

- 2.1 T1 blend 0.5 and 2.0 (1.0 = control) → 6 flights. Read E1/E2 (release) and R2/P4 (grasp).
- 2.2 T2 settle 1.0 and 2.5 with the best blend (settle ≥ blend) → 6 flights. Read P4, P1.
- 2.3 T3 ready delay 0.5 → 3 flights. Read R1–R4.
- 2.4 T4-B (`push_pull_contact_at_ready: true`) → 3 flights, read R2/R3 and R4/P6 for the side
  effect. If R2 drops to the contact-phase level AND R4/P6 hold, build T4-A (the clean node option)
  and fly it 3× — then T4-A is a ship candidate and the Ready thresholds move to the Push ones.
- 2.5 (only if E2 > 3 mm anywhere) T6 exit 2.0 s → 3 flights.

Decision: keep every change that beats the control on its primary metric by > 1.5× pooled std
without regressing a secondary beyond scatter, on all repeats.

### Phase 3: anchor weight and the gain finalists (~18 flights)

- 3.1 Bench Tier 3 (`bench_sweep.py`, hours, no Isaac): rank, then take ≤ 4 finalists that improve
  swing / q3 / q2 by > 10 % over the shipped at BOTH weights without losing delay margin.
- 3.2 Isaac: A1 0.5 and 1.0 (0.75 = control) → 6 flights, hardware-like noise, read P1/P2 vs
  P6/P7. Ship 1.0 only if P6 stays within ±15 mm and the stopper / heavy-box cases of phase 4 still
  trip (its excused lag grows with the weight).
- 3.3 Isaac: the ≤ 4 finalists on top of the phase-2 result → 12 flights. Expect small effects;
  the purpose is margins and no regressions. A finalist that does not beat scatter is NOT shipped.

### Phase 4: confirmation and robustness of the final configuration (~14 flights)

Interleaved with the shipped yaml as control where a comparison is meaningful.

- 4.1 The formal task 500 g / 0.50 m (`MASS=0.500 DIST=0.50`, box at y 0.28): 3× final, 2× control.
- 4.2 240 g / 0.30 m: 2× (the ladder's step 2).
- 4.3 Worst cases, 1× each, final config: stopper after 0.2 m; 1 kg / μ 0.6; the side-edge fall
  (box at 15° yaw, `push_pull_box_half_length=0`, guards on); mocap freeze 3 s mid-push
  (`--mocap-fault freeze@push+2:3`); μ 0.20 and 0.30; mass 300 and 400 g. Every guard must still
  trip where the 10-16/10-19/10-21 records say it did, and no false trip on the nominal pushes.
- 4.4 Noise seeds: 2 extra seeds on the formal task.
- 4.5 Bench: the final config's delay margin ≥ the shipped's (phase 0.4).

### Phase 5: ship

- `make_push_pull_yaml.py` OVERRIDES / ADDITIONS (the controller keys) and its planner block (the
  planner keys), with the measured reason beside each value as the file does today; regenerate
  sim, hardware and coordinated-admittance box yamls; `--check`; the two whole-body hardware stack
  scripts' expected-value lists; `am_plant_from_yaml.sh` / `push_pull_planner_check.sh` if a new key
  must be compared; the arm GS default only if T3 ships (laptop rebuild); the emulator + stack key if
  route A shipped (the mirror `_sim.yaml` at 0.0 keeps every other rig identical).
- README of this campaign: the before/after table (hardware 10-21, Isaac nominal, Isaac HW-like,
  bench), every candidate with its verdict, the as-flown yamls under `runs/`.
- Command.md section 17 and the CLAUDE.md entry (one paragraph, numbers only), the workflow page
  (`~/Documents/box_pushing_workflow`) if a GS default or a step changed; a memory note.
- The hardware ladder stays the report's, with the acceptance gate updated to the new baseline:
  step 1 240 g / 0.10 m → step 2 240 g / 0.25–0.30 m → step 3 500 g / 0.10 m → step 4 500 g / 0.50 m;
  move up only after a flight with the normal Exit, no guard trip, and the arm inside the Isaac
  hardware-like envelope for the same task (write the final q3 ≥ / q2 ≤ numbers into the README).

Stop rule: if phases 2–3 produce nothing that beats the shipped configuration beyond scatter, ship
NOTHING, and deliver the margins table (phase 4 on the shipped yaml) plus the hardware-like Isaac
condition — that is a result.

---

## 5. Coordination and operating rules (binding)

- **The other thread finished its development on the night of 10-21** (the user said so at
  hand-over). Before the first Isaac flight of any day, confirm with the user that no other thread
  is testing. Whenever one is: preparation only — phase 0 and the bench (run it
  `nice -n 19 taskset -c 16-31`; Isaac is pinned to 0–15). No Isaac, no ROS processes on any domain
  (the stack guards `pgrep -f` node names and ignores domains), no colcon builds (a clean slate
  kills them), no edits to live files (`pl_mission.py`, `run_pl.sh`, the 4-D launcher,
  `wb_entry_sim.py`, `push_pull_panel.*`, the generators, the stack scripts, the emulator). Keep
  node names out of every command line.
- **Use the latest tree** (the user's instruction): the unbuckle, the box gate, the push speed, the
  free-axis guard compensation and the station automation are in. Re-read `git status` in every
  repo before phase 1; if the other thread changed a yaml, a generator or the planner, re-run phase
  0.1 and re-fly the control.
- One Isaac at a time; serialized through `touch PAUSE`; chains waited on by PID (`$!`), never by
  `pgrep -f <script>`; `kill_stale_sim_processes.sh -y` in a Bash call of its own; a clean slate
  between every flight; close everything after each chain (`close-sims-after-campaigns`).
- Never rank on one run; interleave candidate and control; 3 repeats minimum; a divergence or a
  guard trip disqualifies the candidate regardless of its means.
- A bench verdict is ordering only; an Isaac-nominal verdict is at half the hardware disturbance;
  only bench + hardware-like Isaac together support a hardware claim.
- `fly.sh` regenerates the yaml every flight: never edit a generator while a chain runs; never edit
  a running shell script (byte-offset trap).
- CLAUDE.md and Command.md edits at the end, in one pass (the other thread edits them too).

---

## 6. Deliberately out of scope here (tracked elsewhere, not gains)

- The stiction-learned u3 feed-forward (claw 8–10 mm low): law code; the user set it low priority.
  Tier 3 only measures whether D2/D6 change it.
- Velocity sanitising after a mocap pause / flip (estimator or processor), the real table into
  `push_pull_table` (lab), the box's marker layout (lab, 10-22), the mocap-fault abort on hardware
  (flight test). Pre-flight items of the 10-21 report, independent of this tune.
- Pulling, the valve, the coordinated-admittance rig.

---

## Appendix A: the shipped configuration (both box yamls agree on every controller value)

Law: `wb_k_x` 50.03, `wb_k_v` 12.58, `wb_k_r` 1.6, `wb_k_w` 1.2, `wb_mrd_x/y/z` 0.139992 /
0.145617 / 0.143795, `wb_ky_x/y/z` 120, `wb_dy_x/y/z` 20.2, `wb_ky_psi` 0.2484, `wb_dy_psi`
0.2903, `wb_my_x/y/z` 1.0, `wb_my_psi` 0.05, `wb_dls_lambda` 0.3, `wb_tau_max` 3.0,
`wb_ee_anchor_com` true, `wb_ee_anchor_free_axis_weight` 0.75, `wb_ee_anchor_blend_s` ABSENT
(node default 1.0), `wb_u3_internal_ff` true (`_use_w_hat` true), `wb_u3_estimate_max_j4` 0.06,
armature joint-diagonal [0.010, 0.0194, 0.0097, 0.0097], arm q̇ from the velocity observer.
Observer (4-D): `wb_l1_omega_c_t/r/q` 1.0 / 0.7428 / 0.8479, `wb_l1_omega_x` 2.0 (the reading),
`wb_l1_omega_x_t/r/q` 0.1 (the feed-forward), `wb_l1_omega_q` 0.2, `wb_l1_contact` false at
startup (the planner switches it), `wb_l1_contact_hold_translation` false, stuck holds all false.
Guards (node): tilt 15 (sim) / `system_wd_max_tilt_deg` 15, rate 360, drift 0.75 m.
Planner block: pose [0, 32, 28, 0], EE offset [0, 0, 0.183], yaw −90 from the box, approach dz 0.15
/ back 0.12, exit dz 0.25 / time 3.0, settle 1.5, speed 0.021 (floor 3.0 s), distance 0.50
(hardware flies 0.10 today), world anchor: push true / ready false / on_handle true, descent trim
false, guards: tilt 15 (hardware 10), force 5 N, box lag 0.040 (vs the law-shifted target), box
tilt 2.0, persist 0.2, stale 0.5, q3 min 10, abort from measured + unbuckle, abort back 0.03,
release wait 1.0 (hardware 1.5), box gate 0.020 m / 10°, edge margin 0.03, half length 0.12,
table [0.600, 0.750, −0.5, 0.5, 0.80], gear depth 0.2425 (hardware 0.248), shared bounds v 0.30 /
a 0.15 / w 0.30 / dw 0.60 / t_max 40.
Plant (sim): mirror; `sim_feedback_mocap_rate_hz` 120, `sim_feedback_vel_noise_mps` 0.022 (white),
`sim_feedback_pos_noise_m` 0.0005; arm friction j2 0.70 / j3 0.65 / j4 1.5, velocity lag 0.048 s.
Arm GS (code defaults): auto ready delay 1.0, auto exit delay 0.2, grip gate 0.02 / 0.003, release
wait 0.0, release timeout 5.0, full close = grasp on hardware.
Scene (campaign defaults): mass 0.240 (ladder) / 0.500 (formal), μ 0.25 / 0.25, box at (0.675, 0.28),
fin grip shim 20 mm.

## Appendix B: tool map

- Flights: `tools/fly.sh <tag> key=val ...` (copied; `MASS DIST SPEED MU_S MU_K BOX_Y0 FREE_W
  NOISE NOISE_BW ARM_GS_AUTO_STEPS ARM_GS_AUTO_EXIT_DELAY PL_DRIVER_ARGS`), which calls
  `../push_pull_20261003/tools/run_pl.sh` → the 4-D stack (`WB_SIM_PROFILE=box_pushing`) + the
  box-pushing scene wrapper + `pl_mission.py --timetable --gs-auto ...`. Clean slate first, in its
  own call. The npz lands in `runs/<tag>.npz` with `.args`, `.yaml`, `.console.log`.
- Scoring: `tools/phase_metrics.py runs/*.npz`; `../push_pull_20261003/tools/pl_metrics.py` (e_imp,
  needs `PYTHONNOUSERSITE=1 /usr/bin/python3`).
- Bench: `tools/bench_sweep.py <round>` on `../box_pushing_gain_tune_20261021/tools/wh_bench.py`
  (gain keys: `k_x k_v k_R k_w mrd_s ky dy ky_psi dy_psi omega_c_t omega_c_r omega_c_q omega_x
  omega_x_blk omega_q dls free_w`; `delay_ms`; `fb` = `FB_RAW` 4.5 cm/s or `dict(FB_RAW, vel_noise=0.022)`).
- Hardware reference: `../wb_box_pushing_flight_20261021/{README.md,phases.txt,push_metrics.txt,spectra.txt,claw_offset.txt}`
  and its `tools/` (re-runnable on the bags).
- Generators: `../push_pull_20261003/tools/make_push_pull_yaml.py` (sim),
  `../box_pushing_campaign_20261016/tools/make_hw_box_pushing_yaml.py` (hardware; imports the sim
  overrides), `../push_pull_coordinated_admittance_20261010/tools/make_coordinated_yaml.py`.
