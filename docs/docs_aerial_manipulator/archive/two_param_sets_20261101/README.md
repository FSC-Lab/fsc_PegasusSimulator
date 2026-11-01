# Two parameter sets (free flight / interaction), the fast anchor hand-back, the guard audit, and the isolation tests (2026-11-01)

User request, after the 10-31 evening valve flight and the 11-01 Step 1 guards: "do you think we can use two sets
of parameters for the free-flight and the interaction phases? ... finish this tests of two sets of parameters and make
sure they work in the simulation. I also want you to think of some tests to isolate the problems ... Please also
double check the safety guards from the simulation and the experiments data, make sure the flight test will be safe.
After you finish all the tasks and tests above, please generate two prompts to deploy these fixes and changes for orin
and experimental laptop."

Uncommitted in fsc_autopilot_ros2 (`dev_robotic_arm`), fsc_trajectory_planner (`main`) and this repo. NOT flown on
hardware. Record of the Isaac runs: `runs/`, `chain_valve.out`, `runner.log`; scorers in `tools/`.

## 1. What changed

### 1.1 The two sets (whole-body node, fsc_autopilot_ros2)

The valve's grip schedule (2026-10-31: one event -- the jaws closed on the handle, read from the gripper's own joint
state -- switched the EE heading gains and the j4 stuck hold) now switches the whole arm-side set, blended over
`wb_grip_blend_s` (1 s):

| parameter | free flight (jaws open) | interaction (jaws closed) | why |
|---|---|---|---|
| `wb_ky_x/y/z`, `wb_dy_x/y/z` (EE impedance) | the mirror's 211.9 / 26.82 (box push: the push axis y x `wb_my_y` 1.64 = 347.5 / 43.98, so free flight is the mirror's along any heading) | the task's contact tune: valve 120 / 20.2, box 120 (196.8 on y) / 20.2 (33.1) | the contact tune halves the contact residual and keeps the arm off q3's stop at the release (10-09); the mirror's stiff task is what every free-flight hardware mission has flown since 10-27 |
| `wb_l1_omega_c_t` (translational observer) | the mirror's 2.927 | valve 2.0, box 1.0 | a box held by static friction reacts with whatever the vehicle applies, so a fast observer pushes through it (10-04); in free flight the wander is a slow force a faster observer cancels (10-22 bench: -25 %) |
| `wb_ky_psi / wb_dy_psi`, `wb_l1_stuck_hold_j4` (EE heading) | the mirror's 2026-10-16 wrist fix: 2.5 / 0.93 + stuck hold | the soft 0.2484 / 0.2903, stuck hold off | the 10-31 valve schedule (the stiff heading fights a clamped fin); NEW for the box push |

Not switched, on purpose: the base position loop (`wb_k_x/k_v`), the attitude pair (`wb_k_r/k_w`), `M_r_d`, the
rotational / arm observer bandwidths -- every contact screen that moved these failed in free flight or in contact
(README 10-21 gain tune, 10-22 contact tune, 10-28/29 valve sweeps), and switching the attitude loop in mid-flight
moves the modes this vehicle is closest to. The contact tasks keep `k_R/k_w` 1.6 / 1.2 throughout.

Keys (all optional, default = the yaml's interaction value throughout, so every config without them is unchanged):
`wb_ky_{x,y,z}_before_grip`, `wb_dy_{x,y,z}_before_grip`, `wb_l1_omega_c_t_before_grip`,
`wb_grip_hold_set_while_free_axis` (true in both contact yamls: the interaction set stays in force after the jaws
OPEN while the claw is still room-held on the handle -- the valve's 0.06 m world-held exit climb, the box's
release-first exit -- so the open jaws around a fin never get a gain step; it releases once the free-axis anchor is
gone), `wb_grip_release_delay_s` (the interaction set stays AT LEAST this long past the OPEN edge whatever the
anchor: box 6.0 s = its whole exit climb, valve 3.0 s -- added after the first Isaac campaign, section 3.2). The
state machine is `wb_grip_schedule.hpp` (`interaction`, `set_edge`, the open timer; gtest
`WbGripSchedule.TheInteractionSetCanOutliveTheOpenEdge`). The node prints both sets at startup (`TWO SETS: FREE
FLIGHT ... -> INTERACTION ...`) and logs every set change (`GRIP: parameters -> the INTERACTION set over 1.0 s`).
Debug append-only: `wb_control_debug` [126] the set in force, [127] K_y x applied, [128] omega_c_t applied,
[129] the CoM-anchor weight, [130] the anchor blend in force. The observer's filter state continues through the
bandwidth change (`omegaVec()` reads the gains every step), so no estimate jumps.

### 1.2 The fast anchor hand-back (node + planner)

Step 1's known limit (archive/valve_room_hold_guards_20261031 section 6): the leash and every guard abort hand the
claw back to the vehicle through the node's `set_ee_anchor_com`, blended over `wb_ee_anchor_blend_s` 1.0 s -- on the
10-31 evening replay a third of the run would still have reached the arm. New node service
`<ns>/whole_body_direct_actuation/set_ee_anchor_com_fast` (SetBool, true = the CoM anchor blended over
`wb_ee_anchor_fast_blend_s`, 0.25 s in both contact yamls; the free-axis weight fades at the same rate; a world
request through it is the normal one). The planner's leash release and the anchor drop of the arm / sway / descent
guard aborts call it (`requestAnchor(com, fast)`; an older node without the service gets the normal one, with a
warning). The Exit's and the release's hand-backs keep the normal 1.0 s blend: there the claw is still clamped or the
open jaws are around the fin, and a fast target move is the 10-17 crash sequence.

### 1.3 Files

- fsc_autopilot_ros2: `wb_grip_schedule.hpp`, the whole-body client (`.hpp` keys + members lists, `.cpp` banner,
  `stepGrip`, `stepEeAnchor` / `stepEeFreeAxis` via `anchorBlendS()`, the `_fast` service, debug [126..130]),
  `test_wb_parity.cpp` (+1 test); `config/*_sim_valve_turning.yaml`, `*_valve_turning.yaml`,
  `*_sim_box_pushing.yaml`, `*_box_pushing.yaml` regenerated; the raw hardware stack script (the valve and box lists
  expect the new keys; refuses a node without `TWO SETS` / `set_ee_anchor_com_fast`, a planner without the fast
  hand-back when the yaml sets `wb_ee_anchor_fast_blend_s`).
- fsc_trajectory_planner: `requestAnchor(bool com, bool fast)` + `anchor_fast_cli_`; `test_push_pull_loopback.py`
  fakes the `_fast` service and asserts the leash uses it.
- this repo: `make_valve_turning_yaml.py` (the free-flight values READ from the mirror), `make_push_pull_yaml.py`
  (the whole grip schedule for the box, same rule), `make_hw_box_pushing_yaml.py` (the real gripper's joint state),
  `utils/wb_l1_set_gains.py` (`omega_c_t_before_grip`), this folder's tools.

## 2. Checks without Isaac

| check | result |
|---|---|
| whole-body gtests (`WbGripSchedule` x2 incl. the release delay, `WbParity`, `WbL1Parity`, `WbReferenceBuilder`) | 18/18 (before and after the release-delay rebuild) |
| planner gtests `test_push_pull` / `test_pick_place` | 29/29, 32/32 |
| both hardware yamls loaded on the node at `/uav_test` (ROS domain 79) | every new key read back (valve: K_y 211.9 / omega_c_t 2.927 -> 120 / 2.0; box: 211.9 / 347.5 / 211.9, 2.927 -> 120 / 196.8 / 120, 1.0; `wb_grip_release_delay_s` 3.0 / 6.0), the `_fast` service advertised, the TWO SETS banner printed |
| planner loopback, SIM box yaml (incl. [9p]: the leash goes through the FAST service) | ALL PASS |
| planner loopback, HARDWARE box yaml | ALL PASS |
| raw hardware stack pre-launch dry run, `AM_HW_PROFILE=valve_turning` / `box_pushing` | all 96 / 82 tuned values present (incl. the new ones), build probes pass |

## 3. Isaac

### 3.1 The valve (`tools/chain_valve.sh`, scored by `tools/score_sets.py`)

Harness = Step 1's (`archive/valve_room_hold_guards_20261031`): the v2 valve at 0.072 N.m (drive model), raw mocap,
headless, the arm GS flying Ready / Turn / Exit itself, stopped 4 s after the exit; drift model
`PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,<seed>`. A = the Step 1 `rh_*` runs (the same build minus today's changes, the
same seeds, flown last night); B = today's regenerated valve yaml. The node's pane lines (`runs/node_events.log`)
confirm the sequence in every B run: `GRIP: jaws CLOSED` -> `parameters -> the INTERACTION set over 1.0 s`; at the
release `GRIP: jaws OPEN -- the interaction set stays (the claw still room-held on the handle)`; at the exit's
anchor restore `free-axis anchor released` -> `parameters -> the FREE-FLIGHT set`. Applied gains read back from
debug [127]/[128]: 212 / 2.93 in the hold, 120 / 2.00 in the turn and the exit.

| run | outcome | CoM rms Go To Start | hold max | CoM rms turn | q3 min pre-grasp / turn / release | turn | tilt |
|---|---|---|---|---|---|---|---|
| A rh_nom1 / rh_nom2 | complete x2 | 12.7 / 12.5 mm | 11 / 9 | 22.6 / 25.9 | 22.1 / 16.4 / 13.9, 23.3 / 17.4 / 13.6 | 86 / 87 deg | 4.5 / 4.4 |
| **B ts_nom1 / ts_nom2** | **complete x2** | **10.2 / 10.6** | 15 / 11 | **18.4 / 18.7** | 21.1 / 22.0 / 14.1, 20.7 / 21.7 / 22.6 | 91 / 90 | 4.0 / 4.1 |
| A rh_s2 / s3 | complete x2 | 19.1 / 17.8 | 14 / 17 | 32.3 / 29.6 | 19.2 / 12.4 / 17.1, 20.7 / 13.7 / 20.2 | 91 / 93 | 4.0 / 4.3 |
| **B ts_s2 / s3** | **complete x2** | **16.9 / 14.8** | 16 / 21 | 20.0 / 33.3 | 18.1 / 15.5 / 10.8, 20.4 / 18.4 / 14.6 | 81 / 91 | 4.1 / 4.6 |
| A on today's build, seed 2 (ts_one_s2: every free-flight key off) | complete | 19.2 | 28 | 38.2 | 20.3 / 15.4 / 13.3 | 99 | 6.0 |
| A rh_s4 / s4b / s4c | complete x3 | 19.0 / 19.3 / 18.4 | 24 / 34 / 32 | 38.7 / 31.6 / 30.7 | 18.7 / 3.0 / 15.0, 19.7 / 14.7 / 12.6, 22.0 / 16.7 / 14.1 | 90 / 85 / 88 | 5.9 / 4.8 / 4.5 |
| **B ts_s4 / s4b** | **complete x2** | **16.4 / 15.9** | 27 / 21 | 32.4 / 33.2 | 20.6 / 9.3 / 16.1, 21.0 / 7.5 / 15.9 | 90 / 88 | 3.8 / 3.7 |

- **6 of 6 unforced B runs complete** (A: 7 of 7). No gain step is visible at either switch (the CoM error and the
  joints are continuous through the 1 s blends; the turn's lead and the arm's fold are the baseline's).
- **The free-flight legs track better with the free-flight set**: Go To Start CoM rms 10.2-10.6 vs 12.5-12.7 mm
  nominal (-17 %), 14.8-16.9 vs 17.8-19.3 mm under the drift (-15 %) -- the faster observer (2.927 vs 2.0) and the
  stiffer task, as the 10-22 bench predicted. The nominal turn is better too (18.4-18.7 vs 22.6-25.9 mm); under the
  drift the turn is the same (20-33 vs 30-39 mm): the contact set IS the old set there.
- **The turn's q3 margin is unchanged**: seed 4 is still the worst (q3 9.3 / 7.5 vs 3.0 / 14.7 / 16.7 deg in A), the
  known mid-turn fold the valve records since 10-18; the two sets neither fix nor worsen it.
- The forced trips (`ts_leash`, `ts_q3`) exercised the fast hand-back in the real stack: every hand-back the leash
  or the guard abort asked for came through `set_ee_anchor_com_fast` and the CoM-anchor weight rose 0.05 -> 0.95 in
  **0.24 s** (debug [129]/[130]; the normal hand-backs of the same runs took 0.86 s). The q3 abort (forced floor 26
  deg, tripped 0.3 s after the room hold engaged, the world blend a third in) was clean: DIRECT kept, tilt 2.4 deg,
  q3 >= 23.5 deg, arm home. **The forced leash (15 mm, gate 10 mm) cycled**: under seed 2 the hold wanders 8-31 mm,
  so release -> settle -> re-engage -> release repeated 8 times in 25 s and the driver's Ready wait ran out (last
  night's slow hand-back let the same forced limits through after 2.3 s, by luck of the drift). With the shipped
  40 mm leash no unforced run released; on hardware the holds reach 20-48 mm (10-28 #2 would have released once),
  and each release costs >= 1.3 s of Ready wait -- the arm GS's automatic Ready retries for 10 s, after that the
  operator presses Ready again. Accepted: a release is not an abort.

**Valve follow-up on the rebuilt node (`wb_grip_release_delay_s` 3.0; `tools/chain_valve2.sh`): 2 of 2 complete.**
ts2_s2: Go To Start 17.3 mm, turn 32.3, q3 20.5 / 3.1 / 17.8 (the seed's known mid-turn fold: A's seed 2 read
12.4, B's 15.5 -- scatter), 79 deg, tilt 4.2; ts2_nom: 11.0 / 24.7, q3 21.4 / 21.0 / 10.9, 88 deg, tilt 4.7. The set
now releases 3.0 s after the OPEN edge (76.67 s vs the open 73.58), still inside the exit (ends 81.5).

### 3.2 The box push (`tools/box_jobs.py` -> `tools/runner.sh`; follow-up `tools/box_jobs2.py`)

Condition = the 10-25 hardware-like one (280 g, 0.40 m at 21 mm/s, the hardware guard values -- tilt 10 deg, abort
release hold 1.5 s --, the drift model per seed, the arm GS flying Ready / Push / Exit itself, the mission flown to
the landing). A = today's build with every free-flight key off (the one-set configuration every box campaign flew);
B = the regenerated box yaml. The node's pane lines confirm the same sequence as the valve's in every B run.

| run | outcome | CoM rms Go To Start | hold max | CoM rms push | CoM rms exit | q3 min pre / push / release | box | tilt (flight) |
|---|---|---|---|---|---|---|---|---|
| A tsA_nom | complete | 13.6 mm | 7 | 6.5 | 7.1 | 31.6 / 32.6 / 32.6 | 401 mm | 2.2 |
| **B tsB_nom** | complete | **8.4** | 7 | 12.1 (*) | 9.1 | 32.0 / 28.3 / 27.7 | 389 | 2.9 |
| A tsA_s2 | **ABORT at the grasp** (the claw drifted 36 -> 59 mm off the grasp point, never met the settle gate: the GS's 6 s wait, jaws open, the 10-24 safe path) | 22.9 | 24 | - | - | 32.7 / - / - | - | 9.1 |
| **B tsB_s2** | complete | **14.2** | 26 | 22.3 | 16.2 | 25.3 / 27.8 / 22.4 | 402 | 3.3 |
| **B tsB_s1** | complete | 17.3 | 12 | 24.1 | 14.1 | 31.3 / 24.2 / 30.7 | 421 | 3.8 |
| **B tsB_s3** | complete | 15.2 | 16 | 21.5 | 24.7 | 31.7 / 21.3 / 30.8 | 392 | 3.5 |
| A tsA_s5 | complete | 24.1 | 20 | 18.3 | 38.8 | 32.8 / 27.9 / 31.0 | 387 | 4.0 |
| **B tsB_s5** | **box-tilt guard abort in the EXIT** (box 7 deg, 1.4 s into the climb; DIRECT kept, tilt <= 10.3, the box stayed on the table) | **13.8** | 28 | 18.6 | - | 30.6 / 25.2 / 26.2 | 392 | 10.3 |

(*) tsB_nom's set switched 1.7 s INTO the push: Isaac's jaws creep on the box's grip shim (-0.091 rad at the GS's
close, past the -0.1 threshold 1.5 s later), so the push began on the stiff set. The threshold is now -0.05 in the
box SIM yaml (hardware: 0.0 m with open +0.019 / closed -0.0097, unaffected); tsB_s1 flew it (the set switched 0.4 s
before the close mark).

- **Free flight: the two sets are better on every pair** -- Go To Start CoM rms 8.4 vs 13.6 nominal, 14.2 vs 22.9
  (seed 2), 13.8 vs 24.1 (seed 5): -38 to -43 %, the box's contact set being the softest of all (omega_c_t 1.0,
  K_y 120). The one-set seed-2 run never reached the grasp (the drifting claw), the two-set one did.
- **Contact: the same** (push 18.6 vs 18.3 on seed 5; nominal 12.1 vs 6.5 is the mid-push switch above).
- **The exit is where the two sets cost**: tsB_s5's box tipped 7 deg 1.4 s into the exit climb, with the set blending
  back to the stiff free-flight task at that moment -- the box's release-first exit restores the CoM anchor BEFORE
  the climb, so the "free axis gone" rule released the set while the open jaws were still around the fin (the
  valve's exit holds the room anchor through 0.06 m of climb, so it never showed this). The known open-claw drag
  (10-16: 1 shipped-config run of ~20), but 1 of 4 here is more than scatter. **Fix: `wb_grip_release_delay_s`** --
  the interaction set stays at least this long past the OPEN edge (box 6.0 s = the whole exit climb, valve 3.0 s);
  gtest + the follow-up flights below.

**Follow-up on the rebuilt node (threshold -0.05, `wb_grip_release_delay_s` 6.0; the node's banner now ends
"... and at least 6.0 s past the OPEN edge"): 3 of 3 complete**, incl. seed 5:

| run | outcome | set on vs the close mark | set off | CoM rms Go To Start / push / exit | q3 min pre / push / release | box | box tilt in the release + exit | tilt |
|---|---|---|---|---|---|---|---|---|
| tsB2_nom | complete | 0.4 s before | at exit:end (87.09 / 87.1) | 8.7 / 10.6 / 11.5 | 31.8 / 28.6 / 30.0 | 397 mm | 0.3 deg | 2.8 |
| tsB2_s5 | complete | 0.9 s before | at exit:end | 13.5 / 20.1 / 13.9 | 28.0 / 27.1 / 31.8 | 408 | 0.1 | 3.5 |
| tsB2_s2 | complete | 1.0 s before | at exit:end | 13.9 / 24.1 / 34.8 | 30.2 / 23.1 / 31.6 | 378 | 0.1 | 4.1 |

The push now starts on the contact set (120 / 1.00 read back at push + 2 s), the set stays through the whole exit
(it releases at the exit's end, 6 s after the open), the box never tilts more than 0.3 deg after the push, and the
free-flight numbers are the same as the first B runs (8.7 / 13.5 / 13.9 mm vs the one-set 13.6 / 24.1 / 22.9).
**Shipped: the box and valve yamls as regenerated by the generators (the follow-up configuration).**

## 4. The guard audit on the hardware data (`tools/guard_audit.py` -> `guard_audit.txt`)

The SHIPPED guard values (read from the hardware valve / box yamls) replayed on all 19 recorded hardware contact
flights (valve 10-28 x2, 10-29, 10-31 x4; box 10-21, 10-23, 10-24 x6, 10-24 evening x2, 10-27 x2) from the signals
the planner itself uses (PX4 attitude, the arm's joint states, the law's CoM error, the raw F_hat, obj_0):

- **The replay is faithful.** On the box flights the q3 rate / box-tilt guards that actually flew reproduce the
  recorded trip times to 0.00-0.01 s (9 of 9), so the other rows can be trusted.
- **Every valve failure is caught earlier than it was**: 10-31 evening -- the leash at +1.16 s into the hold
  (recorded operator abort +2.20 s, tilt trips +3.8 s), then the q3 rate guard +1.39, the descent guard +1.44, the
  sway guard +1.60; 10-29 -- the sway guard +10.41 s, 3.0 s before the q3-rate trip that came too late; 10-31 18:04
  (the wrist swing) -- the descent guard +3.95 s, 3 s before the operator; 10-28 #1 -- the sway guard mid-turn
  (+11.68 s, the real growing sway), 3 s before the false 2 deg trip.
- **The Ready gate would have refused the 10-31 evening Ready** (68 mm, 116 mm/s against 30 / 60) and the 18:04 one
  (35 mm). On 10-28 #2 the leash would have released the room hold at +0.30 s (CoM error > 40 mm) -- that hold
  reached 48 mm; Ready then waits for the base, which is the designed cost (a release is not an abort).
- **The rotor-tilt guard at 4 deg** (was 2 on 10-28) fires only on 10-28 #2 at +7.10 s, after the leash -- the 10-28
  false trips at 2 deg are gone at 4, as that day's analysis predicted.
- **No guard fires on the two static-hold flights** (10-31 17:49 / 17:57), which were aborted by the operator for
  the wrist swing: nothing in the guards reads a wrist swing alone (q3 stayed > 3 deg). Correct: the wrist fix is
  arm-side.
- Not replayable from the bags: the box-lag / box-cross guards (need the plan stream), frozen / stale feeds (need the
  processor), the node's emergency-abort escalation.

Ordering, verified in the yamls and the stack dry runs: planner tilt guard 10 deg / 0.1 s (-> the task's configured
abort) < node soft 15 deg (-> `push_pull/emergency_abort`, SAFETY only on a refused / timed-out abort) < node hard
25 deg (-> SAFETY); q3 static 3 deg (valve) / 10 (box) with the rate guard ahead of it (look-ahead 0.1 / 0.2 s, q2 wall
45); the raw-mocap guard on both the planner and the node; the fused stack refuses both contact profiles.

## 5. Tests that isolate the problems (for the next hardware session)

Each test changes ONE thing against the flight before it and is scored by the same two numbers: the law's horizontal
CoM error (rms / max) and the arm's (q2, q3) excursion, both from `wb_control_debug` / `joint_states`
(`score_sets.py` on an Isaac npz, `guard_audit.py`'s signals on a bag). Fly them in this order; each one answers a
question the earlier flights left open.

| # | test | what it isolates | expected if the hypothesis holds |
|---|---|---|---|
| T1 | Go To Start to the valve, then HOVER above the handle with the claw CoM-ANCHORED (`push_pull_world_anchor_ready: false` for this flight), no Ready. 30 s. | the base's wander at the valve with NO arm coupling (the 20-flight baseline says 0.02-0.10 N.m of roll torque) | CoM error 15-30 mm rms, the arm within 1 deg of its pose |
| T2 | the same, with the ROOM hold (today's config), no Ready. 30 s. | the room hold's arm absorption alone: how much of the base wander the arm takes, and whether the leash fires | the arm moves ~0.8 deg per mm along the nose (10-18); the leash fires if the base passes 40 mm; no abort |
| T3 | T2 with the FREE-AXIS anchor at 0 for the hold (`wb_ee_anchor_free_axis_weight: 0.0`) | the free axis's share: the 10-28/29 valve angle followed 0.75 x the wander / r | the arm absorbs the full wander along the nose; q2/q3 swing larger than T2 |
| T4 | the Ready descent with the jaws OPEN but NOT closed (the GS Ready, no Push; `push_pull_auto_push:=false`); hold 10 s on the handle; Exit | the descent + open-jaw hold: the 10-31 failures' phase, without any contact | the base stays within the gate (30 mm / 60 mm/s); the wrist holds its heading (the 10-31 arm fixes) |
| T5 | T4 + a hand CLOSE on the fin, 10 s static hold, open, Exit (the 10-31 plan) | the grasp event: the set switch (watch `GRIP: parameters -> the INTERACTION set`) and the clamped hold with no turn | no q3 excursion at the switch; CoM error unchanged across the 1 s blend |
| T6 | the 30 deg turn (the 10-31 evening's) | the turn's lead / fold with the new sets | q3 > 10 deg through the turn; the sway guard silent |
| T7 | the 90 deg turn | the full task | -- |

If T2 fails where T1 passes, the problem is the room hold (Step 3: drop the pre-grasp room hold and descend
CoM-anchored, the 10-27 "W" result reversed). If T3 is worse than T2, keep the free axis. If T4 fails, the problem is
the arm side (heading / damping), not the base. If T5 fails at the switch, lengthen `wb_grip_blend_s`. Every flight
runs under today's guards; a guard trip ends the test with the data in the bag.

## 6. Open

- The two sets are validated in Isaac only; the first hardware flight is the static hold (T5), never the turn.
- The fast hand-back moves the EE reference by up to the base's drift in 0.25 s; validated here only with the forced
  leash / q3 trips, not against a real 22 cm run-away.
- The pick-and-drop / mail yamls do not carry the grip schedule (the set-down and drop have no room hold on a
  handle); unchanged.
