# Box pushing: mass, distance, controller, worst cases (2026-10-16)

User request: with a box whose TOTAL mass (box + handle) can be set from 250 to 600 g in
50 / 100 g steps (standard weights), find (1) the best box mass, (2) the best push
distance, (3) the best whole-body controller settings, in an independent box-pushing
yaml, and (4) simulate the worst cases that would be dangerous in the real experiment,
then design the safety triggers and the Abort. Friction: the measured table, 0.29
(static = sliding in the sim, as since 10-04). Push toward -y.

**THE ANSWER** (52 headless Isaac flights, RTF 1, raw mocap, the CAD box, real arm limits):

| question | result |
|---|---|
| 1 box mass | **500 g** total (box + handle), weighed to +-50 g -- best and most repeatable of 250-600 g; 600 g rides the joint stops |
| 2 push distance | **0.50 m** at 0.5 m / 24 s -- best of 0.30-0.70 m |
| 3 controller | the 10-09 push tune **kept** (k_R/k_w 1.6/1.2, K_y/D_y 120/20.2, omega_c_t 1.0, omega_x 2.0, u3 ff 0.1); the push pose **re-split to [0, 32, 28, 0]** (same 60 deg fold: q3 off its +50 stop); the j4 heading fix and K_y 150 flown and rejected. Sim yaml `..._sim_box_pushing.yaml` (generated) + NEW hardware `..._t650_box_pushing.yaml` (`tools/make_hw_box_pushing_yaml.py`, `AM_HW_PROFILE=box_pushing`) |
| 4 worst cases + guards | a JAMMED box made the arm buckle and the vehicle FLIP before the 5 N force guard; a box turned 180 deg was pushed OFF the table; the old ABORT LIFTED a stuck 1 kg box and the vehicle crashed. Added: a plan-time box-on-table check, box-lag / box-tilt / box-feed / q3 guards, an abort that holds the measured pose and backs off before climbing, and Go To Land Start flyable after an abort. Every worst case re-flown aborts safely; 0 false trips in 18 nominal pushes |

Earlier box-pushing work: `../push_pull_20261003/` (design, 10-04 fixes, 400 g),
`../push_pull_cad_box_20261007/` (the CAD box, 5 mm fin), `../push_pull_tune_20261009/`
(push chosen over pull, K_y 120 / D_y 20.2). Commands: Command.md section 17.

## 0. How the four questions couple

Everything scales with the sliding force F = m g mu: 0.71 N (250 g), 1.14 N (400 g),
1.71 N (600 g). F sets the claw's deflection (F / K_y), the load the observer stores and
the release returns, and on hardware the stick-slip amplitude (proportional to
m (mu_s - mu_k), which the sim hides). Against a light box: the hardware force reading's
noise floor (|F_hat| p99.9 ~1.3 N) and rotor wash before the grasp. Tipping does not
depend on the mass (mu h / (L/2) = 0.56 at the fin grasp). The distance couples through
the push TIME at a fixed speed (0.5 m / 24 s, the one stable rate): longer = more base
drift absorbed by the world-held claw and longer arm load. The controller couples through
the operating point (K_y vs deflection, the heading stiffness vs the box's friction torque
155 mm ahead of the grasp).

Hence a sequential design: mass at the 10-09 tune, distance at the chosen mass, the tune
at (mass, distance), then the winner off-nominal, then the worst cases. At least 2 runs
per point, 3 for anything shipped (the 10-09 scatter: slide residual 0.66-1.22 N on one
setting).

## 1. Setup

- The push yaml is generated (`../push_pull_20261003/tools/make_push_pull_yaml.py`). The
  mirror took the pick-and-drop j4 heading fix (wb_ky/dy_psi 2.5 / 0.93 +
  wb_l1_stuck_hold_j4) on 10-16; the generator now PINS the 10-09 values (0.2484 / 0.2903,
  stuck hold off) so the baseline is the config that flew 3/3 on 10-09, and the fix is a
  stage-3 candidate.
- `tools/fly.sh <tag> [key=val ...]`: MASS, DIST (push time at 0.5 / 24 m/s, box centred
  on the table: start y = +DIST/2), MU_S / MU_K; the 10-09 timetable with the release slot
  = push time + 3 s.
- `tools/table.py [--group]`: the 10-09 scorer + mass / distance / box travel / sideways
  excursion / yaw / time q2 spent near its stop.
- Table moved +0.075 m (10-16) and Vertical Margin 0.15 (10-16): both first flown here.

## 2. Results

Headless, RTF 1, raw mocap, the CAD box (5 mm fin + the sim's 20 mm grip shim), T650
gear, the real arm's q2 range [-20, 45] as hard stops, mu 0.29 / 0.29.

### 2.1 Stage 0 + 1: the box mass (0.5 m, the 10-09 push config) -- 500 g

14 flights, all complete, no abort. Means over the repeats (`tools/table.py --group`):

| mass | runs | slide e_imp [N rms] | release peak [N] | time q3 > 48 deg in the push [s] | q3 at the release [deg] | box yaw drift [deg] | exit drag [mm] |
|---|---|---|---|---|---|---|---|
| 250 g | 2 | 0.57 | 2.86 (1.8-4.0) | 0.0 | 49.7 | 8.3 | 0.3 |
| 400 g | 3 | 0.87 | 1.85 | 2.4 | 47.4 | 5.3 | 0 |
| 450 g | 2 | 0.70 | 1.73 | 0.8 | 49.4 | 5.7 | 0.2 |
| **500 g** | **3** | **0.58 [0.56-0.60]** | 1.65 | **0.3** | **44.7** | 6.1 | 0.4 |
| 550 g | 2 | 0.73 | 1.39 | 2.9 | 46.1 | 9.1 | 5.1 (one 10 mm) |
| 600 g | 2 | 0.93 | 1.87 | 6.0 | 48.0 | 5.0 | 0 |

- **q3's +50 deg stop is the task's binding joint margin.** With the claw world-held the
  arm absorbs the base's drift along its length: q2 and q3 swing in antiphase by
  +-9..14 deg about a loaded mean ~3 deg below / ~4 deg above the commanded
  [0, 26, 34, 0]. It was already so on 10-09 (q3 peaks 45.9-49.4 on the shipped config);
  q2's +45 stop is never near (max 39). The stop time grows at 600 g (bigger lean) and is
  scattered between 400 and 550 g -- 500 g is the one mass with all 3 runs clear.
- **500 g chosen.** Best or tied-best on every contact metric, and the most repeatable.
  Sim-blind reasons point the same way: 500 g slides at 1.42 N, above the hardware force
  reading's ~1.3 N noise floor (250 g's 0.71 N is under it, so on hardware the push would
  hardly read as a contact); it is heavy enough not to move in the rotor wash before the
  grasp (the 190 g basket moved 21 mm on 10-10); and it stays short of 600 g, where the
  sim already rides the q3 stop and a real box's stick-slip (proportional to
  m (mu_s - mu_k)) is largest. 250 g has the cleanest slide but the most box yaw and the
  largest release jolt.
- The box tracks its plan within 0..25 mm behind (never > 2 mm ahead), its yaw within
  9.4 deg, and the claw stays within 6 mm of the handle (`tools/guard_signals.py`) --
  the nominal envelope the new run-time guards are sized against (section 3).

### 2.2 Stage 4a: the worst cases with ONLY the existing guards (400 g, 0.5 m)

One run each (`tools/chain_s4a.sh`), the guards as shipped on 10-09: tilt 15 deg / 0.1 s,
the task force the law reads > 5 N / 0.3 s in contact, the node's watchdogs.
Injections: `PEGASUS_PUSH_STOPPER_Y` (new scene knob, a static block across the table),
`PEGASUS_PUSH_BOX_YAW_DEG`, box mass / friction, and the driver's new
`--inject abort|open|exit@S` (S seconds into the push). `tools/worst_case.py <tag>` prints
each run's time series.

| case | what happened | caught by | verdict |
|---|---|---|---|
| **w_stop**: box jammed after 0.2 m (stopper) | lag 29 -> 90 mm in 1.5 s, box tipping on the stopper (+28 mm), **q2 onto +45, q3 36 -> -18 deg** (elbow-singular branch) | force guard at **16.3 N, after the buckle** | **CRASH** -- flipped, 164 deg |
| **w_heavy**: 1 kg, mu 0.6 (5.9 N; tips: mu h / (L/2) = 1.16) | lag 55 mm, box tilted 7.3 deg, vehicle tilt **10 deg** | force guard at 6.1 N | survived, late; box rocked back flat |
| **w_flip**: box turned 180 deg (mocap / placement) | plan ACCEPTED, pushed toward +y, **box fell off the table end** (0.85 -> 0.08 m) | force guard at 9.9 N as it fell | **box on the floor**; vehicle ok |
| w_open: jaws open 8 s into the slide | the open fingers straddle the 5 mm fin and keep pushing; claw-handle gap 4 -> 10 mm | -- | benign (completed, 97 %) |
| w_abort: ABORT button 10 s in, under load | jaws open by +0.5 s, claw slides up the fin, climb + fold, tilt <= 3 deg | -- | **safe** |
| w_abort0: ABORT 1 s in, clamped before the slide | as above, box untouched | -- | **safe** |
| w_exit: Exit pressed mid-slide | refused: "push is in flight -- wait for it, or Abort" | -- | **safe** |
| w_stick: mu_s 0.5 / mu_k 0.29 | not flown -- the stack refused to start (my scratch build's compiler line matched its stale-planner check) | -- | re-flown in 4b |

**The mechanism that matters: a stalled box makes the ARM buckle.** With the claw clamped and
world-held, the base keeps flying the planned push while the claw stops, so the arm is
compressed along its length; q2 runs onto its +45 stop and q3 is driven through 0 into the
elbow-singular branch in under a second. The force reading stays below 5 N until the buckle
is under way -- by then no abort can save it. The BOX shows it about a second earlier:

| signal | nominal (all 16 completed pushes) | w_heavy | w_stop |
|---|---|---|---|
| box lag behind its plan | 0..25 mm (never > 2 mm ahead) | 40 mm at 7.9 s | 40 mm at 13.3 s |
| box tilt | <= 0.67 deg | 2 deg at 8.5 s | 2 deg at **13.1 s** |
| (the buckle / the trip that came) | -- | (trip 9.8 s) | (q3 < 0 at 14.4 s; trip 14.4 s) |

### 2.3 Stage 2: the push distance at 500 g -- 0.50 m

The speed held at the flown 0.5 m / 24 s (time scales with distance), the box centred on the
table (start y = +d/2; the 0.70 m push starts at 0.36 so it ends 4 cm, not 3, from the edge --
the new plan-time check's margin). The new box guards were ON from here on: 0 false trips in
6 of 6 (box lag <= 24 mm vs the 40 mm limit, box tilt nominal).

| distance | runs | travel | slide e_imp | release peak | time q3 > 48 deg | box sideways |
|---|---|---|---|---|---|---|
| 0.30 m | 2 | 96 % | 0.77 N | 1.59 N | 1.5 s | 3.7 mm |
| **0.50 m** | **3** | **98-99 %** | **0.58 N** | 1.65 N | **0.3 s** | 4.1 mm |
| 0.60 m | 2 | 98 % | 0.78 N | 1.60 N | 6.5 s | 6.2 mm |
| 0.70 m | 2 | 99 % | 0.77 N | 1.79 N | 4.5 s (0 / 9.0) | 5.1 mm |

- **0.50 m chosen**: best on the contact metrics and the q3 margin, and 13 cm of table left
  at each end of the 1 m table.
- A short push loses a visible share to the ~12 mm claw deflection (F / K_y at 500 g): 96 %.
- Longer pushes are not worse per second -- the q3 stop contact is STOCHASTIC (the 0.70 m
  pair: 0 and 9.0 s), set by the phase of the base's slow wander that the world-held claw makes
  the arm absorb -- but a longer push has more seconds to meet it, and the box wanders more
  sideways.

### 2.4 Stage 3: the controller and task settings at 500 g, 0.5 m -- the pose re-split

One change at a time against the 10-09 push tune (which stays: k_R/k_w 1.6/1.2, K_y/D_y
120/20.2, omega_c_t 1.0, omega_x 2.0 + the u3 feed-forward per block at 0.1), then the
winner combined with the one other candidate. The new box guards ON in every run (0 trips).

| setting | runs | slide e_imp | rho_UAV slide | q2 / q3 peak | time near a stop | box yaw | release peak |
|---|---|---|---|---|---|---|---|
| 10-09 push tune, pose [0, 26, 34, 0] | 3 | 0.58 N | 0.06 | 32.6 / 48.5 deg | 0.3 s | 6.1 deg | 1.65 N |
| **pose [0, 32, 28, 0]** (P) | **3** | **0.57** | **0.05** | 39.6 / **43.1** | **0.0** | **5.1** | 1.84 |
| P + the j4 heading fix (H) | 2 | 0.70 | 0.07 | 40.3 / 45.0 (one 43.2 / 49.1) | 0.2 | 5.9 | 1.25 |
| H alone (wb_ky/dy_psi 2.5 / 0.93 + j4 stuck hold) | 1 (+1 lost) | 0.62 | 0.08 | 32.0 / 47.4 | 0.0 | 7.2 | 2.41 |
| K_y / D_y 150 / 22.6 | 2 | 0.77 | 0.07 | 35.7 / 48.3 | 0.2 | 6.9 | 2.26 |

- **SHIPPED: the pose re-split [0, 32, 28, 0]** -- the same 60 deg fold (the claw's pitch
  is unchanged), 6 deg moved from q3 to q2. It re-centres the arm's loaded antiphase swing
  between the two stops it was riding: q3 now keeps 6-8 deg to +50, q2 4-6 deg to +45, and
  in 3 of 3 runs neither came within 2 deg (was 0.3 s per push at 500 g, 2.4 s at 400 g,
  6.0 s at 600 g). Everything else unchanged or slightly better.
- **The j4 heading fix is NOT adopted for the push.** It was built for the EMPTY wrist's
  stick-slip hunting in free flight (pick-and-drop); in contact the box holds the wrist, and
  here it gives back the joint margin the pose bought and worsens the slide. Its one gain,
  a lower release peak, is far under the 5 N guard either way. The push yaml pins the 10-09
  heading (0.2484 / 0.2903, no stuck hold).
- **K_y 150 is worse** on every contact metric, as K_y 300 / 211.9 were on 10-09: 120 stays.
- One H run never flew (the arm stack did not come up: "arm joint states stale") -- a
  start-up glitch, not a setting effect.

## 3. The safety guards (stage 4) -- what was added, and the re-flown worst cases

### 3.1 What was added (fsc_trajectory_planner, all default OFF in code, ON in both box-pushing yamls)

| guard | key (value) | trips when | justified by |
|---|---|---|---|
| box stays on the table (PLAN time) | `push_pull_box_half_length` 0.12, `_box_edge_margin` 0.03 | the PUSHED box would end within 3 cm of the table's edge | w_flip: a box (or mocap body) turned 180 deg was pushed off the end and fell; also a too-long typed distance |
| box lag | `push_pull_guard_box_lag` 0.040 | the box > 40 mm behind (jammed / too heavy) or ahead of (running away / falling off) its plan, 0.2 s | w_stop / w_heavy: the box stalls ~1 s before the arm buckles; nominal 0..25 mm |
| box tilt | `push_pull_guard_box_tilt_deg` 2.0 | the box tilted > 2 deg from its pose on the handle, 0.2 s | the stalled box TIPS (mu h / (L/2) near 1); nominal <= 0.7 deg |
| box feed lost | `push_pull_guard_box_stale_s` 0.5 | no box mocap sample for 0.5 s while the two above are armed | they are blind without it |
| arm buckling | `push_pull_guard_q3_min_deg` 10 | q3 < 10 deg on the handle / pushing, 0.2 s | the buckle itself, whatever its cause; nominal >= 12 deg (500 g at the final pose) |
| abort holds the MEASURED pose | `push_pull_abort_from_measured` true | (every abort with the claw on the box) | g_heavy / g_stop: holding the cut reference (40+ mm ahead of a stuck claw) dragged the claw along the fin during the release and wedged it in the open jaw |
| abort backs off first | `push_pull_abort_back` 0.03, `_abort_release_wait_s` 0.5 -> 1.0 | after the release, 3 cm against the push, then the climb | the open claw climbing straight up LIFTED a stuck box (58 mm with 1 kg) |

All trip into the existing ABORT (gripper open, contact off, release hold, [back off], climb
0.30 m, arm home) and name themselves in the status: `ABORTING (SAFETY GUARD: <reason>)`.
The planner's `push_pull/arrival_error` gained [11] the box lag [m] and [12] the box tilt
[deg]; the arm GS's Box Pushing tab shows both on its contact line next to |F_hat|. The
existing guards stay: tilt 15 deg / 0.1 s, task force 5 N / 0.3 s, the node's watchdogs.
The ABORT button was already there (red, the tab's top row) and flew safely in every case
it was pressed in (sections 2.2, 3.2). Exit is refused while the push flies.

### 3.2 Stage 4b: the worst cases again at the FINAL configuration, guards ON (500 g, 0.5 m)

| case | 4a (old guards) | 4b (new guards, OLD abort) |
|---|---|---|
| box jammed after 0.2 m (stopper) | force guard at 16.3 N after the arm buckled -- **flipped** | **box-tilt guard** at 13.3 s, arm normal (q3 >= 25.6), tilt <= 6.5 deg; the climb lifted the tipped box 26 mm, it dropped back flat -- **survived** |
| 1 kg, mu 0.6 (tips) | force guard 6.1 N, tilt 10 deg, survived | **box-lag guard** at 8.1 s (46 mm), 1.7 s earlier -- but the climbing open claw **LIFTED the box 58 mm**, it dropped off, the vehicle passed the node's 15 deg watchdog and **crashed** in SAFETY -> the abort fix (3.1), stage 4c |
| box turned 180 deg | pushed off the table, fell to the floor | **plan refused** before take-off: "the box would end at (1.275, 0.750) ... Is the box's yaw right?" |
| typed distance 0.90 m | (not flown) | **plan refused** |
| mu_s 0.5 / mu_k 0.29 | (not flown: the stack refused to start) | completed, no trip (break-away 2.45 N, q3 47.3 deg); without the new guards also completed -- not a dangerous case at this ratio |
| ABORT button 10 s in | safe | safe (box flat, tilt <= 2.5 deg) |
| 400 g (-100 g) | -- | completed, q3 <= 42.7 deg (at the OLD pose 400 g rode q3's stop 3/3) |
| 600 g (+100 g) | -- | completed, but q2 0.5 s / q3 0.3 s within 2 deg of their stops, release 3.4 N: **the edge** -- weigh the box to 500 +- 50 g |
| mu 0.35 (+20 %) | -- | completed, q3 <= 43.8 deg, release 1.5 N |

The guards never tripped in a nominal push: 0 false trips in the 18 completed pushes flown
with them active (stages 2, 3 and the 4b robustness runs; s3_H_2 never flew).

### 3.3 Stage 4c: the abort fixed (measured-pose hold + back off + 1.0 s release)

| run | trip | tilt before the trip | tilt in the abort | box lifted | q3 min |
|---|---|---|---|---|---|
| h_heavy (1 kg, mu 0.6) | box lag, 8.1 s | 7.5 deg | 7.4 deg | 10 mm (4b: **58 mm, crash**) | 15.5 deg |
| h_heavy2 (repeat) | box lag, 8.1 s | 8.0 | 8.6 | 11 mm | 20.2 |
| h_stop (jammed) | box tilt, 13.3 s | 6.9 | 8.9 | 20 mm (4b: 26 mm) | 17.3 |
| h_abort (ABORT button, sliding box) | 10.0 s | 2.9 | 2.3 | 0 mm | 17.6 |

All four safe. The abort's message now says what it did: "ABORT: contact off, release back then
up 0.30 m and the arm home, from the MEASURED pose (back 0.03 m against the push first)".

**LEAVING THE TABLE AFTER AN ABORT -- a procedural trap, fixed.** The abort leaves the vehicle
0.30 m above the push point, BESIDE the table. The test driver's old landing descended in place
and came down on the table / box (h_heavy: 20.6 deg on the way down). After an abort that
touched the box the planner had cleared its plan, so Go To Land Start (its table-checked path
off the table) was unavailable until Get + Plan -- and re-planning a box already pushed part-way
can be REFUSED by the new box-on-table check (h_stop), leaving no planned way off. The planner
now keeps the cleared plan's Land Start / Land for exactly that (info [3] = 1, "go_to_land_start
to leave the table" in the ABORTED status, the arm GS enables the two buttons on it).
**Operator rule: after an ABORT, press Go To Land Start, then Land -- never land in place
beside the table.**

### 3.4 Stage 4d: leaving the table, end to end

i_stop (jammed, box-tilt guard) and i_abort (ABORT button under load), both with the planner
that keeps Go To Land Start flyable: Go To Land Start accepted DIRECTLY after the abort (no Get /
Plan), flown off the table to Land Start (23.9 s), landed there. Max tilt after the trip to the
end of the run 6.9 / 4.3 deg; the box lifted 17 / 0 mm and set back flat.

## 4. Not covered -- say it rather than imply it

- **Stick-slip.** The sim box slides with static = sliding friction; a real box whose break-away
  is well above its sliding friction stores more load and releases it in jumps (the 10-04 lesson:
  give the real box a PTFE / felt base). mu_s 0.5 / mu_k 0.29 was flown and is not dangerous here.
- **The box-mocap feed guard** (`push_pull_guard_box_stale_s`) is code-reviewed, not flown: the
  Isaac emulator cannot drop obj_0 alone. A FROZEN feed (the 0924 Motive failure republished one
  pose) shows as a growing box lag and trips that guard instead.
- **Hardware.** Nothing flown. The hardware box-pushing yaml carries the SIMULATION's table
  (`push_pull_table`) -- measure the lab table in the mocap frame and set it in
  `make_hw_box_pushing_yaml.py` before the first flight; the stack prints it. obj_0 must be
  defined like the sim's (origin at the box centre, +y along the 240 mm side, z up).
- **The sim claw** cannot close below ~19 mm, so the 5 mm fin carries an invisible 20 mm grip
  shim; the real foam pads close fully -- the grasp and the release are the sim's, not the
  hardware's. Check on the bench that the pads hold ~2 N along the fin.
- One run per worst case in 4a/4b, two in 4c for the heavy box: a stability boundary would need
  more (the 2026-08-10 scatter lesson); none of the guard trips was near one.

## 5. What changed where (nothing committed)

- **fsc_trajectory_planner**: the box-on-table plan check (`push_pull.{hpp,cpp}`,
  `push_pull_box_half_length` / `_box_edge_margin`), the box guards and the q3 guard (node:
  `push_pull_guard_box_lag` / `_box_tilt_deg` / `_box_persist_s` / `_box_stale_s` / `_q3_min_deg`,
  `push_pull/arrival_error` [11] / [12]), the abort on the box (`push_pull_abort_from_measured`,
  `push_pull_abort_back`; `planPushPullAbort`'s retreat), Go To Land Start / Land after an abort
  (info [3]); gtests `ThePushedBoxMustStayOnTheTable`, `AbortRetreatsHorizontallyBeforeTheClimb`
  (push 14/14; the box-point loopback PASS). All default OFF -- every other yaml unchanged.
- **utils_custom_ground_station**: the Box Pushing tab's contact line shows the box lag / tilt;
  Go To Land Start / Land enabled after an abort (planner info [3]). `gs_box_check.sh` PASS.
- **fsc_autopilot_ros2**: `..._sim_box_pushing.yaml` regenerated; NEW `..._t650_box_pushing.yaml`;
  both whole-body hardware stacks: `AM_HW_PROFILE=box_pushing`, its own tuned-law check (34/34),
  a refusal if the installed planner predates the box guards, the sim push identity and the 2.2 m
  fence as sim-only values, the table reminder (dry-run checked, free_flight unchanged 31/31).
- **Pegasus**: `make_push_pull_yaml.py` (heading pinned, pose re-split, guards, abort keys);
  the scene's `PEGASUS_PUSH_STOPPER_Y` knob and 500 g default (+ both wrappers' banner);
  `pl_mission.py` `--inject`, prompt end on a refused plan, Go To Land Start after an abort,
  arrival [11] / [12] recorded; `wb_l1_set_gains.py` knows the stuck-hold keys; this campaign's
  `tools/` (fly.sh, table.py, guard_signals.py, worst_case.py, the chains,
  make_hw_box_pushing_yaml.py). Command.md section 17 updated.
- **On the Orin**: pull, rebuild `fsc_trajectory_planner`; on the laptop rebuild
  `utils_custom_ground_station`. The hardware stack refuses a planner without the box guards.
