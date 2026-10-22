# Box pushing: contact-phase stability tuning (Ready To Push → Push → Exit To Push)

The plan is `PLAN.md` (2026-10-21, the planning session). This README is the campaign record,
filled as the phases run (2026-10-22 night, the executing session).

**Result in one line:** one change shipped, `wb_ee_anchor_free_axis_on_world: true` (the free-axis anchor
latched from the claw's room hold, not only from the grasp); in a hardware-like Isaac condition it turns
the old config's 4/8 with four crashes into 6/8 with none. No gain changed.

**How execution departed from PLAN.md:** the planned "hardware-like Isaac" (band-limited mocap velocity
noise, section 2) turned out to reproduce the 10-21 flight by the wrong mechanism, so the condition was
re-derived from the bags (PX4's attitude-estimate error → a slow plant force); T3 (the automatic-Ready
delay) was dropped because it only moves the hover ABOVE the handle, not the room hold; T4 was built as
the node option T4-A; all Isaac flights ran through a job queue (`tools/runner.sh`, `queue/done/`).

## 0. Code check of the 2026-10-21 evening implementation (before any tuning flight)

The user asked first to check that the evening's uncommitted work is correct: the planner's box
sample gate, abort unbuckle, box-lag guard against the law's free-axis target and T_max refusal
(fsc_trajectory_planner); the arm GS's automatic Ready / Push and the 0.2 s automatic Exit
(fsc_open_manipulator `push_pull_panel.*`); the estimator's velocity hold-off after a mocap pause
(fsc_drone_state_estimator_ros2 `mocap_odom_bridge`); the box yamls, the generator, the hardware
stack checks and the sim driver (`pl_mission.py --gs-auto / --gs-exit`).

Read every diff; nothing found that needs a change. Checks run (every binary newer than its sources):

| check | result |
|---|---|
| planner gtests `test_push_pull` / `test_pick_place` / kinematics / flat / EE / teleop | 20/20, 30/30, 4, 2, 5, 9 pass; `StraightLine.ParityWithPython` fails as before (fixture deleted 09-17) |
| estimator gtest `test_mocap_guard` | 10/10 |
| planner loopback `test_push_pull_loopback.py`, SIM box yaml | ALL PASS, incl. [9f] free-axis shift (30 mm passes, 60 mm aborts naming the shift), [9g] box gate (two 0.3 s mis-solve runs dropped, a jumping feed aborts as JUMPING), [9h] unbuckle (q2/q3 44.9/5.0 → 32/28 after the release wait, claw +9.6 mm, then climb and home) |
| the same, HARDWARE box yaml | ALL PASS (release wait 1.5 s seen in [9h]: held 1.48 s) |
| `test_push_pull_box_point_loopback.py` | PASS (frozen feed and a flipping yaw refused at Get) |
| arm GS offscreen harness `gs_grasp_check.sh` (real panel, fake planner) | PASS, 55/55 cases incl. G1–G5 automatic Ready / Push |
| estimator loopback `estimator_loopback.py` (7 cases incl. the second fault during the blend back) | ALL PASS |
| NEW `tools/vel_holdoff_check.py` (the hold-off had no test): mocap twist = the processor's finite difference, 90 / 30 ms pauses | ALL PASS: the 2.5 / 1.8 m/s spike never reaches the odometry (peak 0.22 / 0.38 m/s = the true motion, within 7 / 1 mm/s of EKF2), and 1.2 s later the twist is the processor's again, bit-exact |

## 1. Additions made for this campaign (all default off)

- **Emulator `velocity_noise_bw_hz`** (isaacsim_optitrack_ros2_emulator node + launch, and the
  4-D raw stack's `sim_feedback_vel_noise_bw_hz` pass-through): 0 = white per sample as before;
  > 0 = the velocity noise first-order band-limited at that corner with the SAME std (exact AR(1)).
  Hardware's raw-mocap velocity error is ~4.5 cm/s below ~5 Hz; Isaac injected 2.2 cm/s white.
- **Whole-body node `wb_ee_anchor_free_axis_on_world`** (client only, parity untouched): latch the
  free-axis anchor as soon as the EE anchor is the WORLD, not only once the contact flag is on —
  PLAN.md T4-A (the room hold with the jaws still open).
- Tools: `tools/fly.sh` (raw yaml keys + `NOISE` / `NOISE_BW`), `tools/runner.sh` (job queue),
  `tools/phase_metrics.py` (the acceptance table), `tools/calib.py` (Isaac vs the 10-21 flight),
  `tools/bench_sweep.py` (Tier 3 on the room-held bench).

## 2. What drives the hardware's wander — and why the 10-21 gain tune's model was wrong

**The 10-21 bench calibration assumed the swing comes from raw-mocap velocity NOISE** (4.5 cm/s,
band-limited at 5 Hz). Measured on the 10-21 bag (`tools/vel_sources.py`) against a smoothed
derivative of the mocap position:

| velocity the law could use | error below 2 Hz (xy) | broadband (xy) |
|---|---|---|
| raw mocap (the processor's finite difference; what flies) | **2.2–4.3 mm/s** | 41–48 mm/s |
| EKF2 (`vehicle_odometry`) | 8–15 mm/s | 10–16 mm/s |

The raw velocity's noise is high-frequency (blue, from differentiating position); in the band of the
wander (0.1–0.6 Hz) it is tiny. Isaac's default white 2.2 cm/s at 120 Hz already has ~4 mm/s there.
**PX4's attitude estimate is the low-frequency source** (`tools/att_err.py`): against the mocap
attitude (mount offset removed) its tilt error is 0.42–0.56° rms at 0.1–0.6 Hz, i.e. a thrust of
37 N misdirected by 0.27–0.37 N — exactly the law's own disturbance-estimate variation on the same
windows (`tools/dist_level.py`: d_hat_t 0.25–0.38 N, CoM error 14–26 mm).

**The calibration consequences:**
- Isaac with 4.5 cm/s band-limited velocity noise (the first condition flown tonight, runs
  `cal45_*`, `p1_*`, `p2_*`) matched the hardware's wander AMPLITUDE (hover 10 mm, world-held
  9–16 mm, d_hat 0.22–0.28 N) by the wrong mechanism, ~2–3x faster. Kept as data, not as the verdict.
- Bench with the measured attitude error (`circle_bench.py` copy: a slow AR(1) world tilt error on
  the law's attitude, 0.35°/axis at 0.2 Hz, plus broadband velocity noise) reproduces the hardware
  with no fit beyond the measurement: hover CoM 15–18 mm (hw 14–19), room-held swing 5–8° with q3 to
  7° at anchor 0 (hw 10°, q3 5°). Broadband velocity noise alone: 6 mm, 2.5°. (`bench_calib.py`)
- Isaac gets the same mechanism as a slow random HORIZONTAL FORCE on the body (new default-off
  scene knob `PEGASUS_PLANT_FORCE_NOISE` = yaml `sim_plant_force_noise_n/_bw_hz/_seed`): at
  0.23 N/axis @ 0.2 Hz the anchor-0 run gives hover 14 mm, swing 6.7° (~70 % of the flight); the
  pairs fly 0.32 N/axis @ 0.12 Hz to be at least as harsh.

**The ranking flips with the model** (bench, 3 seeds, `bench_t3.txt` / `bench_kv.txt` vs `bench_att.txt`):

| | velocity-noise model | attitude-error model (hardware-calibrated) |
|---|---|---|
| shipped, free axis 0.75: swing / CoM | 2.67° / 18.6–22.6 mm | 2.08° / 10.9–15.6 mm |
| omega_c_t 2.0 | (1.5: 2.80°, worse) | **1.66° / 8.4–12.0 mm** |
| omega_c_t 2.927 | — | 1.47° / 7.2–10.2 mm |
| shipped, free axis 0 (Ready today) | 0/3 seeds pass | 2/3 pass, 6.2° |
| omega_c_t 2.0, free axis 0 | — | 3/3 pass, 4.7° |
| any other gain (D_y, K_y, k_v, k_w, omega_c_q/r, omega_x) | within 4 % | within 2 % |
| delay margin | 32 ms | 32 ms (unchanged by omega_c_t) |

Under noise, a faster observer passes more noise; under a slow real force (what the attitude error
is), it rejects more of it. The hardware data supports the second.

## 3. Isaac flights (240 g, 0.10 m, 21 mm/s, hardware guard values, the arm GS running Ready / Push / Exit)

Scored by `tools/phase_metrics.py` (definitions in its header); all CSVs beside this README.

### 3.1 Calibration: the anchor-OFF config must fail like the 10-21 flight

| run | condition | outcome | swing (q2-q3)/2, world-held | hover CoM wander | d_hat_t |
|---|---|---|---|---|---|
| hardware `_170123` | — | q3-guard abort 0.78 s after the push | 10.2° @0.54 Hz | 18.9 mm @0.12 Hz | 0.30–0.38 N |
| `cal45_1/2` | 4.5 cm/s velocity noise, band-limited 5 Hz | 2 aborts (q3 guard in Ready; then the node left DIRECT 0.44 s into the abort and the vehicle crashed) | 4–9° growing | 10 mm @0.4 Hz | 0.22 N |
| `f_cal0_1` | plant force 0.23 N/axis @0.2 Hz | complete, q3 ≥ 14.8 | 6.7° | 14.1 mm @0.25 Hz | 0.15–0.21 N |
| **`g_cal0_1`** | **plant force 0.32 N/axis @0.12 Hz** | **q3-guard abort in the release** (q3 −2.8° → −71°; node left DIRECT 0.25 s into the abort, dropped 0.5 m, SAFETY recovered) | **9.1° @0.11 Hz** | **20.3 mm @0.12 Hz** | **0.22–0.35 N** |

`g_*` = THE hardware-like condition for everything below (Isaac's default white 2.2 cm/s mocap
velocity noise, which matches the hardware below 2 Hz, plus the force).

### 3.2 The four configs, paired force seeds 1–3 (12 flights, all complete)

| config | Ready q3 min per seed | Ready swing p-p, mean | push q3 min (all seeds) | push q2 max | slide e_imp mean | CoM err mean | box back in settle | box travel |
|---|---|---|---|---|---|---|---|---|
| shipped | 28.6 / **11.9** / 22.4 | 9.7° | **12.5** | 42.7 | 1.90 N | 26.0 mm | 0–31 mm | 95–103 |
| free axis on world hold (`wb_ee_anchor_free_axis_on_world`) | 30.6 / 24.0 / 25.7 | **3.0°** | 21.4 | 38.9 | 1.37 N | 21.6 mm | 0–16 mm | 105–121 |
| omega_c_t 2.0 | 29.3 / 21.5 / 21.3 | 9.9° | 19.8 | 38.7 | 1.57 N | 20.8 mm | 1–26 mm | 95–112 |
| both | 30.8 / 26.2 / 26.3 | 3.6° | 19.7 | 37.8 | 1.36 N | 20.1 mm | 0–20 mm | 104–111 |

- **The free axis from the world hold is the structural fix of the Ready window** (the hard seed 2:
  q3 11.9 → 24.0, one degree off the guard on the shipped config). Cost: the box rides the drift
  from the grasp on, travel up to +21 mm on 100.
- **omega_c_t 2.0 is inside the scatter in Isaac with the box** (bench: −25 % CoM wander; Isaac:
  −7 %). On top of the free axis it adds ~2° of Ready margin and loses ~2° in the push.
- Every release window held q3 ≥ 20° in all four; the post-push box drag stayed ≤ 9 mm.

### 3.3 The sensor-noise model flights (kept as data, not the verdict)

`p1_ship_1..3` (shipped) 3/3 complete but q2 ON its 45° limit in Ready every time and q3 down to
7.9–14.9°; `p2_fw_1..3` 2/3 (one q3-guard abort in the release); box drag after the push 17–52 mm.
A harsher, faster wander than the hardware's — the ordering agrees with 3.2.

### 3.4 Everything flown under the hardware-like condition, pooled (`summary.txt`)

Worst q3 / q2 over every seed per window (guard 10°, real q2 limit 45°); swing, residual and CoM
error are means over the completed flights. 0.10 m flights at 240 g, 0.50 m at 500 g (24 s slide).

| configuration | done | Ready q3 | push q3 | release q3 | q2 max | Ready swing p-p | slide e_imp | failures |
|---|---|---|---|---|---|---|---|---|
| shipped (anchor 0.75 from the grasp) | 4/5 | **11.9** | 12.5 | 21.8 | 42.7 | 10.8° | 1.79 N | formal seed 5: buckled on arrival, left DIRECT at 22° tilt |
| **+ free axis from the room hold** | **5/5** | **24.0** | 15.3 | 21.3 | 40.7 | **3.4°** | 1.51 N | — |
| + omega_c_t 2.0 | 3/3 | 21.3 | 19.8 | 20.3 | 38.7 | 9.9° | 1.57 N | — |
| free axis + omega_c_t 2.0 | 5/5 | 25.3 | **11.1** | 22.4 | 42.9 | 4.0° | 1.59 N | — |
| free axis + settle 1.0 s | 5/5 | 21.6 | 14.2 | 20.7 | 42.1 | 5.1° | **1.39 N** | — |
| free axis + exit 1.5 s | 4/5 | 22.7 | 13.0 | 5.9 | 44.1 | 3.3° | 1.59 N | q3 guard in the release window (ended safely) |
| free axis + weight 0.9 | 4/5 | 23.7 | 21.4 | 27.0 | 36.0 | 4.1° | 2.00 N | the claw never met the 20 mm close gate in 15 s |
| free axis + settle 1.0 + blend 0.5 s | 4/5 | 20.1 | 15.6 | 26.3 | 40.5 | 4.8° | 1.57 N | the same close-gate timeout |
| anchor OFF (the 10-21 hardware config) | 0/1 | 10.6 | 17.3 | −71.5 | 45.0 | — | — | q3 guard, left DIRECT, dropped 0.5 m |

### 3.5 Robustness of the candidate (free axis from the room hold + 1.0 s settle; `aborts.txt`)

| case | outcome | after the trip |
|---|---|---|
| stopper 0.2 m into a 500 g push | box-lag guard at 13.4 s (51 mm behind, the law's free-axis shift counted) | DIRECT kept, tilt ≤ 9.9°, q3 ≥ 22.9; the box rocked 12° and fell back flat (as on 10-21) |
| 1 kg, μ 0.6 | tilt guard at 8.8 s (10.6°) | DIRECT kept, tilt ≤ 9.6°, q3 ≥ 17.3, q2 touched 45 |
| side-edge fall (box at 15°, edge check off) | box-tilt guard at 15.6 s (2.5° > 2°) | DIRECT kept, tilt ≤ 3.2°; the released box fell off the table (the 10-19 outcome) |
| mocap freeze 3 s mid-push | guard CONFIRMED → abort | DIRECT kept, tilt ≤ 13.4°, box rocked 11° and stayed |
| μ 0.20 / 0.30 | complete | q3 ≥ 25.8 / 22.3 |

Every abort with the free axis on stayed in DIRECT with the arm off the elbow branch (also the one
release-window q3 trip: q3 5.9°, tilt 2.9°). The anchor-off aborts (`g_cal0_1`, `cal45_2`) left
DIRECT 0.25–0.44 s into the abort and dropped / crashed: when the arm is already collapsing
fast, the 10° q3 guard fires with q3 overshooting to −50..−70°.

### 3.6 The decisive pairing: three FRESH seeds, the shipped config vs the new yaml as generated (`aborts_confirmation.txt`)

Flown after the decision, from the regenerated yaml (no per-flight override), then the shipped config
on the same force seeds:

| seed | task | shipped (free axis from the grasp) | new (free axis from the room hold) |
|---|---|---|---|
| 6 | 500 g, 0.50 m | **left DIRECT** in Ready, q3 to −54.5°, tilt 33.7° | complete (Ready q3 22.9, push 21.9) |
| 7 | 240 g, 0.10 m | **left DIRECT** in Ready, q3 to −51.1°, tilt 32.5° | q3-guard abort in Ready, DIRECT kept, tilt ≤ 4.0° |
| 8 | 240 g, 0.10 m | q3 guard, then **left DIRECT**, q3 to −60.4°, tilt 47.6° | no grasp (the push never started), tilt ≤ 2.6° |

**Over force seeds 1–8: shipped 4/8 complete with 4 crashes; new 6/8 complete, 0 crashes.**
The new config's two failures are what remains under large drift, both safe:
- seed 7: the Ready descent ended with the claw 76 mm from the grasp point and 27 mm across the fin;
  engaging the room hold that far off made the arm pull the base further away (CoM error 53 → 98 mm).
  The planner engages the room hold on arrival whatever the claw's error — a gate there is a
  planner item, not a gain.
- seed 8: the gripper closed 18 mm from the grasp point, then a 40–60 mm forward drift carried the
  open claw (75 % of it) along the fin and SHOVED the box 91 mm before the jaws closed on nothing.
  The free axis's price under large drift: the room-held claw (shipped) would have buckled instead.
- Hover tails of the force model match the hardware (rms 19 / p95 33 / max 53 mm over 48 x 10 s vs
  the flight's 15–24 / 26–31 / 35–45 in 24 s; `tools/tails.py`), so these are not artefacts of an
  over-harsh model.

## 4. Decision

**Shipped: `wb_ee_anchor_free_axis_on_world: true`** in both box-pushing yamls (generator
`../push_pull_20261003/tools/make_push_pull_yaml.py` ADDITIONS; sim + hardware regenerated, `--check`
UP TO DATE; the coordinated-admittance and valve yamls unchanged). Nothing else changes: the free-axis
weight stays 0.75, the blend 1.0 s, all gains as before. Both whole-body hardware stacks expect it
and REFUSE a node build without the key (rclcpp would silently ignore it).

**Not shipped, with the reason:**
- **omega_c_t 2.0.** The bench under the hardware-calibrated model favours it (−25 % CoM wander,
  the same delay margin), but in Isaac with the box it is inside the scatter and gave the worst
  long-slide q3 (11.1°). Worth a hardware A/B later; not a default.
- **Settle 1.5 → 1.0 s.** 5/5, no regression, smaller settle swing (1.6 vs 3.4° mean) and less box
  pushed back (2 vs 9 mm), but not beyond 1.5× the pooled scatter — the plan's rule. One yaml line
  if wanted (`push_pull_push_settle_s`).
- **Weight 0.9, blend 0.5 s.** More arm margin, but the claw then rides the drift along the fin
  far enough to miss the 20 mm close gate (1 of 5 each), and the push travel spreads (75–125 mm).
- **Exit 1.5 s.** Less box drag after the push on the long task (7–17 vs 20–49 mm), but its one
  failure and a 14° release dip were in the release window, before the exit even starts.
- **Every other gain** (D_y, K_y, k_v, k_w, k_x, omega_c_q/r, omega_x): within 2–4 % on the bench
  under either disturbance model.

**What is left on hardware:** the release window (push end → jaws open, 0.5–1.1 s) is the one
remaining exposure — the clamped arm absorbs vertical and lateral drift there (the free axis only
covers the nose). Today's 0.2 s automatic exit is right; a real gripper slower than Isaac's
0.7 s open widens it. And the q3 guard is too slow for a FAST collapse (the anchor-off aborts):
a rate term in that guard is the next safety item, not a gain.

## 5. Deployment (nothing committed)

- **Orin: rebuild `fsc_autopilot_ros2`** (the node's new `wb_ee_anchor_free_axis_on_world`); the
  stack refuses an older node. Tonight's other items (estimator hold-off, planner gate / unbuckle,
  arm GS automation) already required rebuilds of the estimator, the planner and the arm GS.
- Sim-only additions, default off (nothing else changes): the emulator's `velocity_noise_bw_hz`,
  the scene's `PEGASUS_PLANT_FORCE_NOISE` (`sim_plant_force_noise_n/_bw_hz/_seed`).
- First hardware flight: the 10-21 task (240 g, 0.10 m). The ladder stands (240 g / 0.25–0.30 m →
  500 g / 0.10 m → 500 g / 0.50 m); in Isaac's hardware-like condition the shipped change's
  completed flights kept q3 ≥ 22.9 in Ready, ≥ 15.3 in the slide, ≥ 21.3 in the release — move up
  a step only after a flight inside that envelope with the normal Exit.
- The robustness cases (3.5) were flown with the 1.0 s settle on top; the settle does not touch the
  guards or the abort, and the shipped change flew the confirmation seeds (3.6) on its own.

## 6. A rate-aware q3 guard, replayed offline on these runs (2026-10-22, follow-up question)

`tools/q3_rate_guard_replay.py` applies the planner's q3 guard (10 Hz tick, q3 < 10° for 0.2 s) and a
proposed one (every joint sample: trip when q3 + 0.2 s × q3_dot < 10° while q3 falls faster than 5°/s,
for 50 ms; the old check kept) to the recorded joint states, plus a hardware-like copy of the signal
(encoder-quantised, 100 Hz, +30 ms velocity lag), which gives the same trips within ~0.1 s.

- **The folds are fast**: 51–141°/s in the five runs the guard caught, 259–356°/s in the three crashes
  (h_ship_5, z_ship_6/7). In those three, today's guard would fire only AFTER the node had left DIRECT
  (q3 −45..−49°); the rate rule fires 0.25–0.32 s before, at q3 15–16°. In the five caught runs it
  fires 0.24–0.35 s before today's guard, at q3 12–21° instead of −7..+6°.
- **Cost**: 7 of 46 completed runs would have aborted at a 0.2 s look-ahead, 3–4 at 0.1 s — all in the
  older configs or candidates, none in the shipped config's 9 completed runs. One (j_fwx15_3) was a real
  79°/s dip during the jaws' opening that bottomed at 14° and recovered.
- **Not tested**: whether the abort can stop a fold already moving at ~300°/s. Its first action, the
  claw back to the CoM anchor, blends over `wb_ee_anchor_blend_s` (1.0 s), so for the first 0.3 s the
  arm keeps absorbing most of the drift. The anchor-off config (crashes reliably here) is the test case.

**APPLIED the same morning (user request).** fsc_trajectory_planner: `plQ3RateGuard` runs on every arm joint
sample in `onJointStates` (the sample's header stamp when it has one, else receive time; the rate a finite
difference low-passed over `push_pull_guard_q3_rate_filter_s` 0.02 s, restarted after a 0.1 s gap) and calls
the existing `onPlAbort`, armed exactly when the static q3 check is. Keys `push_pull_guard_q3_lookahead_s`
(0 = off), `_rate_min_deg_s` 5, `_rate_persist_s` 0.05, `_rate_filter_s` 0.02 -- gate-only (a change keeps the
plan). On in the three box-pushing yamls through `make_push_pull_yaml.py` (diff = the new block only; valve
yamls unchanged). Both whole-body hardware stacks expect 0.2 and refuse a planner without the key
(negative-tested); the Isaac scene's planner check compares it. Verified: planner gtests as before (only the
pre-existing `StraightLine.ParityWithPython`), the push-pull loopback ALL PASS on the sim and hardware yamls
with a new step [9i] (a 3 deg/s drift to 14 deg and a 60 deg/s dip to 25 deg abort nothing; a 150 deg/s fold
aborts at q3 15.4 / 16.7 deg), the box-point loopback PASS. **Still open:** the abort's first action, the
claw back to the CoM anchor, blends over 1.0 s; whether it stops a ~300 deg/s fold is untested. Not flown.

**ISAAC CHECK, the same morning (a quick one, user request; `tools/validate_q3_rate.py`).** Four headless
flights with the hardware-like force model, the guard at 0.2 s, against last night's flights of the same
config and seed:

| flight | config, seed | now | last night, same seed |
|---|---|---|---|
| v_ship_5 | old config, 500 g / 0.50 m, 5 | completed (no fold this time; prediction min 14.0 deg, no trip) | left DIRECT, q3 to −90, tilt 36° |
| v_ship_6 | old config, 500 g / 0.50 m, 6 | **rate guard at q3 18.2 deg** (44 deg/s), q3 min after 14.9, tilt ≤ 2.9°, abort → Land Start → landed | left DIRECT, q3 to −55, tilt 34° |
| v_ship_7 | old config, 240 g / 0.10 m, 7 | **rate guard at q3 16.3 deg** (50 deg/s), q3 min after 8.8, tilt ≤ 4.8°, abort → Land Start → landed | left DIRECT, q3 to −51, tilt 33° |
| v_fw_1 | shipped, 240 g / 0.10 m, 1 | no rate trip (q3 ≥ 21.6); box-tilt guard 1.3 s into the Exit (the climbing open claw dragged the box ~6 cm, tilted 6.3°), abort → landed | completed |

Both folds were caught while still slow, before they accelerated, and the arm never crossed 0. Not shown:
a fold already at ~300 deg/s when it starts (none happened). v_fw_1's abort is the known exit drag, not this
guard. Simulation closed after the runs.
