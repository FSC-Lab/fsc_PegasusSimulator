# Box pushing: the room-held arm swing — gains, the free-axis anchor, and automatic Ready → Push (2026-10-21)

Fix 5 of the 10-21 hardware report (`../wb_box_pushing_flight_20261021/`): the arm swings while
the claw is held at a fixed point in the room. On 10-21 the swing was (q2−q3)/2 ≈ 10.5° rms at
0.54 Hz, and its last half-cycle tripped the q3 guard. User request: tune the gains for box pushing,
then make Ready To Push and Push run without clicks.

## 1. The bench (offline, minutes per sweep)

`tools/wh_bench.py` flies the exact law with hardware-like feedback, using
`circle_tune_20260927/tools/circle_bench.py` unmodified:
- **the law:** `wb_entry_sim.Law` = controller.py + l1_observer.py, the C++ port's source of truth;
- **the plant:** the mirror plant, rotor lag, 16 ms transport delay, arm friction and joint stops;
- **the reference:** the 10-21 flight's own WholeBodyReference from the moment the claw sat on the
  fin (jaws open), held for 20 s;
- **three settings patched from outside:**
  - the claw held in the room (`ee_ref="world"`) vs following the vehicle;
  - the box yaml's per-block feed-forward filter;
  - raw-mocap feedback.

**Feedback calibration.** The 10-21 raw mocap's velocity, against a smoothed derivative of its
position, has 3–6 cm/s of noise horizontally and 2–3 cm/s vertically. Isaac's emulator injects
2.2 cm/s. With 4.5 cm/s in a 5 Hz band, the bench reproduces the hardware's hover CoM error
(20–23 mm rms vs 13–23 mm flown).

**Validation.** With the claw held in the room, the bench swings like the flight:
- q2 14–49°, q3 7–48°, q1 up to 11° (flown: q2 13–45°, q3 5–50°, q1 11°);
- then it crosses q3 = 0 into the elbow-singular branch after ~6 s (on 10-21 the q3 guard caught
  it at 5°);
- with the claw following the vehicle, the same noise gives 0.8° of swing.

### 1.1 Gains do not fix it (`sweep_r1_hw.txt`, `sweep_r2_hw.txt`)

25 candidates, 2–3 noise seeds each. A seed passes if it completes 20 s held in the room with
q3 ≥ 10° and q2 ≤ 45°.

| | result |
|---|---|
| baseline (the box-pushing yaml) | 0/3, 7.6–7.8° rms, diverges at 7.4 s on seed 0 |
| attitude damping k_w 1.5 / 1.8, pick-and-drop k_R/k_w 2.134/1.567, M_r_d ×1.2 | 0/2 each, 7.4–8.1° |
| claw stiffness/damping K_y/D_y 80/16.5, 200/26, D_y 10 / 30, heading 2.5/0.93 | 0/2 each, 7.5–7.7° |
| position gains k_x/k_v 32/12.58, 32/16, 20/10, k_v 16 | 0/2 each; lower k_x delays the divergence to 15–17 s |
| observer ω_c ×0.7 / ×0.5 / ×0.33, per channel | 0/3; ×0.5 → 7.0°; 3× faster → 24° |
| best: k_x 32 + observer ×0.5 | 1/3, 6.9° rms (~10 % better) |

**Why gains can't fix it** (`ablate.txt`): the swing is the arm taking up the vehicle's wander, not
a lightly damped resonance.

| bench condition | swing rms |
|---|---|
| hardware noise, claw following the vehicle | 0.8° |
| no velocity noise | 1.2° |
| Isaac's noise level | 3.7–4.4° |
| the same 4.5 cm/s spread to 50 Hz (less low-frequency power) | 2.7–3.0° |
| hardware noise (5 Hz band) | 7.6–7.8°, diverges |

It scales with the low-frequency part of the feedback error. The observer bandwidth is the only gain
that moves it, because the deadbeat estimate passes that error on.

### 1.2 The free-axis anchor fixes it (`sweep_r3_hw.txt`)

`wb_ee_anchor_free_axis_weight` already exists in the whole-body law (10-18, valve turning), off
in the box-pushing yamls. While the contact flag is on, the claw's room-held reference also rides
`w` × the CoM error along the nose. For box pushing the nose is the push direction, which is along
the arm, so the arm stops taking up the vehicle's drift along it. It is mirrored into
`wb_entry_sim.py` as `Case.ee_free_w` (default 0).

| weight | passes | swing rms | q2 max / q3 min | claw error in the room |
|---|---|---|---|---|
| 0 | 0/3 | 7.6–7.8° | 49° / 7° | 6 mm |
| 0.25 | 2/3 | 6.0° | 44.5° / 12° | 6 mm |
| 0.5 | 2/3 | 4.3–4.8° | 45.4° / 11° | 8–10 mm |
| **0.75** | **3/3** | **2.6–2.7°** | **39.7° / 18.9°** | 10–14 mm |
| 1.0 | 3/3 | 0.9–1.0° | 35.5° / 23.8° | 13–17 mm |

The cost is the last column: the claw follows part of the vehicle's wander along the push, so the
box does too. The bench has no box, so the push is checked in Isaac (§3).

## 2. Automatic Ready To Push → Push (the station, `utils_custom_ground_station`)

**What changed** (`push_pull_panel.{hpp,cpp}`, box tab only):
- **Ready starts by itself** `push_pull_auto_ready_delay_s` (1.0 s) after the planner's
  FLYING go_to_start → DONE go_to_start edge (`push_pull_auto_ready`, true). The gripper opens and
  the descent follows the OPEN report, as a press. A "wait for it to settle" refusal is retried
  every 0.5 s for `push_pull_auto_ready_timeout_s` (10 s), then the station asks for the press.
- **Push starts by itself** on the FLYING ready → DONE ready edge (`push_pull_auto_push`, true).
  The gripper closes the first time the claw is in the 2 cm / 3 mm gate, and the push follows the
  grasp, as a press. A wrong grip still moves nothing.
- **Exit** follows by itself 0.2 s after the push (the earlier fix).

**Rules, the same as the automatic Exit:**
- only on the planner's own edges, so a restarted station never starts a step;
- any press, ABORT, Reset or a lost DIRECT cancels it;
- the valve tab is unchanged.

**On hardware:** Go To Start is the only press of the push, and ABORT stays available throughout.
The claw's open-jaw room hold before the grasp shrinks from the operator's reaction time (1.4–2.4 s
on 10-21) to the gate.

**Checks:**
- **Offscreen station harness** (`../box_pushing_hardware_20261019/tools/gs_grasp_check.sh`), new
  cases G1–G5:
  - Ready starts 1 s after the edge and descends only on the OPEN report;
  - two settle refusals are retried and the third call succeeds;
  - the jaws close by themselves at the gate and the push follows the grasp;
  - a latched DONE ready does nothing;
  - ABORT inside the delay cancels it.
  - Every earlier case still passes.
- **Isaac:** the station runs Ready → close → Push → Exit while the driver only watches
  (`pl_mission.py --gs-auto`).

**Sim drivers keep owning the steps:**
- `run_pl.sh` / `run_pl_cadm.sh` pass `ARM_GS_AUTO_STEPS=false` unless told otherwise;
- the 4-D and coordinated-admittance launchers forward it as `push_pull_auto_ready` / `push_pull_auto_push`.

## 3. Isaac A/B (station automation on; free-axis 0.75 vs 0)

`tools/chain.sh`, `tools/fly.sh` (`FREE_W` inserts the key into the sim box yaml for one flight),
scored by `tools/ab_metrics.py` → `ab_metrics.txt`. Task = the 10-21 hardware task: 240 g, 0.10 m
at 21 mm/s, hardware guard values.

All 9 flights completed with a normal Exit.

| | driver presses Ready/Push (d02, 3) | station automation, free-axis 0 (3) | station automation, free-axis 0.75 (3) |
|---|---|---|---|
| claw held in the room, jaws open, before the grasp | 2.1–2.4 s | **0.85–1.2 s** | **0.4–1.0 s** |
| arm swing (q2−q3)/2 std, push step | 5.6–6.0° | 4.7–5.4° | **2.2–2.8°** |
| lowest q3, arrival → end of exit (guard 10°) | 12.2–18.0° | 15.7–22.2° | **19.4–25.3°** |
| highest q2 (real limit 45°) | 39.4–42.8° | 38.9–43.3° | **38.1–39.0°** |
| box travel at the push end (plan 100 mm) | 90.9–98.5 mm | **98.6–101.0 mm** | 91.0–107.6 mm |
| box pushed back while clamped, before the slide | 0–4.9 mm | 0–1.7 mm | 0–**19.1** mm |
| box stuck in the slide / peak speed | 0–15 % / 60–75 mm/s | 0–14 % / 53–72 mm/s | 0–20 % / 68–87 mm/s |
| vehicle tilt, push step | 2.5–3.0° | 2.3–2.8° | 1.4–2.5° |

- **Station automation alone** helps in two ways. The open-jaw room hold before the grasp falls to
  under ~1 s. Because the jaws close at the first gate entry, before the arm has swung much, the
  push is also more accurate: 98.6–101.0 mm vs 90.9–98.5.
- **The free-axis anchor 0.75 halves the swing** and keeps both joints well inside their limits:
  q3 ≥ 19°, q2 ≤ 39°. That fits the bench, which predicted a larger cut at the hardware's
  disturbance level (7.7° → 2.6°).
- **The cost: the box follows part of the vehicle's wander along the push.** Push travel spreads to
  91–108 mm. In one flight the clamped box was dragged 19 mm backward during the 1.5 s settle. In a
  0.50 m push that is about ±2 %.

**Not covered.**
- Isaac's wander is smaller than the hardware's: the bench, not Isaac, carries the hardware-level
  evidence.
- The free-axis anchor has never flown on hardware. In valve turning, 1.0 with a faster
  translational observer ran away once (README of valve_turning_20261015 §15). Box pushing flies
  ω_c_t 1.0, the setting under which the valve flights were stable.

## 4. State of the tree (nothing committed)

- **Station automation (shipped as the default):** `utils_custom_ground_station`
  `push_pull_panel.{hpp,cpp}`. Laptop: rebuild `utils_custom_ground_station`.
- **The free-axis anchor: APPLIED** (the user accepted the push-distance cost, 2026-10-21), see §5.
- **Sim plumbing:**
  - `pl_mission.py --gs-auto`;
  - `ARM_GS_AUTO_STEPS` in `run_pl.sh`, `run_pl_cadm.sh` and the 4-D / coordinated-admittance launchers;
  - `wb_entry_sim.py` `Case.ee_free_w` (default 0, so every earlier bench result is unchanged).

## 5. Applied: the anchor at 0.75, and the box-lag guard moved onto the law's target

**The cost the user accepted.** The box follows 75 % of the vehicle's drift along the push, so the push
distance is off by about ±1 cm (at worst ±2.5 cm on hardware), whatever the distance.

**The change:**
- `push_pull_20261003/tools/make_push_pull_yaml.py` ADDITIONS: `wb_ee_anchor_free_axis_weight: 0.75`,
  inserted after `wb_ee_anchor_com` (where the A/B's `fly.sh` put it).
- Regenerated: the sim and hardware box-pushing yamls. The diff is that one line each, plus the sim
  header. Unchanged (`--check`): the coordinated-admittance and both valve-turning yamls.
- The as-flown A/B yaml (`runs/a75_1.yaml`) has the same controller section as the shipped one; the
  differences are the flight's task settings only.

**A guard interaction, found before shipping.** The planner's box-lag guard (40 mm for 0.2 s)
measured the box against the BARE plan. With the anchor on, the box follows the vehicle's drift on
purpose, and the guard read that as lag:

| run | guard reading, bare plan | against the law's target |
|---|---|---|
| a0 (no anchor), Isaac | 6–13 mm | same |
| a75_1, Isaac | **34.6 mm** of 40 | 16.2 mm |
| the 10-21 hardware push, re-estimated with the anchor on | **37.5 mm** of 40 | 23.5 mm |

The real vehicle drifts ~2× Isaac's, so a false abort mid-push was likely.

**The fix (fsc_trajectory_planner):** the guard adds the law's own shift to the plan:
- the shift is `wb_control_debug` [117] × (the vehicle's horizontal nose · the push direction);
- it counts only while that feed and the attitude are fresh; otherwise it is 0, i.e. the old guard;
- the trip message names it: `[law free-axis shift N mm counted]`.

**Checks:**
- Planner gtests 18/18.
- New loopback case [9f], with the box following the plan exactly: bare reading −0.5 mm; a 30 mm shift
  reads +29.7 mm and passes; 60 mm aborts as BEHIND, with the shift named. ALL PASS on both the sim and
  the hardware yaml.
- Both whole-body hardware stacks list the weight (45 / 43 tuned values OK). They refuse a planner
  without the compensation whenever the yaml's weight and its box-lag guard are both > 0
  (negative-tested).
- `tools/fly.sh`: `FREE_W` now replaces the yaml's value (0 included) instead of inserting a second line.

**Isaac, on the shipped yaml** (`tools/chain_apply.sh`, scored by `tools/apply_metrics.py` →
`apply_metrics.txt`; station automation on; the hardware guard values):

| run | outcome | guard reading | q2 max (limit 45) | q3 min (guard 10) | vehicle tilt |
|---|---|---|---|---|---|
| v75_n1: 240 g, 0.10 m | completed, box 99.9 mm, swing 1.9° | 7.8 mm (bare 12.2) | 38.7 | 25.1 | 2.0 |
| v75_heavy: 1 kg, μ 0.6 | **tilt guard, 9.2 s** (10.7°) | 38.4 mm (bare 74.4) | 43.6 | 20.9 | 10.7 at the trip |
| 10-16 h_heavy / h_heavy2 (no anchor) | box-lag guard, 8.1 s | 43.5 / 44.8 mm | 42.5 / 41.9 | 15.5 / 20.2 | 9.8 / 8.6 |
| v75_stop: stopper after 0.2 m | **box-lag guard, 14.0 s** (shift −23 mm counted) | 46.1 mm (bare 69.4) | 40.5 | 22.4 | 7.0 |
| 10-16 h_stop (no anchor) | box-tilt guard, 13.3 s | 42.4 mm | **45.0** | 17.3 | 9.6 |

**What the jams show.** A stuck box drags the vehicle back. The anchor then moves the claw's target
back too, so the compensated reading excuses part of the lag, and the trips came **0.7–1.1 s later**.
- **Why that is acceptable:** the compensated reading is close to what threatens the arm. The arm's
  compression is the bare lag plus the CoM error; the reading is the bare lag plus 0.75 × it.
- **The anchor also makes the arm softer against a stuck box.** In the stopper case the box never
  tipped, and q2 stayed 4.5° off the stop it hit without the anchor.
- **Backstop:** with a heavy box the vehicle's tilt carries the drag (atan(5.9 N / mg) ≈ 9°), and the
  10° tilt guard caught it first. Both anchor aborts ended level (final tilt 2.3° / 1.2°), as the
  three 10-16 ones did (≤ 2.6°). The force guard stayed below 5 N in all of them (3.1–4.6 N).

**A bare-plan guard widened to 55 mm, considered and not taken.** It would have caught both jams
~0.5 s sooner. But its margin is the drift the anchor follows: 17.5 mm on the 10-21 push, shrinking
with a larger drift and on the 25 s push. The compensated reading does not depend on the drift.
Not built: a cap on the excused shift (e.g. 25–30 mm, above the normal drift's 0.75 × 36 mm).

**Not covered.**
- One flight per jam case.
- The jam outcomes rely on the tilt guard at the hardware's 10° (the sim yaml keeps 15).
- Never flown on hardware.

**Deployment.**
- **Orin:** rebuild `fsc_trajectory_planner`. The stack refuses an older one; the node needs nothing
  new (its LATCHED free-axis build, 10-18, is probed already).
- **Laptop:** rebuild `utils_custom_ground_station` (station automation).
