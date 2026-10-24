# 2026-10-23 box pushing on hardware: the q3 rate guard aborted the grasp — is it too conservative?

Bag: `docs/experimental_data_ros2_bag/1023 - T650-AM whole-body Box-Pushing [Fail, safety]-*/…/flight_wb_l1_4d_box_pushing_20261023_172853`
(116 s, the only recorded flight of the day; the same folder's `mocap_valve_spin_20261023_174910`
holds only obj_0 mocap and is not a flight). The day's second trip and the 10-22 trip(s) were not
recorded. No 10-22 bag exists in the repo, so "yesterday's" data here is the 10-21 push
(`../wb_box_pushing_flight_20261021`, bag `_170123`).

User, 2026-10-24: the arm GS reported the q3 safety guard for the 10-23 trips. Both q3 guards were
in that day's planner: the rate guard reached `origin/main` on 2026-10-22 12:04 (`139eb72`), and the
recorded trip's message is the rate guard's. The GS text is not enough to tell which guard fired the
unrecorded trip, because both messages start "q3 …". The rate guard's reads "falling at … deg/s ->
… deg in 0.20 s"; the static guard's reads "q3 … deg < 10.0 deg (the arm buckling …)".

Flown: hardware box-pushing yaml (raw stack, mocap guard), free-axis anchor 0.75 from the room hold
(`wb_ee_anchor_free_axis_on_world`), q3 static guard 10° + the 10-22 rate guard (0.2 s look-ahead),
GS automatic Ready / Push / Exit. Push Distance 0.15 m (planned push 7.14 s at 21 mm/s). Box mass not
in the bag (the plan was 240 g).

## 1. What happened (`timeline.txt`, `trip_window.txt`)

| t [s] | event |
|---|---|
| 11.36 | DIRECT |
| 15.28–40.14 | Go To Start (24.8 s, round the table) |
| 41.82–45.00 | Ready descent, claw CoM-anchored, jaws open |
| 45.00 | on the handle: claw WORLD-held, free-axis anchor latched; the q3 guards arm |
| 47.79 | GS close gate met (claw 10 mm from the grasp point, −2.2 mm across) → gripper CLOSE |
| ~48.36 | jaws closed on the fin (gripper −0.43 rad, stalled; the action had not returned) |
| **48.84** | **q3 RATE GUARD: "q3 18.0 deg falling at 67.0 deg/s -> 4.6 deg in 0.20 s < 10.0 deg"** → abort |
| 49.36 | jaws open |
| 60.34 | abort complete (unbuckled to 32/28°, climbed 0.30 m, arm home) |
| 71.2–110.4 | Go To Land Start, Land; pilot STAB at 111.47 |

The push never started. The abort was calm (`abort_check.txt`): PX4 tilt ≤ 4.0°, no saturation,
peak arm torque 1.31 N·m, q3 ≥ 14.1° throughout. The box ended 9 mm from its Ready-end pose,
upright.

## 2. The trip, replayed exactly

`tools/trip_window.py` replays `plQ3RateGuard` on the recorded joint states (header stamps,
0.02 s rate filter, 50 ms persistence). It fires at **48.836 s, q3 18.0°, −67.0°/s, predicted
4.6°**, identical to the planner's message, so the guard replays below are faithful.

What the arm actually did:
- q2 rose 31.4 → 45.4° and q3 fell 28.6 → 14.1° in 0.5 s (48.4–48.9 s).
- The fold q2 + q3 stayed ≈ 60° the whole time (60.0 → 58.8°).
- **q3 stopped dead at 14.1° the instant q2 reached its +45° limit**: its rate went −59 → −9°/s
  in 40 ms, within 0.08 s of the trip, before the abort could act. The anchor blend was then
  8 % done and the jaws not yet moving.
- The static guard (q3 < 10° for 0.2 s) would never have fired.

**This is the arm's reach motion, and q2's limit bounds it.** q3 = (q2 + q3) − q2; with the fold
held near 60° and q2 unable to pass ~45°, q3 cannot go below ~15° this way. The guard's linear
0.2 s extrapolation does not know q2 will stop, so it over-predicted the drop by ~10°.

## 3. Why the guard cannot simply be removed (`fold_compare.txt`, `guard_variants_q2max45.txt`)

Three buckles in the 10-21/22 Isaac campaign (`box_pushing_contact_stability_tune_20261021`:
z_ship_6, z_ship_7, h_ship_5, old config) left DIRECT at 22–48° tilt. **All three happened while
the GS was closing the gripper, the same phase as today's trip.** Aligned on q3 crossing 20°:

| | q2 at the 20° crossing | q3 rate then | after q2 reaches its limit |
|---|---|---|---|
| HW 10-23 | 41.0 → 45.4 0.08 s later | −63°/s | q3 stops at 14.1° |
| SIM z_ship_6 | 45.0 (pinned) | −31°/s | q3 re-accelerates: −61 (11°), −139 (−1°), −410°/s, left DIRECT |
| SIM h_ship_5 | 45.0 (pinned) | −50°/s | the same, to −642°/s, q3 −90° |

At the trip instant the two look alike (q3 17–19°, tens of °/s, q2 at or reaching 45°). They
differ only in what q3 does **after q2 is pinned**: on hardware it stopped; in a buckle it kept
falling and accelerated.

Guard variants replayed on both hardware pushes and all 67 Isaac runs (48 completed). Each variant
keeps the static check. "Before DIRECT loss" is the lead time on the 3 buckles, with q3 at the
trip.

| rule | 10-23 HW | 10-21 HW | buckles caught before DIRECT loss | false trips (48 completed) |
|---|---|---|---|---|
| static only (rate guard removed) | no trip | static at q3 5.1 | **0 / 3** | 1 |
| today: q3 + 0.2 q̇3 < 10 | **trips** (q3 18.0) | 8.5 s earlier, q3 21.6 | 3/3, 0.25–0.31 s, q3 14–16 | 7 |
| **q2-limit aware (proposed)** | **no trip** | 8.5 s earlier, q3 21.6 | **3/3, 0.25–0.31 s, q3 14–16** | 6 |
| predict q3 < 0 instead of 10 | no trip | 0.36 s earlier, q3 14.1 | 3/3, 0.14–0.20 s, q3 7–9 | 1 |
| look-ahead 0.1 s | no trip | 0.37 s earlier, q3 14.4 | 3/3, 0.19–0.26 s, q3 11–12 | 3 |

- **Removing the rate guard is not safe on this evidence.** The static check alone catches none
  of the three buckles before the node leaves DIRECT.
- **Hardware abort margin.** 10-21 shows what an abort from a low q3 costs: tripped at 5.1°, q3 then
  went to −5.2° within 0.5 s (−7.3° at worst) while the jaws opened. The abort needs ~0.5 s to open
  the jaws and blends the claw anchor over 1.0 s, so later trips eat the margin.
- **"Predict q3 < 0"** and **"look-ahead 0.1 s"** clear today's trip, but catch the buckles later
  and lower (q3 7–9° and 11–12°).
- **The proposed rule** is the same prediction, with q2's predicted motion capped at its limit:
  `q3_pred = q3 + 0.2·q̇3 + max(0, q2 + 0.2·q̇2 − q2_max)`, credited only while q2 is at or below
  its limit. It is exact kinematics, q3 = (q2 + q3) − q2.
  - Today's trip disappears.
  - The buckle catches are unchanged: q2 is already pinned there, so the cap is zero.
  - The result does not depend on the assumed limit, 44–47° (`guard_sensitivity.txt`).
  - Cost: Isaac v_ship_6, a real 10-22 rate-guard trip, would not have fired at that moment (q3
    then bottomed at 14.9° with the abort acting). Its seed-twin z_ship_6 did buckle, and the new
    rule catches that buckle at the same time as today's rule.
- The 10-21 replay trips under every rate rule during the open-jaw swing of the old config
  (q2 ≤ 39°, far from its limit). The free-axis anchor removed that swing; today's open-jaw
  window had q2/q3 within 31–34° / 28–29°.

## 4. What drove the arm to its q2 limit (`window_stats.txt`, `isaac_open_jaw_q1.txt`)

| phase (10-23) | dur | CoM error along / across nose, rms/max [mm] | q1 | q2 | q3 | box moved |
|---|---|---|---|---|---|---|
| hover above the handle | 1.7 s | 27/35 · 12/19 | −0.2 | 33.7–33.8 | 28.0 | 0 |
| Ready descent (CoM anchor) | 3.2 s | 10/23 · 15/24 | −0.4 | 33.7–33.8 | 28.0 | 2 mm |
| open jaws, WORLD-held | 2.8 s | 24/28 · **34/46** | **−8.7 … −0.3** | 31.3–33.8 | 28.0–28.8 | 3 mm |
| jaws closing / closed | 1.0 s | 11/25 · **35/73** | **−8.3 … +11.7** | **31.3–42.6** | **18.0–28.7** | **23 mm** |

- **The free-axis anchor did its job along the nose.** On 10-21 the same open-jaw phase swung
  q2/q3 over 16–39° / 12–45°. Today they stayed still.
- **The motion moved across the nose instead.** The lateral CoM error tripled under the world
  hold (34 vs 12–15 mm rms), and the arm took it up as a q1 twist with q4 counter-rotating.
- **Why the grasp waited 2.8 s** (`push_pull/arrival_error`):
  - **Across:** the descent ended with the claw 21 mm off across the fin. The world hold pulled
    it to 5–6 mm off and stalled there for 1.3 s, with **q1 stuck at −8.5°**: joint-1 stiction
    against the ~0.7 N spring. The GS's 3 mm across gate opened only when the base's own sideways
    swing carried the claw through it.
  - **Along:** the claw sat 20–22 mm short, right at the 2 cm gate. That matches 0.75 × the
    CoM's along-nose error of 24–28 mm, i.e. the free-axis anchor's offset by design.
- **The close landed in that sideways swing** (~0.2 Hz, ±70 mm). Once the jaws clamped, the base
  moved ~50 mm sideways and ~25 mm back:
  - the box was dragged 26 mm and turned 5.5°;
  - the arm yawed (q1 +12°) and extended until q2 reached its limit.
- **The gripper closed in the middle of that swing** (`tools/grasp_chain.py`, `grasp_chain.txt`;
  world x = across the fin):
  - The swing came with a roll attitude error of the same period: model `e_R,y` ≈ −45 mrad
    through the open-jaw hold, +100 mrad at the trip. That is the pure-P attitude loop's
    `T·e_R/k_x` offset again, and the across CoM error (+45 → −72 mm) followed it.
  - At the close command (47.79 s) the claw was moving across at ~40 mm/s, the base at ~55 mm/s.
    The claw was passing THROUGH the 3 mm gate, not settled in it: the GS gate checks position
    only.
  - The jaws took ~0.56 s to close (47.80 → 48.36 s). The base kept going, ~110 mm/s across in
    48.4–48.8 s.
  - Once clamped, the box (26 mm / 5.5°) and the arm (q1 +20°, q2 to its limit) absorbed the rest.
  - Before the close, the swing was 12–15 mm rms in this flight's other hovers and ±18 mm in the
    hover above the handle. One flight cannot tell whether the room hold grew it or it was simply
    large at that moment.
- **Isaac shows the same dependence on the wait.** Shipped-config open-jaw waits are 0.4–2.0 s
  with q1 ≤ 7°. All four 3.0 s waits (k_w09_5, l_wb05_5, z_final_7, z_final_8) swing q1 by
  10–17°. Three of them push q2 to 40–45°, and three failed: two GS timeouts plus z_final_7,
  which took q3 to −6.5°.
- **Side note (`attitude_check.txt`):** mocap and PX4 attitude disagree by 2.8° in pitch today
  (3.1° in roll on 10-21). The rigid body's frame differs day to day; the 4–5° "tilt" in the mocap
  tables is that offset. It is not the cause here.

## 5. Recommendation

1. **Keep both q3 guards; make the rate guard aware of q2's limit** (planner only, ~20 lines).
   **APPLIED 2026-10-24, see §9.**
   - Add `push_pull_guard_q2_max_deg` (0 = today's rule), set to 45 in the box-pushing yamls (the
     real arm's q2 limit, `external_torque_controller_hardware_aerial_pwm.yaml`).
   - Credit the cap only while q2 ≤ the limit + 1°: a q2 pushed past its limit gets no credit.
   - Add gtest / loopback cases: a reach motion into the q2 limit at ~65°/s from q3 28° must not
     trip; q2 pinned at 45° with q3 still falling must trip at today's moment.
   - Run `tools/guard_variants.py` as the regression check.
   - The hardware stacks refuse an older planner.
   - Static guard unchanged (10°, 0.2 s).
2. **Do not remove the rate guard, lower the static limit, or shorten the look-ahead**: §3.
3. **Shorten the open-jaw world hold — the cause.** Candidates, to rank in Isaac with the
   hardware-like force model before flying:
   - (a) trim the Ready descent across the fin, so the claw arrives centred, not 21 mm off;
   - (b) a grasp-wait timeout: back off and retry instead of holding the claw in the room for 3 s;
   - (c) check on the bench whether the foam pads grasp the 5 mm fin centred within ±5 mm, and
     open the across gate to match;
   - (d) add a speed condition to the close gate (claw settled, e.g. < 10 mm/s for 0.3 s), so the
     jaws never close while the claw is being swept through the gate.
     **Checked on this flight (`tools/settle_gate.py`, `settle_gate.txt`):**
     - The condition would NEVER have been met. Even a loose claw < 20 / base < 30 mm/s held 0 %
       of the open-jaw hold.
     - The position gate alone held only 11.8 % of the hold, all of it after 47.75 s, while the
       swing swept the claw through at 38–49 mm/s.
     - Even calm, above the handle, the drone wanders at 18.5 mm/s median (p90 23).
     - So the 3 mm across tolerance is tighter than the hover wander: it is met only mid-swing.
       A speed gate on its own blocks the grasp. It needs a timeout + back-off/retry, and the real
       choice is between a wider across tolerance (if the pads allow it), less wander during the
       grasp, or both.
4. **Data:**
   - Send the planner/GS text of the two unrecorded trips. The abort line names the pose, e.g.
     "unbuckles from q2/q3 42.6/18.0". q2 ≥ ~42° there means the same signature, and the proposed
     rule would have let those pass too.
   - Record every flight.

## 6. Every guard that can end a box push, and how close each came on hardware (`tools/guard_margins.py`)

Thresholds are the HARDWARE box-pushing yaml's. Hardware = the two pushes' peak in the guard's armed
window (10-21 `_170123` / 10-23 `_172853`).

**Planner guards** (abort: jaws open, climb, arm home):

| # | guard | limit | hardware peak | Isaac record | verdict |
|---|---|---|---|---|---|
| 1 | vehicle tilt | 10° for 0.1 s | 5.6 / 4.1° | <= 6.1° in 51 pushes | keep (opens the jaws before the node's 15°) |
| 2 | claw force, LP 3 rad/s | 5 N for 0.3 s | 2.7 / 2.0 N | 0 false trips; never the first to catch a jam (16 N after the buckle) | harmless backstop |
| 3 | box tilt | 2° for 0.2 s | 3.1 (mis-solves, now gated) / 0.75° | caught the side-edge fall 4/4 | keep; widen to 3° only if a real push reads > 1.5° |
| 4 | box lag behind plan | 40 mm for 0.2 s | 23.5 (no free axis) / – | jams caught; nominal 8–16 mm with the free axis | keep |
| 5 | box feed silent/frozen/gated | 0.5 s | none | none | keep (guards 3–4 are blind without it) |
| 6 | arm joint states silent | 0.5 s | none | none | keep (guards 7–8 are blind without it) |
| 7 | q3 static | < 10° for 0.2 s | **TRIPPED 10-21 at 5°** (−7.3° in the abort) / 14.1° | caught slow folds | keep (10-21 was a real near-buckle) |
| 8 | q3 rate (0.2 s look-ahead) | pred. < 10° | **TRIPPED 10-23** (false alarm: bottomed at 14.1°) | caught 3/3 buckles 0.25–0.31 s early | relax: make it q2-limit aware (section 3) |
| 9 | drone mocap guard | CONFIRMED 0.15 s on the box / LOST 1 s | worst 25 / 22 ms | caught every injected freeze | keep (0924 fly-away) |

**Flight-node watchdogs** (revert to SAFETY, hover; the arm folds after 2 s; the GS opens the jaws if
they are on the handle):

| # | guard | limit | hardware peak | verdict |
|---|---|---|---|---|
| 10 | tilt / body rate | 15° / 360°/s | 5.6° / 36°/s | keep (last resort) |
| 11 | horizontal CoM drift | 0.75 m | 0.13 m | keep (last resort) |
| 12 | feedback lost | 1.0 s | none | keep |
| 13 | PX4 left OFFBOARD | the pilot's switch | n/a | by design |

**Only the q3 guards (7, 8) have ever ended a hardware push.** Every other signal stayed at ≤ 50–60 % of
its limit, and none false-tripped in Isaac's nominal runs. The non-abort gates also cost time: Plan
refusals, the Ready settle gate, and the GS close gate, which made the 10-23 grasp wait 2.8 s.

## 7. The q2/q3 limits as a WINDOW (`tools/joint_window.py`, `joint_window_options.py`)

Every limit acting on q2/q3 during the push:

| limit | where | value | what happens |
|---|---|---|---|
| q2 wall | arm controller | +45° (U2D2 board), −20° | past it, the law's j2 torque is REPLACED by a stiff pull-back (kp 101.5 N·m/rad) -- silently |
| q3 wall | arm controller | +50°, −40° | same, kp 116.5 |
| q3 static guard | planner | < 10° for 0.2 s | abort |
| q3 rate guard | planner | 0.2 s look-ahead < 10° | abort |
| torque caps | arm / law | 2.81 / 1.92 N·m (j2/j3), law 3.0 | clipped, not an abort |

Planner model, gripper EE, claw motion along the arm relative to the airframe at constant height:

| pose / limits | room OUT (stretch) | room IN (fold) |
|---|---|---|
| today [0, 32, 28, 0] | 22.0 mm (q3 guard; q2 at 43.9°) | 35.5 mm (q3 wall) |
| [0, 30, 30, 0] | 25.0 | 32.5 |
| [0, 27.5, 32.5, 0] | 28.5 | 29.0 |
| [0, 26, 34, 0] (the 10-07 pose) | 31.0 | 27.0 |
| today, q3 guard 5° | 24.0 (q2 wall) | 35.5 |
| today, q3 guard 5° + q2 wall 50° | 26.5 | 35.5 |

- 1 cm out costs +5.0° q2 and −7.6° q3.
- The total window is ~57 mm whatever the pose; the pose only moves the split.
- Lowering the q3 guard buys 2 mm, because the q2 wall comes next.
- The rate guard shrinks the OUT room with speed: at 67°/s it fires ~13° early, about 6 mm of
  claw travel from today's pose.
- The 10-23 stretch was +20 mm out and +25 mm up.
- In flight the q2 wall's pull-back acted for 0.26 s (10-21) and 0.04 s (10-23).
- Sideways motion goes to q1 (±35°, ~150 mm) and is not the binding axis.

## 8. Why the guards watch q3, and how q2 enters (`tools/sigma_map.py`)

- **The arm's distance from its singular pose (sigma_nd, the planner's measure) depends on q3
  ONLY.** It is identical for every q2 from −20° to 50°.
  - The singular pose is at q3 ≈ −34°.
  - The planner's keep-out (sigma_nd 0.10) is at q3 ≈ −14°.
  - The static guard's 10° sits at sigma_nd 0.215, about 2x the keep-out.
  - So "too close to the singular pose" is a q3 question.
- **q2 has no singularity; its only limit is the U2D2 board at 45°.** The arm controller holds q2
  there (≤ 0.4° past it in flight).
- **q2 matters through the PAIR.** When the arm stretches, q2 and q3 move together. Once q2 is on its
  wall, further stretch can only come from q3 folding the arm on its own: the collapse. That is what
  the q2-aware rate guard detects, so it is the group guard on (q2, q3).
- **A separate q2 abort guard is NOT proposed.** It would fire on the harmless wall touches, 0.26 s
  (10-21) and 0.04 s (10-23). A "q2 on its wall" warning is worth having: the law is not told when
  the wall replaces its j2 torque.

## 9. Applied 2026-10-24: the q2-aware rate guard (uncommitted, not flown)

The rule:

    q3_pred = q3 + 0.2 s * q3_dot + max(0, q2 + 0.2 s * q2_dot - q2_max)

- `q2_max` = `push_pull_guard_q2_max_deg`. 0 = the 10-22 rule; the box-pushing yamls set 45.
- q2_dot is low-passed with the same filter as q3_dot (`push_pull_guard_q3_rate_filter_s`, 0.02 s).
- No credit while q2 is more than 1° past `q2_max`.
- The static guard (q3 < 10° for 0.2 s) is unchanged.
- The abort message now says how much of the fall the wall stopped, e.g. "(… deg of the fall
  stopped by q2's 45 deg wall: q2 … deg rising at … deg/s)".

Where:

- **fsc_trajectory_planner** `whole_body_trajectory_planner_node.cpp`: the new key (gate-only, so
  setting it does not drop a plan) and the q2 term in `plQ3RateGuard`.
- **Loopback** `test/test_push_pull_loopback.py` step [9k]:
  - (a) the 10-23 stretch, q2/q3 32/28 -> 45/15° at 65°/s: no abort;
  - (b) the same stretch with the key at 0: aborts, as on 10-23;
  - (c) q2 pinned at 45° while q3 folds at 150°/s: aborts, with no credit.
- **Yamls**, through `push_pull_20261003/tools/make_push_pull_yaml.py`: the sim and hardware
  whole-body box yamls and the coordinated-admittance box yaml. The diff is the key and its comment.
  The valve yamls do not set it, so the valve turn keeps the 10-22 rule.
- **Hardware stacks** (whole-body raw and fused): expect `push_pull_guard_q2_max_deg: 45.0`, and
  REFUSE a planner build without the key when the yaml sets it.
- **Isaac scene check** `scripts/indoor_sim/lib/push_pull_planner_check.sh`: compares the key.

Checked (no flights):

- Planner gtests: push 27/27, pick-place 31/31.
- Loopback ALL PASS, private ROS domain, on both yamls:

| case | sim yaml | hardware yaml |
|---|---|---|
| [9k-a] 10-23 stretch, wall on | no abort | no abort |
| [9k-b] same, key 0 | abort at q3 19.0°, 66.8°/s | abort at q3 18.0°, 67.8°/s |
| [9k-c] q2 on the wall, q3 folding | abort at q3 15.7° | abort at q3 14.2° |
| [9i] 150°/s fold, q2 32° | abort at q3 14.0° | abort at q3 14.3° |

Case [9k-b] reproduces the flown trip (q3 18.0° at −67°/s on 10-23).

**Orin:** pull fsc_trajectory_planner and rebuild it; pull fsc_autopilot_ros2 (yaml + stack
scripts, no rebuild). The stack names the key in its startup check.

## 10. Can the q2/q3 limits be widened? (`tools/joint_limit_options.py`, `tools/guard_thresholds.py`)

Asked 2026-10-24. Only the q3 FLOOR has anything to do with singularity (section 8). The other
limits are mechanical or software:

| limit | set by | singularity? | can it widen? |
|---|---|---|---|
| q2 <= 45° | the U2D2 board behind the upper arm (mechanical) | no | only by moving the board |
| q2 >= −20° | the real arm's range | no | never reached near the push pose |
| q3 <= 50° | the controller's working range (no recorded reason; same as the sim asset's stop) | no (sigma_nd RISES with q3) | yes, after a bench check: the stock OM-X linkage reaches +80.2° |
| q3 >= 10° | the planner's static + rate guards (one key) | yes | singular at −34°, keep-out at −14°: 5° keeps sigma_nd 0.192 (1.9x the keep-out) |

Only the q3 floor ABORTS. The walls replace the law's torque with a pull-back: the arm stops absorbing
the drift, and the drift then goes into the box.

Claw room from the push pose [0, 32, 28, 0], mm to the first limit:

| option | out | 10-23 direction (out + up) | in | in + up | down |
|---|---|---|---|---|---|
| today | 22.0 (q3 10) | 30.5 (q2 45) | 35.5 (q3 50) | 43.5 | 71 |
| q3 floor 5 | 24.0 (q2 45) | 30.5 | 35.5 | 43.5 | 82 |
| + q3 max 60 | 24.0 | 30.5 | 54.5 | 68.5 | 82 |
| + q3 max 65 | 24.0 | 30.5 | 64.5 | 82.5 | 82 |
| + q2 max 50 (board moved) | 26.5 (q3 5) | 40.5 | 64.5 | 82.5 | 82 |

- **The q2 wall and the 10° floor cut a level stretch at the same place.** A level stretch onto the
  q2 wall parks q3 at 7.6°, below 10. So an arm just resting on its wall still trips the static
  guard after 0.2 s. A 5° floor removes that.
- **OUT room is capped by the q2 wall (mechanical).** A lower floor adds only 2 mm out; the rest of
  its gain is downward.
- **IN room is software.** On 10-21 q3 reached 50.4°.
  - At q3 65° the elbow sits 208 mm under the body origin, 30 mm lower than at the push pose,
    still ~157 mm above the table. The upper arm is near vertical (q2 ~5°).
  - Shapes past q3 50° were never flown: check the wrist, gripper and cables against the upper arm
    on the bench first.

The floor is one key for both guards. Replayed on the same 69 runs as section 3 (q2-aware rule):

| static / rate threshold | false trips on completed runs | buckles caught (lead s @ q3°) | 10-21 hardware |
|---|---|---|---|
| 10 / 10 (today) | 6 | 0.31@14.4, 0.25@14.6, 0.25@15.6 | trips 8.5 s early, q3 21.6 |
| 5 / 5 | 2 | 0.25@11.2, 0.19@11.1, 0.20@12.1 | trips 0.41 s before the recorded static trip |
| 0 / 0 | 0 | 0.20@7.3, 0.14@7.4, 0.16@8.7 | no trip |

- The static threshold alone changes nothing on these runs: the rate guard always fires first.
- The 10-21 early trip is a swing that turned at q3 12.0°, with q2 at 38.8°, nowhere near the
  wall. The 0.2 s linear look-ahead over-predicts a swing that is about to turn.

**APPLIED 2026-10-24 (user): `push_pull_guard_q3_min_deg` 10 -> 5** (uncommitted, not flown).

> **Isaac found a cost the replay missed (2026-10-24, `../box_pushing_guard_gate_20261024`).**
> Seed 2: a SLOW fold with q2 pinned on its wall (q3 16 -> 5.5° at ~10°/s, the wrist rolling to
> −75°) is not caught at a 5° rate threshold; the vehicle crashed in the abort that followed.
> At 10° the rate rule fires 0.7 s earlier, at q3 11.4°. Recommended: back to 10 (or split the
> key: static 5, rate 10). NOT reverted yet: the user's call.

- Set in the generator `push_pull_20261003/tools/make_push_pull_yaml.py`. Regenerated: the sim
  and hardware whole-body box yamls and the coordinated-admittance box yaml. The diff is the value
  and its comment. The valve yamls keep their own 3°.
- Both whole-body hardware stacks now expect 5.0 on `AM_HW_PROFILE=box_pushing`. The Isaac
  scene check compares it against the running planner.
- No planner rebuild: the value is a parameter.
- Loopback `test_push_pull_loopback.py`:
  - [9h] buckles to q3 2° under the limit (the 10-21 pose, 5°, is no longer below it);
  - [9k] stretches at 120°/s instead of 65, so the rule without the wall still trips at the
    lower limit (the 10-23 stretch at 65°/s may no longer trip it).

  Results: ALL PASS on the sim and the hardware yaml. [9i] fold caught at q3 13.6° (both);
  [9k-c] fold with q2 pinned caught at q3 12.6° (sim) / 14.4° (hardware); [9h] abort and unbuckle
  from q3 3.0°.

**The q3 walls (asked 2026-10-24).** In every layer they are −40° / +50°, except Isaac's physical stop:

| layer | q3 range | what happens there |
|---|---|---|
| hardware arm controller (`external_torque_controller_hardware_aerial_pwm.yaml`) | −40° … +50° | the law's j3 torque is replaced by a pull-back, kp 116.5 N·m/rad, kd 0.745 |
| servo EEPROM (xacro `joint3_min/max_deg`) | −40° … +50° | enforced only in position mode; the box push runs PWM torque mode |
| Isaac arm controller (`torque_controller_isaac_aerial.yaml`) | −40° … +50° | same pull-back |
| Isaac asset `AM_T650.usda` manip_joint3 | −90° … +50° | PhysX hard stop |
| planner + law joint box (`Q_MIN/Q_MAX`, `kQMin/kQMax`) | −40° … +50° | planned poses only |
| planner abort guard (not a wall) | < 5° | abort |

The lower wall (−40°) lies past the singular pose (−34°), so it never protects against the
singularity; the planner's 5° guard does.

**q3 max 60°? (asked 2026-10-24; `tools/q3_upper_usage.py`, the re-centred table in
`joint_limit_options.txt`)**

- **At today's push pose the extra room is on the side the arm does not use.** With the free axis
  from the room hold (shipped 10-22), 22 Isaac runs peak at q3 36.5° median and 46.6° at most;
  none passes 48°. Their low side goes down to q3 11–25°. Only the 10-21 hardware push (before
  that change) reached the wall: 50.4°.
- **The benefit comes from moving the push pose with it.** Same claw height (gear and table
  clearance unchanged), claw moved IN along the arm (floor 5°):

| q3 max | push pose | out | in | 10-23 direction |
|---|---|---|---|---|
| 50 | [0, 32, 28, 0] (today) | 24.0 | 35.5 | 30.5 |
| 50 | [0, 29.2, 32.2, 0] (balanced, claw 6 mm in) | 30.0 | 29.5 | 37.5 |
| 60 | [0, 24.7, 38.7, 0] (balanced, claw 16 mm in) | 40.0 | 38.5 | 48.5 |

  The excursions in the 22 shipped-config runs were ~20–30 mm on each side. Only the 60° row
  leaves margin on both sides.
- **A latent defect, found while checking (NOT fixed).** An abort on the box plans from the
  MEASURED arm (`push_pull_abort_from_measured`), unclamped. The planner refuses a leg whose start
  joints lie outside its joint box (q3 ≤ 50°, `WbReferenceBuilder::kQMax`, `flat_planner.cpp`
  check C12), with or without the unbuckle. The abort then ends INFEASIBLE: the drone holds where
  it was cut and does not climb, and Go To Land Start is refused for the same reason.
  - Today: possible only in the ~0.4° the wall lets q3 overshoot (50.4° on 10-21).
  - With a 60° wall: likely, unless the abort's start pose is clamped into the box or the
    planner's box rises too.
- **To lift q3 to 60°:**
  - bench-sweep the real arm from 50 to 60° (wrist, gripper and cables against the upper arm);
  - raise the hardware and Isaac arm controller `max_position` (and the xacro EEPROM value);
  - raise the Isaac asset stop, through a scene knob like `PEGASUS_PUSH_Q2_LIMIT_DEG`;
  - fix the abort start (above);
  - set the new `push_pull_push_pose_deg`;
  - fly it in Isaac.

## 11. The grasp: what to change (asked 2026-10-24; `tools/grasp_q1.py`, `tools/close_gate_replay.py`)

**Why joint 1 stalled 5–6 mm short (`grasp_q1.txt`).** It is not the 10-21 stiction-learning
mechanism: the arm feed-forward on j1 stayed at −0.004 … −0.018 N·m. The law corrects the 5–6 mm
across error mostly with the base. It asks j1 for only −0.03 … −0.06 N·m (the "task" column),
about j1's static friction, so the joint stays stuck at q1 8.5° until the base's swing carries the
claw through the gate.

**The 3 mm across gate is the simulator's number.**
- The GS comment sizes it for the sim claw: 20 mm grip shim, ~8.75 mm clearance a side.
- The real gripper (10-23 joint states): fingers +19 mm open, −7.5 mm closed on the 5 mm fin, so
  ~58 mm open gap and ~25 mm clearance a side.
- Closing off-centre is benign while the claw is still. The first pad pushes the claw TOWARD its
  world-held target (the fin), and j1's ~0.2 N of stiction at the claw is below the box's
  0.6 N (240 g) / 1.2 N (500 g) of friction, so the arm yields, not the box.

**What the gate should check instead.** Replay on both hardware holds (`close_gate_replay.txt`;
claw/base speed = displacement over 0.3 s):

| gate | 10-23: close after | claw / base speed then | 10-21: close after | claw / base speed then |
|---|---|---|---|---|
| today: abs(e) <= 20, across <= 3 mm | 2.77 s | 35 / 46 mm/s | 0.41 s | 49 / 39 mm/s |
| abs(e) <= 25, across <= 8 mm | 1.26 s | 16 / 26 | 0.11 s | 40 / 42 |
| abs(e) <= 25, across <= 8 mm, claw <= 25 mm/s | 1.26 s | 16 / 26 | 0.81 s | 13 / 37 |

- On 10-23 the wider gate closes 1.5 s earlier, before the sideways swing peaked, at half the speed.
- The speed condition costs nothing there. On 10-21 it waits 0.7 s for the claw to slow from
  40 to 13 mm/s.
- A speed condition ALONE never passes (section 5, item 3d); it only works with the wider position
  gate.
- abs(e) <= 20 also bound on 10-23: the along error is the free-axis anchor's 0.75 x the base's
  along wander, by design.

**APPLIED 2026-10-24 with the speed limit at 30 mm/s, not 25** (`close_gate_strictness.py`: 25 mm/s
was met only 23 % of the 10-21 hold); new box-only GS parameters `push_pull_box_grip_tol` /
`_axis_tol` / `_max_speed` / `_speed_window_s` / `_wait_max_s`; Valve Turning keeps its old gate.
Offscreen harness 69/69 (`../box_pushing_hardware_20261019/tools/gs_grasp_check.sh`, cases C1-C6),
Isaac in `../box_pushing_guard_gate_20261024`. The original recommendation:

**Recommendation (arm GS only, `push_pull_panel.cpp` `gripWhenThere`; no planner/law change):**
1. Hardware across tolerance 3 -> 8 mm, keeping 3 mm in Isaac. The station already switches
   `push_pull_full_close_is_grasp` on `simulation`.
2. abs(e) 20 -> 25 mm, after a bench check that a grasp 25 mm off the fin's centre along it still
   has full pad contact (fin 60 mm long).
3. A claw-speed condition, <= 25 mm/s over 0.3 s, computed in the GS from arrival_error [3..5].
4. A wait limit: if the gate is not met within ~2.5 s of Ready done, do not keep the claw in the
   room hold. In Isaac every 3.0 s wait swung q1 10–17° and three of four failed. Start with
   the existing ABORT (jaws open, nothing held); a back-off-and-retry step would need the planner.
5. Bench: measure the real open pad gap and pad width (the 58 mm is inferred from finger
   positions).

Not recommended: tuning the lateral swing (the structural `T·e_R/k_x` offset, section 4), a j1
breakaway feed-forward (with the 8 mm gate the 5–6 mm stall is inside it), or a speed gate on its own.

## Tools

```bash
bash tools/run_extract.sh                          # bag -> npz/ (the 10-21 extractor)
cd tools
/usr/bin/python3 timeline.py 172853 > ../timeline.txt
/usr/bin/python3 trip_window.py 44.8 50.0 0.04 > ../trip_window.txt   # dense table + exact guard replay
/usr/bin/python3 window_stats.py > ../window_stats.txt               # per-phase motion, 10-23 vs 10-21
/usr/bin/python3 grasp_chain.py 40.2 49.2 0.2 > ../grasp_chain.txt    # base/claw/CoM/force/e_R/box through the grasp
/usr/bin/python3 fold_compare.py > ../fold_compare.txt               # today's fold vs the Isaac buckles
/usr/bin/python3 guard_variants.py 45 > ../guard_variants_q2max45.txt  # guard rules on HW + 67 Isaac runs
/usr/bin/python3 guard_sensitivity.py > ../guard_sensitivity.txt
/usr/bin/python3 guard_margins.py > ../guard_margins.txt             # every guard's peak on both hardware pushes
/usr/bin/python3 settle_gate.py > ../settle_gate.txt                 # would a settled close gate have closed?
/usr/bin/python3 joint_window.py > ../joint_window.txt               # claw room before a q2/q3 limit + the q2 wall in flight
/usr/bin/python3 joint_window_options.py > ../joint_window_options.txt
/usr/bin/python3 sigma_map.py > ../sigma_map.txt                     # singularity margin over q2 x q3
/usr/bin/python3 joint_limit_options.py | sed -n '/=====/,$p' > ../joint_limit_options.txt  # room if limits widen
/usr/bin/python3 guard_thresholds.py > ../guard_thresholds.txt       # q3 guard thresholds 10/5/0 replayed
/usr/bin/python3 q3_upper_usage.py > ../q3_upper_usage.txt         # how close pushes come to the q3 +50 wall
/usr/bin/python3 grasp_q1.py 44.0 48.6 0.2 > ../grasp_q1.txt        # joint 1's torque budget in the open-jaw hold
/usr/bin/python3 close_gate_replay.py > ../close_gate_replay.txt     # close-gate candidates on both hardware holds
/usr/bin/python3 close_gate_strictness.py > ../close_gate_strictness.txt  # is the gate satisfiable? share of each hold + wait from any start
```
`bp_common.py` loads either day's npz (this folder, then `../wb_box_pushing_flight_20261021/npz`).
