# 2026-11-03 hardware valve turning: stuck between Go To Start and the turn

User report: one manual abort test and three 30 deg valve turns. In each turn, "after the drone settled, and before it
began to push, it cannot enter the next action". The arm GS screen was recorded on the last flight (`record.mp4`, 19 s).
Bags: `docs/experimental_data_ros2_bag/1103 - T650-AM Valve-Turning-20261103T223048Z-1-001/`. Flown build: the
`valve-isolation-20261102` branch, the 11-03 U push (planner `e614924`, autopilot `6629f88`, arm GS `4496238`).

## What happened, per flight

| bag | what blocked it | how it ended |
|---|---|---|
| 164233 abort test | -- | manual abort in the Ready descent: climbed at once (+0.24 m at 1 s, +0.34 at 1.5 s), tilt <= 3.6 deg, DIRECT kept |
| 164722 | Ready refused 4.6 s ("the claw is still moving"), then started; during the 3 s descent the base drifted 113 mm (CoM error y -35 -> +78 mm), so Ready ended with the claw 91 mm from the grasp point (100 mm behind the fin) | fin-in-the-U guard, 0.11 s after Ready: "fin 27 mm across the claw" |
| 165100 | Ready refused for 43 s: the claw never stayed within 50 mm and below 30 mm/s for 0.5 s. The GS's automatic Ready retried for its 10 s limit (32.2-41.7 s); two manual presses (57.7, 69.0 s) were refused (52/57 mm) | manual abort at 73.6 s |
| 165700 | Ready started at once; it ended with the claw 17 mm from the grasp point, but the hand-over settle (25 mm, 15 mm across, < 30 mm/s for 0.3 s) never passed: the base swung sideways +-18 mm at ~0.4 Hz and carried the claw from -14 to +25 mm across the fin in 0.9 s (GS: "the claw settling above the handle ... ABORT in 5.0 s") | fin-in-the-U guard, 1.0 s after Ready: "fin 28 mm across the claw" |

Nothing hung: every stop was a gate or guard doing what it was built to do. All four aborts climbed at once (the jaws
already read open), tilt <= 5.8 deg, DIRECT kept, the valve unmoved (<= 0.3 deg, <= 0.9 mm).

## The cause: the vehicle wanders above the handle more than the gates allow

Since the 11-02 isolation the valve uses the box push's approach: the claw follows the vehicle (CoM anchor) through the
hold above the handle, the Ready descent and the hand-over settle, and is held in the room only after it has settled.
So the claw carries the vehicle's whole hover wander, against tolerances of 15-50 mm and 30 mm/s.

- **What moves the claw** (`tools/claw_motion.py`, the planner's claw = base + R0 . arm FK): the base's translation.
  Above the handle the claw's 0.3 s speed is median 38-53 mm/s (p90 66-109). Of that, the base is 33-47 (p90 58-94),
  the tilt 9-11 and the arm 0-1. Above the box on 10-27 the same numbers were 14-16 mm/s, and Ready passed at once.
- **What the wander is** (`tools/base_wander.py`, `hold_wander.py`): a slow oscillation with no standing offset.
  165100's 42 s hold: law CoM error mean (-2, -1) mm, rms (31, 26) mm, p-p 147 mm, 0.14-0.3 Hz.
- **The gates against it** (`tools/gate_odds.py`, 165100):
  - The Ready gate (50 mm, 30 mm/s, 0.5 s) was met 12 % of the time, and long enough exactly once (59.8 s, 0.8 s),
    when no Ready request was pending.
  - A settle-like gate (25 mm, 30 mm/s) was met 8.5 % of the time.
  - The claw was more than 25 mm sideways of its mean 43 % of the time.
- **Not new**: valve-pose holds wandered 16-36 mm rms per axis on 10-28, 10-29, 10-31 and 11-01 too. What changed is
  that the claw now follows the vehicle until the settle, so the wander reaches all three steps. The 11-03 fin-across
  guard (25 mm) is armed from Ready's end, i.e. during that settle.
- **Isaac predicted it**: under the hardware-like drift model the shipped approach gates aborted before contact, and
  the 11-02/11-03 drift runs relaxed them to test the contact phase (`../valve_upush_20261102/README.md`,
  `../valve_fin_guard_20261103/README.md`). That was recorded but not raised as a likely hardware blocker before this
  flight.

## Ruled out

- **The valve's mocap**: yaw std 0.2-0.95 deg above the handle, i.e. <= 1.1 mm of fin placement. 165700's body did show
  ~12 deg orientation glitches every 1-3 s, even with the drone 2 m away (37 gate drops, all handled by the gate) --
  check its markers.
- **The arm**: 0-1 mm/s of the claw's speed.
- **The planner's claw estimate**: it reproduces from current_base + current_ee_body to 0.0 mm.
- **The odometry velocity noise** (raw-mocap velocity minus EKF2, 44-71 mm/s rms today vs 12-40 on 10-27): the home-pose
  wander is the same as on other days, so it is not what drives the wander.

## Not settled -> followed up

Why the valve wanders more: see `../wb_hover_wander_history_20261103/README.md` (all 46 flights, split at the 10-29
evening sidewall).
- The extra wander appears only AT THE TABLE SITE and only after 10-29. Mid-field it is unchanged.
- The valve before the wall was as calm as the box.
- It is the vehicle's own tilt, driven by a disturbing moment, not a sideways push.
- It is not flying lower (same height above the table as the box).

## Tools

`tools/run_extract.sh` (bags -> npz, the 10-29 extractor), `timeline.py`, `obj_noise.py`, `claw_motion.py`,
`base_wander.py`, `hold_wander.py`, `settle_trace.py`, `gate_odds.py`, `abort_check.py`. Run with
`PYTHONNOUSERSITE=1 /usr/bin/python3` from `tools/`.

## 192853 (11-03 evening): the wall moved 2 m back -- the hover was calm, the slide-in overshot onto the fin

Bag `.../flight_wb_l1_4d_valve_turning_30deg_20261103_192853-20261104T005627Z-1-001/`, 89 s, 30 deg turn.
Tools: `tools/trace_192853.py`, `tools/geometry_check.py`; the yaw and motor traces were inline.

**1. Above the handle, with the wall 2 m back: calm.** 2.8 s hold.

| | 192853, wall 2 m back | 11-03 afternoon | box days |
|---|---|---|---|
| base speed median | 20-25 mm/s | 33-47 | 18-25 |
| CoM wander | 9.3 mm rms | 16-31 | 9-11 |

- Ready passed 1.1 s after its first try (165100: refused for 43 s).
- One short hold, but it is the A/B the wall question needed.

**2. The abort chain:**
- **Ready's slide-in overshot.** The slide (0.12 m along the push + 0.10 m down, 3 s) brought the claw onto its grasp
  target at 32.5 s, 0.7 s before the reference arrived. It then kept going along the push direction: +16.5 mm past the
  target at 33.0 s, the vehicle overshooting forward by ~20 mm (law CoM error).
- **The bracket met the fin's back edge.** The planned slack is 13 mm; the planner's model gap crossed 0 at 32.85 s.
  The valve started turning at 32.9-33.0 s, and the arm's q3 began to deflect at 33.1 s. The valve then turned 35.7 deg
  in 1.2 s, in the planned direction, before the Turn step ever started.
- **The yaw saturated.** See "Where the yaw demand came from" below: it was the arm's joint 1 pressing the claw
  sideways against the fin, not the push along the turn and not the reference. The vehicle still yawed -8 deg.
- **Then all four motors ran out.** CORRECTED 2026-11-03: the first version of this section said the allocator cut the
  collective 37.7 -> 28.4 N. Wrong -- that was the law's own thrust command. See the same section.
  - the sway guard at 34.28 s (209 mm/s, CoM error 188 mm);
  - the 15 deg watchdog -> the emergency abort (the claw climbing at once);
  - the tilt kept growing to 25 deg -> SAFETY at 34.79 s;
  - SAFETY recovered and held until the pilot landed (84 s).

**3. The geometry, from the flight:**

| quantity | design | measured (planner's FK claw vs the fin from obj_0) |
|---|---|---|
| depth under the fin top | 80 mm | 72-74 mm (the claw 6-7 mm high) |
| across the fin, on arrival | 0 | -4 mm (then -12..-14 as the vehicle drifted) |
| along the push (contact) | bracket touches at model gap 0 | touched at model gap +1..-5 mm |

- **The along-push geometry is right to a few mm.** The bracket met the fin within ~0-5 mm of where the model puts it:
  the joints began to deflect, and the valve to turn, exactly there.
- **The axis offset is NOT confirmed.** The valve's two REST poses, before take-off and after the event, 35.7 deg
  apart, solve to obj_0 -> axis (-0.0944, -0.0090) m vs the CAD (-0.0935, -0.0014): the shaft 7.7 mm along the turn
  direction from the planner's axis. Its radius matches the CAD to 1.3 mm.
  - This contradicts the contact timing unless the valve base shifted ~5 mm during the push or the abort.
  - 10-29 measured 3 mm with the same markers; both crashes hit the valve.
- **Check on the bench**: `valve_turning_v2_20261027/tools/valve_body_check.py` (turn the beam by hand >= 60 deg), and
  make sure the plate cannot move in its corner supports.

**What would have prevented it (none applied):**
- **The 13 mm slack is smaller than the slide-in overshoot (~20 mm).** Options: more slack (`GRASP_BEHIND_CENTRE`
  0.025 -> ~0.040, costs a few degrees of turn), or a slower or shorter slide-in.
- **The yaw margin** is eaten by the standing CW yaw lever (the CW pair 0.4 above the CCW pair at hover). A yaw that
  saturates in contact took the collective with it.

### Where the yaw demand came from (user question: "why does the push use up the yaw authority? is it the reference?")

Tools: `tools/yaw_192853.py`, `tools/yaw2_192853.py`.

**Not the reference.** These were constant from Ready until the abort at 34.3 s:
- the platform heading reference (b1_d, model -180.5 deg = actual -90.5 deg);
- the EE heading reference (b1_de);
- the arm reference q_d.

The abort's heading then started from the measured yaw (-9.5 deg).

**The yaw budget.**
- With the CCW pair at idle, the rotors give at most km (T - 4 f_idle) = 0.64 N.m at the hover collective (36 N); at
  28 N, 0.50.
- Holding off the standing loose-arm lever already took 0.17 N.m in the hover before (28-32.8 s), 0.33 at p95.
- That fast yaw noise is almost all the arm's joint-1 torque, which the law cancels through the rotors (correlation
  0.91).

**The fin pinned the claw sideways.**

| t [s] | claw across the fin [mm] | sideways task error [mm] | q1 [deg] | joint-1 torque [N.m] | yaw command [N.m] | yaw clockwise of the reference [deg] |
|---|---|---|---|---|---|---|
| 32.0 | +6.0 | -3 | -0.3 | +0.08 | 0.32 | -0.2 |
| 32.8 | -10.9 | -5 | -0.3 | +0.08 | 0.30 | 0.5 |
| 33.2 | -12.8 | -11 | -0.3 | +0.23 | 0.52 | 0.6 |
| 33.7 | -14.9 | -28 | -0.4 | +0.32 | 0.70 | 2.4 |
| 34.2 | -13.0 | -70 | -0.4 | +0.64 | 1.26 | 7.4 |

1. **The drift.** Through the slide-in the vehicle drifted left, toward the shaft, at ~25 mm/s. The claw follows the
   vehicle (CoM anchor) until the hand-over settle, so it moved 18 mm across the fin: from 7 mm outside the fin's
   centre to 11 mm inside, just as the bracket reached the fin's back edge.
2. **The pin.** From 32.9 s, while the valve turned 34 deg, the claw stayed 12-14 mm inside the fin, and q1 did not
   move under up to 0.64 N.m. The fin's face was against the side of the U: at 73 mm depth that puts it ~10 mm from the
   claw's centre, in the U's narrow rear part. This is an inference from the model's claw point.
   - **The law's own force reading agrees** (raw F_hat [97..99], 0.5 s low-pass, `tools/yaw2_192853.py`): its
     sideways part grew from 0.1 N (33.0 s) to 0.9 N (33.7) and 2.3 N (34.2), pointing to the vehicle's RIGHT. That is
     the fin pushing the U's right-hand side outward (the claw sat 13 mm toward the shaft, i.e. left, of the fin's
     centre). 2.3 N at the ~0.25-0.3 m from joint 1 to the claw is joint 1's 0.64 N.m.
   - **The fin was also turning relative to the U.** The valve turned +14 / +34 deg while the vehicle yawed -2.4 / -7.7
     deg, so the fin sat ~16 / ~41 deg to the U's axis at 33.7 / 34.2 s. A fin turning counter-clockwise swings its back
     end to the right, into the right rail.
     - CORRECTED 2026-11-03: an earlier version said the U's rear leaves ~20 mm. Isaac measured 4.8 mm from each face
       of a centred fin at this grasp, i.e. ~15 mm. A 40 mm fin turned more than ~13 deg cannot lie inside that along
       its length, so the contact was at a corner near the U's base.
     - The measured positions (mocap fin vs the planner's FK claw point) put the fin just OUTSIDE the right rail. That
       contradicts the rightward force (a fin outside the right rail pushes the claw left), so the FK claw point is
       ~10-15 mm off sideways here (Isaac: 8-16 mm at a similar pose). Which claw surface touched is not resolvable.
   - **A second, independent force estimate: the rotors' yaw balance.** The vehicle turned clockwise while the rotors
     delivered up to 0.75 N.m counter-clockwise (allocator model, from the four motor commands), against 0.17 in the
     hover. The difference over the claw's 0.21 m lead on the CoM is 1.8 / 2.9 N at 33.6 / 34.0 s, the same direction
     as F_hat (1.5 N at 34.0 s); the bench yaw constant overstates the real one (09-26: 0.43-0.78 of bench).
   - Page: `viz/fin_in_u_192853.html` (artifact "Fin Against the U"; data `tools/viz_data_192853.py`, build
     `viz/build_page.py`).

3. **The loop.**
   - The claw's target rides on the CoM, which kept moving left. First it was the drift; then the yaw itself, because
     turning about the pinned claw swings the CoM left 5.8 mm per degree.
   - The sideways error grew, and the law answered with joint 1, the only joint that moves the claw sideways.
   - On a blocked claw that torque moves nothing; its reaction twists the airframe clockwise. The law adds the opposite
     torque to the rotors' yaw command, plus its attitude loop's correction of the growing yaw error.
   - More yaw gives more error, which gives more torque.
   - The observer's yaw estimate went -0.17 -> -0.38 N.m, and its sideways force estimate -0.6 -> -2.1 N: the bearing's
     reaction, cancelled by leaning further into it.

**How the yaw saturation became the sway** (applied = the law's thrust minus the unallocated [55], checked against the
four motor commands to 0.1 N):

| t [s] | law's thrust u1 [N] | applied thrust [N] | roll/pitch NOT delivered [N.m] | tilt [deg] |
|---|---|---|---|---|
| 33.5 | 35.7 | 35.7 | 0 | 2.7 |
| 33.7 | 34.3 | 42.5 | 0 | 3.7 |
| 33.9 | 32.6 | 42.0 | 0.49 | 5.4 |
| 34.2 | 28.4 | 43.1 | 1.61 | 10.9 |
| 34.4 (abort leg) | 24.4 | 42.0 | 0 | 20.6 |

- At 33.7 s the CCW pair reached idle. `wrench_allocator.cpp`'s airmode desaturation then shifted all four motors UP to
  keep the full yaw: +8 N of thrust nobody asked for.
- At 33.9 s the CW pair reached full. With all four motors on a limit there was no roll or pitch left. The tilt grew
  uncontrolled, and the vehicle rose 5 cm on the extra thrust.
- The law read that lift as an upward disturbance (vertical estimate +0.1 -> +5 N) and lowered its own command. The
  rotors could not follow it.
- **This allocator gives yaw priority over roll and pitch.** PX4's own mixer does the opposite and drops yaw first.

**What it points to (nothing applied):**
- The claw's sideways position when the bracket meets the fin. The slide-in reached the fin before the hand-over settle
  checked "across".
- A limit on how hard the arm may push sideways in contact (joint 1, or the sideways EE stiffness). Today the claw
  cannot give way, so the airframe twists instead.
- Yaw as the first axis to give up in the allocator. That trades a few more degrees of yaw for a level vehicle; it is a
  code change and needs an Isaac test.
- The 0.17 N.m standing lever (the loose arm).
