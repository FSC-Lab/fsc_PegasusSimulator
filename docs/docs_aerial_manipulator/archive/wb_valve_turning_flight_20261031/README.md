# The 2026-10-31 valve flights: the wrist roll swung, the jaws never closed

User report: in the static-hold flights and in the turn, joint 4 kept oscillating, so the handle could not be
grasped. Data: `docs/experimental_data_ros2_bag/1031 - T650-AM Mail-Box & Valve-Turning-*/` — the afternoon's three
valve bags (`..._hold_20261031_174943`, `..._hold_20261031_175725`, `..._30deg_20261031_180423`, sections 1–6) and the
evening's one (`..._30deg_20261031_211808`, flown on the fixes of sections 3 and 5: sections 7–8). The four
mail-delivery bags in the same folder were not analysed. Whole-body 4-D L1, RAW mocap; the afternoon at
fsc_autopilot_ros2 `fa8fc9f` / fsc_open_manipulator `d4d062f` (the 10-30 safety fixes), the evening at `2787578` /
`dd28bce`.

## 1. What happened

In all three flights Go To Start was clean (q4 still to 0.01–0.15° std). **The wrist began to swing the moment the
Ready descent started** and kept swinging until the operator pressed Abort:

| flight | Ready → abort | q4 range | handle turned by the open jaws | jaws |
|---|---|---|---|---|
| 174943 hold | 44.1 → 67.0 s | −40 … +57° | up to 14.7° | never closed |
| 175725 hold | 28.6 → 40.5 s | −42 … +41° | up to 27.4° | closed by hand at 33.1 s, on nothing |
| 180423 30° | 31.2 → 36.8 s | −25 … +47° | — | never closed |

The swing shape (`tools/q4_phases.py`, `tools/trace.py`):
- ±25–40° at 0.22–0.35 Hz;
- flat ±51°/s plateaus for ~1 s per half-cycle, then sharp reversals.

It is a coupled mode, not the wrist alone (`tools/coupled_mode.py`). Amplitudes at the swing frequency:

| signal | amplitude | phase vs q4 |
|---|---|---|
| q4 | ±23–25° | 0° |
| q1 | ±7–13° | antiphase |
| base sideways | ±33–62 mm | |
| base yaw | ±2–3° | |

With the claw held in the room (the 10-27 room-held Ready), any sideways base motion must be taken up by q1/q4,
and the arm's reaction moves the base back.

**The 10-28 and 10-29 valve flights had the same swing during Ready** (q4 21–72° p-p in 3 s). Two reasons it was
missed then:
- the operator closed the jaws within ~2 s, so the clamp stopped it;
- the 10-29 report blamed the jaws opening at Ready.

Since the 10-30 fix the jaws open at Go To Start, and the swing still starts at Ready. So **the jaw opening was
not the cause.**

## 2. Why: j4 had no damping left

In the pass-through the arm controller cancels the motor's own back-EMF damping (1.28 N·m·s/rad on j4) by design.
What is meant to remain:
- the error damping `passthrough_error_damping_joints` j4 = 0.05, i.e. 0.064 N·m·s/rad;
- the law's soft EE-heading task (`wb_ky_psi` 0.2484 in the valve yaml).

Two defects removed the first, and the room hold turns the second negative.

**(a) The current trim integrated the error damping away** (`external_torque_controller.cpp`, `finalize_command`).
- The error damping is delivered as a cut in the back-EMF feed-forward, so it appears in the motor as a deliberate
  current, −d·Ke·(v − qd_ref)/R.
- j4's 0.2 Hz current trim targeted `tau_clamped * g` alone, so it read that current as an error and integrated
  it out.
- The trim dates from 2026-09-14, the error damping from 2026-10-16; the two were never wired together.
- On the flights the reconstructed trim cancels 80–95 % of the damping whenever it is active
  (`tools/trim_vs_damping.py`: net −0.005…−0.010 N·m·s/rad instead of −0.064). Same picture on the 10-28/29 valve
  flights.
- **On the mock-hardware loopback the old target settles the trim at −3.54 counts against a 3.79-count damping
  duty: 93 % removed.** That number comes from the real controller code (section 4).

**(b) The 10-30 fix that was meant to cover this never reached the arm.** The Valve tab raises j1/j4 to d 0.2 for
the mission (the 10-29 report's item 7). It sent the raise to the station's `controller` parameter. A station
started without `controller:=external_torque_controller` declares that as `position_controller`. All three flights
logged:

> Valve Turning: the arm controller's parameters (position_controller) are not reachable -- the j1..j4 error
> damping is NOT raised for this mission

This was a WARN in the terminal only. The 10-30 harness missed it because it built the panel without the Control
tab that declares `controller`. The bag has one `/parameter_events` message, and the fitted error damping is
d = 0.05 throughout.

**(c) What tipped it, and the law's share** (`tools/j4_energy.py`, `tools/coupled_mode.py`).
- **Static friction:** j4 breaks free at ~0.045 N·m. In hover the law's j4 torque noise already peaks at
  0.03–0.04 N·m.
- **Trigger in 175725/180423:** the room hold engaging (end of Go To Start). The free-axis offset swung 15–33 mm,
  and the 10-30 heading-radius term (item 9) moved the heading target 5–10° within ~0.5 s. The law torque then
  passed 0.065 N·m.
- **Trigger in 174943:** the Ready descent itself.
- **Once j4 slides,** the law's j4 torque in the room hold pushes WITH the motion (+0.02…+0.036 N·m·s/rad). In the
  CoM-anchored phases after the abort it damps (−0.03…−0.05).
- **Ruled out:**
  - the observer feed-forward on j4 (≤ 0.003 N·m);
  - the back-EMF constant: Ke_true 2.586–2.588 vs 2.570 configured, observer gain 0.94–1.00 (`tools/j4_ke.py`).
  So the feed-forward itself is accurate to ~1 %.

## 3. The arm-side fixes (fsc_open_manipulator `omx-torque-control` dd28bce)

1. **`ExternalTorqueController::finalize_command`:** the trim's target is `(tau_clamped + err_damp_duty) * g`.
   - Same `g`, since both are duty, i.e. volts.
   - Bit-identical where d = 0: j1–j3, and every joint in every other config.
2. **`PushPullPanel` (Valve tab):** the damping goes to its own parameter, `push_pull_valve_arm_controller`
   (default `external_torque_controller`), not the station's `controller`. Every failure path is now an ERROR in
   the log and a red line on the tab ("j1/j4 damping NOT raised ... the wrist flies undamped").

With both, j4 in a valve mission carries d 0.2 = 0.255 N·m·s/rad, ~4× the d 0.05 that was meant to fly. The trim
no longer erodes it, and it outweighs the room-held law's +0.02…+0.036.

## 4. Checks (no Isaac: its arm is current-mode, where this key is inert)

**Pass-through loopback** (`open_manipulator_x_custom_controller/test/run_passthrough_corrections_test.sh`, real
controller on mock hardware with the real hardware yaml):
- **ALL PASS**, sections 1–10 unchanged.
- New 8b: with j4's trim on, d 0.2 and a moving reference, the trim settles at **+0.37 counts**, the fix's exact
  fixed point (g−1)·(τ+ed).
- The same test against the old line: **FAIL, −3.54 counts** (the old target's −3.55).

**Arm GS harness** (`tools/gs_valve_damping_check.sh`): the 10-30 harness plus the hardware station's
`controller = position_controller`.
- **PASS** 15/15: the raise reaches the fake `external_torque_controller` and is restored after the mission.
- Against the old panel: **FAIL 3**, the hardware failure reproduced.

## 5. The grip schedule (same day, user's decisions; fsc_autopilot_ros2 `dev_robotic_arm` 2787578)

Two open items from the first pass were decided by the user:

- "the heading target should change only after hold, not before holding";
- "two different sets of control gains, free-flight and interaction, as long as the overall performance is
  better".

Both now hang on one event: **the jaws closing**. The whole-body node reads it from the gripper's own joint state,
so it does not matter whether the Valve tab closed them or the operator did by hand in a static hold.

| | jaws open (free flight) | jaws closed (interaction) |
|---|---|---|
| heading gains K_ψ / D_ψ | **2.5 / 0.93** (the mirror's 10-16 wrist fix, flown on hardware 10-27/28) | **0.2484 / 0.2903** (the valve yaml's; a stiff heading fights the clamped fin, 10-26 Isaac) |
| j4 stuck hold | **on** (pairs with the stiff heading) | off (the valve yaml's) |
| heading-radius term (item 9) | **off** | **from the grasp**: wf·(s − s_grasp)/R; after the release held at the handle's last angle |

Why it should help:
- **A stiff heading holds the wrist.** ~0.45 N·m/rad on q4 against ~0.045 N·m of static friction parks the
  wrist within ~6° of the fin (soft: ~±25°, outside the GS's 15° close gate). It also moves the wrist's own mode
  from ~0.3 Hz, right on the room-held sway, to ~1 Hz.
- **No offset before the grasp.** With the item-9 term off, the heading target no longer moves with the base's
  wander before the grasp, which was the breakaway trigger in two of the three flights.
- **No steps.** The gains blend over 1 s at each edge. The heading term starts at zero at the grasp, and a
  re-grip continues from the held angle.

**Where:**
- Law (`wb_controller`): the heading term's origin and hold, `WbGains::ee_anchor_free_heading_{on,s0,hold,s_hold}`.
  The defaults reproduce the 10-30 term, so parity is unchanged.
- State machine: `wb_grip_schedule.hpp`, pure and unit-tested.
- Client: `stepGrip()`; debug [120..125] = jaws closed, blend, heading state (0 off / 1 from the grasp / 2 held),
  K_ψ, D_ψ, raw gripper position.
- Keys: `wb_grip_state_topic` (empty = off, every other config unchanged), `wb_grip_joint_name`,
  `wb_grip_closed_below`, `wb_grip_blend_s`, `wb_ky_psi_before_grip`, `wb_dy_psi_before_grip`,
  `wb_stuck_hold_j4_before_grip`, `wb_ee_anchor_free_axis_heading_after_grip`.
- Valve yamls (generators; the free-flight set is read from the mirror):
  - hardware: `fsc_open_manipulator/joint_states`, `gripper_left_joint`, closed below 0.0 m (open +0.019, on the
    fin −0.0097);
  - sim: `isaacsim_manipulator/gripper_state`, `gripper_joint`, closed below −0.1 rad (open 0, on the shim
    −0.2…−0.3).

**Checks (no flights):**
- Whole-body/L1 gtests 25/25. Includes a new law test on the parity fixture's states (defaults = the 10-30 term;
  off = no companion, bit-identical u2/u3; s0 = s starts at 0; held is frozen) and a state-machine test
  (debounce, glitch, ramp, grasp latch, release hold, re-grip continuity, stale feed, fade-out, mode switch,
  option off).
- The node started on both valve yamls reads back every key and prints the schedule.
- The raw hardware stack, dry-run on the valve profile: all 80 tuned values OK, and it REFUSES a node binary
  without the schedule. The other profiles are unchanged.
- `fsc_trajectory_planner` rebuilt against the new law, 6/6.

**Not yet validated: whether the overall performance is better.** That needs flights. Isaac can show the law side
(the arm there has j4 gearbox friction but no PWM back-EMF, so it cannot reproduce the hardware's zero-damping
swing).

## 6. Still open

- **d 0.2 on j1/j4 has never flown on hardware.**
  - First check at Go To Start: the arm GS log line `Valve Turning: j1..j4 error damping [0.20 0.00 0.00 0.20]
    for the mission`, and the arm controller's `LIVE passthrough_error_damping_joints -> [0.2, 0, 0, 0.2]`.
  - Then the node's `GRIP: jaws CLOSED ... -> INTERACTION set` when the jaws close.
  - Watch q4 through the Ready descent before closing.

## 7. The evening round (211808, 21:18): the fixes held the wrist — and the base ran away

The 30° turn again, the valve re-placed at (0.442, −0.106) (afternoon (0.507, 0.173)); the planner's hold geometry is
the same in all four flights to the millimetre (body → axis +0.16 / −0.33 m, the FL rotor over the table strip 0.53 m
above its top; `room_hold_compare.py`). It failed faster and harder than any flight before it.

**The fixes did what they were built to do** (`timeline.txt`, `trace2.py`):
- the arm GS logged `j1..j4 error damping [0.20 0.00 0.00 0.20] for the mission (was [0.00 0.00 0.00 0.05])` at Go To
  Start and the restore after the revert; `/parameter_events` carries both sets. The damping duty tracks q̇1 / q̇4 at
  0.22 / 0.26 N·m·s/rad on the flight data (law[28]/[31]) — the trim no longer eats it;
- the grip schedule: jaws closed at DIRECT entry (−0.0086 m, closed since the landing) → interaction set; Go To Start
  opened them at 7.5 s → the free-flight set 2.5 / 0.93 by 8.7 s; heading companion off (offset 0.0 through the hold,
  against 9–15° this afternoon);
- **the wrist did not swing.** q4 followed q1 — q1 + q4 tracked the base's yaw drift, e_ψ ≤ 2.6° — until the arm
  buckled.

**What happened** (receive time; the planner's and the law's topics agree, `/rosout` is ~6 s late early in this bag):

| t [s] | |
|---|---|
| 22.17 | DONE go_to_start, claw 17 mm off. EE anchor → WORLD (1 s blend), free-axis anchor latched (0 → 0.75 by 22.9). |
| 22.5–23.4 | the base rolls right ~1° against a left command and drifts right / back: lateral 54 mm p-p in the 1.2 s hold, \|e_R\| to 162 mrad; q1 −0.4 → −7.4°, q4 0 → +11° (the afternoon holds: 8–24 mm, 44–71 mrad). |
| 23.41 | Ready, automatic (1.0 s): a 3 s **base** descent, the streamed arm reference constant at [0, 5, 25, 0]. |
| 23.5–23.9 | the base keeps running: CoM error −126 / +85 mm by 23.9 (the mocap base 13 cm to the right and 8 cm behind the hold); the arm absorbs it — q1 → −20°, q4 → +26°, q2 7 → 45°, **q3 27 → 0 (23.65) → −33°**: the straight-elbow singularity, then the elbow branch. |
| 23.84 | `1 joint(s) at tau_max` (j3 at 3.0 N·m): joints at the clamp on 41 % of the Ready ticks (debug [51]). 23.91: the allocator trades 2.2 N of thrust, once (0.4 % of the Ready ticks), after q3 had crossed 0. |
| 24.37 | **ABORT — the operator's** (no guard reason in the status), 0.96 s into Ready; "already aborting" ×4. |
| 24.4–26.0 | j2 / j3 swing between ±3 N·m clamps, q1 on its −35° stop from 24.8; the base at 0.06 m (22 cm right of the hold) at 25.1, tilt growing. |
| 26.00 | watchdog 15.1° → emergency abort (the planner's already runs); 26.42 release, claw → CoM anchor. |
| 27.24 | 25.3° → SAFETY; the base at (1.52, −0.36), 1.3 m from the hold in 5 s; the RELEASE reaches the arm 10 ms later; arm home at 29.3; STAB 47 s, landed. |

**No planner guard fired, by design:** `plQ3RateGuard` and the static q3 / sway guards are engaged only with
`pl_at_handle_ || pushing` (node line 1950) — FLYING ready is neither, and the fold ran 23.5–23.9 **during the
descent**. The planner's 10° tilt guard is armed while a step flies, but the tilt passed 10° only at ~26 s, after the
operator's abort.

**Why the base ran — what is measured, and what is not** (`trace3.py`, `residuals.py`, `room_hold_compare.py`,
`window_ab.py`). A first version of this section blamed the arm's error damping and a saturated yaw channel; the
numbers below rule both out as the start of it.

1. **No external force.** F_res = m a − R e₃ u₁ + m g e₃ from the mocap kinematics is flat through the onset (±0.1 N on
   a −1.5 N world-y standing bias that did not change). The sideways acceleration came from the law's own thrust
   direction. PX4-vs-mocap attitude is steady at the onset (roll −0.7°, pitch +4.5° body-fixed, 1.5° more than this
   afternoon's +3.0°, a trim the observer absorbs). Same hold geometry as the afternoon.
2. **The roll loop was already worse before the room hold** (`window_ab.py`). Roll attitude-error std over the last
   3 s of Go To Start: **39 mrad** this evening, 6–15 mrad in the three afternoon flights (whole Go To Start: 38 vs
   13–30; 17:49 reached 30). In the evening's own hover before Go To Start (jaws closed, soft heading) it was 12,
   like the afternoon. The evening differs from Go To Start on, the phase in which both new things were active (the
   stiff pre-grasp heading from 8.7 s, the j1/j4 damping raise from 9.95 s) — and in which the room position also
   differed (valve axis (0.44, −0.11) vs (0.51, 0.17)).
3. **The onset came with the arm still.** From the room hold (22.17 s) to 22.9 s the joints moved < 0.2°, so the
   arm's error damping was zero (law[28]/[31] ≈ 0) — yet the base drifted 54 mm sideways in the 1.2 s hold (afternoon
   8–24 mm). The rotational observer's roll estimate rose **0.125 N·m** in that hold (−0.079 → +0.046; afternoon
   changes ≤ 0.05), and the law's attitude error grew to 162 mrad by Ready: a roll torque the law did not model,
   learned slowly (ω_c,r 0.74 rad/s) while the attitude loop held the vehicle ~1° the wrong way. At 23.4 s the law
   wanted −6° of roll and the vehicle sat at +1°.
4. **The room hold turned the run into arm motion until the reach ran out.** With the claw and its heading held in
   the room, sideways → q1, yaw → q4, along-nose (25 %, the free axis absorbs the rest) → q2/q3. At β = 30° the margin
   to the straight elbow is 2–3 cm; an 11 cm sideways run adds √(0.224² + 0.11²) − 0.224 = 2.6 cm of reach, the 8 cm
   back ~2 cm more → q3 through 0 at 23.64 s, as measured. The afternoon's 180423 came within 9.8° of the same fold
   (113 mm p-p, oscillatory about the hold). The afternoon's undamped ±40° wrist swing, ugly as it was, kept the base
   decoupled from the arm (q3 ≥ 9.8°).
5. **Ruled out:**
   - *the yaw channel* — the yaw demand averaged 0.37 of the 0.654 N·m free budget in the hold (afternoon 0.28–0.30;
     peaks 0.79 vs 0.54–0.59) and 0.29 in Ready, the same as the afternoon. The allocator traded thrust once, after
     the fold. The 41 % in the law's log is joints at the torque clamp — a consequence.
   - *the error damping as the onset* — zero while the arm was still. It and the stiff heading's larger joint-1
     torque (u3_1 rms 0.142 vs 0.067–0.100 N·m in the hold; max 0.57 vs ≤ 0.28 in Ready) may have fed the growth
     once the arm moved; that is not shown.
6. **Not identified: the source of the roll torque.** One flight, two changes at once (stiff pre-grasp heading,
   damping 0.2) plus a different room position. Candidates: the new heading gains acting through the law's arm-
   reaction pre-compensation while the joints are held by static friction (the law cancels a reaction the stuck arm
   does not deliver — its size is not obviously 0.13 N·m on roll), flight-to-flight variation (17:49's Go To Start
   roll std reached 30 mrad), or the room position.

The afternoon's fixes were checked without an airframe in the loop (arm loopback, GS harness). Whatever started the
roll torque, the failure itself is structural: a room-held claw at a near-straight arm with a 2–3 cm reach margin,
during an unguarded step.

**Over the day:** 10-28 — tilt-guard false trips, the sway underneath; 10-29 — the sway reached the arm's limits;
10-31 afternoon — the wrist swung undamped; 10-31 evening — the wrist held and the base ran. Every valve contact or
room hold on hardware has now ended in the arm leaving its range within 2–8 s, by a different route each time.

## 8. What to change

**Applied the same night, Isaac-checked: the base gate, the leash, the descent guard and the guards before the
handle** (`archive/valve_room_hold_guards_20261031/README.md`). The rest below is still open.

Robust to whatever started the roll torque:
- **Gate Ready on the base, not only the claw.** The room hold keeps the claw still while the base runs, so a
  claw-stillness check passes. Ready started with the CoM 67 mm off its reference and moving ~0.14 m/s (afternoon:
  ≤ 24 mm lateral p-p in the hold). A CoM-error / base-speed gate before Ready, or an abort of the room hold above
  ~40 mm, is the earliest signal in this flight.
- **Arm the q3 guards (static + rate) and the sway guard from the start of Ready, not at the handle**
  (`tools/guard_replay.py`, the flown values): the rate guard would have fired at 23.56 s with q3 already at +4.8°
  and falling 58°/s, 0.82 s before the operator; the static and sway guards 0.2 s later, at q3 −16 … −19°. No trip
  in the three afternoon Ready phases. That catches the fold at the singularity, not before it.
- **Reconsider the room hold before the grasp at this pose.** CoM anchor through the descent with a sideways re-aim at
  the live handle (10-27 Isaac: the CoM-anchored descent met the fin 11–16 mm off), or the room hold only from the
  grasp.

To find the roll torque — the next flight changes one thing:
- the evening build with the pre-grasp heading back to soft (`wb_ky_psi_before_grip` / `wb_dy_psi_before_grip`
  0.2484 / 0.2903, i.e. the schedule's heading change off) and the damping raise kept; compare the roll error std at
  the end of Go To Start (39 mrad today, 6–15 in the afternoon) and the observer's roll estimate in the hold. Then the
  reverse if needed.

Lower priority:
- the law does not model the arm's error damping (a correctness gap with j1/j4 at d 0.2; not shown to matter here);
- the standing yaw torque of the loose front-left arm (0.18–0.24 N·m, 28–37 % of the free budget, both rounds).

## 9. Where the roll torque comes from (2026-11-01, after the user's notes on the films and the arm)

The user added two facts: white films went up on a side wall between the afternoon and the evening flights, at least
2 m from the table; and a cracking sound was heard from the arm in a ground test on 10-30, before all of 10-31's
flights. Five checks, all on data already recorded:

1. **The rotors delivered the commanded roll torque normally** (`tools/authority_id.py`). Fit of the gyro's angular
   acceleration on the torque the motor commands produce, 0.5–4 Hz, Go To Start + hold: roll gain 5.5 (lag 76 ms,
   r 0.82) in the evening, 5.9–8.1 on the other six valve flights; roll/pitch ratio 1.3 vs 1.1–1.4. Not a prop or
   motor.
2. **It acts on the airframe, not the arm** (`tools/roll_source.py`). The law's raw per-tick residual (`[62..71]`)
   carries the slow roll part in the base row only; the joint rows show no matching signature.
3. **It is neither a spring nor a damper in roll** (`tools/roll_regress.py`): no fit to roll angle or roll rate. Its
   slow part moves with the sideways CoM error (`tools/roll_phase.py`: −0.76 in the evening, −0.68 on 17:49, with a
   ~0.12–0.13 Hz peak on both).
4. **It is not abnormal** (`tools/roll_baseline.py`, all 20 whole-body flights with an extracted npz since 10-21,
   valve and box pushing). Slow roll residual during Go To Start (0.5 s averages, sd): **0.020–0.097 N·m**, the evening
   **0.064** — fifth of 20. Above it: the 10-24 evening box pushes (0.097, 0.076), 10-31 17:49 (0.070) and 10-24
   151634 (0.065). Every flight shows the same link to the sideways CoM error (−0.23 … −0.80). Section 7's "3x the
   afternoon" is true of those three flights only: 10-28 18:18 had the same roll attitude error over the last 3 s
   of Go To Start (43 vs 39 mrad) and still reached a grasp.
5. **So neither the films nor the crack is needed to explain it**, and both are a poor fit: the same size is there on
   10-21 … 10-29, before either. The data cannot rule out that the crack has since worsened, so the arm inspection
   still stands as a safety item.

What it is: a slowly varying roll couple on the airframe, 0.02–0.10 N·m = a left/right thrust difference of
0.1–0.4 N, 0.3–1 % of the 37 N hover thrust. That is the ordinary level of unsteady aerodynamics for a multirotor
flying through its own recirculated downwash in a room; its exact source cannot be traced from these bags. The
rotational observer (ω_c,r 0.74 rad/s, 1.35 s) follows its slow part with a lag, and the base sways 7–28 mm sd
sideways with it on every flight.

Why it hurt only in the evening: the push was at the large end of the usual range, and its swing reversed just as
the room hold began. The raw roll residual went from about −0.11 to +0.09 N·m in the hold's first 0.5 s while the
arm was still (q1 0.4°, q4 0.0°, unchanged arm commands); the filtered estimate the law uses was still −0.07. For
about a second the compensation pushed the wrong way, the base drifted, and the room-held arm took it up until the
2–3 cm reach margin was gone. A phase with no margin met an ordinary disturbance at an unlucky moment. That is what
section 8's guards address; chasing the roll torque itself is not a fix. The one-change heading test in section 8
is now low priority: a flight with soft pre-grasp heading would not separate a change from this scatter.

## Files

`tools/run_extract.sh` (bags → `npz/`, with the 10-29 extractor; the bag folder is `1031 - …`) · `vt_common.py` (four
bags) · `timeline.py` (→ `timeline.txt`, all four) · `q4_phases.py` (q4 per phase) · `trace.py` (sampled wrist channel)
· `trace2.py` (base pose, joints, law u3, task error, anchors, grip state, arm damping duty) · `trace3.py` (base loop:
attitude error, body torques, observer, allocator, motors) · `residuals.py` (un-commanded force / torque on the base
from the mocap kinematics; the arm's applied − commanded torque projected on the base) · `room_hold_compare.py` (the
four flights over the room-held phase) · `guard_replay.py` (the q3 / sway guards replayed from Ready's start) · `window_ab.py` (attitude error
and arm torques per phase, the four flights) ·
`report_data.py` + `report_template.html` + `build_report.py` (the report page) · `coupled_mode.py` (amplitude/phase at the swing frequency) · `j4_energy.py`
(torque fits vs q, q̇) · `j4_trim.py` / `trim_vs_damping.py` (trim reconstruction) · `current_check.py` (inconclusive:
j4's Present Current is too biased at these levels) · `j4_ke.py` (back-EMF constant) · `j4_fix_model.py` (the 10-16
single-joint model with today's numbers: it does NOT reproduce the swing, since it lacks the airframe coupling; kept,
not used to rank) · `gs_valve_damping_check.sh` + `gs_valve_damping_harness/` · `cmd_compare.py` (thrust / torque commands per phase, seven valve flights) · `roll_source.py` (base vs joint
rows of the raw residual) · `roll_regress.py` (spring / damper / lateral fit) · `roll_phase.py` (cross-correlation with the base
motion) · `roll_baseline.py` (slow roll residual on all 20 whole-body flights) · `authority_id.py` (roll / pitch authority from
motor commands and gyro).
