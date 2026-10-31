# The 2026-10-31 valve flights: the wrist roll swung, the jaws never closed

User report: in the static-hold flights and in the turn, joint 4 kept oscillating, so the handle could not be
grasped. Data: `docs/experimental_data_ros2_bag/1030 - T650-AM Mail-Box & Valve-Turning-*/` — three valve bags
(`..._hold_20261031_174943`, `..._hold_20261031_175725`, `..._30deg_20261031_180423`). The four mail-delivery bags
in the same folder were not analysed. Whole-body 4-D L1, RAW mocap, flown at fsc_autopilot_ros2 `fa8fc9f` /
fsc_open_manipulator `d4d062f` (the 10-30 safety fixes).

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

## Files

`tools/run_extract.sh` (bags → `npz/`, with the 10-29 extractor) · `vt_common.py` · `timeline.py` (→
`timeline.txt`) · `q4_phases.py` (q4 per phase) · `trace.py` (sampled wrist channel) · `coupled_mode.py`
(amplitude/phase at the swing frequency) · `j4_energy.py` (torque fits vs q, q̇) · `j4_trim.py` / `trim_vs_damping.py`
(trim reconstruction) · `current_check.py` (inconclusive: j4's Present Current is too biased at these levels) ·
`j4_ke.py` (back-EMF constant) · `j4_fix_model.py` (the 10-16 single-joint model with today's numbers: it does NOT
reproduce the swing, since it lacks the airframe coupling; kept, not used to rank) · `gs_valve_damping_check.sh` +
`gs_valve_damping_harness/`.
