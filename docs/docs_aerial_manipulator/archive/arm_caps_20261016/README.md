# Arm joint torque caps, sized on the flight data (2026-10-16)

User request: make sure all four arm joints have sufficient torque caps throughout the whole flight, from the
latest flight test data, and set appropriate limits.

## The limits that can clip a joint

| layer | where | before |
|---|---|---|
| the flight law's clamp | `wb_tau_max` 3.0 N·m, all joints | above every arm cap: never binds first |
| the arm controller's clamp on the TORQUE TERM | hardware arm yaml `max_effort` (duty counts) | [57.6, 364.9, 192.0, 57.6] = [0.34, 2.44, 1.42, 0.39] N·m |
| the servo's EEPROM clamp on the TOTAL duty (torque term + back-EMF feed-forward + trim) | xacro `joint*_pwm_limit` | [180, 545, 330, 180] |
| heat | — | j2: the 2026-08-26 overload shutdown at ~250 current counts sustained |

## What the flights asked for (`tools/caps_audit.py`, `tools/caps_when.py`, analysis/*.txt)

Latest flights: 10-15 E-1 / D-1 / D-2. Cross-checked on every earlier loaded flight (10-09 PP-1..3, 10-10 PP-4,
10-14 D-1/D-2). The DIRECT torque term is fully logged pre-clamp (`law_debug` [9..12]·κ + [18..21] + [22..25]).

- **j1:** peaks at 77–88 % of its cap in every 10-15 flight, even unloaded. It is the law's arm-yaw channel
  ripple (76–80 % of the variance above 5 Hz; median 0.04 N·m, peaks 0.26–0.30). The SAFETY hold sat ON the
  cap for 0.1–0.3 s at every DIRECT → SAFETY handover.
- **j2:**
  - lift at the pick pose 87 %;
  - fold to the old carry pose 103 % (D-1, 0.03 s clipped);
  - 10-10 PP-4 set-down with the basket clamped on the hat 113 % (0.08 s);
  - PP-2 97 %.
- **j3:**
  - lift at the pick pose 97 %;
  - the fold to the old carry pose [12, 38, 42, 0] with the basket 123 % — **clipped 2.5 s** (D-1);
  - PP-4 set-down 114 % (0.09 s), PP-2 98 %.
- **j4:** torque term ≤ 38 % everywhere. But its TOTAL duty sat at the 180 PWM Limit for 0.1–0.4 s in D-1/D-2:
  the back-EMF feed-forward at ~0.9–1 rad/s during the wrist swing and the empty-wrist hunting (fixed the same
  day in `../q4_damping_20261016`).
- **Heat:** j2's worst 10 s mean current carrying the basket is 244 (D-1), 257 (PP-2), 258 (PP-4) counts — at
  the August overload level. No servo hardware error was recorded in any flight.

## Applied (fsc_open_manipulator, `omx-torque-control`; not committed)

| joint | max_effort (counts / N·m) | worst flown demand → at the new cap | stall current at the cap | servo PWM Limit |
|---|---|---|---|---|
| j1 | 57.6 / 0.34 → **85 / 0.50** | 51 → 60 % | 0.22 A | 180 → **212** |
| j2 | 364.9 / 2.44 → **420 / 2.81** | 413 (PP-4 contact) → 98 %; 374 (10-15) → 89 % | 1.16 A (Current Limit 1.35 A) | 545 → **587** |
| j3 | 192.0 / 1.42 → **260 / 1.92** | 235 (10-15 fold) → 90 %; 220 (PP-4) → 85 % | 0.78 A | 330 → **414** |
| j4 | 57.6 / 0.39 (unchanged) | 22 → 38 % | — | 180 → **189** |

- `external_torque_controller_hardware_aerial_pwm.yaml` `max_effort` → [85.0, 420.0, 260.0, 57.6117], with the
  provenance in-file. All still under `wb_tau_max` 3.0 N·m.
- **The servo PWM Limits are now DERIVED:** `external_torque_hardware.launch.py` computes
  `ceil((max_effort + ke_to_pwm·0.6)·1.1)` from the same yaml (the xacro's own documented rule) and passes
  `joint*_pwm_limit` to the xacro. It prints them at bring-up (`servo PWM Limit j1..j4 = [212, 587, 414, 189]`),
  and it refuses a cap that would need more than the servo's 885 full scale. A cap change now carries its PWM
  Limit with it. The same rule on the old caps gives [182, 526, 339, 189], against the hand-set
  [180, 545, 330, 180].
- The xacro defaults and the three PD+ bench configs are UNCHANGED (their own caps; launches that do not pass
  the arguments keep 180/545/330/180).

Replayed through the new limits (`tools/caps_replay.py`, every flight above): the torque term is clipped for
**0.00 s on every joint in every flight** (was j3 2.51 s and j2 0.03 s on D-1, j2/j3 0.08/0.09 s on PP-4). The
only thing left at a PWM Limit is j4 for 0.02 s (D-1) and 0.05 s (D-2), at the ~1 rad/s swing speeds the j4 fix
removes; there a truncated feed-forward leaves some motor damping on.

Checked:
- `test/test_passthrough_corrections.py`, new section 9: a torque above every cap leaves the duty exactly on
  [85, 420, 260, 57.6], both signs, on the real controller with the real yaml. All earlier sections pass. Run
  on ROS domain 77 beside the running simulation.
- The xacro renders 212/587/414/189 with the derived arguments and the old values without them.
- Both whole-body hardware stacks' arm-side check expects the new `max_effort` and warns if the installed arm
  launch does not derive the PWM Limits.

**Not covered:**
- **j2's heat.** A cap cannot reduce the sustained current of holding the basket (244–258 counts against the
  ~250 shutdown level). The levers are a lighter basket, a pose with less j2 gravity load, shorter loaded
  holds, or the servo's temperature (not recorded).
- **The simulation** clamps every joint at 3.0 N·m and never clipped, so it neither needed nor tests this.
- Not flown.
- **On the Orin:** pull fsc_open_manipulator and rebuild `open_manipulator_x_custom_controller` (it installs
  the yaml and the launch; one build covers this and the j4 change). `open_manipulator_x_description` needs no
  rebuild: its xacro already took the `joint*_pwm_limit` arguments, only a comment changed.
