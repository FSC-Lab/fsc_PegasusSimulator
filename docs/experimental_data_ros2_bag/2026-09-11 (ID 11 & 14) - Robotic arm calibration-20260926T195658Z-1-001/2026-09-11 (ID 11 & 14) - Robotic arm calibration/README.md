# 2026-09-11 — motor-constant calibration, the SWAPPED units (j1 and j4)

Same protocol as `../2026-09-09` in every respect — inverted rig, PD+ in PWM
Mode 16 on `computed_torque_controller_hardware_aerial_pwm_calib.yaml`,
`max_velocity` 0.5236 (30 deg/s), dither OFF, `read_present_pwm:=true`, same
sweeps, same wrist steps, same loads. The ONLY difference is which motor is in
which bracket. Design: `doc/Motor Constant Calibration.md` §5.4; runbook:
`doc/Command.md` "Swapping servo units between slots".

## Which unit is where — VERIFIED ON THE BUS 2026-09-11

Nothing in a Dynamixel's EEPROM identifies the physical unit (no serial-number
register; Model Number and Firmware Version read the same on all five), so
this table and the marks on the housings are the ONLY record of which motor a
measured kappa belongs to.

| unit | started in slot | ID it carried | now in slot | ID after re-ID | kappa before | kappa here |
|---|---|---|---|---|---|---|
| U1 | joint1 | 11 | joint2 | **12** | never measured | the point of the session |
| U2 | joint2 | 12 | joint1 | **11** | 164.1 +- 1.7 (2026-09-09) | not measurable in slot 1 |
| U3 | joint3 | 13 | joint4 | **14** | 152.7 +- 0.9 (2026-09-09) | J4STEEP — the same-unit cross-check |
| U4 | joint4 | 14 | joint3 | **13** | never measured | the point of the session |

**How the physical mapping was PROVEN, not assumed** (snapshots in this
directory, all with torque off and no bring-up running):

* `after_swap_before_reid.json` — arm limp and inverted. IDs 11 and 14 hang at
  -7.8 and -63.5 deg; 12 and 13 sit near zero.
* `zero_pose_check.json` — arm hand-posed to all-zero. All four within +-3.7
  deg, so **nothing is mis-clocked** (no unit came back a hole round, no
  reversed direction). ID 13 did not move by a single encoder count.
* `mapping_probe.json` — arm released. IDs 11 and 14 returned to within 3-5
  deg of their drooped values; 12 moved 0.7 deg, 13 moved 0.00 deg. Gravity
  has no lever on a vertical yaw axis, so **the two joints it can move are the
  shoulder and the elbow** -> 11 and 14 are in the joint2/joint3 brackets.
* `wrist_probe.json` — only the claw twisted. **ID 13 moved -45.2 deg**, its
  first movement of the session -> 13 is in the joint4 bracket. ID 12 never
  moved under gravity or under the twist -> joint1.
* `after_reid.json` — after `dxl_servo_ids.py --swap 11:12 --swap 13:14`.

Both readings of the droop test are recorded deliberately: a droop and a pair
of hand rotations produce the SAME table, and only the wrist probe separates
them. Do not skip it on the way back.

## Payload: CERTIFIED STANDARD WEIGHTS, no box (operator's call, 2026-09-11)

The 2026-09-09 kit (cardboard box + loose weights) was dropped in favour of
**bare certified standard weights gripped directly**: 50 / 100 / 200 g. Two
things follow, both improvements:

* **The mass uncertainty is gone.** kappa is inversely proportional to the mass
  the fit is told, and 2026-09-09's masses were never weighed -- they had to be
  assumed (96/196 g) when that session was finally scored on 2026-09-10. A
  certified weight removes that channel outright.
* **The grip is repeatable and its CoM is tighter.** Gripper opening at each
  grasp, as a seating check: 200 g read +0.00345, +0.00287, +0.00310, +0.00309 m
  (+-0.3 mm); 100 g read +0.00022 and -0.00009 m. Every block verifies the grip
  state before it starts (`check_grip.py`), which is what caught a mislabelled
  block -- see DISCARD below.

Loads used: `empty`, `100g`, `200g`. The 50 g level was not needed once load
constancy passed at two levels.

## What 2026-09-09 established, for comparison

Rescored 2026-09-10 with `kt_rls_report.py` over the full 9-bag stacks, masses
assumed at the nominal 96 / 196 g:

| slot | kappa [counts/Nm] | implied Kt [Nm/A] | gravity_scale = a/kappa | 96 g alone | 196 g alone |
|---|---|---|---|---|---|
| joint2 (L2)  | 164.1 +- 1.7 | 2.266 | 0.892 +- 0.002 | 176.1 | 165.6 |
| joint3 (L3f) | 152.7 +- 0.9 | 2.435 | 1.143 +- 0.010 | 155.7 | 153.0 |

yaml carries 173.8 / 146.7 and gravity_scale 0.76 / 0.95. Both implied Kt are
now BELOW their measured Ke (2.415 / 2.478), so eta <= 1 — the physical
impossibility flagged in §1 is gone. j3's constancy test passes (1.8 %), j2's
fails (6.3 %); j2's command touches `max_effort` during the 200 g sweeps,
which is the j2-specific suspect.

## RESULTS (2026-09-11)

**The two units the swap existed to measure**, each by two independent methods
-- mechanical (payload torque vs Present Current, `kt_rls_report.py`) and
electrical (Present PWM vs current and speed, `pwm_mode_report.py`):

| unit | in slot today | kappa [counts/Nm] | Kt [Nm/A] | Ke [V*s/rad] | eta = Kt/Ke | R [ohm] |
|---|---|---|---|---|---|---|
| **U1** -> normally **joint1** | joint2 | **154.0** | 2.414 | 2.559 | **0.943** | 5.05-5.26 |
| **U4** -> normally **joint4** | joint3 | **150.5** | 2.470 | 2.623 | **0.942** | 4.80 |

The yaml carries 160 counts/Nm and Ke 2.323 for both, never measured (the j2/j3
mean). So kappa is 4-6 % off and Ke 10-13 % off.

**eta agrees to 0.1 % between two different motors.** That is the campaign's
strongest result and the one that closes its opening question: the CONFIGURED
constants gave eta = 1.13, which is physically impossible on a machine referred
to one shaft (eta <= 1). Measured, both units give 0.94 -- a sensible gearbox
loss, and the same for two units of the same type.

### Constancy tests

| test | joint2 (U1) | joint3 (U4) | criterion |
|---|---|---|---|
| load level, 100 g vs 200 g | **0.1 %** | **0.5 %** | 2 % |
| speed, 30 vs 10 deg/s | 9.0 % -- see below | **1.2 %** | 2 % |
| thermal drift over 4 h | +0.54 counts mean, 3.49 pk-pk | (same bags) | vs 40-85 count signals |

### HONEST UNCERTAINTY: +-2 %, and it is CONDITIONING-limited

kappa moved as pose families were added, in shrinking steps that had not
reached zero:

| joint2 | kappa | | joint3 | kappa |
|---|---|---|---|---|
| L2 only | 143.8 | | L3f only | 146.7 |
| + L2X, L2XX | 150.5 | | + L3fB | 150.5 |
| + L2E (extended) | **154.0** | | | |

The formal +-0.6 is not the real error. Both joints also showed the SAME
failure mode: a load level that has fewer lever geometries than the others
becomes the outlier and drags the fit (j2's 100 g level read 160.0 with one
geometry, 152.6 once it had three; j3's read 140.4 with one pose family,
151.3 with two). **Rule for next time: every load level must have the same
set of geometries, or the stack is not comparable.**

`gravity_scale` is the tell. It is a LINK property and cannot depend on pose,
yet j3's single-pose fit gave 1.333 and the two-pose fit 0.969 -- a 26 % swing
that can only be bias. (This also retracts an earlier claim of mine that the
1.33 corroborated the campaign's old "true gravity ~1.3x model on j3"; that
agreement was coincidence.)

### Why j2's speed test "fails" and the 30 deg/s value still stands

kappa at 10 deg/s came out 137.0 against 150.5 at 30. eta decides it:

| j2 candidate | Kt | eta vs Ke 2.559 |
|---|---|---|
| kappa 150.5 (30 deg/s) | 2.470 | 0.965 -- physical |
| kappa 137.0 (10 deg/s) | 2.713 | **1.060 -- impossible** |

So the low-speed fit is BIASED, not evidence that Kt depends on speed. The
mechanism shows in the same output: the fitted viscous coefficient swings
-35.6 -> -53.4 counts/(rad/s) on j2 and -33.6 -> -71.8 on j3 between the two
speed ranges. A coefficient that changes with its fitting window is a model
that is wrong -- here the missing **Stribeck** term, which the design doc
lists as unimplemented and which is a low-speed effect. j3, with a tenth of
j2's Coulomb friction (fc 0.4 vs 8.9 counts), is admissible at both speeds.

**Operational consequence worth keeping:** the friction model is inadequate
below ~10 deg/s, which is exactly the band the arm FLIES in. That does not
affect the torque constant, but any feed-forward built on `friction_ff` /
`viscous_ff` at flight speed inherits it.

### j2's 6 Hz buzz at q2 = -60..-45, explained

Reported by the operator during the empty blocks. Measured: ~5.9 Hz, 0.46 deg
pk-pk, EMPTY arm only. Model gravity on j2 crosses ZERO at q2 ~ -47 deg in the
L2 geometry, so the gearbox sits in its backlash band with no load -- the same
phenomenon as j1's documented orbit at the inverted home (Key Items 8). With
200 g gripped the payload moves the crossing out of the swept range, which is
why loaded blocks are clean. Benign for the fit; kd was deliberately NOT
touched, since changing it mid-campaign would break comparability.

## The checks this session set out to make, and how they landed

* **"gravity_scale must not move."** Set up as the sharp check on the
  reassembly, since a/kappa is a LINK property. It did not work as a
  cross-session check, for two reasons found during the session: (a) a/kappa
  scales with the ASSUMED payload mass, and 2026-09-09's masses were guessed,
  so the comparison tested the guess; (b) it moves with POSE when the stack has
  too few geometries -- j3's went 1.333 -> 0.969 on adding a second pose. It IS
  a good WITHIN-session diagnostic of conditioning, which is how it was
  eventually used.
* **J4STEEP, the same-unit cross-check -- NOT RUN.** Needs the 80 mm bracket
  (see above). The unit-vs-slot assumption is therefore supported only
  indirectly, by the eta agreement between two units.
* **Ke and R for the j1 and j4 units -- DONE**, and for the first time. They
  fell out of the same bags via `pwm_mode_report.py` with no extra collection,
  because every block records `pwm_state_broadcaster/gpio_states`. The
  2026-09-08 electrical fit had to assume j1's, since j1 never moved in that
  bag. dxl2's Ke read 2.559 in one bag and 2.555 in another, agreeing to 0.16 %.

## Bring-up and clearance walk, position mode (2026-09-11)

`start_open_manipulator_inverted_position.sh`, log `pos_bringup.log`. The
per-ID EEPROM writes at init put every bracket's ORIGINAL range back, which is
the whole reason the re-ID was preferred over an xacro ID remap:

| ID | bracket | Min/Max Position Limit written | = deg | matches |
|---|---|---|---|---|
| 11 | joint1 | 1650 / 2446 | -35..+35 | j1's original |
| 12 | joint2 | 1138 / 2616 | -80..+50 | j2's original |
| 13 | joint3 | 1593 / 2616 | -40..+50 | j3's original |
| 14 | joint4 | 1024 / 3072 | -90..+90 | j4's original |

The j1 anti-hunt gains (Key Items 8) followed the SLOT correctly: ID 11 got
P 400 / I 0 / D 0 / VP 50 / VI 0, the other three the stock 800/100/100. That
matters at the inverted home, which is the near-zero-load pose where j1's
Mode-3 backlash orbit lives.

Homed to [0, 40, 40, 0] over 16.77 s; `joint_states` read j1 0.088, j2 40.08,
j3 39.99, j4 0.000 deg. Operator confirmed the pose visually (upper arm and
forearm both pitched up 40, wrist straight) — the check the numbers CANNOT
make, since a consistent mis-mapping reads back identically.

Clearance walks, 1 cycle each at position mode's 0.2 rad/s, empty claw. All
seven parked back at home within 0.1 deg on every joint, no clamp or limit
warnings, and the only ERROR lines in the entire bring-up log are the three
FastSyncRead ones at init, before the controllers loaded:

| walk | swept | fixed | wrist | s |
|---|---|---|---|---|
| L2   | j2 -78..+30 | j3 +40 | 0   | 92 |
| L2X  | j2 -54..+30 | j3 0   | 0   | 76 |
| L2XX | j2 -42..+30 | j3 -20 | 0   | 68 |
| L3f  | j3 -38..+48 | j2 +10 | 0   | 77 |
| L3f  | " | " | +85 | 117 |
| L3f  | " | " | -85 | 117 |
| L2   | j2 -78..+30 | j3 +40 | +85 | 132 |
| L2   | " | " | -85 | 132 |

Logs: `walk_L2.log`, `walk_<sweep>_<label>.log`.

## Zero check, and the one thing not measured

At the operator's hand-posed all-zero: j1 +3.69, j2 -2.02, j3 -0.97, j4 -1.85
deg -- all within +-3.7, so **nothing is mis-clocked** (a wrong hole would be
45 or 90 deg). The residual is most likely the eyeballed pose rather than the
servos, and a zero offset CANCELS in gravity_scale anyway, since it biases the
gravity and payload columns identically.

**Not measured:** the true servo zero against a level/inclinometer at a
COMMANDED zero pose. Offered twice, not taken, and it is the only calibration
input this session left unquantified. It matters less than it first appeared
(see the cancellation above), but it is the clean way to close it.

## Block inventory

Driven by three NEW scripts (the originals are untouched):
`collect_kt_block.sh` (fixed sweep table), `collect_kt_pose_block.sh` +
`collect_kt_lever_pose.py` (arbitrary lever pose), `collect_kt_sine_block.sh` +
`collect_kt_sine.py` (velocity axis). Grip and temperature are checked before
every block.

| family | swept | fixed | wrists | loads |
|---|---|---|---|---|
| `L2`   | j2 -78..+30 | j3 +40 | 0, +85, -85 | empty, 100 g, 200 g |
| `L2X`  | j2 -54..+30 | j3 0   | 0, +85      | empty, 100 g, 200 g |
| `L2XX` | j2 -42..+30 | j3 -20 | 0, +85      | empty, 100 g, 200 g |
| `L2E`  | j2 -78..+30 | j3 -35 | 0, +85      | empty, 200 g |
| `L3f`  | j3 -38..+48 | j2 +10 | 0, +85, -85 | empty, 100 g, 200 g |
| `L3fB` | j3 -38..+48 | j2 -20 | 0, +85, -85 | empty, 100 g, 200 g |
| `sineJ2`/`sineJ3` | same arcs at 10 deg/s | | 0, +85, -85 | empty, 200 g |

`L2E` is the "fully extended" family added on the operator's request
(2026-09-11). Note the payload LEVER saturates: 300 mm at j3=0, 311 at -20,
313 at -35 -- so extending buys GEOMETRY, not signal. Its value is as a fourth
independent elbow angle, and it moved kappa2 from 150.5 to 154.0.

`DISCARD_collect_kt_L3fB_200g` -- recorded as 200 g with the claw OPEN and
EMPTY (operator had removed the weight between the grip check and the launch);
aborted at 1.5 of 4 cycles. Kept, renamed, with a .WHY file. This is why every
block now verifies the grip immediately before it runs, with a TWO-SIDED test:
fully open (+0.019 m, ~0 counts) is as empty as fully closed (-0.0099 m,
~26 counts), and a one-sided check calls the open case "gripped".

## Loop health and thermal

Over the whole torque session: **2 723 748 cycles, 0 late, max period 5.8 ms,
0 observer resyncs**, `update()` mean 8 us. Joint windings ran 36-52 degC, j2
hottest; the 55 degC gate never fired.

**The gripper (dxl5) reached 63 degC** -- it held a 200-count grip current
continuously for ~4 h. Under the 80 degC limit but the hottest thing on the
arm. Next session: release the grip between blocks.

## Next

1. **Swap the units back**, reversing the procedure: `dxl_servo_ids.py --swap
   11:12 --swap 13:14`, then the WRIST PROBE (a droop and a pair of hand
   rotations give identical tables -- only the wrist twist separates them),
   then one position-mode bring-up to rewrite the per-slot ranges.
2. **Write the constants back as a PERMUTATION.** kappa, R and Ke follow the
   UNIT: U1's numbers become joint1's yaml entries and U4's become joint4's.
   `gravity_scale` stays with the SLOT. Friction (`friction_ff`,
   `viscous_ff`, `friction_load_coeff`) travels with neither -- do not carry
   it across. `kp`/`kd` are in N*m and rescale through kappa, so re-run the EE
   loop and protocol v2 before trusting any tuning comparison across the change.
3. **Build the 80 mm bracket** if the unit-vs-slot assumption is to be tested
   directly rather than inferred from eta.
4. **Consider a Stribeck term** before trusting friction feed-forward at the
   10 deg/s flight bound.

## Current-tracking evaluation (2026-09-11, after the calibration)

Figures: `report/2026-09-11/current_tracking_all.png` (commanded vs measured vs
motor-equation prediction) and `current_error_all.png` (error, six runs).
Script: `scripts/current_tracking_plots.py` (NEW -- `current_error_plot.py`
hard-codes the pre-calibration constants and is left alone).

Sine excitation, one joint at a time, +-12 deg, peak 10 and 20 deg/s
(`sine_torque_test.py`). Bags `eval_sine_{precal,postcal,loop15}_{10,20}`.

| joint | pre-cal, loop off | calibrated, loop off | calibrated + loop 1.5 Hz |
|---|---|---|---|
| j1 | 11 / 11 mA rms | 11 / 13 | 14 / 15 |
| **j2** | 10 / 17, mean +8 | 6 / 8, mean +12 | **4 / 4, mean +0** |
| **j3** | 6 / 8, mean -8 | 7 / 9, mean +7 | **5 / 5, mean +3** |
| j4 | 11 / 18 | 11 / 17 | 17 / 24 |

### THE METRIC IS BLIND TO Kt -- read this before quoting it

Reconstructing the commanded current cancels Kt exactly:

    i_des = (duty - bemf - trim)/NM2PWM/KT = (duty - bemf - trim)*Vs/(PWM_FULL*R)

Verified numerically: i_des is identical for kappa from 146.7 to 200.0. **This
figure tests R and Ke ONLY.** The torque constant is validated by the lever
campaign and by eta = Kt/Ke <= 1, not by this. A claim that the current-error
curve "confirms the calibration" is wrong in the part that matters most.

What it does show: the big j2 improvement (10 -> 4 mA) is the **Ke** correction
(2.139 -> 2.559, 16 % low), whose error is velocity-correlated -- visible as the
pre-cal 20 deg/s trace swinging +-30 mA in sync with the sine and flattening
afterwards. And the loop drives the MEAN to zero, absorbing the residual R
error (my first R for j3, 4.80, was ~6 % high; measured R_eff is 4.34-4.53).

### The current loop: right for j2/j3, wrong for j1/j4

`current_loop_bandwidth_hz` is a SCALAR, so it is all-or-nothing. On j2/j3 it
zeroes the mean error and gives the best rms. On j1/j4, which carry near-zero
current, their current "error" is mostly noise and the trim integrates against
something it cannot fix -- j1 11->14, j4 11->17 mA. **A per-joint bandwidth is
the change worth making before flight.**

It CANNOT be enabled from the yaml: on_configure() refuses a non-zero bandwidth
unless eff_state_idx_ is populated, and that is filled at ACTIVATION. Set it
live after activation, or the controller lands UNCONFIGURED and the arm is limp
with one ERROR line as the only clue.

## joint1 orbits, and it is MECHANICAL

Reported by the operator during the sweeps. j1 shows 0.8-0.9 deg pk-pk at
~3.7 Hz whenever ANOTHER joint moves (j4 at +-12 deg suffices -- the reaction
through the base is the nudge). It is NOT a control instability:

| joint | 3-5 Hz position amp | 3-5 Hz duty amp | duty per degree |
|---|---|---|---|
| **j1** | **0.357 deg** | **3.29 counts** | **9.2** |
| j2 | 0.012 | 3.13 | 251 |
| j3 | 0.001 | 0.39 | 365 |

If the servo were driving j1's 0.357 deg through kp1 14.67 the duty would be
15.5 counts; it is 3.3, and j1's duty-per-degree is 30x lower than j2/j3. **The
link moves without a matching motor command** -- gearbox backlash and compliance
rattled by reaction torque. No gain touches motion inside the backlash band,
which is why kd1 from 0.069 down to 0.0 changed nothing under excitation.

Unit-specific: the joint-1 bracket now holds the OLD JOINT-2 MOTOR, the unit
that carried the largest gravity load on this arm for months -- the most
gearbox wear, the widest backlash.

**Method warning that cost an hour here:** the orbit is BISTABLE and
hysteretic. Once entered it persists regardless of gain, so an A/B that does
not reset the state compares STATES, not gains. Two of my gain sweeps were
confounded exactly that way before the duty-vs-position test settled it. Run
that test FIRST next time: is the servo driving this, or being pushed?

kd1 is left at 0.0 -- harmless in Mode 16, where the motor's own free damping
is Kt*Ke/R = 1.056 Nm/(rad/s) and kd1 0.035 was a 3 % change. The 0.075 in the
old configs came from Mode 0, which has no free damping.

**If joint1 slop matters for flight** (it is the base yaw, so it propagates to
EE heading), the fix is mechanical, not tuning -- and note slot1 holds the most
worn unit while its kappa is only INFERRED. Putting a less-worn unit at joint1
would improve both at once.

## FINAL evaluation and the joint1 resolution (2026-09-11, late)

Figures regenerated to show ONLY the final configuration (operator's request):
`report/2026-09-11/current_tracking_all.png` and `current_error_all.png`,
bags `eval_final_{10,20}`.

Config (now the DEFAULT in
`config/computed_torque_controller_hardware_aerial_pwm.yaml`, so the plain
`start_open_manipulator_inverted_torque.sh` loads it -- no override to forget):
kappa [162.4, 154.0, 150.5, 153.4], R [5.26, 4.90, 4.53, 4.88],
Ke [2.427, 2.559, 2.623, 2.570], gravity_scale [1, 0.897, 0.969, 1],
kp [14.67, 101.5, 116.5, 31.4], kd [0.069, 0.642, 0.745, 0.327],
current_loop_bandwidth_hz_joints [0.0, 1.5, 1.5, 0.0].

| joint | 10 deg/s | 20 deg/s |
|---|---|---|
| j1 | 12 mA rms, mean +1 | 11, +2 |
| **j2** | **7 mA rms, 2.0 % of peak, mean +2** | **7, 2.0 %, +3** |
| **j3** | **7 mA, 2.8 %, mean +4** | **6, 2.6 %, +4** |
| j4 | 11, mean +8 | 15, +8 |

j1/j4 percentages are meaningless -- they carry a few mA, so "51 % of peak" is
a few mA of a few mA.

### joint1's oscillation was LOOSE MOUNTING SCREWS

Chased for over an hour through kd (0.069 -> 0.035 -> 0.0), kp (29 -> 2), the
current loop and the integral. It responded to NONE of them. The operator
tightened the joint-1 screws and it vanished:

| | before | after |
|---|---|---|
| quiescent pk-pk | 0.615 deg | **0.088 (1 LSB)** |
| >1 Hz sd | 0.216 deg | **0.011** |
| spectrum | 4.2 Hz tone | no tone |
| duty per degree | 9.2 | 31 |

**The test that found it, and the one to run FIRST next time:** band-pass
position and duty in the orbit band and compare duty-per-degree against what
the servo would have to produce, kp*err*k_tau. j1 read 9.2 counts/deg against
j2's 251 and j3's 365, i.e. the link was moving with NO matching motor command.
That is mechanical by definition, and no gain controls motion the motor is not
producing. Everything after that measurement was wasted effort.

Second method warning: the orbit is BISTABLE and hysteretic, so a gain A/B that
does not reset the state compares STATES, not gains. Two sweeps were confounded
that way and gave contradictory answers for the same kp.

kd1 is back to 0.069 (the rescaled original) and verified quiet at 0.075 too --
the kd1 = 0 written earlier was compensating for the loose screws.

### Two controller changes made for this (both additive)

* `current_loop_bandwidth_hz_joints` -- per-joint trim bandwidth. Empty means
  "apply the scalar to every joint", so every pre-existing config is unchanged.
  The loop helps a joint carrying real current and hurts one that does not.
* The EFFORT-state precondition moved from `on_configure()` to `on_activate()`.
  It could never pass at configure time (`eff_state_idx_` is filled by
  `map_interfaces()` at ACTIVATION), which made the loop unusable from a yaml:
  the controller landed UNCONFIGURED, the arm hung limp, and one ERROR line was
  the only clue. Now the loop comes up from the file.

### The probe is optional for this analysis

`read_present_pwm:=true` gives Present PWM / Input Voltage, which the
motor-equation prediction needs. Without it the plotter falls back to
`joint_states.effort` for the measured current and the reference topic's effort
for the commanded duty -- everything the ERROR figure needs. Only the dashed
predicted curve is lost.

## Final figures and the j1 ripple investigation (2026-09-11, last)

Four figures in `report/2026-09-11/`, all from `eval_final_{10,20}` only
(operator asked for the latest curves, not the history):
`current_tracking_all.png`, `current_error_all.png`,
`torque_tracking_all.png`, `torque_error_all.png`.

| joint | current rms | torque rms | max_effort | % of envelope |
|---|---|---|---|---|
| j1 | 10 mA | 23 mN*m | +-340 | 6.8 % |
| **j2** | 7 mA | **17 mN*m** | +-2437 | **0.7 %** |
| **j3** | 6 mA | **15 mN*m** | +-1419 | **1.0 %** |
| j4 | 11-15 mA | 27-37 mN*m | +-388 | 7-9.5 % |

**The torque figures do NOT test Kt.** tau_app = Kt*I_meas and tau_cmd = tau_d
both carry the SAME calibrated Kt, so tau_app/tau_cmd == I_meas/I_des exactly
-- the torque plots are the current plots scaled per joint by Kt. They are
worth having because N*m is the unit a tolerance is written in, not because
they add evidence about the torque constant.

### j1's residual ripple: what it is NOT

The operator reported minor oscillation with j1 yawing while j3 swept. Measured
(j1 +-20 deg @ 0.159 Hz, j3 +-30 deg @ 0.106 Hz, 20 deg/s peak each):

| phase | j1 pk-pk | j1 >1 Hz sd | peak |
|---|---|---|---|
| parked | 0.176 deg | 0.0149 | - |
| j1 alone | 20.5 | **0.1659** | 3.9 Hz |
| j3 alone | 0.264 | 0.0326 | 5.1 Hz |
| j1 + j3 together | 20.5 | **0.1657** | 3.9 Hz |

**NOT coupling.** j1+j3 is identical to j1 alone (0.1657 vs 0.1659); driving j3
adds nothing. j1 ripples whenever j1 ITSELF moves. Every other test today drove
one joint at a time, so this had never been checked -- worth keeping as a gap
that was real even though the answer was negative.

Two software levers tried, both rejected on measurement:

* **kd1**, swept 0.069 -> 0.8 during j1 motion. Monotonic improvement to
  0.0877 sd at kd1 0.5 (zeta 0.45), then it turns around. But at 0.5 j1 ENTERS
  a 4.6 Hz orbit after any excursion and stays: 0.0124 -> 0.1316 sd. That is
  the same kd ceiling the 2026-08-27 session found (it settled on 0.075 after
  0.30 and 0.15 both orbited). 29 % during motion is not worth ringing after
  every move. **kd1 stays 0.069.**
* **friction_ff[0] = 2.9** (j1 was the only joint with none; its static
  friction is ~3 counts). Moving ripple 0.1228 -> 0.1256 sd: **no effect**.
  Kept in the config because it is physically justified, explicitly NOT as a
  validated fix.

The likely remainder is mechanical: j1's duty-per-degree is still ~2.4x below
what its gains predict relative to j2, i.e. some link motion without a matching
motor command, in the most-worked unit on the arm. For scale, 0.12 deg sd of
base yaw is ~0.6 mm at the end-effector.

### Earlier in this session: joint1 read -363 deg

Caught by the audit before any bring-up. The multi-turn position register had
accumulated a full negative turn while the arm was worked on by hand. Bringing
up from there would have commanded a ~328 deg unwind of the base with the servo
cable wrapped. The operator rotated it back by hand. **Read the position before
every bring-up after mechanical work** -- `dxl_servo_ids.py` with no arguments
does it in one line, and in Mode 16 nothing in firmware would have stopped it.

## joint4 tuning, and why its peaks cannot be tuned away (2026-09-11, last)

Figures in `report/2026-09-11/` now take joint4's panels from the retuned runs
(`eval_j4dith_14` at 10 deg/s, `eval_j4final_20` at 20) while j1/j2/j3 keep the
curves they were measured with -- `current_tracking_plots.py --joint-from`.

### The one real win: friction_ff[3] 21.22 -> 7.78

| run | config | j4 rms | mean | torque |
|---|---|---|---|---|
| A | ff 21.2 (old) | 11.2 mA | 7.9 | 27.2 mN*m |
| B | ff 7.78 run 1 | 8.6 | 7.6 | 20.8 |
| C | ff 7.78 run 2, SAME settings as B | 8.9 | 7.2 | 21.6 |

B and C agree to 0.3 mA; the A->B step is 2.4 mA, 8x larger. So the improvement
is real, ~22 %. 21.22 belonged to the OLD j4 unit; 7.78 is the identified
Coulomb friction of the unit now in slot 4, and friction_load_coeff[3] = 0 so
j4's friction is pure Coulomb and follows the UNIT. 21.22 was also 74 % of
j4's max_effort, flipping sign at every reversal.

### The peaks are the CURRENT-SENSOR NOISE FLOOR

At stalled samples I = V/R must hold exactly, so any mismatch is measurement.
Fitting a first-order sensor filter to Present PWM -> Present Current:

| joint | best tau | correlation | residual |
|---|---|---|---|
| joint2 | 5 ms | **0.973** | 3.35 counts |
| joint4 | 2 ms | **0.579** | 7.59 counts (tau=0 gives 8.13) |

j2's stalled current is explained by Ohm's law plus a 5 ms filter to 0.97. j4's
is not explained at any tau -- it is noise. j4 runs at a few current counts, j2
at 100+, and the sensor floor is the same for both.

Three fixes failed for that reason, each tested:

* **current trim ON for j4** -- halves the bias (mean 8.1 -> 3.4 mA, exactly
  what a trim does) but more than doubles the variance (sd 7.8 -> 16.9). It is
  integrating sensor noise. Left OFF for j1 and j4.
* **dither swept 14.4 / 7 / 0** -- the 100 Hz content in the commanded duty
  scaled 4.32 / 3.38 / 1.33 counts, so the dither WAS acting; the error metrics
  moved by under 1 mA and tracking was identical to 0.0003 deg. Not the cause.
* **friction_ff** -- accounts for only ~20 % of the reversal excess.

**Remaining: a +7 mA velocity-dependent mean** (-2.6 mA at rest, +6..+11
moving) = back_emf_ke ~20 % high on that slot. Never measured there -- it is
the j3 unit's value from another slot and day, and j4 draws too little current
to identify its own Ke in place. Needs J4STEEP + the 80 mm bracket, the same
missing hardware that blocks the direct kappa cross-check.

### Method note, earned twice in one session

**Run a REPEAT of the unchanged configuration before claiming any delta, and
keep ONE metric.** Two j4 conclusions here were wrong before that control was
run: "no effect" came from comparing sqrt(mean^2+sd^2) against a plain sd over
a different window, and a "65 % improvement" came from a comparison whose
baseline had been moved by a second simultaneous change. Same family of error
as the bistable-orbit A/Bs earlier in the day.
