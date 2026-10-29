# 2026-10-28 hardware valve-turning flights (whole-body 4-D L1)

Two flights of the 90° / 18 s valve turn, both recorded with `ros2 bag record -a`:
- `rosbag2_2026_10_28-18_06_52_Valve_Turning` (tag `180652`)
- `rosbag2_2026_10_28-18_18_33_Valve_Turning` (tag `181833`)

They are in `docs/experimental_data_ros2_bag/1028 - T650-AM whole-body Tracking, Pick&Drop, Valve-Turning-*/`.
`-a` skips hidden topics, so the gripper action status/feedback are absent.

Flown config: fsc_autopilot_ros2 `bf7f14d`, fsc_trajectory_planner `66bc2fa` (both pushed 2026-10-28 12:07).

| setting | value |
|---|---|
| turn | 90° in 18 s, after a 1.5 s contact settle |
| claw / CoM radius | 0.160 / 0.264 m |
| peak yaw rate | 12.3 °/s |
| arm pose | [0, 5, 25, 0] deg |
| impedance | K_y 120, D_y 20.2, K_ψ 0.25, M_y diag(1, 1, 1, 0.05), shaped |
| free-axis anchor | 0.75, latched at the room hold |
| translational observer | ω_c_t 2.0 |
| handle-tilt guard | 2.0° for 0.20 s |

Figure: `fig_flights.png`. Timeline: `timeline.txt`. Tools: `tools/` (run `run_extract.sh` first).

## What happened

Both flights followed the same sequence:
1. Go To Start ended with the claw 14–39 mm above the handle.
2. Ready descended in 3.0 s.
3. The station closed the jaws (fingers at −0.0080 m).
4. The planner aborted with "box tilted 2.7 / 2.5 deg > 2.0 deg":
   - flight 1: 7.67 s into the 18 s turn, valve at +26°;
   - flight 2: 2.12 s in, valve at −7°.
5. The abort released the fin, climbed and folded the arm.
6. Go To Land Start and Land followed, then a pilot STAB takeover and disarm.

Both aborts were clean (DIRECT kept, vehicle tilt ≤ 5°, valve untouched after the release).

## 1. The aborts were the tilt guard reading noise

The guard is the angle between obj_0's z axis now and at the push step's start.

- **Frame artifact.** The two rest poses either side of flight 1's 28° rotation give:
  - the shaft **2.8° off vertical** (the plate is not level);
  - obj_0's z axis **γ = 1.5°** off the shaft.

  A pure rotation θ then reads 2·asin(sin γ·sin(θ/2)): **0.8° at 30°, 2.1° at 90°**. A perfect 90° turn alone exceeds the 2.0° limit. (`tilt_split.py`)
- **Rigid-body noise.** The z-axis scatter of obj_0, in degrees rms (`tilt_check.py`, `tilt_trace.py`):

  | condition | flight 1 | flight 2 |
  |---|---|---|
  | on the ground, drone far | 0.3–0.4 | 1.4 |
  | claw near the valve | ~1.7 | ~1.7 |

  - Spikes reached 4–10°.
  - Flight 2's body was bad before take-off. Its rest z-axis offset jumped 4.3° → 7.2° between the flights, and kept stepping while the drone flew past. This points to a marker or labelling problem.
  - The gate dropped 11–17° orientation jumps at a few-mm position change.
- **No measurable real rocking.** The tilt reading correlates weakly with the forces on the fin (r ≈ 0.2 in flight 1, ≈ 0 in flight 2).
- **Replay** (`guard_replay.py`) reproduces flight 1's trip to the sample (raw > 2° for 0.2 s at +7.67 s). A raw 3–4° limit would not have tripped in either flown window. Note that `push_pull_guard_box_persist_s` is shared with the static q3 guard, so do not lengthen it.

## 2. The valve angle followed the vehicle, not the plan

The valve's error is the free-axis anchor carrying the vehicle's fore-aft wander. On the valve the nose is exactly the turn's tangent, i.e. the valve's free direction. (`wander.py`, `turn_kin.py`)

| | flight 1 | flight 2 |
|---|---|---|
| valve − plan, rms | 5.0° | 10.4° |
| slope against fore-aft CoM error / r (weight 0.75) | 0.71 | 0.70 |
| correlation | 0.96 | 0.97 |
| offset | +3.0° | +7.3° |

- The offset is the drift during the jaw close and the settle: the valve was at +9.5° and +18° while the plan still held 0°.
- The valve moved at up to 51–60 °/s (p95 26–29 °/s), against the plan's 12.3 °/s peak.

## 3. A growing lateral sway in contact (the real limiter)

The vehicle sways laterally in contact, i.e. radially at the fin. (`sway_growth.py`)

| | free hold above the handle (claw room-held) | contact, flight 1 | contact, flight 2 |
|---|---|---|---|
| lateral CoM error, sd | 2–3 mm (flight 1); 17.6 mm (flight 2) | 39.5 mm rms, 164 mm p-p | 11.4 mm rms |
| frequency | — | 0.56 Hz | 0.60 Hz |
| envelope growth | — | ×2 every 5.2 s (20 → 80 mm) | ×2 every 5.4 s |

- 0.56–0.60 Hz is the CoM loop's own mode: √(k_x/m) = √(50/3.75) = 0.58 Hz, design ζ 0.46. In contact it loses its damping.
- The clamped arm absorbs the sway as an antiphase q1/q4 swing of ±21° (q1 limit ±35°).
- The arm also took 25% of the fore-aft wander: q3 fell to 9° in flight 1, and to 6° at flight 2's abort as the base drifted −80 mm. The q3 guard is 3°.
- At that growth rate flight 1 would most likely have met an arm limit before the 18 s turn ended, even without the false tilt trip.
- This is the same family as the box-pushing contact oscillation (0.47 Hz, 2026-10-24/27).

**Contact-only feedback path** (`task_inertia.py`, `render_term.py`):

| natural task inertia at [0, 5, 25, 0] | kg |
|---|---|
| along the nose | 1.47 |
| lateral | 0.47 |
| vertical | 0.70 |

- With the flown M_y = 1, the law injects −(Λ_y M_y⁻¹ − I)·F̂_y into u3: about ±0.5 × the contact-force estimate in every direction.
- Its lateral part is 0.15–0.18 N rms. It moves with the sway (corr +0.63–0.67 with the lateral error, +0.47–0.66 with its velocity), but the data cannot prove the sign of its effect on the base.
- The box-pushing fix (M_y = Λ along one world axis) cannot follow a 90° turn. The yaml's `wb_impedance_shaped: false` (M_y = Λ_y computed live) can, but it has never been flown.

## 4. Friction: the bench value stands

The resisting torque was fitted from τ_claw = r·F̂_t + M̂_z ≈ −τ_f·sign(θ̇) − I·θ̈ (`friction_flight.py`):
- the same filter applied to both sides;
- inertia fixed at the CAD rotor;
- a free bias.

| | estimate | honest error bar |
|---|---|---|
| flight 1 (8.9 s, 25 reversals) | 65–86 mN·m | ±57 mN·m (correlated residual) |
| flight 2 (3.3 s, noisy body) | unusable | — |
| bench free spin (2026-10-27) | 72 mN·m [63, 84] | — |

The flight agrees with the bench but cannot refine it: the force estimate is about twice the friction torque per sample. No correction is needed. The breakaway torque is still unmeasured; the hanging-weight test at the fin would give it.

## Recommendations (none applied)

Before the next flight, configuration and setup only:
1. **Tilt guard**, valve profile only: `push_pull_guard_box_tilt_deg` 2.0 → 4.0. Keep the 0.2 s persistence (the q3 guard shares it).
2. **Level the valve plate and re-create obj_0 with the plate level**, so the shaft and the body's z axis are both within ~0.5° of vertical.
3. **Fix flight 2's marker problem.** Rest noise should be < 0.3° rms, also with the drone hovering at the handle (occlusion). Run `valve_body_check.py`.

Tuning, which needs an Isaac A/B when the sim is free (Isaac does not reproduce the hardware contact mode, so treat it as a safety check):
4. `wb_impedance_shaped: false` (natural task inertia). Cost: the heading channel becomes faster (Λ_ψ 0.008 vs M_ψ 0.05).
5. Possibly a small k_v raise, 12.58 → ~15 (ζ 0.46 → 0.55). k_v 20/25 were unstable in free flight in the 2026-10-18 valve sim.

Left as they are:
- **Free-axis weight.** Lowering it moves the wander into the arm, whose q3 already reached 6–9°.
- **The 90° / 18 s reference.** The valve's motion is set by the wander, not the reference. The 2026-10-27 sim's 12 s turn went 3/6 under drift.

## 5. Isaac screen, same evening (9 flights; `sim/`)

The other session's campaign was paused for this; its chain is still parked on its PAUSE file.

Setup:
- `sim/chain.sh`, the 10-27 harness (`valve_turning_20261015/tools/try_valve.sh`).
- v2 valve at 0.072 N·m (drive), GS-run steps, each run stopped 4 s after the exit.
- Hardware-like drift (`PEGASUS_PLANT_FORCE_NOISE` 0.32 N, 0.12 Hz), seeds 2/3/4.
- Scored with the bag metrics (`tools/sim_contact.py`, `tools/exit_summary.py`): `sim/contact_table.txt`, `sim/exit_table.txt`.

| config | turn completed | exit completed | valve − plan rms (s2/s3/s4) | failure |
|---|---|---|---|---|
| flown law | 3/3 | 2/3 | 4.5 / 5.5 / 7.9° | s2 exit: force guard 6.6 N, left DIRECT (55° tilt) |
| `wb_k_v` 15.0 | 3/3 | 2/3 | 4.8 / 5.2 / 5.6° | s2 exit: tilt guard, left DIRECT, flipped |
| free-axis 0.5 | 2/3 | 2/2 | 3.8 / 2.4 / 3.4° | s4: q3 guard at 19.0 s of 19.5 s (q2 +25°, q3 −3.2°), clean abort |

**Isaac has the mechanism but not the mode.**
- It reproduces the valve-angle mechanism: valve − plan ∝ fore-aft wander, corr 0.91–0.99, slope 0.33–0.60.
- It does not reproduce the hardware's 0.56–0.60 Hz growing lateral mode. Sim wander sits at 0.10–0.16 Hz with no growth.

**Neither gain change is worth flying.**
- `wb_k_v` 15 changes nothing measurable here, because there is no mode for it to damp, and its seed-2 exit was the worst outcome of the screen.
- Free-axis 0.5 buys 30–55 % lower valve error and less fore-aft wander, but it buckled the arm on the worst-drift seed. On hardware q3 already came within 6–9° of its guard at 0.75.

**The exit is the weakest step of the current valve sequence.**
- Mechanism: at the exit step the planner restores the CoM anchor (`restoreAnchor()`, 1 s blend). The open fingers are still around the fin, 25 mm below its top.
  - During the release, the latched free-axis anchor had let the open claw slide 25 mm along the fin's width.
  - The CoM anchor then hands the vehicle's accumulated drift to the claw. On seed 2 that pressed a finger into the fin at 4 → 11 N (PhysX truth) while climbing. The claw truth point sat 10 mm radially inside the fin centre, far from every marker ball.
- Exit failure counts:
  - this screen: 2 of 6 runs with weight 0.75 (both seed 2, both left DIRECT);
  - 10-28 nominal runs: 1 of 2 (mk_full_2);
  - 10-27 runs: 0 of 8, older layout.
- The hardware flights never reached this step.
- Proposed fix (planner, NOT built): keep the claw room-held for the first part of the climb, until the fingers clear the fin top (~0.05–0.06 m), then restore the CoM anchor. This is the 10-18 "open with the claw world-held" lesson, extended into the climb.

## 6. The exit fix and the 4° tilt limit (applied the same night, user's go-ahead)

**Planner** (fsc_trajectory_planner `whole_body_trajectory_planner_node.cpp`, uncommitted):
- New `push_pull_exit_world_held_dz` (default 0 = unchanged) and `push_pull_exit_world_held_max_s` (4 s cap).
- A claw that is room-held when the exit starts stays room-held until the measured claw (FK on the encoders, odometry, PX4 attitude) has climbed `dz`. Then the CoM anchor takes over.
- The hold also ends at the cap, on a confirmed mocap fault, or when the exit stops flying. An abort owns its own anchor.
- A claw already on the CoM anchor is unaffected: the box tab's release-first exit, and box pushing generally.
- Both keys are gate-only.

**Valve yamls** (generator `valve_turning_20261015/tools/make_valve_turning_yaml.py`, sim + hardware regenerated; `make_hw_valve_turning_yaml.py --check` UP TO DATE): `push_pull_exit_world_held_dz` 0.06 and `push_pull_guard_box_tilt_deg` 2.0 → 4.0. Each file changed by exactly those two settings plus their comments.

**Both whole-body hardware stacks:**
- They expect both values; the raw stack's pre-launch checks pass, 62/62 tuned values.
- They refuse a planner build without `push_pull_exit_world_held_dz`. The probe finds the string in the new binary and not in an unrelated one.

**Box pushing unchanged:** the planner loopback reports ALL PASS on both the sim and the hardware box yamls, and the room-held exit line never printed.

**Isaac** (`sim/chain_exit.sh`, the same harness and drift seeds; `sim/exit_table_fix.txt`, `sim/contact_table_fix.txt`):

| run | turn | exit | exit peak tilt | exit peak contact |
|---|---|---|---|---|
| ex_s2a (seed 2) | ✓ | ✓ | 4.0° | 1.5 N |
| ex_s2b (seed 2) | ✓ | ✓ | 3.0° | 1.6 N |
| ex_s3 | ✓ | ✓ | 2.6° | 0.5 N |
| ex_s4 | q3 guard at +8.3 s (q3 −59°, q2 on its 45° wall) | — | — | — |
| *without the fix, seed 2* | ✓ | ✗ | *55° / 176°* | *30 / 14 N* |

- **The hold works as designed.** On seed 2 the claw climbed 60 mm in 1.17 s with ~19 mm of sideways slide (the latched free-axis anchor, along the fin's width), then followed the vehicle. Without the fix it never climbed 30 mm and was dragged 63–70 mm sideways.
- **ex_s4 is independent of both changes.** Isaac's rotor tilt reads 0.00 (a rigid joint), and the hold only acts in the exit. It is the arm-on-its-stops collapse: q3 went from its +50° stop to −59° in ~0.6 s, too fast for the static 3° / 0.2 s guard. Seed 4's turn has now gone 3 of 5 across all configs.
- **Still open:** the q3 rate guard is off for the valve (10-22: it false-tripped completed turns at the box's settings). A q2-wall-aware rate rule (the box's 10-24 design) replayed over the valve runs is the next safety item.

## 7. The night's continued work: new markers, mocap noise, pose, gains, the q3 rate guard

**The redesigned markers (user, 10-28 night).**
- `Valve_Assembly_v2.stl` now puts one marker just past the fin (r 0.193 m), so the markers span 188 mm (was ~120). The rotor, beam and fin are unchanged, so `push_pull_ee_offset` stays.
- The asset was regenerated (`stl_to_usda_valve.py`; the previous one is kept as `Valve_Assembly_v2_markers20261028am.usda`). The centroid is now (93.5, 1.4, 89.9) mm, so `push_pull_valve_axis_offset` becomes **[-0.0935, -0.0014, -0.0899]**.
- Updated: the generator's `MOCAP_PIVOT` (both yamls), both hardware stacks' expected value, `valve_body_check.py`, and the comments and Command.md.
- The arm GS's Axis Offset row reads the planner parameter, so it follows the yaml.
- Isaac: a nominal flight completed; the axis from the new obj_0 lands on the table's centre (0.6755, 0.0004, 0.800); no claw–marker contact.

**Is the contact sway the raw mocap noise? No** (`tools/mocap_noise.py`, `vib_check.py`, `att_noise.py`).

On the 10-28 bags the raw feed is clean: 123 Hz, gaps ≤ 14 ms, no repeats.

| | free hover | contact |
|---|---|---|
| drone mocap position content above 5 Hz (the IMU stays at 0.10–0.13 m/s² in 1–5 Hz, so this is measurement, not motion) | 0.5–1.6 mm | 1.2–5.3 mm |
| apparent velocity noise | 11–17 mm/s | 57–101 mm/s |
| PX4 attitude error, 0.4–0.8 Hz | 0.04–0.06° | 0.23–0.33° (tracks the sway: accelerometer-aided tilt, an amplifier) |

- The noise's share in the sway band is ≤ 0.11 N against ~1.7 N of loop force, and the sway GROWS, so it is not noise-forced.
- Isaac at contact-level noise (pos 1.5 mm, vel 60 mm/s per axis; the law sees 70 mm/s), seeds 2 and 3: both completed, about a third noisier, but no 0.56–0.6 Hz mode (lateral 0.10–0.16 Hz, growth +0.03/+0.04 /s).
- Driver fixes needed for that test, in a copy only (`tools/driver/pl_mission.py`, via the new `PL_PUSH_TOOLS` in `run_valve.sh`): the hover check low-passes the velocity, and a refused DIRECT request is retried.
- The drone's mocap ATTITUDE is noisier than PX4's above 2 Hz (0.48–0.62° vs ~0.20° in contact). Feeding the law the raw mocap attitude would trade a slow error for fast noise.

**Turn arm pose** (`tools/pose_screen_now.py`). The measured arm-absorbed claw displacement over completed turns at [0, 5, 25, 0] is fwd −41…+24 mm and down ±15 mm (p0.5/p99.5).
- No pose (q1 = q4 = 0) keeps q2/q3 inside [-20, 45] / [10, 50] over that cloud.
- The best, [0, 0, 27.5, 0], improves the worst margin by only ~3° (−1.2 vs −4.2°).
- The arm already uses ~45° of q3's window, so the pose is a weak lever.

**Gains** (Isaac, all with the exit hold and new markers; drift seeds 2–4):

| config | completed |
|---|---|
| shipped (ω_c_t 2.0, free-axis 0.75) | 5 of 9 under drift tonight, + nominal |
| `wb_l1_omega_c_t` 2.927 | 1 of 3 |
| free-axis weight 1.0 | 1 of 3 (one buckle in the exit) |

Keep the shipped gains. Exits with the exit hold were 7 of 8 clean (peak contact 0.5–2.4 N); the failure was on free-axis 1.0.

**The q3 RATE guard** (`tools/valve_q3_replay.py`, `valve_q3_pick.py`; 77 recorded valve runs with a grasp):
- The static rule (3° / 0.2 s) caught only 5 of 11 buckles before q3 crossed 0, ex_s4 at −48.5°.
- q2-wall-aware rate rule, look-ahead 0.1 s / 30°/s / q2 wall 45: 9 of 11 before the crossing. Its only false trip was on the non-shipped free-axis 0.5 run.
- Hardware 10-28 margin: 5.8–7.7°. A 0.2 s look-ahead dipped below the limit on flight 2.
- Isaac with it on, drift seeds 2–5: no false trip. Seeds 4 and 5 aborted on the vehicle-tilt guard (10.4 / 10.7°, not arm buckles). Seed 5's abort then reached 34° and left DIRECT, the known "abort from a misbehaving vehicle" limit.
- **APPLIED** to both valve yamls; both hardware stacks expect it (valve now 64 tuned values; box unchanged 62).

**The parallel research thread** (`archive/observer_interaction_study_20261029/README.md`). Neither the disturbance observer nor the interaction-mode logic is the cause; no redesign is proposed.
- Energy split on the bags: the law's lateral command DAMPS the sway; the base's ROLL tracking error (the attitude loop lagging a ~0.28 N·m roll moment at the sway frequency) feeds it. The observer's share is small.
- Hypothesis, not reproduced on its bench: at the 30° fold the q1/q4 pair that absorbs the sway also tilts the clamped claw ~9° about the nose. The fin resists, so the base gets a displacement-proportional roll moment, and the 4-D task has no claw-roll row.
- Ground test: close the jaws on the fin and twist the claw about its axis by hand.

**Tomorrow's valve config** (vs the committed hardware yaml):
- `push_pull_valve_axis_offset` → [-0.0935, -0.0014, -0.0899];
- `push_pull_exit_world_held_dz` 0.06 (needs the rebuilt planner);
- `push_pull_guard_box_tilt_deg` 2 → 4;
- `push_pull_guard_q3_lookahead_s` 0 → 0.1, + `_q3_rate_min_deg_s` 30, `_q2_max_deg` 45.

Gains and pose are unchanged.

**Diagnostic flights for the next session** (no runbook change):
1. **D0, ground.** Level the plate; create obj_0 with the fin along the room's +x; run `valve_body_check.py`. Rest noise should be < 0.3° rms, also with the drone hovering at the handle. Twist the closed claw on the fin by hand.
2. **D1, clamped hold.** Before Plan: `ros2 param set /uav_0/whole_body_trajectory_planner push_pull_push_settle_s 10.0`. The fin is held 10 s before the turn; does the lateral sway grow?
3. **D2.** D1 again with `push_pull_contact false` (live-settable). If the sway persists, the interaction logic is ruled out on hardware.
4. **Optional, needs a yaml edit + relaunch:** `wb_attitude_odometry_correction_rad_s` (PX4 attitude with mocap correcting its slow error).

## 8. The fin rolls with the claw (2026-10-29, `tools/fin_roll.py`)

The user: the printed shaft is not stiff, so the fin may roll when the arm pushes it. Checked on the
10-28 bags. obj_0's orientation with the rotation about the fitted shaft removed, split in the valve's
polar frame at the fin; band 0.35-0.9 Hz rms.

| flight 180652 (flight 2 is too short and its body too noisy to say more) | fin roll about the tangent | centroid z | claw roll (FK) | sideways CoM |
|---|---|---|---|---|
| hovering far from the valve (noise floor) | 0.04° | 0.03 mm | 0.6° | — |
| clamped, 0-2 / 2-4 / 4-6 / 6-8 s after the handle | 0.14 / 0.09 / 0.32 / 0.41° | 0.17 / 0.12 / 0.27 / 0.54 mm | 2.0 / 2.8 / 3.6 / 4.9° | 6 / 17 / 32 / 49 mm |

- **The roll is real.** The marker centroid sits 73 mm out from the shaft, so a roll lowers it. Its
  height tracks the orientation-derived roll: corr −0.86, slope −0.96 mm/deg, against −1.27 for a pure
  lean. This is an independent measurement (position, not orientation).
- **It is driven by the claw's ROLL, not by a sideways push.** It correlates with q1 +0.93, q4 −0.88,
  the claw's FK roll +0.89 and the sideways CoM error +0.90. It does not correlate with the task
  spring's radial force (corr +0.03).
- **The fin rolls only ~7 % of what the claw rolls** (0.29° vs 3.8° rms). Most of the claw's roll is
  taken up between the jaws and the fin, not in the shaft.
- **Order-of-magnitude stiffness:** the wrist-roll joint's commanded torque against the fin roll gives
  3-7 N·m/rad (0.05-0.12 N·m per degree). The roll couple through the clamped fin is ~70 mN·m rms on
  flight 1, and it goes back into the vehicle through the arm.
- **A stiffer shaft would not reduce that couple.** The grip and the shaft are springs in series, so
  stiffening the shaft passes MORE torque per degree of claw roll. What removes it is a handle the
  closed jaws can turn on about the tangent (a round pin/rod), or a grip that slips in roll.
- **Consequence for the 4° tilt guard:** including the slow part, the rotor's real lean while clamped
  reached p95 1.5° / max 2.0° on flight 1 (only the sway part is confirmed by position). Add an
  uncorrected γ = 1.5° (2.1° of artifact at 90°) and the 4° limit has no margin left. **Levelling the
  plate and re-creating obj_0 before the flight is required, not optional.**

## 9. Quick sweep: gains, impedance, grasp pose, turn reference (2026-10-29)

**Result: nothing beats the shipped config. Keep it.** No simulator reproduces the hardware's growing
contact sway, so the sweep was split by what each tool can say.

**Grasp pose, offline** (`tools/roll_coupling_screen.py`): the claw's roll per mm of sideways sway, with
the claw pinned (planner IK), screened over q2/q3. The flown pose gives 0.155 °/mm, which matches the
hardware's 0.10-0.15. The coupling follows the FOLD q2+q3: fold 15 → 0.08 °/mm, fold 0 → 0. Every pose
with less coupling than the flown one loses 24-41° of stop margin on the measured arm motion. No
candidate keeps the margin, and the flown pose itself sits at −5.4°. Not a lever.

**Control gains, offline** (`tools/lateral_roll_model.py`, `tools/gain_rank_model.py`): a linear
sideways + roll model built from `wb_controller.cpp` lines 128-189, with its 2-level attitude
feed-forward, rotor lag, a 25 ms delay, the rotational observer, and a roll spring at the fin acting
on the claw's world roll.
- Free flight it reproduces the known 1.5 Hz roll mode.
- The fin's roll coupling eats the sideways mode's damping (−1.9 → −0.05 1/s as it stiffens), but the
  model never goes unstable. Only the ordering is used.
- **Every gain that adds sideways damping costs the 1.5 Hz mode.** k_R/k_w 2.134/1.567 gives +40 %
  sideways decay, but the 1.5 Hz decay falls 4× and the delay margin drops 32.5 → 27.5 ms (the tune
  Isaac rejected for box pushing).
- k_v 15, k_x 60, k_w 1.5, M_r_d ×0.85 and ω_c_r 1.0 make the same trade at smaller sizes.
- **A handle free in roll restores the full free-flight damping (−1.92 1/s)** and costs nothing.

**Impedance, heading and the turn reference, Isaac** (`sim/chain_sweep.sh`, drift seeds 2/3,
baseline = last night's `qg_s2`/`qg_s3`, whose yaml is value-identical to today's generator;
`sim/contact_table_sweep.txt`, `sim/exit_table_sweep.txt`). Each cell is seed 2 / seed 3:

| | completed | fore-aft sd mm | sideways sd mm | valve vs plan rms | q3 min |
|---|---|---|---|---|---|
| shipped (K_y/D_y 120/20.2, K_ψ 0.25, 18 s) | 2/2 | 27 / 22 | 27 / 12 | 6.0 / 4.5° | 13.4 / 13.3° |
| K_y/D_y 80/16.5 | 2/2 | 25 / 22 | 19 / 12 | 5.0 / 4.3° | 15.4 / **8.6°** |
| K_y/D_y 160/23.3 | 2/2 | **50** / 30 | 31 / 19 | **11.5** / 6.7° | **7.4 / 8.5°** |
| K_ψ/D_ψ 1.0/0.58 | 2/2 | 32 / 33 | 16 / 20 | 6.2 / 7.0° | 10.1 / 7.9° |
| 90° in 15 s | 1/2 | 30 / 34 | 15 / 14 | 6.0 / 6.7° | **4.9 / 2.2°** |

- **K_y 160** is worse on both seeds.
- **K_ψ 1.0** is worse on both seeds: more fore-aft sway, lower q3.
- **15 s**: one q3-rate-guard abort at 75°. The other completed with q3 down to 2.2°, below the 3° guard
  floor.
- **K_y 80** is within the shipped config's seed-to-seed scatter: better on seed 2, a 4.7° lower q3 on
  seed 3. Not adopted on two runs.
- Isaac shows only the drift response and the arm's margins, not the hardware mode.
