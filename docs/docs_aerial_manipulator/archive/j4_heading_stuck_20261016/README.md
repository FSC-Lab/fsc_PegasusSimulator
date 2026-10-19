# Wrist roll j4: stiffer EE heading + stuck-joint hold on the observer (2026-10-16)

User request, after the 1016 pick-and-drop flight analysis (`../wb_pick_drop_flight_20261016/`): apply both
fixes for the empty-wrist stick-slip hunting — a stiffer heading on q4, and stop the observer winding up a
stuck joint.

## The mechanism (10-16 E-1, `../wb_pick_drop_flight_20261016/analysis/q4_empty_e1.txt`)

- The heading spring on q4 was 0.043–0.057 N·m/rad. Against j4's static friction (~0.035 N·m in flight,
  0.045 in the 10-16 bench identification) that spring alone cannot restart the wrist anywhere within
  ~±35° of its target, so the wrist parks wherever it stops.
- The observer then winds it free. In free flight the 4-D feed-forward's j4 row `d_int_f[9]` is the
  deadbeat estimate low-passed at `ω_x` 0.2072 rad/s (τ 4.8 s). On a stuck joint the deadbeat reads minus
  the applied torque, so the share ramped 13–22 mN·m in ~5 s each time, the total reached ~35 mN·m, and
  the wrist broke loose.
- Once moving, the arm's relay cancels kinetic friction and the stored push takes ~1 s to unwind, so the
  wrist slid 20–40° past the target and re-stuck: −23 / +16 / −25 / +18° every ~10.5 s.
- The existing per-joint cap `wb_u3_estimate_max_j4` 0.06 never bound: it was sized on 09-13 under a
  0.105–0.13 N·m breakaway, now above the re-identified static friction. The share peaked at 23–50 mN·m.

## The two fixes

1. **Stiffer heading.** Through the exact law the heading row lands on q4 almost alone: K_q = 0.178 K_ψ,
   D_q = 0.178 D_ψ at every arm pose (`tools/joint_gains.py`), q1 unchanged (4.33 N·m/rad from the position
   rows). `wb_ky_psi` / `wb_dy_psi` 0.2484 / 0.2903 → **2.5 / 0.93** (ζ 1.2): q4 0.043 → 0.434 N·m/rad.
2. **Stuck-joint hold** (new, observer): `wb_l1_stuck_hold_j1..j4` (j4 only: **true**), `wb_l1_stuck_speed`
   0.02 rad/s, `wb_l1_stuck_filter_s` 0.25 s. While a flagged joint's q̇ (recovered as (T⁻¹ξ) joint rows;
   ξ's own joint rows are not q̇), low-passed at 0.25 s, is below 0.02 rad/s, its rows of the Layer-2 filter
   `d_f` and of the 4-D feed-forward filter `d_int_f` keep their state and resume from it once the joint
   moves. The deadbeat estimate and the predictor keep running, so nothing is stored that a release could
   dump into the joint. Same pattern as `wb_l1_contact_hold_translation`. The low-pass is needed because the
   arm's velocity observer spikes to 3–5.5°/s on one encoder count while the joint is stuck (measured on all
   three 10-16 bags); 0.25 s holds that to ~0.35°/s against real slides of 3–35°/s. `wb_control_debug[115]`
   = held joints (bit j−1, 8 = j4).

Neither works alone (`analysis/j4_sweep.txt`): the hold alone parks the wrist 30° off, the stiffness alone
keeps slipping (amplitude 11° → 2.4° p-p, 4 slips in 80 s at every gain).

## Offline evidence

- **j4 model** (`tools/j4_study.py`, built on `../q4_damping_20261016/tools/j4_model.py`; reproduces the
  10-16 E-1 hunting amplitude 31–40° vs 40–43° flown and slide speed 19–22 vs 19–35°/s, its cycle ~2×
  slower). Both fixes, K_ψ 2.0 / 2.5 / 3.0: no slips, parks 1.8 / 1.5 / 1.3° from a −30° kick, roll
  tracking lag 8.9 → 1.1 / 0.7 / 0.5°, loaded swing none. Robust in all 15 variations
  (`analysis/j4_robust.txt`: static 0.035/0.060, kinetic ±25 %, viscous 0.010/0.030, transport 5 ticks,
  +20 ms on q̇, inertia ×0.7/×1.5, ω_x 0.6, threshold 0.01/0.04, filter 0.10 s): zero slips, parked
  0.6–2.2°. Wrist delay margin on the linear loop (`analysis/j4_margin.txt`): >240 ms shipped, 88 / 76 /
  68 ms at K_ψ 2.0 / 2.5 / 3.0, on top of the ~20 ms already in the loop.
- **Whole vehicle** (`tools/heading_gate.py`: the 09-27 circle bench unchanged — exact Python law + 4-D
  L1 + mirror plant + flight-like feedback, RTF 1): H1b vs K_ψ 2.0 / 2.5 / 3.0 and 2.5 + hold. Circle EE
  error 11.9 / 13.4–13.5 mm, delay 24 / 28 ms pass and 32 ms aborts for ALL (= H1b), gust and robustness
  plant identical, |e_R|, tilt, joint torque peak and ripple, saturation identical to three decimals.
  Heading error 0.33 / 0.31° → 0.35 / 0.24° (showcase / flown circle).

## Implemented

fsc_autopilot_ros2 (`dev_robotic_arm`, uncommitted):
- `wb_l1_observer.{hpp,cpp}`: `WbL1Gains::stuck_hold / stuck_speed / stuck_filter_s`, the hold in both
  branches (4-D and 6-D), `WbL1Estimate::stuck_held`. All flags false = byte-identical.
- the client: the five keys (`WB_L1_PARAM`, both `members()` overloads), a cyan `LAW CHECK: STUCK-JOINT
  HOLD` banner, `wb_control_debug[115]`.
- the parity test: the new gains mapped, `stuck_held` compared per step, and
  `StuckHoldFixtureExercisesBothStates`. The fixture got two rollouts **appended** (`four-d-stuck`,
  `stuck-6d`, scripted q1/q4 velocities through stuck and moving); the nine earlier rollouts are
  byte-identical (a full regeneration moves their last bits by 8.6e-13).
- yamls: hardware 4-D source + its sim mirror (identical change), regenerated: hardware `_pick_and_drop`
  and `_pick_and_place`, sim `_pick_and_drop`. The sim `_pick_and_place` (the paper's comparison, as flown
  10-08/09) keeps K_ψ 0.2484 / 0.2903 and the hold off through `make_pick_place_yaml.py`'s
  `GAIN_OVERRIDES_PICK_PLACE`. Not regenerated: push-and-pull, valve turning, interaction, ps4test,
  robustness, coordinated admittance, modular (their own campaigns; they keep the old heading).
- both whole-body hardware stack scripts (raw + fused): the tuned-law list expects 2.5 / 0.93 and the hold,
  and a FATAL build check refuses a node without `wb_l1_stuck_hold_j4` when the yaml enables it.

Pegasus: `l1_observer.py` (the reference; self-test section 7, 8 checks), `generate_wb_l1_truth.py` (the two
gain sets + `STUCK_SCHEDULE`), `wb_entry_sim.make_l1` accepts the keys (inert defaults).

Tests: `WbParityTest` 4/4, `WbParityJointDiagTest` 4/4, `WbReferenceBuilderTest` 2/2, `WbL1ParityTest` 5/5
(incl. the new rollouts at 1e-8); the full binary 106 pass, 3 fail (`TestRotation.QuaternionToAngleAxisAndBack`,
`TestPolynomial.testQuadratic/testCubic`: random-input math tests that include only `math/` headers, fail on
every run, unrelated). Planner gtests 65/66 (`StraightLine.ParityWithPython` is a stale 09-17 binary of a
test whose source and fixture were deleted).

## Isaac (2 flights, 2026-10-16)

Headless at RTF 1, the full pick-and-drop mission with the 190 g basket, through the exact harness of the
same-day baseline `../lift_lurch_gains_20261016/` (its variant D = the shipped sim pick-and-drop config before
this fix, 6 flights). Variant J = the regenerated sim `_sim_pick_and_drop.yaml` + `pick_place_drop_auto:
false` (D carried the same), `tools/make_variant.py`; runner `tools/run_isaac.sh J1:J J2:J`. The sim node
printed `LAW CHECK: STUCK-JOINT HOLD on joints [0 0 0 1]`.

| | D1–D6 (before) | J1 | J2 |
|---|---|---|---|
| completed | 6/6 | yes | yes |
| lift / release lurch | 87–104 / 121–137 mm | 114 / 136 mm | 83 / 123 mm |
| 1.5 Hz pitch (all DIRECT), peak tilt | 0.20–0.46°, 5.3–7.8° | 0.35°, 7.8° | 0.34°, 6.0° |
| basket in the tray | 17–42 mm, flat | 11 mm, flat | 35 mm, flat |
| q4 slow error (< 0.2 Hz) max, carry | 4.5–5.7° | 1.7° | 2.4° |
| q4 slow error max, empty after release | 1.7–5.9° | 0.7° | 3.1° |
| q4 swing (> 0.2 Hz) rms, carry | 0.26–0.71° | 0.05° | 0.48° |
| observer's j4 share, max | 12–15 mN·m | 4.3 | 20.2 |

(`analysis/lurch_score.txt`, `analysis/wrist_score.txt`, `analysis/basket_swing.txt`.)

- **The stick-slip sawtooth is gone.** Before, q4 crept up to ~6° and slipped back ~5° at 4–5°/s every
  ~6 s through the whole carry (D4). With the fix the slow error halves and nothing cycles.
- **J2's carry motion is the basket, not the heading loop.** It is a steady ±1° at 0.44 Hz that does not
  grow. In every flight the wrist's 0.25–1 Hz motion equals the basket's own roll swing (D 0.21–0.61° vs
  basket 0.26–0.60°; J1 0.03 vs 0.14; J2 0.46 vs 0.43). The swing is lift-excited and already present
  before the fix; J1 happened to have almost none.
- **J2's empty wrist parks 3.1° off**: held (no wind-up), inside the friction deadband. That deadband is
  wider in the sim (j4 friction ~0.055 N·m, config A) than on the hardware (0.035–0.045).
- Vehicle: within the D scatter on every metric. J1's lift lurch (114 mm) is above D's range on one flight;
  J2 (83 mm) is below it.
- The hold is on 99–100 % of the time while nothing moves the wrist (its reference is constant all mission),
  so on this mission the j4 feed-forward row stays at its last moving value.

Hardware-yaml check: the node loaded on the hardware `_pick_and_drop.yaml` (private domain, `/uav_test`) reads
back `wb_ky_psi` 2.5, `wb_dy_psi` 0.93, `wb_l1_stuck_hold_j4` true (j1 false), 0.02 rad/s, 0.25 s, and prints
the banner.

**Not flown on hardware.** On the Orin: pull, then rebuild `fsc_autopilot_ros2` and `fsc_trajectory_planner`
(the planner links `libwb_law.a`). Both hardware whole-body stacks refuse a node without
`wb_l1_stuck_hold_j4`. Things to watch on the first flight: q4 against its reference after the release
(`wb_control_debug[8]` vs `[12]`, and `[115]` = 8 while held), and the 1.5 Hz pitch band at the lift.
