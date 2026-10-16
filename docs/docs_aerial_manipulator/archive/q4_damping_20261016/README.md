# The wrist roll q4 (j4): its compensation terms, tuned from the 10-15 flights (2026-10-16)

User request: fix the underdamped q4 by tuning the q4 compensation terms on the flight data. Data: the 10-15
pick-and-drop bags (`../wb_pick_drop_flight_20261015/npz/{d1,d2,e1}.npz`). D-1 and D-2 are the first flights
where j4 really moved: the basket swinging, then the empty wrist after the drop (D-1) or after the basket
pulled out (D-2). E-1 (no payload) never left q4 = 0.

## 1. What the flights say about j4

**The electrical side is fine** (`tools/ident_motor.py`, `tools/j4_net_damping.py`; analysis/*.txt).
- The fit V = R·I + Ke·q̇ reproduces j2/j3's bench Ke to 3–4 % (R² 0.999), which validates the method.
- On j4, R and Ke are collinear: its duty is almost all back-EMF feed-forward, so its current is ~∝ q̇. With
  R pinned at the calibration, **Ke(j4) = 2.53–2.54 against the configured 2.570**. The feed-forward
  over-cancels by 1.4 %, about −0.017 N·m·s/rad. It is NOT the "~20 % high" the 2026-09 notes suspected.
- Delivered − intended torque has a slope of +0.01…+0.04 N·m·s/rad against q̇ (slightly with the motion).
- The observer tracks the encoder velocity with gain 0.998–1.007.

**The friction relay is too big** (`tools/j4_friction_id.py`, unloaded slides, gravity-neutral, R² 0.97, both
flights agree):

| | measured on the flights | the arm controller's term |
|---|---|---|
| kinetic friction | 0.0335 N·m | relay `friction_ff[3]` 7.7776 counts = **0.052 N·m** |
| viscous | 0.020 N·m·s/rad | — |
| static | ~0.045 N·m (law torque at breakaway) | — |

On every slide the relay pushed 0.019 N·m more than the friction it is meant to cancel, i.e. WITH the
motion. The loaded fit (D-1 swing) returns a negative inertia, because the basket also swings inside the
jaws; so the model takes the unloaded friction plus the flown 0.51 Hz swing frequency.

**Two flown failures, one mechanism.** The loaded swing: ±10° at 0.5 Hz through the whole carry, pitching
the airframe 1.8° rms. The empty wrist: it sticks for 0.6–3.4 s at ±20…60°, then jumps 60–90° at up to
~47°/s. Inside one hunting cycle (D-2, 34–46 s):
- **While stuck**, the law pulls back with 0.01–0.03 N·m and the L1 joint feed-forward ramps the push up
  until breakaway.
- **Once moving**, the relay adds its full 0.052 N·m in the direction of motion, so j4 slides 70° past
  the target.
- **The only damping** is the law's soft EE-heading task (~0.045 N·m·s/rad), against the loaded
  pendulum's critical 0.13–0.17.

## 2. The model (`tools/j4_model.py`)

Built ONLY from the numbers above, with no fitted knob:
- j4 inertia; stick-slip friction (static / kinetic / viscous);
- the 20 Hz observer on the 0.0879° encoder;
- the law's PD (K 0.05, D 0.045, 8 ms transport) and its slow L1 joint feed-forward (0.3 rad/s, the ramp
  measured while stuck);
- the relay, the back-EMF residual (Ke 2.535 vs 2.570), and the 0.2 Hz current trim with its deadband
  and leak;
- loaded: J 0.025 and a 0.20 N·m/rad pendulum (the flown 0.51 Hz).

It reproduces both flown failures:
- **Loaded:** a 20–22° p-p swing after a kick, self-sustained even with zero external forcing (flown
  19–26°). The relay excess pumps energy in every half-cycle.
- **Unloaded:** stick-slip hunting at 63° p-p, slides up to 54°/s, stuck 53 % of the time (flown 60–90°
  jumps, ~47°/s, 1–3 s sticks).

## 3. Candidates (`analysis/j4_candidates.txt`) and robustness (`tools/j4_robust.py`, `analysis/j4_robust.txt`)

The new knob, d: in the pass-through, the back-EMF feed-forward compensates `v_hat − d·(v_hat − qd_ref)`
instead of `v_hat`. A fraction d of the motor's own damping (Kt·Ke/R; j4 1.276 N·m·s/rad) then stays on,
acting on the velocity ERROR at zero lag. It does not drag a planned move (nothing at v = qd_ref), and
d = 0 is the old code path.

| relay N·m | d | loaded swing | unloaded hunting | 20°/s teleop roll, error at the end |
|---|---|---|---|---|
| 0.052 (shipped) | 0 | 20° p-p, sustained | 63° | 33° |
| 0.030 | 0 | 13° p-p, sustained | 44° | 10° |
| 0.0335 (= measured) | 0.05 | gone | 40° | 1° |
| **0.030** | **0.05** | **gone (< 2° within 1 s)** | **36°** | **2°** |
| 0.030 | 0.07 | gone | 36° | 4.5° |

Robustness, each varied alone: kinetic friction ±25 %, static 0.035/0.060, viscous 0.010/0.030, Ke ±2 %,
law K/D 0.045–0.064, L1 feed-forward 0.2/0.6 rad/s, transport 5 ticks.
- **relay 0.030 + d 0.05 removes the loaded swing (0.0° p-p) in all 14 variations** and parks the wrist
  within 2.3° of target.
- relay 0.0335 (exactly the measured friction) lets the swing back (13.9°) if the real friction is 25 %
  lower: an over-compensating relay is what pumps, so the relay must err LOW.
- d 0.07 fails two variations by parking the wrist off target.

**What it does not remove:** once the empty wrist is stuck more than ~2° off target, the L1 joint
feed-forward still ramps it to breakaway (after ~80 s at 2°, 31 s at 5°; never within 120 s at ≤ 1°). It then
overshoots by about static friction ÷ the law's 0.05 N·m/rad. The hunting is halved (36–41° vs 63–82° p-p),
but damping cannot move that balance point. With the loaded swing gone, the release leaves the wrist
within ~2° of target, so a mission should no longer trigger it. Removing it outright is a law-side question
(the heading task's stiffness, or how the L1 joint channel treats a stuck joint), not an arm compensation
term.

## 4. Applied (fsc_open_manipulator, `omx-torque-control`; not committed)

- `external_torque_controller_hardware_aerial_pwm.yaml`:
  - `friction_ff[3]` 7.7776 → **4.455** (0.030 N·m);
  - new `passthrough_error_damping_joints: [0.0, 0.0, 0.0, 0.05]` (+0.064 N·m·s/rad on j4's velocity error).
  - `back_emf_ke` UNCHANGED (1.4 % is inside the robustness band).
- `ExternalTorqueController` (`external_torque_controller.{hpp,cpp}`):
  - the new per-joint key: declared empty, validated [0, 1] at configure, LIVE-settable, refused in
    current mode;
  - a configure banner per joint with its damping in N·m·s/rad;
  - `law_debug` 28 → 32 (append-only: [28..31] = the term's duty);
  - `ComputedTorqueController` (the PD+ bench law, no pass-through) is untouched.
- `test/test_passthrough_corrections.py` section 8, on mock hardware with the real yaml:
  [28..31] == s·ke·d·qd_ref, j4 only; live 0.2 accepted; 1.5 refused; zero at a resting reference.
  **All checks pass**, sections 1–7 unchanged.
- fsc_autopilot_ros2 `scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator{,_fused}.sh`:
  the arm-side check expects the new `friction_ff` and the new key, and the build probe warns (red,
  non-fatal like the rest of that block) when the arm controller build lacks
  `passthrough_error_damping_joints`.

**On the Orin: pull fsc_open_manipulator and rebuild `open_manipulator_x_custom_controller`.** An old build
ignores the new key, so the wrist would fly with the lower relay but without the damping it was validated
with. NOT flown. Isaac cannot test it: the simulated arm is a current-loop-outcome plant (no back-EMF), and
its arm config is current mode, where the key is inert.

## Files

`tools/ident_motor.py` (R, Ke per joint) · `tools/j4_net_damping.py` (delivered − intended vs q̇, Ke with
R pinned) · `tools/j4_friction_id.py` (unloaded slides) · `tools/j4_model.py` (the model, validation,
candidate table) · `tools/j4_robust.py` · `analysis/*.txt`.
