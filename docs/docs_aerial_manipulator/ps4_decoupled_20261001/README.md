# PS4 teleoperation on the decoupled rig — Isaac check (2026-10-01)

The §7.21 real-time pad teleoperation, integrated into the DECOUPLED rig (geometric+L1 of Cai et al.
+ position-mode arm) and checked in Isaac at RTF 1 with a synthetic pad. Command.md §7.24 has the
run sequence and the list of changes.

## How the check works

`run_check.sh <tag> [decoupled|wb]` brings the rig up exactly as an operator would (stack → Pegasus
launcher → `ps4_teleop_bringup.py up`), except that the real pad window is suppressed
(`PEGASUS_JOY_DEV_NODE` pointed at nothing) and `tools/ps4_pad_check.py` publishes the DualShock 4
itself on `/uav_0/rc/input` — same message, same axis/button layout, 25 Hz, as joy_node +
gamepad_input. It engages through the **arm station's own** `ps4_remote/set_engaged` service (what
the tab's button calls), then drives one channel at a time and, for each leg, compares the planner's
TARGET change (`whole_body_planner/teleop/state`) with the MEASURED change:

- airframe legs (D-pad, △/✕, □/○): odometry, projected on the heading-frame axes;
- grasp-point legs (left stick, right stick up/down): the planner's `current_ee_body` (FK of the
  measured joints, actual body frame);
- wrist roll and PS: `joint_states`.

A leg passes when the target moved the commanded way by a meaningful amount and the measurement
followed in the same direction by at least half of it. Over the session it also records the
controller mode (leaving DIRECT fails the run), tilt, the error against the reference the decoupled
law is actually flying (`position_controller/reference_direct`) and the reference bridge's
same-motion residual (its vector [23]: the bridge-recovered airframe pose must put the planner's own
EE reference where the planner says it is). Then a dropped pad (1 s silent) and the release.

Prerequisite: the two clean-slate calls (see the script header). Headless by default; the arm
station still opens on `DISPLAY`.

## Results (shiqi-desktop, RTF 1.000 in every 10 s window, robustness plant = the committed `_sim.yaml`s)

Pad rates: the planner defaults (0.20 m/s xy, 0.15 m/s z, 20 °/s yaw, 4 cm/s EE, 20 °/s roll, scale 1.0).

| leg | decoupled d2: target → measured | whole-body w1 (PS4 yaml) |
|---|---|---|
| PS → pad home [0,30,30,0] | 0.5° off | 1.3° off |
| D-pad fwd / left / back / right | +0.504→+0.490, +0.504→+0.490, −0.496→−0.485, −0.504→−0.509 m | +0.504→+0.496, +0.496→+0.498, −0.504→−0.510, −0.504→−0.506 m |
| △ / ✕ | +0.300→+0.300, −0.300→−0.300 m | +0.300→+0.297, −0.300→−0.297 m |
| □ / ○ | +45.4→+45.5°, −44.8→−44.8° | +44.8→+45.0°, −45.6→−45.8° |
| EE left / right | +0.060→+0.062, −0.060→−0.062 m | +0.059→+0.058, −0.061→−0.060 m |
| EE down / up | −0.040→−0.043, +0.040→+0.054 m | −0.040→−0.040, +0.040→+0.044 m |
| EE fwd / back (wall ±1.5–2.4 cm) | +0.014→+0.016, −0.024→−0.028 m | +0.014→+0.013, −0.024→−0.024 m |
| roll + / − | +30.4→+30.7°, −29.6→−30.1° | +30.4→**+20.7**°, −29.6→**−0.7**° (FAIL) |
| dropped pad | disarmed, held (2 mm), live again | disarmed, held (9 mm), live again |
| release | → HOLD | → HOLD |
| session tilt max | 1.8° | 2.9° |

**Repeat d3** (decoupled, after the bring-up gained its transient wait): ALL PASS 17/17 again; D-pad
0.496–0.511 m, roll +30.7/−31.2°, session tilt ≤ 1.8°, |x − ref| ≤ 101 mm, yaw lag ≤ 15.4°,
same-motion ≤ 0.002 mm; the bring-up reported ready 4.6 s after the switch.

**The real pad** (`logs/real_pad_engage.log`, windowed run with the launcher's `joy` window): one
publisher (gamepad_input), the planner and the arm station subscribed, 11.6 Hz idle repeat, worst gap
102 ms; engaging through the arm station went live on it (pad fresh 1, inputs live 1), held 6 mm over 5 s
untouched, released to HOLD.

**Decoupled-rig specifics** (d2): error vs the reference the law flies ≤ 80–93 mm during 0.2 m/s
D-pad moves, 35–50 mm standing; **yaw lags the reference by up to 15.3° while □/○ is held** — the
paper law's ω_d = 0 specialization, (K_ω/K_R)·ψ̇ = (0.35/0.49)·20 °/s ≈ 14°, recovered after release
(+45.5° for +45.4°); the bridge's same-motion residual stayed ≤ 0.002 mm through every teleop leg, so
the conversion is exact for pad-generated references as it is for planned ones. The 374 mm / 4.4°
seen right after the bring-up exited (before engaging) is the geometric+L1 law's own SAFETY→DIRECT
transient; `ps4_teleop_bringup.py up` now waits for the airframe to be still before reporting ready.

**Whole-body wrist roll is stick-slip** (w1), not a wiring fault: the target moved correctly, the joint
lagged, stuck at 12.7°, jumped to 20.7°, and on the return crept 20.7→17.4° in 8 s against a 0.8°
target while q1–q3 stayed within ~2°. The torque-controlled arm's EE-heading task is deliberately soft
(K_ψ ≈ 0.25) and the plant carries config A's j4 gearbox friction (~52 mN·m ×1.05); the arm-side
friction feed-forward relays on MEASURED velocity, so it is zero while the joint is stuck. The
position-mode arm on the decoupled rig tracks the same reference to < 1°. Not tuned here.

Files: `logs/` (each run's stack/Pegasus/bring-up/check/land logs and the Isaac pane), `data/*.npz`
(time series: odom, ref, bridge vector, teleop state, joints, current_ee_body, mode; `legs`).

## 2026-10-02: rehearsal of the hardware configuration (Command.md §7.24.3)

`hw0928` = the HARDWARE controller config (09-28 flown gains) on the flight-identified mirror plant
(`WB_SIM_YAML=rtf_profile_20261001/variants/geometric_l1_mirror_sim.yaml ./run_check.sh hw0928
decoupled`); `tuned1001` = the same with the 2026-10-01 tune (`geometric_l1_tune_20261001/variants/
geometric_l1_mirror_sim_tuned.yaml`). Both 17/17 + dropped pad + release; standing offset 180–195 vs
40–45 mm, D-pad 237–258 vs 54–57 mm, yaw lag 15.7 vs 6.1°, tilt 2.9 vs 3.5°.
`tools/hw_param_readback.sh` starts the control node on the hardware yaml and the planner with the
fused stack's own arguments in a test namespace, and prints what they load.
