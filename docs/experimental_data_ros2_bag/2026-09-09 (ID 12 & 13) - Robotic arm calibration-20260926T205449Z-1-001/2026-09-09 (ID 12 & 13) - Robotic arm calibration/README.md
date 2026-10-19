# 2026-09-09 — motor-constant calibration campaign (Kt / gravity_scale split)

Inverted rig, PD+ in **PWM Mode 16** on
`computed_torque_controller_hardware_aerial_pwm_calib.yaml`, `max_velocity`
0.5236 rad/s (**30 deg/s = 3x the 10 deg/s flight bound**, the user's number),
dither OFF, `read_present_pwm:=true`. Loop health over the session: **1 012 564
cycles, 0 late, 0 observer resyncs**, max period 7.6 ms, `update()` mean 9 us.

Design and reasoning: `src/fsc_open_manipulator/doc/Motor Constant Calibration.md`.
Drive: `utils_calibration/scripts/collect_kt_lever.py`.
Score: `scripts/kt_rls_report.py` (fit), `scripts/kt_block_quicklook.py` (per block).

## Why the blocks are shaped this way

`kappa = 1/(Kt*I_LSB)` (= `nm_to_effort_joints`) and the payload's centre of mass
are **degenerate within one geometry** — measured on this arm at 0.0 % of the
payload signal outside the CoM span. Two things break it, both used here:

* **wrist steps** (j4 = 0 / +85 / -85). Rotating j4 leaves the payload torque
  alone but rotates the direction an unknown CoM acts in, so one CoM cannot mimic
  the payload at all three angles. 25 % (j2) / 55 % (j3) outside the span.
  **Any PAIR of angles is still degenerate** — visible in the fit, where kappa
  stays at -1600 through two angles and resolves the moment the third lands.
* **lever shapes** (j3 held at +40 / 0 / -20 for the j2 sweeps) — a second
  geometry, 23.6 % on its own, and an independent cross-check.

## Blocks

| prefix | swept | fixed pose | wrist angles | masses |
|---|---|---|---|---|
| `L2`   | joint2 -78..+30 | j3 +40 (folded)       | 0, +85, -85 | empty, 100 g, 200 g |
| `L2X`  | joint2 -54..+30 | j3 0 (extended)       | 0, +85      | empty, 200 g |
| `L2XX` | joint2 -42..+30 | j3 -20 (past straight)| 0, +85      | empty, 200 g |
| `L3f`  | joint3 -38..+48 | j2 +10                | 0, +85, -85 | empty, 100 g, 200 g |

Start angles are chosen so the tool never reaches more than 110 mm behind the
mount (the 2026-08-17 session walked 90 mm); the extended shapes hang the tool
223-387 mm below the plate, deeper than the folded one.

`collect_kt_L2_empty_shakedown` is a 1-cycle dry run, not for fitting.

## Session checks

* **Drift.** `collect_kt_L2_empty_close` repeats `collect_kt_L2_empty` after
  90 min and +12 degC on j2: direction-averaged current agrees to **+0.6 counts
  mean, 3.3 pk-pk**, against an 85-count payload signal. The mount did not creep.
* **Grip.** Verified by MOTION, not by a hold: at home a 100 g payload moved j2's
  holding current by 1.5 counts against 43 predicted (it disappears inside the
  +-37-count stiction cone), while the same weight read moving and
  direction-averaged gave 41.9/40.7/40.7 vs 41.8/42.6/42.9 predicted. The 200 g
  read 192-197 g the same way.
* **Clamping.** j2's command touches `max_effort` (365 duty) during the 200 g
  sweeps — the kp term fighting a ~0.8 deg tracking error, not gravity, which
  peaks at 0.93 N*m. Costs tracking, not validity: the fit compares measured
  current against the model at the MEASURED pose.
* **Thermal.** j2 peaked at 51 degC and held; no ramp at fixed pose (the 2026-08-26
  overload signature).

## Scored 2026-09-10 — the first end-to-end run of `kt_rls_report.py`

Full 9-bag stacks per sweep, payload masses taken as the NOMINAL 96 / 196 g
**because the weighed values were never written down** (fix that next
session — see `../2026-09-10/README.md`). Both stacks reproduce the
wrist-angle story above exactly: kappa sits at its prior through the three
empty runs, goes to a nonsense value through the first two loaded wrist
angles, and locks the moment the third lands.

| slot | kappa [counts/Nm] | implied Kt [Nm/A] | gravity_scale = a/kappa | 96 g alone | 196 g alone |
|---|---|---|---|---|---|
| joint2 (L2)  | **164.1 +- 1.7** | 2.266 | **0.892 +- 0.002** | 176.1 | 165.6 |
| joint3 (L3f) | **152.7 +- 0.9** | 2.435 | **1.143 +- 0.010** | 155.7 | 153.0 |

against the yaml's `nm_to_effort_joints` 173.8 / 146.7 and `gravity_scale`
0.76 / 0.95. **Both implied Kt are now BELOW their measured Ke** (2.415 /
2.478 V*s/rad), i.e. eta = Kt/Ke <= 1 — the physical impossibility that opened
the campaign (doc §1) is gone, and it is gone on both joints.

Fitted payload CoM came out at (-3, +4..+7, +8..+11) mm on the L2 stack and
(-1..-3, +4..+8, +16..+19) mm on L3f — small and consistent between the two
load levels, which is what a non-slipping grip looks like.

**Constancy: j3 passes (1.8 %), j2 fails (6.3 %).** A wrong assumed mass
cannot be the whole story — kappa scales as 1/m for BOTH joints equally and j3
is tight; a ~3 g box error explains j3's split and the shared sign (the light
load reads high on both), but not j2's extra 4 %. The j2-specific suspect is
already in the Session checks above: **j2's command touches `max_effort`
(365 duty) during the 200 g sweeps**, so its two load levels are not in the
same regime. Check the clipped fraction per block before stacking next time.

Reproduce:

```bash
cd ~/ros2_ws/results_robotic_arm
xacro ../src/fsc_open_manipulator/open_manipulator_x_description/urdf/open_manipulator_x_custom_robot.urdf.xacro \
      invert:=true use_fake_hardware:=true > /tmp/custom_inv.urdf
B=bags/2026-09-09
python3 scripts/kt_rls_report.py --urdf /tmp/custom_inv.urdf --joints 1 \
  $B/collect_kt_L2_empty:0 $B/collect_kt_L2_empty_w+85:0 $B/collect_kt_L2_empty_w-85:0 \
  $B/collect_kt_L2_100g:0.096 $B/collect_kt_L2_100g_w+85:0.096 $B/collect_kt_L2_100g_w-85:0.096 \
  $B/collect_kt_L2_200g:0.196 $B/collect_kt_L2_200g_w+85:0.196 $B/collect_kt_L2_200g_w-85:0.196
# --joints 2 and the L3f bags for joint3.
```
