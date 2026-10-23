# Arm count↔torque path, and a +10 % servo calibration error under whole-body L1

2026-09-08. Three full-mission SITL flights of the whole-body **L1** rig
(`wb_l1_tune_cycle.sh l1 … fsc_lab_machine`, the §7.14 mission: takeoff →
DIRECT → soak → x/y/yaw steps + compatible-trajectory leg → abort → land).
Scored by `wb_l1_metrics.py`; raw `metrics.txt` beside this file.

## What was added

Until now the simulated arm took a **continuous torque**: Isaac applied the
commanded effort exactly, and `servo_model.py`'s `duty()` — which computes the
Goal PWM count — was documented "diagnostics only" and called nowhere. The real
arm takes an **int16 Goal PWM register**, so three effects sat between the law
and the winding that the sim did not have:

1. **quantization, truncating toward zero** (`ConvertUnitToValue` ends in
   `static_cast<T>`, `dynamixel_info.hpp:140`) — a systematic loss of up to one
   full count, never a gain, and an exact dead zone below one count. One count
   is 6.43 / 5.92 / 7.99 / 6.43 mN·m of commanded torque;
2. **the ±885 rail**, unreachable from the torque term at a 3.0 N·m cap
   (467 / 507 / 375 / 467 counts) but reachable once the back-EMF feedforward is
   stacked on top of it;
3. **calibration error** — the chain converts N·m→counts with what it BELIEVES,
   the winding makes torque per count according to what is TRUE. Delivered
   torque scales by `nominal/true`.

Config: `sim_arm_counts_enable` + `sim_arm_counts_{nominal,true}_j*` in both
whole-body `_sim.yaml`s, forwarded to Isaac by the launcher (same `sim_`
contract as `sim_arm_backemf_*`). Shipped value **true = 1.10 × nominal**, i.e.
every commanded N·m arrives as **0.909 N·m**, and the controller is not told.

The continuous model is the exact matched, unquantized limit of the new path
(`ke_to_duty/nm_to_duty == b` identically); the self-test measures the residual
at **1.8e-15 N·m**, twelve orders of magnitude under one count.

## Runs

| tag | plant | purpose |
|---|---|---|
| `l1_ctrl_armoff` | count path OFF, matched | control = the pre-change plant |
| `l1_gain10` | **injection did not reach the plant** | see the defect below |
| `l1_gain10b` | quantized register + true 1.10× nominal | the test |

`l1_gain10` is kept deliberately: it and the control are the SAME plant, so the
pair is a measured **run-to-run noise floor** for this rig — `tau_q2` 0.2 %,
tilt 1.5 %, CoM error 2.0 %, EE error 2.1 %, phantom `F̂_y` 6.8 %. Every claim
below is sized against it.

## Result: L1 absorbs it

All three flights completed the full mission — no abort, no rotor saturation
(`n_sat_frac` 0.000), no torque clamping, `tau_max` 0.95 of 3.0 N·m.

**The observer identifies the deficit on the right channels, at the right
magnitude, with no fitted parameter.** A servo delivering `g = 1/1.10` of
command puts `-(1-g)·τ` on that joint's `d_hat` channel:

| | predicted | measured | agreement |
|---|---|---|---|
| `d_hat` joint 2 shift | −0.0729 N·m | −0.0739 N·m | 1.3 % |
| `d_hat` joint 3 shift | −0.0306 N·m | −0.0293 N·m | 4.2 % |

**And it restores the delivered torque.** The law raises the command by the
shortfall, so the joint gets what it needed:

| | control | +10 % | commanded | ×0.909 delivered |
|---|---|---|---|---|
| `tau_q2` | 0.7198 | 0.8021 N·m | +11.4 % | 0.7292 (+1.3 %, inside noise) |
| `tau_q3` | 0.2853 | 0.3364 N·m | +17.9 % | 0.3058 (+7.2 %) |

Flight quality is unchanged within resolution: CoM error 78.2 → 81.3 mm and EE
error 83.9 → 87.1 mm are both ~2× a 2 % noise floor, i.e. at the edge of what
one run pair can resolve. Arm sag grows 1.87° → 2.03° (joint 2) and
1.06° → 1.23° (joint 3). Tilt 1.24° → 1.32°.

## The cost, and it is not in the flight

**Phantom `F̂_y` more than doubles: 0.187 → 0.410 N** (17× the noise floor;
late-window 0.325 → 0.489 N). In free flight the true interaction wrench is
exactly zero, so all of that is fictitious — and it is precisely the quantity
the L1-attribution work exists to drive to zero. An arm calibration error is
therefore a **direct contaminant of end-effector interaction-force estimation**,
even while it barely moves the flight. Anything that reads `F̂_y` as a real
contact force inherits it.

## Two traps found while doing this

* **A knob can be reported ACTIVE and still reach nothing.** `l1_gain10` had the
  yaml's `sim_arm_backemf_enable: false` → launcher resolved servo model
  `ideal` → 06's `ideal` branch skipped the count path entirely. The launcher
  printed the injection, and the flight was bit-identical to the control. Fixed
  by making the droop and the register **orthogonal**: `ideal` now means "no
  back-EMF droop", nothing more. Two runs agreeing to three decimals is the
  symptom to watch for.
* **The npz's joint columns are mislabelled.** `log_cols` calls them `q1..q4`
  but the data is in the ros2_control broadcaster order
  **[joint2, joint3, joint1, joint4]**, while `wbref`'s `q_d` is in model order.
  Comparing index-for-index invents ~35° of tracking error. `wb_l1_metrics.py`
  now carries the permutation and the evidence; the driver's labels are
  untouched, so anything else reading these files needs the same correction.

## Whole-system compatible trajectory, and the two figures

Two further flights (`l1_matched_traj`, `l1_gain10_traj`) repeat the A/B with the
driver now logging the **applied** torque — Isaac fills `joint_states.effort`
with what the servo actually made, resolved by name into model order — so the
gap is measured rather than replayed through the model. Figures in `fig/`, made
by `application/robotic_arm/utils/wb_arm_torque_plot.py`, over the `traj_both`
leg where the base translates and yaws while all four joints move.

The pair reproduces the first A/B: `tau_q2` 0.7199 → 0.8087 (+12.3 %), `d_hat`
joint 2 shift −0.0688 against −0.0735 predicted, phantom `F̂_y`
0.1775 → 0.4155 N (×2.34, against ×2.19 on the first pair).

**Coverage of the gap by the L1 arm channel, measured:**

| window | joint 2 | joint 3 |
|---|---|---|
| static hold (pure, DIRECT+settle → first leg) | gap 73.9 → **96 % covered** | gap 33.4 → 74 % |
| `traj_both` manoeuvre | gap 83.1 → **69 % covered** | gap 42.4 → 24 % |

The augmentation absorbs the coefficient error almost completely while the arm
is still and lags badly while it works — `wb_l1_omega_c_q` is 0.5 rad/s, a ~2 s
time constant against a 13 s manoeuvre.

**The matched flight is the control that makes Figure A attributable**: with
`true = nominal` and quantization still on, the residual gap is +3.3 / +4.5 mN·m
mean — half a count, exactly what truncation toward zero predicts.

**A paired sample-by-sample difference between runs does NOT work here** and was
abandoned after being tried: each flight's planner solves its legs from its own
anchor, so the two runs' commanded torques differ by ~100 mN·m *mean* over
`traj_both` — larger than the 83 mN·m effect. Only the static hold, where both
flights sit on the anchor captured at DIRECT entry, is a valid cross-run
comparison; Figure B therefore uses a single **constant** offset from that
window, not a time-varying difference.

## Not covered

Only the ARM calibration was perturbed here, on top of the config's existing
+15 % `alloc_thrust_coeff`, ×1.10 mass/inertia and 10 mm CoM injections. The
quantization and rail were active in `gain10b` but are **not separately
measured** — at hover the calibration error is ~11× the quantization loss
(64 mN·m against 5.7 mN·m on joint 2), so this campaign says nothing about the
register on its own. One flight per condition; the noise floor above is the only
repeat evidence.
