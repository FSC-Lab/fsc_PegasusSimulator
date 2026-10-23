# The arm's current loop, in the simulator — 2026-09-09

The arm controller (`fsc_open_manipulator`) now closes a **software current
loop** around Dynamixel Operating Mode 16, designed and bench-validated on
2026-09-09 (`fsc_open_manipulator/doc/Current Loop Design.md`). This campaign
replaces the simulator's back-EMF droop model with what that loop actually
leaves behind, and asks whether the L1-adaptive whole-body controller still
flies against it.

## What changed in the plant

**Before** (2026-09-03 – 2026-09-08), `servo_model.py` modelled Mode 16 as the
voltage source it is without a current loop:

```
tau_app = clip(tau_cmd, ±tau_cap) − b·qd,     b = Kt²/R = [0, 0.934, 1.493, 0] N·m/(rad/s)
```

a **bias**: the motor under-delivered every cycle, in both directions of travel,
by an amount proportional to joint speed. That is the mechanism the 0902/0903
flight audit measured as 54–67 % torque delivery, and it is what the current
loop was built to remove.

**After** (2026-09-09), the loop removes it — bench-measured, loop off → 1.5 Hz:

| | joint 2 | joint 3 |
|---|---|---|
| torque delivered, 10 °/s | 93.3 % → **99.4 %** | — |
| torque delivered, 20 °/s | 95.0 % → **99.5 %** | 87.5 % → **91.8 %** |
| current error | 13–15 → **4.2–4.6 mA rms** | 14–16 → **6.7–6.9 mA rms** |

and the shape of the error changes, which is the real result: with the loop off
both joints sat on a persistent one-sided ≈ −20 mA offset and never crossed
zero; with it closed the error is centred and symmetric. So the plant model is
now

```
tau_app = clip(tau_cmd, ±tau_cap) + Kt·i_err(t)
```

with `i_err` zero-mean, **7 mA rms** (joint 3, the worse of the two calibrated
joints, applied to all four per the user's instruction — joints 1 and 4 carry
commands near the current sensor's floor and are uncharacterised), band-limited
by a first-order low pass at **5 Hz**, which is the band
`doc/current_error_all.png` plots. Through each joint's `Kt` that is
**[16.3, 15.0, 17.7, 16.3] mN·m rms**, ≈ 2 % of the arm's hold torque.

`i_err` is generated as an exact-discretization AR(1),
`a = exp(−2π·fc·dt)`, `i ← a·i + sqrt(1−a²)·σ·randn(4)`, so its stationary rms
is σ at **any** step size — an Euler form's variance would scale with `dt` and
silently retune the plant if the physics step ever changed. Seeded, so a run
reproduces.

## Three things worth stating plainly

- **The noise is a plant disturbance the controller cannot see.** The
  whole-body node never subscribes to `joint_states.effort` — verified by
  grep; it only *publishes* its own commanded torque there. The residual
  reaches the law only through the physics, which is what makes this a fair
  test.
- **The software current loop is deliberately NOT run in simulation.** The
  Isaac plant models its *outcome*. `IsaacTopicEffortSystem` does export an
  effort state interface (06 republishes the applied torque), so setting
  `current_loop_bandwidth_hz > 0` in `torque_controller_isaac_aerial.yaml`
  would close a second loop around a signal that is already post-loop and
  cancel the very disturbance under test. It defaults to 0.0 and that config
  now carries a comment saying why.
- **What is NOT modelled**: content above 5 Hz (the bench figure filtered it
  out, so it is uncharacterised, not absent); the loop's own 1.5 Hz dynamics
  (only its steady outcome); gearbox friction, which sits downstream of the
  electromagnetic torque and cannot be separated from the flight data.

## The flights

Plant = the shipped `_sim` configuration, unchanged apart from the arm: +15 %
allocator `kf`, plant mass and inertia ×1.10 with a 10/10/5 mm CoM shift,
MN4010 rotor lag. Law = the 2026-09-09 shipped config — posture term absent,
`wb_ee_anchor_com` + `wb_u3_internal_ff` on, `K_y` 20 / `D_y` 12, `k_x` 32 /
`k_v` 20. **No gain was touched for this campaign.**

Mission = the rig's standard 8-leg test (§7.15.5): 1 m hover → DIRECT → x/y
steps ±0.5 m → yaw steps ±30° → compatible-trajectory EE legs → both-move legs
→ SAFETY abort → land.

```bash
# the two matched runs (16 s holds, so they are comparable with the baseline)
docs/docs_aerial_manipulator/arm_current_noise_20260909/run_noise.sh shiqi_machine
/usr/bin/python3 application/robotic_arm/utils/wb_l1_metrics.py \
    docs/docs_aerial_manipulator/arm_current_noise_20260909/*.npz \
    docs/docs_aerial_manipulator/posture_ablation_20260909/l1_mission_kx32.npz
```

`l1_noise_A.npz` was flown first at the driver's default 6 s holds — it is a
valid "does it fly", but **its settled numbers are not comparable** with the
16 s-hold baseline (§7.15.7's hold-length trap: at 6 s this rig has not
settled, so the 2 s average lands at an arbitrary phase of a slow, lightly
damped mode).

## Result — the L1 whole-body law handles it, no gain touched

Three missions, three completions, no aborts, **0.00 % joint clamp and 0.00 %
rotor saturation in every one**. Runs B and C are the matched pair (16 s holds,
directly comparable with `l1_mission_kx32.npz`); A is the 6 s-hold first flight.

| | noise B | noise C | ideal arm |
|---|---|---|---|
| legs / aborted | 10 of 10, no | 9 of 10 * , no | 10 of 10, no |
| DIRECT | 276.2 s | 314.0 s | 276.9 s |
| CoM err mean / late | 21.0 / 9.6 mm | 16.9 / 5.4 mm | 18.8 / 5.0 mm |
| EE err mean / max | 3.5 / 33.4 mm | 3.2 / 28.5 mm | 2.9 / 30.4 mm |
| tilt p-p / \|e_R\| max | 3.08° / 0.181 | 2.95° / 0.160 | 2.78° / 0.170 |
| peak arm torque | 0.798 of 3.0 N·m | 0.785 | 0.751 |
| q2 / q3 abs err | 0.81 / 1.21° | 0.76 / 1.19° | 0.58 / 0.97° |
| phantom \|F̂_y\| | 0.123 N | 0.120 N | 0.077 N |
| u₁ / d̂_z | 47.563 / −10.809 N | 47.562 / −10.809 | 47.562 / −10.809 |

\* C lost `traj_both_back` to the known **driver race** (planner went
CALCULATING→PLANNED inside one 10 ms sample, `send` hit CALCULATING, the driver
waited out its 45 s exec timeout while the vehicle held at hover). Not a control
failure — Command.md §7.15.9 records the same race on a previous campaign, and
the fix belongs in `wb_l1_campaign_driver.py`.

`u₁` and `d̂_z` agreeing to three decimals across all three runs is the check
that the plant really was identical in each.

### Where the noise actually lands

Std over the quiet soak hold, N·m:

| | d̂ᶜ UNFILTERED (deadbeat) q1/q2/q3/q4 | d̂ FILTERED (what the law sees) |
|---|---|---|
| noise, 3 runs | .0247–.0254 / .0389–.0496 / .0229–.0266 / .0199–.0206 | .0029 / .0031–.0050 / .0039–.0042 / .0038–.0040 |
| ideal arm | .0138 / .0330 / .0143 / .0108 | .0003 / .0047 / .0025 / .0006 |

The deadbeat piecewise-constant estimate picks the injection up almost exactly:
the quadrature excess over the ideal run is **16.7 mN·m on q4 and 17.9 on q3
against the 16.3 / 17.7 injected**. (q1 and q2 read ~21 mN·m because the law's
own reaction adds there.) The L1's explicit `C(s) = ω_c/(s+ω_c)` then cuts it
about **5×**, to 3–5 mN·m against the arm's 700 mN·m hold torque. That is the
bandwidth-vs-accuracy split the observer exists for, and it is why 7 mA of
current error costs single-digit millimetres of tracking.

**Do not read this off the collective channel.** `dc_z_std` is 0.73 / 0.23 /
0.29 N on noise-B / noise-C / the **ideal** run — the arm's joint-space noise is
not what dominates it, and the run-to-run spread swamps the effect. Scored that
way the noise looks 2.5× worse than ideal on one run and identical on the next.

### Measured end-to-end from telemetry, not just from the model

Over a 33 s quiet hold in flight A, `applied − commanded` per joint:

| | mean | rms | injected |
|---|---|---|---|
| j1–j4 | **+0.1 to +0.8 mN·m** | 21–24 mN·m | 15–18 mN·m |

The mean is the result — **the bias really is gone**, which is the whole point
of the current loop. The rms excess is not extra injection: the law's own 250 Hz
commanded torque carries 13–19 mN·m of ripple in the same window (it is
reacting to the noise-driven joint motion), and the log samples two
asynchronous streams at 50 Hz.

**Trap:** the log's `tau_app1..4` columns are in **model order**, unlike the
position columns `q1..q4`, which need `LOG_Q_OF_JOINT = (2,0,1,3)`. Verified by
cross-correlation. Mixing them up scores joint 2 against joint 1 and invents
~0.7 N·m of error.
