# The 2026-09-11 arm calibration + per-joint current loop — and why this machine cannot validate it

Two things were aligned into the simulator and the two whole-body L1 launchers from
the `fsc_open_manipulator` pull of 2026-09-11, and then flown. **The alignment is
verified end to end in the plant. The flight validation did NOT pass — but it does
not pass with an IDEAL, noiseless arm either, so nothing here is attributable to
the alignment.** The cause is measured below and is a property of this machine.

## 1. What was aligned

**The motor calibration** (`servo_model.py`). `nm_to_effort_joints`
`[160.0, 173.8, 146.7, 160.0] -> [162.4, 154.0, 150.5, 153.4]` counts/N.m and
`motor_resistance_ohm` `[4.90, 4.90, 4.30, 4.90] -> [5.26, 4.90, 4.53, 4.88]`, both
measured by the arm repo's lever/RLS campaign (the j1/j4 UNITS were swapped into the
j2/j3 brackets so gravity could give them a lever). Two checks now ASSERT in the
self-test, and they are what makes this set trustworthy rather than merely newer:

* the derived `NM_TO_DUTY` comes out `[169.47, 149.70, 135.25, 148.51]`, which is
  exactly what the arm controller logged on hardware on 2026-09-11;
* `eta = Kt/Ke` against the same session's measured `Ke` is
  `[0.943, 0.943, 0.942, 0.943]` — four independent motors agreeing to 0.1 % on a
  sensible gearbox loss. The superseded set implied `eta > 1` on j2, which is
  physically impossible and is what opened the campaign.

**The current loop is per joint.** `current_loop_bandwidth_hz_joints` ships
`[0.0, 1.5, 1.5, 0.0]`: the trim helps the two loaded joints and HURTS j1/j4, whose
commanded current is mostly noise, so the integral chases it. The plant's residual
therefore stops being one number:

| joint | trim | in-band bench rms | shipped sigma | torque noise |
|---|---|---|---|---|
| j1 | OFF | 11 mA | 15.6 mA | 35.7 mN.m |
| j2 | 1.5 Hz | 4.2-4.6 mA | 6.2 mA | 15.0 mN.m |
| j3 | 1.5 Hz | 6.7-6.9 mA | 9.6 mA | 23.7 mN.m |
| j4 | OFF | 11 mA | 15.6 mA | 37.8 mN.m |

against the superseded uniform 7 mA = `[16.3, 15.0, 17.7, 16.3]` mN.m. **The two
joints with no trim are the two noisiest**, and they are the pair that carries the
EE-heading task.

**Plumbing.** The sim yaml still carried `sim_arm_backemf_*`, an interface deleted
from the plant on 2026-09-09, so the launcher fell back to built-in defaults and
printed a NOTE every launch. Replaced with per-joint `sim_arm_current_noise_*`; `06`
and `start_single_drone_x650.sh` now accept the four-value form. Verified live in the
Isaac pane, which is the proof the whole chain works:

```
[AM-T650-WB] arm servo model 'current' (noise from the controller yaml):
    current-loop residual [15.6, 6.2, 9.6, 15.6] mA rms @ [5.0,...] Hz
                        = [35.7, 15.0, 23.7, 37.8] mN.m rms
```

## 2. THE SIGMA CONVENTION IS A JUDGEMENT THE MEASUREMENT DOES NOT SETTLE

The bench scores the **5 Hz content** of `I_measured - I_commanded`, so it reports an
IN-BAND rms, while the model's `current_noise_a` is the TOTAL rms. A first-order low
pass puts exactly half its variance below its own corner, so `total = sqrt(2) x
in-band` — IF the real spectrum keeps falling like the model's above 5 Hz, which
`servo_model.py` says in as many words is "not characterised".

The superseded model used `sigma_total = the in-band number directly` (7 mA was j3's
in-band figure). The shipped values use `sqrt(2) x in-band`. That is a 41 % inflation
of every channel and it is a modelling choice, not a measurement. Both were flown
(section 4) and neither is the cause of the failures, so this is left as shipped and
flagged rather than quietly decided.

## 3. j1/j4 — IS IT TORQUE AT ALL?  ANSWERED: NO. (updated after arm 612775a)

**The values flown in section 4 were wrong on j1/j4 and have since been corrected**
to j3's figure, `[0.0096, 0.0062, 0.0096, 0.0096]` A. The arm repo settled the
question below on 2026-09-11 (commit `612775a`), the same day and after these
flights: at stalled samples, where `I = V/R` must hold exactly, **j4's measured
current correlates only 0.58 with its applied duty** at any filter time constant,
against **0.97 on j2** — the residual is 7.6 counts against a signal of similar
size. Their verdict is "stop quoting a metric below its own noise floor", and it is
what explains three failed fixes there (the trim made j4 WORSE, the dither did
nothing, only ~20 % of the reversal excess was ever friction).

So j1/j4's 11 mA measures the CURRENT SENSOR, not the motor, and injecting it as
applied torque was modelling the wrong thing. They now take j3's value — the worse
of the two joints whose residual IS real — which is what this model did before
2026-09-11, when the reason was "uncharacterised" rather than "characterised and
found to be below the floor". That is deliberately conservative: a duty/R error
scales with duty and j1/j4 command ~500x less of it than j2.

**This does not change any conclusion in section 4**, because the flights were
never sensitive to it — `ideal`, with zero residual on all four joints, aborted 2/2.

Also now known and NOT modelled: j4's remaining error is a **velocity-dependent
bias** (-2.6 mA at rest, +6..+11 mA moving, ~8 mA mean), which the arm repo
attributes to a `back_emf_ke` ~20 % high — slot 4's Ke was never measured in that
slot, it is the j3 unit's value from another slot and day. That is a real torque
error on the flight path, since the pass-through compensates back-EMF with it, and
it is structurally a DROOP-like term rather than noise. It is left out because the
20 % is an inference from an unmeasured constant; fixing it needs J4STEEP and the
80 mm bracket.

## 3b. The original framing, kept because the reasoning is the reusable part

At the steady hold the commanded current per joint, in Present-Current LSBs (2.69 mA):

| | j1 | j2 | j3 | j4 |
|---|---|---|---|---|
| commanded current | **0.2 LSB** | 108 LSB | 36 LSB | **0.0 LSB** |
| measured residual | 4.1 LSB | 1.6 LSB | 2.5 LSB | 4.1 LSB |

On j2 the residual is unambiguously a real torque error — the command is 108 LSB. On
j1/j4 the command is ZERO and the "error" is ~4 quanta of the sensor, which is why the
arm repo's own note says the trim there "chases noise". **The bench cannot distinguish
real winding-current ripple (real torque, which the law must reject) from
current-sense noise (no torque at all).** The two have opposite consequences in the
plant and the shipped model assumes the first. A bench run at a NON-ZERO j1/j4 load
would separate them: a real error scales with command, sensor noise does not.

## 4. THE FLIGHTS — 8 runs, 7 aborts, and the residual was never implicated

All on `fsc_lab_machine`, shipped `_sim` config otherwise (+15 % allocator kf, plant
mass/inertia x1.10 with a 10/10/5 mm CoM shift, MN4010 rotor lag), `--hold-between 16`.

| run | arm plant | DIRECT | verdict |
|---|---|---|---|
| calib_A / calib_B | per-joint, sqrt(2) conv. | 258 / 165 s | aborted 2/2 |
| conva_A / conva_B | per-joint, in-band conv. | 165 / 279 s | aborted 1/2 |
| unif7 | uniform 7 mA — the exact 09-09 plant | 91 s | aborted |
| **ideal / ideal_B** | **exact torque source, zero residual** | **105 / 150 s** | **aborted 2/2** |
| calib_hl | per-joint, sqrt(2) conv., HEADLESS | 278 s | aborted |

**`ideal` aborts 2/2.** That plant is bit-identical to the 09-09 baseline that flew
the full mission clean, so the arm residual is exonerated outright — and the ordering
is not even monotonic in noise, since `ideal` aborted EARLIEST of all. Every run shows
the same signature: step peaks 290-325 mm (the shiqi baseline was 182-226), tilt
p-p 18.7-20.1 deg, and a growing ~0.6 Hz oscillation in q2/q3.

**A pattern that looked like a finding and was not:** in the first three aborts every
base-only leg passed and the failure began with the first leg that MOVED THE ARM. That
survived three runs and died with `ideal`, which failed on `step_y+` with the arm
parked at home. Three runs is not a mechanism.

## 5. THE CAUSE: this machine runs the sim at RTF 0.34, and the feedback is a staircase

Measured live during a flight, stable over 25 s:

```
REAL-TIME FACTOR = 0.336        (0.351 headless; Isaac is CPU-bound at ~400 %,
                                 GPU only 40 % — rendering is NOT the bottleneck)
```

`calib_hl` is the headless control for exactly this: same shipped plant, rendering
off, RTF 0.351 instead of 0.336 — and it aborts too. Headless is worth having (it is
now baked into the Isaac pane) but it is NOT the remedy on this box.

The plant therefore produces a new state at **85 Hz wall** (250 Hz sim x 0.34) while
the estimator -> controller chain runs on the **wall clock at 235-250 Hz**. Measured
directly on the odometry the controller consumes:

```
odom samples 2524
  consecutive samples with an IDENTICAL position: 65.2 %
  => each distinct plant pose is republished ~2.87 times   (1/RTF = 2.85)
```

**Two of every three control ticks act on a stale, byte-identical measurement**, while
the law's observers and derivative terms assume a 4 ms step. The base velocity feedback
is a staircase. That accounts for the degraded step tracking, the growing low-frequency
mode, and the independence from the arm plant.

NOTE THE DIRECTION, because the obvious guess is backwards: a slow sim makes transport
delays SMALLER in simulated time, not larger. The damage is oversampling a slow plant,
not inflated delay.

Ruled out along the way, each by measurement: control-loop stalls (clean 250 Hz,
p99 4.9 ms); the `bspline` transition-planner backend (introduced with the L1 observer
commit, so it was in effect for the 09-09 baseline too); arm pass-through fidelity
(applied - commanded: mean 0.0-2.0 mN.m); and rendering (headless gained 4 %).

## 6. What this means

* **The alignment is done and verified in the plant.** It is not validated in flight,
  and cannot be on this machine.
* **Re-run the validation where the tune was established** (`shiqi_machine`, or any box
  holding RTF near 1). The 09-09 baseline this was first scored against ran there;
  cross-machine absolute numbers are not comparable and should not be quoted as one.
* The rig's own sensitivity to this is not new — its attitude tune was retuned once
  already for transport latency (2026-08-22) and `M_r_d` was chosen on a ~32 ms delay
  margin. A 2.87x feedback staircase is far outside what any of that assumed.

## Files

`run_calib.sh` (shipped per-joint) · `run_calib_conv_a.sh` (in-band convention) ·
`run_discriminate.sh` (ideal + uniform 7 mA) · `run_repeats.sh` (ideal_B) ·
`run_headless.sh` (the RTF test). Scored with
`application/robotic_arm/utils/wb_l1_metrics.py`; npz are gitignored.
