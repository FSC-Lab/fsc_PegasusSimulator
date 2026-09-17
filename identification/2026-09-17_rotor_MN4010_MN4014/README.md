# Rotor identification — T-Motor MN_4010 and MN_4014, 15×5 prop (2026-09-17)

Thrust-stand summaries supplied by the user, stored here so every number that reaches
`airframe_id.cfg` is traceable to the measurement behind it.

| | MN_4010 | MN_4014 |
|---|---|---|
| **τ_spin at the hover point** (0.077 / 0.067 s) | 0.077 s | 0.067 s |
| ω_max (extrapolated to 100 %) | 784 rad/s (7482 rpm) | 886 rad/s (8458 rpm) |
| max_thrust (extrapolated) | 28.3 N | 36.3 N |
| thrust coefficient (T = c·ω²) | 4.616e-5 | 4.627e-5 |
| torque / thrust (cQ) | 0.0190 m | 0.0190 m |
| hover throttle (9.69 N/rotor) | 58 % | 52 % |

Both props give the same c and cQ to three figures — the same 15×5 on two motors, which is the
consistency check the pair provides.

## What is here

- `MN_40xx/data/` — the raw stand logs as recorded: `steps_*`, `tri_wave_*`, `cos_wave_*`, each as an
  `_inputs.csv` (PX4 side: `timestamp_unix_ms`, `throttle_cmd_pct`, arming/nav state) and an
  `_outputs.csv` (stand side: thrust N, torque N·m, optical RPM, V, A, vibration, at ~71 Hz).
- `MN_40xx/*_report.pdf`, `*_global_fit_*`, `*_lambda_*` — the supplier's own analysis.
- `MN_40xx/fit.txt` — **our** fit, `scripts/ana_spin_lag.py --dir MN_40xx`, which is what the cfg quotes.

## Why we refit rather than take the shipped λ

The reports ship two estimators for λ (= 1/τ) and they disagree by 3×. DIRECT rearranges the model
equation at every sample, λ = ω̇/(ω_cmd − ω) — it divides by a quantity that goes to zero at steady
state, and its own report gives R² 0.24 with **18–24 % of the λ values negative**. LAG inverts the
30–70 % threshold-crossing time, which is a fixed fraction of τ only if you correct for which
threshold. The two archives also ship *different* estimators (4010 = DIRECT, 4014 = LAG), so the
shipped numbers are not comparable to each other.

`ana_spin_lag.py` fits each step as a dead time followed by a first-order rise on **ω** — the quantity
the plant actually integrates and the L1 predictor actually models — with the dead time free so that a
transport delay cannot be absorbed into τ. `--selftest` recovers a known τ to 3 % and a known dead
time to 1 ms at this bench's noise and sample rate.

## Two things this data cannot say

1. **`PROP_J0` (the zero-thrust advance ratio).** A static stand has no airflow. On the geometric
   estimate a 15×5 gives J0 ≈ 0.33, and at 16 m/s the cruise sits at J ≈ 0.30 — *at* the point where
   this prop stops making thrust. See the 2026-09-17 entry in `claud_implementation.md`: this is a
   propeller-choice question, not a control question.
2. **The dead time.** The fit gives 81–88 ms from the PX4 throttle timestamp to the first RPM
   response, but the stand's `ESC signal (µs)` column is empty, so that number cannot be separated
   from the clock offset between the PX4 logger and the stand. **For the next bench session: log the
   ESC PWM on the stand**, and the dead time becomes a one-clock measurement.
