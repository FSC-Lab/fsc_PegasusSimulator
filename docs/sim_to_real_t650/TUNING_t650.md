# T650 parameter tuning against flight C

Empirical fit of `rotorcraft/t650_params.py` to
`docs/experimental_data_ros2_bag/debug_recording_20260806_164742`, replayed through the same
`fsc_autopilot_ros2` baseline stack in IsaacSim (**2026-08-06**).

**Scope: parameters only.** No modelling was changed — `LaggedQuadraticThrustCurve` and every
other model is untouched. **Mass was not changed** (`MASS = 3.033921 kg` throughout).

## Result

| parameter | bench / CAD | shipped | factor |
|---|---|---|---|
| `ROTOR_CONSTANT` k | 4.540431e-05 | **4.679931e-05** | ×1.030724 |
| `ROLLING_MOMENT_COEFFICIENT` c | 8.247173e-07 | **2.474152e-06** | ×3.0 |
| `INERTIA_DIAG` Ixx, Iyy | 0.05777498, 0.06408172 | **0.05955006, 0.06605057** | ×1.030724 |
| `INERTIA_DIAG` Izz | 0.065004565 | 0.065004565 | unchanged |
| `ROTOR_LAMBDA` | 10.0265 | 10.0265 | unchanged |
| `MASS` | — | 3.033921 kg | **unchanged** |

| metric (flight C) | real | before | after |
|---|---|---|---|
| hover thrust command (t=5–15 s) | 0.5025 | 0.5117 | **0.5025** |
| yaw step overshoot | 9.16 % | 44.88 % | **7.80 %** |
| yaw step rise 10–90 | 0.555 s | 0.840 s | **0.588 s** |
| yaw step settling ±5% | 2.98 s | 10.15 s | **2.07 s** |
| yaw shape RMSE | — | 4.109° | **1.268°** |
| whole-run yaw RMS diff / corr | — | 1.66° / 0.984 | **0.83° / 0.996** |
| position shape RMSE | — | 0.0553 m | 0.0559 m |
| position overshoot bias | — | +2.73 pts | +2.67 pts |

Yaw shape error drops **3.2×** and the overshoot error from +35.7 to −1.35 percentage points.
Position is unchanged within noise (see below). Figure:
`figures/C_tuning_before_after.png`.

## What was fitted, and why

**`ROTOR_CONSTANT` — hover match at fixed mass.** The real vehicle's hover command early in
flight C, before battery sag accumulates, is 0.5025 ± 0.0009. With mass pinned at
3.033921 kg the bench k puts the simulated hover at 0.5117, +1.8% high, so k absorbs the
difference. This is a mass/thrust trade, **not** evidence the bench thrust curve is wrong: at
the previous 2.95 kg *total* the bench k reproduced the measured hover to +0.14%. If the mass
convention is ever revisited, revert `THRUST_FIT_FACTOR` to 1.0 first.

**`ROLLING_MOMENT_COEFFICIENT` — effective yaw torque, standing in for a missing term.** The
simulated yaw axis was badly under-powered, and the cause is a missing *term* rather than a
wrong number: the thrust curve applies only the steady drag torque `c·ω²` and never the
reaction to the rotors' own angular acceleration, `I_rotor·ω̇`, which during a yaw command
adds across all four rotors and is 2–4× larger than the modelled torque. Independent
corroboration: PX4's own autotune could not identify the yaw axis at all — it returned 5.6×
less response per unit excitation than roll/pitch and timed out at 20 s, while roll and pitch
converged in ~5 s each (`figures/autotune_diagnostic.png`).

Since modelling was out of scope, c is inflated to stand in for that term. **c is therefore
an effective yaw-torque coefficient, not the bench drag coefficient** — it is valid near the
tested step sizes and will over-predict steady-state yaw drag.

Only the ratio c/Izz sets yaw dynamics, so this could equally have been done by lowering Izz.
c was chosen because Izz has independent CAD support, and matching by inertia alone would
require Izz ≈ 0.029 kg·m², less than half any physical estimate.

Factor chosen by measurement:

| factor | yaw overshoot | yaw rise | yaw RMSE |
|---|---|---|---|
| real | 9.16 % | 0.555 s | — |
| 1.0 | 44.88 % | 0.840 s | 4.109° |
| 2.5 | 13.51 % | 0.628 s | 1.292° |
| **3.0** | **7.80 %** | **0.588 s** | **1.268°** |

3.0 ships because it centres the overshoot error. The 2% RMSE difference between 2.5 and 3.0
is not meaningful.

**`INERTIA_DIAG` Ixx/Iyy — bookkeeping, not a claim.** Roll/pitch authority goes as k/I, so
raising k by 3.07% would have sped those axes up by the same amount. They already fit well,
so Ixx and Iyy are scaled by the identical factor to leave that fit exactly where it was.
Izz is deliberately not scaled — the yaw axis is corrected through c instead, and scaling both
would double-count.

## Noise floor — read this before trusting small differences

The 2.5 and 3.0 runs differ **only** in c, which affects yaw alone, so their position metrics
are two samples of the same configuration. They gave position shape RMSE **0.0642 m and
0.0559 m**. That is a run-to-run spread of ±0.008 m, so **position differences below ~0.01 m
are not resolvable** with single runs, and the apparent position regression at factor 2.5 was
noise, not a real effect.

Everything here rests on one flight and one run per configuration. The yaw fit in particular
comes from **n = 2 steps**, both ±20°.

## Limitations

- **c is not physical.** It is a lumped stand-in for a missing dynamic term. It should be
  reverted to the bench value if `LaggedQuadraticThrustCurve` ever gains the rotor
  spin-up reaction torque — that is the real fix, and the factor here is a measure of how
  much torque is missing.
- **Fitted to one flight, one vehicle, one gain set.** The PX4 tune
  (`MC_*RATE_K=0.3`, `MC_ROLL_P/MC_PITCH_P=3.25`, `MC_YAW_P=1.4`) is baked into the result;
  changing it invalidates the fit.
- **Yaw settling is now 0.9 s faster than reality** (2.07 s vs 2.98 s) — the one metric that
  degrades with more yaw gain.
- **Battery sag remains unmodelled.** k is fitted to the *early*-flight hover; the real
  command climbs to ~0.5111 by t=60 s, and the simulation stays flat.
- **The roll/pitch trim offset (~2.5°) is unaffected** by any of this — it is a constant
  attitude-datum offset, not a dynamic error. See `REPORT_flightC.md` §4.
- **The flight A and B figures and metrics are now stale**: they were produced against the
  pre-tuning parameters and the older 2.95 kg *total* mass. Their conclusions about
  `sensor_combined` vibration and battery sag still stand — neither depends on these
  constants — but their step numbers do not correspond to the shipped parameter set.

## Reproducing

```bash
tools/run_stack_case.sh ref_C.npz sim_stack_C_tuned.npz tunedC   # full stack, closed loop
tools/metrics_odom.py C sim_stack_C_tuned.npz
tools/plot_odom.py   C sim_stack_C_tuned.npz figures
tools/plot_tuning.py                                             # before/after figure
```
