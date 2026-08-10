# Snapshot — T650 parameters and results as tuned 2026-08-06

Frozen before the 2026-08-07 re-fit. Everything needed to restore or to compare against.

Git ref at snapshot: `9256009` ("Compared with experimental data and updated model of T650").

## Files here

| file | what |
|---|---|
| `t650_params.py.20260806` | verbatim copy of the tuned parameter module |
| `params_20260806.json` | the same values as machine-readable numbers |
| `../figures/baseline/` | the 13 flight-A/B/C figures produced with these parameters |
| `../data/sim_stack_C_tuned.npz`, `../data/odom_metrics_C_tuned.*` | flight C sim run and metrics at these parameters |
| `../TUNING_t650.md`, `../REPORT_flightC.md`, `../REPORT.md` | the write-ups |

## The parameters

| parameter | bench / CAD | shipped 2026-08-06 | factor |
|---|---|---|---|
| `ROTOR_CONSTANT` k | 4.540431e-05 | **4.679931e-05** | ×1.030724 |
| `ROLLING_MOMENT_COEFFICIENT` c | 8.247173e-07 | **2.474152e-06** | ×3.0 |
| `INERTIA_DIAG` Ixx, Iyy | 0.05777498, 0.06408172 | **0.05955006, 0.06605057** | ×1.030724 |
| `INERTIA_DIAG` Izz | 0.065004565 | 0.065004565 | unchanged |
| `ROTOR_LAMBDA` | 10.0265 | 10.0265 | unchanged |
| `BODY_MASS` / `MASS` | — | 2.95 / **3.033921 kg** | unchanged |

Derived: `HOVER_COMMAND` 0.502525, `THRUST_SCALING` 0.040232, `IDLE_THRUST` 0.203169,
`THRUST_TO_WEIGHT` 3.352.

## Method used (for reference)

Parameters only — no modelling changed, `LaggedQuadraticThrustCurve` untouched, mass pinned
at 3.033921 kg. Fitted against flight C (`debug_recording_20260806_164742`) replayed through
the same `fsc_autopilot_ros2` **baseline** stack in Isaac.

- **k** — hover match at fixed mass. Real hover command t=5–15 s was 0.5025 ± 0.0009; bench
  k gave 0.5117 (+1.8%). A mass/thrust trade created by reinterpreting 2.95 kg from total to
  body mass, not a bad bench curve.
- **c** — swept 1.0 / 2.5 / 3.0 against flight C's two ±20° yaw steps; 3.0 centres the
  overshoot error. A stand-in for the unmodelled `I_rotor·ω̇` reaction.
- **Ixx/Iyy** — bookkeeping only: roll/pitch authority goes as k/I, so the identical factor
  holds that fit where it was. Izz deliberately not scaled.

## Results at these parameters (flight C, baseline mode)

| metric | real | before tuning | after |
|---|---|---|---|
| hover thrust command (t=5–15 s) | 0.5025 | 0.5117 | **0.5025** |
| yaw step overshoot | 9.16 % | 44.88 % | **7.80 %** |
| yaw step rise 10–90 | 0.555 s | 0.840 s | **0.588 s** |
| yaw step settling ±5% | 2.98 s | 10.15 s | **2.07 s** |
| yaw shape RMSE | — | 4.109° | **1.268°** |
| whole-run yaw RMS diff / corr | — | 1.66° / 0.984 | **0.83° / 0.996** |
| position shape RMSE | — | 0.0553 m | 0.0559 m |
| position overshoot bias | — | +2.73 pts | +2.67 pts |

## Restoring

```bash
cp docs/sim_to_real_t650/backup_20260806/t650_params.py.20260806 \
   extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/t650_params.py
```

Every derived quantity recomputes from the constants, so nothing else needs touching.
