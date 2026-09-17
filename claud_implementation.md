# Implementation record — `slosh` branch, hardware-transfer campaign

What was built, why, how to reproduce it, and what it measured. Newest first. `CLAUDE.md` carries the
narrative and the operating rules; this file carries the *implementation*: the edits, the exact
commands, the artefact paths and the commits, so any entry can be re-run without reading the log.

Repos are paired — sim `fsc_PegasusSimulator` (`slosh`), controller `fsc_autopilot_ros2-master`
(`slosh`), GS `ros2_ground_station_gui` (`vtol-cascade-work`), msgs. Baseline for the hardware
campaign is the tag `hw-transfer-1` in all four (sim 05a4a0e / controller f26e09d / GS 96a1fb3 /
msgs ebeded8).

---

## 2026-09-16 (evening) — The actuator-lag tolerance on the flown control law

**sim `3f53a14` · controller `a5a6a16` untouched (no identity hash moved)**

### Question

Piece 2 left the rotor spin lag as a cliff with nothing measured between the modelled 0.12 s and the
failing 0.20 s, and its surface-lag row was never a mismatch test. The design question behind it: the
L1 augmentation is meant to absorb actuator error — how much of it actually does?

### What changed

| file | change |
|---|---|
| `scripts/robust_sweep.sh` | the control law is a knob: `ROBUST_NODE` / `ROBUST_LINE` (defaults are piece 2's exactly, so `env1` reproduces); writes `<tag>/law`; resolves a relative cells-file path **before** it `cd`s into the controller repo |
| `scripts/cells_actuator_lag.txt` | new — the 16 paired cells |
| `CLAUDE.md` | the result block + a pointer on piece 2's two superseded rows (left as written, per the house rule) |

**The bug that fix closes:** `robust_sweep.sh` `cd`s into the controller repo early, so a relative
cells path resolved against the wrong repo. The sweep ran **zero cells, printed `done`, and exited 0**;
only `robust_metrics.py` noticed, via a missing `cells.txt`. A sweep that runs nothing must not look
like a sweep that passed.

### Experiment design — the cells are paired, and the pair *is* the experiment

```
--tau_spin X --l1_tau_spin 0.12    the PLANT moves, the controller's model is PINNED  -> a real mismatch
--tau_spin X                       the plant moves, l1_tau_spin FOLLOWS               -> the controller is TOLD
```

The pair separates *"the model is wrong"* from *"the actuator is too slow to fly"* — which no single
cell can do. Checked before the run: the actuator lags live **only** in `L1Aug.applied_moment`
(`nmpc_cascade_steso_node.py:269` passes them through); `slosh_predictor.py` carries none, so pinning
is a complete mismatch with nothing else quietly told the truth.

### Reproduce

```bash
# the flown "Adaptive NMPC + Geo" law: imu-mdob + EKF + L1        (16 cells x 16 seeds, ~25 min)
ROBUST_NODE=nmpc_cascade_imu_node.py \
ROBUST_LINE="--est mdob --l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh --noise_acc 0.02" \
  bash scripts/robust_sweep.sh act1 scripts/cells_actuator_lag.txt
~/envs/isaac_env/bin/python3 scripts/robust_metrics.py ~/flight_bags/robust/act1

# the bridge on piece 2's own law (defaults: the steso node, velocity force twin)
bash scripts/robust_sweep.sh act1_steso <cells: nominal, spin_0.20, spin_0.20_told>
```

Mission and pass rule are piece 2's: the straight mission, back-transition at 60 s, y0 10 m, noise
0.1° / 1°/s, CG tank; PASS = 16/16 landed, solve-fails sum ≤ 4, corridor peak ≤ nominal + 3°,
z ≥ 29.0 m, stop ≤ nominal + 3 s. Never while a flight is in the air.

### Result — `~/flight_bags/robust/act1`

| cell | landed | corridor pk | fails / 16 flights | verdict |
|---|---|---|---|---|
| nominal | 16/16 | 14.2 | 2 | PASS — matches piece 2's nominal (16/16, 14.3, 3) |
| spin 0.06, model 0.12 | 16/16 | 13.9 | 1 | PASS |
| spin 0.09, model 0.12 | 16/16 | 14.0 | 2 | PASS |
| spin 0.14, model 0.12 | 16/16 | 14.2 | **5** | flies; over the solve budget |
| spin 0.16, model 0.12 | 16/16 | 14.3 | **6** | flies; over the budget |
| **spin 0.16, TOLD** | **16/16** | 14.3 | **1** | **PASS — measuring removes the whole cost** |
| spin 0.18, model 0.12 | **11/16** | 14.2 | 4 | FAIL (5 crashed before the BT) |
| **spin 0.18, TOLD** | **15/16** | 14.4 | 5 | FAIL, just — the knee |
| spin 0.20, model 0.12 | **0/16** | — | — | FAIL (all 16 before the BT) |
| **spin 0.20, TOLD** | **4/16** | 14.3 | 2 | **FAIL — the truth buys 4 seeds, not the flight** |
| spin 0.30, model 0.12 | 0/16 | 34.5 | 6 | FAIL |
| **spin 0.30, TOLD** | **0/16** | — | — | **FAIL — the truth buys nothing** |
| surf 0.045 / 0.06 / 0.10, model 0.03 | 16/16 | 14.3 / 14.3 / 15.3 | 1 / 1 / 0 | **PASS, all three** |
| surf 0.10, TOLD | 16/16 | 15.5 | 1 | PASS |

Bridge, `~/flight_bags/robust/act1_steso` (piece 2's law): nominal 16/16, spin 0.20 pinned **0/16**,
spin 0.20 told **3/16**.

### Reading

1. **Two regimes, boundary 0.18 s.** Below it the cost is the MISMATCH and identification removes it
   outright (0.16 pinned: 16/16 at 6 solve failures; 0.16 told: 1 = nominal). Above it the cost is the
   ACTUATOR: told the true lag, 0.20 still loses 12 of 16 and 0.30 loses all 16.
   **So the L1's MODEL covers the band up to ~0.16 s once measured — which is why `l1_tau_spin` is a
   parameter and not something the adaptation discovers — and past 0.18 s nothing covers it.**
2. **Why knowing stops helping, in one number.** Pitch authority in the transition corridor *is*
   front/rear differential thrust (the wing carries nothing at 39° of tilt and 5 m/s), and its
   bandwidth is 1/τ: **8.3 rad/s at the modelled 0.12, 6.3 at 0.16, 5.0 at 0.20 — against a 8.9 rad/s
   short period.** At 0.12 the actuator is already only *at* the mode it must control. No augmentation
   stabilises a mode faster than its actuator.
3. **The failure is identical in both cases and it is the corridor** (spin 0.20, seed 1, t = 5–7 s,
   V 4–7 m/s, tilt 39°): pitch +23° (told: +14.7 then +58), elevator saturated at −1.00, outer solve
   13 → 45 ms with `ok=0`, then roll 156° / yaw 180° and the ground. **Never at the back-transition** —
   every loss in this sweep is before it, the same place piece 2's CG and control-derivative cells fail.
4. **The estimator is irrelevant to it.** Bridge 0/16 and 3/16 vs the flown law's 0/16 and 4/16 — one
   seed of scatter. The limit is a property of the airframe and the inner loop, so piece 2's spin row
   transfers to the flown law unchanged (and its "0.20: 1/16" was that same seed).
5. **★ Correction to piece 2's surface row.** Its surf cells never pinned `--l1_tau_surf`, so the
   controller was told the true servo lag — never a mismatch test. Run as one: 0.045 / 0.06 / 0.10 all
   pass 16/16 with the controller believing 0.03. 3.3× the modelled surface lag costs **one degree of
   corridor peak and nothing else**; the old "0.06, 5 fails, marginal" was seed scatter.

### The spec it writes (for the thrust-stand campaign)

- Measure `tau_spin`, set `l1_tau_spin` to it. Then the airframe requirement is **τ_spin ≤ 0.16 s**
  (0.18 marginal, 0.20 unflyable at any model value) — i.e. rotor bandwidth ≥ ~6 rad/s, 0.7× the
  short period. Unmeasured (model left at 0.12) the tolerance narrows to ~0.14 s.
- Fit the lag on **ω**, not on thrust: the plant lags ω and then squares it, so fitting the raw thrust
  trace gives a systematically different constant. Fit √T if ω is not logged.
- Spin-**up** and spin-**down** separately (drag brakes differently than the motor accelerates; the
  model has one constant — if they differ materially that is a finding, not an average).
- At the **hover throttle** — that is where the differential authority the corridor needs is set.
- `PROP_J0` is **not** a static-stand number; it needs airflow.

### Method notes worth keeping

- The pass rule's `fails sum ≤ 4` is summed over **16 flights**, so a FAIL at 5–6 is 0.3–0.4 solve
  failures per flight against nominal's 0.13. Read the *landed* column first and the budget second.
- When an actuator mismatch crashes a gate, run the **told** cell before blaming the estimator: one
  run separates a wrong model from a slow actuator.

---

## 2026-09-16 (afternoon) — The sensor audit

**sim `f6fdbfc` · controller `a5a6a16` untouched · page https://claude.ai/artifact/M836419j6Lg3CGCbBuSUMH**

### What changed

| file | change |
|---|---|
| `scripts/sensor_sweep.sh` | the cells: IMU noise / bias / walk / scale / vibration / lag / gyro bias, the state source (rate, lag, velocity noise), the pitot, and the combinations |
| `scripts/sensor_metrics.py` | the merged table, extended with the **per-axis** bob (recomputed per seed from the gate CSVs) |

The gate knobs themselves (`--vib A,f`, `--acc_walk`, `--acc_scale`, `--imu_lag`, `--bias_gyro`)
shipped earlier with the hover hold, controller `a5a6a16` — so every identity hash still stands.

### Reproduce

```bash
bash scripts/sensor_sweep.sh 1 3                  # all cells, 3 seeds, 3 cells in flight
~/envs/isaac_env/bin/python3 scripts/sensor_metrics.py base hw_imu hw_ekf2 hw_all ...
```

### Result — 30 cells × 3 seeds, `~/flight_bags/est_sweeps/sens_*`

The decisive split is per axis, because the tank's only moment arm is r_x, so M = r_x·F_y:
**y-bob** is the channel the cancellation acts on, **x-bob** has no arm and is the loop's own response.

- **The cancellation is an IMU-only loop and it holds.** y-bob 0.015–0.088 N in **29 of 30 cells**
  (uncancelled 0.77 N) — through 5× the sim's accelerometer noise, 0.5 m/s² turn-on bias, 3% scale
  error, 1 m/s² of vibration, 1 deg/s of gyro bias, a 10 Hz GPS, and the whole hardware set at once.
- **The one hard tolerance is the accelerometer's *declared* delay.** 40 ms undeclared → y-bob 0.364 N
  and ~15 wall impacts; the same 40 ms passed to the delay buffer → 0.084 N, zero hits. Not the
  latency — the mis-declared latency.
- **Everything else is the state source** and lands in the x-bob, never in the cancellation. Decomposed:
  EKF2's 35 ms of fusion lag drives u_ad 0.006 → 0.081; 0.05 m/s of white velocity noise adds the wall
  hits but leaves u_ad and roll at base. A bare 10 Hz GPS 0.1 s late is 10× worse than EKF2 on every
  column (orbit roll sd 1.57 vs 0.20).
- **The realistic hardware aircraft flies the mission**: `hw_ekf2` 3/3, 1 solve failure, stop 5.8 m,
  orbit roll sd 0.57° (HIL's own band is 0.83–1.26°).
- **The velocity twins collapse on the same set** (y-bob 0.58–0.72 of 0.77) where imu-mdob holds 0.039.

### Standing items it produced

- Measure `IMU_DELAY` on the Orin link before the first tank flight (`scripts/ana_imu_latency.py`).
- FFT the first hover bag's `sensor_combined`; treat a peak below ~40 Hz as a mount to fix.
- Keep the loop on EKF2's odometry, never on a raw receiver.

### Correction carried since

`vehicle_attitude` is published on **every IMU sample** (EKF2's output predictor), and
`sensor_combined` at `IMU_INTEG_RATE` — so both run at 250 Hz on an FMU configured that way, while
`vehicle_odometry` publishes inside `if (_ekf.update())`, i.e. at `EKF2_PREDICT_US`. Open: whether the
250 Hz survives the DDS hop (in the v1.16.0 tree every sent topic gets `orb_set_interval(10 ms)` in
`SendTopicsSubs::init`, a 100 Hz ceiling). `ros2 topic hz` on the companion vs `listener` on the FMU
console settles it; if DDS really delivers 250, piece 6's firmware rebuild comes off the list.
