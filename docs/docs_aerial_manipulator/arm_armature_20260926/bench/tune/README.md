# Task-gain tune AFTER the two structural fixes (2026-09-26)

Order, per the user's instruction: (1) armature on the joint diagonal at the bench
values, (2) the law's q̇ = the arm's 12 ms velocity observer, THEN (3) K_y / D_y.
Offline arm sim `../../arm_stiffness_sim.py` via `../tools/sim_bench_plant.py <plant>
<law> [present|observer] [<law ms>+<cmd ms>]`: exact law, 4-D hardware config, stick-slip
friction at the flight-identified ratios, flown friction FF, servo caps, flight 3's
arm design (fold 55°, q2 25 ± 15°, 12 s), CoM held.

**Plant** = the bench-measured arm: `diag` = joint-diagonal [0.010, 0.0194, 0.0097, 0.0097];
`hht` = the same row totals with the j2–j3 cross term (0.0097 h hᵀ); `diag075` / `diag130`
= the calibration's ±30 % band. **Law** `D<K_y>_<D_y>` = joint-diagonal bench armature.

**Velocity observer model** (validated on the 09-11 bench bags: the same 20 Hz / ζ 1
discrete observer on the recorded encoder positions reproduces the PUBLISHED
`~/velocity_observer` to 0.02 rad/s rms on 0.2 rms signals, corr 0.99, and its rest
noise 0.012–0.030 rad/s to 5 %).

## 1. Gain sweep, observer velocity, nominal transport

j2 / j3 error rms [deg] (hardware flight 3: 5.0 / 8.3; the flown law on this plant 5.4 / 8.9):

| K_y / D_y | diag plant | hht plant |
|---|---|---|
| 20 / 12 (flown gains) | 3.1 / 7.1 | 3.1 / 7.1 |
| 33 / 20 | 1.9 / 4.3 | 1.9 / 4.3 |
| 50 / 20 | 1.3 / 3.0 | 1.3 / 2.9 |
| 50 / 28 | 1.3 / 2.9 | 1.2 / 2.8 |
| **80 / 24** | **0.8 / 1.9** | **0.8 / 1.9** |
| 80 / 30 | 0.8 / 1.9 | 0.8 / 1.8 |
| 120 / 30 | 0.6 / 1.3 | 0.5 / 1.2 |
| 120 / 36 | 0.5 / 1.2 | 0.5 / 1.2 |
| 200 / 40 | 0.3 / 0.8 | 0.3 / 0.7 |

At K_y 20 the velocity source makes no difference (Present Velocity 3.1 / 7.1 too): the
observer buys nothing by itself, it raises the CEILING. With Present Velocity every
K_y ≥ 80 limit-cycles (cap 92–99 %, tilt 11–24°).

## 2. Robustness, observer velocity, arm transport added

`8+8` = 8 ms extra on the joint state into the law + 8 ms on the torque out (wall);
`12+12`, `16+16`, `20+20` likewise. LC = limit cycle on the servo caps.

| law | diag 8+8 | diag 12+12 | diag075 8+8 | diag130 8+8 | hht 12+12 | hht 16+16 | hht 20+20 |
|---|---|---|---|---|---|---|---|
| **80 / 24** | — | — | — | — | **0.9 / 1.9** | **1.1 / 2.1** | **1.2 / 2.2** |
| 80 / 30 | 0.8 / 1.9 | 0.8 / 1.9 | 0.8 / 1.9 | 0.8 / 1.9 | 1.1 / 2.1 | 1.3 / 2.3 | 1.5 / 2.5 |
| 120 / 30 | 0.6 / 1.3 | 0.6 / 1.3 | 0.6 / 1.3 | 0.6 / 1.3 | 0.8 / 1.5 | 0.9 / 1.6 | **ABORT 12 s** |
| 120 / 36 | 0.6 / 1.3 | 0.6 / 1.3 | 0.6 / 1.3 | 0.6 / 1.3 | **LC 38 / 15** | — | — |
| 200 / 40 | 0.4 / 0.8 | 0.4 / 0.8 | 0.4 / 0.9 | 0.4 / 0.8 | **LC 31 / 16** | — | — |

**Shipped: K_y 80 / D_y 24** (both 4-D yamls). Rationale: the highest gain that stays stable
on the COUPLED plant with 40 ms of extra arm transport; 120 fails there at 40 ms and
120/36 already at 24 ms; the ±30 % armature band changes nothing. Joint stiffness
Kq (j2/j3 principal) 0.20 / 1.35 flown → 1.09 / 2.70 N·m/rad; the soft direction 5.4×.
`M_r_d` rescaled with `M_r` (×1.122 / 0.999 / 1.073 at home) so the attitude loop is
untouched.

## 3. Isaac (`sim2real_tuning_20260926/tools/replay.sh a3 …`, mirror plant, RTF 0.48)

See `../README.md` §"Isaac A/B" for the flown numbers. Two harness facts learned:

* The planner refuses an INTEGER override for a double parameter and exits (the first
  attempt wrote `ee_traj_fold_deg: 55`); `replay.sh` now always writes floats.
* At RTF < 1 the observer differentiates the encoder against WALL time while 06 reports
  the plant-time rate (measured on the new-law run: `wb_control_debug[111..114]` rms =
  2.0 × `[107..110]` on all four joints = 1/RTF), so in the replay the law's q̇ is RTF × the plant rate — the
  damping the law applies is effectively D_y × RTF in plant time. The replay is therefore
  CONSERVATIVE on damping for the observer configuration; the Present-Velocity baseline
  does not have this handicap. On hardware RTF = 1 and it disappears.
