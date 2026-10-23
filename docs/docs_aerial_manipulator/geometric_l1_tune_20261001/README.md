# Geometric + L1 adaptive (Cai et al.) tuned for the RTF-1 comparison — 2026-10-01

The comparison report (artifact "Simulation: Free-flight Comparison", v8) flew the geometric+L1
law with its 2026-09-28 HARDWARE gains (206 mm / 9.6° EE error) while the whole-body law flew a
bench-tuned set and the modular law a CMA-tuned set. This campaign tunes the geometric law the
same way, on the same bench, and re-flies it in Isaac under the report's launch condition.
Commands: Command.md §7.23.6. Simulation only — nothing here was flown on hardware, and the
hardware and committed `_sim` yamls are untouched.

## Tools (`tools/`)

| file | what |
|---|---|
| `geo_bench.py` | the law on `circle_tune_20260927/tools/circle_bench.py` (unchanged): line-by-line Python ports of `l1_geometric_controller.cpp`, `l1_adaptive_augmentation.cpp`, `arm_state_feedforward.cpp` (r_os — matches the node's startup print exactly), the client's u = u_b + u_L1 tick (L1 advanced once per 60 Hz feedback sample, achieved-wrench anti-windup), `decoupled_reference_bridge.py`, and 06's position-mode arm servo. `python3 geo_bench.py` = calibration |
| `explore.py` | hand-picked candidates at 16 and 28 ms |
| `tune_geo.py` | CMA-ES over the 11 gains (tune_modular.py's machinery, runs and cost); `--delay-b` sets run B's delay |
| `geo_finalists.py` | robustness battery: 3 noise seeds, Isaac-like feedback, delays 28/36/44/52 ms, stress plant, 1.5 N gust |
| `margin_sweep.py` | delay margin of all three laws on one bench (28–40 ms) |
| `margin_screen.py` | re-screens logged CMA candidates at longer delays |
| `make_tuned_yaml.py` | `variants/geometric_l1_mirror_sim_tuned.yaml` = rtf_profile's mirror yaml with ONLY the 15 gain lines + vehicle_name changed (checked byte-identical otherwise) |

`run_isaac.sh` = a copy of `rtf_profile_20261001/run_rt1.sh` writing into this directory.

## Results

| gain set | bench EE / heading (3 seeds) | delay survived | stress plant | Isaac RTF 1 |
|---|---|---|---|---|
| hardware (09-28) | 241 mm / 10.3° | 44 ms (aborts 60) | aborts | 206 mm / 9.60°, 3/3 |
| stage 1 (28 ms bar) | 43 mm / 2.0° | 28 (aborts 30) | 53 mm | `decoupled_geot1`: diverged, 1.53 Hz attitude mode, 42.7° tilt 19 s into DIRECT |
| **stage 2 (44 ms bar)** | 58 mm / 3.4° | ≥52 (highest tested) | 70 mm | `geot2/3/4`: **49.6 / 48.0 / 49.1 mm, 2.96 / 3.00 / 2.94°**, 3/3 |

Same-bench margins of the other two: whole-body H1b survives 28 ms, aborts 30; modular survives 34,
aborts 36. **The 28 ms bar those two meet is NOT enough for this law in Isaac**: the bench reproduces
Isaac's divergence at 30 ms. Stage 2 met the 44 ms bar mainly by lowering the L1 filter ω_c 6 → 1
rad/s (keeps the L1 torque correction out of the ~1.5 Hz rotor-lag band); position/attitude gains
stayed near stage 1.

Shipped stage-2 gains (`analysis/geo_final_gains.json`): K_p 20.11/20.11/13.50, K_v 11.05/11.05/10.82,
K_R 3.337/3.337/1.737, K_ω 0.951/0.951/0.411, A_s −4.33/−2.79, ω_c 1.0.

Isaac (3 flights, RTF 1.000, standard 0.10 m Start gate — the hardware set needed 0.25): base 33 mm,
EE path 0.541 m (+40 mm radial, +24 mm along-track), motors 0.49–0.73, tilt ≤ 2.3°, DIRECT-entry
transient 50–64 mm (hardware set 204). Remaining error is structural to the paper's law: the
lateral body-fixed force is estimated (γ̂_um ≈ (0.46, −0.45) N) but never actuated, F/K_p ≈ 37 mm
(measured hover offset 38); ω_d = 0 heading lag (K_ω/K_R)·15°/s ≈ 3.5°.

## Reproduce

```bash
cd ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/geometric_l1_tune_20261001/tools
OMP_NUM_THREADS=1 /usr/bin/python3 geo_bench.py                               # calibration
OMP_NUM_THREADS=1 /usr/bin/python3 tune_geo.py --gens 32 --jobs 30 --sigma 0.35 --delay-b 44 \
    --log cma_geo2_log.jsonl --best-out cma_geo2_best.json \
    --start '{"kp_xy":16,"kv_xy":11,"kr_xy":2.0,"kw_xy":0.7,"kr_z":1.6,"kw_z":0.3,"as_w":3.0,"omega_c":4.0}'
OMP_NUM_THREADS=1 /usr/bin/python3 geo_finalists.py --log cma_geo2_log.jsonl --out finalists2.json
/usr/bin/python3 make_tuned_yaml.py --gains ../analysis/geo_final_gains.json
cd .. && WB_SIM_YAML=$PWD/variants/geometric_l1_mirror_sim_tuned.yaml ./run_isaac.sh decoupled:<tag>
```

**Never run a CMA job in tmux while an Isaac cycle runs**: the cycle's clean slate
(`kill_stale_sim_processes.sh`) kills the whole tmux server, and it also skews Isaac's real-time
pacing. Run them one after the other.
