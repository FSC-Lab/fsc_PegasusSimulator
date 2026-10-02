# Modular adaptive (Yadav et al., TMECH 2025) vs whole-body L1 — simulation comparison, 2026-09-30

Paper: R. D. Yadav, S. Dantu, W. Pan, S. Sun, S. Roy, S. Baldi, "Modular Adaptive Aerial
Manipulation Under Unknown Dynamic Coupling Forces", IEEE/ASME Trans. Mechatronics 30(4),
2688–2698, 2025 (`docs/comparison references/`, arXiv 2410.08285). Simulation only — no hardware
twin is planned. Commands: Command.md §7.22.

## What exists

| piece | where |
|---|---|
| Python reference law (source of truth, self-test) | `extensions/.../robotic_arm/utils_controller/modular_adaptive.py` |
| C++ law (pure Eigen) | fsc_autopilot_ros2 `fsc_autopilot_ros2_node/single_aerial_manipulator_modular_adaptive_direct_actuation/client_lib/src/modular_adaptive_law.cpp` |
| C++ node (parallel fork of the whole-body client) | same fork, `autopilot_modular_adaptive_direct_actuation_node` |
| parity test (270 rollout steps, 3 configs, 1e-9 rel / 1e-8 abs) | `fsc_autopilot_lib/single_vehicle_baseline/tests/src/test_modular_parity.cpp` |
| parity fixture generator | `application/robotic_arm/utils/generate_modular_truth.py` |
| sim yaml (GENERATED) | fsc_autopilot_ros2 `config/params_single_aerial_manipulator_modular_adaptive_direct_actuation_t650_sim.yaml` |
| yaml generator (copies plant/SAFETY/allocator/planner verbatim, checks it); shipped gains `analysis/modular_final_gains_rt1.json` | `tools/make_modular_yaml.py` |
| controller stack | fsc_autopilot_ros2 `scripts/isaacsim/start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh` |
| Pegasus launcher | `scripts/indoor_sim/start_t650_aerial_manipulator_modular_adaptive_direct_actuation_sitl.sh` |
| launch file | fsc_autopilot_ros2 `launch/single_aerial_manipulator_modular_adaptive_direct_actuation_launch.py` |
| offline bench adapter (circle_bench, unchanged, with the law swapped) | `tools/modular_bench.py`, `tools/mapped.py` |
| CMA-ES tuner + finalist evaluation | `tools/tune_modular.py`, `tools/final_eval.py` |

## Design choices where the paper is silent (also printed by the node at startup)

1. `Λ ξ := Λ(λ1 e + λ2 ė)` with P solved for that closed-loop A (Table II lists Λ and λ; eqs. 9/35
   agree only in that form; Theorem 1's proof is unchanged).
2. `Q = diag(q_e, q_v)` per axis (Table II's Q is 3×3 against a 6×6 P).
3. **The one structural addition:** `τ_p += m g e3` (nominal weight). Without it Table II's gains
   would carry the 36.7 N weight through `ρ r / ϖ`, i.e. metres of error. The arm gets NO gravity
   model (configuration-dependent, and the paper claims no model).
4. Desired attitude rates from the planner's flat reference (CoM chain to snap + heading chain);
   `R_d` built from the commanded `τ_p` and the reference heading, as the paper does.
5. `χ̈` by first-order-filtered differences of `[ṗ; Ω; α̇]`; `α̈_d` the same on the streamed `α̇_d`.
6. Adaptive laws integrated exactly over a tick (ZOH), not Euler.
7. The base reference `p_d = x_cd − R r_0c(α_d)` from the planner's own model (the reference layer,
   the same conversion the decoupled bridge does), so both rigs fly the same planned EE motion.

## Findings so far

- Table II LITERAL on this plant: abort at 1.7 s (arm sags 26° — `M̄_αα K1` = 0.3 N·m/rad against
  0.7 N·m of gravity; `M̄_qq` = 0.015 is ~1/6 of this vehicle's inertia).
- Table II's POLES with `M̄` = the nominal inertias: abort at 5 s (attitude parks 9–17° against the
  0.2 N·m arm moment; no integral action anywhere in the law).
- Q = I makes the adaptive term inert on stiff poles (`|r|` ≈ 1e-4); scaling Q up makes the
  switching gain climb until the loop goes unstable against the 10 rad/s rotor-lag pole; the
  `K̂₃‖χ̈‖` term feeds on squared acceleration noise. The paper's per-term leakage
  (`ν0` small, `ν1..3` large) is what makes the adaptive term usable.

## Results (2026-10-01) — full tables in Command.md §7.22.4 and `report.html`

| | whole-body L1 (shipped H1b) | modular adaptive (tuned) |
|---|---|---|
| Isaac EE mean, 2 flights [mm] | 62.1 / 62.2 | 54.8 / 55.0 |
| Isaac EE error radial / along-track / vertical [mm] | −44 / −53 / −1 | +5 / −17 / −50 |
| Isaac steady hover offset [mm] | 0.4 / 1.6 | 40.5 / 39.8 |
| bench RTF 1, EE rms, 3 seeds [mm] | 16.5 / 15.3 / 15.1 | 65.9 / 65.9 / 65.3 |
| bench robustness plant [mm] | 15.1 | 296.2 |
| bench Isaac clock (RTF 0.48) [mm] | 61.1 | 45.7 |

Isaac's wall clock (RTF 0.48) reverses the ordering; the bench reproduces Isaac's whole-body number to
1 mm under that clock, so the RTF-1 bench is the hardware-relevant comparison.

## Real time (2026-10-01, `../rtf_profile_20261001/`, Command.md §7.23)

Isaac now runs at RTF 1 on shiqi-desktop. Re-flown: **whole-body 15.6 / 14.9 mm rms, modular
76.9–77.1 mm** (3 flights) — the RTF-1 bench's ordering, 5×. The §7.22 modular tune completed only 1 of
3 real-time flights (the DIRECT-entry transient fed its adaptive switching gain to a tilt trip), so the
leakage on K̂₁₋₃ was raised 10× in all three modules: `analysis/modular_final_gains_rt1.json`, both sim
yamls regenerated from it, identical tracking on bench and in Isaac, 3/3 completed.

**`report.html` restructured 2026-10-01 (v4) to RTF-1 only, user request.** The RTF-0.48 numbers
(this README's table above, and every `isaac048`/bench-RTF-0.48 figure the earlier versions
carried) are dropped from the published report entirely — not shown even for contrast — since
they are not the hardware-relevant comparison. Two sections only: (1) tracking performance —
the 3-D end-effector/base trajectory figure (`rtf_profile_20261001/figures/rtf1_compare_3d.png`,
`tools/pose_rmse.py` for the RMS table: EE/base position, EE/base heading, joints, pose-level
only, no velocities) over WB-1/2/3 and MOD-1/2/3 (the shipped ×10-leakage tune); (2) simulation
settings — the injected plant uncertainties (`sim2real`-identified mirror plant: kf scale/sag,
km scale, rotor lag, body-fixed force/torque bias, CoM offset, arm friction/current-noise/
velocity-lag, mocap noise) and both controllers' gains, read verbatim off the generated sim
yamls. Source template: `report_template.html` (single source now — the stale RTF-0.48 template
was retired into it, no parallel copy kept).

## Files

- `data/` — driver npz + logs. `wb_c24a` / `modular_c24a` are the two failed attempts (gate below the
  mocap noise floor; stage-2 arm tune with a 16–24 ms arm-delay margin). The comparison flights are
  `wb_c24b`, `wb_c24c`, `modular_c24b`, `modular_c24c`.
- `analysis/` — CMA logs (stage 1/2/3), `modular_final_gains.json`, `final_eval*.json`, `score_isaac.json`,
  `isaac_*.json` (hover offsets, path geometry, adaptive-term share), `bench_series.json`, report payload.
- `tools/` — `modular_bench.py`, `mapped.py`, `tune_modular.py`, `final_eval.py`, `bench_series.py`,
  `make_modular_yaml.py`, `report_data.py`.
- `report_template.html` → `report.html` (payload injected from `analysis/report_payload.json`).
