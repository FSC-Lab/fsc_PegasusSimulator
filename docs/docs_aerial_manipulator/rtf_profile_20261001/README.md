# Isaac at real time (RTF 1) — 2026-10-01

Why Isaac ran at RTF 0.48 on shiqi-desktop, the three fixes that make it real-time, and the 0.5 m
end-effector circle re-flown on both rigs at RTF 1. Commands and tables: Command.md §7.23.

## Fixes (Pegasus repo)

| fix | where | effect |
|---|---|---|
| arm gravity evaluated on demand, not every physics step | `application/robotic_arm/06_px4_t650_aerial_manipulator_free_flight.py` `g_arm_now()` | callback 3.98 → 0.79 ms; RTF 0.484 → 0.872 windowed, 1.014 headless |
| Isaac pinned to the P-cores | `ISAAC_CPUS` (machine conf) / `PEGASUS_ISAAC_CPUS` → `taskset -c` in `start_single_drone_x650.sh` | main thread no longer parked on an E-core (that alone measured RTF 0.76) |
| real-time pacer | 06 `_RealTimePacer`, `PEGASUS_REALTIME` / `SIM_REALTIME` | sim time held to the wall clock, `RTF x.xxx over the last 10.0 s` printed every 10 s |
| windowed runs step physics singly | 06 `PEGASUS_RENDER_EVERY` (default 8 under the pacer) | `world.step(render=False)` + `world.render()` every N instead of 4 physics steps per frame |
| `shiqi_machine.conf` | `SIM_RTF=1.0`, `SIM_REALTIME=1`, `ISAAC_CPUS=0-15` | the mirror plant's lags are no longer rescaled |

## Profiles

`prof_baseline_render.{txt,prof}` (before), `prof_lazy_render.*`, `prof_lazy_headless.*` (after the
lazy-gravity fix). Open a `.prof` with `python3 -m pstats`.

## Flights (`data/`, all mirror plant, circle r 0.5 m / 24 s lap / q2 25 ± 15° @ 6 s / fold 55°)

| file | rig | mode | result |
|---|---|---|---|
| `wb_prof1-3` | WB | profiling runs at time scale 0.48 | EE numbers INVALID (wrong clock/scale pairing) |
| `wb_rt1a`, `wb_rt1b` | WB | headless, RTF 1 | EE rms 15.6 / 14.9 mm |
| `wb_rt1g` | WB | windowed, old 4-steps-per-frame | CRASHED (1.55 Hz roll mode grew in hover) |
| `wb_rt1g2` | WB | windowed, single steps | EE rms 16.7 mm |
| `modular_rt1a/b/c` | MOD (§7.22 tune) | headless, RTF 1 | 76.7 mm / trip / trip |
| `modular_nu10a/b/c` | MOD, ν₁₋₃ ×10 (`variants/modular_sim_nu123x10.yaml`) | headless, RTF 1 | 76.9 / 77.0 / 77.1 mm |
| `wb_rt1d`, `wb_rt1e` | WB | headless, pinned from launch, RTF 1 | 15.5 / 14.5 mm (replace `rt1a` — pinned by hand mid-startup — and `rt1g2` — window open) |
| `decoupled_geo1/2/3` | geometric L1, `variants/geometric_l1_mirror_sim.yaml`, start gate 0.25 m | headless, RTF 1 | 209.8 / 199.0 / 210.3 mm |

**The report's nine flights** (all one condition): WB `rt1b`, `rt1d`, `rt1e`; GEO `geo1–3`; MOD `nu10a–c`.
Geometric rig: `tools/make_geometric_mirror_yaml.py` builds its controller yaml on the mirror plant
(checked key by key against the whole-body mirror); 06's `_arm_gravity()` (exact, ~6.5x cheaper than
`dynamics()`) is what lets its position-mode arm run at RTF 1 (was 0.72). Command.md §7.23.5.

`logs/isaac_pane_*.txt` hold each run's pacer RTF series; `logs/cycle_*.log` the cycle, `data/logs/` the
driver/stack/Pegasus logs.

## Analysis

- `analysis/score_rt1.json` — `am_ee_compare_score.py` on every RTF-1 flight.
- `analysis/pose_rmse_rt1.json` — `tools/pose_rmse.py`: the report table (EE/base position + heading, joints, RMS).
- `analysis/sim_budget.json` — `tools/sim_budget.py`: the 0928 hardware report's exact EE error split
  (airframe / attitude / joints), arm-sweep band share and detrended CoM spectrum, on Isaac flights.
  WB at RTF 1: 15.3 = 13.7 / 5.3 / 5.4 mm against hardware 26.2 = 25.3 / 6.7 / 8.2; the gap is the
  airframe swing at the 6 s arm sweep (24 vs 77 mm p-p).
- `analysis/delay_sweep.json` — `tools/delay_sweep.py`: bench rotor-delay margin, both laws complete
  at 28 ms and abort at 40; softened modular attitude modules abort at 16.
- `analysis/report_payload_rt1.json` — `tools/report_rt1.py`, injected into
  `../modular_adaptive_20260930/report_template.html` → `report.html` (artifact v2).

## Reproduce

```bash
cd ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/rtf_profile_20261001
./run_rt1.sh wb:<tag> modular:<tag>            # PEGASUS_HEADLESS=0 for a window
```
