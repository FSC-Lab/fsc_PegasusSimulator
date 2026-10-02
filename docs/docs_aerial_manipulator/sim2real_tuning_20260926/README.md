# Sim-to-real flight performance tuning — the mirror plant (2026-09-26)

Question (user): with three whole-body L1-4D hardware campaigns in hand (0918 mission,
0921 circle attempt, 0924 circles), make the simulator behave like the real vehicle
**at the flown control gains** — tune the plant, not the controller — and keep the
old stress-test configuration for robustness work.

## What changed

* **Two 4-D sim yamls** in fsc_autopilot_ros2/config:
  * `..._whole_body_l1_4d_direct_actuation_t650_sim.yaml` — NEW, the **experiment
    mirror**: section 2 is the hardware yaml verbatim (gains, observer, allocator's
    believed kf/km, thrust map, guards); section 1 is the plant identified here;
    section 3 lists the three sim-topology keys that must differ.
  * `..._sim_robustness.yaml` — the former `_sim.yaml`, byte for byte (config A).
  * `WB_SIM_PROFILE=mirror|robustness` (default mirror) on BOTH the stack script and
    the Pegasus launcher; `WB_SIM_YAML=<path>` overrides. Campaign scripts written
    before this date point at the robustness file explicitly.
* **New plant knobs** (06 + servo_model.py + launcher lib + base launcher), all
  default off: `sim_plant_km_scale`, `sim_plant_rotor_lambda`,
  `sim_plant_kf_sag_per_min`, `sim_plant_force_bias_{x,y,z}`,
  `sim_plant_torque_bias_{x,y,z}`, `sim_arm_vel_lag_s`, `sim_arm_vel_quant_rad_s`,
  per-joint `sim_arm_friction_scale_j1..j4`.
* Campaign driver: `--mission "ee:...;home;base:...;..."` replays an explicit leg
  list (the 0918 sequence).

## Identification (tools/fit_plant.py → analysis/fit_plant.{json,txt})

| quantity | method | result |
|---|---|---|
| kf | hover thrust balance kf = kf_bel·mg/u1 over planner holds | 4.03–4.31e-05 by pack state, mean 4.19e-05 (×1.037 of the plant) |
| battery sag | linear kf(t) over each DIRECT window | −2.6…−4.5 %/min |
| yaw coefficient | same-command yaw gain vs the bench c | 0.43–0.78 (0921 #3: 0.78) → 0.70× bench |
| roll/pitch inertia | same-command gains at the coupled home inertia | 1.11/1.03 on 0921 #3 (corr 0.72/0.83) → nominal |
| rotor lag | delay scan of the roll/pitch response | ≤10 ms beyond λ=10.03 → unchanged |
| standing force | filtered d̂_t in every hover hold | (+0.55, −0.50) N body FLU, all flights |
| standing yaw torque | filtered d̂_r,z | −0.07…−0.13 N·m |
| arm friction | 0924 friction_id + breakaway | j2 0.95, j3 0.75, j4 1.5 (0913 ground test), j1 1.0 |
| joint-velocity feedback | reported velocity vs d/dt position | 44–56 ms lag, 0.024 rad/s quantum |

## Replays (tools/replay.sh)

`replay.sh f18|a2|a3 <tag>` flies one hardware mission in Isaac on the mirror plant
with that flight's kf/sag, records the hardware bags' topic set to `bags/<tag>`,
and restores the yaml. Because Isaac runs at **RTF ≈ 0.48** on this desktop while
the law and planner run on the wall clock, the runner (not the yaml) scales the
planner's kinematic bounds by RTF, requests EE-trajectory time scale s = RTF, and
expresses the rotor and arm-velocity lags in wall time (λ/RTF, τ·RTF). The first
attempt without the lag rescaling (`data/sim_f18_r1_plantlags.npz`) diverged in a
growing roll oscillation 16 s into DIRECT: the controller was seeing ~210 ms of
rotor lag and ~100 ms of velocity lag, twice what hardware has.

Then `extract_bag.py bags/<tag> data/sim_<tag>.npz`, `compare.py data/<flight>.npz
data/sim_<tag>.npz <tag>` and `build_report.py`.

Outputs: `analysis/`, `figures/`, `report.html` (the published artifact).

### Replay attempt log (0918 = f18, 0924 F2 = a2)
| tag | outcome | lesson |
|---|---|---|
| f18_r1 | diverged 16 s into DIRECT (roll, 0.26 Hz) | plant lags must be expressed in wall time at RTF 0.48 |
| f18_r2 | stable 135 s; home legs mis-targeted | home = the planner's go_home service, not an EE target |
| f18_r3 | full 5-leg sequence, no abort | the 0918 comparison in the report |
| a2_r1 | never lifted off | kf at lift-off (SAFETY UDE −3 N → ×1.030), not the DIRECT-hold value |
| a2_r2–r4 | circle flown at s = 0.09–0.11 (4× slow) | s_max is the CoM/EE-speed bound; a safe copy of the yaml taken mid-replay carried the temporary RTF scaling into later runs |
| a2_r5 | killed-shell attempt's driver survived and aborted the relaunch mid-run | never relaunch a replay while `pgrep -f ee_trajectory_sim_driver` finds one; launch replays with `setsid nohup` |
| a2_r6 | circle at s = 0.48 (= RTF) of s_max 1.13, no abort | the 0924 F2 comparison in the report |
| f18_r4 | 0918 sequence with the mocap rate/noise on | the 0918 comparison in the report |
