# 4-D whole-body L1 rig on the C++ trajectory planner — EE circle and figure-8 (2026-09-17)

Two Isaac flights of the four-dimensional-attribution rig (Command.md §7.17) flying
`fsc_trajectory_planner`'s END-EFFECTOR TRAJECTORY mode. The purpose was to check
that the 4-D controller works with the planner that replaced the Python one on
2026-09-14. Machine `shiqi_machine` (shiqi-desktop). Plant: §7.15 config A.
Write-up and tables: Command.md §7.17.5.

## Reproduce

```bash
cd ~/fsc_PegasusSimulator
WB_L1_MISSION=ee_circle  WB_L1_EE_SCALE=0.8 WB_L1_OUT=$PWD/docs/docs_aerial_manipulator/l1_4d_planner_20260917 \
  application/robotic_arm/utils/wb_l1_tune_cycle.sh l1_4d ee_circle_A  shiqi_machine
WB_L1_MISSION=ee_figure8 WB_L1_EE_SCALE=0.8 WB_L1_OUT=$PWD/docs/docs_aerial_manipulator/l1_4d_planner_20260917 \
  application/robotic_arm/utils/wb_l1_tune_cycle.sh l1_4d ee_figure8_A shiqi_machine
PYTHONNOUSERSITE=1 /usr/bin/python3 docs/docs_aerial_manipulator/l1_4d_planner_20260917/ee_run_score.py \
  docs/docs_aerial_manipulator/l1_4d_planner_20260917/l1_4d_ee_*_A.npz
```

The driver is the planner package's installed `ee_trajectory_sim_driver.py`. It
publishes what the arm GS's "EE trajectory" tab publishes: select, time scale,
go_to_start, start.

## Files

| file | what |
|---|---|
| `scores.txt` | `ee_run_score.py` output for both runs |
| `driver_ee_{circle,figure8}_A.log` | driver event log (phases, service replies, EE tracking summary) |
| `cycle_ee_{circle,figure8}_A.log` | the tune cycle's stage log |
| `ee_run_score.py` | the scorer: raw EE error, best time-lag fit and residual, radius (circle), path extent |
| `l1_4d_ee_*_A.npz` | raw recordings (gitignored, like every campaign's npz) |

## Result

| run | s / s_max | raw EE err mean / max | lag | residual | flown vs commanded |
|---|---|---|---|---|---|
| circle 0.5 m ×2 | 0.905 / 1.131 | 124 / 178 mm | 1.05 s | 55 mm | radius 0.450 of 0.500 m |
| figure-8 0.5×0.25 m ×2 | 0.286 / 0.358 | 50 / 106 mm | 1.37 s | 27 mm | 0.970 × 0.464 of 1.0 × 0.5 m |

Both flights completed end to end, with no abort, no refusal and no INFEASIBLE.
**Not comparable with §7.15.12's 6-D circle.** That flight used a different
machine (RTF 0.34), a frictionless arm and the 6-D design. Each shape was flown
once.
