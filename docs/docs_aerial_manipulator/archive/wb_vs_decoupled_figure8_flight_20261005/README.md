# 2026-10-05 figure-8 hardware flights — whole-body 4-D L1 vs decoupled geometric + L1

Bags: `docs/experimental_data_ros2_bag/1005 -  T650-AM whole-body vs decoupled Figure-8-*/` (note the two spaces after "1005 -")

| run | bag | speed | planner run |
|---|---|---|---|
| WB-3, WB-4 | `flight_wb_l1_4d_figure8_vel010_20261005_162406` | 0.10 m/s | runs 1 and 2 (EXECUTING T=46.7 s) |
| DEC-5 | `flight_decoupled_l1_figure8_vel010_20261005_163734` | 0.10 m/s | one run; second DIRECT engagement (operator reverted the first) |
| DEC-6 | `flight_decoupled_l1_figure8_vel013_20261005_165431` | 0.13 m/s | one run (EXECUTING T=36.8 s) |
| WB-5, WB-6 | `flight_wb_l1_4d_figure8_vel013_20261005_170129` | 0.13 m/s | runs 1 and 2 |

Plan: A 0.70 / B 0.35 m figure-8, long axis on world x, centred on the gripper's position at selection, path 4.268 m,
q2 = 25° ± 15° four cycles per lap, fold 55°. Gains = the circle flights' (Tables 3 and 4 of the report), tilt
watchdog 15° on both (fsc_autopilot_ros2 `d85be8a`). All six runs completed, 0 motor saturation, no mocap faults.

Report: section 2 of "Experiment: Free-flight Comparison 0928 + 1002 + 1005"
(https://claude.ai/artifact/LhpXomd3ooNuKquPoQ9Joq), built by `../decoupled_flight_20261002/tools/build_summary_report.py`
from `analysis/fig8_{metrics,ee3d}.json`. Same definitions as section 1 (the 0928 `metrics.analyse`), each run scored over
its whole EXECUTING span with 0.1 s trimmed at each end.

## Run order

1. Extract with the 0928 extractor (ROS sourced, numpy-2 system python), then `np1_compat.py` on the SAME interpreter,
   and link the run names the scripts use:
   ```bash
   cd ../wb_vs_decoupled_flight_20260928/tools
   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   B="<bag root>/1005 -  T650-AM whole-body vs decoupled Figure-8"
   /usr/bin/python3 extract_bag.py "$B/flight_wb_l1_4d_figure8_vel010_20261005_162406"   $AM_NPZ/f1.npz
   /usr/bin/python3 extract_bag.py "$B/flight_decoupled_l1_figure8_vel010_20261005_163734" $AM_NPZ/f2.npz
   /usr/bin/python3 extract_bag.py "$B/flight_decoupled_l1_figure8_vel013_20261005_165431" $AM_NPZ/f3.npz
   /usr/bin/python3 extract_bag.py "$B/flight_wb_l1_4d_figure8_vel013_20261005_170129"   $AM_NPZ/f4.npz
   /usr/bin/python3 np1_compat.py $AM_NPZ/f[1-4].npz                  # NOT with PYTHONNOUSERSITE=1
   cd $AM_NPZ && ln -s f1.npz w3.npz && ln -s f1.npz w4.npz && ln -s f2.npz r5.npz && ln -s f3.npz r6.npz \
              && ln -s f4.npz w5.npz && ln -s f4.npz w6.npz
   ```
2. From `tools/`: `AM_NPZ=<dir> PYTHONNOUSERSITE=1 /usr/bin/python3 check_conventions.py` (joint order/sign, FK vs the
   planner's `current_ee`, the bridge conversion — all sub-mm on these bags), then `summary_data.py`
   -> `analysis/fig8_metrics.json`, `analysis/fig8_ee3d.json`.
3. `cd ../decoupled_flight_20261002/tools && python3 make_templates.py && python3 build_summary_report.py &&
   python3 build_artifact_page.py <out.html>`; check with `python3 measure_layout.py <out.html> 1440|1100|500`.

`tools/f1005.py` is the shared setup: run table, and `windows()` = the n-th planner `EXECUTING T>=20 s` span of a bag.

## Results (Table 9 of the report)

| | WB-3 | WB-4 | DEC-5 | WB-5 | WB-6 | DEC-6 |
|---|---|---|---|---|---|---|
| speed [m/s] | 0.10 | 0.10 | 0.10 | 0.13 | 0.13 | 0.13 |
| EE position rms [mm] | 37.3 | 43.7 | 72.6 | 42.7 | 58.2 | 76.9 |
| EE heading rms [deg] | 0.46 | 0.48 | 3.58 | 0.54 | 0.48 | 4.69 |
| airframe position rms [mm] | 36.8 | 43.6 | 66.6 | 42.1 | 58.3 | 68.7 |
| tilt, worst [deg] | 2.21 | 2.20 | 4.18 | 2.18 | 2.29 | 4.26 |

The whole-body EE error is larger than on the 09-28 circle (24–26 mm); the decoupled one is close to its 10-02 circle
(73–78 mm).

## Traps met here
- The archived 0928 `common.py` resolved the repo root four levels up (written before the move into `archive/`); fixed to
  five, otherwise `transition_planner` does not import.
- The bag directory name has two spaces after "1005 -": quote every path.
