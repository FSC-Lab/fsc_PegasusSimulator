# 2026-10-02 decoupled (geometric + L1) flights on the 2026-10-01 tune — analysis

Bags: `docs/experimental_data_ros2_bag/1002 - T650-AM geometric L1 adaptive Circle and Gamepad-*/`
- `flight_decoupled_l1_circle_20261002_134247` — the EE circle flown twice in one DIRECT engagement (run 1, run 2)
- `flight_decoupled_l1_circle_ps420261002_134830` — PS4 teleoperation only

Report: `report.html` = ONE section (2026-10-03, user request): the six circle runs of 09-28 and 10-02 -- flight tags,
the gains as flown, 3D trajectories, one full-state error table, every run scored on its first 27.2 s
(`tools/summary_data.py` -> `tools/build_summary_report.py`). The earlier multi-section version (0928 report + a
section 3, artifact version 3) is rebuilt by `tools/build_report.py` into `report_v3_sections.html`; published as the artifact "Experiment: Free-flight Comparison 0928 + 1002"
(https://claude.ai/artifact/LhpXomd3ooNuKquPoQ9Joq). Command.md §7.24.4 has the summary and the run-2 diagnosis.
Since 2026-10-05 the page has a section 2: the six figure-8 runs of 10-05 in the same layout (Tables 6-9, Fig. 2), data
from `../wb_vs_decoupled_figure8_flight_20261005/` (its `tools/summary_data.py`; see its README), title "... 0928 + 1002 + 1005".

## Run order

1. Extract with the 0928 extractor (ROS sourced, numpy-2 system python), then `np1_compat.py` on the SAME interpreter:
   ```bash
   cd ../wb_vs_decoupled_flight_20260928/tools
   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   /usr/bin/python3 extract_bag.py "<bag>/flight_decoupled_l1_circle_20261002_134247" $AM_NPZ/e1.npz
   /usr/bin/python3 extract_bag.py "<bag>/flight_decoupled_l1_circle_ps420261002_134830" $AM_NPZ/e2.npz
   /usr/bin/python3 np1_compat.py $AM_NPZ/e1.npz $AM_NPZ/e2.npz          # NOT with PYTHONNOUSERSITE=1
   ln -s $AM_NPZ/e1.npz $AM_NPZ/r1.npz; ln -s $AM_NPZ/e1.npz $AM_NPZ/r2.npz; ln -s $AM_NPZ/e2.npz $AM_NPZ/tp.npz
   ```
   The 09-28 decoupled bags are needed too (`d1`, `d2` — `_182835`, `_183608`) for the figures and the diagnosis.
2. Everything below: `AM_NPZ=<dir> PYTHONNOUSERSITE=1 /usr/bin/python3 <script>` from `tools/`.
   - `analyse.py` → `analysis/{metrics,ee_budget,mechanism}.json` (Tables 8–11)
   - `fig_data.py` → `analysis/{ee3d_dec,err_ts_dec,teleop_ts}.json` (Figs. 4–6)
   - `run2_diagnosis.py` → `analysis/run2_diagnosis.json`; `lateral_mode_model.py` → `analysis/lateral_mode_model.json`
3. `python3 make_templates.py && python3 build_report.py && python3 build_artifact_page.py <out.html>`;
   check with `python3 measure_layout.py <out.html> 1440|1100|500`.

`c1002.py` is the shared setup: it imports the 0928 `common.py`/`metrics.py` and overrides the scoring windows
(both runs on their first 27.2 s; run 2 was cut 0.8 s before its end by the operator's SAFETY command).

## Traps met here
- `np1_compat.py` must run on the numpy-2 interpreter that wrote the npz (`/usr/bin/python3` with the user site);
  with `PYTHONNOUSERSITE=1` it fails on `numpy._core`.
- Header stamps are not trustworthy across this flight: the Orin clock stepped +0.6 s at 73.57 s and the PX4-clock
  odometry stamps followed at 84.58 s. Score on receive time (as all the scripts do).
- `re.subn` with a replacement string that contains JavaScript (`\d` in a regex) raises "bad escape" — pass a function.
