# 0928 whole-body vs decoupled circle flights — analysis tools

Bags: `docs/experimental_data_ros2_bag/0928 - T650-AM whole-body vs decoupled Circle-*/` (WB-1 `_171154`, WB-2 `_172242`,
DEC-1 `_182835`, DEC-2 `_183608`); last week's whole-body flights come from the 0924 bags (F1 `_120228`, F2 `_120546`,
F3 `_165654`, F4 `_172101`). Report: `../report.html`, published as the artifact "0928 Circle Comparison".

Run order:

1. `extract_bag.py <bagdir> <out.npz>` with ROS sourced (`/opt/ros/humble` + `~/ros2_ws/install`), system python
   (numpy 2). Names the scripts expect: `w1 w2 d1 d2 a1 a2 a3 a4`.npz in `$AM_NPZ` (default: this session's scratchpad).
   It adds the decoupled topics to the 0924 extractor (`l1_control_debug`, the bridge output `reference_direct`, the
   position-mode arm topics, planner `current_base`/EE-trajectory topics).
2. `np1_compat.py <npz...>` — rewrites the object arrays so numpy 1.21 can read them. Everything below runs on
   `PYTHONNOUSERSITE=1 /usr/bin/python3` (numpy 1.21 + the apt matplotlib/scipy).
3. `check_conventions.py` — verifies what `common.py` assumes: joint order/sign, odometry velocity frame, FK against the
   planner's `current_ee` and the law's x_c / task error, the airframe-reference conversion against the bridge.
4. `metrics.py` -> `../analysis/metrics.json` — full-state RMSE over the circle run, one definition for both rigs
   (Table 1 and the task/airframe rows of Table 5). `ee_budget.py` — the exact EE error split into airframe position,
   attitude and joints (Table 2). `mechanism.py` — the body-fixed lateral force and what each law does with it
   (Table 3). `sweep_band.py`, `com_spectrum.py` — how much error sits at the arm-sweep frequency.
5. `arm_stats.py` (joint tracking + torque delivery, the 0924 report's definitions), `holds.py` (hover before the
   circle), `ff_check.py` (was the new friction feed-forward live: amplitude, sign source, value while stuck).
6. `fig_data.py` -> `ee3d.json`, `err_ts.json`, `arm_trk.json` for the three interactive figures
   (templates `ee3d_section.html`, `err_section.html`, `arm_trk_section.html`).
7. `build_report.py` -> `../report.html` (tables generated from the JSON; style in `report_style.css`, the 0924 report's).
   Then `python3 build_artifact_page.py <out.html>` (refreshes the outline via `add_outline.py`) and
   `python3 measure_layout.py <out.html> <width>` at 1440 / 1100 / 500.

Traps met here:
- The bag timeline starts at the first recorded message; `/rosout` lines latched at recorder start carry receive time.
- The recorder's subscription to the position-mode arm reference hit a QoS durability mismatch on DEC-1; the joint
  reference used for the decoupled flights is `q_d` from the planner's `WholeBodyReference`.
- A reference peak speed taken as a max over receive-stamped samples is set by one spike across a feedback gap
  (WB-2 read 20.5 deg/s on a 15.6 deg/s plan) — use a percentile.
- System python is 3.10: no same-quote nesting inside f-strings.
