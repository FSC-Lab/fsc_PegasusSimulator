# 0924 whole-body L1-4D circle flights — analysis tools

Run order (system python with ROS sourced for the extractor; `PYTHONNOUSERSITE=1 /usr/bin/python3` for the rest,
the apt matplotlib is built for numpy 1.x):

1. `extract_bag.py <bagdir> <out.npz>` — generic rosbag2 -> npz flattener (0921's + `esf`, `torque_in`, `ee_trajectory/*`).
   The scripts expect `a1.npz` (flight 1, `_120228`), `a2.npz` (flight 2, `_120546`), `a3.npz`/`a4.npz` (flights 3/4), `c3.npz` (0921 `_123637`) and
   `f18.npz` (0918 `_144828`) in one directory; edit the path in `common.py:load()`.
2. `feedback.py` — feedback-quality comparison across the four flights (SEL=a1,a2 to restrict).
3. `drift.py`, `drift2.py` — flight-1 drift anatomy (1 s table, 0.1 s / 0.2 s zooms, EV input, EKF flags).
4. `phases.py` — per-phase tracking metrics, lap-periodic base error, mocap health per heading sector.
5. `arm.py` — joint tracking of the streamed sinusoid, lag, torques, friction signature.
6. `noise.py` — PSD bands of feedback, law outputs and sensors.
7. `figures.py` — report figures 3, 4, 7 and 11 (files `f1_`–`f4_`); `ee_figure.py` — figure 2 (`f_ee_errors.png`, EE x/y/z/yaw error, all four flights) and its per-phase numbers.
   `ee_check.py` verifies absolute = task + base error and the heading convention; `ee_joint_attrib.py` maps q − q_d through the model FK.
8. Interactive figures 1 and 2: `ee3d_data.py` -> `ee3d.json` (3D circle trajectories, all four flights) and `ee_err_data.py` -> `ee_err.json`
   (EE error by channel, 25 Hz bin means), then `embed_interactive.py <dir with both json>` puts the templates `ee3d_section.html`
   (four independently rotatable 3D views + line toggles) and `ee_err_section.html` (signal toggles, shared/own y-scale, flight picker on
   narrow screens) into report.html between `<!--ee3d:*-->` / `<!--eeerr:*-->` markers (idempotent). Static fallbacks, shown only if
   Plotly fails to load: `ee3d_static.py` -> `f_circle_3d.png`, `ee_figure.py` -> `f_ee_errors.png`. Old note: `ee3d_section.html` = the interactive Plotly block embedded in report.html (the JSON is inlined at `__DATA__`).
9. `probe_all.sh` — planner feasibility probe of faster/larger arm sinusoids (uses `l1_4d_planner_20260917/ee_plan_probe.py`).

Outputs as run on 2026-09-24 are in `../analysis/`.
Trap: the fused estimator stamps `local_position/odom` on PX4's clock (hdr − recv ≈ −2.5e9 s); use receive time.
10. Mocap freeze source (report section 3, fig. 5): `gap_catalog.py` (every mocap gap: duration, heading,
    staleness of the first frame back, orientation snaps; per-heading skip rates), `imu_deadreckon.py`
    (IMU dead-reckoning through the 2.85 s pause, validated on four clean 2.85 s windows),
    `mocap_source_fig.py` (figure 8; run from the directory holding `imu_deadreckon.py`).
    Processor source: git@github.com:FSC-Lab/fsc_optitrack_processor_ros2 (the HTTPS URL needs credentials).
11. Flights 3 and 4 (16:56, 17:21; the RMSE table is report section 2, torque and friction section 4, VRPN section 3): extract as `a3.npz`/`a4.npz` (the extractor now also
    takes `/vrpn_mocap/uav_0/pose`). `rmse_table.py` (circle-phase RMSE of the full state + EE 4-D pose, all four
    flights -> `rmse_rows.json`), `torque_check.py` / `torque_lp.py` (commanded vs applied arm torque; the LP
    version scores against the INTENDED torque = law command + gravity correction + friction FF, which is the
    right comparison once the back-EMF duty is active), `friction_id.py` (in-flight friction from
    applied − S·g(q) − I·q̈ vs measured velocity, using `controller.dynamics`), `burst_stats.py` (stick–slip
    statistics and the task impedance's joint stiffness), `vrpn_check.py` (raw VRPN stream health),
    `f9_f10_f11.py` (figures 9, 10 and 6; file names keep their original numbers).
12. `build_artifact_page.py <out.html>` — builds the publishable page from `report.html` (title + font link +
    style + body only; all layout rules live in report.html). Layout rules that must hold: no `max-width` cap on
    paragraphs or list items, every `<table>` inside a `.tscroll` div, one `ul, ol` indent.
13. `add_outline.py` (three levels: h2 sections, h3 subsections, h4 parts) — regenerates the clickable outline in `report.html` (ids on every heading, the Contents
    block under the subtitle, the floating "Contents" bar with a current-section label, its CSS and script), in
    place and idempotently. `build_artifact_page.py` runs it first, so adding an `<h2>`/`<h3>` to the report is
    all it takes to get a new outline entry. Number sections in the heading text ("8. ...") — the outline
    reuses those numbers. Verified in headless Chrome: every link lands the heading 60 px below the top, clear of
    the 40 px bar (the last, short section cannot scroll that far).

14. Report structure (2026-09-25): 1 timeline · 2 tracking statistics of all four flights (RMSE table, four-flight 3D
    view, EE error by channel) · 3 OPEN questions (3.1 arm sinusoidal trajectory tracking) · 4 ARCHIVED questions
    (4.1 the flight-1 drift, 4.2 the configuration changes) · 5 traps. A question moves between 3 and 4 by moving
    its <h3> block; number it "3.N"/"4.N" and the outline follows. Figures are numbered in document order, so
    figure numbers do not match file names: 1 = 3D (interactive, fallback `f_circle_3d.png`), 2 = `f_ee_errors`,
    3 = q2/q3 tracking + joint torques (interactive only), 4 = `f10_` (friction ID), 5 = `f1_`, 6 = `f2_`, 7 = `f8_`,
    8 = `f11_`, 9 = `f4_`. `f3_`, `f7_`, `f9_` (the old per-pair arm figures and the sim check) are no longer in the
    report since 2026-09-26; their information is in figure 3 and the 3.1 statistics tables.
15. Figure 3 (2026-09-26): `arm_trk_data.py` -> `arm_trk.json` (q, q_d from wb_control_debug; torques law / intended /
    applied with torque_lp.py's exact processing, so its circle-run rms matches the torque table), template
    `arm_trk_section.html`, embedded by `embed_interactive.py` before 3.1's first part. `measure_layout.py` = the
    headless-Chrome layout audit (was a scratchpad script; the scratchpad is wiped on restart, so it lives here now).
16. Section 3.1 statistics (2026-09-26): `arm_stats_table.py` -> `arm_stats.html` (two tables: joint tracking, torque
    delivery; circle run, all four flights, definitions in the file header) + `arm_stats.json`; `embed_interactive.py`
    swaps it into report.html between `<!--armstats:start-->` / `<!--armstats:end-->`. 3.1 is now: fig. 3 -> the
    tables -> causes ranked with data -> fixes ranked. Trap: a raw `np.gradient` of the reference on the bag's receive
    stamps reads hundreds of deg/s on a reference that never exceeds 17 — resample to a uniform grid first.
