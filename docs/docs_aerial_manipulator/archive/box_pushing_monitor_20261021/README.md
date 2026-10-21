# Box Pushing tab: the MONITOR row (2026-10-21)

User request: a monitoring area under the points of the arm GS's Box Pushing tab,
first item the landing gear's height above the table top.

**What it shows** (`fsc_open_manipulator/utils_custom_ground_station/src/push_pull_panel.{hpp,cpp}`,
`updateGearMonitor()`), row "Gear − table top":

- **now [m]** = `current_base.z − push_pull_gear_depth − push_pull_table[4]`, from the planner's
  `whole_body_planner/current_base` (its odometry position); "—" if older than 1 s.
- **plan min [m]** = the same over the planned body path (`push_pull/drone_path`, after Plan),
  minimum over the samples where the gear is over the table; "—" before Plan.
- "Over the table" is the planner's own rule (`push_pull.cpp`): the body within
  `push_pull_gear_radius + push_pull_table_margin_xy` of the table's footprint. Over it the
  value is green `≥ push_pull_table_clearance`, amber below, red under the top; elsewhere grey
  (the number is still the height difference).
- The five parameters come from the planner in a request of their own (`loadGearParams()`), so an
  older planner without one of them still loads the rest of the tab; parameter events keep them live.

Nominal values (sim yaml): 0.258 at the 1.3 m takeoff hover (not over the table), 0.123 at the
grasp / push, plan min 0.123. Hardware gear 0.248 → 0.117.

**Check** (no Isaac, private ROS domain 77): `tools/gs_monitor_check.sh` builds
`tools/gs_monitor_harness` (the real panel offscreen, linked from the station's
`libjoint_plot_core.a`), starts the real planner on the box-pushing sim yaml, plays
`current_base` at six places and a planned body path, reads the panel's own boxes and colours:
PASS (0.258 grey / 0.123 green / 0.038 amber / −0.052 red; plan 0.123 green). It also writes the
workflow page's `gs_panel.png` / `gs_rows.png` (`gs_monitor_panel.png`, `gs_monitor_rows.png`).

Also changed: the Land row's tooltip (the hover is over the start mark since 2026-10-21).
