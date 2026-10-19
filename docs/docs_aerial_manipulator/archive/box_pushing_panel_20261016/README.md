# Box Pushing tab: two margins + editable Box row (2026-10-16)

User request: lay the Box Pushing tab out like the Pick & Drop tab -- a Vertical
and a Side Margin in a column right of the points -- and make the Box row
editable.

- Vertical Margin = `push_pull_approach_dz` (the old "Safety Margin"), Side
  Margin = `push_pull_approach_back`. Push Distance and Turn sit in the same column.
- Box row = the planner's new `push_pull_box_point` (fsc_trajectory_planner),
  the box capture's parameter twin, like `pick_place_pick_point`.

Check (no Isaac, no PX4, private ROS domain): `bash tools/gs_box_check.sh` --
the real panel offscreen against the real planner; screenshots
`gs_box_{0_not_captured,1_after_get,2_after_edit}.png`. Planner side:
`fsc_trajectory_planner/test/test_push_pull_box_point_loopback.py`.
