# Deployment prompt -- the EXPERIMENT LAPTOP (ground-station computer)

Paste this into the Claude Code session on the laptop that runs the OptiTrack processor, the drone ground station, the
arm ground station and the bag recorder, AFTER the user has reviewed and pushed the 2026-11-02 work and the Orin has been
deployed (`deploy_orin_prompt.md` beside this file).

---

You are on the aerial manipulator's experiment laptop. Bring the arm ground station up to the 2026-11-02 valve/box
isolation. The record is in fsc_PegasusSimulator
`docs/docs_aerial_manipulator/archive/valve_box_isolation_20261102/README.md` (sections 1, 3 and 6).

What changed on the ground side (the arm GS, `utils_custom_ground_station`):
- VALVE TURNING tab: the jaws stay CLOSED in free flight -- the tab closes them at Go To Start (it used to OPEN them
  there), Ready opens them above the handle (the box's way), the gate closes them on the fin only after the planner's
  hand-over (the claw settled and held in the room), and they CLOSE again once the Exit is done or an abort has
  climbed clear. NEW "Static hold [s]" box on the tab: 0 = the turn follows the grasp (as before); > 0 = a STATIC HOLD --
  on the grasp the tab calls the planner's new `push_pull/grasp` (the contact phase on), holds, reports the release
  (contact off) and runs the valve's exit (open with the claw room-held, Exit after the OPEN report). The static hold
  needs NO `push_pull_auto_push:=false` and NO hand close from the Control tab any more -- the planner must hear of the
  grasp, and only the Valve tab tells it. The trim switch `trim_target_includes_error_damping` rides with the mission's
  j1/j4 damping raise and is restored with it.
- BOX PUSHING tab: back to its 10-27 behaviour; the one post-10-27 change it had (opening the jaws on a lost DIRECT from
  the tab's own grip state) is Valve-only again.

Steps:
1. Pull fsc_open_manipulator on the branch the user names (`git fetch --tags` too; `box-pushing-20261027` must exist),
   report its HEAD, and REBUILD `utils_custom_ground_station` (`colcon build --packages-select
   utils_custom_ground_station`). An older station would OPEN the jaws at Go To Start and never call `push_pull/grasp`.
2. Start the arm GS (hardware) and confirm the Valve Turning tab logs `Valve Turning: the jaws stay CLOSED in free flight
   and open at Ready; after the grasp the TURN ...` and shows the "Static hold [s]" box. For the first flight set it to
   10 (or start the station with `-p push_pull_valve_hold_s:=10.0`).
3. Bring-up: CLOSE the jaws from the Control tab BEFORE take-off -- the Valve tab acts only from Go To Start on; the
   take-off itself flies with whatever the bring-up left.
4. Recorder: keep the 2026-11-01 topic list (explicit topics, not `-a`). `wb_control_debug` is still 131 values;
   [120] is now the CONTACT flag, [121], [125] and [126] are NaN (retired grip fields), [122] the heading companion.
   The planner logs each grasp event on `/rosout`; nothing new to record.
5. Session plan (the user decides; never the turn first): STATIC HOLD -- Go To Start (jaws close) -> Ready (automatic:
   jaws open, descend) -> the planner settles the claw and holds it in the room -> the gate closes the jaws -> the
   status strip reads "Static hold: the jaws hold the handle (contact phase on) -- Exit To Turn in 10 s" -> release,
   open, Exit (automatic) -> the jaws close. A planner guard trip reads `ABORTING (SAFETY GUARD: ...)`; the tab opens the
   jaws, and closes them once the abort is done.
6. THE WAY BACK to the 10-27 box push: check out `box-pushing-20261027` in fsc_open_manipulator, rebuild
   `utils_custom_ground_station`, and fly the Box Pushing tab -- the Orin checks out the same tag in its three repos.

Report: the HEAD commit, the rebuild tail, the Valve tab's startup line, and the session plan.
