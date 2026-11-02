# Deployment prompt -- the EXPERIMENT LAPTOP (ground-station computer)

Paste this into the Claude Code session on the laptop that runs the OptiTrack processor, the drone ground station, the
arm ground station and the bag recorder, AFTER the Orin has been deployed (`deploy_orin_prompt.md` beside this file).
The `valve-isolation-20261102` branches are pushed (2026-11-02).

---

You are on the aerial manipulator's experiment laptop. Bring the arm ground station up to the 2026-11-02 valve-turning
work on the branch `valve-isolation-20261102`. The records are in fsc_PegasusSimulator
`docs/docs_aerial_manipulator/archive/valve_cup_abort_20261102/README.md` (today) and
`docs/docs_aerial_manipulator/archive/valve_box_isolation_20261102/README.md` (the isolation from the box push).

What changed on the ground side (the arm GS, `utils_custom_ground_station`, VALVE TURNING tab only; the Box Pushing tab
is the 2026-10-27 one):
1. THE HALF CLOSE. On the fin the tab no longer closes the jaws fully: it sends the gripper to
   `push_pull_valve_cup_position` (hardware default 0.0 m, about 5-6 mm of clearance on each side of the 5 mm fin) --
   a U that pushes the handle, never a clamp on a handle bolted to the table. A close that reaches that position is the
   grasp; one that stalls no wider than `push_pull_valve_cup_max_open` (default cup + 0.004 m) is a pad resting on the
   fin (still accepted); wider is a wrong grip (nothing moves). `push_pull_valve_cup:=false` gives the old full close.
2. THE ABORT. The button, or any planner guard (the tab sees ABORTING), opens the jaws FULLY at once -- on the valve
   tab now also far from the handle. The planner climbs the moment the jaws are open: it reads the gripper's joint
   state itself on the Orin, and the tab also reports the OPEN goal's success on the new
   `whole_body_planner/push_pull/abort_jaws_open` (whichever comes first; at most 1 s without either). The first
   60 mm of the climb take 0.8 s; the claw follows the vehicle from the first tick of the abort.
3. Earlier today: the automatic exit after the turn / static hold waits for a calm base AND the law's CoM error under
   20 mm (`push_pull_valve_exit_max_base_err`), then OPENS with the claw still held in the room and starts the exit on
   the OPEN report (`push_pull_valve_exit_wait_s` 0.0).

Steps:
1. Pull fsc_open_manipulator on `valve-isolation-20261102`, report its HEAD, and REBUILD `utils_custom_ground_station`
   (`colcon build --packages-select utils_custom_ground_station`). An older station full-closes on the fin, opens on
   an abort only near the handle, and never reports the open (the planner's own gripper reading still climbs).
2. Start the arm GS (hardware) and confirm the Valve Turning tab logs
   `Valve Turning: the grasp is a HALF CLOSE -- the jaws close to 0.0000 m around the fin ...`.
   If the Orin's bench check (its step 5) found a different contact-free position, start the station with
   `-p push_pull_valve_cup_position:=<that value>` (and `-p push_pull_valve_cup_max_open:=<value + 0.004>`).
3. Bench, props OFF, with the Orin stack up in SAFETY on `AM_HW_PROFILE=valve_turning`: close the gripper from the
   Control tab. The Valve tab's ABORT button is disabled outside DIRECT, so call the tab's own abort service instead
   (`ros2 service call /uav_0/fsc_open_manipulator/valve_turning_panel/abort std_srvs/srv/Trigger`): the jaws must
   OPEN, and the GS terminal must print `Valve Turning ABORT: gripper OPEN -- reported to the planner: no abort is
   waiting for the jaws` (the planner itself refuses the abort outside DIRECT; the report path is what this checks).
   Isaac reference (2026-11-02): in every abort from a sound arm the jaws read open ~0.4 s after ABORTING, the climb
   started then, the claw was 56 mm up 1 s after ABORTING, DIRECT kept. The half-close turn completed 4 of 5 (the full
   close 3 of 3); the static hold with the half close was clean -- the record's section 2.2.
4. Recorder: keep the 2026-11-01 topic list (explicit topics). Add nothing for today; the planner logs the abort's
   timeline on `/rosout` ("the jaws are open X s into the abort -- climbing now").
5. Session plan (the user decides; never the turn first): STATIC HOLD with the half close -- Go To Start (jaws close) ->
   Ready (automatic: jaws open, descend) -> the planner settles the claw and holds it in the room -> the gate sends the
   HALF CLOSE -> the status strip reads "Static hold: the jaws hold the handle ..." -> calm wait, open, Exit
   (automatic) -> the jaws close. Watch that the fin stays FREE between the pads (it must slide out, not be pinched).
   Then, if the hold is clean, the turn.
6. THE WAY BACK to the 10-27 box push: check out `box-pushing-20261027` in fsc_open_manipulator, rebuild
   `utils_custom_ground_station`, and fly the Box Pushing tab -- the Orin checks out the same branch in its repos.

Report: the HEAD commit, the rebuild tail, the Valve tab's half-close startup line, and step 3's abort report line.
