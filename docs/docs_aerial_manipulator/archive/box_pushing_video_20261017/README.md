# Presentation videos: whole-body circle, figure-8 and box pushing (2026-10-17)

User request: full-screen videos for a presentation, Isaac on the left, a ground station
on the right — the drone GS through take-off and the switch to DIRECT (its Flight Log tab
and Controller tab showing), then the arm GS on the task's tab until the task ends (box
pushing: until the Land step has finished and the vehicle hovers). No idle phases, high
resolution. Layout confirmed by the user from two screenshots, with one change: the arm
GS's tab page (Mission + 3-D view) takes the height and the log keeps a few lines.

Outputs (`~/Videos/`), 2560 x 1440, 25 fps, H.264 (x264, constant quality 17):

| video | length | take | mission |
|---|---|---|---|
| `Aerial Manipulator - Isaac Sim - Whole-Body Circle.mp4` | 58.2 s | `takes/circle_2` | r 0.5 m, 24 s lap, s = 1.0 (T 28 s), 06 scene |
| `Aerial Manipulator - Isaac Sim - Whole-Body Figure-8.mp4` | 54.5 s | `takes/figure8_1` | 0.70 x 0.35 m, 24 s lap, s = 1.0 (T 28 s), 06 scene |
| `Aerial Manipulator - Isaac Sim - Whole-Body Box Pushing.mp4` | 102.7 s | `takes/box_pushing_1` | the 10-16 final setup + the 10-17 release fix, 07 scene; box pushed 499 mm |
| `Aerial Manipulator - Isaac Sim - Whole-Body Valve Turning.mp4` | 122.3 s | `takes/valve_rep4` | 2026-10-18, the final valve config; beam +361.1 deg; Isaac view RENDERED FROM A HEADLESS FLIGHT'S LOG (valve section) |
| `Aerial Manipulator - Isaac Sim - Whole-Body Pick and Drop.mp4` | 115.5 s | `takes/pnd_rep1_tray` | 2026-10-18, the pick-and-drop with the latest payload (Box_Assembly, 190 g) into the tray; replayed like the valve; re-rendered with the opaque tray floor (pick-and-drop sections) |
| `Aerial Manipulator - Isaac Sim - Whole-Body Pick and Place.mp4` | 124.6 s | `takes/pnpa_rep1` | 2026-10-18, the SAME payload set down on the place pillar's hat, no tray (last section) |

Each video starts 1.5 s before lift-off (rotors already spinning) and switches from the
drone GS to the arm GS 3 s after the DIRECT switch. Every simulation ran at real time
(RTF 1.000) except about 20 s around the end of the box push, the release and the exit
(10 s windows 0.80, 0.90); during the 25 s push Isaac repeated about one frame in five
(the contact physics is heavier there). At the push's 2 cm/s that is under a millimetre
of motion per repeat.

## Pipeline (tools/)

```bash
# per video: clean slate (TWO separate calls), then
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y
bash tools/setup.sh tools/view_circle.json          # or view_figure8.json / view_box_pushing.json
/usr/bin/python3 tools/shoot.py circle_3 -- $PWD/tools/ee_video_driver.py --shape circle --time-scale 1.0 --direct-settle 6 --post-hold 3
/usr/bin/python3 tools/shoot.py box_3 -- $PWD/tools/pl_video_driver.py --release-at-open --direct-settle 6 \
    --hover-dwell 1.0 --push-hold 1.0 --leg-pause 1.0 --post-hold 3 --out $PWD/takes/box_3/mission.npz
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/compose.py circle_3 out/circle_3.mp4            # box: --start CLIMB+0.5
```

- `setup.sh` — the 4-D whole-body stack (raw mocap; profile mirror or box_pushing) and the
  scene through the 4-D launcher with `video_scene.py`; replaces the stack's drone GS with
  `drone_gs_video.py`; starts the arm GS at `QT_SCALE_FACTOR` 0.8 (via `tmux setenv -g`);
  lays the windows out; `split.py` drags the arm GS's page/log splitter down.
- `video_scene.py` — 06 or 07 unchanged, loaded as a module, with Kit window settings as
  extra argv (viewport fills an undecorated 1244 x 1400 window, rendered at that size) and
  the camera from `video_view.json`, re-applied every 0.5 s (edit it live to frame a shot).
- `drone_gs_video.py` — the drone GS (unmodified repo) with its widgets moved for a tall
  pane, Flight Log + Controller tabs selected, the three lab-network lamps hidden.
- `shoot.py` — one capture per window (Isaac all through; drone GS until DIRECT + 5 s; arm
  GS from DIRECT), x264 intermediates, wall-clock start times in `timeline.json`.
- `compose.py` — the cut: 2560 x 1440, left = Isaac, right = drone GS / arm GS.
- `ee_video_driver.py` — the planner's EE-trajectory sim driver with the fixed waits cut,
  an absolute time scale and JSON events; ends hovering after the run (no landing).
- `pl_video_driver.py` — `push_pull_20261003/tools/pl_mission.py` unchanged + JSON events.
- `xwin.py` — window list / place / raise / click / drag over libX11 + libXtst (no wmctrl,
  xdotool or python-xlib on this machine). `frame.py`, `grab.sh`, `search_cam.py`,
  `frame_cam.py` — still composition, window grabs and camera framing.

## Findings worth keeping

- **Recording costs Isaac.** The GPU was only ~25 % busy, but each window read-back
  (XGetImage) stalls Isaac's frame: three captures at 30 fps held the scene at RTF 0.95.
  Drawing every 10 physics steps (25 Hz) with a 25 fps video, and at most Isaac + one GS
  captured at a time, held RTF 1.000 (0.46 s of slack per 10 s).
- Covered windows capture correctly (composited), so both ground stations can sit in the
  same place on screen.
- XTest clicks and drags work; the arm GS splitter is a ~5 px handle found by its grip
  dots (row 823 in the 1244 x 1400 window at scale 0.8). The station re-splits only on a
  window resize or a tab change, so the drag holds for a whole take.
- The PyQt drone GS ignores SIGTERM; stop it with SIGKILL.

`takes/` holds the raw per-window captures (~430 MB) for re-cutting; not for git.

## Valve turning (2026-10-18) -- NOT recorded: the windowed simulation cannot fly the turn

`setup.sh tools/view_valve_turning.json` (video_scene.py gained the `valve_turning` scene; arm GS on
tab 7) and the camera in `view_valve_turning.json` (pos [-1.55, 1.35, 2.25], target [0.70, -0.05,
0.80]: the vehicle whole at take-off, the valve near the centre) work. **Six takes (valve_1..6), 0 of 6
completed the turn** -- 3 with the box video's short waits, 2 with the campaign's (15 s settle), all
leaving DIRECT 6-24 s into the turn; the same configuration headless completed 6 of 7. Measured from
the rotor mocap's wall-clock rate (published every 4th physics step, 62.5 Hz at real time): windowed
RTF 1.00 in the hover but **0.83-0.84 during the turn** (the valve's contact physics + rendering +
captures); headless 1.00 throughout. The law runs on the wall clock, so a 17 % slow plant is the
Isaac wall-clock artefact (memory: isaac-wallclock-l1-artifact), and the valve's contact phase is
marginal enough to lose to it (the box push survived its 0.80-0.90 dip). Options, none built: a
KINEMATIC REPLAY of a recorded headless run in a windowed scene (no control loop, so RTF does not
matter; the ground-station windows would need a topic replay too), or a lower render rate
(render_every 20-25: a choppier viewport, RTF gain unmeasured). `takes/valve_1..6/` hold the raw
captures of the failed takes (not for git).

## Valve turning (2026-10-18 evening) -- RECORDED as a REPLAY

`~/Videos/Aerial Manipulator - Isaac Sim - Whole-Body Valve Turning.mp4` -- 122.3 s, 2560 x 1440, 25 fps,
take `takes/valve_rep4` (the final valve configuration of `valve_turning_20261015/README.md` section 15.10:
beam turned +361.1 deg, mission complete). Same layout and cut rules as the three videos above; two idle
hovers cut out (`--skip`): 9.1 s after the DIRECT switch and 13.0 s waiting for the landing slot.

**Why a replay.** In contact the valve scene's physics already overruns its 4 ms step (RTF ~0.98 headless),
so any frame drawn costs the plant real time, and the wall-clock controller loses the turn to it (the six
takes above). With the adaptive-render gate (06, `PEGASUS_RENDER_ADAPTIVE`) the plant keeps up but the
viewport drops to 1-4 fps in the turn. So the flight runs HEADLESS -- the plant exactly as in the results
campaign -- and the Isaac view is rendered afterwards from a log:

```bash
# per take: clean slate (TWO separate calls), then the recording flights until one completes
bash tools/record_takes.sh tools/view_valve_turning.json 4 3 -- --timetable --slot-release 39.0 --slot-land-step 40.0 --push-hold 1.0
#   = setup.sh with REPLAY_TAKE=valve_rep<k> + shoot.py with SHOOT_WINDOWS=drone,arm SHOOT_TOPICS=1
bash tools/replay.sh valve_rep4 --view $PWD/tools/view_valve_turning_replay.json     # Isaac view -> takes/valve_rep4/isaac.mp4
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/compose.py valve_rep4 OUT.mp4 \
    --skip "DIRECT+5:go_to_start:start-1" --skip "go_to_land_start:end+2:land:start-1"
```

- `record_scene.py` -- the task's scene class unchanged plus, after its control step, every 5 physics steps
  (50 Hz) the world pose of every rigid body of the vehicle and the valve rotor (15 prims) through
  `dynamic_control`, wall- and sim-time stamped -> `poses.bin` / `poses.json`; the stage's root layer exported
  once built -> `stage.usda` (23 KB: environment, table, valve and vehicle are references in it). Read-only;
  measured: the loop stays regular (records every 20.0 ms sim, wall p99 22.8 ms), all handles valid.
- `topic_recorder.py` -- the three planner topics the overlays are drawn from (`viz_path`, `viz_pose`,
  `push_pull/info`), JSON lines, wall-time stamped (started by shoot.py with `SHOOT_TOPICS=1`).
- `replay_scene.py` (+ `replay.sh`) -- Isaac windowed, no stack, no PX4, the timeline NEVER played: the recorded
  layer composed into the app's own stage as a SUBLAYER, `/Render` stripped from a copy first (opening it as a
  new stage crashed Kit in `omni.graph.image.core` on the next update); every logged prim gets one
  `xformOp:transform` in the root layer set from the log at the replay's wall time mapped onto the flight's
  (positions linear, rotations normalised-linear); the overlays redrawn by the scenes' OWN methods (06's
  `_draw_trajectory`, the 07 scene's `_wp_*`, borrowed onto a bare object) fed the logged messages; window
  captured as shoot.py does, its start written into the take's `timeline.json` mapped onto the flight's clock,
  so compose.py cuts it unchanged. Draws ~155 fps (no physics), captured at 25.
- `view_valve_turning_replay.json` -- the camera FITTED on valve_rep4's log: the recorded direction pulled back to
  3.7 m and aimed at (0.20, 0.05, 1.05), so every vehicle prim and rotor tip stays 50 px inside the frame from the
  take-off on the ground to the landing hover close to the camera (the recorded framing cut the vehicle at both
  ends). A replay can be re-rendered with another camera at any time (`--view`).
- compose.py / replay_scene.py accept step marks by name as cut points (`push:start`, `exit:end`, ...).

**Four recording flights for one complete mission** (the result, not the tooling): valve_rep1 (driver waits
shortened: 8 s DIRECT settle) aborted on the 10 deg tilt guard 8 s into the turn, clean abort; valve_rep2 (the
driver's defaults, no timetable) aborted on the guard ~25 s in and the abort ENDED INVERTED (the arm on its
stops, the known unrecoverable case of valve README 15.2); valve_rep3 (the campaign's exact driver arguments)
aborted 2.2 s into the push step with the wrist pair already oscillating in the Ready hover (q1 / q4 std
4.4 / 7.2 deg vs <= 0.4 in every other run); valve_rep4 (same arguments) complete. Checked and NOT different
from the campaign runs that passed 3 of 3 an hour earlier: the yaml (byte-identical to the one they flew), the
binaries, the WB / joint / odometry stream timing, the arm velocity feed (fresh 100 %), the recorder's handles.
Read it as the final configuration's real success rate at the measured bearing being lower than the campaign's
3/3 suggests: today 6 of 9 (the two confirmation runs l6_ship_1/2, the campaign's 3, valve_rep4).

## Pick-and-drop (2026-10-18) -- RECORDED as a REPLAY

`~/Videos/Aerial Manipulator - Isaac Sim - Whole-Body Pick and Drop.mp4` -- 115.5 s, 2560 x 1440, 25 fps,
take `takes/pnd_rep1`. The user asked for a whole-body pick-and-place video "with the latest payload
model" and chose the pick-and-drop mission: the real payload `Box_Assembly` (basket + fin, no arch) at
**0.190 kg** picked from the 0.77 m pillar and dropped into the tray on the place pillar. Configuration = the
10-16 validated Isaac flights (j4_heading_stuck_20261016 J1/J2): `tools/pick_and_drop_video_sim.yaml` =
the shipped `..._sim_pick_and_drop.yaml` with `pick_place_drop_auto: false` (the driver times the release),
byte-identical to that campaign's `variants/J.yaml`; raw mocap; driver `pnp_mission_v2.py --rig wb
--drop-place --grip-via-action --grip-axis-tol 0.003`. **Completed at the first take**: claw 7.7 mm from the
drop point at the open, the basket at rest FLAT in the tray, 7 / 13 mm off its centre (`drop_score.py`).
One idle span cut (`--skip DIRECT+5:go_to_start:start-1`, 9.1 s: the driver's 15 s DIRECT settle).

**Why a replay here too.** Windowed, this scene held RTF 1.000 but drew only 10-16 fps even seated on the
ground (06's adaptive render gate skipping 90-150 of 250 frame slots per 10 s: ~0 s of slack per step), so a
live take would have been choppy. Headless flight + replay gives 25 fps with the plant exactly as flown.

```bash
# clean slate (TWO separate calls), then
RT_DRIVER=pnp_video_driver.py RT_PREFIX=pnd_rep WB_SIM_YAML=$PWD/tools/pick_and_drop_video_sim.yaml \
PEGASUS_PNP_PAYLOAD_MASS=0.190 bash tools/record_takes.sh $PWD/tools/view_pick_and_drop.json 1 3 -- \
    --rig wb --drop-place --grip-via-action --grip-axis-tol 0.003
bash tools/replay.sh pnd_rep1 --view $PWD/tools/view_pick_and_drop_replay.json
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/compose.py pnd_rep1 OUT.mp4 --skip "DIRECT+5:go_to_start:start-1"
```

What was added for the 07 pick-and-place scene (all additive; box / valve takes behave as before):
- `setup.sh` -- scenes `pick_and_place` (profile pick_and_place, arm GS tab 4, `PEGASUS_PNP_TASK=place`)
  and `pick_and_drop` (tab 5, `drop`); it pushes `PEGASUS_PNP_TASK` and every `PEGASUS_PNP_*` in its own
  environment into the tmux server (the pick-and-place wrapper's job, which setup.sh bypasses), after
  clearing stale ones. `WB_SIM_YAML` in its environment reaches the stack and the plant.
- `video_scene.py`, `record_scene.py`, `replay_scene.py` -- the two scene names -> `AmT650PickAndPlace`;
  record_scene also logs `PAYLOAD_PRIM` (15 prims: 14 vehicle bodies incl. both gripper fingers + the payload).
- `topic_recorder.py` -- also `pick_place/info` (`ppinfo`) and, every 2 s, the planner parameters the
  scene's waypoint markers are drawn from before Plan (`params`, logged on a change); `replay_scene.py`
  feeds both to the scene's own `_wp_*` methods (`_payload_pose` stubbed: Get happens before the video).
- `record_takes.sh` -- `RT_DRIVER` / `RT_PREFIX` (defaults = the valve's).
- `pnp_video_driver.py` -- `pnp_mission_v2.py` unchanged + the JSON events (pl_video_driver.py's pattern).
- `fit_cam_log.py` -- the replay camera fitted on a take's log: every logged prim over the cut (0.2 m spheres)
  plus fixed points (`--fixed=x,y,z`, repeat; `=` for negatives), 50 px margin, largest median vehicle.
  `view_pick_and_drop_replay.json` = its pick: from the NORTH, 38 deg up, 7 m, 49 mm -- the pick (vehicle
  west of the pick pillar) and the drop (vehicle east of the tray) both side-on, vehicle ~275 px.
  (`search_cam_pnp.py [drop]` = the a-priori search from the mission's nominal points; the log fit replaced it.)

## Pick-and-drop follow-up (2026-10-18): the tray, the release shake, pick-and-place with the latest payload

User review of the pick-and-drop video: the place pillar looked "too black", the arm shook after the drop,
and the demonstration should be a PICK-AND-PLACE with the pick-and-drop payload, set down on the pillar
(the tray is the experiment's; the simulation demonstrates the place).

**The black tray.** The black disc was the WHITE HAT seen through the tray's glass floor (OmniGlass,
thin-walled, ior 1.0) -- RTX real-time drew it black; the rim and corners of the same floor were fine. Fixed in
`07_px4_t650_aerial_manipulator_pick_and_place.py` `_build_bin`: the FLOOR is now opaque light plastic
(`BIN_FLOOR_RGB` 0.86/0.90/0.94, no glass, no opacity); the walls keep the glass and the dark edges. The
pick-and-drop take was NOT re-flown: `takes/pnd_rep1_tray` = pnd_rep1 with the same change made in its exported
`stage.usda` (pxr, the floor prim only; ground-station captures symlinked), re-rendered with the same camera.
Before/after: `preview/tray_floor_before_after.png`.

**The release shake** (measured, `mission.npz` of pnd_rep1 vs the 10-16 runs J1/J2/D1/A1 and pnpa_rep1): EVERY
release kicks the arm -- q2 +18..25 deg, q3 +10 deg within 1 s, j2 torque swinging ~3 N.m p-p -- because the
4-D L1 observer's internal-disturbance feed-forward on u3 (`d_int^f`, low-passed at `wb_l1_omega_x` 0.2072
rad/s, a ~5 s time constant) still carries the payload's weight torque when the payload is gone: on j2 it reads
0.75 N.m before the open and decays 0.75 -> 0.22 over 9 s, while the arm channel's filtered estimate decays over
~3 s. Normally the kick is gone in ~3 s (J1, D1, A1, pnpa_rep1: law j2 command peak 2.15..2.59 N.m). In pnd_rep1
the j2 command hit the 3.0 N.m clamp (`n_sat` 1) in the first second, and from there the arm RANG at ~3 Hz for
~7 s at full clamp swing until the stale feed-forward had decayed to ~0.3 N.m -- an unlucky instance of the same
transient (same yaml, same mass, same driver). Not changed: the fix is a law change (reset / re-seed the
payload's share of the observer state on the gripper-open event, or a payload model switched by the gripper),
not a gain -- `wb_l1_omega_x` >= 0.5 rings the 0.94 Hz attitude mode (2026-09-17). The pick-and-drop video keeps
the flight as flown.

**Pick-and-place with the latest payload** (`setup.sh` scene `pick_and_place_assembly`): the 07 scene with
`PEGASUS_PNP_TASK=place`, `PEGASUS_PNP_PAYLOAD=assembly`, both pillars 0.77 m with the hat, no bin, 190 g;
planner yaml `tools/pick_and_place_assembly_video_sim.yaml` = `pick_and_drop_video_sim.yaml` with
`pick_place_place_drop: false` and `pick_place_place_point` z 0.822 (the payload centre rests at 0.8105 m on
either hat: released 11.5 mm above it), `vehicle_name` "...-PNP"; the controller is the pick-and-drop's. Driver
`pnp_mission_v2.py --rig wb --grip-via-action --grip-axis-tol 0.003` (the default place: descend, open
within 2 cm, back out 0.10 m, climb). **Completed at the first take (pnpa_rep1)**: claw 10 mm from the target
at the open, the payload at rest on the PLACE hat 7.2 mm from its axis, flat (0.0 deg), yaw 179 deg, untouched by
the exit. Camera fitted on its log (`view_pick_and_place_assembly_replay.json`, the pick-and-drop's direction).

```bash
# clean slate (TWO separate calls), then
RT_DRIVER=pnp_video_driver.py RT_PREFIX=pnpa_rep bash tools/record_takes.sh $PWD/tools/view_pick_and_place_assembly.json 1 3 -- \
    --rig wb --grip-via-action --grip-axis-tol 0.003
bash tools/replay.sh pnpa_rep1 --view $PWD/tools/view_pick_and_place_assembly_replay.json
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/compose.py pnpa_rep1 OUT.mp4 --skip "DIRECT+5:go_to_start:start-1"
```
