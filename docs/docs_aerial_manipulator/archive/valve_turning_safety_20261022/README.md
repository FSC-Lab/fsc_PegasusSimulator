# Valve turning: the box push's safety features carried over, validated in Isaac (2026-10-22)

User request: apply to the valve-turning task the safety guards and improvements made for box pushing
(2026-10-19..22, the 10-21 hardware push report), then validate them in Isaac.

Rig: whole-body 4-D L1, RAW mocap, the valve scene at the measured-bearing defaults (Coulomb 0.048 N.m =
0.30 N at the fin; the user's 10-22 ground test measured ~38 mN.m running), the generated valve yaml, the arm
GS's Valve Turning tab flying the steps itself (`--gs-auto`, as on hardware). Mocap at 120 Hz, the hardware
rate. **Every valve flight before 2026-10-21 flew 250 Hz**: the emulator ignored `frequency:=` until that
day's launch fix. 46 headless flights, 2026-10-22, all in `runs/` (npz, driver log, as-flown yaml, args).

Tools (this folder): `tools/valve_guard_replay.py` (the box push's guards replayed on 118 recorded valve
runs), `tools/chain*.sh` (the flight chains), `tools/score.py` (one row per run + Isaac's truth after an
abort), `tools/grasp_offset.py` (where the jaws closed on the fin, how far the free rotor had turned),
`tools/turn_compare.py` / `tools/tilt_spectrum.py` (the turn bin by bin / the ~0.9 Hz ringing),
`tools/gs_valve_auto_check.sh` (the arm GS, offscreen). Outputs: `analysis/` (`score_all.txt`,
`grasp_all.txt`, `tilt_ringing.txt`, `replay_*.txt`).

## 1. Shipped (the valve yaml, its hardware twin, the stacks, the planner, the arm GS)

| item | valve setting | evidence |
|---|---|---|
| raw-mocap guard: estimator `mocap_guard:=true`, planner `push_pull_mocap_guard_topic`, node `system_mocap_guard_topic` | on for `valve_turning` (isaacsim + hardware raw stack) | detects a frozen drone pose in 0.15-0.25 s, every time |
| box sample gate on obj_0 (the rotor) | 20 mm / 10 deg + growth over the gap | s22_jump_rotor: the one 180 mm / 110 deg sample dropped, good again 8 ms later, turn complete |
| box tilt guard | 2 deg / 0.2 s | the rig lifting or its clamp giving (a turn is a yaw) -- 0 false trips |
| box stale / frozen-feed guard | 0.5 s | s22_mf_rotor: abort 0.54 s into a 2 s rotor freeze, clean |
| q3 guard (static) | **3 deg**, not the box's 10 | completed valve turns dip q3 to ~10 deg (10 deg would false-trip 20 of 57 recorded completions). Thin: three completions today dipped to 3.9-4.1 deg |
| arm GS automatic Ready / Turn / Exit + mocap lamp | Valve Turning tab; the valve's OWN exit kept (open world-held, Exit 1 s after the OPEN report) | offscreen harness 19 checks (V1-V8); flown in every run today |
| **free-axis anchor** `wb_ee_anchor_free_axis_weight` 0.75 + `_on_world` true (the box push's 10-22 setting) | **on** (was 0) | section 3: the drift model 0 of 3 without, 3 of 3 with; nominal 2 of 2 with (shipped 2 of 4 today) |
| planner: a mocap-guard abort is never room-held (CoM anchor at once; a fault during a room-held release ends it) | code | s22_mf_drone: the room hold on EKF2 dead-reckoning flipped the vehicle; loopback [9j] |
| planner: `push_pull_abort_mocap_release_wait_s` | **0.7 s** (default -1 = the release wait) | the 2 s release at fin height let the dead-reckoning drift sweep the open claw into the fin, 2 of 2 |
| planner: a turn's Ready error (the GS's close gate) against the LIVE handle | code | the open jaws turn the free rotor 5-20 deg before the grasp under drift; the shipped config then grasped the fin's corner and flipped. s22_final_hw_2: the gate held the close 1.5 s (live error 22-32 mm) and closed at 17 mm |
| hardware raw stack (valve profile) | expects the free axis, the mocap release, unbuckle off; refuses a planner without the 10-22 fixes | pre-launch checks run on this desktop: all 54 tuned values present |

**Not carried over, with the evidence:**
- the q3 RATE guard (lookahead): the valve arm folds and unfolds fast in a NOMINAL turn -- at the box's
  10 deg / 0.2 s it false-trips 41 of 57 recorded completions; no lower limit separates failures.
- a turn-lag guard (the box-lag analog): completed turns lag up to 29 deg, a heavy-valve failure buckled
  the arm at 11 deg.
- the abort UNBUCKLE (measured hold + q2/q3 back to the push pose): it crashed the commonest abort state
  (section 4); the valve keeps its planned-hold abort.
- table-from-box and the Go To Start bypass: refused with a turn (the valve has its own keep-outs).

## 2. Defects found while validating

1. **A mocap-guard abort held the claw in the room on EKF2 dead-reckoning** (s22_mf_drone). The valve's
   abort keeps the claw world-held for its 2 s release; after a drone-mocap freeze "the room" is EKF2's
   estimate, 20 / 51 / 79 / 111 mm off at 1 / 1.5 / 2 / 2.5 s. The arm absorbed the phantom drift, q3
   went through 0, the vehicle left DIRECT, fell off the table, ended inverted. Fixed in the planner.
2. **CoM-anchored, the 2 s release still sweeps the open claw into the fin** (s22_um_mfd, s22_mfd_fix_1):
   the law holds the drifting estimate still, so the REAL vehicle moves by the drift (here 50-90 mm/s)
   and the open claw with it -- the rotor was yanked 100-120 deg at 1.5-2 s, DIRECT lost, both ended on
   the floor upright. Cut to 0.7 s, then straight up.
3. **The free handle turns under the open jaws.** Under the drift model the base wanders during the Ready
   descent and the open jaws turned the rotor 5-20 deg before the grasp in 6 of 9 runs; the jaws then
   closed 35-60 mm along the fin (half-length 20 mm). Shipped: the fin's corner, then a flip (s22_hw_2/3).
   Fixed: the live-handle close gate.
4. Harness: the scorer labelled feed / mocap / q3 aborts "watchdog" and read the drifting EKF2 odometry
   after a mocap fault ("moved 17 m, z -4.9 m"); `score.py` names them and prints the truth. And a
   self-inflicted fault: editing `try_valve.sh` IN PLACE while a flight used it made bash re-run that
   flight under the same tag (s22_nom_2: attempt 1 aborted, its npz overwritten; recorded in the args).

## 3. Results

Outcome after an abort = Isaac's truth over 10 s. "Clean" = DIRECT kept, vehicle level.

**Nominal (no drift model).** Shipped + guards: 2 of 4 complete (s22_nom_1 tilt-guard abort 33 s in,
s22_nom_2 attempt 1 tilt-guard abort at the turn's end; both clean). The split is the ~0.9 Hz ringing --
true-tilt ringing 2.4-2.6 deg in the aborted runs, 1.0-1.6 deg in the completed ones -- the same split the
10-18 video takes showed at 250 Hz, so 120 Hz is not shown to be the cause (10-18 final config: 6 of 9).
**Final config (free axis): 3 of 3 complete** (s22_fa0_nom_1 with the free axis latched at the grasp,
s22_final_nom_1/2), arm on a stop 0-4 % of the turn (shipped 5-12 %), peak tilt 4.5-6.3 deg, the beam
+351..355 deg (the free axis lets the valve angle follow the base: a few degrees short).

**The hardware-like drift model** (`PEGASUS_PLANT_FORCE_NOISE` 0.32 N @ 0.12 Hz, the box push's 10-22 fit
of the real AM's attitude-estimate wander):

| config | seed 1 | seed 2 | seed 3 | seed 4 |
|---|---|---|---|---|
| shipped | q3 guard 8 s in, DIRECT lost | fin corner grasped, flipped | fin corner grasped, DIRECT lost | -- |
| free axis, on_world (old planner) | complete | rotor turned 17 deg: no close, safe | complete | -- |
| free axis, grasp only (old planner) | complete | q3 guard at the turn's end, clean | rotor turned 20 deg: no close, safe | -- |
| **FINAL** (free axis on_world + the planner fixes) | -- | **complete** (gate waited 1.5 s) | **complete** | **complete** |

FINAL under drift: arm never on a stop, peak tilt 4.3-7.3 deg, beam +364..369 deg.

**Faults and stress:**

| run | injected | outcome |
|---|---|---|
| s22_mf_rotor | rotor mocap frozen 2 s | stale-feed guard 0.54 s, clean |
| s22_jump_rotor | one rotor sample 180 mm / 110 deg off | dropped by the gate, turn complete |
| s22_mf_drone (shipped) | drone mocap frozen 3 s | guard 0.21 s; then defect 1: inverted |
| s22_um_mfd, s22_mfd_fix_1 (CoM release 2 s) | the same | defect 2: DIRECT lost, on the floor |
| **s22_final_mfd** (FINAL) | the same, 12 s in | **clean**: up 0.33 m on EKF2, 5.9 deg |
| **s22_final_mfd_2** (FINAL) | the same, 12 s in | release + climb clean; 0.5 s after the mocap returned (EKF2 had drifted 0.87 m) the 15 deg watchdog tripped -- SAFETY held it level at 1.2 m |
| **s22_final_mfd_late** (FINAL) | the same, 30 s in (the swing-back) | crashed: ~30 mm up the open claw caught the fin's top edge, the wrist rolled 3 -> 46 deg, inverted |
| s22_heavy, s22_um_hv_1/2, s22_final_gov_hv | damping 2.0 N.m.s/rad (~9x the measured torque at peak rate) | 0 of 4 survived the buckle (two flips, two DIRECT lost); the governor did not save it on the free-axis config |

**Abort recipe A/B** (measured hold + unbuckle vs the shipped planned hold): injected at 20 s (arm on
both stops) -- the unbuckle dragged the open claw through the fin, inverted (planned hold: clean, 10-18);
at 33 s clean (planned hold: clean); one slowly buckled arm (q3 -16 deg) recovered; heavy valve 0 of 2
(planned hold 0 of 1). Kept: the planned hold.

## 4. What this does NOT cover -- read before the hardware flight

- **A drone-mocap loss while clamped is still not reliably survivable**: 1 clean, 1 safe, 1 crash on the
  final config (was 0 of 3). In this sim EKF2 dead-reckoning in contact drifts 20-300 mm/s and the failure
  develops in 1-3 s -- faster than a pilot. Mocap health is a precondition: the 0924 flight found a Motive
  stall sector at headings 135-188 deg, and **every valve turn sweeps all headings** -- fly a hover yaw
  sweep at the valve with raw VRPN recorded before the first turn.
- **A valve that resists much more than the measured one is not safe to turn**: the heavy valve buckled
  the arm in 4 of 4 whatever the abort. The bearing was measured (~38 mN.m running); re-measure if the rig
  changes. No guard separates "heavy" from nominal early enough (contact force and lag overlap).
- **The q3 guard's margin is thin**: three completed turns dipped q3 to 3.9-4.1 deg against the 3 deg
  limit; expect an occasional abort of a turn that would have completed (an abort is clean in that state).
- The drift model is the box push's fit of ONE hardware flight; seeds 1-4 only.

## 5. Hardware flight

Orin: rebuild `fsc_trajectory_planner` (the mocap-abort fixes + the live-handle gate) and
`fsc_drone_state_estimator_ros2` (`dev_robotic_arm`, the raw-mocap guard); pull `fsc_autopilot_ros2`
(yamls + stack scripts). Laptop: rebuild `utils_custom_ground_station` (the Valve Turning tab's
automation + mocap lamp). MEASURE `push_pull_table` / `push_pull_obstacle` into
`archive/valve_turning_20261015/tools/make_hw_valve_turning_yaml.py` and regenerate. Then:
`AM_HW_PROFILE=valve_turning ./start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh`
(the RAW stack -- the fused one refuses the valve). The pre-launch check must print
"OK all 54 tuned values present" and no "planner predates" line.
