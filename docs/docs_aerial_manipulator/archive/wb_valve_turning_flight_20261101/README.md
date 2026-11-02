# 2026-11-01 valve-turning flights (four static-hold attempts, one crash) and the state-machine review

User request (2026-11-02): check the valve-turning state machine / phase logic against the box-pushing one
(the 10-27 flight that worked), fix what differs, then analyse the 11-01 bags and say whether the crash came
from the sequencing. Report artifact: "1101 Valve Turning Review".

Bags: `docs/experimental_data_ros2_bag/1101 - T650-AM Valve-Turning-20261102T013324Z-1-001/1101 - T650-AM
Valve-Turning/flight_wb_l1_4d_valve_turning_hold_20261101_{173835,174531,175259,175932}`. Extraction:
`tools/run_extract.sh` -> `npz/b<tag>.npz` (the 10-29 extractor), timelines `npz/timeline_<tag>.txt` (the
10-31 `timeline.py` with `VT_NPZ` pointed here), the crash trace `npz/trace_175932.json` (the report's chart).

## 1. What flew

The Orin flew the 11-01 PUSHED build, not the isolation branch (which was being written during the flights and
is still unpushed): fsc_autopilot_ros2 `9e4b03e` (the 10-31 grip schedule + the 11-01 two parameter sets:
`wb_control_debug` is 131 wide, the node logs `GRIP: jaws CLOSED ... parameters -> the INTERACTION set`),
fsc_trajectory_planner `17fd566` (the 10-31 night leash / Ready CoM gate / descent guard, the fast hand-back),
and the valve yaml of that build: `push_pull_world_anchor_ready: true` (W, the claw ROOM-held from the end of Go
To Start), leash 40 mm, Ready CoM gate 30 mm / 60 mm/s, descent guard 60 mm. The arm GS opened the jaws at Go To
Start (the 10-30 behaviour). All four missions were the planned static-hold test (a 90 deg or 30 deg turn
planned, the Valve tab's "Static hold" set); none reached the hold.

## 2. The four flights

Times are bag seconds. "Pre-grasp hold" = Go To Start done -> Ready; the claw room-held, the leash cycling.

| flight | turn planned | Go To Start done | pre-grasp hold | Ready | outcome |
|---|---|---|---|---|---|
| 173835 | 90 deg | 31.2 s | 4.1 s, leash released at 49 mm, re-engaged at 78 mm peak | 35.3 (room-held descent) | **descent guard**: base 89 mm off / 138 mm/s at +3.0 s; abort clean |
| 174531 | 30 deg | 40.5 | 6.3 s, leash 44 -> 62 mm | 46.8 | **descent guard**: 66 mm / 77 mm/s; abort clean |
| 175259 | 30 deg | 29.5 | 6.9 s, leash twice (45, 49 mm) | 36.3 | **q3 rate guard** 0.6 s into the descent: q3 5.4 deg falling 57 deg/s, q1 -13 deg; abort clean |
| 175932 | 30 deg | 36.4 | 1.2 s | 37.6 -> 40.7 | jaws closed 43.05; **sway guard** 43.88 (249 mm/s); abort; tilt 25 deg at 47.0 -> SAFETY -> fell 4.4 m away, inverted |

Common numbers (`CoM err xy` = `wb_control_debug` [48..50]-[45..47], horizontal norm, mm). The arrival error
is ordinary; what follows is not:

| flight | claw before the grasp | end of Go To Start | hover peak | descent peak | on the handle, peak |
|---|---|---|---|---|---|
| box 10-27 171114 | follows the vehicle | 18 | 41 | 45 | 18 |
| box 10-27 172227 | follows the vehicle | 38 | 42 | 31 | 31 |
| valve 11-01 173835 | room-held | 16 | 78 | 89 | -- |
| valve 11-01 174531 | room-held | 5 | 62 | 67 | -- |
| valve 11-01 175259 | room-held | 43 | 49 | 84 | -- |
| valve 11-01 175932 | room-held | 25 | 26 | 33 | 110 |

The room-held base grows to 62-110 mm and the arm takes it: q1 -7..+19 deg (-23.5 in 175932), q3 down to
5-12 deg (home 25). Two box flights against four valve flights: consistent, not proof that the room hold
causes the growth. (An earlier draft quoted "52-61 mm at arrival": that was the peak over the last 3 s of
Go To Start, not the arrival instant.) In 175932 the operator closed the jaws by hand (static-hold test, automatic Push off) with the base ~100 mm off and
swinging; the room-held claw sat ~20 mm from the grasp point, so nothing on the claw showed it. The station's automatic
close gate would not have caught it either: it checks the claw, not the base.

## 3. The crash (175932), second by second

| t [s] | event |
|---|---|
| 36.40 | Go To Start done (base 25 mm off); the claw -> WORLD (`push_pull_world_anchor_ready`), free axis latched |
| 37.46-37.63 | auto Ready (1.0 s): gripper already open, descent T 3 s, ROOM-held |
| 40.69 | Ready done: "the contact phase waits for the jaws" |
| 40.7-43.0 | base swings along world x: CoM error -5 -> -107 mm, q1 -4.7 -> -23.5 deg (the arm absorbing); the OPERATOR closes the gripper by hand from the Control tab at 42.58 (automatic Push off for the static-hold test; the station logged no automatic-Push line), fingers closed by 43.02 |
| 43.05 | node `GRIP: jaws CLOSED` -> the INTERACTION gain set blends in over 1 s (K_y 212 -> 120, omega_c_t 2.93 -> 2.0, heading 2.5 -> 0.25) -- a gain switch read off the gripper's joint, mid-swing |
| 43.0-43.9 | the base swings back: -107 -> +44 mm in 0.8 s (~190 mm/s); the clamped claw turns the valve 8 deg; q3 4 -> 21 deg |
| 43.87 | **sway guard** 249 mm/s (CoM error 32 mm): ABORT -- contact off, FAST hand-back to the CoM anchor; `abort_from_measured false` -> the hold is the PLANNED pose, a rest (zero velocity); the base is 32-44 mm from it and moving at ~350 mm/s |
| 43.9-45.9 | 2.0 s release wait at fin height: the jaws open by 44.2; the base's momentum carries it on to +149 mm, then the position loop swings it back to -49 mm (0.5 Hz, roll +-7 deg). The claw follows the base (CoM anchor): the opening jaws sweep +151 / -40 mm sideways and +-50 mm along the nose at fin height and knock the fin: valve yaw -171 -> -161 -> +178 deg, |F_hat| spikes 12-16 N |
| 45.9 | the abort's climb starts (0.30 m in 3.95 s), base still swinging |
| 46.3-46.9 | the base swings +x again (CoM error +54 -> +214 mm) while the claw stops at +38..+55 mm from the grasp point: caught on the fin. The rotor obj_0 z 0.870 -> 0.875 m (the rotor lifted), q2 12 -> -19 deg, F_hat x -21..-35 N, d_hat_r,y -0.1 -> -0.9 N.m; motors m1 = m2 = 0.00, m3 = 1.00 (allocator saturated, -10 N unallocated); roll -14, pitch -15 deg |
| 46.90 | node watchdog 15.1 deg -> "emergency abort (already running)" |
| 47.02 | 25.6 deg > hard limit -> SAFETY; arm RELEASE; fold home scheduled |
| 47.1-48.5 | tilt 45 -> 110 deg, the vehicle travelled from (0.59, 0.03) to (4.7, 0.8) m and came to rest inverted at z 0.15 (mocap jump gates, the pilot's STAB / MANUAL at 48.2-51.2) |

So the chain is: pre-grasp ROOM HOLD -> the base error grows (section 2's table) and the arm takes it -> the
jaws closed by hand mid-swing (the room-held claw looked settled) -> the gripper-position
gain switch -> the sway guard fires with the base moving at ~350 mm/s -> the abort starts from a STILL
reference (every abort leg is rest-to-rest), the base overshoots by its own momentum and rings at 0.5 Hz, and
the claw, handed to the CoM anchor, follows it: the opening jaws sweep across the fin for 2 s, then catch it as
the climb starts. (An earlier draft said the abort held a pose "150 mm away". Wrong: at the abort the base was
32-44 mm from the hold; the 149 mm was its overshoot afterwards.)

## 4. Why the box push does not do this

The 10-27 box push: claw CoM-anchored through Go To Start, the Ready descent and the arrival on the handle;
the room hold starts only when the claw is within 25 mm (15 across) of the grasp point and slower than
30 mm/s for 0.3 s (the hand-over settle), i.e. 0.4-1.2 s before the jaws close; contact ON at the Push press
(a planner event, sent by the station on the grasp result); release-first exit. The valve flown on 10-28,
10-29, 10-31 and 11-01 held the claw in the room from the end of Go To Start (W, chosen 10-27 for a sim
drift-seed result) and, from 10-31, read the gripper joint for its gain switch. Every hardware valve failure
since 10-29 is in that pre-grasp / grasp window or in an abort from it; the 10-28 grasps succeeded on the same
W because that build had no leash, no gain sets and a quiet base that day -- the turns then failed on the
in-contact sway, a different problem.

## 5. What is fixed, where

| problem | fix | where |
|---|---|---|
| room hold before the grasp (W) | `push_pull_world_anchor_ready: false` + the box's hand-over settle (25 / 15 mm, 30 mm/s, 0.3 s) | isolation branch, valve generator (decision P, 11-02) |
| gain switch / contact read off the gripper joint | the grip schedule and the two parameter sets DELETED from the node; one gain set; contact only on planner events (Turn press, `push_pull/grasp` for the static hold) | isolation branch (11-02) |
| jaws open from Go To Start | jaws CLOSED in free flight, opened by Ready, closed by the gate after the hand-over, closed again after the Exit | isolation branch, Valve tab (11-02) |
| leash / Ready CoM gate / descent guard cycling the anchor | all off for the valve (they guarded the room-held descent that no longer exists) | isolation branch |
| abort from a swinging base: the claw follows the base across the fin | **today (NOT a fix for this)**: `push_pull_abort_from_measured true`, `push_pull_abort_release_wait_s 1.0` (sim) / 1.5 (hardware, the box's), `push_pull_abort_release_world_held false` = the box's recipe; unbuckle stays off. Removes only the 32-44 mm offset at the abort instant; the base still overshoots and the claw still sweeps. Reverses the 10-18 Isaac result against from_measured on the valve (s_abort20/33, with the world-held release). Untested | `make_valve_turning_yaml.py`, `make_hw_valve_turning_yaml.py` (HW_PLANNER), both valve yamls regenerated (`--check` UP TO DATE); the raw hardware stack's expected values |
| exit: 1 s extra at fin height after the OPEN report | **today**: `push_pull_valve_exit_wait_s` default 1.0 -> 0.0 (exit on the OPEN report, the box's `push_pull_release_wait_s` 0.0) | `push_pull_panel.{cpp,hpp}`, station rebuilt |

Checks after today's edits: `gs_valve_phase_check.sh` 25/25 (the real Valve + Box tabs); the raw hardware
stack's pre-launch part on `valve_turning` (87 tuned values, the new abort values OK) and `box_pushing`
(62, unchanged). No planner or node code changed today; the planner loopbacks of the isolation record stand.

NOT fixed, and not a sequencing problem: the in-contact lateral sway on a free handle (10-28: 0.56-0.6 Hz
growing in 3 of 3 contacts; 10-29: to the arm's limits in 7 s). The static hold is the measurement that
separates it from the approach; the roll-fin handle (10-30/31) addresses the grip's roll coupling.

Nothing is deployed: the Orin still runs `9e4b03e` / `17fd566`. The isolation branch (five repos) plus today's
edits must be pushed and rebuilt per `valve_box_isolation_20261102/deploy_orin_prompt.md`.

## 6. Isaac caveat on the box approach

The isolation record (section 5/7): with P the Isaac turn did 2 of 3 nominal and 0 of 1 under drift (an open
finger met the fin during the CoM-anchored descent), against 13 of 13 unforced turns with W on the 10-31/11-01
builds. Isaac has never shown the hardware's pre-grasp sway, so it cannot weigh the two sides; on hardware W
failed at or before the grasp on three days. The static hold first, as planned.

## 7. The logic comparison page (2026-11-02)

`logic_compare.html` (artifact "Valve-turning vs Box-pushing logic"): the 10-27 box push A (171114, tag
`box-pushing-20261027-flown`) and this crash flight (175932) replayed on one clock from Go To Start done, with the
arm station's buttons, the anchor state, jaws, contact and the base CoM error. Built by `logic_compare/build_trace.py`
then `logic_compare/build_page.py`. Box: room hold 0.46 s before the jaws closed (settle 0.40 s after Ready,
station gate met at once, base 16 mm off); valve: 6.62 s, from the end of Go To Start, jaws closed by hand, base
106 mm off, q1 -21.5 deg.

## 8. The abort that crashed, step by step, and its reference in 3-D (2026-11-02)

User request: what happened in the abort that caused the crash, recorded on the logic page with the abort's reference
trajectory drawn in 3-D. Page section "Archive · The abort that crashed, 11-01" (artifact "Valve-turning vs Box-pushing
logic", version 4). Tools: `tools/abort_trace.py` (table every 0.1 s in the valve's frame -> `npz/abort_175932.json`),
`tools/abort3d_data.py` (-> `logic_compare/abort3d.json`: the abort PLAN the planner published on `viz_path` at 43.91 s,
120 samples [x_cd, nose, r_ed, claw axis], and the flown CoM / claw / claw reference / anchor weight / skeleton / jaws /
valve angle, 43.40-47.04 s). `logic_compare/build_page.py` now splices `logic_compare/abort_section.html` +
`abort3d.json` in before `</main>` (or at an `<!--ABORT_SECTION-->` marker), so `page_template.html` is untouched.

The plan was settle 2.0 s at the PLANNED Ready hold, climb 0.30 m in 3.95 s (min-snap), arm home 3.82 s. What made it a
crash, measured:
- **The claw reference followed the base.** The sway guard drops the room hold (fast hand-back, CoM-anchor weight
  `wb_control_debug[129]` 0 -> 1 by ~44.13 s), and from then the law's claw reference equals `r_ed + (CoM - x_cd)` to a
  few mm (checked at five instants). The claw tracked it: 143 mm inward of the fin (almost over the hub) at 44.45 s,
  50 mm outward at 45.3 s, all at fin height.
- **The hold was a rest point for a base already at ~0.4 m/s** (32 mm off at the trip): CoM error peaked at 153 mm
  at 44.45 s, swung back to 49 mm at 45.35 s (~0.5 Hz), and went 23 -> 216 mm during 46.0-47.0 s.
- **Corrections to section 3, from re-reading the data:** the valve did NOT turn during the overshoot (it held
  ~8 deg, 43.9-45.4 s). By the arm's kinematics the claw was already 23-52 mm inward of the fin at the trip with the
  jaws still closed: the jaws had closed on nothing beside the fin's end (see the jaw paragraph below) and the
  closed claw had pushed the fin. The fin was knocked 8 -> 19 deg at 45.4-45.7 s, when the swinging claw came back at the fin's end 13-17 mm
  below its top. From ~45.9 s (the climb's start) the claw stayed at the fin and DRAGGED it: the valve turned back
  19 -> -3 deg by 46.9 s (-8 by 47.0), the rotor rose 3.4 mm, while the claw reference ran 138 mm away with the
  base; two motors at 0 and one at 1 from 46.40 s; tilt 6 -> 15 deg.
- At SAFETY (47.02 s) the climb's reference was 28 mm up; 60 mm (the exit's finger-clear height) was due at 47.31 s.

Not changed: any controller or planner value. Planner source being edited by another session on 11-02 (uncommitted,
`push_pull_abort_exit_hold`) keeps the claw room-held only after the turn, after a static hold's release and in the
exit's climb; a sway or arm guard while the jaws are closed on the handle still hands the claw to the base in 0.25 s.

Gripper added to the 3-D view (page version 5, same day, user request "see if it was open or closed"): two pads in the
planner's claw frame (`current_ee` orientation: x = claw, y = the jaws' closing axis, verified across the fin at the
grasp), the real claw's geometry from `open_manipulator_x_custom.urdf.xacro` (rails +/-37.1 mm, 49.3 mm up the bracket,
fingertips 16 mm past the grasp point, stroke -9.7..+13.2 mm = 0..46 mm between the pads), the gap scaled between this
flight's own closed (-9.9 mm) and open (+19.0 mm) joint readings; a "Gripper close-up" view follows the claw. Jaw state
over the abort: closed 43.05 s ON NOTHING beside the fin's end (claw 19 mm along the 40 mm fin, 12 mm outward of it;
closed reading -9.88 mm / effort 36 vs the fin grasps of 10-28 18:06 + 18:18 and 10-29 14:55 at -9.55..-9.69 / 99-150 and
the 10-31 17:57 close on nothing at -9.91 / 26); the closed claw pushed the fin (valve 0 -> 8.7 deg), slid past its end
(21 -> 30 mm along it) before the trip; open goal 43.903 s (0.03 s after the trip), moving 43.93, fully open ~44.5;
OPEN for the sweep, the knock and the hook. The close goal (gripper_controller, 42.577 s) came from the Gripper box:
automatic Push was off for the static-hold test.
