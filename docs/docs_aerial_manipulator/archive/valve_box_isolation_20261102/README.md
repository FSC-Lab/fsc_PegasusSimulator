# 2026-11-02 — isolate valve turning from box pushing; valve phases the 10-27 box way

User request (2026-11-02, the Orin session's prompt): run the valve-turning phases the way the successful 10-27
box push did (planner events, not the gripper position); jaws CLOSED in free flight, opened and closed only inside
the interaction; the valve's changes isolated so the 10-27 box push can always be flown again ("if anything goes
wrong we can't go back to the successful box-pushing").

**Decisions (asked first, as the prompt required; the user chose every recommended option):**

| | question | decision |
|---|---|---|
| D1 | valve gains | **ONE set for the whole flight**, as the 10-27 box: the valve's contact set (K_y 120, D_y 20.2, K_ψ/D_ψ 0.2484/0.2903, ω_c_t 2.0, j4 stuck hold off) |
| D2 | box on the main branches | **EXACT 10-27**: every post-10-27 box behaviour gated off |
| G | who closes the jaws on the fin | **the Valve tab, automatically**, after the planner's hand-over; it tells the planner (a planner event) |
| P | the valve's approach | **the box's**: CoM-anchored descent, the settle hand-over, then the room hold |

Everything is on branch `valve-isolation-20261102` in the five repos, committed locally, NOT pushed.

## 1. The baseline (the user's way back)

Branch `box-pushing-20261027` in every repo, at the commits below, plus the annotated tag `box-pushing-20261027-flown` at
the same commits (the tag is not named like the branch: a tag and a branch of one name make `git push origin
box-pushing-20261027` fail as ambiguous).

| repo | branch at | what flew 10-27 17:07 / 17:21 EDT | why it is exact |
|---|---|---|---|
| fsc_autopilot_ros2 | a316625 | yamls + stack scripts a316625, C++ built at 53877d3 | `git diff 53877d3..a316625` touches no C++/CMake/xml |
| fsc_trajectory_planner | 7d8ede9 | C++ built at ceaab3b | `ceaab3b..7d8ede9` touches only test/ |
| fsc_drone_state_estimator_ros2 | 4badef8 | the flown build | — |
| fsc_open_manipulator | a0cb8b8 | arm GS (laptop) a0cb8b8; controller built at 98f44e2 | `98f44e2..a0cb8b8` touches only utils_custom_ground_station |
| fsc_PegasusSimulator | 4939d04 | the yaml generators | its make_push_pull_yaml.py + make_hw_box_pushing_yaml.py, run against a316625's config, `--check` UP TO DATE |

Checked on this desktop (2026-11-01 evening):
- The baseline builds: worktree workspace `~/baseline_box_20261027_ws` (`build.sh`: the five packages,
  `-DFSC_BUILD_COORDINATED_ADMITTANCE=OFF` as on the Orin), 99 s, rc 0.
- The baseline raw hardware stack's pre-launch part (cut before line 1275, its first `tmux` session action; no
  launch or send-keys in the cut), `AM_HW_PROFILE=box_pushing`, run against that workspace: all 62 tuned values
  present, every build probe passes (`prelaunch_box_baseline.log` there).
- Not provable from here (nor on the Orin): that the 10-27 builds had no uncommitted edits.

## 2. What had changed for the BOX since the baseline, and what was done with each

The box yamls' PLANNER sections were unchanged; the node sections had gained 21 keys. A = reached the box through a
box-yaml key, B = reached it with no key, C = inert for the box. (Mapped by reading every diff, 2026-11-02.)

| # | change (commit) | reached the box | done |
|---|---|---|---|
| 1 | grip schedule + two parameter sets (2787578, 9e4b03e): the gripper's position switched K_y/D_y, K_ψ/D_ψ, ω_c_t, the j4 stuck hold | A, 17 keys | keys removed from the box yamls; **the schedule DELETED from the node** (for both tasks) |
| 2 | emergency abort (fa8fc9f + planner 987f034) | A, 4 keys | keys removed from the box yamls; code kept, valve-only by its keys |
| 3 | fast CoM hand-back on a q3 guard abort (9e4b03e + planner 17fd566) | A | key removed; new planner key `push_pull_fast_hand_back` (default false; valve true) |
| 4 | 0.1 s all-NaN arm RELEASE at a SAFETY handover (fa8fc9f + arm d4d062f) | B, hardware | new node key `wb_arm_release_on_stop` (default false = 10-27); true in the valve and mail-delivery yamls (mail flew it 10-31) |
| 5 | j4 current-trim target includes the error damping (arm dd28bce) | B | new arm-controller LIVE param `trim_target_includes_error_damping` (default false = 10-27); the Valve tab sets it for its mission with the damping raise |
| 6 | mocap jump gate centred on EKF2's velocity (estimator 70d547d) | B | new launch arg `mocap_guard_predict` (default true); the box stacks pass `false` (= 4badef8's gate exactly: no velocity handed in) |
| 7 | Box tab opens the jaws on a lost DIRECT also from its own grip state (arm cb6cf54) | B, rare | that rule is Valve-only again |
| 8 | abort reply text, arrival_error [30]/[31] appended | cosmetic | the text follows the fast key; the appended values kept (indices ≤ 29 unchanged) |
| 9 | `wb_control_debug` [118..130] appended | telemetry | kept; [120] = contact flag, [121]/[125]/[126] = NaN (retired grip fields) |
| — | valve keep-outs, axis offset, exit room-held climb, sway guard, leash, Ready CoM gate, descent guard, guards before the handle, heading companion, mail delivery, Valve-tab-only GS code | C | unchanged (key-gated off for the box) |

Not reverted on purpose (no box behaviour): the Box tab's staged-value / amber-threshold UI details (cb6cf54: the box
yaml's values have ≤ 3 decimals, so identical) and the operating caveat that a valve mission that failed to restore the
arm's error damping leaves [0.2,0,0,0.2] for a later box flight in the same arm-stack session.

## 3. What was built (branch `valve-isolation-20261102` everywhere)

**fsc_autopilot_ros2** (the node, yamls, stacks)
- Node: `wb_grip_schedule.hpp` → `wb_contact_heading.hpp` (`wb::ContactHeading`): the heading companion FOLLOWS while
  the planner's contact flag is on, HOLDS after it, is forgotten with the free axis. The gripper subscription, the
  gain blend, the free-flight set and all 17 `wb_grip_*` / `*_before_grip` keys and `wb_l1_omega_c_t_before_grip` are
  gone — **the gripper position is no input to the node**. New keys: `wb_ee_anchor_free_axis_heading_after_contact`
  (replaces `_after_grip`), `wb_arm_release_on_stop`. Debug [120..126] slots kept (append-only).
- Tests: `WbGripSchedule.*` → `WbContactHeading.FollowsTheContactFlagAndHoldsAfterIt`.
- Yamls: both box yamls byte-identical to a316625; both valve yamls regenerated (one set, the box approach, the new
  keys); the mail-delivery hardware yaml + `wb_arm_release_on_stop: true`.
- Raw hardware stack: the box tuned-value list = 10-27's (62 values); the emergency-abort expectation valve-only; the
  valve list rewritten (87 values); refuses a yaml still naming the removed grip keys; probes the new node/planner
  keys; `mocap_guard_predict:=false` for the box; the valve's arm build must have `trim_target_includes_error_damping`.
  Fused stack: the removed-key refusal (it still refuses both contact profiles). Isaac 4-D stack: predict false for
  the box.

**fsc_trajectory_planner**: `push_pull_fast_hand_back` (default false: the q3/sway guard aborts and the leash use the
normal blend, as 10-27); `push_pull_grasp_event` + service `whole_body_planner/push_pull/grasp` (SetBool; true = the jaws
hold the handle → contact on; false = about to open → contact off, the anchor untouched; refused unless the key is on,
the plan is a turn, the claw is on the handle after Ready and held in the room, nothing is flying). Loopback test: the
fast hand-back is asserted unused on the box yaml ([9p]); grasp-event refusals ([9q]). A grasp event republishes
the info array at once (98500a3; [62] = the contact flag -- found by the Isaac driver, which waits on it).

**fsc_open_manipulator**: the controller's `trim_target_includes_error_damping` (LIVE; loopback 8b checks both
targets); the Valve Turning tab — jaws CLOSED at Go To Start (`push_pull_valve_close_in_free_flight`, default true;
`push_pull_valve_open_at_go_to_start` default now false), opened by Ready, closed by the gate after the planner's
hand-over (`push_pull_valve_settle_wait_max_s` 6 s), CLOSED again on DONE exit / ABORTED; "Static hold [s]" box
(`push_pull_valve_hold_s`, 0 = turn): grasp(true) → hold → grasp(false) → the valve's exit; the trim switch rides with
the damping raise. Box tab: the lost-DIRECT rule back to 10-27.

**fsc_drone_state_estimator_ros2**: launch arg `mocap_guard_predict` (default true).

**fsc_PegasusSimulator**: box generators restored to the baseline (`make_push_pull_yaml.py`, `make_hw_box_pushing_yaml.py`);
`make_valve_turning_yaml.py` self-contained (no import of the box generator) and moved to D1/P; the hardware valve
generator without the jaw sensor; the mail generator + the release key; `push_pull_planner_check.sh` compares the two
valve planner keys; the 4-D launcher's `ARM_GS_VALVE_HOLD_S` and `ARM_GS_EXTRA_ARGS` knobs. Tools here: `tools/param_dump.sh` +
`param_dump_compare.py`, `tools/gs_valve_phase_check.sh` (+ harness), `tools/driver/pl_mission.py` (`--gs-hold`),
`sim/chain.sh`.

## 4. The coupling after the isolation

What the two tasks still share, and why a valve change can no longer move the box:
- **Yamls / generators**: the valve generator no longer imports the box generator; the box generators are the baseline
  ones. A valve regeneration writes only the two valve yamls.
- **Node**: one law and one client. Every valve behaviour is a key the box yamls do not set, and each key's default is
  the 10-27 behaviour (parameter dump: section 5).
- **Planner**: one push-and-pull state machine. The valve's additions are keys whose defaults are off, or run only
  with `push_pull_turn_deg` ≠ 0 (which the Box tab forces to 0 at Plan).
- **Arm controller / GS**: one controller (the valve's trim switch is LIVE and default off); one panel class — the
  Valve-only code is under `valve_mode_`.
- **Estimator**: one guard; the box passes its 10-27 gate.

## 5. Tests

Logs in `checks/`, parameter dumps in `param_dumps/`, Isaac runs in `sim/runs/` (scorer `tools/sim_score.py`,
output `sim/score_*.txt`).

**Offline (no Isaac):**

| check | result |
|---|---|
| node + law gtests (`fsc_autopilot_tests --gtest_filter='Wb*:*Contact*'`) | 18/18, incl. the new `WbContactHeading` |
| planner gtests | push-pull 29/29, pick-place 32/32, ee-traj 5/5, teleop 9/9 (`StraightLine/Kinematics.ParityWithPython` fail as before: fixture deleted 09-17 / cwd-dependent) |
| planner loopback on the SIM and the HARDWARE box yaml | ALL PASS both; [9p] "no guard abort used the FAST hand-back (the 10-27 box)"; [9q] the grasp event refused with the key off, on a straight push, off the handle |
| arm loopback (`run_passthrough_corrections_test.sh`, mock hardware) | ALL PASS; 8b: key false → j4 trim −3.54 counts = the 10-27 target (−3.55), key true → +0.36 = the fix (+0.37) |
| **parameter dumps, box yamls (hardware + sim), the 10-27 build vs the new** | node: **164 common parameters, 0 differ**, 8 new ones all unset (code default = off); planner: **255 common, 0 differ**, 37 new ones at inert defaults (`tools/param_dump.sh`, `param_dump_compare.py`) |
| raw hardware stack pre-launch, `box_pushing` | "all **62** tuned values present -- the box-pushing tune AS FLOWN 2026-10-27" (the baseline's own check: 62); the guard banner `mocap_guard:=true mocap_guard_predict:=false` |
| raw hardware stack pre-launch, `valve_turning` | all 87 tuned values present, no "predates" line, the arm build declares the trim switch |
| fused stack, both contact profiles | refused, as before |
| new panel harness `tools/gs_valve_phase_check.sh` (real Valve + Box tabs, fake planner/arm) | **25/25**: V1 jaws CLOSED at Go To Start + damping/trim raised and restored + closed again after the Exit; V2 Ready opens, no close while the planner settles, close after the hand-over, the turn, no grasp event; V3 static hold grasp(true) → 1 s → grasp(false) → open (no release call) → Exit; V4 abort in the hold retires it, ABORTED closes the jaws; V5 no grasp service → nothing moves; V6 the Box tab: no gripper command at Go To Start or after the Exit, a lost DIRECT reads the planner only |
| the ten older panel harnesses | 7 PASS (all five Box-tab ones + `gs_valve_check`, `gs_valve_auto_check`); `gs_valve_damping_check` and `gs_valve_grasp_check` fail G1/G6 "jaws opened at Go To Start" -- the intended change; `gs_axis_check` A1 expects the pre-10-28-night axis offset [-0.0720, 0.0093, -0.0851] while the yaml has carried [-0.0935, -0.0014, -0.0899] since the marker redesign (stale before today) |

**Isaac** (headless, RTF 1, raw mocap, this desktop, 2026-11-01 evening; outcomes from `tools/sim_score.py`, box
metrics from `pl_metrics.py` in `checks/box_metrics_*.txt`).

*Box regression.* The 10-28 comparison campaign's whole-body flight (280 g, 0.40 m at 21 mm/s, its slots, the
release-first exit), flown three times on the NEW build (`iso_box_*`, 17:53-18:08) and three times on the 10-27 BASELINE
(`iso_base_*`, 19:35-19:50: node, planner and estimator from `~/baseline_box_20261027_ws`; the same scene, arm stack and
driver). All six complete.

| set | slide e_imp rms [N] | release e_imp [N] | eps slide max [mm] | slide claw deflection [mm] | jaw stall [rad] |
|---|---|---|---|---|---|
| new build, 3 runs | 1.08 ± 0.04 | 1.20 ± 0.38 | 23.0 ± 5.1 | 6.44 ± 0.41 | −0.215 / −0.241 / −0.227 |
| 10-27 baseline, same evening, 3 runs | 0.88 ± 0.02 | 0.75 ± 0.24 | 19.7 ± 2.5 | 4.80 ± 0.07 | −0.085 / −0.109 / −0.128 |
| 10-28 campaign (pre-isolation code), 5 runs | 0.99 ± 0.11 (0.78–1.08) | 0.65 ± 0.07 | 22.2 ± 4.2 | 5.67 ± 0.72 | −0.126 … −0.212 |

The new build reads ~0.2 N higher in the slide, consistently over its three runs. **This tracks how the simulated
jaws closed on the fin's grip shim, not the build.** The stall splits into two groups: shallow, around −0.1 rad
(0.46–0.54 s close), or deep, around −0.2 rad (0.60–0.82 s). Over all eleven runs the slide residual follows the group:
- shallow: 0.78–0.91 N;
- deep: 1.03–1.12 N.

The pre-isolation code shows the same split: on 10-28 one shallow run read 0.78 N and four deep ones 1.03–1.08 N. That
evening the baseline happened to close shallow 3 of 3 and the new build deep 3 of 3.

What the runs share:
- The planner's grasp reference at the close agrees to 0.3 mm.
- The 250 Hz debug stream and the step timing are identical.
- After the room hold engages, the claw follows the vehicle's ±5–10 mm wander through the free-axis anchor, with the
  same amplitude in both sets.

What decides deep or shallow is not identified; the claw's offset along the fin alone does not predict it. This agrees
with the parameter dumps (0 of 164 node / 255 planner parameters differ) and with section 2 (every post-10-27 box path
keyed off). Three runs a side cannot rule out a small code effect; interleaved re-flights would settle it.

*Valve* (v2 valve, 0.072 N·m drive model, the ARM GS flying every step by itself, each run stopped 4 s after the exit):

| run | case | outcome |
|---|---|---|
| `iso_vhold_2` | **static hold**, 10 s | **complete**. Jaws CLOSED at Go To Start, opened by Ready, closed by the gate after the hand-over. grasp(true) → 10 s → grasp(false) → open → Exit → jaws closed. Tilt ≤ 3.0°, q3 22.6–35.4°. |
| `iso_vhold_1` (superseded/) | static hold, 10 s | The GS ran the whole hold and the exit. The driver timed out waiting for the info array, which the planner did not republish on a grasp event (fixed, 98500a3). Re-flown as `iso_vhold_2`. |
| `iso_vturn_1`, `_2` | 90° turn | **complete**: beam +87°, tilt 5.8 / 4.1°, q3 ≥ 21.1°, no stop contact |
| `iso_vturn_3` | 90° turn | Tilt-guard abort at +61° of the turn (13.0 > 10°). The ~1 Hz q1/q4 swing (11–19° p-p in the completed turns) grew to 45° p-p. Clean: DIRECT kept, moved 0.03 m. |
| `iso_vturn_s2` | 90° turn, hardware-like drift (seed 2) | The GS aborted before the grasp (close gate not met; an open finger met the fin and turned the valve ~18°). Clean: tilt 2.9°. |

Forced aborts:

| run | forced fault | new build | the 11-01 build (`~/head_20261101_ws`) |
|---|---|---|---|
| `iso_q3trip` | q3 guard at 25° | clean (tilt 1.8°, DIRECT kept) | — |
| `iso_emerg` | watchdog tilt 3.5° → emergency abort | clean (tilt 4.7°) | — |
| `iso_holdfreeze_2` | mocap freeze 2.5 s inside the static hold | clean (DIRECT kept, peak 11.0°, moved 0.02 m) | — |
| `iso_mfreeze`, `_2`, `_3` | mocap freeze 2.5 s mid-turn | flipped / DIRECT lost but upright (40.5°) / clean (5.9°) | `iso_mfreeze_head`: clean (12.9°) |
| `iso_swaytrip`, `_2`, `_3` | sway guard forced to 0.05 m/s under drift seed 2 | flipped mid-turn / tripped before the turn, clean / the GS's automatic Ready refused for 25 s (the box's "claw still 0.5 s" gate under drift), never started | `iso_swaytrip_head`: flipped mid-turn |

Verdict:
- **The static hold works**, and every abort tried from it was clean. That is the planned first hardware flight.
- **Mid-turn aborts under a forced fault can still flip the vehicle on both builds.** The 11-01 build crashed the forced
  sway trip the same way, and the 10-22 record had the mocap freeze at 1 clean / 1 safe / 1 crash. Not a regression.
- **The turn itself did worse than on the 11-01 build:**
  - this build: 2 of 3 nominal, and the one drift run never grasped;
  - the 10-31 night and 11-01 builds: 13 of 13 unforced turns (configurations A and B, nominal and drift seeds 2/3/4;
    `archive/two_param_sets_20261101`).

  The drift failure is the one the 10-27 valve study found with a CoM-anchored descent (0 of 3 under drift then). The
  room-held descent (`push_pull_world_anchor_ready: true`, 3 of 3) had fixed it, and decision P (the box's approach)
  removed it. The nominal abort is the known ~1 Hz q1/q4 mode, now on a one-set config. These counts are small, but
  they point the same way. See section 7.

## 6. The way back (rollback to the 10-27 box push)

On the Orin (the three flight repos) and the laptop (the arm GS):
1. In each repo: `git fetch && git checkout box-pushing-20261027` (the branch; the tag `box-pushing-20261027-flown`
   marks the same commit).
   - fsc_autopilot_ros2 → a316625, fsc_trajectory_planner → 7d8ede9, fsc_drone_state_estimator_ros2 → 4badef8,
     fsc_open_manipulator → a0cb8b8 (laptop: the arm GS; Orin: the controller, identical to its 10-27 build 98f44e2).
2. Rebuild in order: `fsc_autopilot_ros2`, then `fsc_trajectory_planner` (links the law), then
   `fsc_drone_state_estimator_ros2`; in the arm workspace `open_manipulator_x_custom_controller` (Orin) and
   `utils_custom_ground_station` (laptop). A build that skips one of them leaves a binary from the newer branch.
3. Launch the RAW stack exactly as on 10-27:
   `AM_HW_PROFILE=box_pushing scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh`
   — its pre-launch check must print "all 62 tuned values present".
4. To return: check out the working branch again and rebuild in the same order.

The yamls do not need the baseline branch: on the new branches the two box yamls are byte-identical to a316625's.

## 7. Not done / known gaps

1. **The valve TURN is weaker in Isaac with the box's approach (decision P)**: section 5. Three ways on, the user's
   call:
   - fly more runs to firm up the count (about 4 min per run);
   - give the valve back its room-held descent: the valve generator's `push_pull_world_anchor_ready: true`. That is
     the 10-27 W, 3 of 3 under drift. It moves the valve's approach away from the box's again, and the settle keys
     then have nothing to do;
   - keep P and fly the static hold first as planned; the hold worked, and it says nothing about the turn.
2. **Mid-turn aborts after a forced fault can flip the vehicle**, on the pre-isolation build too (section 5). Not
   addressed here.
3. **Under hardware-like drift the GS's automatic Ready can run out of retries** (`iso_swaytrip_3`): the box's Ready
   gate (claw still at 30 mm/s for 0.5 s) never held in 25 s. On hardware the operator presses Ready again.
4. **The box regression is not resolved at three runs a side** (section 5). The scatter is the simulated grasp, and the
   parameters and code paths are the 10-27 ones; interleaved re-flights would settle it.
5. **Three older panel harnesses still fail, on purpose or stale.** `gs_valve_damping_check` and
   `gs_valve_grasp_check` expect the jaws to OPEN at Go To Start (the old behaviour). `gs_axis_check` expects the
   pre-10-28 axis offset. None were updated; `tools/gs_valve_phase_check.sh` covers the new behaviour.
6. **The coordinated-admittance valve yaml** has been stale since 10-28 and is untouched.
7. **The jaw state at TAKE-OFF is the operator's.** The Valve tab acts from Go To Start on; close the jaws from the
   Control tab before take-off.
8. **Nothing is pushed or deployed**, nothing is flown on hardware, and the Orin and the laptop are untouched. The
   prompts for both are beside this file.

## 8. 2026-11-02 follow-up: the 11-01 flights and the abort / exit aligned with the box (Pegasus archive/wb_valve_turning_flight_20261101)

The four 11-01 hardware flights flew the 11-01 PUSHED build (grip schedule + two sets, the room hold from Go To
Start, the leash / Ready CoM gate / descent guard), not this branch; three aborted in the room-held descent, the
fourth grasped, tripped the sway guard 0.8 s later and crashed inside the abort (the base moving ~350 mm/s at the
abort, the claw following it across the fin for 2 s, caught in the climb). On this branch, in addition to sections 3-4: the valve abort is now the
box's recipe (`push_pull_abort_from_measured true`, `push_pull_abort_release_wait_s` 1.0 sim / 1.5 hardware,
`push_pull_abort_release_world_held false`; unbuckle stays off), and the Valve tab's exit starts on the OPEN
report (`push_pull_valve_exit_wait_s` default 1.0 -> 0.0). The abort change does NOT address the 11-01 crash
mechanism and reverses a 10-18 Isaac result; untested. The raw stack's valve expectations follow; the tab
harness 25/25 and both pre-launch checks pass. Still uncommitted on the branch.

## 9. 2026-11-02 (later): the valve's exit, aligned to the box's workflow

The user asked for the valve to run the box's workflow (page "Valve-turning vs Box-pushing logic",
`archive/wb_valve_turning_flight_20261101/logic_compare.html`). Every phase up to the grasp already matched; the exit was
then made the box's (release first) and, after the concern below, set to the VALVE's order with the box's start conditions:
- **Start (the box's):** 0.2 s after the turn ends, or after a static hold, the calm wait: base slower than
  `push_pull_auto_exit_max_speed` 40 mm/s for 0.3 s (at most 3 s), plus (valve only) the law's CoM error under
  `push_pull_valve_exit_max_base_err` 0.020 m -- the planner now publishes it as `push_pull/arrival_error` [32] ([33] = its
  rate). A large error holds the exit even without a base pose. The Exit button still goes at once.
- **Order (the valve's):** a static hold reports grasp(false), then the jaws OPEN with the claw still held in the room, the
  climb starts on the open report, and the planner keeps the claw held until it has climbed `push_pull_exit_world_held_dz`
  0.06 m, then the CoM anchor. Station parameter `push_pull_valve_box_exit` (default **false**; true = the box's release-first
  order).
- **Why not the box's order:** the handle cannot move sideways. Isaac: release-first 2 of 7 vs open-first 3 of 3 (10-18); the CoM
  anchor at the exit's start pressed a finger into the fin, 2 of 6 left DIRECT, the 60 mm hold 7 of 8 clean (10-28).
- **The exit-window abort (planner, new `push_pull_abort_exit_hold`, default false, valve yaml true):** an abort after the turn,
  after a static hold's release, or in the exit's room-held climb keeps the claw HELD IN THE ROOM -- whatever guard fired --
  through the release wait (0 in the climb: the jaws are already open) and until the claw has climbed
  `push_pull_exit_world_held_dz` (at most `push_pull_exit_world_held_max_s` after the wait), then the CoM anchor. A mocap-guard
  abort still hands back at once. The raw hardware stack expects the key and refuses a planner without it.
- Checks: station harness `tools/gs_valve_phase_check.sh` **30/30** (V3 static hold: calm, grasp off, open, exit, no release; V7:
  waits while the base moves, then open, exit, no release; V8: the switch gives the box's order; V9: a 45 mm base error holds
  the exit, 10 mm lets it go; V6 the Box tab unchanged); planner loopback new [9r] (arrival [32]; a sway-guard abort after the
  push keeps the room hold through the wait and hands back only after the claw climbed 70 mm); raw stack pre-launch: valve 88
  values, box 62. Planner + station rebuilt. Not flown, not in Isaac.
