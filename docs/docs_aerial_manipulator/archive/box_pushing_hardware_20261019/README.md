# Box pushing: review of the simulation and the hardware flight version (2026-10-19)

User request: double-check the box-pushing simulation's logic, then make its hardware
flight-test version use the latest tuned box-pushing parameters; runbook commands
(flight stack, ros2 bag) and the operator workflow
(`~/Documents/box_pushing_workflow/box_pushing_workflow.html`).

Nothing was flown: no Isaac run and no hardware flight. All verification below is
offline: unit tests, loopback tests against the built planner, offscreen GS harnesses,
and dry runs of the stack scripts' pre-launch checks.

## 1. What was reviewed

| part | where | how |
|---|---|---|
| planner push-and-pull mode | fsc_trajectory_planner `push_pull.{hpp,cpp}`, node push-pull section | read in full |
| arm GS Box Pushing / Valve Turning tabs | fsc_open_manipulator `push_pull_panel.*`, `control_panel.*` | read + reviewer agent |
| whole-body node contact path | fsc_autopilot_ros2 whole-body client, law, observer, schema | reviewer agent (key-by-key load check) |
| Isaac scene, wrappers, driver, generators | Pegasus `07_..._box_pushing.py`, launchers, `pl_mission.py`, `make_push_pull_yaml.py` | reviewer agent (FK + geometry recomputed) |
| hardware stacks | `start_whole_body_l1_4d_..._stack_t650_aerial_manipulator{,_fused}.sh` | read + dry runs |

**Verified correct:**
- **Geometry:** table [1.200, 1.350, -0.5, 0.5, 0.80]; box centre (1.275, 0.28); grasp 0.233 m above the table, i.e. 0.183 m above the box centre.
- **Push direction:** nose yaw = box yaw - 90 deg, so the box moves along world -y.
- **Skid clearance at the push pose [0, 32, 28, 0]:** 122 mm above the table and 22 mm above the box top, and the skids straddle both.
- **The box-on-table check.**
- **The ROS interfaces** (topics, services).
- **The info / arrival_error index maps.**
- **Parameters loaded:** all 148 node keys of the hardware yaml are declared and loaded, with matching types.
- **Untouched paths:** the pilot-takeover fix is intact, and the free-axis anchor is inert at weight 0.

## 2. Findings and what was done

### Hardware blockers or hazards (fixed)

1. **A real grasp on the fin would not start the push.**
   - The GS pushed only on a stall (ABORTED + stalled), the Isaac claw's behavior on its grip shim. The real foam pads close almost FULLY on a thin fin: 10-16 hardware, -0.0097 of the -0.010 target on the 3 mm holder fin. That is inside the 2 mm goal tolerance, so the result is SUCCEEDED, which the tab read as "nothing in the jaws".
   - The opposite case was also wrong: a stall at -0.0010 (10-16 D-1, a wrong grip) counted as held.
   - New: `ControlPanel::gripperCloseResult(outcome, position)`. The push tabs apply the HARDWARE rule (a close that reaches its goal, or stalls no wider than `push_pull_grip_max_open` -0.005 m) whenever the station's `simulation` parameter is false, which is what the hardware launcher gives. Isaac keeps the stall rule. `push_pull_full_close_is_grasp` overrides.
2. **Tilt limits:** the planner's tilt guard equalled the node's watchdog (15 / 15 deg). The node trips instantly; the planner needs 0.1 s, so it never ran first. The node then reverts to SAFETY with the jaws clamped (the valve's 10-18 lesson).
   - Data: 51 completed Isaac box-pushing missions peaked at 6.1 deg (p95 4.3); every failure exceeded 12.
   - Fix: hardware `push_pull_guard_tilt_deg` 10. It is a HW_PLANNER substitution in `make_hw_box_pushing_yaml.py`; the sim yaml, its results and the comparison rig are untouched.
3. **The fold after a SAFETY revert:** the node folds the arm 1.0 s after the revert. The real gripper opens in 0.60-0.64 s to 90 % and settles in 0.65-0.70 s (all ten opens in the 10-15/16 bags), and the GS opens it only after it sees the revert.
   - Fix: hardware `system_arm_go_home_delay_s` 2.0, a new HW_NODE substitution.
4. **A frozen mocap pose:** Get accepted one, and the box guards were blind to it.
   - On 10-09 the processor republished ONE obj_0 pose 11 423 times with fresh stamps.
   - Get now refuses a frozen feed or a flipping yaw (`push_pull_capture_max_yaw_spread_deg` 10).
   - In contact, a pose unchanged for longer than `push_pull_guard_box_stale_s` trips the box-feed guard, like a silent feed. It needs at least 3 bit-identical samples, so a repeated VRPN frame is not enough.
5. **Dead-ends that left only ABORT, with the jaws on the box:**
   - Any plan-dropping edit (Land, EE offset, margins, distance, Adjust) was accepted while the claw was on the handle. Afterwards Exit needed the plan, and Plan and Reset were refused. Now the planner refuses those edits, and the GS disables them.
   - A failed, rejected or missing close result left the tab "closing" forever. Now the failure is reported, and a 6 s timeout covers a missing result.
   - A refused push, pressed again, re-ran the 3 mm centring gate on a clamped claw. Now, with the jaws still gripping, Push calls the push at once.
   - A release request that got no reply blocked the tab. Now a timeout ends the wait and nothing opens.
6. **Ready sent the open and the descent in one click.** A rejected or slow open slid the claw onto the fin with the jaws shut, and the force guard is not armed then. Now the descent starts only on the gripper's OPEN report.
7. **Box Exit's "older planner" fallback** opened the jaws without the release, which is the 10-17 crash sequence. A WiFi discovery drop looks the same. Now nothing opens.
8. **ABORT on a push tab** opened the gripper and cut another mode's leg, so a pick-and-drop carry would drop its basket. Now the planner refuses that abort, and the GS opens only when this mode holds the handle. The double OPEN on ABORT is gone too.
9. **Refusal messages** were wiped by the planner's 100 ms status republish. They now stay up their 8 s unless the planner's state changes, or another tab switches its mode.

### Feedback: RAW mocap for box pushing (decided on data)

- **Isaac:** EKF2-fused feedback failed every push (0 of 6, a ~3 N phantom from a 10-15 mm drift in contact).
- **The raw path on the real vehicle today**, measured on `/uav_0/mocap` in the six 10-15/16 hardware flights (`npz/` of the two flight folders):

  | measure | result |
  |---|---|
  | one-sample jumps > 20 mm | 0 |
  | orientation jumps > 10 deg | 0 |
  | frozen runs | 0 |
  | max gap | <= 31 ms |
  | velocity noise above ~2 Hz | 0.7-2.0 cm/s rms (fused 0.5-0.8) |

  None of the 09-21 flips (heading <= -70 deg, which the push holds), and nowhere near the 10.8 cm/s of that day.
- So the fused hardware stack now REFUSES `AM_HW_PROFILE=box_pushing` (`ALLOW_FUSED_CONTACT=1` overrides), as it already did for the valve. The raw stack gained the fused stack's DDS/WiFi decoupling check, which it lacked.

### Simulation harness (fixed)

- **`pl_mission.py`:**
  - After one of the driver's OWN aborts (no grasp, a refused step, a timeout, too much disturbance) near the table, it used to descend in place onto the box. It now opens the jaws, calls the planner's ABORT and leaves by Go To Land Start.
  - The release-first exit is the default (`--open-first-exit` for an A/B), and `fly.sh` records it.
  - `--slot-land-step` 50.5 -> 27 s (the leg is now one 23.9 s transition).
- **Sim wrapper:** `PUSH_FEEDBACK` defaults to `raw` (it defaulted to the fused feedback that failed every push).
- **`lib/push_pull_planner_check.sh`:**
  - It also compares the box guards and the abort.
  - It requires the running planner to declare `push_pull_capture_max_yaw_spread_deg`, i.e. today's build or later. A planner older than 10-16 used to pass.
- **Stale comments** in the planner block, copied verbatim into the HARDWARE yaml, corrected: 400 g, post handle, 95 mm, mu 0.29, gear 0.313, 0.10 m margin, contact timing, header lists. The scene docstring and Command.md section 17 were corrected too. The yaml VALUES are unchanged: all generated box-pushing yamls were regenerated, and the coordinated-admittance one diffs to comments only.
- **The hardware stacks' config check:** it expected `wb_l1_omega_x` 0.2072 on every profile and printed a false MISSING on box pushing / valve turning. It is now profile-aware, and the tuned-check summary names the tune it verified.

### Not changed (open)

- **Instant `F_y` step at the contact switch:**
  - The 4-D observer gates `F_y = chi_free ? 0 : F_f` with no blend.
  - At Push, `F_y` jumps by `F_f`. That is ~1 N in Isaac, but on hardware the free-flight reading has p99 2.4 N (0929). The planner's 5 N force guard low-passes the same reading.
  - Log `wb_control_debug` [97..99] / [105] across the first hardware Push before trusting the margins.
- **CoM-anchor blend velocity term:** the blend lacks the `w_dot * e_x` term, a ~0.6 N D-term pulse over the blend. Flown fine in Isaac.
- **The grip gate fires on one 10 Hz sample of the MODEL claw.** It is hardware-proven on the pick. If the jaws consistently close off-centre across the fin on hardware, correct EE Offset x.
- **The table** ships as the simulation's. Place it by ruler at the reference corner (1.20, 0.50) with the top at 0.80 m, or measure it and edit `make_hw_box_pushing_yaml.py`.
- **`make_coordinated_yaml.py --check`** reports the coordinated-admittance MIRROR yaml (`..._t650_sim.yaml`) stale. That was already the case before this work: its source is the free-flight mirror. Not regenerated.

## 3. Verification

| check | result |
|---|---|
| planner gtests (`colcon test`, fsc_trajectory_planner) | 73 / 73 |
| `test_push_pull_loopback.py` (full mission, guards, abort, release) | ALL PASS |
| `test_push_pull_box_point_loopback.py` (+ [7] frozen, [8] flipping refused) | PASS, clean teardown |
| GS `gs_box_check.sh` (real planner) / `gs_release_check.sh` / `gs_valve_check.sh` | PASS / PASS (D updated: no release service -> nothing opens) / PASS |
| GS `tools/gs_grasp_check.sh` (new, 27 checks: H hardware rule, S sim rule, R Ready, A abort, E edits) | PASS |
| `push_pull_planner_check` against a live planner (ok / missing required key) | rc 0 / rc 1 |
| hardware stacks, pre-launch dry run (cut before `tmux new-session`, private domain) | raw box_pushing: 40 / 40 tuned, no MISSING; raw free_flight and fused pick_and_drop unchanged; fused box_pushing refused |
| generators `--check` | box sim, box hw, valve sim / hw, coordinated box: UP TO DATE |

## 4. Deploying to the hardware

User decision (2026-10-19): only the whole-body box-pushing experiment. That is now
committed LOCALLY, one commit per repo. Nothing is pushed.

| repo (branch) | commit | contents |
|---|---|---|
| fsc_trajectory_planner (`main`) | `c8d3d37` | the push-and-pull mode as flown in Isaac, plus today's capture gates and handle / abort locks |
| fsc_open_manipulator (`omx-torque-control`) | `84f6a98` (on your `f812b48`) | the Box Pushing tab: the grasp rule, release-first exit, no dead-ends. No Valve Turning tab |
| fsc_autopilot_ros2 (`dev_robotic_arm`) | `002c301` (on your `63d77e4`) | the hardware box yaml + box-only stack scripts (raw: the box profile; fused: refuses it) |

Kept OUT of the commits, still in the working trees:
- **Valve turning:** the valve profile in the stack scripts, the Valve Turning tab in `control_panel.cpp`, the valve yamls.
- **The unflown 2026-10-16 j4 law fix:** the node C++, and the free-flight and pick yamls' heading / stuck-hold values, together with the stack scripts' expectations of them.
- **The coordinated-admittance fork:** its OSQP FetchContent would break an offline Orin build.
- **All simulation-side files,** including fsc_autopilot_ros2's pre-staged sim-yaml rename, which is still staged.

The planner's turn form shares the push-pull code and is inert at `push_pull_turn_deg` 0; it ran in every box-pushing Isaac campaign since 10-15.

The two script commits were built with a temporary index, from box-only versions of the scripts. Those were dry-run against the COMMITTED free-flight / pick yamls (box_pushing 37 / 37, free_flight and pick_and_drop 28 / 28, the arm check finding the j1 relay at 0, fused box_pushing refused).

The GS commit was built and tested in its box-only form: the three harnesses passed, and the real station binary started with the hardware grasp rule. The working tree then got its Valve Turning tab back.

After a push:

| where | steps |
|---|---|
| Orin | pull fsc_trajectory_planner and REBUILD it; pull fsc_autopilot_ros2 (the stack reads its yaml and scripts from the source tree; the node itself is unchanged since `f6395dd`, which declares every key the box yaml relies on) |
| laptop | pull fsc_open_manipulator and rebuild `utils_custom_ground_station` |

The box-pushing stack refuses to start on a planner without today's capture gates.

## 5. The automatic exit and the safety-guard audit (2026-10-19, second request)

User decision: **Exit To Push starts by itself.** The clamped hold after the push is the
mission's least-guarded moment: no push is running, so the box-lag guard is off. Every Isaac
push released within 1–1.5 s of `DONE push`.

### The automatic exit (arm GS, Box Pushing tab)

- **When:** `push_pull_auto_exit_delay_s` (1.0 s) after the planner's `FLYING push` → `DONE push`
  edge. It runs the same sequence as the button: release, open, then exit on the gripper's
  OPEN report. The lamp turns purple and counts down.
- **Re-checked when it fires:** still `DONE push`, the claw still on the handle, nothing flying,
  no exit already started, and this tab active.
- **Cancelled by:** ABORT (the button or a planner guard's `ABORTING`), a DIRECT loss, Reset, or
  pressing 4 Exit To Push (which exits at once, exactly once).
- **Only on the edge:** a station started while the box is clamped never exits by itself. It
  also never fires after a failed grasp (no push flew) or on the Valve Turning tab.
- **`push_pull_auto_exit_delay_s` ≤ 0** = by hand only.
- **Sim campaigns:** the autonomous drivers (`pl_mission.py` through `run_pl.sh` /
  `run_pl_cadm.sh`) run release / open / exit themselves. They now start the arm GS with
  `ARM_GS_AUTO_EXIT_DELAY=0.0`, which both Isaac launchers forward as the parameter.
  Campaign behaviour is unchanged.
- **Tested** with `tools/gs_grasp_check.sh`: 42 checks, PASS. New cases:
  - X1: release → open → exit on the report, nothing inside the 1 s.
  - X2: a latched `DONE push` without the edge does nothing.
  - X3 / X6: `ABORTING` or `NOT IN DIRECT` inside the delay — no release, one open.
  - X4: a manual press inside the delay gives exactly one exit.
  - X5: delay 0 does nothing.

  The release, box and valve panel checks pass as before. The real GS binary accepts the override
  (`push_pull_auto_exit_delay_s` reads back 0.0).

### Every guard in the box-pushing mission (hardware values)

| guard | armed | trip | response |
|---|---|---|---|
| planner tilt | DIRECT, a push-pull step done / flying / on the handle | > 10° for 0.1 s | planner ABORT |
| claw force (law's F̂, LP 3 rad/s) | **claw on the handle or the push flying** (was: the law's contact phase only) | > 5 N for 0.3 s | planner ABORT |
| box lag | the push flying | > 40 mm behind / ahead of plan, 0.2 s | planner ABORT |
| box tilt | the push flying | > 2°, 0.2 s | planner ABORT |
| box mocap | claw on the handle / pushing | silent OR frozen (≥ 3 identical samples) 0.5 s | planner ABORT |
| q3 buckle | claw on the handle / pushing | q3 < 10° for 0.2 s | planner ABORT |
| **arm joint states (new)** | same as q3 | silent 0.5 s (the q3 guard is blind) | planner ABORT |
| Get / Plan | before Ready | frozen or flipping obj_0, box would leave the table (3 cm), keep-outs | refused |
| node watchdog | DIRECT | tilt 15°, rate 360 °/s, CoM drift 0.75 m, feedback 1.0 s | SAFETY; GS opens the jaws, arm home 2.0 s later |

**The planner ABORT on the handle:**
1. The GS opens the jaws (on the button, or on the `ABORTING` edge).
2. Contact goes off; the claw goes back to the CoM anchor (not world-held: the 10-17 crash needed
   a world-held open claw).
3. The vehicle holds the measured pose **1.5 s** (sim 1.0), backs 3 cm off, climbs 0.30 m, and the
   arm goes home.

Then Go To Land Start / Land.

### Fixed in the audit

1. **Force guard:** it was armed only in the law's contact phase, so the hold before the push and
   the hold after it had none. It is now armed whenever the jaws can be on the fin. Isaac, 51
   completed missions, filtered as the guard filters (peak values):

   | phase | peak |
   |---|---|
   | before Push | 1.06 N |
   | push | 2.57 N |
   | hold after the push | 1.87 N |
   | exit | 1.05 N |

   All are far below 5 N. Loopback [9b]: 6 N before the push aborts.
2. **The q3 guard skipped its check without a word when the arm's joint states stopped.** A
   0.5 s silence now aborts. Loopback [9c].
3. **Two tilt guards, one tilt:** the pick-and-place and push-pull guards read the same attitude.
   When both trip in one tick, only the mode that holds the handle (or is flying) aborts; the
   other re-arms. Before, both ran.
4. **Abort hold 1.0 → 1.5 s on hardware** (`make_hw_box_pushing_yaml.py` HW_PLANNER; both stacks
   check it, 41 tuned values). The OPEN goes planner → laptop GS → arm over two WiFi hops, then
   the gripper takes 0.65–0.70 s; 1.0 s left 0.2–0.3 s for the hops.
5. The loopback test no longer core-dumps at teardown (spin thread joined before the node goes).

Verification:
- planner gtests 73 / 73;
- `test_push_pull_loopback.py` ALL PASS (rc 0);
- the GS harnesses above;
- the stack dry-runs: raw box_pushing 41 / 41 OK; fused refused (with `ALLOW_FUSED_CONTACT=1`,
  41 / 41).

### Residual risks (not changed; the user's call)

1. **Only the laptop arm GS opens the jaws.** With the laptop or WiFi down in contact, a planner
   abort climbs after 1.5 s with the jaws closed and lifts the 500 g box until it drops. Fixing
   it means the planner (on the Orin) commanding the gripper itself on an abort: a second owner
   of the gripper, so a design change.
2. **The SAFETY revert dips ~20 cm.** The SAFETY thrust map (`vehicle_thrust_scaling` /
   `idle_thrust` 0.036758 / 0.241528) is the bench-kf map (4.540431e-05), while the allocator
   flies 4.260431e-05. Every airborne DIRECT → SAFETY revert on this rig dipped 186–225 mm
   (0918–0929). With the claw at the fin, that dip goes toward the box and the table.
   - It happens only on a node-watchdog trip; the planner's 10° guard aborts upward first.
   - The map matched to the flown kf is **0.037947 / 0.252449** (hover 0.6011 vs 0.5792, the
     measured 0.015–0.023 step). It is not applied: the experiment kf is the user's.
3. **Losing the vehicle's mocap in contact** makes the node land in place at 0.2 m/s after 1.0 s,
   i.e. onto the table. The GS opens the jaws at once and the arm folds after 2 s. The 10-15/16
   hardware `/uav_0/mocap` was clean.
4. **Box lag 40 mm:** Isaac's nominal push lags 0–25 mm, and hardware tracking has run 1.6–2.9×
   the sim's. A false trip is an abort, i.e. the safe direction. If it happens, read
   `push_pull/arrival_error[11]` in the bag before raising it.
5. **obj_0 near the claw:** single-sample flips pass the 0.2 s persistence; a sustained mis-solve
   trips the box guards (abort). Keep the markers asymmetric and away from the fin.
6. **Force noise:** hardware F̂ is much noisier than Isaac's (0929 free flight: 2.4 N p99 at
   2 rad/s). The guard's 3 rad/s low-pass and 0.3 s persistence should hold 5 N; on a false trip,
   read `arrival_error[10]`.

### Commits (local, not pushed)

| repo (branch) | commit | contents |
|---|---|---|
| fsc_trajectory_planner (`main`) | `6ca6e9d` | the three guard fixes + loopback [9b] / [9c] |
| fsc_open_manipulator (`omx-torque-control`) | `b6ab037` | the automatic exit (the two panel files only; built and harness-tested in that box-only form) |
| fsc_autopilot_ros2 (`dev_robotic_arm`) | `dab6c69` | the 1.5 s abort hold in the hardware box yaml + both stacks' checks (temporary index, box-only script versions; raw dry run 38 / 38) |

The sim-side changes (the launchers' `ARM_GS_AUTO_EXIT_DELAY`, the drivers, the generator,
these tools) are in the uncommitted Pegasus tree, like the rest of today's sim work.

## 6. Pushed for the flight test (2026-10-19)

User request: commit and push all local work, simulation and experiment. Pushed:

| repo (branch) | head | added since section 5 |
|---|---|---|
| fsc_trajectory_planner (`main`) | `6ca6e9d` | — |
| fsc_autopilot_ros2 (`dev_robotic_arm`) | `be1f6cb` | `05c9cdb` j4 stuck hold, free-axis latch, anchor-blend load, valve-turning profile, sim yaml rename; `be1f6cb` the coordinated-admittance fork |
| fsc_open_manipulator (`omx-torque-control`) | `98f44e2` | the Valve Turning tab, the coordinated-admittance Isaac config |
| ros2_ground_station_gui (`dev_robotic_arm`) | `2aaad05` | rotor pies for the coordinated-admittance fork |
| fsc_PegasusSimulator (`dev_robotic_arm`) | this commit | every sim / tool / doc change since 10-16 |

**The coordinated-admittance fork is behind a CMake switch.** Its planner fetches OSQP from
GitHub at configure time, so `FSC_BUILD_COORDINATED_ADMITTANCE` is OFF by default on aarch64
(the Orin) and ON on x86. Both checked: OFF in a fresh build fetches no OSQP and builds neither
fork binary; ON builds both and runs their parity tests.

**For box pushing the rebuilt node flies the same law as `f6395dd`**: the box yaml keeps the
j4 hold off, the 10-09 heading and no free-axis anchor, and sets no anchor blend (1.0 s
either way). Rebuild anyway, so the Orin flies the binary that was checked in Isaac today.

Both planner loopbacks pass against the HARDWARE box yaml (`PL_YAML=...t650_box_pushing.yaml`),
which is how the Orin self-check runs them.

**Not committed (raw data, the repo's `.gitignore` policy):** 132 bag `.db3` files (~9.4 GB:
hardware flight bags 10-05..10-16, the 09-09 / 09-11 arm calibration bags, the `results/`
copies), 88 `.mat` + 2 `.zip` MATLAB exports (~1.6 GB), and the 10-10 flight video (516 MB).
Their `metadata.yaml`, analysis figures and notes are committed. Hardware flight bags under
100 MB have been committed before as separate data commits; that is still open for these.
