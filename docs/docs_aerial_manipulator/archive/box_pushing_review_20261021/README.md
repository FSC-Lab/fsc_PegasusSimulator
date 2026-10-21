# Code review of the 2026-10-21 box-pushing work (read-only, before the push)

Four read-only reviewers on the uncommitted work of 2026-10-20/21: the planner
(`fsc_trajectory_planner`), the arm GS Box Pushing tab (`fsc_open_manipulator`), the
raw-mocap guard (`fsc_drone_state_estimator_ros2`, the emulator, the whole-body client and
stacks in `fsc_autopilot_ros2`), and the configs/tooling (generators, yamls, scene, scripts).
Nothing was built or run. The two findings marked **verified** were re-read in the code
afterwards. The code was first pushed as it was (estimator `c196507`, planner `c4fd5c3`,
autopilot `4d44739`, arm `f5dde69`, emulator `d7f88e6`, drone GS `485d8ac`); the same day,
on the user's request, items 1, 2 and 7 and three GS display items were **FIXED** in a second
push (see "Fixed" at the end).

## Fix before the hardware flight (both FIXED, see the end)

1. **Estimator: a second fault during the blend back steps the odometry** (verified).
   `mocap_odom_bridge.cpp`, `advance()`: at an episode start `d0_ = d_`, the offset of
   the RAW mocap, and `blend_active_ = false`. While a blend runs the published position
   is `mocap + w * blend_delta_`, so the bridge (EKF2 + d0 = the raw mocap) jumps by
   `w * blend_delta_` in one sample. At the next recovery `blend_delta_` is computed
   from that raw-based bridge, so the step is never undone. A long pause leaves
   delta = 0.2–0.6 m (Isaac), the blend then runs 2–6 s at 0.10 m/s, and pauses cluster
   (09-24: three in one heading sector). Fix: at the episode start, if a blend is
   active, `d0_ = d_ + w(t) * blend_delta_` (the offset of what was PUBLISHED).
2. **Planner: a push slower than distance / `t_max` flies faster than shown** (verified).
   `planPushPullPush()` passes `T = |d| / v` to the flat planner, which clamps
   `T = min(max(T, T_min), T_max)` (`flat_planner.cpp`, `T_max` 40 s in the box yamls)
   without saying so. 0.40 m at 8 mm/s shows 50 s GREEN and flies in 40 s. Not reached
   at the shipped 21 mm/s (0.50 m = 23.8 s). Fix: refuse `T > t_max`, or cap the
   speed range's lower end at `d_max / t_max`.

## Worth deciding

3. **Planner: Exit is refused while the guard is BRIDGING** (a gap over 50 ms), and the
   arm GS calls Exit ONCE, on the gripper's OPEN report. A short gap at that moment
   leaves the open claw around the fin, with no retry and no abort. The tab shows the
   refusal; pressing Exit again works once the mocap is back.
4. **Planner: a CONFIRMED fault during Exit (or Ready) still waits the release hold**
   (1.5 s on hardware) at the EKF2 pose before climbing, though the jaws are already
   open.
5. **Planner: steps are allowed again during the blend back** (only `data[0]` is read;
   `data[3] == 2` means the odometry is still sliding to the mocap for up to a few s).
6. **Estimator: LOST has no time bound and the bridge ignores EKF2's validity / reset
   counters**: after `EKF2_NOAID_TOUT` (5 s) PX4 may mark the horizontal solution
   invalid while the law still hovers on it. The pilot is expected within ~2 s.
7. **Estimator: the 1.0 s default `mocap_guard_test_fault_s` reaches LOST.** The age of the
   last good sample passes `t_lost` (1.0 s) before the next good sample, so the hover
   test shows LOST (red, "take over and land") and aborts a flying step. Use 0.5 s to
   test BRIDGING → CONFIRMED only. An integer on the launch line (`:=1`) is refused by
   the double parameter.
8. **Landing: the 0.30 m Land hover is below the SAFETY UDE gate** (`ude_height_threshold`
   0.4 in both whole-body box yamls) and every DIRECT → SAFETY revert on this rig dips
   0.19–0.23 m. The runbook's "back to SAFETY, land" at 0.30 m is a touchdown from ~5 cm.
   Not flown in Isaac at this Land height.

## Low

- Estimator: the linear blend steps the published velocity by `|delta| / dur` (≤ 0.1 m/s)
  at both ends; non-finite samples are reported as "stale"; after a > 1 s gap the jump
  gates are wide open (a flip at re-acquisition is accepted); the first ~50 ms of a
  freeze reaches EKF2.
- GS: the push's held numbers last only while `push_pull/arrival_error` keeps arriving
  (lost on an abort that drops the plan, out of DIRECT, a stale odometry); a typed
  pull of the planner's distance (−0.10 vs 0.10) shows the plan GREEN; a transient red
  "nan m" note before the parameters load; a failed gear-parameter load is never asked
  again; the Mocap lamp reads "guard off" when the link is down at startup; the LOST
  beep cannot be silenced and sounds on every tab (the X bell may be off on the laptop);
  the contact-line tooltip still says "CONTACT from the arrival on the handle to Exit"
  (it is grasp → end of the push).
- Configs: the fused stacks accept box pushing (with `ALLOW_FUSED_CONTACT=1`) but start
  no guard, so every step is refused, silently; the campaign's `fly.sh` flies 0.5 m / 24 s
  with the floor 0, not the shipped 0.021 m/s / 3 s; the coordinated-admittance rig's
  `mpc_p_min` z equals the 0.30 m Land target; comment-only drift in four yamls and two
  docstrings; `view_box_pushing.json` still frames the old table.

## Checked and found correct (summary)

Array sizes and indices on both sides (`push_pull/info` 77, `arrival_error` 30, the GS's
constants and size checks); `pushPullPushTime()` edge cases; the tested ranges are gate-only;
valve turning unchanged; frames NED/FRD → ENU/FLU in the bridge; no bad sample reaches PX4
by another path; ladder parity with `guard_reference.py`; the flight client's DIRECT refusal
and failsafe excuse; every generated yaml byte-identical to its generator; the hardware
stack's expected-value list passes on the hardware yaml; the layout move consistent in the
scene, yamls and tools, and absent from the valve rig.

## Fixed (2026-10-21, second push)

| finding | fix | verified by |
|---|---|---|
| 1 estimator: blend lost at a second fault | `advance()`: at an episode start during a blend, `d0 = d + w(t_last good) * blend_delta` -- the offset of what was published | `tools/estimator_loopback.py` NEW case 7 (EKF2 drifts 0.225 m in a 1.5 s freeze, a 0.3 s freeze 0.6 s into the 2.2 s blend back): the unfixed estimator stepped **161 mm** in one message; fixed, the largest offset change is **17.8 mm** = the frozen detector's own onset latency (the odometry stands still for ~50-80 ms before a freeze is recognised, minor finding 7, unchanged), and it ends on the mocap; all 28 checks PASS; gtest `test_mocap_guard` 10/10 |
| 2 planner: slow push clamped to `t_max` | `planPushPullPush()` refuses a straight min-snap push whose time exceeds `PlanOptions::T_max`, naming `push_pull_push_mean_speed` and the distance | gtest `PushPull.PushTimeComesFromTheMeanSpeed` (0.40 m at 8 mm/s refused naming t_max; at 10.5 mm/s planned at its own 38.1 s); push suite 18/18; box-pushing loopback ALL PASS on the sim AND the hardware yaml |
| 7 test fault reached LOST | `mocap_guard_test_fault_s` default 1.0 -> **0.5** s (header + launch): CONFIRMED and back, never LOST; above 1.0 s it exercises LOST on purpose | loopback case 5 now expects states {0, 1, 2}, PASS |
| GS: a typed pull of the planner's distance showed green | the preview compares the SIGNED distance | `gs_push_check.sh` PASS (real planner, offscreen) |
| GS: "nan m is outside" before the parameters load | an unknown value is uncoloured, not red | same |
| GS: the contact-line tooltip | describes the box-pushing timing (grasp -> end of the push) | -- |

Still open (decisions, not code): 3, 4, 5, 6, 8 and the low items above.
