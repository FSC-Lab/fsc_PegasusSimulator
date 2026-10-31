# Valve turning on the v2 valve, 90 degrees (2026-10-27)

User requests, in order:
1. Fly the new valve mechanism with its new calibration data.
2. Borrow what the box-pushing experiments taught.
3. Test only the interaction phase: stop each run after the turn.
4. The experiment turns the valve 90°, not 360°.

The setup is whole-body 4-D L1, Isaac headless at RTF 1, raw mocap at 120 Hz. The arm GS's Valve Turning
tab flies the steps (`--gs-auto`), as on hardware. Each run ends 4 s after the exit
(`pl_mission.py --stop-after-exit --stop-hold 4`), so the exit climb is still recorded.

## What was set up

**The v2 valve, `Valve_Assembly_v2.usda`.**
- Built by `valve_turning_20261015/tools/stl_to_usda_valve.py`, extended to read the 5-shell v2 CAD.
- The two new static half-rings join the base visual. They get no collider: the beam passes 10 mm
  above them, and the claw grips at r 0.160.
- The rotor is v1's, byte for byte.
- Regenerating v1 with the extended converter gives the shipped `Valve_Assembly.usda` unchanged
  (outside the doc string).
- The scene default stays v1. These runs set `PEGASUS_VALVE_USD`.

**The valve's resistance: the bench law, `PEGASUS_VALVE_FRICTION=0.072` (`archive/valve_damper_spin_20261027`).**
- Coulomb, no damping.
- Uses the `drive` stick-slip friction model: cloth on plastic holds the valve where it is left.
- Rotor 204 g.
- The scene default (0.048, tanh) is unchanged.

**The turn: 90° in 12 s for rounds 1–3,** the user's own 10-26 design.
- Peak yaw rate 18.5°/s. The peak angular acceleration is 2.25x the 360° / 36 s turn's.
- Round 4 then adopted **18 s** (see "The hardware preparation").
- `make_valve_turning_yaml.py` now writes `push_pull_turn_deg: 90.0` / `push_pull_push_time_s: 18.0`
  (was 360 / 36). The hardware valve yaml was regenerated, and both hardware stacks expect it.

**From the box push:**

| lesson | carried? |
|---|---|
| the hardware-like drift model (`PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,k`) | yes: seeds 2/3/4 |
| the hand-over settle gate + "Ready only when the claw is still" (10-24) | config **B**: keys added to the generator, OFF by default |
| the room hold from Go To Start (the box rejected it on 10-07) | config **W** (`push_pull_world_anchor_ready: true`) |
| natural inertia along the push axis (10-25) | no: it is fixed to a world axis, and a turn's direction rotates |
| the q3 rate guard, the abort unbuckle | no: measured and rejected for the valve on 10-22 |
| matched SAFETY thrust map, 2 s arm-fold delay (hardware) | not yet in the hardware valve yaml (see "Hardware") |

## Results (`tools/table.py` → `table.txt`)

- **A** = the 10-22 final valve config.
- **B** = A + the settle gate (0.025 / 0.015 / 0.030 m/s / 0.3 s; Ready still 0.5 s at < 0.030 m/s).
- **W** = A + the claw room-held from the end of Go To Start.
- `pushed` = how far the valve turned under the open jaws before the close.
- `along` = where along the 40 mm fin the jaws closed (mm).

```
run              cfg  drift outcome       beam  pushed  along  close  q3 min  stop  tilt
t90A_nom_1       A    --    ok             +86     0.1    1.5    3.3    16.6   0.0   4.2
t90B_nom_1       B    --    ok             +83     0.1    1.9    4.3    10.1   0.0   4.1
t90W_nom_1       W    --    ok             +87     0.0    0.7    3.1    14.6   0.0   3.2
t90A_nom_2       A    --    ok             +86     0.0    5.8    3.2    16.9   0.0   3.0
t90B_nom_2       B    --    ok             +85     0.1    4.3    3.5    19.8   0.0   3.5
t90A_hw_2        A    s2    no close        --     --     --     nan      --    --    --
t90B_hw_2        B    s2    no Ready        --     --     --     nan      --    --    --
t90W_hw_2        W    s2    ok             +89     0.0   11.5    3.2     8.3   0.0   3.5
t90A_hw_3        A    s3    no close        --     --     --     nan      --    --    --
t90B_hw_3        B    s3    no close        --     --     --     nan      --    --    --
t90W_hw_3        W    s3    ok             +88    -0.0    0.4    3.1     6.8   0.0   2.9
t90A_hw_4        A    s4    tilt guard     +53    12.6  -17.5    3.2     8.1   1.0   8.2
t90B_hw_4        B    s4    ok             +84     3.4  -13.0    3.6     8.4   0.0   4.8
t90W_hw_4        W    s4    ok             +81     0.2   13.1    3.1     7.0   0.0   4.6
yw_ref_f048      ref  --    ok             +88     0.0   -1.5    7.3    21.7   0.0   7.2
yw_ref_f048_2    ref  --    ok             +91     0.4   -6.0    7.3    23.8   0.0   5.9
```

| | nominal | drift (seeds 2/3/4) |
|---|---|---|
| A | 2 / 2 | 0 / 3 |
| B | 2 / 2 | 1 / 3 |
| **W** | **1 / 1** | **3 / 3** |

**Every run ended safely:** DIRECT kept, upright, true tilt ≤ 8.2°.

## Why A and B fail under the drift (`tools/descent_push.py`)

**Every failed drift run had the same cause: an open finger met the fin's near edge on the way in,
11–16 mm off radially, and dragged the valve 12–18° along the tangent.**
- The valve starts turning while the claw point is still 27–30 mm behind and 14–35 mm above the
  grasp point. So it is not an overshoot.
- The planner then keeps the claw on the ORIGINAL grasp point, 35–60 mm from the moved fin. The GS's
  close gate is never met, and the run ends in the driver's 20 s timeout.
- A seed 4 (13 mm off radially) still closed, near the fin's end (−17.5 mm). The turn then rode the
  arm onto its stops and tripped the tilt guard at 53°.
- The 10-22 runs on the same seeds fit the same threshold: 11 mm radial → 6.9°, 7 mm → 2.3°, 2 mm → 0°.

**The simulation is harsher than hardware here.**
- The sim claw cannot close below ~19 mm, so the fin carries an invisible 20 mm grip pad
  (`PEGASUS_VALVE_FIN_GRIP_MM`). The open fingertips clear it by only ~8.75 mm a side.
- The real 5 mm fin leaves ~16 mm a side.
- The hardware drift is real, though: 21–36 mm arrival errors on the 10-23/24 box pushes.

**B does not fix it, and its stillness check can block the task.**
- The settle gate acts after the descent, where the damage is already done.
- Under the drift, the planner's (mocap-fed) claw speed never stayed < 30 mm/s for 0.5 s in seed 2's
  hover, so Ready never started.
- Isaac's truth was below 30 mm/s for 42 % of that hover, in runs of up to 1.1 s
  (`tools/hover_speed.py`).
- In seed 4 the check passed only after 8.7 s.

**W fixes it.**
- With the claw held in the room, the arm flies the slide-in. The radial offset stayed ≤ 6.2 mm
  through the descent (4 runs), and the fin moved ≤ 0.2°.
- The free-axis anchor (0.75, latched from the room hold) still lets the claw ride the base along the
  tangent, so the jaws closed 0.4–13 mm along the fin.
- **Cost:** a larger arm motion in the approach: q1/q2/q3 ranges up to 14 / 10 / 10° under drift, vs ≤ 6° for A and B (`tools/approach_arm.py`).
  That is still below the 13–25° that made the box push reject it on 10-07 (a different pose and
  before the free-axis anchor).
- Tilt is unchanged.
- With W the settle gate is skipped (the planner only settles when the room hold starts at the handle).

## The hardware preparation (user, same evening: "go ahead")

**What the experiment flies** (`make_valve_turning_yaml.py`, imported by `make_hw_valve_turning_yaml.py`):
- W: `push_pull_world_anchor_ready: true`.
- 90° in **18 s** (round 4 below).
- The keep-outs placed from the valve (user's choice).
- The box push's way home: Go To Land Start over the start mark at 1.3 m, Land to 0.30 m, fence floor 0.25.

**The keep-outs placed from the valve** (fsc_trajectory_planner, `push_pull_keepouts_from_valve`).
The box push's table-from-box, for a turn:
- **At Plan**, the beam's swept square (±`push_pull_valve_reach` 0.2001) is centred on the captured axis,
  with its top `push_pull_valve_top_above` 0.054 over obj_0.
- **The table** (`push_pull_table_size` 1.000 × 0.150) is centred on the axis. Its depth runs along the
  line from the start, and its top is obj_0's height (obj_0's pivot is on the axis at the plate bottom,
  which is the table top).
- `push_pull_table` / `push_pull_obstacle` are ignored; every later step checks the plan's own keep-outs.
- **Plan refuses** a start outside `push_pull_valve_start_range` [0.6, 3.0] m, and a beam (obj_0 +x)
  more than `push_pull_valve_beam_tol_deg` 45° from the start. That is the layout rule, and it catches
  a mocap body defined turned.
- **Adjust** takes the start wherever the vehicle is.
- **Tests:** gtest `PushPull.KeepoutsPlacedFromTheValve` (the scene's typed keep-outs reproduced to
  1e-9, keep-outs that follow a moved and turned valve, all four refusals). Push-pull 28/28,
  pick-place 31/31.
- **In Isaac** the planner printed axis (1.200, 0.000, 0.800), table top 0.800, beam top 0.854, the
  scene's.
- **One imprecision:** the table's long axis is taken square to the start-to-valve line. The sim start
  sits 7 cm off that line, so the placed table came out turned 3.6° (32 mm at its ends, inside the
  planner's 50 mm footprint margin). On hardware, start the vehicle within ~10 cm of the table's
  middle line.

**The hardware yaml** (`make_hw_valve_turning_yaml.py`; diff = exactly these):
- Planner:
  - `push_pull_gear_depth` 0.248 (measured).
  - `push_pull_abort_mocap_release_wait_s` 1.0: the real jaws need ~0.7 s after two WiFi hops;
    Isaac uses 0.7.
  - `push_pull_fence_max_z` 1.8.
- Node: the box push's margins.
  - `system_arm_go_home_delay_s` 2.0.
  - The SAFETY thrust map matched to the flown kf, 0.037947 / 0.252449.
- Both whole-body hardware stacks:
  - expect all of it (35 valve-specific values, all OK on the generated file);
  - print the layout rules;
  - refuse a planner build without `push_pull_keepouts_from_valve`.
- The fused stack still refuses the valve profile.

**obj_0's pivot.** The 10-27 bench bag had it on a 73 mm circle (Motive's marker centroid), the plate
tilted 2.6°, and the pivot 31 mm above the table top. **Superseded 2026-10-28** (section "The mocap
markers and obj_0"): the pivot stays at the centroid and the planner carries the axis offset; the
"31 mm" assumed a 0.80 m table.
- `tools/valve_body_check.py` checks a definition while the beam is turned by hand: pivot on the axis
  (< 3 mm), axis tilt, pivot height vs `--table-top`, obj_0 z vs the shaft.
- It fails that bag (exit 1), as it should, and passes a synthetic correct body.

**Round 3 (`tools/chain3.sh`, the config as generated, 90° / 12 s):**
- 2 full missions: complete, with the landing exactly as planned (Land Start at 1.30 m and Land at
  0.30 m over the start mark).
  - hwp_full_2's turn was rough: the valve overshot to 105°, the lowest q3 was 4.6°, and the arm sat on
    a joint stop 13.5 % of the turn.
- Drift seeds 2/3/4: 0 of 3 completed. Each was a q3-guard abort, and each ended safe (DIRECT kept,
  tilt ≤ 9.4°).
  - hwp_hw_2 buckled to q3 −65.7° before the abort recovered.
  - **Mechanism** (`tools/turn_trace.py`, `tools/turn_start.py`): during the clamped settle and the
    turn's acceleration, the vehicle lags. The free-axis anchor carries 75 % of that lag onto the fin,
    so the valve slips 7–14° backwards and the arm compresses.
  - The W runs on the same seeds slipped as much and survived. Same mechanism; the margin is thin.
  - The 10-22 360° / 36 s runs slipped 7.5–10.6° but bottomed at q3 11–14°.

**Round 4 (`tools/chain4.sh`, 90° over 18 s, the 360° / 36 s turn's peak angular acceleration): 4 of 4
under drift.**

| | 12 s, drift | **18 s, drift** |
|---|---|---|
| completed | 3 of 6 | **4 of 4** |
| lowest q3 over the turn | −65.7..8.3° | 4.4..7.8° |
| valve slip back at the start | 6.6–14.1° | 3.4–10.3° |
| beam / tilt (completed) | 81–89° / 2.9–4.6° | 81–95° / 3.5–6.2° |

18 s is in both yamls and the stacks. **The q3 margin stays thin** (4.4° against the 3° guard), and
the slip at the start remains.

## The mocap markers and obj_0 (2026-10-28, user request)

The user added five markers to `Valve_Assembly_v2.stl` and will place them that way in the
experiment. obj_0 is the rigid body Motive makes from them; the planner takes its pose and adds
offsets. Asked: check and modify the arm GS and the localization logic, as for the box push.

**The markers** (`stl_to_usda_valve.py` now reads the 14-shell CAD):
- Five 30 mm balls on stems, on four printed mounts plus one mount fused to the rotor shell. All five
  are on the ROTOR, so they move rigidly with it.
- Ball centres from the axis on the plate bottom, rotor frame [mm]: (4.9, 15.0, 92.5),
  (40.1, −19.8, 125.5), (85.1, −42.7, 68.5), (104.8, 43.4, 70.5), (125.1, −42.5, 68.5).
- **Their centroid, Motive's default pivot: (72.0, −9.3, 85.1) mm, 72.6 mm off the shaft, 85 mm up.**
- Pairwise distances 40.0 / 59.5 / 76.2 / 88.3 / 88.3 / 101.7 / 104.9 / 105.9 / 106.1 / 135.4 mm.
  Two are equal (the C–D–E triangle is isosceles). Motive solves all five together, so this is not
  ambiguous on its own, but watch for 180° flips when the claw is near.

**Why the planner had to change.** It placed the turn's centre, the live handle and the keep-outs
at obj_0 itself. With the pivot at the centroid, a 90° turn about obj_0 ends the claw
√2 × 72.6 = 103 mm off the fin's circle, and the table top is placed 85 mm high.

**The change (fsc_trajectory_planner):** new `push_pull_valve_axis_offset` [x, y, z].
- It is the vector from obj_0 to the bearing axis on the plate bottom, in obj_0's frame, turned with
  obj_0's yaw (like the EE offset).
- The axis point = obj_0 + Rz(yaw) × offset. The turn, the live handle, the start-range check and the
  keep-outs placed from the valve all start from it.
- **In a turn the EE offset is measured from the axis point**, so it keeps its numbers
  [0.160, 0.0006, 0.179]. Only the axis offset depends on the markers, and a re-measured axis offset
  needs nothing else changed.
- Default [0, 0, 0] = obj_0 on the axis (unchanged). A non-zero offset with a straight push is refused.
- Both valve yamls carry [−0.0720, 0.0093, −0.0851]. The Plan log prints the axis and the offset.
- gtest `PushPull.TheValveAxisOffsetPutsTheTurnOnTheShaft`: at two beam yaws the plan equals the
  obj_0-on-the-axis plan to 1e-12, and without the offset the claw ends > 9 cm off. Push-pull 29/29.

**The arm GS (Valve Turning tab):** an **Axis Offset** row under the EE Offset, bound to the new
parameter; its own request, so an older planner leaves it read-only. The Box Pushing tab is unchanged.
`tools/gs_axis_check.sh` (real planner, valve yaml, domain 77): 9/9 (loads, edit + button writes,
an outside set shows up, the box tab hides the row). `gs_axis_0_loaded.png`. The existing valve, valve
auto-step and box harnesses still PASS.

**The simulation now publishes obj_0 the way Motive will** (`PEGASUS_VALVE_MOCAP_PIVOT=markers`, the
default with an asset that has markers): at the centroid, with the rotor's axes.
- **The scene's default asset is now `Valve_Assembly_v2.usda`** (pad + markers; the balls are sphere
  colliders), and its bearing defaults follow the asset: v2 0.072 N·m in the drive model (the 10-27
  bench), v1 0.048 tanh as before.
- **The layout is the box push's** (the 10-21 reason: the table at x 1.20 sat near the field's edge):
  table centred on (0.675, 0), x 0.600–0.750; the valve axis there; start mark (−1.0, 0); the vehicle
  spawned at (−0.90, −0.07); Land Start / Land over the start mark. Start to axis 1.675 m.
- Checked in Isaac (no flight): obj_0 reads (0.603, 0.009, 0.885) = the axis + Rz(180°) × centroid,
  and after Get the planner's table is centred on (0.675, 0.000), top 0.800 — the scene's table.

**`tools/valve_body_check.py` measures the definition** from a hand turn, in obj_0's yaw frame, the way
the planner reads it:
- the axis offset (x, y; z is the CAD's — a turn cannot see it) against `--axis-offset` (default the
  CAD's): within 3 mm OK, else it prints the value to type into the Axis Offset row;
- the frame's turn about z against the beam (from the offset's direction): within 3° OK, else
  re-create the rigid body with the fin along the room's +x;
- axis tilt, obj_0 z vs the shaft, the axis point's height vs `--table-top`.
- Synthetic bodies: a 0°, 2° and −15° frame turn are recovered exactly.

**What the bench bags say (run through it):**
- **10-27: obj_0's +x points −89.5° from the beam, and turned back the centroid sits 0.4 mm from the
  CAD's.** That bench already had this marker layout; only the frame was off.
- 10-23: −87.6°, and 4.7 mm off once turned back (a different layout).
- So re-create the rigid body with the fin along the room's +x, keep the default pivot, and check it.

**Clearance to the markers** (not in the keep-out): the planner's valve keep-out is the beam
(top 54 mm). The tallest marker (40.1, −19.8) mm tops out at 140.5 mm above the plate bottom; at the
grasp the skids are 187–193 mm up, so ≥ 46 mm above it even if overlapping. The others top out at
84–108 mm. The fingers stay ≥ 17 mm beside and ~75 mm above the nearest ball. Raising
`push_pull_valve_top_above` to the marker top would refuse the plan (the claw itself goes below it,
as it goes below the fin top on purpose), so it was left at the beam.

**Flown: one full mission, nominal, windowed, the arm GS running Ready / Turn / Exit
(`runs/mk_full_1`, try_valve.sh with the 10-27 gs() settings, no drift) — COMPLETE:**
- Get captured obj_0 at (0.603, 0.009, 0.885); Adjust shifted the points by (0.101, −0.070).
- **The axis from obj_0 + the offset sat at (0.6750, 0.0000, 0.8000) and moved 0.0 mm through the turn,
  while obj_0 itself (the centroid) moved 104.6 mm.** Claw radius about the axis: planned 0.1600 m,
  measured 0.1608 ± 0.0003 m.
- Jaws closed 4.6 mm along the fin, 1.2 mm off its centre line, 0.10 s after Ready, the rotor not
  turned under the open jaws.
- Beam +85.7° in the 19.5 s turn step (+86.5° over the run); worst base lead 27 mm; q2 −3.5..12.2°,
  q3 16.3..36.6° (stops −20 / 50, the q3 guard 3°); 0 % of the turn on a stop; tilt ≤ 4.9°; |τ| peak
  0.93 N·m; contact (truth) 1.23 N mean, 0.75 N tangential.
- Then the way home: Go To Land Start 10.2 s, Land 7.4 s, disarmed.
- The 10-27 W nominal row for comparison (v2, the old layout): beam +87, q3 min 14.6, tilt 3.2.
- `valve_score.py` now reads the run's `push_pull_valve_axis_offset` (or `--axis-offset`); older runs
  score as before. `grasp_offset.py` (10-22) still assumes obj_0 on the axis: add the centroid
  (72.0, −9.3) mm to its along / radial numbers for runs from today on.

**Not done:** the coordinated-admittance valve yaml was already stale (360° / 36 s) and is still not
regenerated; with the new scene defaults that rig needs regenerating (or
`PEGASUS_VALVE_MOCAP_PIVOT=axis` + the old layout) before it flies again. One flight, nominal only (no
drift seed). Hardware: rebuild fsc_trajectory_planner (Orin) and utils_custom_ground_station (laptop).

## Review before the hardware flight (2026-10-28, user: "check the code and logic ... make sure its hardware implementation is also correct")

Three independent read-only reviews (planner turn logic; the hardware config and stacks; the arm GS
Valve Turning tab), then fixes. Findings and what was done:

| # | finding | fix |
|---|---|---|
| 1 | **Rotor-tilt guard false trip (hardware only).** The guard compares obj_0's z now with its z at the grasp (2° limit). Motive gives a new body the room's vertical; a body whose z is γ off the shaft reads 2·asin(sin γ·sin(θ/2)) of false tilt after a turn θ. On the 10-27 bench (γ 2.2°) it would abort ~54° into the 90° turn, clamped. Isaac's rotor is on an exact vertical joint, so no sim run could show it. | **Procedure:** level the plate when CREATING the rigid body (afterwards a tilted table does not matter: body and shaft tilt together). `valve_body_check.py` now fails a body whose predicted false tilt after the turn + 15° exceeds 0.75 × the limit (γ ≤ ~0.95° for 90° / 2°). Both stacks print the rule. |
| 2 | **The Valve tab's close gate was Isaac's on hardware** (20 mm, 3 mm across, no speed gate, no wait limit): 3 mm is sized for the sim's 20 mm grip shim; a never-met gate held the open claw beside the fin indefinitely. | New `push_pull_valve_grip_*`: 20 mm, **8 mm across on hardware** (3 mm in Isaac), 30 mm/s over 0.3 s, **2.5 s then ABORT** (the box tab's 10-24 hardware gate). |
| 3 | Plan sent the typed Turn but not other amber values (an unsent Axis Offset = the turn 73 mm off the shaft). | The Valve tab's Plan refuses while EE Offset / margins / Axis Offset are amber. |
| 4 | The pre-Plan EE preview ignored the axis offset (73 mm off). | Fixed. |
| 5 | Open-on-DIRECT-loss could miss a clamped claw (status and info race). | Also keyed on the GS's own grip state. |
| 6 | The Box tab could not clear a valve axis offset, and the planner refused a straight push with it. | The planner now IGNORES the offset for a straight push. |
| 7 | A NaN offset passed every keep-out check; a bad `push_pull_valve_start_range` blocked every later step. | Refused at the parameter set (finite; [min, max] with 0 ≤ min < max; beam tol ≥ 0). |
| 8 | Staged writes cleared amber with the box value, not the sent one; unedited boxes sent their rounding (0.0006 → 0.001); a late axis reply could overwrite a newer event. | Fixed (sent values, the planner's own value for an unedited box, an event counter). |
| 9 | The gtest checked waypoints only. | Now: three beam headings, the turn leg's claw on the fin's circle (≤ 0.2 mm, the acceleration tilt), the whole mission dry-run from the start mark at 1.3 m, the no-offset case (grasp and circle 72.6 mm off), ignore-for-a-push, NaN refused. |
| — | Get's reply in a turn now names the axis point (obj_0 + offset), to check by eye. Lamp texts and the Valve row tooltip corrected. | |

**Checked and fine:** every key of the hardware valve yaml is declared and loaded with the yaml's value
(runtime: the real planner and node started on it, domain 77; 152 + 154 keys); the whole-body node's
`members()` lists agree; the hardware yaml differs from the sim one only by the intended lab values;
the raw stack selects the right yaml, starts the estimator's mocap guard, checks 61 tuned values and
refuses a planner without the axis offset; the fused stack refuses the valve profile.

**Tests after the fixes:** planner push-pull 29/29, pick-place 31/31, box loopback ALL PASS; all nine
push-and-pull GS harnesses PASS (valve, valve auto-steps, axis row, box, grasp, release, push,
monitor, mocap); hardware stack dry run 61/61 + the negative build test.

**Confirmation flight after the fixes (`runs/mk_full_2`, headless, GS-run steps):** the new close gate
closed 0.15 s after Ready; the turn ran on the true axis (fixed to 0.0 mm, claw radius 0.1604 m); the
valve lagged its plan (82° at the end); after the jaws opened the room-held claw pushed it on to 90°,
then in the Exit climb a finger caught the fin (q4 25 → 51°, beam +10°) and the 10° tilt guard aborted
(11.6°); the abort climbed clean and the vehicle landed. The known release-window weakness, not the
fixes (1 of 2 complete today; 10-18 final config 6 of 9).

**Pushed 2026-10-28:** fsc_trajectory_planner main `66bc2fa`, fsc_autopilot_ros2 dev_robotic_arm
`bf7f14d`, fsc_open_manipulator omx-torque-control `cb6cf54`, Pegasus dev_robotic_arm `c5dbe71`
(this README's earlier state; the confirmation flight and these lines are after it).

**Known and NOT fixed (operator rules instead):**
- On hardware a full close on the 5 mm fin (−0.009 m) and an empty close (−0.010 m) differ by ~1 mm,
  so the GS cannot tell a grip from a miss by position; the position gate before the close is the
  protection.
- Landing yaws are absolute (0 = world +x): lay the start → valve line along the room's +x, or Go To
  Land Start adds a turn.
- Only the valve may be named `obj_0` in Motive (the box body from 10-27 must be disabled).
- The q3 guard (3°) margin is thin under drift (4.4–7.8° in Isaac); expect a q3 abort rather than a
  crash on a sticky real arm.
- The coordinated-admittance valve rig (sim only) is stale (no axis offset).

## Not covered

- One or two flights per seed. 18 s was flown on drift seeds only; its nominal and landing legs are
  12 s round 3's (the same paths).
- Not built, with what they would address:
  - a shorter clamped settle, or the free-axis anchor ramped in only after the turn starts: the slip
    at the turn's start;
  - a q3-rate term in the valve's guard (rejected on 10-22): the late trip seen in hwp_hw_2.
- The beam stops 1–9° short of, or runs up to 15° past, 90°: the free-axis anchor lets the valve move
  with the base.
- The breakaway friction is the kinetic 0.072 here; the real cloth may hold more at rest.
- The coordinated-admittance rig's valve yaml was not regenerated (comparison rig, still the 10-22
  planner block).

## Files

- `tools/chain.sh`: round 1, A and B; the first 360° attempt is in `runs/stopped_360/`.
- `tools/chain2.sh`: round 2, W.
- `tools/chain3.sh`: round 3, the hardware-prep config as generated.
- `tools/chain4.sh`: round 4, 18 s.
- `tools/turn_trace.py`, `tools/turn_start.py`: the turn's arm margin and the valve's slip.
- `tools/valve_body_check.py`: obj_0's definition check (axis offset + frame turn since 2026-10-28),
  live or on an extracted bag.
- `tools/gs_axis_check.sh` + `tools/gs_axis_harness/`: the arm GS's Axis Offset row (2026-10-28).
- `tools/table.py`, `tools/score.py`: the 10-22 scorer on these runs, `--ref` = the 10-26 90° runs.
- `tools/descent_push.py`, `tools/hover_speed.py`, `tools/approach_arm.py`.
- `runs/`: npz (gitignored), logs, as-flown yaml, args.
