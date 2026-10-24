# 2026-10-24 box pushing on hardware: three flights, 240 g, the new guards and close gate

Bags: `docs/experimental_data_ros2_bag/1024 - T650-AM whole-body Box-Pushing-.../`.
`flight_wb_l1_4d_box_pushing_240g_20261024_` + `150423` (#1), `151159` (#2), `151634` (#3), flown in that
order on one battery. First flights with the 2026-10-24 changes
(`../wb_box_pushing_flight_20261023` sections 9–11): the q2-aware q3 rate guard and the arm GS's
close gate (25 mm / 8 mm across / 30 mm/s, ABORT after 2.5 s).

## What happened

| flight | outcome | cause found |
|---|---|---|
| #1 | gripped and pushed ~70 mm, then box-tilt guard abort (box 6.3°) | the fin was **35 mm** across from where the planner aimed; closing the jaws dragged the box sideways |
| #2 | no grasp; q3 rate guard abort 1.2 s after Ready | the claw arrived ~47 mm beside the real fin, 20–43 mm short along it; the arm stretched in the room hold |
| #3 | box BLOWN OFF the table during Go To Start; aborted 1.5 s into Ready | the downwash, with the vehicle 0.46 m above and 0.36 m behind the box |

## 1. The fin was not where the planner aimed (`fin_offset.txt`)

While the jaws hold the fin, the claw is on the fin, so the planner's FK claw minus (obj_0 origin +
`push_pull_ee_offset` in the box frame) measures where the fin really is.

| push | across | along | up |
|---|---|---|---|
| 10-21 | −5.5 mm | +4.9 | −3.9 |
| 10-23 | −1.5 mm | −0.7 | −4.5 |
| **10-24 #1** | **−34.8 mm** | +2.3 | −8.4 |

- The FK model error is a few mm (10-21, 10-23).
- **Which way.** Across = box x = the jaws' closing axis; the box sat at yaw −0.1°, so box x = room
  x. The drone took off at x −1.47 and hovered behind the box at heading −91.5° (nose −y, the push
  direction). The real fin was 34.8 mm toward room −x of the aimed point: the near side of the table,
  toward the take-off spot, the drone's right while pushing. The user re-placed the box's markers
  before 10-24 (confirmed), so the mocap origin moved 35 mm toward the far side (+x) of the fin.
- **Only the across offset is measured.** The fin is 5 mm thick but 60 mm wide along the push and
  tall, so the grip does not show the origin's along / vertical shift. Height: obj_0 z 0.8498 on 10-24
  vs 0.8508 on 10-21 (unchanged); 10-23's 0.8704 had the table elsewhere (box at x 1.31), so the
  height says nothing about the markers. (An earlier version of this README read it as proof that
  the body was re-created; that was wrong.)
- So Get captured a grasp point 35 mm beside the fin, and every step aimed there.
- **The STL's marker layout** (`Box_Assembly_Push.stl`, five markers on posts;
  `tools/marker_centroid.py`): the five ball centres' centroid is, in the simulation's box axes,
  −10 mm across (fin = centroid + 10 mm), +20 mm along and +29.7 mm up from the box centre. Motive puts
  a new body's pivot at that centroid. The planner's EE offset (0, 0, 0.183) is from the box centre,
  so markers placed exactly like the STL, with the pivot left at the centroid, would aim 10 mm beside
  the fin and 5 mm above its top. Matching the STL makes the layout repeatable; the pivot (or the EE
  offset) still has to be set to it. The 10-24 body was not that case (its origin sat at box-centre
  height).

## 2. Flight #1 step by step (`window_stats.txt`, `grasp_chain_150423.txt`)

- **The open-jaw hold (47.52–49.02 s) was the calmest of all three hardware pushes.** It lasted
  1.5 s (10-23: 2.8 s). CoM error across 4 mm rms (10-23: 34), the arm still. The new gate closed
  1.49 s after Ready with the claw 14 mm from the AIMED point, 7.5 mm across it, at 13 mm/s. The old
  3 mm gate would have kept waiting.
- **But the claw sat 24–26 mm from the real fin across, inside the ~25 mm jaw clearance with the
  near pad next to the fin.** Closing (49.02–49.65 s), the pads pulled the claw onto the fin and
  dragged the box **38 mm sideways**. The claw ends 1–3 mm from the fin.
- **The base was already drifting sideways** (+x, from ~48.85 s, 20–40 mm/s). The room hold keeps
  the CLAW still and the arm absorbs the base's motion (the wrist rolled 4 → 10°), so the gate's
  claw speed read 13 mm/s while the base moved. The drift continued to +108 mm by 50.1 s.
- **That led to the abort.** Roll attitude error up to 136 mrad, single-sample rotor saturation
  (the counter-clockwise pair at 0.00 at 49.05 and 49.60 s), the clamped box dragged sideways and
  tilted, and the box-tilt guard at 52.41 s (6.3° > 2°), 1.26 s into the slide.
- **The q3 guards never fired in #1** (q3 ≥ 14.2°), and the abort was clean.

## 3. Flight #2: missed the fin

- The claw arrived ~12 mm from the aimed point, i.e. ~47 mm from the real fin, outside the jaws.
  Along the push it drifted from 6 to 43 mm short during the hold.
- The room hold stretched the arm: q2 39° rising at 30°/s, q3 15.3° falling at 47°/s. The rate
  guard aborted 1.2 s after Ready.
- q2 was 6° below its stop, so the q2 credit (0.1°) correctly did not apply. A real fast fold,
  caught early.

## 4. Flight #3: the box was blown off before the descent

- At the end of Go To Start (43.4–44.2 s) the box slid 15 → 46 → 122 mm, flipped and fell to the
  floor (z 0.11 m). The vehicle hovered 0.46 m above it and 0.36 m behind it, at the same place as
  in #1 and #2.
- **Nothing noticed.** The GS started Ready 2 s later toward the captured grasp point, and the
  operator aborted at 47.89 s.
- The box guards (tilt, lag, stale feed, sample gate) act only with the claw on the box or during
  the push. Nothing checks, before Ready or before the close, that the box is still where Get
  captured it.

## 5. Battery and the yaw budget (`power_budget.txt`, `yaw_balance.txt`)

- **Battery.** #1 started at 24.76 V, #2 at 23.72 V, #3 at 23.49 V. The mean rotor command in the
  hover rose 0.577 → 0.597 → 0.616 (+7 %). #1 was not limited by the battery.
- **The standing yaw imbalance is unchanged on every flight.** The clockwise pair runs +0.20 above
  the counter-clockwise pair in the hover (10-21, 10-23 and 10-24 alike): the loose front-left arm
  noted on 10-14. It leaves the counter-clockwise pair ~0.48 from zero, so a hard transient
  saturates yaw first. In #1's hold and push 5.7 % of samples had a rotor at a limit (earlier
  pushes: 0–0.6 %).

## 6. Verdict on the 2026-10-24 changes

- **Valid.** Every rule behaved as designed:
  - the gate opened fast and calm;
  - the q2-aware q3 guard was silent where it should be (#1) and tripped on a real fast fold (#2);
  - the abort paths were clean.
- **Helpful, but not where #1 failed.** The calm, short hold is a clear improvement over 10-23.
  #1 failed on the 35 mm aim error, which no gate can fix: the gate measures the claw against the
  AIMED point.
- **One weakness the flight exposed.** The gate's speed check looks at the claw, which the room
  hold keeps still. The base was drifting at 20–40 mm/s when it opened. A base-speed check would
  have held the close ~0.5 s. Its strictness is not yet checked on the hardware holds.

## Recommendations (not applied)

1. **Fix the aim, before anything else.**
   - Re-define obj_0 consistently: its pivot at the box centre under the fin, as before.
   - Or, for the present markers, set the GS's EE Offset x to −0.035 m (box frame; the sign follows
     from the measurement: fin = origin − 34.8 mm along box x). This corrects only the across
     offset.
   - With markers exactly as in the STL and the pivot left at their centroid: EE Offset
     (+0.010, −0.020, 0.153) in the simulation's box axes (signs depend on how the body's axes are
     set in Motive).
   - Verify before each session: put the claw on the fin by hand (vehicle on the ground or held)
     and compare the planner's FK claw with Get's grasp point. They must agree within ~5 mm.
2. **A box-moved check:** refuse Ready, and the close, if the live box moved more than ~10 mm or
   tilted more than ~2° since Get. That would have stopped #3's descent toward an empty table.
3. **The downwash:** #3's box was blown off at the same hover as #1/#2. Either a heavier box (the
   500 g target) or a higher / further hover point before Ready.
4. **Gate:** add the base's speed (or the CoM error rate) to the close gate; check its strictness on
   the three holds first.
5. **Battery:** one push per fresh pack, or check ≥ 24 V before Go To Start.

## Tools

```bash
bash tools/run_extract.sh                                   # the three bags -> npz/ (the 10-21 extractor)
cd tools
/usr/bin/python3 timeline.py 150423 > ../timeline_150423.txt  # also 151159, 151634
/usr/bin/python3 window_stats.py > ../window_stats.txt        # per-phase motion, 10-24 #1 vs 10-23 vs 10-21
/usr/bin/python3 grasp_chain.py 47.4 53.0 0.15 150423 > ../grasp_chain_150423.txt
/usr/bin/python3 fin_offset.py > ../fin_offset.txt            # where the fin really is (gripped claw vs obj_0 + EE offset)
/usr/bin/python3 power_budget.py > ../power_budget.txt        # battery, rotor commands per phase
/usr/bin/python3 yaw_balance.py > ../yaw_balance.txt          # the standing yaw imbalance, every push
```
`bp_common.py` loads this folder's npz, then the 10-23 and 10-21 folders'.
`stl_parts.py <stl>` lists an STL's closed parts; `marker_centroid.py` fits the five marker balls of
`Box_Assembly_Push.stl` (ball centres from the post axis, ±1 mm) and prints their centroid against the
box centre and the fin.

Report page (artifact "1024 Box-Pushing Flights", https://claude.ai/artifact/8128dFtRPArxQLSD1xpzJb):
`cd tools && /usr/bin/python3 report_data.py > ../report_data.json && /usr/bin/python3 build_report.py`.

## 7. Afternoon flights A (162850), B (163413), C (164419): the aim is fixed, the hand-over is not

`arrival_table.txt`, `ee_used.txt`, `pitch_offset_fit.txt`, `grasp_window_<tag>.txt`, `timeline_<tag>.txt`.

| flight | outcome |
|---|---|
| A 16:28 | gripped, pushed 154 of 150 mm; q3 rate guard 0.7 s after the push (release window) |
| B 16:34 | no grasp; q3 rate guard 1.1 s after Ready |
| C 16:44 | no grasp; q3 rate guard 0.6 s after Ready |

- **Aim fixed.** EE Offset (0, 0, 0.183) in all three; with the jaws on the fin (A) the claw sat
  −1.1 mm across from obj_0 (morning −35 to −38). The re-defined box body works.
- **Every trip was real.** At the trip q3 17–18° falling 39–67°/s, q2 37–40° (no q2 credit);
  q3 then bottomed at 7.1 (A), 9.4 (B), 11.9° (C) with q2 at 41–45° (morning #2: 6.2°). The arm was
  being pulled into q2's wall. Do not loosen the guard; remove the pull.
- **The pull = the grasp hand-over.** At Ready's DONE edge the planner switches the claw to the room
  hold (`push_pull_world_anchor_on_handle`), blended over `wb_ee_anchor_blend_s` 1.0 s, while the
  free-axis weight ramps in over the same second, so the arm must close most of the along-push gap
  itself for ~1 s. Ready ended 29 / −28 / 35 mm short (A / B / C; B had overshot and was swinging
  back 65 mm in 1 s). The morning's #1 had the same +36 mm and its arm did not move: its base happened
  to be moving forward at the hand-over (luck of phase, same software).
- **"The gripper misses the fin" = it never closes.** The close gate (25 mm / 8 mm / 30 mm/s) was never
  met in B and C (finger stayed +19 mm). The fin is 60 mm long, so 28–36 mm short puts the claw at its
  back end; the gate was right to wait.
- **Why the base is short: its pitch error.** CoM error along the push vs attitude error about model x
  ([28]), Go To Start to Ready's end: slope 0.56 / 0.71 / 0.65 mm/mrad (r 0.84 / 0.95 / 0.90) against
  u1/k_x = 0.71–0.75 -- the structural T·e_R/k_x of a pure-P attitude loop. Pitch errors of 60–95 mrad:
  at Go To Start's end (the arm finishes unfolding, the vehicle stops; base overshoots 45–68 mm) and
  in Ready's 150 mm descent (base 30–45 mm behind). The deadbeat rotational estimate ([65]) moves at
  once by 0.15–0.25 N·m; the filtered one the law uses ([34]) follows 1.5–2 s later (ω_c,r 0.74). Source
  of the moment change not in the bag. Ready also starts only 1.6–2.2 s after Go To Start, still 15–49 mm
  past the point. Go To Start overshoots in every flight since 10-23 (−35 to −51 mm).
- **A's push: stiction, then a lurch.** q2/q3 at exactly zero velocity 48.0–51.5 s while the law raised
  j2 0.89 → 1.33 N·m; the arm-side friction FF (measured-velocity relay) gave ~0. The vehicle drifted
  27 mm back and dragged the box 22 mm back; the base fell 82 mm behind its reference. The 4-D observer
  booked the stuck joints as a 4.6 N backward contact force (the 240 g box slides at ~0.6 N): the force
  guard reached 4.7 of 5 N; the box-lag guard read 22 of 40 mm (its free-axis correction excused ~60 mm).
  The joints broke loose at ~60°/s and threw the box 190 mm at up to 240 mm/s (plan 21). The release
  (auto Exit 0.24 s after the push) came while the base moved back ~100 mm/s; the jaws (~0.5 s to open)
  were still closed and the arm stretched: q3 26 → 7°.
- Battery fine: 24.86 / 24.30 / 24.68 V at start, hover 0.577 / 0.595 / 0.574.

### Next steps (proposed; 1-3 APPLIED the same evening, section 8)
1. Planner: settle before the room hold -- keep the CoM anchor after Ready and hand over only when the
   claw is ≤ ~15 mm (≤ 8 across) and < 30 mm/s for 0.5 s; wait ≤ ~6 s, else stay and say so. Start the
   GS's 2.5 s close-gate timer at that hand-over.
2. GS: start the automatic Ready on the claw's error / speed, not 1.0 s after Go To Start (stop-gap:
   `push_pull_auto_ready_delay_s` ~3 s).
3. GS: start the automatic Exit when the base is calm (speed, CoM error), not 0.2 s after the push.
4. The push's stick-slip (stuck arm, guards excused it): Isaac first, with the drift model and joint
   stiction.
5. Reduce the moment change: unfold the arm before Go To Start's final straight; a 10 s hover at the
   Ready-end pose, jaws open, to see whether the moment stays (near-table effect) or decays.

Tools: `ee_used.py` (EE offset in use + fin offset from a gripped push), `grasp_window.py <tag> <T0> <t0>
<t1>` (sample-by-sample grasp window), `arrival_table.py` (every push: overshoot / shortfall / q3 reaction),
`pitch_offset_fit.py` (along error vs pitch error), `report_data_pm.py` (Figures 2 and 3).
`run_extract.sh [tags]` now covers all six bags.

## 8. Applied 2026-10-24 evening: next steps 1-3 (uncommitted, not flown)

Steps 1-3 above, as built. Code checks only (no Isaac flight, as asked).

**1. Planner: the room hold waits for a settled claw** (fsc_trajectory_planner node, default off):
- `push_pull_handle_settle_tol` 0.025, `_axis_tol` 0.015 (0.008 until the Isaac check, `../box_pushing_settle_gate_20261024`), `_max_speed` 0.030, `_dwell_s` 0.3. With
  `push_pull_world_anchor_on_handle`, the claw stays on the CoM anchor after Ready ends. It moves to the
  room hold only once it has been within 25 mm of the grasp point (15 mm across) and slower than
  30 mm/s for 0.3 s. Speed is taken over 0.3 s on the arrival tick.
- The proposal said ~15 mm. 25 mm is the GS close gate, so the planner never waits on a claw the GS
  would already close on. The room hold then pulls in at most 25 mm, from rest. A/B/C had 28-36 mm
  plus a swing.
- No time limit in the planner; the GS has one (below). The status reads
  `DONE ready next=push (settling: ...)`. Info appends [87] = settling and [88] = seconds waited
  (89 values).
- A step, an abort or a mode change ends the settle.

**2. Planner: Ready waits for a still claw** (`push_pull_ready_max_speed` 0.030,
`push_pull_ready_settle_s` 0.5). Ready is refused unless, at Go To Start's hold, the claw has been
slower than 30 mm/s for 0.5 s. The refusal says "wait for it to settle", which the GS's automatic
Ready already retries every 0.5 s for 10 s. A/B/C pressed Ready 1.6-2.2 s after Go To Start, still
swinging. On the 10-21/23/24 holds the claw met this test 79 % of the time (median 12 mm/s).

**3. Arm GS (Box Pushing tab only):**
- The close gate stays shut while info [87] = 1. Its 2.5 s limit starts at the hand-over.
- `push_pull_box_settle_wait_max_s` 6 s: no hand-over by then → ABORT with the jaws open.
- An older planner (no [87]) behaves as before.
- The automatic Exit also waits for a calm vehicle: `current_base` slower than
  `push_pull_auto_exit_max_speed` 40 mm/s over 0.3 s, held `push_pull_auto_exit_calm_s` 0.3 s.
  After `push_pull_auto_exit_calm_max_s` 3 s it exits anyway (logged). With no fresh `current_base`
  it exits at once, as before. A released at 0.24 s with the base moving back ~100 mm/s.
- Both rules are printed in the station's startup log (the close-gate line ends "... no hand-over
  within 6.0 s -> ABORT"; the step line says Exit To Push starts "after the push (+200 ms), once the
  vehicle is slower than 40 mm/s for 0.3 s (at most 3.0 s)").

**Configs:**
- The six planner keys are in `make_push_pull_yaml.py`'s planner block. The sim, hardware and
  coordinated-admittance box yamls were regenerated (each diff = the six keys). The valve yaml is
  unchanged (its own block; the keys are off).
- Both whole-body hardware stacks expect the six values. They REFUSE a planner build without
  `push_pull_handle_settle_tol`.
- The Isaac scene's planner check compares them.

**Checks:**
- Planner loopback ALL PASS on the sim and hardware box yamls. New: [5r] Ready refused while the
  claw swings (53-66 mm/s), still refused 0.15 s after it stops, accepted once still; [5h] no room
  hold while 0.19 m off or swinging, handed over 0.50 / 0.70 s (sim / hardware yaml) after the claw
  went still.
- Box-point loopback PASS; planner gtests 27/27 + 31/31.
- GS harness (`../box_pushing_hardware_20261019/tools/gs_grasp_check.sh`), new cases:
  - P1: no close and no abort for 3 s while settling, closed and pushed after the hand-over.
  - P2: the settle limit aborts.
  - P3: a DONE ready status that arrives before the settling info does not start the 2.5 s limit.
  - K1: no exit while the vehicle moves 100 mm/s; exit 0.3 s after it is calm.
  - K2: never calm → exit after 3 s.
  - K3: a calm vehicle exits at once.
- The release, box, valve-auto, push, monitor and mocap panel harnesses all pass.
- Hardware stack dry runs: raw 60/60 tuned values; fused 58/58 with `ALLOW_FUSED_CONTACT=1` (it
  still refuses box pushing by default). A planner binary with the settle string removed is refused.

**To fly it:** Orin: rebuild fsc_trajectory_planner, pull fsc_autopilot_ros2 (yaml + scripts).
Laptop: rebuild utils_custom_ground_station. What to watch: the time from Ready's end to the
hand-over (info [88]; planner log "the claw settled ... now WORLD-held"), and how often the 6 s
settle limit fires.

