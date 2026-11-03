# Valve turning: when the fin leaves the U -- the fin-in-the-U monitor and the clearance guard (2026-11-03)

User request (after the U push, `../valve_upush_20261102/`): *"what if the fin goes out of the U shape during the
turning? will that be dangerous to the system? ... what is your plan to guarantee the safety of losing fin? Note
that we have mocap info for the fin."* Then: *"If mocap goes wrong and causes fake abort, that is fine, since the
drone safety is of the highest priority ... implement this safety consideration and validate them in the isaac,
also validate some other changes that haven't been tested in isaac."*

## 1. The design

Losing the fin is harmless; the turn carrying on while the valve stays put is not (Isaac f0, the old shallow grasp:
the landing gear flew into the fin ~70 deg later). So, on EVERY valve mocap sample (~120 Hz):

1. **The fin in the U** (planner, `plFinGuardCheck`): the fin placed from its mocap (obj_0 + the axis offset + the
   valve CAD), the claw from the arm's joints + the odometry + the attitude; in the fin's own frame: the claw's
   depth under the fin top, the fin across the claw, the gap from the bracket's push corner to the fin's back edge.
   From Ready's end (the claw on the handle) through the turn, not the exit (which leaves the fin on purpose).
2. **The clearance guard**: whatever the cause, the skids, struts, rotor discs and arm links (NOT the gripper, which
   pushes the fin by design) against the fin, beam and hub boxes and the five marker balls.
3. **The response**: the push-and-pull abort at once (0.02 s persistence): the turn cut where it is, contact off,
   the claw following the vehicle, ONE 0.30 m climb in 1.5 s (the jaws are already open), then the arm home.
4. Also: the valve's mocap and the arm's joint states going silent / frozen / jumping abort after 0.2 s (was 0.5);
   a lag that HOLDS the turn (the governor) aborts after 2 s (was 6).

## 2. The thresholds, from the recorded runs (not the first guesses)

The strict values first proposed (depth 50 mm, across 15, ahead 20, clearance 30) were replayed on the U-push
campaign's 24 Isaac runs before any code (`../valve_upush_20261102/tools/fin_monitor_replay.py`,
`clearance_replay.py`; the Python chain reproduces the planner's own claw to 1.8 mm median):

| check | completed deep-grasp runs | the failures | shipped |
|---|---|---|---|
| claw depth under the fin top | >= 71 mm | <= 27 (the shallow grasps that could not push) | < 50 mm |
| fin across the claw | <= 22.6 mm (drift, in the hold) | -- | > 25 mm |
| fin BEHIND the bracket | <= 6 mm | 173 mm (f0, lost) | > 15 mm |
| fin AHEAD of the bracket | <= 47 mm (the slack at the turn's start; a drifting vehicle catching up) | 69 (k3) | > 60 mm |
| clearance to fin / beam / hub | >= 25 mm (the forearm by the fin) | -0.9 (f0: the skid into the fin) | < 15 mm |
| clearance to the marker balls | 10-22 mm | -2 (m3, k3: brushed) | < 5 mm |

- "Ahead" is NOT a fin loss: every turn starts with the bracket 13 mm (plus the claw's error) behind the fin's back
  edge, and under drift the vehicle falls back and catches up (k3: 69 mm, then it pushed the valve 80 deg). The
  loss signal is the fin BEHIND the bracket, which cannot happen while the fin is inside the U.
- **A finding the guard exposed**: the LEFT SKID's front end passes 10-22 mm over the tallest marker ball (#2, by
  the hub, top 140 mm above the plate; the others <= 107 mm) mid-turn, as the vehicle runs ahead of its circle --
  and in two drift runs (m3_final_drift3, k3) it BRUSHED it (Isaac's contact report: 12-15 points, ~0.25 s, light).
  The real T650's gear sits 5.5 mm deeper. Lowering that ball on the real valve gives the guard (and the vehicle)
  room.

## 3. What changed

- **fsc_trajectory_planner**: `finCapture()` and `valveClearance()` (push_pull.hpp/.cpp, gtest
  `PushPullValveGuards.*`); the node's `plFinGuardCheck` on every gate-accepted valve mocap sample, its keys
  `push_pull_guard_fin_*`, `push_pull_valve_fin`, `push_pull_guard_clearance*`, `push_pull_guard_skid/_strut/_rotor/
  _arm_link_radius`, `push_pull_valve_beam/_hub/_markers/_marker_radius` (all 0 / off by default; gate-only),
  telemetry `push_pull/arrival_error` [34..38]. Push-pull gtests 32/32, the box loopback ALL PASS.
- **Valve yamls** (generator `../valve_turning_20261015/tools/make_valve_turning_yaml.py`, sim + hardware): the values
  above, `push_pull_guard_box_stale_s` 0.50 -> 0.20, `push_pull_follow_hold_timeout_s` 6.0 -> 2.0.
- **The raw hardware stack**: expects them (108 valve values; box 62 unchanged) and refuses a planner without them.
- **The valve scene**: `PEGASUS_VALVE_FAULT` (force / jam / drop / freeze at a turn angle) for this validation.

## 4. Isaac validation

Headless, the bare fin, the v2 valve (0.072 N.m drive), the station flying every step after Go To Start, faults from
the scene's new `PEGASUS_VALVE_FAULT` 30 deg into the turn. Scored by `tools/score_guard.py` (who aborted and when,
the guards' live readings, the abort's climb, tilt, q3, DIRECT, and Isaac's contact report by link).

**Chain 1 (the first build)**

| run | fault | outcome | first guard | after the fault | after |
|---|---|---|---|---|---|
| g1 | none | **complete**, valve 91.7 deg | -- | -- | live readings: depth 74-85 mm, across +-6, gap -4..23, boxes >= 26, balls >= 12 |
| g2 | lift +15 N, 1.5 s | abort | q3 rate (q3 5.8 deg at 260 deg/s) | 0.12 s | DIRECT kept, +0.28 m, tilt 14 deg, the arm through q3 -90 |
| g3 | back -10 N, 1.5 s | abort | **fin 72 mm ahead of the bracket** | 0.30 s | **crashed** (the push went on through the abort) |
| g4 | side +10 N, 1.5 s | abort | q3 rate (352 deg/s) | 0.38 s | **crashed**, the wrist on the valve |
| g5 | jam at 30 deg | abort | tilt 12 deg | 1.25 s | **crashed**: the fin stayed IN the U, q3 32 -> 50 (the stop) as the vehicle pressed on it |
| g6 | fin mocap dropped 1 s | abort | **fin 16 mm behind the bracket** | 0.66 s | clean: DIRECT kept, +0.28 m, tilt 4 deg |
| g7 | fin mocap frozen 1 s | abort | **clearance: forearm 14 mm from the fin** | 0.30 s | clean, tilt 2 deg |
| g8 | frictionless valve | abort | **fin 62 mm ahead** (it coasted out the front) | 5.7 s | clean, tilt 3 deg |
| g9 | drift seed 2 | **complete**, 85.9 deg | -- | -- | balls >= 8 mm |
| g10 | drift seed 3 | **complete**, 92.6 deg | -- | -- | balls >= 10.5 mm |
| g11 | drift seed 4 | abort BEFORE the turn | fin 62 mm ahead, in the hand-over hold | -- | a FALSE trip (nothing pushes there); clean |

- 10-15 N for 1.5 s is 27-40 % of the 36.7 N weight and kept pushing through the abort: the arm buckled within
  0.1-0.4 s, faster than any guard. Those runs measure the disturbance, not the guards -- chain 2 repeats them at a
  survivable size.
- The fin mocap faults are caught by the GEOMETRY, not the 0.2 s feed timeout: Isaac's mocap emulator keeps
  republishing the last pose with noise added, so the planner sees a valve that stopped while the arm kept turning.
  (On hardware a lost body is republished bit-identical -- the frozen-feed check catches that at 0.2 s.)
- Two fixes from chain 1, re-flown in chain 3: **`push_pull_guard_fin_q3_max_deg` 40** (completed turns q3 <= 34.7
  deg; the jam's signature is the arm folding onto its stop while the fin stays in the U -- the lag never exceeded
  9 deg, inside the completed turns' 5.7-8.7, so the lag cannot catch it in time), and **the fin-AHEAD check only
  while the turn flies** (g11).

**Chain 2: the same forces at a realistic size (1 s)**

| run | fault | first guard | after the fault | after |
|---|---|---|---|---|
| h1 | back -3 N | the sway guard (178 mm/s) | 0.66 s | DIRECT kept, +0.29 m, tilt 8.4 deg; q3 down to 1.5 deg |
| h2 | side +3 N | the sway guard (191 mm/s) | 0.62 s | DIRECT kept, +0.29 m, tilt 8.2 deg |
| h3 | lift +5 N | q3 rate (160 deg/s) | 0.35 s | DIRECT kept, +0.29 m, tilt 10.2 deg -- but the arm folded THROUGH q3 0 (-26 deg) and the wrist touched the valve |

All three survive the abort. They also show where the real risk sits: with the claw held in the room through the
turn, the ARM absorbs any push on the vehicle -- a 5 N lift for 1 s folded it through its singular angle before the
abort could take it back. The U removed the clamp, but not the room hold.

**Chain 3 (the rebuilt planner: the elbow limit, the ahead check in the turn only)**

| run | fault | outcome | notes |
|---|---|---|---|
| j1 | none | **complete**, 92.0 deg | depth 77-87 mm, across -3..8, boxes >= 24.9, balls >= 14.6 |
| j2 | jam at 30 deg | abort by **q3 41.2 > 40 deg**, 1.09 s after the jam | **crashed** (tilt 45 deg, the wrist on the valve) -- only 0.16 s earlier than the tilt guard did |
| j3 | drift seed 4 | **complete**, 86.7 deg | the run that false-tripped in the hold (g11) |
| j4 | drift seed 2 | **complete**, 92.7 deg | balls >= 13.8 mm |

## 5. Conclusions

- **No false trips** on the final build: 2 of 2 nominal and 4 of 4 drift turns completed (g1, g9, g10 flew the first
  build, whose only later change narrowed a check; j1, j3, j4 the final one), the valve at 85.9-92.7 deg.
- **The cases the new checks target are caught and survived**: the fin's mocap dropped or frozen (0.66 / 0.30 s)
  and the valve coasting out of the U's front -- clean aborts, DIRECT kept, the one-climb abort up 0.28 m within
  1 s, tilt <= 4 deg. The fin-out-of-the-U check also caught the 10 N back push first (0.30 s).
- **Realistic pushes (3-5 N for 1 s) are survived**, caught first by the existing sway and q3-rate guards in
  0.35-0.66 s, tilt <= 10 deg -- but a 5 N lift folded the arm through q3 0 and the wrist touched the valve.
- **NOT made safe: a valve that seizes mid-turn, and pushes of 10 N or more.** In a jam the fin stays IN the U, so
  nothing about the fin looks wrong; the vehicle presses on it, and by the time anything trips (the elbow limit at
  1.09 s, the tilt guard at 1.25 s) the arm is near its stops and the vehicle tilted ~10 deg -- the abort from there
  crashed both times. The lag cannot catch it sooner (9 deg at the trip, completed turns reach 8.7).
- **What the faults show underneath**: with the claw held in the room through the turn, the ARM absorbs every push
  on the vehicle. The U removed the clamp but not the room hold. Candidates, none tried: the claw FOLLOWING the
  vehicle during the turn (a push then moves the claw off the fin -- caught by the fin checks -- instead of loading
  the arm), and a stall detector on the valve's rate against the plan's after the bracket has engaged.
- Isaac's mocap emulator republishes a lost body with noise, so the 0.2 s stale / frozen-feed abort was not
  exercised; on hardware a lost body comes back bit-identical, which that check catches.

Tools: `tools/score_guard.py`; the replays that set the thresholds `../valve_upush_20261102/tools/
fin_monitor_replay.py`, `clearance_replay.py`; chains `sim/chain1-3.sh` (runs/ is not committed).
