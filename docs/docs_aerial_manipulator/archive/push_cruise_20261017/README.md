# Box pushing: a constant-speed (cruise) push profile -- tried, and NOT adopted (2026-10-17)

User request: the shipped push is one smooth min-snap move (0.50 m in 24 s, speed 0 -> 46.5
mm/s at mid-push -> 0, no constant-speed part). Could a profile with a constant-speed cruise be
safer? Highest priority: safety.

**Answer: no, not as tried. 2 of 3 cruise pushes ended in a q3-guard abort AFTER the release,
and one of those crashed the vehicle. The min-snap push completed 2 of 2 on the same day and
build (13 of 13 including the 10-16 runs). The shipped yaml keeps the min-snap push.**

## 1. What was built (kept, default OFF)

- **fsc_trajectory_planner**: `PushPullConfig::push_ramp` / parameter `push_pull_push_ramp_s`
  (default 0.0 = the min-snap push, unchanged). > 0: the straight push ramps its SPEED up over
  that many seconds on a septic smoothstep, holds a constant `push_distance / (push_time -
  ramp)`, and ramps down, all inside `push_pull_push_time_s`. `CruiseTrajectory` in
  `push_pull.cpp` (analytic, through `flatState`, CoM smooth through snap at every join, the
  arm and heading held); refuses a ramp >= half the push time; the Push button's reply names
  the profile ("slides 0.50 m at a constant 29.4 mm/s (3.0 s ramps, 20.0 s)" / "slides 0.50 m
  in 24.0 s"). gtest `PushCanCruiseAtConstantSpeed` (constant speed to 1e-9, never faster, FD
  chain, every join continuous through snap, rest at both ends, refusal); push suite 15/15.
- **Loopback** `test_push_pull_loopback.py`: checks the streamed cruise when the yaml sets a
  ramp. Two pre-existing breakages found and fixed while running it (both since the 10-16 box
  guards): the rig's fixed box made the box-lag guard abort every push (the rig now runs with
  `push_pull_guard_box_lag` / `_box_tilt_deg` 0), and step [9] re-planned from where the push
  left the box, which the box-on-table check now refuses (it re-plans from the start). ALL PASS
  on the shipped yaml and on a cruise copy (cruise streamed at 29.41 mm/s).
- **make_push_pull_yaml.py**: `push_pull_push_ramp_s: 0.0` in the planner block (so a run can
  override it); sim and hardware box-pushing yamls regenerated (the only change: that line).
- **Refusals**: the Isaac scene wrapper's planner check compares `push_pull_push_ramp_s`; both
  whole-body hardware stacks refuse a planner build without it when the yaml sets a ramp > 0
  (an old planner would fly that push time as the min-snap move, ~1.6 x the cruise's peak).

## 2. Flights (Isaac, headless, RTF 1, raw mocap, the 10-16 final configuration)

500 g, mu 0.29 / 0.29, 0.50 m along -y, push pose [0, 32, 28, 0], the 10-09 push gains, all
10-16 guards on. Cruise = 20 s with 3 s ramps, 29.4 mm/s (chosen offline: 37 % lower peak speed
than the min-snap, the box sliding without stopping above ~25 mm/s in the 10-16 runs).
`tools/compare.py` (full table: `compare.txt` / `compare.csv`).

| run | profile | outcome | box stood still | restarts | q3 range in the push | after the jaws opened |
|---|---|---|---|---|---|---|
| s3_P_1..3 (10-16) | min-snap 24 s | 3/3 complete | 19-22 % | 4-7 | 16.9..43.6 | q3 >= 26 |
| m_nom_1, m_nom_2 (today) | min-snap 24 s | 2/2 complete | 19-21 % | 3-5 | 16.9..47.2 | q3 >= 19 |
| c_nom_1 | cruise | complete | 6.7 % | 4 | 18.4..46.7 | q3 >= 24 |
| c_nom_2 | cruise | **q3 guard, 1.7 s after the release** -- safe | 6.0 % | 0 | 21.4..48.4 (0.5 s near +50) | q2 on its 45 stop, q3 7.1 |
| c_nom_3 | cruise | **q3 guard, 2.4 s after the release -- CRASH** | 15.3 % | 10 | 20.7..50.0 (1.1 s ON +50) | q2 on its stop, q3 through 0 to -79 deg; thrown ~8 m |

The cruise did what it was built for during the slide in 2 of 3 runs (the box stood still 6-7 %
of the time against 19-22 %). Every box was pushed 490-499 mm and stayed on the table, released.

## 3. What went wrong: the release window

- From the Push press to the Exit press the planner holds the claw IN THE WORLD
  (`push_pull_world_anchor_push`); Exit restores the CoM anchor. Between the jaws opening and
  Exit the OPEN claw holds nothing, but its reference is still fixed in the room, so every bit of
  vehicle motion goes into the arm. The sim driver leaves that window 2.8 s; the arm GS on
  hardware ~1 s (Exit opens the jaws, waits 1 s, exits). Both cruise failures developed 1.7-2.4 s
  after opening.
- After the release the base drifts 16-44 mm in every run, min-snap included
  (`tools/release_window.py`, output in `release_window.txt`). The
  min-snap worst cases came within 4-7 deg of the q3 guard (10-16 g_stick 14 deg, s3_PH_2 17,
  g_m600 17). **So the window is a latent hazard of the shipped push too; the cruise made it
  bite.**
- c_nom_3: the base drifted back ~25 mm in the first second; q2 rose 26 -> 42 deg; at 2.0 s a
  second surge put q2 on its 45 deg stop; q3, the only joint left, went through zero into the
  elbow-singular branch within 0.2 s; the guard tripped at -21.8 deg (it needs 0.2 s below 10),
  the node left DIRECT and the vehicle was thrown 8 m and came down at z 0.24 m.
- Why the cruise: the arm's slow q2/q3 swing (the arm absorbing the base's wander) is the same
  in every run while gripped and still (q2 std 4-7 deg), but WHILE SLIDING it is 3.6-4.4 deg for
  every min-snap run against 4.0 / 5.2 / 5.5 for the cruise, and it carries into the release
  (q2 p-p in the window 9.6-18.4 min-snap, 11.8-20.6 cruise). Hypothesis, NOT proven: each
  stick of a stick-slipping box clamps the claw to the table and damps that swing; a box sliding
  at constant speed offers only constant kinetic friction, which damps nothing. c_nom_2 failed
  with an ordinary sliding swing (4.0), so this is not the whole story either.
- Odds it is chance: 2 failures among 16 pushes, both in the 3 cruise runs: p = C(3,2)/C(16,2)
  = 0.025.
- Also seen, c_nom_1 only: one grip slip 11 s into the push -- the jaws slipped ~4 mm up the fin
  (the sim's 20 mm grip shim), the box rocked 6.6 mm up / 2.9 deg, the true contact force spiked
  to 12.7 N, the planner's box-tilt reading reached 1.7 deg of its 2.0 deg guard. Not seen in
  any min-snap run (box rise <= 0.7 mm, tilt <= 0.4 deg). Cause not found.

## 4. Recommendation

1. **Fly the min-snap push** (shipped: `push_pull_push_ramp_s: 0.0`, 24 s). Do not set a ramp.
2. **The release window needs a fix for the min-snap push too** (not done -- needs flights):
   restore the CoM anchor, or start the exit climb, the moment the jaws are open, so the open
   claw is never held in the room. On hardware keep the arm GS's 1 s between the jaw opening and
   the exit as short as it is; do not hover with open jaws on the fin.
3. The cruise option stays in the planner, default off, so it can be re-tested after (2); delete
   it if it is not wanted.

## 5. Not covered

- One flight per configuration beyond the normal pushes; the planned cruise worst cases (sticky
  box, jam, 1 kg, 600 g, ABORT button) were NOT flown -- the campaign stopped after the crash.
- No hardware. The sim box slides with static = kinetic friction (`MU_S` = `MU_K`).
- The damping mechanism in section 3 is a hypothesis from 16 runs.

## 6. Traps hit (each cost a run)

- **An integer for a double parameter kills the planner at startup**: `push_pull_push_time_s: 20`
  -> `[FATAL] ... setting it to {integer} is not allowed`, and the driver then waits 5 min for a
  planner that is not there ("old binary?"). `fly.sh` now writes bare integers as floats.
- **Stopping a chain by killing its bash leaves `fly.sh` / `run_pl.sh` running**, and their
  `ros2 param get ... /uav_0/<the planner>` poll names the planner, so the NEXT stack's
  stale-planner guard refuses to start (2 runs lost). Stop a chain with its process group:
  `kill -- -<PGID>`. The harness also runs every command as `bash -c "<full text>"`, so a
  command of your own that names the planner trips the guard the same way.

## Files

`tools/fly.sh` (one flight, `T` / `RAMP` set explicitly), `tools/chain_*.sh` (the queues as
flown), `tools/compare.py` (cruise vs the 10-16 min-snap runs, case by case), `runs/` (the
flights; `runs/failed_*` = the attempts lost to the traps above, no flight data).
