# Box pushing: the box's mocap samples gated before the box guards use them (2026-10-21)

From the 10-21 hardware report (`../wb_box_pushing_flight_20261021/`): obj_0 mis-solved during the
push, and the bad samples reached both box guards and the push monitor. Only the guards' 0.2 s
persistence kept them from aborting. The fix: drop a sample that jumps more than 20 mm or 10°
from the last good one, as the drone's raw-mocap guard does.

## The gate

- **The test:** the drone mocap guard's jump test (`fsc_drone_state_estimator_ros2` MocapGuard),
  with its constants:

  `|p − p_good| ≤ 20 mm + 1 m/s·dt + ½·2 m/s²·dt²` and `angle(q, q_good) ≤ 10° + 90°/s·dt`,

  measured from the LAST GOOD sample, `dt` the time since it.
- **The growth:** the allowance grows over a gap, so a pose that really moved is let back in
  (61 mm after 41 ms, a 95° turn after 0.94 s).
- **What a dropped sample reaches:** nothing — not Get's buffer, the push monitor or the box
  guards. While samples keep being dropped, the box feed ages; with the claw on the box the
  0.5 s stale guard then aborts, naming the feed JUMPING.
- **Garbage:** a broken quaternion is dropped too ("garbage is never a pose", as MocapGuard).
- **Where:** fsc_trajectory_planner, `plBoxGateAccept` in the node + `boxSampleWithinGate` in
  `push_pull.{hpp,cpp}`. Parameters `push_pull_box_gate_jump_m` / `_jump_deg`; node default 0 = off,
  every sample used as before; the box-pushing yamls set 0.020 / 10.0.

## Replayed on the recorded streams (`tools/gate_replay.py`, pure numpy)

| stream | samples | dropped | what they were | largest step kept |
|---|---|---|---|---|
| 10-21 `_170123`, the push | 11 528 | **80, in 24 runs of 1–25 samples** | all the 94–96° roll with the origin 61 mm off, or a 180° flip; 78 during the hover and Go To Start, **2 with the claw on the box** (40.85 / 40.87 s) | 6.7 mm / 4.1° |
| 10-21 `_162604` (11 min on the ground) | 85 201 | 16, in 4 runs | the same two mis-solves, all outside DIRECT | 17.3 mm / 3.7° — a person carrying the box onto the table at 1.9 m/s |
| 10-21 `_161745`, `_164338`, `_165311` | 23 551 | 0 | — | ≤ 4.2 mm / 2.4° |
| Isaac, 10 runs of 10-21 | ~80 000 | 0 | (the sim never mis-solves) | ≤ 3.5 mm / 1.6° |

- **The longest mis-solve run, 25 samples (199 ms), stays dropped:** at that gap the angle
  allowance is only 28°.
- **A run of that length with the claw on the box would not abort:** the stale guard needs 0.5 s.

## Checks — to run when the simulator is free (a valve-turning test was flying when this was written)

- gtest `PushPull.BoxSampleGateDropsTheMisSolvesOnly` (the numbers above).
- Loopback `[9g]`:
  - a 0.3 s run of the mis-solve while pushing (ungated it would trip the 2° / 0.2 s tilt guard)
    gives no trip, and the monitor's box travel stays on the plan;
  - a feed that keeps jumping aborts as JUMPING;
  - every earlier step runs with the gate on (no false drop on the rig's 0.5 mm noise).
- The box yamls regenerated (sim, hardware, coordinated admittance); the valve yamls unchanged.
- Both whole-body hardware stacks list the two values and refuse a planner without the gate.

## Not covered

- Never flown.
- The gate only stops single-sample discontinuities. A mis-solve that persists for more than 0.5 s
  with the claw on the box still aborts; that is the intended response, since the box can no longer
  be trusted.
- The cause is the marker layout. The user re-distributes the markers on 2026-10-22 to make the
  body less symmetric.

## Applied and checked (2026-10-21 night, after the valve test)

- Built; gtest `BoxSampleGateDropsTheMisSolvesOnly` passes (planner ctest 6/6); loopback `[9g]`
  PASS on the sim and hardware box yamls (two 0.3 s mis-solve runs: no trip, the monitor within
  1.2 mm; a feed that keeps jumping aborts as JUMPING).
- One addition found by the box-point loopback: the gate dropped the flipping samples, so Get
  captured instead of refusing. Since the gate keeps the solution it holds (not necessarily the
  true one if the feed flipped from the start), Get now refuses any capture window in which the
  gate dropped a sample ("the box's mocap is flipping or jumping ... NOT captured").
