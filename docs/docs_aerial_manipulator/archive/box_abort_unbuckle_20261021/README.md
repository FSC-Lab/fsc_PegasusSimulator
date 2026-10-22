# Box pushing: the abort unbuckles the arm once the jaws are open (report item 6, 2026-10-21)

The 10-21 hardware push (`../wb_box_pushing_flight_20261021/`) ended in a q3-guard abort that
held the MEASURED arm, q2 on its +45 limit and q3 at ~2 deg (-6.3 at worst while the jaws
opened), for ~9 s until the fold home (`tools/abort_window.py`).

## The change (fsc_trajectory_planner)

- New planner key `push_pull_abort_unbuckle` (node default false; true in the box-pushing sim,
  hardware and coordinated-admittance yamls via `make_push_pull_yaml.py`).
- With an abort held on the measured pose, the FIRST motion leg after the release wait (the
  3 cm retreat, else the climb) brings q2 / q3 to the push pose's; q1 / q4 stay as measured
  until the fold (they move the claw across the fin). The climb and the fold start from there.
- If that would lower the claw > 10 mm against the base (an over-folded arm), the climb carries
  it instead, so the open claw never drops along the handle (`planPushPullAbort`).
- An arm already within 1 deg of the push pose, or an unbuckle the planner refuses, keeps the
  old abort. The abort reply names the unbuckle.
- Geometry (`tools/unbuckle_geometry.py`): from the 10-21 trip pose the claw moves 25 mm back
  and 9 mm up against the base, inside the fin's plane.

## Checks (code only, no flights, per the user)

- gtest `PushPull.AbortUnbucklesTheArmOnceTheJawsAreOpen` (buckled and over-folded arms, no
  retreat); planner ctest 6/6.
- Loopback `[9h]` on the sim AND the hardware box yaml: the q3 guard trips on a buckled arm,
  the abort holds it through the release (1.0 / 1.5 s), reaches 32/28 deg by 4.0 / 4.4 s with
  the claw +9.6 mm, then climbs and folds home. ALL PASS both.
- Both whole-body hardware stacks check the key and refuse a planner without it.
- NOT flown (Isaac jammed-box test skipped on purpose).
