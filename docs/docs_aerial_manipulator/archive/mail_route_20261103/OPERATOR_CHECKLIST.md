# Mail route (3 letters, 2 mailboxes) — operator checklist

For the arm GS operator and the safety pilot flying the three-letter route on the
AM-T650 (Orin + experiment laptop). Every item comes from a problem found on
2026-11-03 in the route audit, the operator fuzzer
(`fsc_trajectory_planner/test/fuzz_mail_route_loopback.py`) or the Isaac route runs
(`runs/route2`..`route15`). Requires fsc_trajectory_planner and the arm GS at the
2026-11-03 audit commits or later — older builds lack the carrying interlocks below.

## 0. Before the session
1. Orin: rebuild `fsc_autopilot_ros2`, then `fsc_trajectory_planner` (it links the law).
   Laptop: rebuild `utils_custom_ground_station`. The Mail Route tab must NOT say
   "the planner has no pick_place/route/plan" — if it does, the Orin's planner is stale.
2. No gain changes: the hardware mail yaml keeps `wb_l1_omega_c_t: 2.927`, and the
   stack prints "all N tuned values present … (09-27 H1b + 10-16 j4 fix)".
3. The insert is AUTOMATIC and cannot be triggered by hand: leave `pick_place_insert_auto`
   true. In a route the planner refuses a manual Insert and the tab's Insert button stays
   disabled. Check the mail yaml carries `pick_place_place_approach_back: 0.25` (the Insert
   Margin), `pick_place_insert_auto_tol: 0.030`, `pick_place_insert_auto_settle_s: 3.0` and
   `pick_place_insert_auto_settle_tol: 0.015`.
4. Record the 10-31 mail topics (`archive/mail_3cm_tune_20261031/tools/hw_topics.txt`)
   + `/uav_0/whole_body_planner/pick_place/{status,info}`,
   `/uav_0/whole_body_planner/pick_place/route/drop_now`, the raw `/vrpn_mocap/*`.

## 1. Layout
5. Hats in a line in front of the boxes, the nearest ~1 m from the box face, 0.6 m apart.
6. Letters turned 90° on the hats, seated exactly as the localization frame sat. The
   planner cannot see a misplaced letter (route12: the claw pushed one 62 mm and closed
   on air).
7. Mailboxes: the two flown boxes (same build, same height), centres 0.5 m apart. While
   posting into one box the props pass only 9–13 cm over the OTHER box's top and the
   planner does not check it: nothing on the box tops, no taller boxes, not closer.
8. Take-off / landing spot (Start / Land in the yaml) off the hat line, floor clear.
   Put the drone ON the Start mark: Adjust refuses an offset over 2 m ("is the vehicle on
   the START mark?") and the session stops there. To take off elsewhere, change Start /
   Land Start / Land in the yaml first.

## 2. Gets (on the ground)
9. Frame on hat k → Get Letter k → wait for the row; one at a time. A Get with the frame
   still on another target is refused (the frame-left-behind check).
10. Frame seated on the RED top → Get RED box; on the BLUE top → Get BLUE box.
11. Take the frame OFF the box top after the last Get (it cuts the prop clearance over
    that box to ~5 cm).
12. Then place the letters. Anything moved later: re-Get it and Plan again.

## 3. Take-off and DIRECT
13. SAFETY take-off, hover; whole-body DIRECT only with the arm at rest ("arm is NOT at
    rest" in the planner log = wait).
14. Hover 1–2 min in DIRECT before letter 1 (early-flight oscillations ended 2 of 9 sim
    flights). Roll / pitch visibly swinging and growing: do not start — SAFETY.
15. Plan from the Mail Route tab only, wait for READY. Another tab's Plan ends the route.

## 4. Each letter
16. Press Letter k (any order; posted letters are disabled).
17. Go To Pick Start → Ready To Pick → Pick.
18. CHECK THE GRASP before lifting: a close that stalls part-way = held; a FULL close
    (≈ −48° in sim) = empty. The tab does not catch a missed hook grasp. Empty: open,
    Pick again — or ABORT.
19. Exit To Pick before Go To Mailbox Start (skipping it is now tracked, but the exit is
    the clean lift).
20. Go To Mailbox Start → Ready To Insert.
21. The vehicle holds 0.25 m back from the mouth and the planner slides the letter in BY
    ITSELF once the drone has settled (its body within 15 mm of its own mean for 3 s) and
    the claw is within 30 mm of the point in front of the mouth and still for 1 s; the
    status says "waiting for the drone to settle" until then. The gripper opens by itself
    at the target and Exit To Insert follows by itself. Watch the drone: creeping toward
    the box, or the status stuck on "settle" -- ABORT (it backs out of the mouth first).
    (Why: at the old 0.15 m margin the letter waited ~5 cm from the box face; a letter
    touching the box while waiting dragged the drone into it -- the insert aborts in sim.)
22. After the release: confirm the letter fell in before the next letter.
23. Mid-letter: never the current letter's button or Reset with a letter in the claw
    (refused now). Margin edits are fine — then Plan again in the Mail Route tab (works
    mid-route, also at the pick).

## 5. Aborts and SAFETY
24. ABORT opens the gripper: a carried letter is dropped and marked ✗ — put it back on
    its hat (re-Get if moved) before flying it again. An abort during Exit To Insert
    counts as posted.
25. The tilt guard aborts at 10°, the node's watchdog drops to SAFETY at 15°. Let an
    abort finish before acting.
26. After the pilot hands DIRECT back: status "NOT PLANNED" → Plan, carry on. An exit cut
    by SAFETY can be pressed again; the carried letter stays tracked.

## 6. Landing and between flights
27. Go To Land Start only with the claw empty (refused otherwise).
28. Land target ≥ 0.35 m (resting height 0.305 m). "Disarming denied: not landed" is
    known on this rig: land by reference, then disarm.
29. Between flights: Reset, letters back, re-Get what moved. After any crash or abort to
    SAFETY, restart the stack in full rather than reusing the launch.
