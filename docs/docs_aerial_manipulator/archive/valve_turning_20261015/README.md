# Valve turning (2026-10-15)

The user's valve-turning experiment: a beam on a bearing with a fin handle,
bolted to the box-pushing table. The aerial manipulator grasps the fin as it
grasps the box's fin, flies ONE circle that turns the beam a full revolution,
releases, and flies round to the landing spot as the box push does. A
physical-interaction task with its own parallel parameter files, on the
whole-body 4-D L1 law.

## 1. The CAD (`rotorcraft/assets/Valve_Assembly.stl`)

Metres, +Z up, three closed shells (`tools/stl_to_usda_valve.py` reads the
axis-aligned faces and asserts every vertex lies in the blocks it declares):

| part | shell | geometry (mm) |
|---|---|---|
| base plate | 398 tris, 51 cm³ | 150 x 50 x 20, the bearing pocket in its centre, an 8 mm shaft |
| bearing | 464 tris | a 608: 22 OD, 8 bore, 7 wide, at z 5.5..12.5 |
| rotor | 11 082 tris, 196 cm³ | a 50 mm hub on the bearing (z 5.5..35.5); the BEAM 225 x 50 x 14 (z 35.5..49.5; 200 mm one way from the axis, 25 the other; pockets); a 10 x 50 x 20 root at x 153.9..163.9; the FIN 5 (radial, x 156.4..161.4) x 40 (tangential, y -16.4..23.6) x 130 (z 69.5..199.5) |

Axis at CAD (-1.1, 3.0); plate bottom at z = -4.5. So the fin is at **radius
0.160 m**, 0.6 mm off the beam's centre line, its top **204 mm** above the
plate bottom; the beam top is 54 mm up.

## 2. The asset (`Valve_Assembly.usda`)

Frame: origin = the bearing axis on the plate-bottom plane, +x along the beam
toward the fin, +z up.

* `/base` -- a KINEMATIC rigid body (fixed; a joint needs a body on each
  side), visual = plate + bearing, `Collision/plate` one convex box.
* `/rotor` -- a DYNAMIC rigid body at the SAME origin, so its published pose
  IS the axis + the beam's yaw (= mocap `obj_0`). Visual = the rotor shell;
  `Collision/{beam, root, fin}` exact boxes (the hub is not a collider: it
  overlaps the plate's box and touches nothing). MassAPI from the closed
  solid at 1240 kg/m³ (solid PLA): **243 g, CoM 85 mm along the beam,
  0.0031 kg m² about the axis** (`--mass` overrides; a printed part at 20 %
  infill is ~a third).
* `/joint` -- `UsdPhysics.RevoluteJoint`, axis Z, base -> rotor, both local
  frames at the origin, an `angular` DriveAPI with stiffness 0 whose damping
  the scene sets.
* `fsc:*` attributes carry the fin's radius / thickness / width / top /
  bare bottom, the beam's reach / top, the plate size, the rotor's mass and
  inertia, so the scene and the yaml never re-measure the mesh.

## 3. The scene (`application/robotic_arm/07_px4_t650_aerial_manipulator_valve_turning.py`)

06 loaded as a module, the box push's table (the same constants), the valve
referenced onto `/World/valve_turning/valve` at `PEGASUS_VALVE_XY`
(1.20, 0.00 = the table's centre) with the beam pointing
`PEGASUS_VALVE_YAW_DEG` (180 = the fin toward the start, at (1.040, 0)). The
plate's long axis then runs along the table's 150 mm depth, exactly.

* Bearing: viscous damping `PEGASUS_VALVE_DAMPING` on the joint drive
  (**USD angular drives are per DEGREE/s -- the scene divides by 180/pi; the
  first flight did not, see 5.1**) plus a Coulomb torque
  `PEGASUS_VALVE_FRICTION` applied every physics step as
  `-f tanh(w / 0.05)`, momentum-clamped (PhysX maximal joints have no
  friction knob). Flights v1-v6 used 0.02 N m s/rad + 0.01 N m; since
  2026-10-17 the defaults are **0.0 + 0.048 N m and a 204 g rotor** (section 7).
* Grip shim: the sim claw cannot clamp 5 mm (its pads stop ~19 mm apart), so
  the fin's bare part carries an invisible `PEGASUS_VALVE_FIN_GRIP_MM` (20)
  collider, as the box's fin does.
* `obj_0` = the rotor (axis + beam yaw) at 62.5 Hz; `claw_0` = the real claw
  point; `/push_pull_truth/contact` = the rotor's PhysX contact wrench (the
  box push's layout); `/obj_0/state/angle` = the beam's unwrapped angle, rate
  and degrees turned; the pane prints every 2 deg of beam motion.
* The T650 gear (`AM_T650.usda`), the real arm's q2 stops [-20, 45], the
  gripper torque cap 0.3 N m, the waypoint view -- the box push's.

**ON HARDWARE define the `obj_0` rigid body the same way**: pivot ON the
bearing axis at the plate-bottom plane, x along the beam toward the fin -- or
move `push_pull_ee_offset` by the difference.

## 4. The planner: the push step as a TURN (`fsc_trajectory_planner`, uncommitted)

The push-and-pull mode gained ONE option, `push_pull_turn_deg` (0 = the
straight push, unchanged; the box push yaml does not set it):

* `pushPullWaypoints`: with a turn, the grasp rest is rotated about the
  captured body's vertical axis by the angle (`wp.pushed`), and the nose must
  lie along the turn's tangent at the handle (+90 deg from the handle's
  bearing for a + turn, -90 for -), else the plan is refused with the
  reason. `wp.drone_yaw` is now WRAPPED (a beam at 180 + 90 used to give
  270, and Go To Land's "turn as typed" then flew 630 deg).
* `planPushPullPush`: settle, then a `TurnTrajectory` -- an analytic rigid
  rotation of the whole rest on a nonic phase (zero velocity through snap at
  both ends): the CoM on its circle about the axis (exact), the heading
  turning with it, the arm held; the derivatives `flatState()` reads come
  from Faa di Bruno on `e^{i s(t)}`. No solve. The claw rides on the
  compatible attitude, so its circle deviates by the acceleration tilt
  (0.7 mm at 36 s).
* A second keep-out `push_pull_obstacle` with the table's rule (gear and arm
  points 5 cm clear of a z_top over a footprint): the valve's swept circle,
  z_top = the beam top.
* `push_pull/info[60]` = the turn angle; the arm GS "Push & Pull" tab gained
  a **Turn [deg]** box beside Push Distance (disabled on an older planner).
* gtests: `TurnRotatesTheRestAboutTheAxisWithTheArmHeld` (circles exact,
  arm held, derivative chain FD-consistent to 1e-4, the tangent rule, a
  clockwise turn's Go To Land flown as typed = 450 deg from the +90 nose),
  `TheObstacleIsCheckedLikeTheTable`; the suite 11/11.

### The pose

The fin is only 0.179 m above the table at the grasp (25 mm under its top),
so the box push's 60 deg fold [0, 26, 34, 0] (claw 0.140 m under the body)
would put the T650's skids 2.7 cm over the beam top. Model FK over the real
q2 range (`transition_planner.arm_fk_model`): **[0, -5, 30, 0]** -- claw
0.187 m ahead / 0.275 m under the body, 25 deg from vertical, sigma_nd 0.298
-- puts the skids 0.8 cm ABOVE the fin top, 0.16 m above the beam, 0.21 m
above the table, with q2 15 deg and q3 20 deg from their stops. The fin
passes between the skids (0.28 m apart) under the claw.

## 5. Flights (headless, raw mocap, `tools/run_valve.sh <tag>`)

### 5.1 v1 -- aborted: the bearing damping was 57x too high (a unit)

The whole chain worked to the grip: Go To Start 8.0 s, Ready 3.0 s, the jaws
stalled on the fin after 1.5 s, CONTACT on, the turn executing (37.5 s
planned). Then the beam did not follow: it crept <1 deg for 7 s while the
plan advanced 2 deg, the arm wound toward its stops (q2 -9 -> -20, q3 34 ->
50 by 11 s) with the claw within 10 mm of its reference, and once the arm was
rigid on both stops the force rose to 9.5 N and the beam ran to 78 deg/s --
speed PROPORTIONAL to force: 9.5 N x 0.16 m = 1.5 N m against
1.15 N m s/rad x 1.36 rad/s. The scene had written 0.02 straight into the
drive's damping, which USD Physics reads per DEGREE per second
(= 1.146 N m s/rad). The force guard aborted at 12.4 s into the step, the
node's watchdog then handed over. Fixed in the scene (divide by 180/pi).

### 5.2 v2 -- the bearing is right, the ARM FOLDS ONTO ITS STOPS: the airframe wander

Damping fixed (0.02 N m s/rad), same mission, same point of failure (12.5 s
into the turn, 13.3 N on the guard). This time the beam FOLLOWED the plan --
claw lag <= 5 mm, tangential force on the fin <= 0.5 N (PhysX truth) -- so
the bearing and the grasp are right. What failed is the box campaign's known
mechanism (push_pull_20261003 README 9.2): the claw is world-held on the fin,
the airframe wanders fore-aft around it (+-1 to 3.5 cm at ~0.2 Hz, already
+-1.5 cm during the settle), and the arm absorbs every millimetre of it --
q2 swung 2 -> -2 -> -13 -> -7 -> 0 -> -15 -> -20, q3 the mirror image, until
q2 met its -20 stop and q3 its +50; the arm then went rigid, the base's
momentum yanked the fin (13 N) and the beam spun free. Two things ate the
margin: the grasp settled at q2 +0.5 / q3 23.7 instead of the planned
-5 / 30 (the pads seating on the fin), and at that pose the fold-back side
(q2 falls, q3 rises when the base leads) had only 15 / 20 deg to the stops.

| t into the step [s] | plan [deg] | beam | claw lag [mm] | base lead [mm] | q2 / q3 [deg] |
|---|---|---|---|---|---|
| 2 | 0 | -1.5 | -4 | -15 | -2 / 27 |
| 4 | 0.1 | 2.3 | +6 | +11 | -13 / 35 |
| 8 | 4.5 | 4.5 | 0 | +5 | -7 / 32 |
| 10 | 13.9 | 15.8 | +5 | +17 | -15 / 37 |
| 12 | 31.9 | 35.5 | +10 | +36 | **-20 / 47** (stops) |

Remedy flown as v3: the pose **[0, 10, 20, 0]** -- 30 / 35 deg of q2 and
30 / 20 deg of q3 to the stops either way (skids 13.5 cm over the beam, 1.6 cm
below the fin top). The alternative, not flown: a CoM-anchored claw during
the turn (`push_pull_world_anchor_push false`) -- with a free bearing nothing
is stuck, so the beam could simply follow the base's wander instead of the arm
absorbing it; the box push needed the world hold only because its box was.

### 5.3 v3 -- [0, 10, 20, 0]: the FIRST COMPLETE MISSION, with the arm on its stops

The full mission flew: Go To Start, Ready, grip, the turn (**+362.3 deg** on
the beam in 37.5 s), release, Exit, Go To Land, hover, landing. Claw radius
about the axis 0.1604 +- 0.0006 m against the planned 0.1602, claw height
0.980 +- 2.5 mm, tangential force on the fin 0.6 N mean, peak arm torque
1.23 N m, tilt <= 6.6 deg. But the arm touched BOTH its stops during the fast
half: the base LED its moving CoM reference by ~2.5 mm per deg/s of yaw rate
-- 0 below 15 deg/s, 40-62 mm through the 24 deg/s peak, back to 10-20 mm as
the rate fell -- and the world-held arm absorbed it (q2 -20, q3 50 from 15 to
26 s, one 10 N / 23 mm jolt that the guard's 0.3 s filter let through).

| t into the step [s] | plan [deg] | plan rate [deg/s] | base lead [mm] | q2 / q3 [deg] |
|---|---|---|---|---|
| 9 | 8 | 4.6 | -1 | 7 / 19 |
| 13.5 | 52 | 15.3 | -14 | 6 / 27 |
| 18 | 143 | 23.9 | **+62** | **-20 / 50** |
| 24 | 282 | 19.1 | +54 | -20 / 49 |
| 30 | 352 | 4.7 | +22 | -11 / 47 |
| 36 | 360 | 0 | +15 | 4 / 29 |

### 5.4 v4 -- the CoM-anchored claw during the turn: REJECTED

`push_pull_world_anchor_push false`: the arm did keep its pose (q2 5-11, q3
17-28 for 14 s, as predicted), and the claw reference followed the base --
but the base's fore-aft slosh (tilt to 4.8 deg at 2 s, a 4-5 s period) now
moved the claw AND its heading against the fin instead of the arm absorbing
it; at 13 s a 54 mm lunge, the wrist rolled to -55 deg (the fin twisting the
claw), the fin was wrenched out and the node's watchdog handed over at 16 s.
The world hold stays; the remedy is the turn's SPEED (5.5).

### 5.5 v5 -- a 72 s turn: the slosh grows instead

Same configuration as v3 at half the rate (peak 12 deg/s). The base lead
stayed <= 28 mm and the arm 7 / 5 deg clear of its stops -- the lead is the
rate's, as 5.3 said -- but the base's ~6 s fore-aft slosh against the pinned
claw (already +-12 mm during the settle) now had time to GROW: tilt 2.1 ->
3.7 -> 5.8 -> 6.7 -> 7.4 deg at 9 / 15 / 18 / 27 s, the node's watchdog
handed over at 29 s, the beam swung back 135 deg when the jaws let go.
v3's 36 s turn finished inside that horizon (its tilt peaked at 6.6 deg at
15 s and decayed). So the turn's speed is a TRADE between the stops (fast)
and the slosh (slow); 36 s is shipped, and the contact phase on a free
handle is MARGINALLY STABLE on a 30-40 s horizon -- the box push's contact
wander (10-29 mm std, 0.15-1.5 Hz) with nothing stuck to damp it.

## 6. Open

* **The contact-phase slosh.** Not a gain problem solved here: the base's
  position loop wanders 2-6 cm fore-aft (and q1 +-12 deg = 4 cm radially)
  around a claw pinned on a circle, and the arm absorbs it. Candidates, none
  flown: `wb_l1_contact_hold_translation` (the box's pl_4 rejected it with a
  stuck box; a free beam is a different plant), a stiffer base loop in
  contact, a CONTACT-off turn (the fin's reaction is ~1 N -- the free-phase
  observer may absorb it where the box's 1.1 N of static friction could not),
  a bearing with more friction (`PEGASUS_VALVE_FRICTION`: a real valve has
  some, and it damps the slosh through the claw).
* The grasp pose seats 5-10 deg off the planned [0, 10, 20, 0] (the pads
  seating on the fin); the plan's margins are measured from the plan.
* On hardware: the real bearing's friction and the real pads' grip on a
  5 mm fin are unmeasured; the skids pass 1.6 cm below the fin top -- check
  the real T650's skid span against the fin before the first grasp.
* The GS Turn box is built, not flown; the coordinated-admittance rig has no
  valve profile.

### 5.6 v6 -- the repeat of v3: the turn completes, the watchdog trips at its end

Same configuration as v3. The beam turned the full revolution again (+363 deg
at 31 s), with the same stop contact through the fast half (q3 on +50, q2 to
-18.8, base lead 30-47 mm) -- and this time the tilt kept growing while the
arm sat on its stops (2.3 -> 5.1 -> 6.9 -> 14.7 deg at 24 / 30 / 33 / 36 s)
and the node's 15 deg watchdog handed over at the end of the turn step; the
freed beam swung back 180 deg.

**Tally at the shipped configuration (world hold, [0, 10, 20, 0], 36 s):
2 / 2 full revolutions of the valve, 1 / 2 complete missions.** The grasp,
the circle, the release and the landing chain all work; what is marginal is
the CONTACT PHASE itself -- the base's lead / slosh against the pinned claw
puts the arm on its stops at the fast half (36 s) or grows past the watchdog
over a slow one (72 s). That is a tuning campaign (section 6), not a scene or
planner defect, and it is the same contact wander the box push records.

| run | config | turn | outcome |
|---|---|---|---|
| v1 | [0,-5,30,0], damping 1.15 N m s/rad (unit bug), 36 s | 78 deg | force guard 9.5 N at 12 s |
| v2 | [0,-5,30,0], 0.02, 36 s | 90 deg | force guard 13.3 N at 12 s (arm on its stops) |
| v3 | [0,10,20,0], world hold, 36 s | **+362 deg** | **COMPLETE** (stops touched, one 10 N jolt) |
| v4 | ... CoM-anchored claw | 60 deg | watchdog at 16 s (fin wrenched out) |
| v5 | ... world hold, 72 s | 64 deg | watchdog at 29 s (slosh grew) |
| v6 | = v3 | **+363 deg** | watchdog at the end of the turn (tilt 14.7 deg) |

## 7. The real valve's bearing, measured -- and the conservative sim values (2026-10-17)

The user built the rig (the printed valve, a "608RS ... Double Rubber Sealed"
8 x 22 x 7 mm bearing, the plate clamped across a console table's 150 mm depth
with two C-clamps, the beam overhanging an edge) and pulled the fin through a
hole 4 cm under its top and 1 cm in from its edge with a 10 N spring gauge.
**The reading never passed 0.2 N -- the gauge's first division** (4 minor ticks
per newton). The hole goes through the fin's 5 mm radial thickness, so a
tangential pull acts at r = 0.160 m; a pull off the tangent reads high. So the
real bearing is **<= 0.2 N x 0.160 m = 32 mN m**, an upper bound, not a value
(the sim's old 0.01 N m was a third of one division). The gauge was zeroed
hanging, then used flat; its flat zero was not checked (the hook's weight no
longer loads the spring flat, so it reads low by that weight). A sealed 608 at
this load is estimated at a few mN m, nearly all seal drag (not measured).

What the scene flies now (defaults; the user asked for a conservative value
for the robustness test, no flick video possible):

| | v1-v6 | since 2026-10-17 |
|---|---|---|
| Coulomb `PEGASUS_VALVE_FRICTION` | 0.01 N m | **0.048 N m** = 0.30 N at the fin, 1.5x the bound (covers a flat-zero offset up to half a division) |
| viscous `PEGASUS_VALVE_DAMPING` | 0.02 N m s/rad | **0.0** (a sealed bearing's drag at <= 25 deg/s is negligible; the constant term already bounds the reading) |
| peak bearing force at the fin, 360 deg / 36 s turn | 0.116 N | **0.300 N** |
| rotor `PEGASUS_VALVE_ROTOR_MASS` | 0.243 kg (solid PLA) | **0.204 kg**, 0.00257 kg m^2 about the axis |

The rotor mass: the pick-and-drop payload, same printer and material, weighs
190 g, ~10 g of it screws -> 180 g printed over 172.5 cm^3 of CAD solid
(basket 153.1 + holder 11.8 + two brackets 7.5; the five marker balls are
solid in the CAD, hollow in reality, left out) = 1044 kg/m^3 (84 % of solid
PLA); the rotor's 195.8 cm^3 at that = 204 g. An upper estimate: the beam is
chunkier than the thin-walled basket, so more of it is infill. It hardly
matters -- I alpha peaks at 0.12 mN m in the turn.

Model limits, checked offline (the scene's friction law stepped alone at
250 Hz, no simulation launched): stable, the beam never reverses
(f / (I w0) dt = 1.49, the momentum clamp makes the stop deadbeat); no static
friction is modelled -- below ~0.07 rad/s the clamp (I w / dt) sets the
torque, and a steady 0.03 N m creeps the beam at 0.047 rad/s instead of
holding it. At the planned turn the friction is >= 90 % of 0.048 N m over beam
angles 7..353 deg (21.5 of the 36 s), fading only in the creeping ends.

**Fly both ends.** 0.048 N m bounds the force the claw must deliver; the real
bearing is probably near the other end, and section 6's untested idea is that
friction DAMPS the contact-phase slosh -- so also fly
`PEGASUS_VALVE_FRICTION=0` (frictionless; v1-v6's 0.01 + 0.02 is in between).
Neither flown yet (box-pushing runs had the sim).

## 8. The 2026-10-17 tuning campaign (user: "make sure the physical parameters are in, then tune")

Harness, all in `tools/`: `try_valve.sh <tag> [key=val ...]` (regenerate the valve yaml, apply
the overrides -- `push_pull_*` into the planner block, anything else through
`wb_l1_set_gains.py` -- fly `run_valve.sh` headless with the release-at-open sequence, restore
the yaml; `PEGASUS_VALVE_*` pass through and land in `runs/<tag>.args`), the chains
`chain_f.sh` / `chain_c.sh` / `chain_d.sh` (a clean slate between flights; run them with
`setsid`, the kill-stale step kills its invoking shell). `valve_score.py`'s `[105]` label fixed:
it is `chi_free` (1 = free, 0 = CONTACT); the contact flag WAS on in every turn.

**The valve yaml has its OWN controller tune since today** (`make_valve_turning_yaml.py`
OVERRIDES / ADDITIONS, seeded with the box push's 2026-10-09 values -- the first regeneration is
value-identical, every changed line now carries a `VALVE-TURNING` marker). The planner block
gained `push_pull_contact: true` (false = the turn flown in the FREE phase).

### 8.1 The user's windowed run and the conservative bearing (v7)

Flown windowed at RTF 0.98-1.00 with the section-7 values (Isaac printed `rotor 204 g, bearing
damping 0.0, Coulomb friction 0.048 N.m`): grip, CONTACT on, then from the jaws' close a
0.88 Hz ROLL oscillation grew (std 5.5 deg over 2-10 s of the turn, v3's was 0.26) with the arm
swinging across both stops (q2 -9..45, q3 -25..49), the node's 15 deg watchdog at 11.8 s into the
turn. The beam followed its plan (lag <= 7 mm).

### 8.2 Phase 1 -- the bearing is NOT the lever (shipped tune, one friction per flight)

| run | friction [N.m] | outcome | beam | base lead max | q2 min / q3 max in the turn | tilt max in the turn |
|---|---|---|---|---|---|---|
| v7 (windowed) | 0.048 | watchdog at 11.8 s | -- | -- | -8.9 / 48.6 | 15.6 |
| f048_r2 | 0.048 | **complete** | +365.5 | 64 mm | -20.0 / 50.0 | 5.7 |
| f032 | 0.032 | watchdog at 31 s (after the full turn) | +367 then -270 | 76 mm | -20.0 / 50.0 | 36 |
| f016 | 0.016 | **complete** | +368.9 | 58 mm | -20.0 / 50.0 | 5.4 |
| f000 | 0.0 | watchdog at 12.7 s | ran away (3051 deg after the release) | 20 mm | 5.8 / 21.7 | 29.5 |

Completion is 2 of 4 and not monotonic in friction; every run with friction put the arm ON BOTH
STOPS in the middle half of the turn (12-36 s, the fast half) and the aborts are the runs whose
tilt grew while the arm sat there (v6 and f032 identical: the beam turned the full revolution,
then 15 deg at the end). v7 was the same marginal contact phase seen from the other side of the
scatter, not a new failure. The frictionless end fails differently: nothing damps the slosh, the
beam runs away.

### 8.3 Phase 2 -- four single-variable changes at 0.048 N.m, 0 of 4

| run | change | outcome | base lead | q2 min / q3 max |
|---|---|---|---|---|
| c_hold | `wb_l1_contact_hold_translation` true | watchdog; the beam turned the WRONG way (-216 deg) | 72 mm | 11 / 16 (arm free, base gone) |
| c_free | `push_pull_contact` false (the turn in the FREE phase) | watchdog after 457 deg | 64 mm | -19 / 50 |
| c_t48 | `push_pull_push_time_s` 48 | watchdog after 428 deg | 60 mm | -18 / 50 |
| c_oct05 | `wb_l1_omega_c_t` 0.5 | **force guard 7.2 N** at a 214 mm lead | 214 mm | -20 / 50 |

The lead is NOT rate-dependent (60 mm at the 48 s turn's 18 deg/s vs 64 at 36 s / 24.6 deg/s --
section 5.3's "2.5 mm per deg/s" does not hold). Holding the translational observer rows lets
the base walk off (the box's pl_4 again); a slower observer walks it further.

### 8.4 What the swing is

Over f048_r2's turn, at 0.5 s samples: q2 correlates -0.93 with the base's TANGENTIAL error and
+0.88 with its RADIAL error (the two co-vary), -0.23 with the vertical; the fit is
`q2 = -463 deg/m * e_tang + 122 deg/m * e_z`, `q3 = +510 * e_tang - 697 * e_z` -- a 20 mm std of
base wander becomes a 9 deg std of joint swing because the [0, 10, 20, 0] arm (fold 30 deg,
83 % extended) is badly conditioned for it. The base errors: tangential +12 mm mean (std 21),
radial -23 (std 16), vertical **+28 mm mean (std 3)**; the tangential contact force is -0.23 N
mean (truth), so `F/k_x` = 5 mm does NOT explain the lead -- the wander is the base loop's own.
The swing is SYMMETRIC (q3 9..50 around 20), so re-splitting the pose cannot buy margin, and a
more folded arm (the box's beta 60) is barred by the landing gear over the beam. Every k_x/k_v
pair flown on the interaction rigs so far (50/12.58, 32/10, 64/14) keeps zeta ~0.44: the base
loop's DAMPING is the one untouched lever -> phase 3 (`chain_d.sh`: `wb_k_v` 20 / 25, a 24 s
turn, a repeat).

### 8.5 Phase 3 -- the base loop's damping: NOT a lever (0 of 4)

| run | change | outcome |
|---|---|---|
| d_kv20 | `wb_k_v` 12.58 -> 20 | watchdog in FREE flight, during Ready (tilt 14 deg) |
| d_kv25 | `wb_k_v` 25 | watchdog in free flight, at Go To Start |
| d_t24 | `push_pull_push_time_s` 24 | force guard 9.0 N (base lead 100 mm) |
| d_kv20_r2 | `wb_k_v` 20 | force guard 6.9 N (base lead 205 mm) |

A stiffer velocity loop destabilises the vehicle before it reaches the fin -- the box push's own
note ("k_v 20 left DIRECT in free flight") again.

### 8.6 Phase 4 -- the translational observer's bandwidth UP: THE TURN WORKS (4 of 4 full revolutions)

The one large effect so far had a direction: `wb_l1_omega_c_t` 1.0 -> 0.5 tripled the base's lead
(c_oct05). The valve inherited the box push's 1.0, chosen there so the observer would not
integrate a STUCK box's static friction; the valve's handle is free on its bearing.

| run | omega_c_t | beam | base lead max | q2 min / q3 max in the turn | outcome |
|---|---|---|---|---|---|
| e_oct2 | 2.0 | +370.5 | 42 mm | -19.2 / 49.2 | **complete** |
| e_oct2_r2 | 2.0 | +377.6 | 48 mm | -19.2 / 49.9 | **complete** |
| e_oct29 | 2.927 (the mirror's) | full turn | **20 mm** | **-12.3 / 43.9** (off the stops) | watchdog 0.9 s into the EXIT |
| e_oct2_p15 | 2.0 + pose [0, 15, 25, 0] | full turn | 45 mm | -11.4 / 50.0 | watchdog 1.3 s into the exit |

**The new failure is in the exit, after the turn and the release:** within the first second of
Exit To Turn the wrist roll q4 swings 30-45 deg -- NOT commanded (the planned arm pose stays
[0, 10, 20, 0] for the first 2 s of the exit; the EE heading error is <= 3.6 deg at the release
and +17..+30 deg 0.5 s later) -- with the open jaws still around the fin; the jaws strike it
(contact 2.5 -> 7.5 -> 17 N), tilt 27-39 deg, watchdog. The completed runs show the same swing
(q4 to 44 / 51 deg); their jaws happened to clear first. A joint-4 torque left over from the
contact phase against the soft EE heading (`wb_ky_psi` 0.25 -> ~0.044 N.m/rad at q4: 0.05 N.m
of residual is ~65 deg).

Also measured offline (`transition_planner.arm_fk_model`, the seating offset of +7 / -7 deg on
q2 / q3 applied): no turn pose the gear allows tolerates more than 5-19 mm of TANGENTIAL base
wander before a joint stop or q3 < 3 deg (the elbow-singular branch); the flights show 20 mm std
and 40-60 mm peaks. The wander has to shrink -- which the observer bandwidth does.

### 8.7 Phase 5 -- the stiff EE heading (the 2026-10-16 wrist fix): 0 of 4, all in the TURN

`wb_ky_psi` / `wb_dy_psi` 2.5 / 0.93 + `wb_l1_stuck_hold_j4` true, with omega_c_t 2.927 (x3) and
2.0 (x1): every run aborted in the middle of the turn (18-24 s, beam 226-347 deg, the arm onto
q3's +50 stop as the lead built). A stiff heading fights the fin through the turn. The heading
stays soft for the valve.

### 8.8 Phase 6 -- the decision: omega_c_t 2.0, the soft heading (SHIPPED)

| run | change (soft heading) | beam | base lead | q2 min / q3 max | outcome |
|---|---|---|---|---|---|
| h_oct29_sh | 2.927 + `wb_l1_stuck_hold_j4` | full turn | 21 mm | -8.0 / 44.0 | watchdog in the exit |
| h_oct2_r3 | 2.0 | +373.4 | 24 mm | -11.8 / 42.0 | **complete** |
| h_oct29_sh_r2 | 2.927 + stuck hold | full turn | 38 mm | -19.9 / 50.0 | watchdog in the exit |
| h_oct29_r2 | 2.927 | full turn | 32 mm | -12.3 / 42.4 | watchdog in the exit |

**Tally at the conservative bearing (0.048 N.m):** omega_c_t 1.0 (the box push's) 2 of 6
complete; **2.0: 3 of 3**; 2.927: 0 of 4 (every one turned the full revolution and lost the
exit's wrist roll); the stuck-joint hold does not cure that roll. Shipped in
`make_valve_turning_yaml.py`: `wb_l1_omega_c_t` 2.0, everything else the seed. The exit's
uncommanded wrist roll still happens at 2.0 (q4 to 44-51 deg) -- the jaws clear the fin first
there; a planner-side cure (climb before the wrist is free, or hold q4 through the first second
of the exit) is not built. Results campaign: `results/utils/run_valve_turning_campaign.sh`.

### 8.9 Phase 7 -- THE EXIT, fixed: open the jaws while the claw is still world-held

On the results timetable omega_c_t 2.0 completed 2 of 7 (runs set aside in
`results/simulation_results/Isaac_simulation_data/valve_turning/previous_release_at_open_20261018/`): every loss the
exit's wrist roll. **The law did not cause it** -- through the release and exit its own j4
torque OPPOSES the roll (+0.25..+0.30 N.m once the heading error is 30 deg), the contact flag
was already free, and the estimate feed-forward on j4 is ~0.01 N.m. The roll is EXTERNAL: with
`--release-at-open` (the arm GS's 2026-10-17 Exit for the box push) the claw is handed to the
CoM anchor (1 s blend) BEFORE the jaws open, so the open jaws ride the base's 20-50 mm wander;
the fin cannot move radially, a jaw presses it ~18 mm off the wrist axis, and against the soft
EE heading (~0.044 N.m/rad at q4) 1 N of jaw force is ~23 deg of roll, which levers the jaw
further in (7-17 N, tilt 27-39 deg). The run that survived simply drifted the other way.

| run | sequence | anchor blend | outcome |
|---|---|---|---|
| i_old_b3 | open while WORLD-held, then Exit (the driver's default) | 3.0 s | **complete** |
| i_old_b3_r2 | same | 3.0 s | **complete** |
| i_rel_b3 | release first (`--release-at-open`), then open | 3.0 s | watchdog in the release |
| i_old_b1 | open while world-held, then Exit | 1.0 s | **complete** |

Shipped: `wb_ee_anchor_blend_s` 3.0 (a VALVE-TURNING ADDITION) and the campaign's exit =
open-then-exit (`RELEASE_AT_OPEN=0` in `try_valve.sh` / `run_valve_turning_campaign.sh`).
**The valve is the opposite of the box push here:** the box push needed the claw released
before the open (its open claw, world-held, absorbed the base's drift onto q3's guard); the
valve needs it held (its open jaws must stay centred on a fin that cannot move radially). **The
arm GS's Valve Turning tab still uses the box's release-first Exit** (it shares
`PushPullPanel`) -- on hardware, until that tab gets the valve's sequence, release the valve
with the gripper's Open button, wait ~1 s, then press Exit. Not changed in the GS tonight.

## 9. The coordinated-admittance comparison rig on the valve (2026-10-18)

New, all additive: `make_coordinated_yaml.py --profile valve_turning` (the CA box-pushing tune;
the whole-body valve yaml's planner block verbatim except the rig's gentler transitions
0.20 m/s / 0.08 m/s^2 / 0.20 rad/s and its contact-phase span), the CA stack and launcher accept
`WB_SIM_PROFILE=valve_turning`, the CA launcher passes `ARM_GS_START_TAB` to the arm GS, a scene
wrapper `scripts/indoor_sim/start_t650_aerial_manipulator_coordinated_admittance_valve_turning_sitl.sh`,
and `tools/try_valve_cadm.sh` (the runner).

**Round 1, 0 of 2 -- a DRIVER bug, not the rig:** both timed out at the grip gate ("the claw
never came within 25 mm / 4 mm across the fin (last 3 / -158.7 mm)"). The box push's
`--grip-live-box` gate (the CA rig's) compared the claw and the body in WORLD x / y -- right for
the box (yaw 0, the grasp point over its origin), wrong for the valve, whose grasp point is
0.160 m out along the beam in the rotating rotor frame (-158.7 mm = the fin's radius). Fixed in
`pl_mission.py`: the gate in the body's frame minus `--grip-ee-offset` (default 0, 0 = the box,
numerically unchanged); the valve passes 0.160,0.0006.

**Round 2, the mission flies:** both completed the eight steps; ca_v2 turned the beam **+374
deg**, ca_v2_r2 only +288 (short). Its decoupled base lags the turning reference by 156-236 mm
and its heading by 41-42 deg (the yaw loop cannot follow the 24.6 deg/s peak); its arm, held in
contact, stays far from its stops (q2 >= 3.5, q3 <= 23.5). Its Go To Land Start takes 35.9 s at
the gentler bounds (the whole-body rig 23.9 s), so both campaigns fly ONE landing slot of 40 s
(the box comparison's rule); the whole-body runs flown at 28 s were set aside
(`previous_land_slot28_20261018/`).

## 10. The video -- not recorded (2026-10-18)

The windowed scene runs at RTF 0.83-0.84 during the turn (headless 1.00), and the wall-clock law loses
the contact phase to it: 0 of 6 windowed takes completed (box_pushing_video_20261017/README.md, last
section). The user's own windowed run of 2026-10-17 (v7) is the same failure.

## 11. Path following for the turn (2026-10-18, user request: "still use 360 degrees, apply the path following fixes")

**What was built** (fsc_trajectory_planner, uncommitted; default OFF, the scheduled turn unchanged):
`push_pull_follow` flies the turn at a GOVERNED PROGRESS instead of the wall clock -- the same rigid rotation,
its schedule slowed or held. `Trajectory::evalScaled(tau, rates)` (trajectory.hpp; Pause / Hold / Sequence /
Turn implement it) chains the progress rates through snap (Faa di Bruno), so the reference stays compatible;
`ProgressGovernor` (push_pull.hpp) gates the target rate asymmetrically (falls at once, rises at most
1 / `recover_s`), then a 4-stage chain filter (`bandwidth`, 4 rad/s) -- rate always in [0, 1], never reverses.
Three channels, each low-passed `filter_s` (1 s) and `[soft, hard]` deg (rate 1 at soft, 0 at hard): **lead** =
the planned claw rotation minus the MEASURED valve's (obj_0 yaw; a one-tick jump > 30 deg is rejected as a
mocap flip), **tilt**, **wrist** |q4 - q4 planned|; a stale feed holds; a hold longer than `hold_timeout_s`
(6 s) aborts (release, climb, fold). Telemetry: push_pull/arrival_error [13..20], recorded by pl_mission.py
AFTER the step column. gtests: rate 1 is the scheduled turn bit for bit; a slow / hold / resume progress keeps
the CoM on its circle, the arm still and the derivative chain FD-consistent; planner suite 67/67.
Scored by `tools/follow_score.py`; flown by `tools/chain_pf.sh` / `chain_pf2.sh` (open-then-exit, no timetable).

**The first version reacted to the contact ring.** pf_n1 (unfiltered channels, lead [4, 8]): the lead jitters
+-4 deg at ~1 Hz with the arm's swing, so the governor cut the rate to 0.63 in the fast middle of the turn;
the run then failed at its end. Shipped thereafter: every channel low-passed 1 s, lead [6, 12].

**Results, the experiment's bearing (0.048 N.m, no damping, 204 g)**:

| run | follow | outcome | rate min | force p95 / max | on a stop |
|---|---|---|---|---|---|
| pf_n1 | on (unfiltered) | watchdog at 344 deg (end of turn) | 0.12 | 3.7 / 10.9 N | 15 % |
| pf_n2 | on | **complete** | 1.00 | 2.3 / 6.3 N | 4.8 % |
| pf_n3 | on | **complete** | 1.00 | 1.9 / 3.3 N | 3.7 % |
| pf_n4 | on | watchdog at 362 deg (end of turn) | 0.72 | 3.3 / 10.1 N | 11.9 % |
| base_n1 | off | **complete** | -- | 2.6 / 6.8 N | 9.2 % |
| pf_win1 | on, windowed | force guard 6.0 N, 7 s into the turn | 0.96 | -- | -- |
| pf_win2 | on, windowed | force guard 10.8 N, 8 s into the turn | 0.43 | -- | -- |

With the filtered channels the governor never acts on a nominal turn (rate 1.00 throughout pf_n2/n3, and pf_n4
until its last 2 s) -- it flies the schedule exactly, so nominal reliability is the scheduled turn's, which this
day's runs put at 3 of 4 headless (base_n1 + pf_n2 + pf_n3 complete, pf_n4 not) against 7 of 8 earlier.
Windowed (RTF ~0.83 in contact) it does not help either: 0 of 2 governed vs 0 of 7 scheduled; the contact
phase falls apart within 1-2 s, faster than a 1 s-filtered governor (both ended in a CONTROLLED force-guard
abort, not the tilt watchdog with closed jaws -- but that guard is the existing one).

**Why it cannot fix the nominal failure: the arm's fold follows the valve's ANGLE, not the turn's speed.** Every
run, completed or failed, folds the arm the same way through the middle of the turn (5-s means q2 10 -> -10..-15,
q3 20 -> +45, against the -20 / +50 stops, a ~1 Hz ring on top) and swings back past the pose as the turn
decelerates; the completed runs settle at q2/q3 ~[10, 20], the two failures overshoot to [+25..+28, +6..+9] --
toward the elbow-singular branch -- and the ring grows there to the trip. Binned by beam angle, the 48 s turn
(peak 18 deg/s, c_t48) folds as far (-12..-17) as the 36 s ones (-10..-15). Slowing the progress -- the only
lever path following has -- does not reduce it (pf_n1's 37 % mid-turn slow-down left the fold unchanged).

**Where it does help: a valve that resists with speed (STRESS CASES, not the experiment's bearing):**

| run | bearing (env override, this run only) | follow | outcome | force p95 / max |
|---|---|---|---|---|
| pf_visc | damping 2.0 N.m.s/rad (5.4 N at the fin at the scheduled peak) | on | **complete**, 41.9 s, rate >= 0.50 for 17 s (lead) | **3.1 / 4.5 N** |
| base_visc | same | off | force guard 5.6 N at 270 deg, arm buckled (q3 -56) | 3.8 / 16.6 N |
| pf_stuck / base_stuck | friction 1.0 N.m | on / off | both complete, governor idle | 2.5 / 6.6, 2.4 / 4.8 N |

The governed turn settled where predicted (rate ~0.55 at ~2.9 N: lead x claw stiffness 120 N/m balancing the
speed-proportional drag). **The scene's Coulomb friction cannot make a valve stick**: the momentum clamp
caps the friction torque at I|w|/dt (0.26 N.m at peak speed), so "1.0 N.m" flies as a heavier drag and both
runs completed; a sticking valve needs a new scene mechanism (not built).

**Decision left to the user: `push_pull_follow` stays false in the valve yaml.** It costs nothing nominally
(inert) and bounds the load if the real valve turns out much heavier or speed-sensitive than measured, but it
does not make the nominal sim more stable. The nominal failure is the angle-locked fold + its release at the
end of the turn -- a pose / controller-damping question, not a timing one.

## 12. A taller fin and a more folded pose -- screened offline, does NOT help (2026-10-18)

User proposal: print a taller handle so a more folded arm pose clears the landing gear, keep the jaws
closed. `tools/pose_fin_screen.py` carries the CLAW DISPLACEMENT the airframe imposed at the flown pose
(body frame, through the exact FK: the mid-turn fold q2 10 -> -11 / q3 20 -> +45 = **44 mm toward the
body**, the failures' end-of-turn swing-back q2 +27 / q3 +7 = 37 mm away and 31 mm up; a +-7 deg ring)
to every candidate pose, and asks for the margin left to q2 [-20, 45] / q3 [10, 50] and the fin
extension that keeps the skids 10 cm over the beam:

| pose [q2, q3] | fin +H | deg per mm fore-aft | worst margin |
|---|---|---|---|
| [10, 20] (flown) | 0 | 0.84 | -10 deg |
| [5, 25] | 0 | 0.77 | -5 (the best) |
| [20, 30] | +38 mm | 0.82 | -12 |
| [30, 30] | +83 mm | 0.84 | -16 |
| [20, 40] | +70 mm | 0.75 | -20 |

The arm's joint cost per mm of fore-aft claw motion is ~0.8 deg/mm at EVERY pose (folding does not
condition it better in this direction -- the 2026-10-18 claim that it would was wrong), and folded poses
sit nearer q3's +50 stop, so the 44 mm retraction runs q3 past it. ~80 mm of fore-aft swing is ~65 deg of
joint motion against q2's 65 and q3's 40 deg of range: NO pose fits it, so no fin height helps. The swing
is the airframe's offset from its circle while it turns (present at 18 and 24.6 deg/s alike); that offset
has to shrink. Assumes the offset is the same at every pose -- a ranking, not flown.

**Flown 2026-10-18 (user: "try a different arm pose"): [0, 5, 25, 0], 3 of 3 complete** (`tools/chain_pose.sh`,
measured bearing, scheduled turn, jaws closed, open-then-exit; the planner's push pose only -- same fold
q2 + q3 = 30, so the claw's pitch and grasp are unchanged, skids ~14 cm over the beam). Against the same
day's [0, 10, 20, 0] runs (base_n1, pf_n2, pf_n3 complete; pf_n4 failed -- the governor never acted in
any of them, so all four fly the scheduled turn):

| | [0, 5, 25, 0] | [0, 10, 20, 0] |
|---|---|---|
| complete | 3 of 3 | 3 of 4 |
| q3 minimum (the swing-back toward the elbow branch) | +17.5 / +12.9 / +8.5 deg | +14.6 / +6.0 / +9.0 / **-7.9** |
| mid-turn fold, 5-s means | q2 to -11..-17, q3 to +43..+50 | q2 to -10..-12, q3 to +42..+46 |
| on a stop | 19.7 / 3.2 / 5.8 % | 9.2 / 4.8 / 3.7 / 11.9 % |
| force p95 | 3.2 / 1.9 / 2.4 N | 2.6 / 2.3 / 1.9 / 3.3 N |

As the screen predicted, the pose moves margin from the fold side to the swing-back side: q3 stays further
from 0 at the end of the turn (the failures' point), and rides its +50 stop more in the middle (one run
19.7 % of the turn). 3 of 3 vs 3 of 4 is not a measurable difference. The bend itself is unchanged.
**ADOPTED the same day (user: "apply the better arm pose")**: `make_valve_turning_yaml.py` now writes
[0, 5, 25, 0]; the whole-body and the coordinated-admittance valve yamls regenerated (the CA one copies
the task block; its diff = the pose + the path-following keys, off). Runs flown before keep their own yaml copies.

## 13. The airframe's ~4 cm offset while turning, read from the recorded runs (2026-10-18)

Sections 8.4 and 12 left one open problem: the airframe sits 30-60 mm off its circle through
the fast half of the turn, and the world-held arm absorbs it onto its stops. Read from the
npz files of base_n1, e_oct2, e_oct29, h_oct2_r3, pose525_1..3 and the failed pf_n4 with the
law's own debug stream (`tools/turn_drift_decomp.py`, `tools/turn_drift_omega_c.py`,
`tools/turn_drift_omega_c_fd.py`; the columns follow the whole-body client's
`wb_control_debug` layout: [45..47] x_cd, [48..50] the law's x_c, [28..30] e_R, [31..33]
d_hat_t, [34..36] d_hat_r, [65..67] the deadbeat rotational residual):

1. **It is a CoM error, along the NOSE, and it is the structural attitude-error offset
   `T e_R / k_x`.** Section 8.4's "vertical +28 mm mean" was the BASE against the CoM
   reference (the arm offset moves with the fold); the law's CoM holds its height to 2 mm.
   In the body frame the CoM error through the fast half is +25..+44 mm LONGITUDINAL (the nose
   = the tangent; the base leads the fin) and +3..+15 mm lateral, and on every run it pairs
   with a standing attitude error about the model x axis (= the actual body PITCH axis) of
   -25..-55 mrad: 38 N x 0.038 / 50 = 29 mm against the 32-35 mm measured (base_n1),
   38 x 0.055 / 50 = 42 vs 44 (pose525_1). The same pairing explains the free-hover offset
   every run shows before the turn (-12 mm lateral against +13 mrad about the roll axis). The
   fold then follows: the claw is pinned, so a CoM 3 cm ahead folds the arm ~25 deg.
2. **Not feedback.** Odometry equals the truth here: attitude to 0.03 deg, position to
   0.5 mm (raw mocap). The heading tracks to 1.0-1.6 deg mean, 2.4-4.1 max.
3. **Not the 2026-09-26 circle mechanism.** Reconstructing the law's `omega_0c` from the
   logged quantities (exact chain, model `e_ax`), the rate error the law sees about the pitch
   axis averages 2-7 mrad/s, i.e. -(k_w/k_R) e_w = 1.5-5 mrad of the -17..-27 mrad logged; a
   `dhat_rate_ff` fix buys at most ~15 % here. Both observers are converged: the rotational
   filter (0.74 rad/s) lags the deadbeat residual by ~10 mN.m while the pitch moment it is
   learning grows -20 -> -80 mN.m with the fold.
4. **What the P-term balances.** `k_R e_R + k_w e_w` about the pitch axis = -32..-37 mN.m
   through the fast half. ~10 mN.m is the observer's lag, ~2 the rate term; ~25 mN.m is not
   attributable from the log alone (the law's own feed-forward / coupling terms under a
   0.43 rad/s yaw with the arm swinging +-7 deg at ~1 Hz are the candidates). A pinned-claw
   plant in `utils/wb_entry_sim.py` (a stiff spring from the claw to a point on the planned
   fin circle plus the bearing's Coulomb friction) replaying the recorded reference would
   expose every term of the equilibrium -- that is the next analysis step, and the place to
   rank gain or feed-forward candidates before flying them.
5. **It scales with the yaw RATE within a run** (2-s bins: e_R0 -4 at 4 deg/s, -25 at 14,
   -35..-45 at 17-24, back to +10 at the end), and d_hat_t's body-frame longitudinal part
   drops ~0.8 N when the jaws take the fin (the fin's reaction), with ~1 N lateral. The
   pitch moment of that force at the claw's ~0.27 m lever below the CoM is the right order
   for the ~60-80 mN.m the observer learns.
6. **The planner's FK claw differs from the truth claw by up to 8-16 mm** (mean -2.5 mm along
   the nose, std 3-5 mm): the known pose-dependent model-vs-asset mismatch (2026-09-18,
   0.7 mm at home, ~1 cm at a circle rest). It does not drive the lead, but it is the error
   budget of the grip gate and of `push_pull_ee_offset` on hardware.

**Levers this points at, untested, in the order I would fly them** (each a campaign of >= 3
flights; the contact phase scatters):

| | what | why | risk |
|---|---|---|---|
| L1 | a 48-60 s turn at the SHIPPED tune (omega_c_t 2.0) | the pitch error and the lead scale with the yaw rate; v5 (72 s) and c_t48 both flew at omega_c_t 1.0, BEFORE the tune that stopped the slosh | the slow slosh may return |
| L2 | omega_c_t 2.927 + open-then-exit | the smallest lead of the campaign (20 mm) with the arm off its stops; its 0 of 4 were all the exit's wrist roll, which 8.9 fixed at 2.0 -- the combination was never flown | the exit |
| L3 | q3 guard + abort-from-measured in the valve yaml (`push_pull_guard_q3_min_deg` 10, `push_pull_abort_from_measured`, `_abort_back`, `_abort_release_wait_s`) | the box campaign's 2026-10-16 guards; pf_n4 reached q3 -7.9 deg. The BOX-LAG guard does NOT fit a turn (the rotor's origin never moves) -- leave it 0 | none (safety) |
| L4 | the attitude channel: k_R 1.6 -> 2.0, omega_c_r 0.74 -> 1.0 | the offset is e_R x T / k_x and e_R = (uncompensated moment) / k_R | both rejected on OTHER rigs (pick-and-drop 1.25 diverged; the box push's 2.134/1.567 hover abort); the valve rig never swept k_R |
| L5 | contact-moment feed-forward at the attribution bandwidth (code: the rotational rows' contact image `S_e^T F_hat` at omega_x instead of omega_c_r) | removes the ~10 mN.m lag part and, if the unresolved ~25 is contact-related, more | a law/observer change, parity fixture, offline first |
| L6 | `dhat_rate_ff` | <= 15 % | low value |

**Also fly at the shipped tune: `PEGASUS_VALVE_FRICTION=0` and 0.016.** The real bearing is
<= 32 mN.m and probably a few; f000 and f016 (8.2) flew at omega_c_t 1.0 -- the low-friction
end has never been flown at the configuration that will go to hardware.

### 13.3 The windowed simulation: the viewport is no longer the cost, the contact phase stalls on its own (2026-10-18)

Measured first, from the rotor mocap's wall-clock rate (62.5 Hz at real time) in the recorded npz
files: the windowed runs pf_win1 / pf_win2 (section 11) ran at RTF **0.81 / 0.90 already in free
flight** (Go To Start, Ready) and 0.18-0.99 in 2-s bins of the turn -- the viewport's fixed frame
every 8 steps cost more than the ~0.8 ms/step of slack the physics leaves. The headless base_n1 runs
1.000 in free flight and **0.98 mean / 0.91 min bin in the turn**: the contact phase has stalls of
its own, headless too.

**Fix in 06 (`PEGASUS_RENDER_ADAPTIVE`, default 1, baked by `start_single_drone_x650.sh`; the
RTF line now prints frames / fps / frame cost / skipped):** under the pacer a frame is drawn at the
`PEGASUS_RENDER_EVERY` cadence only while the simulated time is not behind the wall clock by more
than 2 ms, and never later than `PEGASUS_RENDER_MAX_GAP_S` (1.0) after the previous one. Physics
first, viewport best-effort: a frame costs the plant at most one render of lag, repaid before the
next is allowed. Standalone scene (no stack): RTF 1.000 at 30 fps, 4 ms frames, 5-17 skipped per
10 s. **Full windowed flight win_adapt2 (`tools/chain_win.sh`, the shipped configuration):**

| phase | RTF (pacer, 10-s windows) | fps | frames skipped / 10 s |
|---|---|---|---|
| free flight (take-off, Go To Start, Ready, exit) | **1.000** | 25-31 | 0-13 |
| the turn | **0.925-0.972**, 7-13 re-anchors (stalls of 150-190 ms) per 10 s | 1.2-4.4 | 245-283 |

So with the window the free-flight cost is gone (0.81-0.90 -> 1.000) and in contact the gate
correctly starves the viewport (1-4 fps) -- but the plant still runs at 0.93-0.97 there with
**~1 Hz stalls of 150-190 ms** that are not the drawing (12 frames x 9 ms is 0.1 s of a 10 s
window). The headless turn carries the same stalls at a smaller size (0.98 / 0.91). The source is
in the scene or the physics of the contact phase (candidates: the per-contact-point Python loop
of the contact-truth report, the waypoint scene view, the debug draw's rebuild, PhysX on the
closed chain vehicle-arm-jaws-fin-rotor), to be found with `PEGASUS_PROFILE_START` placed inside
the turn (`win_prof1`). The flight itself aborted at the END of the turn (q3 3.9 deg at the
deceleration swing-back, the pf_n4 mode) -- one run, not attributable to the window.

**win_prof1 (windowed, profiler window inside the turn): COMPLETE** -- beam +362 deg, claw error
5.6 mm mean / 13 max, base lead to 30 mm, tilt max 4.4 deg, the arm on both stops in the fast
half exactly as headless (the first windowed completion of the campaign: 1 of 2 with the gate vs
0 of 8 before). The profile (`runs/win_prof1_profile.txt`) shows NO Python hotspot: window A
4.14 ms wall per physics step (the app's callback 0.71 ms, PhysX ~1.2, Pegasus vehicle update
0.4, rclpy spin 0.5, publishes 0.2), window B at RTF 1.000 under cProfile. The deficit is
DISCRETE: the rotor mocap stream shows exactly 3 gaps in the 38 s turn, **170-171 ms each, 10.6 s
apart** (8.9 / 19.5 / 30.1 s into the turn), and the pacer logs one 128-155 ms stall per 10-s
window in free flight too; headless base_n1 has none > 60 ms and the standalone windowed scene
none either -- so the stall needs the window AND the live stack. The waypoint-label hook is not
it (its redraw is signature-gated). The 10-s period points at Python's generation-2 garbage
collection over Kit's object graph (allocation-driven, so present only with the stack's message
traffic; longer with the window's UI graph). 06 now does `gc.collect(); gc.freeze()` once the
scene plays (`PEGASUS_GC_FREEZE`, default 1) and the pacer's RTF line reports the longest step,
whether it drew a frame, and the GC's collections -- `win_gc1` was the test, and **it was the GC**:

| | win_prof1 (gate only) | win_gc1 (gate + gc.freeze) |
|---|---|---|
| mocap gaps > 60 ms, whole flight | 3 x 170 ms in the turn, 10.6 s apart; 1 per 10 s in free flight | **0 in every phase** |
| longest loop step, free flight / turn | 150-200 ms | **9-12 ms (a frame) / 18-23 ms** |
| gen-2 collections | (unmeasured) | the one explicit `gc.collect()` before the freeze: 163 ms; afterwards 0-3 per 10 s, **1-2 ms** |
| RTF free flight / turn | 1.000 / 0.935-0.966 | 1.000 / **0.923-0.984**, re-anchors from lags creeping to 50-68 ms, no stalls |
| outcome | complete | **complete** |

The windowed tally is now 2 of 3 with the gate (2 of 2 with gate + freeze) against 0 of 8 before.
What remains in contact is a SMOOTH 3-8 % overrun of the 4 ms step (PhysX in contact ~1.2 ms + the
fixed Python stack: rclpy `spin_once` twice per step 0.5 ms, Pegasus sensors 0.4, publishes 0.2,
the law's own callback 0.7), the same order as headless's 0.98; the next 5 % would be one spin per
step or a lighter sensor set, not the viewport. Watch the window; judge the contact phase as
headless-equivalent, with a 3-8 % slow plant through the turn that the headless run shares at 2 %.

## 14. Before the first hardware turn: the safety audit of the current state (2026-10-18)

What is in place: the node's watchdogs (tilt 15 deg, rate 360 dps, drift 0.75 m), the planner's
tilt (15 deg) and filtered force (5 N) guards, the abort sequence (hold 0.5 s on the handle while
the GS opens the jaws, climb 0.30 m, fold home), the pilot-takeover fix (no NaN motors outside
OFFBOARD; the hardware stacks refuse a stale node binary), the obstacle keep-out (the beam's swept
circle) and the table keep-out. What is NOT, in the order it should be closed:

1. **The valve has never been flown on the FUSED (EKF2) feedback the hardware stack uses.** Every
   sim turn in this campaign ran `VALVE_FEEDBACK=raw`. On the box push the fused path lost every
   contact phase in sim (push_pull README 8: EKF2 position drifts 10-15 mm and PX4's attitude
   1-3 deg while the IMU is shaken by the contact -> a ~3 N phantom on a 1 N box). Fly
   `VALVE_FEEDBACK=fused` first; if it fails the way the push did, the hardware experiment needs the
   raw-mocap stack or the attitude-from-odometry option before anything else.
2. **The arm GS Valve Turning tab still exits release-first** (`PushPullPanel::kExit`:
   `push_pull/release`, then open, then exit) -- the sequence that lost 4 of 4 turns at omega_c_t
   2.927 and 5 of 7 on the results timetable to the wrist roll (8.9). The valve needs: open with the
   claw still world-held -> the gripper's OPEN report -> ~1 s -> Exit (the driver's
   `RELEASE_AT_OPEN=0`, 3 of 3). A ~10-line change in the tab (skip the release call when
   `valve_mode_`, keep `release_wait_ms_` ~1000 for that tab). Until then the operator rule is: the
   gripper's Open button, wait 1 s, Exit -- never Exit first.
3. **The valve yaml carries only the tilt and force guards.** Add the box campaign's
   `push_pull_guard_q3_min_deg` 10 (with `_box_persist_s`), `push_pull_abort_from_measured`,
   `_abort_back` 0.03 and `_abort_release_wait_s` 1.0 through `make_valve_turning_yaml.py`'s
   PLANNER_BLOCK (pf_n4 reached q3 -7.9 deg; every failure is the q3 swing-back toward the elbow
   branch). Leave `push_pull_guard_box_lag` / `_box_tilt_deg` at 0: the lag guard measures the
   body's ORIGIN displacement along the push and the rotor's origin never moves, so it would read
   the planned claw's travel and trip at once.
4. **There is no hardware valve profile** (`AM_HW_PROFILE` knows free_flight, pick_and_place,
   pick_and_drop, box_pushing). Generate `..._t650_valve_turning.yaml` the way
   `box_pushing_campaign_20261016/tools/make_hw_box_pushing_yaml.py` does (hardware free-flight file
   + this generator's OVERRIDES / ADDITIONS / PLANNER_BLOCK, fence 1.8), add the profile to both
   whole-body hardware stacks with the planner check, and MEASURE in the lab: `push_pull_table`
   (the valve yaml still carries the pre-10-16 table 1.125-1.275), `push_pull_obstacle` (the
   beam's swept circle, z_top = the beam top), the valve's height (`push_pull_ee_offset` z 0.179
   = 25 mm under the fin top). Define the rotor's mocap body ON the bearing axis at the
   plate-bottom plane with x along the beam toward the fin, or shift `push_pull_ee_offset` by the
   difference -- Get captures that body.
5. **The low-friction end at the shipped tune.** The real bearing is <= 32 mN.m and probably a
   few; f000 (0) and f016 (0.016) flew at omega_c_t 1.0. At 0 the beam ran away after the
   release; with the open-then-exit sequence that should be harmless, but it is unflown.
6. **The grip on a 5 mm fin.** The sim clamps a 20 mm shim. Bench-check that the foam pads hold
   >= 3 N tangential at the fin (nominal 0.3-0.8 N in the turn, 2-3 N p95 in the jolts, 5 N = the
   guard) without the fin twisting in the jaws (the wrist roll is only 0.04 N.m/rad stiff).
7. **Skids vs the fin.** On the T650 gear at [0, 5, 25, 0] the skids pass ~1 cm below the fin's
   top and 14 cm over the beam, the fin between them (0.28 m apart in the asset); a 5-7 deg tilt
   in the turn moves a skid tip ~1.5 cm. Check the real skid span and the fin's height before the
   first grasp.
8. **The arm's hardware guards.** q2 [-20, 45], q3 [-40, 50] (`external_torque_controller_
   hardware_aerial_pwm.yaml`): on +50 the position guard REPLACES the law's torque with its
   pull-back, which is what the sim's hard stop stands in for; the sim rides it 3-20 % of the fast
   half. Expect the same on hardware; it is the q3 swing-back (item 3) that must be guarded.
9. **The rotor's mocap can flip near the claw** (seen with the basket on 0918 / 1009). The law
   never reads obj_0; the planner's lead channel rejects > 30 deg one-tick jumps and Get has a
   flip gate; the beam-angle display may jump. The box-TILT guard would read a flip as 180 deg of
   tilt -- another reason to leave it off.
10. **The released beam swings back 135-180 deg** when the jaws open under load (v5, v6) -- keep
    people and cables out of the beam's circle, and expect the fin to come round.
11. **The entry handover** (0.3-0.9 m at the SAFETY -> DIRECT switch) happens before Go To Start:
    the Start mark must keep >= 1 m from the table and the valve (the sim starts 1.1 m from the
    axis).
12. **Before any of the above, the performance question stays open** (section 13): the arm reaches
    its stops in the fast half of every nominal turn, so one bad transient is a jam. The levers
    there (slower turn at the shipped observer, omega_c_t 2.927 + open-then-exit, the attitude
    channel) are the tuning campaign; the guards above are what makes a failed attempt land.

## 15. The safety pass and the campaign of 2026-10-18 afternoon (Opus)

Tools: `tools/guard_signals.py` (q3 minima and what happened after every abort / watchdog trip,
all runs), `tools/campaign_score.py` (one row per run: outcome, beam, the in-turn lead along the
nose, e_R, the arm's extremes and time on its stops, tilt, contact p95, and the 8 s after an
abort), `tools/chain_opus{1,2,3}.sh`, `tools/gs_valve_check.sh` + `gs_valve_harness/`.

### 15.1 A failed turn was a crash, not an abort

Every one of the 34 recorded failures ended 30-180 deg tilted in the 6 s after it, most of them
INVERTED. The chain, read off pf_n4, v6, win_adapt2: a ~1 Hz roll oscillation with q1 / q4
swinging +-25 deg in antiphase builds at the end of the turn; the NODE's 15 deg watchdog hands
over to SAFETY **with the jaws still closed on the fin** -- the planner's guard was also 15 deg, so
the node always tripped first, and nothing opens the jaws on a watchdog trip (the GS and the
driver open them only on a planner ABORT); the fin is torn out (contact 15-20 N) and the vehicle
falls ~1 m. Fixed, all four:

* **planner tilt guard 15 -> 10 deg** (valve yaml). Over all recorded runs a 10 deg / 0.1 s guard
  catches 27 of 34 failures 0.1-5.9 s before the watchdog and would have aborted 1 of 26 completed
  runs (completed turns peak 4.4-7.5 deg, free flight <= 6.3). No false trip in the 9 completed
  flights since.
* **the jaws open when the node leaves DIRECT with the claw on the handle**: the arm GS's Box
  Pushing and Valve Turning tabs (`push_pull_panel.cpp`, on the planner's NOT IN DIRECT edge
  after an on-handle state) and the sim driver (`pl_mission.py`). Offscreen check against fake
  planners: 13 of 13 (`gs_valve_check.sh`), the box tab's release harness still 11 of 11.
* **the Valve Turning tab's Exit = open with the claw world-held, Exit 1 s after the OPEN report**
  (`push_pull_valve_exit_wait_s`), no release call -- section 8.9's sequence; the Box Pushing tab
  keeps its release-first Exit. Layout unchanged (no snapshot to refresh).
* **the free-handle abort** (planner `push_pull_abort_release_world_held`, default false; true in
  the valve yaml): the claw stays world-held through the release wait and is handed to the CoM
  anchor when it ends; on a TURN `push_pull_abort_back` now retreats along the CURRENT nose (it
  used the grasp's tangent, i.e. sideways at 90 deg and into the fin at 180). Planner tests 17/17
  push-pull, 30/30 pick-place.

### 15.2 How the abort itself must work (injected and natural aborts)

| run | hold | release | after the release | outcome |
|---|---|---|---|---|
| s_abort20 (injected at 20 s) | measured | 1 s, world-held | straight climb | released cleanly (tilt <= 1.9 deg, jaws open at 1 s) -- then the arm sat on BOTH stops and the climbing claw caught the fin (8.8 N): 45 deg, DIRECT lost, 5.3 m away, upright |
| s_abort33 (guard at 10.8 deg, 14.6 s) | measured | 1 s, world-held | straight climb | the same: 12 N on the fin at +2 s, DIRECT lost, dipped to 0.27 m, upright |
| p_f000_1 (guard, frictionless bearing) | cut (planned) | 1 s, world-held | straight climb | the FREE beam coasted on after the release (216 -> 293 deg) into the held claw: wrist 71 deg, crash |

Then the two candidate motions after the release, from the PLANNED hold at the cut
(`from_measured` is the jammed box's fix: here the beam follows the claw to <= 7 mm, so the
planned claw is the claw, and the planned base unfolds the arm off its stops during the release):

| run | after a world-held release | outcome |
|---|---|---|
| s2_abort8 (8 s) | 1.5 s, back out 0.12 m along the nose, climb | **clean** (DIRECT kept, tilt <= 4.4 deg; q3 flicked to -3 deg for ~0.5 s as the jaws opened) |
| s2_abort20 (20 s) | same | the open pads DRAGGED the fin back 40 deg (7.8 N), tilt 13 deg, DIRECT lost -- SAFETY recovered at 0.91 m, landed upright 1.2 m away |
| s2_abort33 (33 s) | same | dragged the fin back 44 deg (7.6 N), 16 deg, DIRECT lost -- recovered at 0.73 m, upright |
| a_abort33 (33 s) | **2 s, straight climb** | **clean**: DIRECT kept, tilt <= 1.9 deg, 0.30 m up, arm home, landed |
| a_abort20_f0 (20 s, FRICTIONLESS, arm at q2 -16 / q3 48) | **2 s, straight climb** | **clean**: the hold unfolded the arm in 2 s, no contact on the climb, DIRECT kept |

**Shipped: the planned hold, a 2 s world-held release, the straight climb** (`push_pull_abort_back`
0.0, `_release_wait_s` 2.0, `_from_measured` false, `_release_world_held` true). A vertical climb
drags the fin along the axis the bearing holds; a tangential back-out rotates the beam.

**What no abort recipe fixes: an abort started with the arm already on its stops or swinging.**
Every such abort crashed (c_oct05, p_f000_1, s2_abort20_f0, a_abort20's natural force-guard trip
at 17.9 s, p_fused_2/3, both fa_w1_oct29). The 10 deg guard is what keeps a failure from reaching
that state -- and on the frictionless bearing the oscillation grew from 10 to 15 deg faster than
the 0.1 s persistence could act (s2_abort20_f0). Two things still made every failure SURVIVABLE
that used to crash: the jaws opening at the handover (a released fin; SAFETY then recovered in
s2_abort20 / s2_abort33) and the planner's guard firing before the node's watchdog.

### 15.3 The performance levers, flown (measured bearing unless stated, open-then-exit)

| config | runs | complete | lead along the nose [mm] | e_R [mrad] | on a stop [%] | tilt [deg] |
|---|---|---|---|---|---|---|
| shipped (omega_c_t 2.0, 36 s) + new guards | p_base_1,2 | 2/2 | 24-25 | -27..-29 | 5-9 | 5.3-5.5 |
| 48 s turn | p_t48_1..3 | 3/3 | 22-29 | -25..-33 | 5-11 | 5.2-6.8 |
| omega_c_t 2.927 | p_oct29_1..3 | **3/3** | **17-23** | **-19..-27** | **1-8** | 5.7-6.7 |
| friction 0.016 | p_f016_1 | 1/1 | 28.5 | -31.5 | 9.1 | 5.6 |
| friction 0.0 | p_f000_1 | 0/1 (guard at 20.8 s) | -- | -- | 19.5 | -- |
| **EKF2-fused feedback** | p_fused_1..3 | **0/3** | -- | -- | 17-22 | -- |

* **Neither the turn's speed nor the bearing's drag sets the offset**: the 48 s turn and a third of
  the friction leave the lead and e_R where they were. Within a run e_R tracks the arm's FOLD, not
  the rate (base_n1: -26 mrad at q3 36 deg, -32 at q3 46 deg at similar rates; -11 as the arm
  unfolds while the rate is still 6.7 deg/s) -- a loop: fold -> pitch error -> lead -> more fold,
  closed at the stops. The net force on the fin is only 0.3-0.5 N tangential + 0.3-0.8 N inward
  (the PhysX normal and friction sums cancel to that).
* **omega_c_t 2.927 (the hardware free-flight file's value) is ~20 % better on the lead and 3 of 3
  with the open-then-exit release** -- its 0 of 4 in 8.6 / 8.8 were all the exit's wrist roll.
* **The fused (EKF2) feedback the hardware runbook uses loses the valve 3 of 3**: the jaws closed on
  nothing (the claw 12 mm off along the fin and 5.5 mm across while the planner, on odometry + PX4
  attitude, reported it on target), and twice the fin was lost ~6 s into the turn and the arm
  folded through into the elbow branch (q3 -37 / -82 deg), a crash. The box push's 0 of 6 again
  (push_pull README 8): PX4's attitude drifts 1-3 deg in contact and the claw is 0.36 m out.
  **The fused hardware stack now REFUSES AM_HW_PROFILE=valve_turning** (ALLOW_FUSED_CONTACT=1
  overrides for an A/B) and warns for box_pushing: fly contact tasks on the RAW-mocap stack.
* The frictionless end failed (1 of 1) at the shipped tune -- the beam has nothing to stop it
  between the claw's pushes; with the real bearing (<= 32 mN.m, probably a few) expect the
  frictionless end's behaviour more than the 0.048's. Not enough runs to call.

### 15.4 A parameter the node never read

`wb_ee_anchor_blend_s` is declared in the whole-body client but was missing from BOTH
`WholeBodyLawConfig::members()` lists, so `loadMembers()` never read it: every flight blended the
EE anchor over the 1.0 s default. The valve's "3.0 s blend" (8.9) never flew -- i_old_b3 and
i_old_b1 were the same configuration and the 3 of 3 exit result is a 1.0 s result; the valve yaml
now writes 1.0, what flew. Every pick-and-place / pick-and-drop yaml writes 1.0 = the default, so
registering the key (done, with the free-axis anchor's build) changes nothing for them. A sweep of
every WB_PARAM in that header found no other unregistered key.

### 15.5 The hardware profile

`tools/make_hw_valve_turning_yaml.py` writes
`..._whole_body_l1_4d_direct_actuation_t650_valve_turning.yaml` (the hardware free-flight file +
this generator's OVERRIDES / ADDITIONS / PLANNER_BLOCK, fence 1.8; `--check`), and both whole-body
hardware stacks take **AM_HW_PROFILE=valve_turning**: they print the two keep-outs with a MEASURE
note (`push_pull_table` and `push_pull_obstacle` ship as the simulation's, valve axis at
(1.20, 0.00) -- set them in the generator from the lab), refuse the sim file, check the valve's 42
tuned / guard values, and refuse a planner without the free-handle abort. Verified by running only
the scripts' pre-launch part (cut verified, private ROS domain, nothing launched). **Before the
first hardware turn: rebuild fsc_trajectory_planner (+ fsc_autopilot_ros2) on the Orin and
utils_custom_ground_station on the laptop; fly the RAW stack.**

### 15.6 The free-axis anchor (law option, default off; NOT shipped -- 15.9)

`wb_ee_anchor_free_axis_weight` (whole-body node, new 2026-10-18; parity 15/15 unchanged with it
off): while the CONTACT flag is on, the world-held share of the EE reference also rides the CoM
error along one horizontal body axis -- `r_ed += (1 - w) wf (a . e_x) a`, `a` = the horizontal
unit of `R_0 * wb_ee_anchor_free_axis_{x,y,z}` (model frame, default the nose), with the velocity
chain exact and the acceleration chain's centripetal term kept (`tools/free_axis_fd_check.py`:
exact at a constant yaw rate; the dropped omega_0_dot terms are ~2 mN at a valve turn); blended in and out with the contact flag over
`wb_ee_anchor_blend_s`; debug [116] the weight, [117] the offset [m]. The idea: the base's in-turn
lead lies along the fin's TANGENT, the one direction the valve is free, so let the valve absorb
it instead of the arm.

| config | runs | complete | arm on a stop | CoM along the nose max [mm] | valve vs plan | tilt max |
|---|---|---|---|---|---|---|
| shipped (anchor off) | p_base_1,2, p_t48_1..3 | 5/5 | 5-11 % | 45-48 | +-4 deg | 5.2-6.8 |
| **anchor 1.0** | fa_w1_1..3 | **3/3** | **0 %** (q3 14-39) | **84-96** | **+29 deg mid-turn, +12 at the end (one)** | 3.8-5.1 |
| anchor 0.5 | fa_w05_1 | 1/1 | 0 % (q3 to 47) | -- | -- | 7.1 |
| anchor 1.0 + omega_c_t 2.927 | fa_w1_oct29_1,2 | **0/2** | 0 % until the failure | runaway BACKWARD to -113 | dragged 23 deg behind | -- |

It does exactly what it was built for -- the arm never reaches a stop -- and it removes the claw's
TETHER on the base along the nose: the world-held claw's 120 N/m impedance had been holding the
base back (~3 N at the 25 mm lead). Without it the base wanders 2x further along the nose, and at
the turn's deceleration with the faster observer it RAN AWAY backward (-9 -> -113 mm over 6 s,
pitch nose-up to 150 mrad), dragging the valve and unfolding the arm into the elbow branch: a crash,
twice. With omega_c_t 2.927 it is not flyable; at the shipped 2.0 it is (15.8), and that is where it
is used -- the next step for the runaway would be a stiffer base loop along the free axis (or a
weight scheduled down at the deceleration), offline first.

### 15.7 omega_c_t 2.927

3 of 3 complete (p_oct29_1..3), the lead ~20 % smaller and the arm on its stops 1-8 % of the turn
against 5-11 % at 2.0, and it is the hardware free-flight file's own value. **Not shipped on 3
runs**: the low-friction boundary (15.8) was measured at 2.0, the shipped value. It is the first
candidate for the next campaign.

### 15.8 The bearing's friction

| bearing [N.m] | final config (anchor off, omega_c_t 2.0) | anchor 1.0 (omega_c_t 2.0) |
|---|---|---|
| 0 (frictionless) | **1 of 3** natural (p_f000_1, s2_abort20_f0 failed; l6_ship_f0 complete) | 3 / 3 (fa_f0_1,2, c5_ship_f0) |
| 0.005 | 2 / 2 (b_f005_1,2) | 1 / 1 |
| 0.010 | 1 / 1 (b_f010_1) | -- |
| 0.016 | 2 / 2 | -- |
| 0.048 (the sim default) | 23 of 26 headless at omega_c_t 2.0 (failures s_abort33 at 14.6 s, a_abort20 at 17.9 s, pf_n4 at the turn's end; fused, stiff-heading and unfiltered-governor runs left out) | 6 / 6 |
| arm on a stop | 5-13 % of every turn | 0 % (one excursion: q1 on its stop, q3 -16) |

The final config completes from 0.005 N.m of bearing friction upward and is the weak end only on a
truly frictionless bearing. A sealed 608RS is unlikely to be there, but it was measured only as
<= 32 mN.m: **measure its breakaway torque before the first hardware turn** (a weight on a string
at the fin, r = 0.160 m: 3 g = 5 mN.m, 19 g = 30 mN.m); if it is below ~5 mN.m, add friction to the
shaft (a felt or O-ring washer) rather than fly a lower-friction valve.

### 15.9 The anchor's latch, and why it is not shipped

The first anchor build faded the anchor with the contact flag -- and the planner turns the contact
flag off at the END of the turn (`push_pull_contact_off_at_push_end`), a second before the jaws open,
and at once in an abort. That dumped the turn's accumulated lead on a still-clamped arm: in the
release q3 dipped to 1.4 deg (c5_ship_2), and an abort injected at 33 s FLIPPED the vehicle (q3 to
-75 deg, c5_abort33). The client now LATCHES it from the contact start until the EE anchor is asked
back to the CoM (Exit, or the abort's release ending), so it fades with the CoM anchor's blend after
the jaws are open (log line "free-axis anchor LATCHED"; the hardware stacks refuse a node without it
when the yaml sets the weight). Latched: aborts at 33 s x2 and 20 s clean (l6_abort33_1,2,
l6_abort20; q3 >= 18.9 deg through the releases).

**Not shipped anyway.** Across all eleven anchor turns the base swings 55-96 mm along the nose (43-54
without) and 37-65 mm sideways (29-36): the claw's world hold was a tether on the base, and without
it a slow pitch oscillation (+-115..+-170 mrad) moves the base, whose world-fixed error rotates into
the arm's lateral direction as the heading turns. In l6_abort33_2 that reached 87/96 mm, q1 on its
-35 deg stop and q3 at -16 deg (recovered). The configuration without the anchor rides its stops
5-13 % of every turn but has the flight history, and its failure path is the one the safety pass
guards. Kept in the node (default off) for the next step: damping that slow base mode, offline first.

### 15.10 The final valve config (shipped in `make_valve_turning_yaml.py` and the hardware profile)

omega_c_t 2.0 (README 8), pose [0, 5, 25, 0], the free-axis anchor OFF, `wb_ee_anchor_blend_s` 1.0
(what flew), planner tilt guard 10 deg, the free-handle abort (planned hold, 2 s world-held release,
straight climb); arm GS: jaws open on a DIRECT loss on the handle, Exit To Turn opens with the claw
world-held. Confirmation on the final yaml: l6_ship_1,2 (0.048) and l6_ship_f0 (frictionless)
complete. Hardware: the RAW-mocap stack with AM_HW_PROFILE=valve_turning (the fused stack refuses);
measure the table, the valve's position and the bearing first.

## 16. The results campaign and the video on the final config (2026-10-18 evening)

**Results** (`results/simulation_results/Isaac_simulation_data/valve_turning/`, MATLAB `matlab_simulation_data/valve_turning/`, the
earlier 02:33 campaign moved to `previous_pre_safety_pass_20261018/`). The coordinated-admittance valve yaml was
STALE against its generator (15 deg guard, no free-handle abort) and was regenerated first, so both rigs flew
section 15.10's guards and abort. `build_valve_turning.py` now reads the release sequence, the guards and the
abort recipe from each run's yaml / args instead of printing the box push's.

- Whole-body **3 of 3 at the first attempt** (tags final_wb_r*_a1): beam 359.8 / 359.8 / 361.5 deg, plan error
  1.9 deg rms, impedance residual 1.0 N rms over the turn, tilt max 4.2-4.9 deg, the arm on its stops 11-21 %
  of the turn, q3 >= 21 deg in the release.
- Coordinated admittance **0 of 9**: 5 planner tilt-guard aborts (10.1-13.2 deg), 3 node watchdog trips, and
  one complete mission (final_ca_r2_a3: +375 deg, tilt 8.1 deg) rejected only for running 0.27 s past the
  timetable's landing slot. Its aborts are not survivable on the free handle: the base keeps tilting while the
  jaws open, the fin drags it over (26 N) -- 4 of 8 ended inverted. Under the old 15 deg guard it kept 2 of 7.

**Video** -- recorded as a replay of a headless flight (box_pushing_video_20261017/README.md, last section):
4 recording flights for one complete mission. The three failures (tilt guard at 8 s and ~25 s into the turn,
and 2.2 s into the push with the wrist pair oscillating already in the Ready hover) used the same yaml,
binaries and stream timing as the campaign's three passes. **The final config completes 6 of 9 at the measured
bearing today** (l6_ship_1/2, the campaign's 3, valve_rep4), not the 23 of 26 of the earlier configurations,
and one of the three failures (valve_rep2, arm on its stops) ended inverted after the abort. Before the first
hardware turn, treat the contact phase as marginal on this evidence.
