# Pick-and-place with the CAD box payload: the hook grasp and its arm poses (2026-10-07)

User request: use `Box_Payload.STL` (the user's CAD, one rigid body, handle up, 200 g)
as the pick-and-place payload, and identify the arm's **pick / place pose** and **place
start (carry) pose** -- q2 and q3 -- clear of singularities and of the worst-case
overshoot, inside the real arm's q2 range **[-20, +45] deg** (the U2D2 board behind the
upper arm) and q3's +50 deg stop; test in simulation.

## Result

| pose | q1 | q2 | q3 | q4 | claw axis | joint margin at its worst flown moment | sigma_nd |
|---|---|---|---|---|---|---|---|
| **Pick / place** | 0 | **32** | **38** | 0 | 20 deg below horizontal | q2 <= 38.9-40.1 (5-6 deg to +45), q3 <= 44.2-45.5 (5-6 deg to +50), q2 >= 20.4 (40 deg off -20) | >= 0.31 |
| **Place start (carry)** | **12** | **38** | **42** | 0 | 10 deg below horizontal, yawed 12 deg | q2 <= 39.3, q3 <= 44.5 on the carry leg (5.7 / 5.5 deg to the stops) | >= 0.31 |

The carry pose was first [0, 35, 40, 0] (hook5/6). The user then asked for a pose clearly
different from the pick pose, so the arm can be seen moving with the payload. That became
**[12, 38, 42, 0]** (hook9/10, section "The showcase carry pose").

Flown 4/4 clean in Isaac with the final flow (hook5/6 on the old carry pose, hook9/10 on
the new one). Each run picked, carried and set the basket down upright, 11-53 mm off the
place pillar's axis. The basket was untouched after the set-down, peak vehicle tilt was
<= 4.4 deg, and there were 0 aborts. The worst joint moments are the two load steps: the
**set-down** springs q2 and q3 UP by +5..+8 deg (the basket's weight leaving the claw),
the **lift** dips q2 by -10..-12 deg. Singularity is never the binding constraint here
(sigma_nd 0.31-0.36 for every candidate, keep-out 0.10; the law's own 0.87-0.90).

**Friction is a hardware requirement no pose fixes.** The arch hangs on the fingers' top
edges, which slope down toward the tips (claw tilt + ~6 deg of taper: ~26 deg at the pick
pose, ~21 at the carry pose). At the scene's friction (1.2) the hook holds; at **mu 0.4**
the arch slid off the fingers at the lift with BOTH poses (hook7 / hook8). Give the finger
tops mu >~ 0.6 (rubber / grip tape) or put a notch on the arch.

## The payload (`stl_to_usda.py` -> `rotorcraft/assets/Box_Payload.usda`)

SolidWorks binary STL, mm, +Y up: an **open perforated basket 110 x 115 x 65 mm** (side
tabs included) with a **flat wire hanger** in one vertical plane 27.5 mm off the basket
centre -- two diagonal struts and a cross strut meeting a **vertical stem 3 x 2.5 mm**
(from +58 to +168 mm above the basket top) that ends in an **arch** (flat underside at
+166.7 mm between +-28 mm, arms curving down to +-40 mm at +155.7 mm). Converted to metres,
+Z up, origin at the basket centre (= mocap `obj_0`); +x = the hanger plane's normal, +y =
along the arch. Visual = the full welded mesh; colliders = the basket's bounding box + the
hanger as 36 exact convex prisms (one per triangle of its flat face, extruded through the
2.5 mm); mass 0.2 kg with the closed STL solid's CoM (-0.5, 0.1, -3.4) mm and inertia.
`07` references it on the payload body (`PEGASUS_PNP_PAYLOAD=box`, the default;
`plate` = the 10-01..10-03 box + clamped handle).

## Why a hook, and the geometry

The claw is a hub + slider-crank (crank 15 mm, rod 40 mm). Fitted to the asset's joint
frames, its pads can NEVER close below ~19 mm (~14 mm at the fingertips), even past the
-50 deg limit at the crank's dead centre (-90 deg) -- it cannot clamp a 2.5 mm stem. The
user's design (picture, 2026-10-07): grasp the vertical bar with one finger on each side,
**both under the horizontal bar** -- a hook: close around the stem without clamping, lift,
and the arch rests on the closed fingers. That needs the claw near HORIZONTAL (a downward
claw's palm would sit where the arch is) and a SIDEWAYS approach under the arch.

Finger profile (new `HOOK PROFILE` probe in 07, claw frame): fingers 30 mm tall (+-15 about
the claw point), open inner gap 38 mm, ~11 mm thick, tips 15-20 mm beyond the claw point.
Gear (from AM_xfwd.usda): skids x +-0.163 m, |y| 0.142-0.168, bottom -0.3125 m -- they end
~3 cm short of the cap and straddle it laterally (6 cm a side). The hung basket stayed
>= 58 mm from them at the [0, 35, 40, 0] carry pose and >= 32 mm at the yawed
[12, 38, 42, 0] one.

**EE offset (basket centre -> claw point) [-0.02, 0, 0.16] m**: the stem 7.5 mm behind the
claw point (24 mm from the fingertips), the open fingers' tops 21 mm under the arch and
10 mm under its arms on the way in. **Place point z 1.01** (the basket centre at release =
pillar top + 32.5 - 20 mm): the hooked basket lands and the claw goes on down out of the arch.

## What changed (all default-off where shared)

- **fsc_trajectory_planner**: `pick_place_pick_approach_back` (the pick's ready point that
  far BEHIND the target at its height; the "descent" is a level slide in) and
  `pick_place_place_exit_back` (the place exit backs out, then climbs). Both 0 = the
  vertical legs. gtest `TheHookGraspApproachesLevelAndThePlaceExitBacksOutFirst`; 23/23.
- **Task block** (mirror `_sim.yaml` and, verbatim, the HARDWARE 4-D yaml): poses above,
  ee_offset, place point 1.01, `pick_place_exit_time_s` 1.6 -> 0 (a gentle 3.2 s lift: at
  1.6 s the arch engaged at ~0.15 m/s, the basket swung 21 deg, q2 dipped 13.6 -- hook1),
  `pick_place_world_anchor_pick` true -> **false** (the world-held claw swings q2 by up to
  12 deg; the hook's +-17 mm lateral tolerance does not need it), approach/exit back 0.10.
- **Arm GS** (utils_custom_ground_station, Pick & Place tab), keyed on the planner's
  `pick_place_pick_approach_back` > 0: a full close counts as the grasp (Exit To Pick
  auto-starts); at the place the jaws stay CLOSED through the descent, then OPEN and
  **Exit To Place start at once** at its end.
- **Driver** (`pick_place_controllers_20261003/tools/pnp_mission_v2.py`): `--hook-place`.
- **07**: `PEGASUS_PNP_PAYLOAD` / `_PAYLOAD_USD`, `PEGASUS_PNP_GRIP_FRICTION` ("s,d",
  combine min), the `HOOK PROFILE` probe; both pick-and-place wrappers forward the knobs.
- **Hardware stacks** (4-D WB raw/fused, decoupled raw/fused): REFUSE a planner build
  without `pick_place_pick_approach_back` -- a stale planner would fly the vertical
  approach straight down onto the hanger.

## The missions (Isaac, whole-body 4-D L1, headless RTF 1, raw mocap, 200 g)

| run | pick/place, carry | flow | result | q2 max / q3 max / q2 min [deg] | note |
|---|---|---|---|---|---|
| hook1 | [35,40], [35,40] | exit 1.6 s, place 1.03, open at 2 cm | placed 38 mm | 42.0 / 46.8 / 21.4 | back-out dragged the basket 10.5 mm / 10 deg |
| hook2 | [35,40], [35,40] | exit 3.2 s, place 1.01, 3 s dwell | placed 15 mm | 40.4 / 44.8 / 25.1 | clean |
| hook3 | [32,38], [35,40] | as hook2 | **knocked off** | 40.1 / 45.5 / 21.7 | closed claw re-entered the stem after the lurch |
| hook4 | [32,38], [35,40] | + open before the place descent | **abort** (16.5 deg) | -- | open fingers hold the arch on their outer corners: touchdown slid it 19 mm onto a finger |
| **hook5** | **[32,38], [35,40]** | **hook place: closed descent, open + exit at once** | **placed 34 mm** | 39.4 / 44.2 / 20.4 (place-step q2/q3) | untouched after a 129 mm lurch |
| **hook6** | same | same | **placed 11 mm** | 38.9 / 44.4 / 21.4 | untouched after a 149 mm lurch |
| hook7 | [32,38], [35,40], mu 0.4 | same | not picked | -- | arch slid off at the lift |
| hook8 | [35,40] x3, mu 0.4 | same | not picked | -- | arch slid off at the lift |
| **hook9** | **[32,38], [12,38,42]** | hook place | **placed 53 mm** | 39.3 / 44.5 / 22.9 (carry leg 39.3 / 44.5) | basket moved 63 mm carry vs place pose; skids >= 34 mm; hooked 20 mm deeper at the pick |
| **hook10** | same | same | **placed 28 mm** | 39.3 / 44.2 / 20.6 | basket moved 69 mm; skids >= 32 mm |

(The run-wide q2 maximum, ~40.5, is the takeoff/landing HOME pose [0, 40, 40, 0] -- 5 deg
from +45 by itself; not a hook pose.)

**The set-down lurch** (every run, 11-15 cm): once the basket rests, its weight leaves the
claw and the observer that had learned it pushes the vehicle back ~10-13 cm, which returns
in ~3.5 s. Waiting for the claw to come back within 2 cm before opening is waiting for a
closed claw to re-enter around the 3 mm stem with 5-7 mm a side (hook3); opening before the
descent hangs the arch on the fingers' outer corners, a wedge (hook4). Opening and backing
out at the END of the descent rides the lurch out (hook5/6).

## The showcase carry pose [12, 38, 42, 0]

The user asked for a place-start pose that differs clearly from the pick pose, so the arm
can be seen moving with the payload. With q1 = 0 the arm can only fold and unfold, and
inside the q2 / q3 stops that moves the hung basket at most ~5 cm. Yawing the arm (q1) is
what moves it sideways. The landing gear limits the yaw: the hung basket clears the skids by
29 mm at 12 deg and by about 1 cm at 15 deg (`tools/carry_screen.py`, offline).

[12, 38, 42, 0] yaws the arm 12 deg and folds it 10 deg flatter (beta 80). The claw sits
10 deg below horizontal, flatter than the pick pose, so the hook needs less friction while
carrying.

Measured from the hung basket's position in the body frame, averaged over the last second of
Go To Place Start (carry pose) and of Ready To Place (back at the place pose):

| run | carry pose | basket move, sideways / up / total | skid clearance | carry-leg q2 / q3 peak |
|---|---|---|---|---|
| hook5 | [0, 35, 40, 0] | 4 / 20 / 20 mm | 61 mm | 36.5 / 42.6 |
| hook6 | [0, 35, 40, 0] | 4 / 24 / 25 mm | 58 mm | 36.8 / 42.4 |
| hook9 | [12, 38, 42, 0] | 51 / 34 / 63 mm | 34 mm | 39.3 / 44.5 |
| hook10 | [12, 38, 42, 0] | 57 / 36 / 69 mm | 32 mm | 39.3 / 44.2 |

The offline screen predicted 53 / 37 / 65 mm. The arm held the carry pose at
[12.6-12.8, 36.3-36.7, 42.8-43.1, -1.7] deg, so q2 sags about 1.5 deg under the 200 g
basket. Basket swing during the carry was unchanged at 8.6-8.9 deg mean tilt. Figure:
`runs/hook_carry_6_vs_9.png`.

hook9's 53 mm place offset comes from the pick, not the carry. The arch caught 20 mm deeper
on the fingers at the grasp, the basket kept that position through the carry (it moved
< 2 mm along the fingers), and the claw drifted during the descent. hook10 placed 28 mm off,
inside the old pose's 11-34 mm.

## The decoupled controller (geometric + L1) on the same task (2026-10-07)

The same scene, payload, planner block and poses (pick / place [0, 32, 38, 0], carry
[12, 38, 42, 0]) on the decoupled rig: geometric + L1 node, POSITION-mode arm, the
pick-and-place controller yaml of 2026-10-03 (`..._geometric_l1_..._sim_pick_place.yaml`),
raw mocap. Mission: `run_pnp.sh geo <tag>` with
`PNP_DRIVER_ARGS="--hook-place --grip-via-action --grip-axis-tol 0.003"`, i.e. the arm GS's
path: the GripperCommand close, gated on the claw being centred to 3 mm on the closing axis.
One driver change was needed: the action path aborted on a close that ends
`stalled=False reached=True` ("closed on NOTHING"). For the hook a full close IS the grasp,
so `pnp_mission_v2.py` now accepts it when `--hook-place` is set, as the GS's hook mode
already does.

| | decoupled geo_hook1 | decoupled geo_hook2 | whole-body hook9 / hook10 |
|---|---|---|---|
| result | **placed 38 mm** | **placed 30 mm** | placed 53 / 28 mm |
| descent trim at the pick (the hover offset) | [-38, +31] mm | [-40, +33] mm | [+2, -1] / [-6, -1] mm |
| closing-axis error at the close | 0.0 mm | -0.1 mm | -- |
| peak vehicle tilt (at the lift) | 8.0 deg | 8.5 deg | 4.3 / 4.2 deg |
| basket move, carry vs place pose | 65 mm | 67 mm | 63 / 69 mm |
| basket-to-skid clearance | 31.5 mm | 34.3 mm | 34 / 32 mm |
| carry-leg q2 / q3 peak | 36.4 / 42.6 | 36.8 / 42.5 | 39.3 / 44.5, 39.3 / 44.2 |
| arm held at the carry pose | [11.9, 36.2, 42.6, 0] | [12.0, 36.5, 42.5, 0] | [12.6-12.8, 36.3-36.7, 42.8-43.1, -1.7] |
| set-down lurch | 215 mm | 219 mm | 106 / 117 mm |
| basket after touchdown | moved 2.7 mm, tilt 2.4 deg | moved 3.4 mm, tilt 2.7 deg | 0 / 0 |
| sigma_nd min | 0.313 | 0.310 | 0.314 / 0.309 |

Both decoupled missions completed: picked on the first close, carried, set down upright on the
place pillar. Compared with the whole-body rig:

- **The position-mode arm tracks the poses more quietly.** There is no ~1 Hz q2 / q3 ripple
  (figure `runs/hook_wb10_vs_geo1.png`). The joints stay 3 deg further from the stops: q2 sags
  1.5-2 deg under the basket and does not overshoot.
- **The vehicle moves more at the two load steps.** Lift tilt is 8.0-8.5 vs 4.2-4.3 deg (the
  decoupled pick-up transient of 10-03). The set-down lurch is about twice as large, 215-219 vs
  106-149 mm.
- **The bigger lurch costs a little after touchdown.** The back-out nudges the basket by
  3 mm / 2.5 deg (the whole-body runs left it untouched). It is still upright and on the cap.
- The descent trim works as designed. The planner trimmed the slide-in by the decoupled rig's
  ~5 cm hover offset, and the claw arrived centred to 0.1 mm.

## Where the singular region is, and which poses to test in hover (2026-10-07)

Question: which arm poses to test in hover before the hardware pick-and-place, without stepping
into a singular region. Answer from `tools/hover_pose_map.py` (figure `runs/hover_pose_map.png`,
table `runs/hover_pose_map.txt`), offline, on the 4-D law's model (gripper EE, joint-diagonal
armature):

- **The planner's measure sigma_nd** (sigma_min of the base-relative arm Jacobian J_3y^0,
  translation rows / Lchar, keep-out 0.10) **depends on q3 alone**. Over the real range, q1, q2
  and q4 do not change it. It is zero only in a valley at **q3 ~ -34 deg**, where link 2 lines
  up with the wrist + gripper. It falls below 0.20 at q3 < 6, below 0.10 at q3 < -10, and
  rises steadily above that: 0.17 at q3 = 0, 0.26 at 20, 0.30 at 30, 0.33-0.36 at 38-50.
  "beta = 0" (claw pointing down) is NOT singular for this task (0.215).
- **The matrix the law inverts** (the arm block of (J_y^#)^T, CoM-relative and inertia-weighted)
  stays well conditioned **everywhere** in the range, cond 28-40, the valley included. Relative
  to the system CoM the base supplies the radial motion the stretched arm cannot. The base-relative
  sigma_nd is therefore the conservative (planner / IK) measure.
- The whole pick-and-place envelope flown (q2 20-40, q3 33-45, q1 0-13) has sigma_nd >= 0.31, so
  it is 40+ deg of q3 away from the valley. **What binds the task is the joint stops, not
  singularity**: q2 +45 is 5 deg from home and from the set-down spring, and q3 +50 is 5 deg from
  the set-down spring. The 09-05 "elbow-singular branch" failures were the arm DRIVEN across
  q3 = 0 by an entry transient, a branch / stop problem, and the q3 >= 0 wall exists for it.

Per-pose numbers (no payload / 200 g hung at the grasp point; dM = change of the standing moment
on the airframe vs home):

| pose | q [deg] | sigma_nd | stop margins q2 lo / hi, q3 | J2 / J3 hold [N.m] | dM [N.m] |
|---|---|---|---|---|---|
| home | [0, 40, 40, 0] | 0.334 | 60 / 5, 10 | 0.70 / 0.22 (1.20 / 0.52) | 0 (0.50) |
| pick / place | [0, 32, 38, 0] | 0.327 | 52 / 13, 12 | 0.71 / 0.28 (1.20 / 0.61) | 0.00 (0.50) |
| carry | [12, 38, 42, 0] | 0.340 | 58 / 7, 8 | 0.69 / 0.22 (1.18 / 0.52) | 0.15 (0.51) |
| carry mirrored | [-12, 38, 42, 0] | 0.340 | 58 / 7, 8 | 0.69 / 0.22 | 0.15 |
| lift-dip corner | [0, 20, 35, 0] | 0.317 | 40 / 25, 15 | 0.68 / 0.36 (1.16 / 0.72) | 0.03 (0.46) |
| set-down corner | [0, 40, 45, 0] | 0.350 | 60 / 5, 5 | 0.67 / 0.19 (1.14 / 0.46) | 0.03 (0.44) |
| wrist +-30 at pick | [0, 32, 38, +-30] | 0.327 | 52 / 13, 12 | 0.71 / 0.28 | 0.01 |
| (not needed) unfolded | [0, 25, 25, 0] | 0.278 | 45 / 20, 25 | 0.74 / 0.38 | 0.04 |
| (do not) toward the valley | [0, 20, -20, 0] | 0.071 | -- | -- | -- |

The 200 g basket's moment (~0.5 N.m) is the same at every pose. The pose itself changes the
moment by at most 0.15 N.m (the q1 yaw); hardware servo caps are j2 2.44 / j3 1.42 N.m.

## Tools

`stl_to_usda.py` (the asset), `hover_pose_map.py` (singularity map + hover-test poses), `pose_screen_hook.py` (offline: tilt, reach, sigma_nd,
margins, hung-load torques), `run_pnp.sh` (a mission; `PNP_PROBE_ONLY=1`,
`PNP_GRIP_FRICTION`, Isaac pane saved to `runs/<tag>.isaac.log`), `hook_score.py`
(pnp_score + joints vs the stops, sigma_nd, basket swing / skid clearance, set-down lurch
and disturbance), `plot_hook.py`. Data: `runs/`.

## Not covered

Hardware (the real claw's closed gap and the hanger-finger friction decide it);
repeatability beyond 2 runs per rig and pose; the decoupled rig at friction 0.4 (only the
whole-body rig was flown there); the lift's dynamic friction demand with a real hanger.
