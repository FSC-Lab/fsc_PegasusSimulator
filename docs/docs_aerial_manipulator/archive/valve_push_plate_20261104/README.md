# Valve turning: the PUSH PLATE for a closed claw (2026-11-04)

User request, after the 11-03 192853 failure (`../wb_valve_turning_flight_20261103/`, the fin twisted inside the open U
and its back corner pressed the U's rail sideways, twisting the vehicle until its yaw saturated):

> 1. Simplify the U to the closed gripper (or the open gripper nose-down) and push the handle with the nose only,
>    never letting the fin be in the claw: a rectangular push plate fixed to the fin, pushed by the end-effector.
> 2. Change the arm pose for the push: the yaw is the sensitive axis and needs margin.
> Update the design page (artifact BppzF9zYUZoNwrdPymSQoi) with that plate only: where, at what height on the fin, how
> large. Copy Valve_Assembly_v2.stl into Valve_Assembly_v21.stl with the plate; rename v2 to Valve_Assembly_v20.stl.
> Then find in Isaac the best plate dimension and push pose, and whether it makes the task more likely to succeed.

## 1. The design

- **The closed claw, not the open one nose-down.** Closed, the two tips touch the plate as one blunt nose; nothing of
  the claw can straddle anything. Open nose-down, the fingertips stand 37 mm apart: a plate edge, or the fin itself,
  can still slip between them when the claw is low or off to one side.
- **Push-only contact.** A nose on a flat face carries the face's normal force (the valve's resistance, ~0.45 N at the
  hub pad's 72 mN.m) plus friction. Sideways the nose slides: the force that twisted the vehicle on 11-03 is bounded
  by mu x N, ~0.1 N, i.e. ~0.02 N.m of yaw at the 0.21 m lever (11-03: 0.64 N.m of joint-1 torque, 1.26 N.m of yaw
  command). An early touch only nudges the free valve; a lost contact only stops it.
- **The plate** (`tools/make_v21_stl.py`, asset frame = from the bearing axis on the plate bottom):
  - 120 mm along the beam (radial, centred on the fin's 0.160 m radius: 0.100 to 0.220 m from the shaft) x 79 mm tall
    (0.125 to 0.204 m above the table, its top flush with the fin's) x 5 mm thick, on the fin's PUSHING (-y) edge,
    its face at y = -0.0244 m; a 15 mm clip either side of the fin (3 mm cheeks), one M3 bolt (not modelled).
  - One closed solid, 54.5 cm^3. Printed like the rotor (204 g / 197.6 cm^3) it adds ~56 g; the sim flies the rotor at
    0.260 kg (`PEGASUS_VALVE_ROTOR_MASS`), inertia about the shaft scaled with it.
  - Face covered with PTFE tape on the real valve (the only sideways force is the face's friction).
- **Why these numbers** (`tools/plate_screen.py`, `tools/page_data.py`):
  - The closed tips reach 20 mm either side of the claw's centre (the CAD: tips at +-9.8..19.4 mm); the nose = the
    tips' upper front corners, 16.9 mm ahead of and 5.4 mm below the model claw point at a 60 deg claw.
  - Width: the claw's centre may sit ~40 mm off the fin's radius (Isaac drift <= 23 mm across + the model claw point's
    8-16 mm off the real one; 11-03 needed ~13) -> 2 x (20 + 40) = 120.
  - Height: the landing gear sets it. The left skid passes over the hub where marker B's ball tops out 140 mm up; with
    the vehicle 90 mm ahead of its circle the body must stay ~0.43 m up, and a 60 deg claw then puts the nose at
    165 mm. A higher contact needs a flatter claw, which no longer reaches the fin's height.
- **The arm pose.** Every pose that keeps the gear clear has the 30 deg fold (claw 60 deg down). Within that family the
  yaw lever (nose ahead of the CoM) shrinks only 5-10 % before q2 nears -20 deg:

  | pose [deg] | lever | q2 to -20 / 45 | q3 to 10 / 50 | sigma_nd | body z | gear over ball B, held / leading 90 mm | base stroke, claw room-held, to the joint ranges fwd/back/up/down | the same to the flown q3 guards (3 / 40 deg) |
  |---|---|---|---|---|---|---|---|---|
  | **[0, 5, 25, 0] (today)** | 214 mm | 25 / 40 | 15 / 25 | 0.278 | 426 mm | 99 / 44 mm | 52 / 21 / 26 / 71 mm | **28 / 29** / 35 / 36 mm |
  | [0, 0, 30, 0] | 204 mm | 20 / 45 | 20 / 20 | 0.298 | 429 mm | 91 / 47 mm | 44 / 31 / 34 / 53 mm | 20 / 39 / 43 / 23 mm |
  | [0, -5, 35, 0] | 195 mm | 15 / 50 | 25 / 15 | 0.317 | 430 mm | 84 / 48 mm | 34 / 41 / 43 / 38 mm | 10 / 49 / 51 / 11 mm |

  The yaw fix is the contact, not the lever. **Keep [0, 5, 25, 0].** I first chose [0, 0, 30, 0] for its balanced
  margins to the q3 RANGE (10 / 50 deg). But the limits that fire are the planner's q3 floor (3 deg) and the fin
  guard's q3 jam limit (40 deg). Against those, [0, 5, 25, 0] leaves the most even fore-aft room (last column), and
  Isaac agreed (section 3).

## 2. What changed

- **Assets** (`extensions/.../rotorcraft/assets/`): `Valve_Assembly_v2.stl` renamed **`Valve_Assembly_v20.stl`** (git
  mv; byte-identical); **`Valve_Assembly_v21.stl`** = v20 + the plate (44 triangles appended, every v20 triangle
  unchanged); **`Valve_Assembly_v21.usda`** from it. `Valve_Assembly_v2.usda` keeps its name (regenerated from v20 it
  is identical but for its source filename in the doc string).
- **`../valve_turning_20261015/tools/stl_to_usda_valve.py`**: recognises the plate shell (wide, above 100 mm, out at
  the fin), adds it to the rotor's visual and mass, a convex collider `Collision/push_plate` (the plate proper, not
  the clip) and `fsc:push_plate_*` attributes. Three archived tools that read the old STL path now read v20.
- **`Valve_Assembly_v21.stl` is now the user's own Onshape design** (2026-11-04, the user's `Valve_Assembly_v21_.stl`
  renamed to it, replacing the generated model; the model above stays reproducible with `tools/make_v21_stl.py --out <elsewhere>`, which now refuses to
  overwrite a file it did not write). Checked in the fin's own frame (the v20 fin, and the plate with it, is turned
  0.21 deg about z): plate 120.0 x 79.0 x 5.0 mm, centred on the fin to 0.00 mm, face 5.0 mm ahead of the fin's
  pushing edge, z 125..204, cheeks 3.0 thick x 15.0 deep, two Ø3.2 holes 7.5 mm from the pushing edge at z 144 and
  189 through both cheeks and the fin -- the sketches of the design page. The one difference: the slot is 5.0 mm
  (zero clearance; print it 5.3). Because the cheeks touch the fin, the hole circles share vertices and the
  converter's shared-vertex split made plate + rotor one shell; `stl_to_usda_valve.py` now splits it again along
  manifold edges (`split_merged_plate`), a no-op for every older STL (v21 model regenerated byte-identical).
  `Valve_Assembly_v21.usda` regenerated from it: rotor 312.4 g at 1240 kg/m^3 (-0.2 g: the holes), the plate's
  collider 0.2 mm further out (its axis-aligned box over the 0.21 deg turn), the same 19 prims. Not flown.
- **fsc_trajectory_planner** (branch `valve-isolation-20261102`): new `push_pull_turn_tangent_lead` [m], default 0 =
  unchanged. The planner refuses a turn whose nose is more than 15 deg off the tangent; it checked at the grasp point,
  and the nose push's claw point sits 61 mm behind the fin's radial line (21 deg off). The check is now made at the
  push point, this far ahead of the grasp point along the nose. gtest `PushPull.ThePushPlatesNoseIsCheckedAtThePushPoint`;
  push-pull 33/33.
- **The valve generator** (`make_valve_turning_yaml.py`): writes `push_pull_turn_tangent_lead: 0.0` (the shipped U
  push unchanged). The hardware valve yaml was NOT regenerated.
- **The plate push = KEY=VAL overrides** of the generated valve yaml (`sim/chain2.sh`), nothing shipped:
  `push_pull_ee_offset [0.1600, -0.0613, 0.1704]` (the nose 20 mm behind the face, 165 mm up),
  `push_pull_turn_tangent_lead 0.0369`, `push_pull_turn_deg 97` (the 20 mm slack is 4-7 deg; the pose stays the
  yaml's [0, 5, 25, 0], the campaign also flew `push_pull_push_pose_deg [0, 0, 30, 0]` and `[0, -5, 35, 0]`), `push_pull_valve_fin [0.160, -0.0219, 0.204, 0.079, 0.0025, 0.060]` (the fin guard describes the
  plate), `push_pull_guard_fin_bracket_back -0.0169` (the push point AHEAD of the claw point), `push_pull_guard_fin_depth_min
  0.012`, `push_pull_guard_fin_across_max 0.035`, `push_pull_exit_world_held_dz 0.08`, `push_pull_follow_lead_deg
  [14, 24]`; the arm GS `-p push_pull_valve_cup_position:=-0.010` (the claw stays closed: Ready and the grasp gate move
  nothing).

## 3. Isaac

**Setup.** Headless, RTF 1, the mirror plant, raw mocap at 120 Hz, the arm GS flying every step (automatic Ready /
Push / Exit). The valve = `Valve_Assembly_v21.usda` at the hub pad's measured 72 mN.m (`drive` model), rotor 0.260 kg.
The plate push = the generated valve yaml + the overrides of section 2 (`sim/chain2.sh` .. `chain5.sh`, through
`../valve_turning_20261015/tools/try_valve.sh`); the U push = the shipped 11-03 configuration on the v2 valve. "Drift
seed k" = `PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,k` (the slow random force that reproduced the 10-21 hardware hover
wander) with the approach gates relaxed exactly as the 11-02 U-push campaign did (`GATES`, `GSGATES`); the U push's own
drift runs (j4, g10, j3) come from `../valve_fin_guard_20261103/sim/runs`. Scored by `tools/sim_table.py` (the
fin-guard scorer `../valve_fin_guard_20261103/tools/score_guard.py` + `tools/plate_score.py`: where the claw touched the
valve from the scene's contact log, the yaw command, the motors); the table below is `tools/sim_page.py --md`, the same
as the page's.

| Run | Setup | Outcome | Valve turned | Peak tilt | q3: lowest / highest in the turn | Yaw command p95 / max | Sideways force p95 | Nose on the plate: out / up [mm] | Gear and arm to the nearest marker ball | Before the turn: closest gap / valve moved |
|---|---|---|---|---|---|---|---|---|---|---|
| p1_nominal | Plate, [0, 0, 30, 0] · nominal | completed | 92.8° | 2.5° | 25.2° / 36.0° | 0.19 / 0.23 N·m | 0.13 N | 142–185 / 159–170 | 65 mm | 11 mm / 0.0° |
| p3_drift2 | Plate, [0, 0, 30, 0] · drift seed 2 | completed | 96.6° | 3.2° | 17.1° / 39.2° | 0.19 / 0.23 N·m | 0.19 N | 141–188 / 157–171 | 60 mm | 16 mm / 0.0° |
| p4_drift3 | Plate, [0, 0, 30, 0] · drift seed 3 | completed | 99.9° | 3.7° | 20.9° / 37.2° | 0.19 / 0.24 N·m | 0.17 N | 141–188 / 155–171 | 54 mm | 1 mm / 0.0° |
| p5_drift4 | Plate, [0, 0, 30, 0] · drift seed 4 | completed | 97.6° | 3.6° | 22.6° / 38.1° | 0.20 / 0.26 N·m | 0.19 N | 138–185 / 156–176 | 65 mm | 1 mm / 0.1° |
| p8_lat12 | Plate, [0, 0, 30, 0] · aimed 12 mm off the fin's radius | completed | 93.8° | 2.6° | 23.0° / 35.5° | 0.19 / 0.23 N·m | 0.15 N | 153–196 / 158–170 | 63 mm | 15 mm / 0.0° |
| q1_edge30 | Plate, [0, 0, 30, 0] · aimed 30 mm toward the plate's edge | aborted (guard): the plate 36 mm across the claw (> 35 mm) | 1.3° | 2.8° | 27.7° / 32.0° | 0.18 / 0.21 N·m | 0.18 N | 184–213 / 163–166 | 75 mm | 13 mm / 0.0° |
| q2_high20 | Plate, [0, 0, 30, 0] · aimed 20 mm higher | aborted (guard): the claw only 12 mm under the plate top (< 12 mm) | 0.0° | 2.6° | 29.7° / — | 0.14 / 0.20 N·m | — | — | — | 14 mm / 0.0° |
| q3_noslack | Plate, [0, 0, 30, 0] · no slack: the nose on the face at the end of Ready | aborted by the station: the hand-over never settled within 6 s | 14.7° | 2.9° | 29.1° / — | 0.15 / 0.22 N·m | — | — | — | 1 mm / 14.7° |
| p10_pose5_25 | Plate, [0, 5, 25, 0] · nominal | completed | 91.3° | 2.7° | 20.1° / 31.8° | 0.20 / 0.26 N·m | 0.12 N | 142–184 / 159–169 | 70 mm | 17 mm / 0.0° |
| p11_pose5_25_drift2 | Plate, [0, 5, 25, 0] · drift seed 2 | completed | 93.3° | 3.5° | 17.1° / 31.8° | 0.20 / 0.25 N·m | 0.16 N | 139–186 / 155–170 | 66 mm | 8 mm / 0.0° |
| p12_pose5_25_drift3 | Plate, [0, 5, 25, 0] · drift seed 3 | completed | 101.4° | 3.3° | 17.2° / 34.5° | 0.20 / 0.25 N·m | 0.17 N | 141–188 / 152–171 | 56 mm | 1 mm / 0.0° |
| p13_pose5_25_drift4 | Plate, [0, 5, 25, 0] · drift seed 4 | completed | 96.1° | 3.1° | 15.7° / 34.2° | 0.20 / 0.26 N·m | 0.17 N | 139–184 / 155–173 | 68 mm | -0 mm / 0.4° |
| p14_above_nominal | Plate, [0, 5, 25, 0], from above · nominal | completed | 93.2° | 2.4° | 19.1° / 31.5° | 0.19 / 0.23 N·m | 0.15 N | 141–185 / 158–169 | 72 mm | 17 mm / 0.0° |
| p15_above_drift3 | Plate, [0, 5, 25, 0], from above · drift seed 3 | completed | 97.4° | 3.7° | 15.1° / 34.0° | 0.20 / 0.25 N·m | 0.17 N | 141–186 / 157–172 | 54 mm | 6 mm / 0.0° |
| p16_above_drift4 | Plate, [0, 5, 25, 0], from above · drift seed 4 | aborted (guard): the plate 37 mm across the claw (> 35 mm) | 0.0° | 3.8° | 23.0° / — | 0.14 / 0.17 N·m | — | — | — | 12 mm / 0.0° |
| q6_pose_m5_35 | Plate, [0, −5, 35, 0] · nominal | aborted (guard): q3 40.1° over the 40° jam limit | 13.6° | 2.6° | 32.4° / 40.1° | 0.19 / 0.23 N·m | 0.15 N | 152–186 / 160–166 | 65 mm | 17 mm / 0.0° |
| p2_nominal_com | Plate, [0, 0, 30, 0], claw follows the vehicle · nominal | completed | 96.9° | 2.8° | 29.8° / 32.2° | 0.21 / 0.22 N·m | 0.22 N | 130–185 / 159–168 | 70 mm | 13 mm / 0.0° |
| p6_drift2_com | Plate, [0, 0, 30, 0], claw follows the vehicle · drift seed 2 | aborted (guard): the plate 61 mm ahead of the nose (> 60 mm) | 107.2° | 3.2° | 26.7° / 32.5° | 0.22 / 0.25 N·m | 0.23 N | 116–198 / 156–172 | 59 mm | 11 mm / 0.0° |
| p7_drift3_com | Plate, [0, 0, 30, 0], claw follows the vehicle · drift seed 3 | aborted (guard): the plate 36 mm across the claw (> 35 mm) | 91.9° | 3.2° | 26.7° / 31.2° | 0.22 / 0.25 N·m | 0.22 N | 123–202 / 155–172 | 64 mm | 7 mm / 0.0° |
| q5_drift4_com | Plate, [0, 0, 30, 0], claw follows the vehicle · drift seed 4 | aborted (guard): the plate 62 mm ahead of the nose (> 60 mm) | 0.0° | 3.1° | 28.7° / 30.5° | 0.14 / 0.14 N·m | 0.19 N | — | 81 mm | -0 mm / 0.8° |
| j1_nominal | U push (11-03 config), [0, 5, 25, 0] · nominal | completed | 92.0° | 2.3° | 19.2° / 31.9° | 0.22 / 0.29 N·m | 0.29 N | — | 15 mm | 9 mm / 0.0° |
| j4_drift2 | U push (11-03 config), [0, 5, 25, 0] · drift seed 2 | completed | 92.7° | 3.1° | 16.5° / 35.0° | 0.26 / 0.32 N·m | 0.30 N | — | 14 mm | 14 mm / 0.3° |
| g10_drift3 | U push (11-03 config), [0, 5, 25, 0] · drift seed 3 | completed | 92.6° | 3.2° | 15.8° / 34.4° | 0.23 / 0.28 N·m | 0.38 N | — | 10 mm | 5 mm / 2.8° |
| j3_drift4 | U push (11-03 config), [0, 5, 25, 0] · drift seed 4 | completed | 86.7° | 3.3° | 16.5° / 35.8° | 0.22 / 0.27 N·m | 0.20 N | — | 16 mm | 0 mm / 5.1° |
| p9_upush_lat12 | U push (11-03 config), [0, 5, 25, 0] · aimed 12 mm off the fin's radius | completed | 91.1° | 2.2° | 19.3° / 32.4° | 0.21 / 0.26 N·m | 0.12 N | — | 16 mm | 12 mm / 1.1° |
| q4_upush_noslack_lat12 | U push (11-03 config), [0, 5, 25, 0] · no slack + 12 mm off (the 11-03 situation) | completed | 91.4° | 2.0° | 17.9° / 31.4° | 0.18 / 0.23 N·m | 0.18 N | — | 15 mm | 9 mm / 2.2° |

Columns: peak tilt from DIRECT entry and the lowest q3 from Ready, both to the end of the run; the highest q3 during the
turn only (the window of the fin guard's 40 deg q3 jam limit). Yaw command = the law's tau_z (`wb_control_debug` [23]) in the
turn. Sideways force = the controller's own estimate of the force on the claw across the vehicle (raw F_hat
[97..99], 0.5 s low-pass). Nose on the plate = where the claw touched the plate in the turn (out from the shaft / up
from the table; the plate spans 100-220 / 125-204 mm). Before the turn = the pushing point's closest approach to the
face it pushes (the plate's face, or the fin's back edge for the U push; planned 20 / 13 mm) from Ready's end, and how
far the valve turned between Ready and the turn. The abort reasons are the planner's, renamed for the plate: its own
messages still say "fin" and "bracket".

**Findings** (22 flights today: chains 2-5; chain 1 = two Plans refused, `sim/runs/refused_chain1/`, before the
planner learned `push_pull_turn_tangent_lead`).

1. **The plate push completes as reliably as the U push in Isaac.** Every unforced room-held plate flight completed,
   9 of 9: p1/p3/p4/p5/p8 at [0, 0, 30, 0] and p10-p13 at [0, 5, 25, 0], nominal, drift seeds 2/3/4 and a 12 mm aim
   error. Valve 91.3-101.4 deg, tilt <= 3.7 deg, DIRECT kept. The U push also completed all six of its runs, the 11-03
   analogue included.
2. **Its failure modes abort cleanly** (DIRECT kept in all 26 runs, tilt <= 3.8 deg in every abort). Aimed 30 mm
   toward the edge (q1): the across guard at 36 mm, the outer tip 7 mm from the edge. Aimed 20 mm high (q2): the depth
   guard before the turn. No slack (q3): the nose pushed the valve 14.7 deg in the hold, the planner's target follows
   the LIVE handle, so the claw chased the plate until the station's 6 s settle limit aborted. -> keep the slack.
3. **It needs the room hold.** CoM-anchored through the turn: 1 of 4. The free valve then follows the vehicle's wander
   (p6 turned it 107 deg against the 97 planned, then lost it 61 mm behind; p7 across 36 mm; q5 62 mm behind).
4. **The pose: keep [0, 5, 25, 0]** (today's; no yaml change). The binding limits are the planner's q3 floor (3 deg)
   and the fin guard's q3 jam limit (40 deg), not the q3 range: under drift q3 peaked in the turn at 31.8-34.5 deg at
   [0, 5, 25, 0] but 37.2-39.2 at [0, 0, 30, 0] (0.8 deg from the limit); [0, -5, 35, 0] tripped it (q6). The yaw
   lever (214 vs 204 mm) changes nothing measurable: yaw command p95 0.19-0.20 N.m at all three poses.
5. **Coming down from above** (`push_pull_approach_back 0`: Go To Start holds 0.10 m above the grasp point, Ready
   descends 96 mm with <= 12 mm of horizontal motion, against 118 mm along the nose with the 0.12 m Side Margin):
   2 of 3 (p14, p15). p16 aborted in the hold on the across limit, 37 mm > 35, nothing touched: the hover's sideways
   wander at the end of Ready. Over the 8 drift flights that error peaked at 6-37 mm, so the limit trips ~1 in 8
   either way. Isaac's slide-in does not overshoot (nominal Ready ends within 2 mm of the planned 20 mm gap), so Isaac
   cannot show the benefit; on the hardware the 11-03 slide-in overshot 18 mm, which against the plate's 20 mm slack
   would push the plate before the turn (the q3 case: safe, but an aborted attempt). **Recommended for the hardware.**
6. **Before the turn the plate is pushed far less.** Under drift the nose came within 1 mm of the face in 4 of 8
   flights (the hold's wander, not Ready) and the valve moved <= 0.4 deg; the U push's bracket moved it 0.3-5.1 deg.
7. **Yaw.** The hold before Ready already reads |tau_z| ~0.125 N.m mean in every run (the plant's standing load); the
   turn raises the p95 by ~0.06 N.m with the plate (0.19-0.20) and 0.05-0.12 with the U push (0.18-0.26). Isaac's U
   push never jammed, so the 11-03 yaw load (1.26 N.m command) has no Isaac counterpart; the plate's sideways force is
   bounded by the face's friction (estimate 0.12-0.19 N p95, U push 0.12-0.38).
   **The contact's own yaw moment, from Isaac's contact report** (`tools/contact_yaw.py`, the force on the vehicle about
   the vertical through the law's CoM, every 5th physics step in contact): plate, 16 flights with a turn, |Mz| p95
   0.004-0.017 N.m, max <= 0.045 (room-held <= 0.029), **never above 0.05 N.m**; true sideways force p95 0.05-0.09 N
   against a 0.35-0.49 N mean push (sideways/push p95 0.13-0.23; the plate wears the table's 0.29 slide material, not the
   fin's 1.0-1.2 grip). U push, 6 flights: p95 0.017-0.052 N.m, max 0.07-0.97 N.m, above 0.05 N.m for 0.02-0.22 s in 5 of 6
   (a claw part catching the fin's edge: up to 4.4 N sideways for one logged step). Scale: shifting the hover thrust
   (~9 N a rotor) between rotor pairs gives ~0.65 N.m of yaw (drag torque only, bench km 0.018 N.m/N) against ~8 N.m
   of pitch (the same shift on the 0.23 m arms), so a contact force that pitches the vehicle is cheap and one that
   yaws it is not.
8. **The gear.** The vehicle flies ~46 mm higher (the closed claw pushes with its tip; the U pushed with its bracket
   54 mm up the claw), so the landing gear and arm stay 54-72 mm from the nearest marker ball (U push 10-16).
9. **The plate's size.** No plate sizes were flown: the size matters only through where the nose touches, which every
   flight logs. In the turns the nose touched 138-196 mm out / 152-176 mm up (incl. the 12 mm aim error); the claw's
   centre stayed within -6..+15 mm across (+8..+20 with the aim error) and 25-43 mm under the top. 120 x 79 mm keeps
   >= 20 mm across and >= 27 mm vertically to spare; the hold's wander (up to 25-37 mm across) matters only to the
   across guard, not the plate. Keep it.
   **The aim is the plate's MIDDLE, and should stay there** (checked 2026-11-04, the user asked about aiming toward the
   outer edge): the claw point at 160 mm (the plate spans 100-220) and 165 mm up (125-204). The closed claw touches with
   BOTH fingertips, a ~40 mm patch: the outer one 6-19 mm outward of the claw point (it leads and carries most of the
   push), the inner one 9-21 mm inward (p10-p14). The claw's radial offset from the plate's centre line (+ = outward,
   the planner's across telemetry): in the hold before the turn BOTH ways, -25..+35 mm under drift (nominal -9..+6); in
   the turn slightly OUTWARD, mean +1.7..+3.7, range -6..+15 mm. So an outward aim loses margin on the side the turn
   already drifts to, while the hold's wander is two-sided; centred, the claw can move ~38 mm either way before a
   fingertip leaves the face (the across guard aborts at 35), and the push is lost only past ~80 mm. For more margin,
   widen the plate on both sides and raise the across guard with it.
10. **Valve angle.** Planned 97 deg, the valve ended 91.3-101.4 deg (the slack costs 4-6 deg nominal; the free-axis
    anchor adds the vehicle's lead). For a nominal 90 deg, plan ~95 deg.

**What Isaac cannot show:** the 11-03 jam (no Isaac U push ever jammed, the 11-03 analogue q4 included), and the
hardware's slide-in overshoot. The plate removes the jam by construction (no fin inside the claw; a nose on a flat
face is pushed sideways only by friction).

## 4. For the hardware (not done)

- **Bench first:** the real closed claw on the plate -- the tips' nose (do the foam pads close fully, a flat front?),
  the PTFE face, the 5.3 mm slot and the M3 bolt, the plate square to the push.
- **Configuration:** the plate push is still `KEY=VAL` overrides of the sim yaml; the hardware valve yaml was not
  regenerated. A `plate` option in the two valve generators (EE offset, tangent lead, the fin guard as the plate,
  turn ~95 deg, exit dz 0.08, follow lead [14, 24], Side Margin 0) and the arm GS's cup position -0.010 on the
  hardware station are the remaining work. The fsc_trajectory_planner change (`push_pull_turn_tangent_lead`, a NaN
  refused at the set and in the plan) is pushed on `valve-isolation-20261102` (2026-11-04); both valve yamls carry the
  key at 0.0, so the hardware still flies the U push. Deploy prompts: `deploy_orin_prompt.md`,
  `deploy_laptop_prompt.md`. **Do not mount the plate on the real valve and fly the hardware yaml**: its grasp
  point (80 mm under the fin top, 25 mm behind its centre) sits on the plate's face.
- **The hover wander at the table** (11-03: 25-31 mm rms) is larger than the 20 mm slack and comparable to the 35 mm
  across limit. On a calm day (192853: 9 mm) the plate's numbers fit; on an 11-03 day expect the nose to touch the
  plate in the hold (a nudge) and the across guard to abort some holds. Options if that day comes: slack 30-35 mm (and
  the turn +3-5 deg), or arm the plate's across guard only from the room hold (a planner change: before the hand-over
  the nose pushes nothing).
- The planner's guard messages still say "fin" and "bracket".

## 5. Launching it with the Isaac window

`sim/launch_plate.sh` (after the clean slate in two calls, Command.md section 18 step 0; `TAKEOFF=1` also climbs to
1.3 m and enters DIRECT): the p10 configuration -- the v21 valve at the measured bearing, the plate overrides of
`chain6.sh` (chain 4's plus the four-edge guards and the abort that climbs at once, section 6) written to `sim/plate_sim_valve_turning.yaml` and passed as `WB_SIM_YAML` (the shipped yaml untouched),
the station's claw closed (`-p push_pull_valve_cup_position:=-0.010`), automatic Ready / Turn / Exit. Logs and the
scene's contact log go to `sim/interactive/<time>/`. Fly from the arm GS Valve Turning tab: Get, Plan, Go To Start.

## 6. The four-edge guards and the abort that climbs at once (2026-11-04, user request)

**What changed.**
- fsc_trajectory_planner: a new guard, `push_pull_guard_fin_depth_max` (default 0 = off). It aborts when the claw
  point is more than this far under the top of `push_pull_valve_fin`. With that box set to the plate, the fin
  checks now bound all four edges: `depth_min` 0.012 the top, `depth_max` 0.060 the bottom, `across_max` 0.035
  both sides. The bottom edge matters most: below it the tip would sit under the plate's overhang, and the
  abort's straight climb would hit the plate's underside.
- The valve generator writes `push_pull_guard_fin_depth_max: 0.0` (the U push, unchanged) in the sim and
  hardware yamls.
- Abort at once, for the plate only: `push_pull_abort_release_wait_s 0.0` and `push_pull_abort_climb_on_open
  false`. With only the second, the abort still waits the 1.0 s release. With no release wait the claw is also
  no longer held in the room: it follows the drone at once.
- The vehicle stays in whole-body DIRECT: contact off, climb 0.30 m in 1.5 s, the arm home, hold.
- All three keys are in the plate overrides (`sim/chain6.sh`, `sim/launch_plate.sh`). Scored by
  `tools/edge_score.py`.

**Isaac, 9 headless flights (the p10 configuration plus the three keys).**
These flew the USER'S ONSHAPE v21 (two M3 holes through cheeks and fin, a 5.0 mm slot; copied in and the asset
rebuilt by another session at 09:18, before chain 6). Its pushing geometry is within 0.2 mm of the generated v21's
(face -24.6 vs -24.4 mm, 100-220 x 125-204 mm, 5.2 mm thick), so the guard box and every earlier number still apply.


| Run | Claw aimed | Result | Climb +10 / +100 mm after the trip |
|---|---|---|---|
| s1 nominal | centre | completed, valve 91.7 deg, tilt 2.6 deg | - |
| s2 drift seed 3 | centre | completed, valve 98.1 deg, tilt 3.4 deg | - |
| s3 outer side | +31 mm | sideways guard in the hold | 0.36 / 0.66 s |
| s4 inner side | -34 mm | sideways guard in the hold | 0.39 / 0.67 s |
| s5 top | 13.5 mm under the top | depth-min guard in the hold | 0.38 / 0.68 s |
| s6 bottom | 58.5 mm under the top | NEW depth-max guard in the hold | 0.40 / 0.68 s |
| m1 outer, in contact | +29 mm | sideways guard 6.2 s into the turn, the tip touching the face | 0.41 / 0.68 s |
| m2 outer, drift seed 2 | +28 mm | sideways guard 6.1 s into the turn (slack phase) | 0.36 / 0.67 s |
| m3 outer | +27 mm | full turn (92 deg, the claw up to 35.7 mm), then the sideways guard in the release | 0.37 / 0.68 s |

- Before the change the same climb took 1.39-1.41 s to +10 mm and 1.67 s to +100 mm (q1, q2). What remains is
  the climb leg's own start, not a wait.
- In every abort: DIRECT kept, tilt <= 4.4 deg, no non-gripper contact. In m1 the tip touched the plate for
  0.59 s at <= 0.36 N and the valve moved 1.4 deg; m3 0.55 s, no force; the others nothing.
- No false trip in s1 / s2 with the bottom guard (the claw 25-43 mm under the top in the turns).
- Not shown: an edge trip while the valve was turning fast. The edge runs tripped in the hold, in the slack
  phase, or right after the turn.

**What the guards cannot see.** They compare the planner's claw point with the plate placed from obj_0 plus the
axis offset. An error in either is invisible to them. On 11-03 the computed claw point was 10-15 mm off sideways,
and a wrong axis offset would make the claw circle off-centre and slide across the real plate unnoticed. Check
the axis offset with `valve_body_check.py` before flying. A hardware plate config does not exist yet; when it
does, it needs these three keys, and the raw hardware stack must refuse a planner without
`push_pull_guard_fin_depth_max`.

## 7. The plate is the valve design everywhere, the hardware included (2026-11-04, user request)

| Where | What changed |
|---|---|
| `valve_turning_20261015/tools/make_valve_turning_yaml.py` | `HANDLE = "plate"` (the U push: `"upush"`). The planner block carries the plate: `push_pull_ee_offset` [0.1600, -0.0613, 0.1704], turn 97 deg, tangent lead 0.0369, follow lead [14, 24], exit room-held 0.08 m, `push_pull_valve_fin` = the plate's box, the four-edge guards (depth 0.012 / 0.060, across 0.035, push point -0.0169), the abort with no pause (release 0.0, climb-on-open false). |
| `make_hw_valve_turning_yaml.py` | the hardware's 1.5 s release wait becomes 0.0 for the plate. The hardware yaml = the sim's plate block plus the lab facts (gear 0.248, fence 1.8, the real gripper's topic / joint / open reading, mocap release 1.0, which the 0.0 release caps). |
| fsc_open_manipulator `push_pull_panel.cpp` | the Valve Turning tab's claw is CLOSED by default (`push_pull_valve_cup_position` = `gripper_closed`, -0.010); its startup line says "the claw CLOSES on the handle ... the PUSH PLATE". A close that stops wider than -0.006 m is a wrong grip (aborted). The U push: `-p push_pull_valve_cup_position:=0.019`. |
| `07_px4_t650_aerial_manipulator_valve_turning.py` | the default asset is v21; with a push-plate asset the fin grip defaults to 0 and the rotor mass to 0.260 kg. The U push: `PEGASUS_VALVE_USD=<assets>/Valve_Assembly_v2.usda`. |
| fsc_autopilot_ros2 hardware raw stack | `AM_HW_PROFILE=valve_turning` expects the plate's values (113 tuned values) and refuses a planner without `push_pull_guard_fin_depth_max` or `push_pull_turn_tangent_lead`. |

Checks: both generators `--check` UP TO DATE; the hardware stack's pre-launch part (cut before its first
`tmux has-session -t "$SESSION"`, line 1607, no launch line in the copy) prints "all 113 tuned values present" for
`valve_turning` and "all 62" for `box_pushing`; with a planner binary whose two new keys are blanked it REFUSES with
both new lines.

Isaac with the SHIPPED DEFAULTS (`sim/chain9.sh`: the generated yaml, the scene's default asset, the station's default
claw; no overrides, no scene knobs, no station arguments):
- d1 nominal: completed, valve 91.8 deg, tilt 2.9 deg, q3 >= 19.7 deg. The nose pushed the PLATE's face (139-183 mm out,
  155-168 mm up, 0.36 N mean), and the station closed the claw for the turn by default (gripper -0.873 rad, closed).
- d2, the claw aimed 58.5 mm under the plate top (the only override): the bottom-edge guard at 60 mm in the hold; the
  climb +10 mm at 0.39 s, +100 mm at 0.68 s; no contact, the valve unmoved, DIRECT kept, tilt 2.3 deg.
