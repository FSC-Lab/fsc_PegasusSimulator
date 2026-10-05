# Push-and-pull with the whole-body 4-D L1 law (2026-10-03)

User request: a box on the lab's console table, grasped by a handle on its SIDE
and pushed 50 cm along the table, the arm held at a fixed pose, the law in its
contact (impedance) phase for the interaction -- five buttons on a new arm GS
tab, the same logic as pick-and-place, safety first. Commands: Command.md
section 17. FLOWN IN ISAAC 2026-10-04 (section 7): three changes were needed,
then 3 / 3 complete missions, the box pushed 495-502 mm and left on the table.

## 1. The scene (07_px4_direct_t650_aerial_manipulator_push_and_pull.py)

| item | value |
|---|---|
| table | HOOBRO BB52UXG01G1 (amazon.ca B0G62DWL11): 39.4 x 5.9 x 31.5 in = **1.000 x 0.150 x 0.800 m**; centred on (1.20, 0), its LENGTH along y; 18 mm top, 25 mm legs, a lower shelf |
| box | 200 (y) x 100 (x) x 60 mm, **200 g** with its handle, centre (1.20, 0.25, 0.83) -- mocap **obj_0** (centre + yaw) |
| handle | a vertical FIN on the box's +y face: 20 mm thick along x (the jaws close along it), 90 mm out, 20..110 mm above the table top |
| grasp point | 15 mm under the fin top, 55 mm out from the box face: **EE offset [0, 0.155, 0.065] m** in the box frame |
| friction | box / table static **0.6** = dynamic **0.6** -> **1.18 N** to break the box loose and to keep it sliding (equal on purpose, section 7; first designed 0.7 / 0.6); the fin carries the pick-and-place grip material (1.2 / 1.0, combine max); gripper drive capped 0.3 N.m |

Why these numbers:
- **Force bound.** The 2026-09-27 interaction campaign's along-the-arm
  capacity is ~6 N (8-10 N nears the tilt guard). 1.2 N is a 5x margin, and it
  sits above the hardware |F_hat| noise floor (p99.9 1.3 N at the 2 rad/s
  reading), so the push reads as a contact. The planner aborts above 5 N.
- **Tipping is mass-independent**: the box tips about its leading bottom edge
  when mu_s * h > L / 2 (h = the grasp height above the table, L = the box
  length along the push). 0.6 x 0.095 = 0.057 m vs 0.100 m: 1.75x margin before
  the grip's moment helps. **For the real box: keep mu_s * h < L / 3** -- choose
  the length first, then the payload / surface for the friction you want.
- **The table is only 15 cm deep**: a 100 mm box leaves 25 mm a side. The claw
  holds the fin, so the box follows the claw's lateral error (mm in sim).

## 2. The geometry that decides the height

The push pose is the pick pose [0, -20, 30, 0] deg (sigma_nd 0.298, claw
pitched 10 deg forward): the claw 0.130 m ahead of and **0.316 m under** the
body origin. The landing gear, measured from AM_xfwd.usda: two skids along the
body x, **0.28 m apart inside**, bottoms **0.313 m** under the origin, struts at
x ~ 0 from |y| 0.10 (z -0.10) to 0.16 (z -0.27). With the grasp at 0.895 m
the body origin flies at 1.211 m and **the skids are 9.8 cm above the table
top** -- and they straddle both the 150 mm table and the 100 mm box laterally
anyway. The planner checks every step against an axis-aligned table keep-out
(`push_pull_table`, 5 cm clearance for the gear footprint and every arm point);
a grasp 6 cm lower is refused.

## 3. The mission (planner: fsc_trajectory_planner push_pull.{hpp,cpp} + the node's push-and-pull section)

| button | planner | what flies (dry run on the sim yaml) |
|---|---|---|
| 1 Go To Start | `push_pull/go_to_start` | the claw 0.20 m above the grasp point, yaw -90 (the box's - 90), arm home -> push pose; 9.8 s. On arrival the claw is held in the WORLD (the pick's anchor) |
| 2 Ready To Push | GS opens the gripper; `push_pull/ready` | straight down 0.20 m onto the fin (gated: claw within 5 cm of the point above, descent trim as the pick's); 3.2 s; on arrival CONTACT on |
| 3 Push | GS closes once within 2 cm / 3 mm across the fin; on the stall `push_pull/push` | the claw stays WORLD-held (`push_pull_world_anchor_push`); 1.5 s settle; the base 0.50 m along the nose, arm / heading / height fixed, >= 12 s (peak 0.09 m/s); 13.5 s |
| 4 Exit To Push | GS opens, waits 1 s; `push_pull/exit` | CONTACT off; straight up 0.20 m (3 s, arm fixed: the open jaws leave the fin vertically), then the arm home in place; 9.5 s |
| 5 Go To Land | `push_pull/go_to_land` | to the land hover 0.5 m behind the start, yaw -90 -> -360 = **270 deg clockwise** (flown as typed); 23.9 s |
| ABORT | GS opens + `push_pull/abort` | contact off, CoM anchor; on the handle a 0.5 s hold for the jaws, then up 0.30 m, then arm home. Touched box -> capture + plan dropped |

Safety guards in the planner (both -> ABORT): tilt > 15 deg for 0.1 s; the task
force the law reads (wb_control_debug [97..99], low-passed at 3 rad/s) > 5 N for
0.3 s while in contact.

## 4. The contact phase (fsc_autopilot_ros2, whole-body client)

New service `whole_body_direct_actuation/set_contact` (std_srvs/SetBool): the
4-D attribution's chi from the task. DIRECT and 4-D L1 only; every DIRECT entry
and SAFETY revert restores the yaml's `wb_l1_contact` (false here) -- no call =
the node as it was. The yaml (generated, `tools/make_push_pull_yaml.py`) = the
mirror + vehicle_name AM-T650-WB-L1-4D-PUSH + k_R/k_w 1.6/1.2 (the
pick-and-place pair) + the contact reading split (wb_l1_omega_x 2.0, per-block
0.2072 -- the interaction campaign's fix; free flight identical) + the planner
block. The pick-and-place campaign was evaluating k_R/k_w 1.4/1.5 on
2026-10-03; not adopted here.

## 5. Verified (offline, 2026-10-03)

- `test_push_pull` (gtest, 8): the grasp on the fin, yaw -90, the push holds the
  arm (< 1e-6 rad) and heading and slides 0.50 m, the claw height moves < 1 mm
  (the acceleration tilt), the descent is vertical, the exit climbs before it
  folds, the land turn is 270 deg clockwise, the table check refuses a low
  grasp, the mission dry-runs at rest at both ends of every step, the abort
  climbs then folds. All other planner gtests pass (49 in total).
- `test_push_pull_loopback.py` (the built node, a perfect-plant rig, fake
  contact / anchor / force feeds): ALL PASS -- Adjust, Get, Plan, out-of-order
  refusals, the world anchor on arrival, ready refused 80 mm off, push asks for
  CONTACT + the CoM anchor (2026-10-04: the world anchor, per the yaml) and slides 0.500 m with the arm fixed to 1e-14, exit
  turns contact off and climbs 0.200 m before folding, 270 deg clockwise land,
  the force guard trips at 5.6 N mid-push (release hold 0.55 s, climb, arm
  home, plan dropped), SAFETY silences the stream.
- The whole-body node: set_contact registers and refuses outside DIRECT.
- The arm GS tab rendered offscreen at 2560 x 1440 against the loopback
  planner (holding above the handle, on the handle, mid-push).

Not covered offline: the friction, the grip on the fin, the box's behaviour,
the contact rendering under a real push -- section 7 flies them.

## 6. Tools

- `tools/make_push_pull_yaml.py [--check]` -- the sim yaml from the mirror.
- `tools/run_pl.sh <tag>` -- one headless flight (after a clean slate in a
  separate call); `tools/pl_mission.py` -- the tab's steps, scripted.

## 7. Flown in Isaac (2026-10-04)

Headless, RTF 1, raw mocap, the mirror plant (`run_pl.sh <tag>` -> `runs/`,
scored by `tools/pl_score.py runs/<tag>.npz`). Every run: Adjust, Get, SAFETY
climb to 1 m, DIRECT, Plan, the five steps as the GS would press them
(`tools/pl_mission.py`), landing. The first three steps were clean in every
run (claw 1-10 mm above the handle, the grasp 2-4 mm from the point, 0-3 mm
across the fin); everything below is about the PUSH.

| run | what changed | push | box |
|---|---|---|---|
| pl_1 | as designed: CONTACT at the Push press, CoM anchor | node watchdog (tilt 20 deg) 7.7 s in | off the table |
| pl_2 | + the law's platform observer HELD in contact, CONTACT from the end of Ready | planner tilt guard 4.2 s in | |
| pl_3 | + hold only the TRANSLATIONAL rows | completed, 0.3 Hz limit cycle, heading 90 deg | 468 mm, yaw -35 deg |
| pl_4 | + the claw WORLD-held through the push | node watchdog 6.3 s in (arm at the wrist singularity) | |
| pl_5 | world anchor, translational hold OFF | force guard (5.3 N) 6.0 s in | 126 mm |
| **pl_6** | **+ box friction 0.6 / 0.6 (was 0.7 / 0.6)** | **completed** | **502 mm, 14 mm sideways, yaw +2.6 deg** |
| **pl_7** | repeat | **completed** | **495 mm, 49 mm sideways, yaw +14.9 deg** |
| **pl_8** | repeat | **completed** | **501 mm, 2 mm sideways, yaw -2.9 deg** |

The push step on the shipped configuration (pl_6 / pl_7 / pl_8): peak tilt
5.5 / 9.8 / 5.5 deg, |e_R| <= 0.12 / 0.16 / 0.10, CoM error peak 51 / 49 / 53 mm
(rms 23 / 24 / 22), EE position error peak 11.5 / 20.4 / 11.4 mm (rms 5 / 7 / 5),
EE heading peak 6 / 12 / 13 deg, peak arm torque 1.20 / 1.44 / 1.32 of 3 N.m,
**0 rotor saturation, 0 joint clamp**, claw-to-box slip <= 22 / 61 / 22 mm.

**The mechanism, in the order it was found.** A box held by STATIC friction
reacts with exactly the force the vehicle applies, so anything that
INTEGRATES that reaction pushes through it until break-away:

1. pl_1: the law's CoM observer (the note's Layer 1, the lumped d_t that f_d
   cancels) integrated it -- d_t,y -0.2 -> +1.3 N and 51 mm of CoM drift on a
   FIXED reference during the 1.5 s settle, then a growing stick-slip cycle.
2. pl_2: holding the WHOLE platform estimate removed that, but the box's
   friction acts ~0.3 m below the CoM, and the pure-P attitude loop alone is
   k_R M_r / M_r_d ~ 1.6 N.m/rad: 0.37 N.m tilted the vehicle 0.26 rad.
3. pl_3: holding only the translational rows completed the mission, but with
   the CoM anchor any base offset drags the claw reference against the stuck
   box, and the ROTATIONAL observer then pushed through it -- a 0.3 Hz cycle,
   box yaw to +-90 deg.
4. pl_4 / pl_5: holding the claw in the WORLD through the push decouples the
   base from the box (base drift moves no reference): the first 4 s of pl_5
   were the cleanest of the campaign. pl_4 also held the translational rows and
   the base sat 25 mm off its plan, stretching the arm to the wrist singularity
   (beta = q2 + q3 = 1.5 deg) -- with the world anchor the hold is not needed.
   pl_5 failed at the break-away: 0.7 -> 0.6 is a 14 % force drop while the
   observers still carried the stuck box's reaction and its moment; the base
   surged 7 cm and the world-held claw folded the arm onto q3's +50 deg stop.
5. pl_6: equal static and kinetic friction removes that drop. The rest of
   the configuration is pl_5's.

**What changed, where:**
- planner (`fsc_trajectory_planner`): CONTACT requested at the END OF READY,
  not at the Push press (the jaws close in between, and the box joins the
  chain at the stall); new `push_pull_world_anchor_push` (default **true**)
  keeps the claw world-held through the push. Loopback updated, ALL PASS.
- law (`fsc_autopilot_ros2`, 4-D L1 observer): new
  `wb_l1_contact_hold_translation` (default false = byte-identical, parity
  17/17): holds the CoM observer's rows in contact. NEEDED WITH THE CoM ANCHOR
  ONLY; written out `false` in the push-and-pull yaml (pl_4). Python mirror in
  `l1_observer.py` (self-test passes).
- scene: box / table friction default **0.6 / 0.6**.
- yaml: regenerated by `tools/make_push_pull_yaml.py` (the planner key, the
  law key written out false, comments).
- tooling: `run_pl.sh` read array parameters wrongly ("value is:" vs "values
  are:") and now starts the scene `--in-terminal` so its output is logged;
  `am_plant_from_yaml.sh` did not know `WB_SIM_PROFILE=push_pull` (the scene
  launcher died silently in its own terminal window); `pl_mission.py` now opens
  the jaws on a planner ABORT and lets the abort finish (pl_5 climbed with the
  box still clamped and threw it 3.5 m); new `tools/pl_score.py`.

**For the real experiment:**
- **The box's base: static friction = sliding friction** (PTFE glides, felt
  pads) -- a grippy base sticks and then lets go, which is exactly what
  defeated pl_5. Keep mu_s * h < L / 3 (section 1).
- **The heading is the soft axis.** The box yaw follows the EE heading error
  (pl_7: -9 -> +12 deg over the slide): pushing compresses the box, and its
  friction acts 155 mm ahead of the grasp (~0.18 N.m/rad of destabilising
  stiffness against K_psi 0.25). A shorter box along the push, or a grasp
  closer to its centre, reduces it.
- **The push pose is near the wrist singularity** (beta = 10 deg): with the
  box clamped, q1 and q4 counter-rotate +-25-35 deg while q1 + q4 holds the
  heading. Harmless here (peak torque 1.4 N.m), worth watching on hardware.
- Not covered: hardware (there is no hardware push-and-pull block), pull
  (negative distance -- not flown), the hardware |F_hat| noise against the 5 N
  guard.

