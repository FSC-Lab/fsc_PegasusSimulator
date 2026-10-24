# 2026-10-23 — Mail delivery: the final pick-and-place experiment

User design (2026-10-23, sketch): the AM takes off, flies front-left to the pick pillar, picks a RED
ENVELOPE lying flat on the pillar's hat, flies to two mailboxes (235 L × 125 W × 272.5 H mm) fixed on the
thin console table (the box-pushing table), and slides the envelope through a rectangular mouth in the
front of the RED mailbox. The BLUE mailbox is scenery. Because the envelope lies flat, the wrist roll J4
turns 90° so the jaws close vertically; J4 is the arm's softest joint, so the payload must stay light. The
envelope's size and mass and the mouth's size and position were left to this study, sized from the flight
data; the arm GS gets a "Mail Envelope" tab reusing the pick-and-place logic.

![design](figures/mail_design.png)

![the Isaac scene](figures/isaac_scene.png)

The arm GS's new tab, live: `shots/gs_live_mail_tab.png`.

## 1. The design

### The binding constraint is the mailbox's 125 mm depth (115 mm inside)

At the release three things must hold at once, each with the claw's fore-aft hover error:

* the jaw tips stay **outside** the wall,
* the envelope's far end stays **short of the back wall**,
* its **centre of mass is past the mouth's lip**, or it falls back out when the jaws open.

With the jaws gripping the near edge and ±20–30 mm of error, a UNIFORM envelope cannot satisfy all three
in a 115 mm deep box, at any insertion angle. Its centre of mass is L/2 from the tip, but the box only
admits ~110 mm and the jaws need ~60 mm outside (worked out in `tools/mail_geometry.py`). The way out is a
**weighted far end**: the printed core carries a steel strip there, so the centre of mass sits 40 mm from
the far end. A shorter envelope then has its centre of mass inside the box while the jaws are still
35 mm out. The same weighting makes the pick easy: the near edge can overhang the hat by 70 mm, giving the
jaws room, while the centre of mass is still 40 mm inside the rim (60 mm short of the hat's centre).

### The envelope

| | |
|---|---|
| size | **150 × 90 × 4 mm** (a 2 mm PLA core in red paper); long axis = the approach direction |
| mass | **60 g** (core + paper ~25 g spread over the length, steel strip ~35 g centred 15 mm from the far end; 55–65 g is fine) |
| centre of mass | **40 mm from the far end** |
| on the hat | flat, long axis along the drone's approach, near edge **70 mm past the hat's rim** |
| grasp | 15 mm inside the near edge (EE Offset x −0.06); the jaw tips reach 31 mm in |
| mocap body obj_0 | pivot at the geometric centre, x along the long axis toward the far (weighted) end, z up; low-profile markers (≤ 4 mm tall) on the top face 40–90 mm from the near edge, clear of the jaws |

* **Why 60 g: J4 is not what limits it.** At q4 = 90° with the envelope gripped by its edge, its centre
  of mass lies on the claw axis, so gravity puts almost no torque on J4. The only J4 load is m·g × the
  lateral grip offset: 24 mN·m at 40 mm, against a heading stiffness of 0.445 N·m/rad (the 10-16 j4 fix),
  i.e. ~3°. Keeping it light keeps the other loads small: the pinch moment the jaws resist (56 mN·m), and
  the lift/release kicks on j2/j3 that the 190 g basket made (+18..25° q2 at release). Those scale with
  mass, so expect a few degrees here.
* **Downwash.** A flat plate presents almost no frontal area to the outwash. The basket that moved
  21 mm on the hat (10-10) was 65 mm tall. At 60 g the envelope's friction (~0.2 N) is well above the
  edge drag.
* **Stiffness.** A 2 mm PLA core deflects ~1 mm under its own tip mass when held by its edge; 1 mm would
  sag ~10 mm.

### The grasp pose: [0, 36, 44, 90] deg (pick = carry = place)

* β = q2 + q3 = 80° is the folded home's fold: the claw points 10° below level, 0.247 m ahead of and
  0.068 m below the body origin.
* q4 = 90° turns the jaws to close 10° off vertical, so they pinch the flat envelope.
* q2 is 9° from the real arm's +45 stop and q3 is 6° from +50; σ_nd 0.347.
* A level claw (β = 90°) would need q2/q3 on their stops.
* **The heading task is not singular at q4 = 90°.** The law's 4th task row is the rotation about the
  claw's own axis (`wb_controller.cpp`, b3e = the claw axis), and the planner streams the full 3-D FK
  frame (`flatState`: `b1_de = (R0 Re) e1`).

### The mouth: 190 × 90 mm, centred across the face, top edge 25 mm under the box top

(i.e. 157.5 mm above the box bottom to 247.5 mm). Sized from the measured hover error
(`hover_error/README.md`, 13 hardware flights 10-09..10-21). The unloaded claw at a hold:

* lateral p95 26–29 / p99 41–43 mm
* vertical p99 8 mm
* at the moment the GS fires the gripper (its 20 mm gate): 3-D median 10, max 32 mm

The envelope in the jaws is pitched **0–20°** nose-down: the claw's 10°, plus its weight moment rotating
it in the pinch. The sim measured 8° with a centred grip and 14° with a 21 mm off-centre one. So the
section crossing the wall plane spans −48..+6 mm about the claw over the insert. With ±12 mm vertical,
±40 mm lateral and ±20 mm fore-aft, every margin stays positive (`mail_geometry.py`):

| margin (after the error budget) | mm |
|---|---|
| band below / above the mouth | +6 / +5 |
| each side (envelope 90 in a 190 mouth, ±40 lateral, 2° yaw in the jaws) | +5 |
| jaw tips to the wall, envelope tip to the back wall (fore-aft +) | +15 |
| centre of mass past the lip (fore-aft −, 10 mm kept) | +8 |
| pick: jaw tips to the hat rim / 10 mm of grip left | +9 / +1 |
| front props over the mailbox top, incl. a 3° nose-down pitch | +55 (77 nominal) |

**The props decide the mouth's height.** At the release the body is 0.30 m in front of the wall, but the
front rotor discs reach 0.423 m ahead of the body. They pass 125 mm over the box top, so the claw must
be high enough to keep them 77 mm above it. That is why the mouth is near the top of the face.

### The layout (scene defaults; sim yaml values)

* **Vehicle:** start mark (0.10, −0.07), yaw 0. The drone yaw is **0 throughout**.
* **Pick pillar** (1.0, 1.0), 0.77 m under the hat. The envelope centre is at (0.905, 1.0, 0.780), near edge
  toward −x.
* **Table** front edge x = 1.70, centred on y = 0. Mailboxes with their fronts flush with it: **red at y = +0.25**,
  blue at y = −0.25 (0.5 m apart, so the right front prop barely reaches the blue box).
* **Mailbox Start** (1.0, 0.25, 1.10). **Place point** (envelope centre at the release) (1.709, 0.25, 1.024)
  = claw 21.5 mm above the mouth's centre, 9 mm past the front face. Insert Margin 0.15 m.
* **Land Start / Land** (0, 0, 1.0 / 0.35).

## 2. What was built (all additive; the pick-and-place and pick-and-drop paths are unchanged)

* **Planner** (`fsc_trajectory_planner`): `pick_place_place_insert` + `pick_place_place_approach_back`
  (`PickPlaceConfig::place_insert`).
  * Ready To Place holds the claw the Insert Margin behind the place target at its height; Place slides it
    straight in (the hook pick's level approach, mirrored to the place).
  * Exit backs out `pick_place_place_exit_back`, then climbs.
  * Refused with `place_drop`, or with a margin ≤ 0. Status wording: "press Insert to slide in".
  * gtest `TheInsertPlaceApproachesLevelFromBehindAndSlidesStraightIn`; pick-place suite 31/31.
* **Arm GS** (`utils_custom_ground_station`): `PickPlacePanel::Mode {kSetDown, kDrop, kMail}` and a new
  **"Mail Envelope"** tab at the end (tab index 8).
  * Labels: Envelope / Mailbox Start / Mailbox; Ready To Insert / Insert / Exit To Insert.
  * An **Insert Margin** box (`pick_place_place_approach_back`).
  * Its Plan switches the planner to the insert place (`place_drop` false, `place_insert` true); the other
    two tabs' Plan switch `place_insert` off. Only the matching tab acts on the gripper.
  * The pick closes only centred **vertically** (`pick_place_grip_vertical_tol`, 8 mm: the jaws close
    vertically).
  * The insert opens the gripper once the claw is within 20 mm. **Exit To Insert starts only on the OPEN
    report** (`pick_place_auto_exit_mail`), so the jaws never drag the envelope back out. The gripper
    closes after the exit.
  * Abort service `mail_envelope_panel/abort`.
* **Isaac scene** `application/robotic_arm/07_px4_t650_aerial_manipulator_mail_delivery.py`.
  * It loads the pick-and-place scene as a module, like every 07 loads 06.
  * T650 gear, real q2 stops [−20, 45]; the pick pillar + hat, the console table, the two hollow
    mailboxes with their mouths, the envelope (obj_0, weighted, 20 mm invisible grip pad on its near
    45 mm).
  * Prints the envelope's state on every change ("IN THE RED MAILBOX").
  * Knobs `PEGASUS_MAIL_*`.
* **Configs.** `make_pick_place_yaml.py` profile **mail_delivery** writes `..._t650_sim_mail_delivery.yaml`
  (the mirror's controller + the mail task block). `make_hw_pick_and_place_yamls.py` writes the hardware
  `..._t650_mail_delivery.yaml` (the pick-and-drop's controller + the same task block).
* **Launch.**
  * `WB_SIM_PROFILE=mail_delivery` is accepted by `am_plant_from_yaml.sh`, both 4-D sim stacks, and the
    pick-and-place scene wrapper (which then runs the mail scene and opens the arm GS on tab 8).
  * `AM_HW_PROFILE=mail_delivery` is accepted by both whole-body hardware stacks. They refuse the SIM mail
    yaml, and a planner build without `pick_place_place_insert`.
* **Tools** (`tools/`): `mail_geometry.py` (every design number), `mail_drawing.py`, `mail_score.py`,
  `run_mail.sh` / `chain1.sh` (headless flights), `mail_gs_check.sh` + `mail_rig.py` + `gs_harness/` (the
  tab against the real planner, no Isaac). `pnp_mission_v2.py` gained `--mail`.

## 3. Results

### The tab against the real planner (no Isaac): ALL PASS

`tools/mail_gs_check.sh <gs_mail_snapshot> <out>` runs the real planner on the sim mail yaml, the real
tab offscreen, and a loopback rig pressing the tab's own buttons, on ROS domain 77. Checks:

* the tab is inactive while the planner flies another place, and its Plan switches it;
* q4 = 90° from Ready To Pick through the carry;
* Ready To Insert holds exactly 150 mm behind the target, level;
* the close happens only on the target, and Exit To Pick by itself;
* the open happens on the target, and Exit To Insert only after the OPEN;
* the gripper closes after the exit, and the arm folds home (q4 0) by Land Start.

Snapshots: `shots/`.

### Isaac, headless, whole-body 4-D L1 rig (RTF 1)

`tools/run_mail.sh <tag>` runs one flight (`MAIL_FEEDBACK=raw|fused`), driven by `pnp_mission_v2.py --mail`
(the tab's sequence, gripper via the action). `tools/mail_score.py` scores it.

All four missions completed (pick → carry → insert → release → exit → land) with 0 aborts. mail2–4 fly
the final configuration:

| run | feedback | envelope ends | mouth margin below / above / side [mm] | jaw tips → wall | far end → back wall | props over box | pitch in the jaws, mean (min–max) | claw error at close / open (3-D) | peak tilt pick / carry / insert |
|---|---|---|---|---|---|---|---|---|---|
| mail1 † | raw | in the box, **hung on the lip** | (80 mm mouth) | 27 mm | 39 mm | 71 mm | 13.9° (7–18) | 21 / 9 mm | 2.6 / 2.5 / 2.8° |
| mail2 | raw | **on the floor of the red box** | 36 / 17 / 42 | 35 mm | 35 mm | 75 mm | 8.2° (6–11) | 6 / 9 mm | 2.9 / 2.7 / 3.4° |
| mail3 | raw | **on the floor of the red box** | 30 / 18 / 45 | 35 mm | 27 mm | 78 mm | 10.9° (8–14) | 10 / 11 mm | 3.1 / 2.8 / 4.2° |
| mail4 | **fused** (the hardware path) | **on the floor of the red box** | 29 / 24 / 45 | 41 mm | 43 mm | 74 mm | 12.9° (12–14) | 17 / 15 mm | 4.0 / 2.9 / 3.1° |

† mail1 is the first flight: 80 mm mouth, claw 9.5 mm above its centre. It showed the two things fixed
afterwards:

* The envelope rides **pitched 7–18° in the jaws** (its weight moment in the pinch), not ≤10°. The claw was
  raised to 21.5 mm above the mouth's centre and the mouth made 90 mm.
* The invisible grip pad **hooked the mouth's lip**: the envelope stood vertical inside, hanging from it.
  This is a sim artifact; the pad's collider now goes off when the jaws open at the mailbox.

Other observations:

* q4 held 90–95° through every carry.
* The pitch tracks the grip offset: 8° with a 6 mm grip error, 13–14° with 17–21 mm. The heavy end hangs
  further when the jaws take the envelope off-centre.
* The released envelope falls and comes to rest on the box floor standing on its end (90–130°). The box is
  only 125 mm deep against the 150 mm envelope, so it stands rather than lies flat.
* Every margin stayed ≥ 17 mm; the closest was the band above the mouth (17–24 mm) and the back wall
  (27 mm).

Data in `runs/` (npz, driver / stack / scene / Isaac logs, `*.score.txt`).

## 4. On the hardware

* **Fly the EKF2-fused stack** (no contact: the pick-and-drop's path):
  `AM_HW_PROFILE=mail_delivery start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator_fused.sh`.
  Orin: pull and rebuild `fsc_trajectory_planner`. Laptop: rebuild `utils_custom_ground_station`.
* **Envelope row:** the envelope on the hat as designed, then Get. Check the yaw is the drone's approach
  heading.
* **Mailbox row:** lay the envelope flat on the mouth's lower lip, pushed in until its far end touches
  the back wall, hold it level (its centre of mass is then inside and it would tip in), then Get. By hand:
  **x − 0.036** along the heading (the design release is 36 mm short of that touch point) and **z + 0.065**
  (the claw rides 64.5 mm above an envelope lying on the lip). Type yaw 0 (facing the mouth). The boxes
  show two decimals, so set the exact value with `ros2 param set`: rounding to x − 0.04 / z + 0.06 puts the
  claw 4.5 mm low and leaves 1.5 mm of the 6 mm room below the mouth.
* **Hover ≥ 6 s before Insert.** The descent trim averages the claw's error over its window (5 s on
  hardware) and cancels the standing part.
* **Open items:**
  * The real OM-X gripper's open gap at the fingertips. The sim's is 37 mm; the envelope needs ~4 mm
    plus the vertical error.
  * How the foam pads hold a 4 mm plate. Its pitch in the jaws is assumed 0–20°.
  * The props' downwash on the box top. They pass 77 mm over it for ~3 s. The 10-14 bin drove a 0.6 Hz
    pitch oscillation from 1.3 m; watch it on the first flight.
  * The mailboxes' real wall thickness (5 mm assumed). Every distance above is measured from the outer
    faces.
