# A valve handle the clamped claw can roll on (2026-10-30/31)

**Current design: the ROLL FIN (third version, 2026-10-31, user request: "fewer components and fewer
3D-printed components, a visualization of each component, and the fin must stand vertically before it
is grasped").** Section 4 of the "Valve Turning Design" page (https://claude.ai/artifact/BppzF9zYUZoNwrdPymSQoi)
shows it: assembled/exploded 3D, a picture of each component, dimensioned face-on and section drawings.
Earlier versions kept below: the round grip sleeve (second version, `section4_sleeve.*`,
`tools/roll_grip_sleeve_design.py`, `stl/superseded_sleeve/`) and the hinged paddle (first version,
`section4_paddle.*`, `valve_turning_design_paddle.html` = the page base the builder splices into,
`tools/handle_design.py`).

## The roll fin

- Saw today's fin off 30 mm above its root (z 104). Printed **base** (1): clamp over the stump (5.4 mm slot,
  4 mm walls, sits on the root's ledges) with 2 x M3 x 16 + 2 M3 nuts (in pockets on +x), a 5 mm web, a
  Ø6 neck and a Ø9 boss on the claw's side holding the M4 screw (it cuts its own thread in a Ø3.5 pilot).
  Printed **roll fin** (2): a flat plate 56 (along the axis) x 52 x 8 mm, rounded, with a Ø4.4 bore along
  the roll axis (fin plane, 30 deg up toward +y, through G), turning on an **M4 x 70** socket head (3); an
  **O-ring 4 x 1 mm** (4) between the neck and the fin. Two printed parts + 4 kinds of hardware, 6 pieces
  (the sleeve needed 8 pieces: washers, nyloc). Two printed parts is the floor: one part must turn, one must
  hold the screw on the 30-deg line, and the 5 mm stump cannot.
- **Stands vertical before the grasp**: the screw squeezes the O-ring; 2-4 mN.m of friction holds the fin where
  it is set (bench: a 10 g weight at the fin's bottom edge, 27 mm out = 2.6 mN.m, should just turn it).
  Gravity adds 0.8 mN.m/rad (CoM 4 mm below the axis). A fin up to ~30 deg off is still taken (the jaws open
  37 mm and squeeze it flat). After the grasp it turns against the same 2-4 mN.m (vs ~70 mN.m rms of the
  fixed fin on 10-29); during the turn the push pulls the fin up the screw OFF the O-ring onto the head, so
  ~1 mN.m. Screw tight = locked (the A/B).
- Flat faces like today's fin: 14.5 mm jaw clearance a side; the 8 mm fin may stop the real gripper before
  its target -- check on the bench; if the Valve tab calls it a wrong grip, start the arm GS with
  `push_pull_grip_max_open` just wider. No planner/law change (G, offsets, obj_0 unchanged).
- **Sized from the three hardware grasps (2026-10-31, user's question "is the hold height high enough?")**:
  at the jaw close (planner arrival_error + the EE frame) the claw was within 4 mm of G ALONG THE CLAW
  (+3.5 / -1.7 / +1.2) but 10 / 22 / 13 mm SHORT along the fin's length, always toward the approach side
  (-s). The first cut (50 x 44, 14 mm above G along the claw vs today's fin's 23-35) would have put 7 mm of
  pad onto the boss's edge on the 22 mm grasp. Now: fin s -34..+22 and c -22..+30 -> slop 24 mm toward -s,
  12 toward +s; the pads reach the boss only 30.8 mm off; the gripper body 18 mm above the fin's top
  (today's fin ~11); the handle's top z 208 (today's 204). Boss shortened to 7 mm of thread so the
  M4 x 70 still reaches.
- Numbers (`roll_grip_design.txt`): fingertips 13 mm above the web; screw 67 MPa / M3 19 N at 5 N;
  +35 g on the rotor (~+35 % inertia).
  `check_on_model.py`: no overlap with any of the 12 bodies of Valve_Assembly_v2.stl except the drilled M3
  holes; marker E's ball 10.8 mm from the base, 8.3 from the bolts; the fin swings +-45 deg without touching
  anything (0.8 mm O-ring gap, constant).
- Files: `stl/handle_base.stl` (upright), `stl/roll_fin.stl` (flat on its face), mm;
  `stl/Valve_Assembly_v2_roll_fin.stl` = the model with the fin cut, the stump drilled and the handle fitted,
  every part a separate closed body (20; metres, CAD frame). **The same file is the asset
  `rotorcraft/assets/Valve_Assembly_v3.stl`** (user request 2026-10-31), written by
  `tools/check_on_model.py --asset`. No overlap with the model; the only overlap between new parts is the
  M4's thread in the boss's Ø3.5 pilot (34 mm^3, intended). The neck is a Ø4.4 clearance hole (2026-10-31).
  `stl_to_usda_valve.py` was NOT updated for v3 (it expects v2's shells): no Isaac asset from it yet.
  `img/c0..c6_*.png` = the component pictures. Rebuild:
  `PYTHONPATH=~/sources/pymanifold /usr/bin/python3 tools/roll_grip_design.py`, `tools/check_on_model.py`,
  `PYTHONNOUSERSITE=1 PYTHONPATH=~/sources/pymanifold /usr/bin/python3 tools/render_components.py`, then
  `python3 tools/build_page.py`.
- Not done: no Isaac asset; the OM-X finger/pad numbers are assumed; the O-ring friction is estimated
  (set it with the weight on the bench).

## Second version: the roll-joint sleeve (superseded)

- Saw today's fin off 30 mm above its root (z 104; 98..106 fits). A printed **handle body** clamps over the
  stump (5.4 mm slot, 4 mm walls, sits on the root's ledges) with 2 x M3 x 16 drilled through the stump
  (nuts in pockets on +x); a 5 mm web (the fin's thickness, with a 60-deg lightening window) rises to a
  Ø13 boss + Ø8 neck on the claw's side (-y). A printed **grip sleeve** Ø16 x 50 turns on an M4 x 70 socket
  head screw (50 mm smooth shank = a plain bearing) along the roll axis of the first version (fin plane,
  30 deg up toward +y, through G), held by an M4 nyloc in a hex pocket in the boss, M4 washers both ends.
- The sleeve IS the joint and it is round, so nothing locks or centres it before the grasp; once the jaws
  close it turns with them. Everything else the pads can reach is thinner than the sleeve (web 5, neck 8,
  washers 9, head 7); the Ø13 boss is reachable only 21.8 mm off along the axis (close gate 20).
- G, push_pull_ee_offset, the axis offset and obj_0 are unchanged (sleeve centred on G, markers untouched).
- Lock for the A/B: tighten the screw (sleeve clamped, ~0.6-1.2 N.m holding); free = half a turn back.
- **Arm GS on hardware: `push_pull_full_close_is_grasp:=false`** (append `-p ...` to
  `fsc_open_manipulator/scripts/indoor_exp/start_ground_station_inverted.sh`): the jaws STALL on the 16 mm
  sleeve, so a stall is the grasp and a full close a miss. Read at start-up; shared with Box Pushing, so
  valve sessions only. No planner/law change.
- Numbers (`roll_grip_design.txt`): friction 0.4 mN.m in the turn dry (5 at the force guard); screw 52 MPa,
  M3 19 N shear, web 2 MPa at 5 N; rotor +28 g at r 160 (about +28 % inertia, masses at 84 % of solid PLA);
  top z 198 (fin 204); fingers to clamp 22.7 mm and gripper body 17.5 mm at 20 mm claw error.
- Files: `stl/handle_body.stl` (print upright, clamp down, brim), `stl/grip_sleeve.stl` (on its end), mm.
  Regenerate: `PYTHONPATH=~/sources/pymanifold /usr/bin/python3 tools/roll_grip_design.py` (manifold3d 3.5.4
  installed with `pip install --no-deps --target ~/sources/pymanifold`; no venv module here), then
  `python3 tools/build_page.py` (splices section4.html/.js + roll_grip_parts.json into the paddle page).
- **Checked against the whole current model** (`tools/check_on_model.py`, all 12 bodies of
  `Valve_Assembly_v2.stl`, the 10-28 21:57 export with the redesigned markers): no overlap with anything except
  the two M3 bolts through the stump (0.069 cm^3 = the drilled holes); the clamp sits on the root (gap 0.0);
  nearest marker body is marker E's ball: 10.8 mm from the handle body, 8.3 mm from the M3 bolt ends; every
  other body >= 20 mm. It writes `stl/Valve_Assembly_v2_roll_grip.stl`: the model with the fin cut at z 104
  and the new parts fitted, in the model's own frame and units (metres, CAD frame), so it opens in place over
  the original in CAD.
- Not done: no Isaac asset (a sim collider would need >= 20 mm: the sim claw cannot close below ~19 mm);
  the OM-X finger/jaw numbers are assumed (20 mm wide, 37 mm open) -- check on the real gripper.

## First version: hinged paddle (superseded)

User request: design the handle on `rotorcraft/assets/Valve_Assembly_v2.stl` to alleviate the contact
sway of 10-28/10-29. Published as section 4 of the "Valve Turning Design" page
(https://claude.ai/artifact/BppzF9zYUZoNwrdPymSQoi, version 4; source `valve_turning_design.html` here,
built from `section4.html` + `section4.js` + the cropped CAD data). Numbers: `tools/handle_design.py`
(output `handle_design.txt`); the axis comes from `wb_valve_turning_flight_20261029/tools/handle_joint_mobility.py`.

## The design

- The fin comes off. A paddle (40 x 80 x 6 mm, the old fin's top 80 mm: y -19.4..20.6, z 124..204)
  hangs from a 4 mm steel pin through the grip point G (x 160, y 0.6, z 179 mm = push_pull_ee_offset).
- The pin lies in the fin's plane, 30 deg above horizontal, rising toward +y (the turn direction):
  square to the claw's axis (60 deg down at the flown pose). Roll and sideways sway turn the clamped
  claw about exactly this axis: at the flown pose 1.34 deg per 10 mm of sway -> 0.01 deg left (a
  horizontal pin leaves 0.67). Pitch / fore-aft / vertical turn it about the jaws' axis and are left.
- It must pass through G: an offset d costs K_y d^2 (120 N/m): 12 mN.m/rad at 10 mm, 1.3 N.m/rad for a
  hinge at the fin's base -- more than today's pads + fin (~0.9 N.m/rad: 70 mN.m rms for 4.5 deg rms).
- The post is on the CLAW side (-y): the pin line falls that way, so the post stays low (housing top
  z 168, 11 mm below G), under the gripper, which rises toward the wrist. A first layout with the post
  on +y put it in the fingertips' path (0 mm at the close gate's 20 mm error).
- Two MR84ZZ bearings in the post's housing (s -46..-36 mm along the pin); the pin fixed in a boss on
  the paddle's -y edge (s -27..-16); E-clip behind the far bearing (the + turn's push pulls the pin out
  of the post: 4.3 N at the 5 N force guard); no spring, no damper; gravity centring ~5 mN.m/rad (the
  paddle's CoM 13 mm below the pin + 10 g of steel 42 mm below); stops at +-20 deg; a 3 mm dowel locks
  it for a locked/free A/B.
- Clearances (assumed OM-X finger 20 mm wide, 40 mm back, 20 mm past G): closed fingers to the post
  26 / 16 / 6 mm for the claw 0 / 10 / 20 mm off G; gripper body 38 mm (18 at 20 mm off) all the way
  in, with the post passing between the open fingers; the paddle's +-20 deg swing clears the root and
  marker E. Rotor inertia about +20 %.
- It removes a coupling; it does not damp. A damper at the pin would put the coupling back.

## Not done

No CAD/STL of the parts (no CSG library here; the user owns the CAD). No Isaac asset with the joint
(the sim does not reproduce the hardware sway anyway; it could check the grasp and the turn). For a
-90 deg turn mirror the handle.
