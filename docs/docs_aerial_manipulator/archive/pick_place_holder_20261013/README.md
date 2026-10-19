# Pick-and-place scene: the screwed-on HOLDER and the experiment's pillar heights (2026-10-13)

User request: replace the simulated basket's handle with `rotorcraft/assets/Holder.STL`, and make
the pillar heights consistent with the experimental settings. Scene:
`application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py`. Commands: Command.md §16.

## What changed

| | until 2026-10-12 | now (default) |
|---|---|---|
| Payload | `Box_Payload.usda`: the one-piece basket + wire hanger (3 × 2.5 mm stem) | `Box_Payload_Holder.usda`: the same basket + the holder (`PEGASUS_PNP_PAYLOAD=holder`; `box` and `plate` still selectable) |
| What the jaws close on | nothing (the sim claw cannot close below ~19 mm) | an invisible 20 mm grip pad on the stem + fin (`PEGASUS_PNP_FIN_GRIP_MM`, 0 = off) |
| Pick pillar top / platform top / basket at rest | 1.000 / 1.008 / 1.0405 m | 0.8595 / 0.8675 / **0.900 m** |
| Place pillar top / platform top / basket at rest | 1.000 / 1.008 / 1.0405 m | 0.8415 / 0.8495 / **0.882 m** |
| Sim place point z (`pick_place_place_point`) | 1.02 | **0.87** (sim pick-and-place yaml only) |

## The holder (`tools/holder_stl_to_usda.py`)

`/usr/bin/python3 tools/holder_stl_to_usda.py` (plain python3: `pxr` is in the user site) reads
`Box_Payload.STL` and `Holder.STL` and writes `assets/Box_Payload_Holder.usda`.

- `Box_Payload.STL` holds two closed shells: the basket (110 × 110 × 65 mm, 152.7 cm³) and the old
  wire hanger (2.8 cm³). Only the basket is kept.
- `Holder.STL` (mm, exported upside down, 13.3 cm³): the old hanger's arch and struts, 10 mm deep
  instead of 2.5; a stem 3 × 10 mm; a FIN under the arch, 3 mm thick, 20 mm deep, 50 mm tall, two
  10 mm corner radii; two lugs 2.5 × 15 × 15 mm with two 3.3 mm holes each.
- **How it sits on the basket is derived, and the converter refuses a part that does not fit:**
  the lugs have the old feet's outer width (115.25 mm) and height (15 mm), and the old feet carry
  the same two holes. The holder's hole line is put on the old feet's: 25 mm behind the basket
  centre, 23 and 28 mm above it (the basket has the matching hole pairs at x = ±25 mm). The arch
  is then exactly where the old one was (top 200.9 mm, flat underside 199.5 mm above the basket
  centre).
- **Which way it faces is the one thing the holes do not fix** (they are on the lugs' centre line).
  Default `--fin centre`: the fin points to the basket centre, away from the approaching vehicle.
  Reasons: the flat part is then flush with the lugs' back face, as the old hanger was with its
  feet; its mid-plane stays at x = −27.5 mm, the old hanger plane; and the claw point of EE Offset
  x = −0.02 lands inside the fin (x −22.5 … −2.5 mm). `--fin vehicle` turns it round. **If the real
  holder is mounted the other way, regenerate with that flag.**
- Frame as `Box_Payload.usda`: origin = basket centre (= obj_0), +z up, +x = the approach
  direction, +y = the jaws' closing axis. Colliders: basket box + 40 prisms (the flat part, exact)
  + the fin (a 24-gon prism) + 2 lug boxes. Mass properties: both closed solids at one density,
  scaled to 200 g (holder 16 g); CoM (−1.9, 0.0, +4.7) mm from the basket centre.
- Free-hang angle by its arch: atan((27.5 − 1.9) / (199.5 − 4.7)) = 7.5°.

## The grip pad

The real gripper's foam pads close fully on the 3 mm fin. The sim claw's slider-crank stops the
pads ~19 mm apart (~14 mm at the fingertips), so it would close on air. The scene therefore adds an
invisible collider on the payload over the stem + fin (x −32.5 … −2.5 mm, 149.5 … 199.5 mm above
the basket centre), `PEGASUS_PNP_FIN_GRIP_MM` wide (default 20, the width the push scene's fin flies
with; 0 = no pad). The pads span 15 mm beyond to 51 mm behind the claw point, so with the claw
point at x = −20 mm they close on it.

## Pillar heights

From the 2026-10-10 flight's bag: Get read z = 0.9002 m for the basket on the pick hat, and the
basket lay flat on the place hat at 0.882 m. The scene makes each pillar as tall as puts the
basket centre there: rest height − 32.5 mm (half the basket) − 8 mm (the hat's platform above the
pillar top) = 0.8595 m (pick) and 0.8415 m (place). `PEGASUS_PNP_PICK_PILLAR_HEIGHT` /
`PEGASUS_PNP_PLACE_PILLAR_HEIGHT` set the pillar tops directly (1.0 for both = the old scene).
This is an inference from mocap readings: the real mocap body's origin need not be the basket
centre to the millimetre. A tape measure of the pillars would be better.

The drone waypoints were already the experiment's (Start / Place Start / Land Start at z = 1.0,
Land 0.8), so only the place point changes: `make_pick_place_yaml.py` (pick_place_tune_20261001)
gained `PLANNER_OVERRIDES` and writes `pick_place_place_point: [-1.0, -1.0, 0.87, -180.0]` into
the SIM pick-and-place yaml — 0.87 is what was typed in the 10-10 flight (Get z − 0.01). The mirror
and the hardware yaml keep 1.02 (on hardware it is typed or taken with Get before every flight).
The modular pick-and-place yaml was regenerated from it. `pnp_score.py` accepts the new place
platform top and tests `picked` against the basket's own start height.

Resulting claw targets: pick 0.900 + 0.16 = 1.06 m, place 0.87 + 0.16 = 1.03 m — the flight's 1.0602
and 1.03.

## Test flights (headless, whole-body 4-D L1, raw mocap, `tools/run_pnp.sh wb <tag>`)

| | `holder1`: grip pad 20 mm (default) | `holder_nopad1`: `PEGASUS_PNP_FIN_GRIP_MM=0` |
|---|---|---|
| Mission | complete, basket picked and placed upright | complete, picked and placed upright |
| Jaws at the pick | stall on the pad at −39.6° | close fully, −50° (on air) |
| Basket vs the fingers' pitch (`tools/hang_check.py`) | turns with the claw one to one (slope 1.02, corr 1.00): +7° after the lift, −9° in the carry pose, 0° in the place descent | hangs at 8–9° throughout (slope 0.00) |
| Basket centre below the claw point | 152–161 mm (held where it was clamped) | 186 mm (the arch on the fingers) |
| Basket centre along the heading from the claw, place descent | +29 mm (planner assumes +20) | +5 mm |
| Touchdown, from the place pillar's axis | 18 mm | 28 mm |
| After the touchdown, before the release | basket dragged 61 mm toward the vehicle, last motion 1.36 s after touchdown | moved 9 mm, 0.32 s |
| Vehicle pushed back by the set-down | 225 mm | 125 mm |
| Basket at the end, from the axis | 50 mm, flat, z = 0.882 | 36 mm, flat, z = 0.882 |
| Peak tilt | 5.0° (the lift) | 3.0° |

What this says:

- **The clamped sim reproduces the flight's set-down.** The vehicle is pushed back 225 mm (10-10
  flight: 227 mm) and drags the clamped basket across the hat before the release (sim 61 mm; flight
  ~12 cm, over the rim). In the sim the basket stays on the 200 mm hat because it touches down
  18 mm from the axis, where the flight's touched down 85 mm from the hat centre.
- **The sim's clamp is rigid; the real one slips.** In the flights the stem slipped to about the
  free-hang angle at the lift and then followed 57–84 % of the claw's pitch changes, so the basket
  reached the place tilted 18–22° with its centre 17–27 mm BEHIND the claw. In the sim it follows
  100 %, and because the pick and place arm poses are equal it arrives level, its centre ahead of
  the claw. So the 37–47 mm "hang" part of the flights' place error is NOT in the sim. Lowering
  `PEGASUS_PNP_GRIP_TORQUE` (0.3 N·m) or `PEGASUS_PNP_GRIP_FRICTION` would let it slip; not tuned.
- **The sim fingers carry the arch 186 mm above the basket centre; the real ones 163–171 mm.** The
  sim finger tops are lower relative to the claw point than the real gripper's. With the pad on it
  does not matter (the basket never hangs by its arch).
- One flight each. Not repeated.

## Not changed

- The hardware yamls (the place point default there is still 1.02; typed or Get at flight time).
- `results/utils/build_pick_and_place.py` (its scene text and pillar drawing describe the 10-09
  campaign's 1 m pillars) and `pick_place_top_hat_20261008/tools/abort_score.py` (`HAT_Z = 1.04`):
  both belong to runs flown on the old scene. Update them before re-flying those campaigns.
- The decoupled rig was not flown on this scene.

## Files

- `tools/holder_stl_to_usda.py` — the converter (asserts the fit; `--fin centre|vehicle`, `--mass`).
- `tools/run_pnp.sh` — one headless mission (the 10-08 copy + `PNP_SCENE_ENV="K=V …"`).
- `tools/hang_check.py` — basket tilt vs fingers' pitch for a run here (uses the 10-10 analysis tool).
- `runs/holder1.*`, `runs/holder_nopad1.*` — the two flights (npz, driver log, scene / stack logs).
