# Pick-and-place tuning for 07's scene (2026-10-01/02)

The user's five requests, for the whole-body L1 4-D rig in
`application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py`:
tune the two claw setpoints and EE offsets for the payload, choose a safe
claw pose (round values), check the gains for a robust and safe task, treat
the payload as **model uncertainty** (no impedance / contact switching), and
test headless at RTF 1. The four drone setpoints were declared fine and are
unchanged.

Everything here flew on shiqi-desktop, headless, real-time pacer on
(RTF 0.99-1.00 every window), the EKF2-fused stack, the mirror plant.

## The configuration (what to fly)

Generated sim yaml
`fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_place.yaml`
= the mirror `_sim.yaml` byte for byte plus the keys below. **Regenerate with
`tools/make_pick_place_yaml.py`, never hand-edit** (`--check` verifies).
Selected by `WB_SIM_PROFILE=pick_place` on the stack; the scene wrapper sets
it itself and warns if the running planner lacks the block.

| what | value | why |
|---|---|---|
| pick / place / carry pose | **[0, -30, 30, 0] deg**, all three | claw straight down (payload level); sigma_nd 0.268 (2.7x the 0.10 keep-out); q3 30 deg from q3 = 0 (the elbow-singular branch) and 20 deg from its +50 stop. [0, -20, 20, 0] crossed q3 = 0 on a release kick (run 7). One pose = no arm reconfiguration with the payload in hand. |
| pick EE offset | **[0, 0, 0.250]** m | claw 15 mm below the handle top; the open jaws pinch the handle ~20 mm in (runs 1, 3 stalled there) |
| place EE offset | **[0, 0, 0.260]** m | box bottom 10 mm above the pillar on release. Not set down (runs 10, 12, below) |
| approach | **`pick_place_approach_dz` 0.10 m, `_hold_s` 2.0 s** | NEW planner option: each claw leg flies to the goal +0.10 m, holds, then descends straight down |
| EE anchor | **`pick_place_world_anchor_pick` true**, `_place` false; node `wb_ee_anchor_blend_s` 1.0 | NEW: the claw is world-held from the approach point through the grasp, CoM-held otherwise |
| arm sweep (execute_place) | q2 -40..-20, q3 20..40 (mirrored, phase 90/270), q1 not swept | payload stays level; the +-25 deg q1 sweep hit the q1 stop during the 177 deg turn (run 3) |
| attitude gains | **`wb_k_r` 1.6, `wb_k_w` 1.2** (mirror 2.134 / 1.567) | H1b's 1.5 Hz pitch mode grew to 8-12 deg in 2 of 8 hovers (runs 7, 8); bench delay margin 28 -> 32-36 ms, nominal tracking unchanged |
| everything else | the hardware controller verbatim | `wb_l1_contact` false (chi free): the payload is model uncertainty |

Scene (07) defaults that changed: **handle 20 mm thick** (was 30; knob
`PEGASUS_PNP_HANDLE_THICKNESS`), `GRASP_BELOW_TOP` 0.015, `PLACE_DROP` 0.010.

**Operator rules** (the driver does exactly this):
1. After `execute_pick`'s gate, close the jaws and **request `go_to_place_start` the moment they stall**.
2. After `execute_place`'s gate, **open at once and request `go_to_land_start` the moment they are open**.
Clamped on a pillar, the vehicle is in a closed chain with the table; under either anchor it
drifts within ~1-3 s (runs 10, 11, 12, 13).

## Results

| run | config change | pick | carry | place | vehicle |
|---|---|---|---|---|---|
| r1 | first try: pose [0,-20,20,0], offset 0.211, approach from above, CoM anchor | stalled 14 mm high, payload pushed off | - | - | crashed at the close |
| r2 | world anchor, H1b arm K_y 211.9 | - | - | - | diverged in hover (q2/q3 +-4 deg at 2 Hz, 11 s) |
| r3 | world anchor + K_y 80/24 | grasped (pinch stall 20 mm in) | carried | sweep: q1 to its stop | tripped |
| r4 | offset 0.250, no q1 sweep | 4 mm, clamped -7 deg | lift: q1 to its stop | - | tripped |
| r5 | back to CoM anchor | 15-20 mm lateral in the descent, knocked off | - | - | - |
| r6 | NEW anchor switch: world for the claw legs | 3-6 mm lateral, fingertip on the 30 mm handle -> arm flip | - | - | crashed |
| r7 | **20 mm handle** | 3.1 mm, clamped -37 deg | carried | placed, then arm flip at release | tripped after release |
| r8 | pose [0,-30,30,0], set-down | missed | - | - | 1.5 Hz mode grew from DIRECT entry |
| r9 | **k_R 1.6 / k_w 1.2** | 4.1 mm | carried | world-held set-down fought the pillar, flip at release | tripped |
| r10 | world anchor for the PICK only | 0.4 mm | lost in the lift (clamped 4 s, q1 to stop) | - | **completed the mission** |
| r11 | CoM before the clamp | 5.4 mm | base dragged the clamped payload off | - | tripped |
| r12 | world through the clamp, lift 0.2 s after | 1.0 mm | carried | set-down: base dragged it 46 mm off | completed, 0 sat |
| r13 | release from 10 mm | 7.5 mm | lost in the lift | - | completed |
| r14 | lift AT the stall | 6.0 mm, -40.8 deg | carried, 3.2 mm vertical slip, tilt <= 8.7 deg | **ON THE PILLAR, UPRIGHT, 17.9 mm off axis** | completed: 0 saturation, 0 clamp, peak tau 1.53 N.m, tilt <= 6.7 deg |
| r15 | repeat of r14 | 1.4 mm, -34.7 deg | carried, 2.3 mm vertical slip, tilt <= 5.2 deg | **ON THE PILLAR, UPRIGHT, 30.2 mm off axis** (released 13 mm up) | completed: 0 sat, 0 clamp, peak tau 0.96 N.m, tilt <= 2.7 deg, \|e_R\| <= 0.071 |

**The final configuration flew 2 of 2 complete missions (r14, r15)**: payload
untouched before the clamp, carried, set upright on the place pillar
(17.9 / 30.2 mm off its axis, pillar radius 50 mm), vehicle landed.

Scores: `tools/pnp_metrics.py runs/<run>.npz`. Offline geometry:
`tools/pp_geometry.py` (the planner's six goals + a port of the sweep, the
payload, pillars, gear and pads). Offline control screen:
`tools/hover_anchor_screen.py` (circle_bench's exact law + mirror plant).

## What was learned (the parts that transfer)

1. **The gripper's fingertips are narrower than its pads.** 07's original
   probe measured 43.3 mm between the pad CoMs; the new JAW GAP probe shows
   **37-38 mm at the fingertips** (16 mm below to 24 mm above the claw
   point), and the handle is pinched ~20 mm in (a ~31 mm gap off the centre
   line; two stalls at the same depth, then clamps at only -3.5 / -7 deg).
   A 30 mm handle leaves 3.5-4 mm a side. 4 of 4 30 mm grasps failed.
2. **The claw's lateral error on the way down is 3-6 mm world-held and
   6-20 mm CoM-held** (the CoM anchor passes the base's hover wander and the
   descent transient straight to the claw). That is why the world anchor is
   needed for the pick descent and a 20 mm handle (8.75 mm a side) is needed
   with it. Hardware hover wander is ~3x the sim's; budget accordingly.
3. **The claw's approach must be vertical.** Planned straight in, the claw
   swings sideways at the end (the heading and joint channels converge
   slower than the CoM, (T-t)^3 vs ^5): 12 mm across the handle 25 mm before
   arrival. NEW `pick_place_approach_dz`: goal +dz, hold, straight down.
4. **The world anchor only while nothing is touched.** A world-held claw in
   contact (a box set on its pillar, a clamped payload still on its pillar)
   fights the contact with the arm: q1 runs to its stop, q4 spins, the arm
   crosses q3 = 0. Flight legs are CoM-held too (the base strays tens of mm).
5. **Any closed chain with the table drifts within ~1-3 s, under either
   anchor** -- lift as soon as the jaws stall; release without setting down.
   This is the one place the "model uncertainty" treatment is thin: while
   clamped on the pillar the payload is not a mass but a contact, and the
   rig survives it only by keeping it short.
6. **H1b's 1.5 Hz pitch mode** (the attitude loop next to the 10 rad/s
   rotor-lag pole, documented at ~2 deg p-p in every RTF-1 run) grew to
   8-12 deg in 2 of 8 hovers here. k_R/k_w x0.75 removed it in every later
   run (r9-r14: hover tilt <= 2.2 deg). Bench delay margin 28 -> 32-36 ms.
7. **The payload as model uncertainty works for flight.** Carrying, the L1
   books -2.7 N on z (weight 1.96 N + the CoM moment's share) and -0.15 to
   -0.28 N.m on j2/j3; u1 rises 38.6 -> 41.3 N; it returns after release.
8. The model claw (current_ee) and the real claw (claw_0, between the pads)
   agree to 1-5 mm at the claw-down pose: no offset correction needed.

## Code changes (all default-off / additive)

- **fsc_trajectory_planner** `pick_place.{hpp,cpp}`: `approach_dz` / `approach_hold`
  (approach point, dwell, vertical descent; `approachRest()`), gtest
  `ApproachFromAboveEndsOnAVerticalDescent`. Node: params `pick_place_approach_dz`,
  `_approach_hold_s`, `_world_anchor_pick`, `_world_anchor_place`, `_anchor_service`;
  calls the WB node's anchor service at the approach point, restores CoM at any
  other motion and on leaving DIRECT. 10/10 pick-place gtests, 29/29 planner gtests.
- **fsc_autopilot_ros2** whole-body client: service
  `whole_body_direct_actuation/set_ee_anchor_com` (SetBool, DIRECT only) blending
  the anchor over `wb_ee_anchor_blend_s`; the yaml anchor is restored on every
  mode change. Law: `WbGains::ee_anchor_weight` (1.0 = bit-identical).
  WbParity/JointDiag/ReferenceBuilder/L1 + modular parity 17/17.
- **07**: probes (JAW GAP, HANDLE CEILING, gear y layout), `claw_0` ground
  truth, `PEGASUS_PNP_HANDLE_THICKNESS`, defaults above.
- **Pegasus launchers**: `WB_SIM_PROFILE=pick_place` in `am_plant_from_yaml.sh`;
  the wrapper defaults it and checks the running planner.

## Not covered

No hardware. The 20 mm handle is a sim choice the physical payload has to
confirm (fingertip gap measured on the USD gripper). The clamped/set-down
contact phases are survived by brevity, not controlled. The hardware yaml has
no pick-and-place block.
