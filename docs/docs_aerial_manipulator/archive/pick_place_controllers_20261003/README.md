# Pick-and-place: arm pose, pillar caps, and both controllers (2026-10-03)

User request: (1) the pick / place arm pose [0, -30, 30, 0] swings in hover --
pick a healthier pose and a "hold" pose for the carry; (2) a flat cap on each
pillar so the payload is not dropped; then fly the AUTONOMOUS pick-and-place
task with the whole-body 4-D L1 controller AND the decoupled geometric + L1
controller -- the same scenario, the same task, only the controller differs.

## 1. The arm pose (offline screen)

`tools/pose_hover_screen.py`: the exact 4-D whole-body law hovering at a pose
on circle_bench's mirror plant (hardware-like feedback, rotor lag, 16 ms
transport, arm friction), with the pick-and-place yaml's gains (H1b, k_R /
k_w 1.6 / 1.2), the claw CoM-anchored (flight) or WORLD-anchored (the planner's
pick anchor: hover above the target, descent, grasp). Raw output:
`runs/pose_screen_1.txt`, `runs/pose_screen_2.txt`; `tools/world_anchor_timing.py`.

| pose [deg] | claw pitch | reach | sigma_nd | world anchor | joint swing (CoM anchor) |
|---|---|---|---|---|---|
| [0, -30, 30, 0] (10-02) | 0 | 0.088 m | 0.268 | **diverged 2 of 3 seeds within ~4 s** | 1.69 deg |
| [0, -40, 40, 0] | 0 | 0.067 m | 0.206 | diverged 1 of 2, the other at 43 deg joint std | 2.10 deg |
| **[0, -20, 30, 0] pick / place** | 10 | 0.130 m | 0.298 | 3 / 3 held 55 s, EE 2.4 mm | **1.18 deg** |
| **[0, -30, 40, 0] hold** | 10 | 0.108 m | 0.329 | 3 / 3 held | 1.36 deg |
| [0, -20, 40, 0] | 20 | 0.146 m | 0.334 | 3 / 3 held | 0.90 deg |

Mechanism: with the claw world-held the arm must cancel the base's hover
wander, and a SIDEWAYS claw error can only be taken out by the arm yaw q1 at a
lever of the claw's horizontal reach: 8.8 cm at [0, -30, 30, 0], so small
errors ask for large q1 swings, which feed back into the base. The vehicle's
own tilt ripple (~2.7 deg p-p at 1.4 Hz, the attitude loop's mode) is the same
at every pose. A 10 deg claw pitch keeps the handle's top edge within +-5 mm of
the grasp depth across the 60 mm plate (20 deg: +-10 mm, into the ~20 mm pinch).

## 2. The pillar caps

07 puts a 160 x 10 mm disc on each pillar, its top at the old pillar top (1.0 m),
so no point or offset moves. 160 mm covers the 110 mm box's whole footprint at
any yaw (diagonal 155.6 mm): a box set down up to ~80 mm off the pillar axis
still rests with its centre supported (the 10-02 places landed 18-30 mm off
axis on a 100 mm pillar). `PEGASUS_PNP_CAP_DIAMETER` overrides.

## 3. The descent trim (planner)

The decoupled law parks the vehicle at (unmatched force) / Kp -- ~37 mm on the
mirror plant with its 10-01 tune -- against +-8.75 mm of jaw clearance on the
20 mm handle. Rather than change either law, the planner now averages the
claw's error while holding above the target and shifts the descent goal by
it (`pick_place_descent_trim`, default on), so the MEASURED claw lands on the
target with either controller. The trim is refused unless the claw's residual
after it is inside the 50 mm tolerance and the trim itself is under
`pick_place_descent_trim_max` (node default 0.10 m; the pick-and-place yaml
sets 0.15 m for the decoupled rig's softer position pair, section 6).

## 4. The autonomous missions

`tools/run_pnp.sh wb|geo <tag>` (after a clean slate in a separate call) brings
the rig up headless and flies `tools/pnp_mission_v2.py` -- the arm GS's
Pick & Place steps, scripted: Ready To Pick (open, hover at the 0.20 m safety
margin) -> Pick (descend, close once the EE is within 2 cm) -> Exit To Pick ->
Go To Place Start -> Ready To Place -> Place (descend, open within 2 cm) ->
Exit To Place -> Go To Land Start -> Execute To Land -> land. Both rigs fly
RAW mocap odometry (the decoupled rig has no fused sim stack) and the same
planner block (the whole-body ..._sim_pick_place.yaml). Controller files:
whole-body `..._l1_4d_..._sim_pick_place.yaml` (archive tool
make_pick_place_yaml.py), decoupled `..._geometric_l1_..._sim_pick_place.yaml`
(`tools/make_geo_pick_place_yaml.py`). `tools/pnp_score.py` scores the npz.


## 5. Results -- both controllers complete the task with the 200 g payload

Final configuration, same scene / task / planner block / feedback, only the
controller differs. Isaac headless at RTF 1, raw mocap odometry, 200 g payload,
mirror plant. `tools/pnp_score.py`; errors are the claw (FK of the measured
state) against the planner's EE reference.

| | whole-body 4-D L1 | decoupled geometric + L1 |
|---|---|---|
| runs | wb_1, wb_2, wb_3: **3 / 3 completed** | geo_11, geo_12: **2 / 2 completed** |
| payload set down, off the place pillar's axis | 2.9 / 17.7 / 29.0 mm | 29.5 / 8.8 mm |
| peak tilt, whole mission | 3.3 / 2.8 / 2.9 deg | 4.3 / 6.0 deg |
| claw from target when the jaws closed / opened | 3.9 & 4.6 / 2.6 & 6.2 / 2.7 & 17.9 mm | 12.3 & 12.2 / 12.7 & 14.2 mm |
| claw hover offset above the target, pick / place | 4.0 / 7.1, 2.3 / 7.7 mm | 52.0 / 46.3, 49.6 / 55.0 mm |
| descent trim applied (wb_2, wb_3 / geo_11, geo_12) | 1-7 mm | 37-55 mm |
| claw rms error carrying (go_to_place_start) | 12.8 / 12.2 / 10.6 mm | 44.6 / 55.0 mm |
| payload slip in the jaws while carried | 0.6 / 3.1 / 1.4 mm | 2.6 / 0.7 mm |
| DIRECT mission time | 100.6 / 104.1 / 104.0 s | 104.3 / 104.5 s |

wb_1 and wb_2 flew before the stream-race fix and the 0.15 m trim cap
(neither touches the whole-body law), wb_3 on the final code. wb_1's driver
requested each descent at once, so its trims read zero.
What the numbers say: both laws carry the payload pillar to pillar and set it
upright on the cap. The whole-body law flies the claw ~5-10x closer to its
reference. The decoupled law parks its claw ~5 cm off (its unmatched
lateral force over Kp 15, no integral); it gets the grasp only because the
planner measures that offset while holding above the target and shifts the
descent by it. Its lift-off kick (the payload's weight landing on the claw as
an unmodelled moment) shows as the 4-6 deg peak on Exit To Pick, then decays.

## 6. What it took for the decoupled law (12 flights)

| run | payload | change being tested | outcome |
|---|---|---|---|
| geo_1 | 200 g | the 10-01 tune as is | exit: z lagged 0.1 m behind the climb, box dragged on the cap, roll grew -> guard 28.7 deg |
| geo_2 | 200 g | kp_z / kv_z 40 / 18 (lift at once) | lifted; post-lift sway -> next leg refused (body 153 mm off its hold) |
| geo_3 | 100 g | lighter payload | **completed** (placed 18.8 mm off axis) |
| geo_4 | 100 g | -- | place hover 74 mm > the 50 mm gate -> led to the descent TRIM + residual gate |
| geo_5 | 100 g | trim | exit: the sway grew -> guard 18.1 deg |
| geo_6 | 50 g | -- | the driver's jaw-stall detector never fired (5 s clamp) -> guard; fixed in the driver |
| geo_7 | 200 g | kp_xy / kv_xy 30 / 13.5 | exit: guard 17.4 deg |
| geo_8 | 100 g | all of the above | pick + exit fine; then a PLANNER STREAM RACE (below) put one reference sample 2.2 m away -> the node's drift watchdog left DIRECT |
| geo_9 | 100 g | race fixed | the sway grew through the next leg -> guard 18.3 deg |
| geo_10 | 50 g | -- | carried to the place start; the sway grew while holding there -> guard 17.1 deg |
| **geo_11** | **200 g** | **kp_xy / kv_xy 15 / 10** | **completed**, peak tilt 4.3 deg |
| **geo_12** | **200 g** | repeat | **completed**, peak tilt 6.0 deg |

**The binding problem was the decoupled law's ~0.6 Hz roll/pitch sway, not the
payload mass.** It is the zeta ~0.1 mode the 10-02 hardware flights found in
the 10-01 tune (archive/decoupled_flight_20261002, lateral_mode_model.py). With
the payload the arm joints flex with it (q1 +-1.2 deg, q2 +-2.4 deg, correlation
0.8 with roll) and the mode grows -- at 50 g too, only more slowly. The grasp
itself stays rigid (claw-to-box 223 +- 1 mm). `tools/geo_sway_screen.py` runs
that linear one-axis model of the node with the payload as extra true inertia
and Isaac's delays: worst-case damping +0.03 for the 10-01 position pair
(20.1 / 11.05), **-0.14 for the kp 30 / kv 13.5 I had tried first** (stiffer was
worse: kv erodes this mode once the payload adds inertia), and **+0.20-0.22 for
kp 15 / kv 10 at 0, 100 and 200 g**. The nonlinear bench keeps the 10-01 tune's
44 ms delay margin (`tools/geo_kp_margin.py --sway`); the price is a larger
standing offset (circle-bench EE 55 -> 68 mm), which the descent trim cancels
(cap raised 0.10 -> 0.15 m in the pick-and-place yaml). Attitude gains and the
L1 are the 10-01 tune's, untouched. Isaac agrees with the model's ordering
(peak tilt per 4 s window after the grasp): at kp 30 the sway grew in 3 of 4
runs (geo_7, 9, 10; geo_8 decayed), at kp 20 in 2 of the 4 runs it can be read
in (geo_2, 5; geo_3, 4 decayed from ~6 to ~1.5 deg; geo_1 and geo_6 failed for
other reasons first), at kp 15 it decayed in both (3.5-6.0 -> 1.2-1.7 deg).

**The planner stream race (fixed, affects BOTH rigs).** `FlatPlan::eval()`
(fsc_autopilot_ros2 wb_law) was `const` but wrote into `mutable` member scratch
buffers. The planner node's worker draws `viz_path` by sampling the new plan out
to t = T without the lock while the 100 Hz stream tick evaluates the same plan
near t = 0, so a tick that overlapped published the leg's GOAL for one sample
(geo_8: x_cd 2.17 m from the vehicle). Now `thread_local` scratch; regression
tests `PickPlace.APlanCanBeEvaluatedFromTwoThreadsAtOnce` (it HANGS against the
old library) and `PickPlace.EveryLegStartsWhereTheHoldIs`. The whole-body rig
never happened to hit it here, but it would have taken the same step on
hardware.

## 7. Controller files

- whole-body: `fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_place.yaml`
  (generated, archive/pick_place_tune_20261001/tools/make_pick_place_yaml.py):
  the mirror yaml with k_R / k_w 1.6 / 1.2, plus the planner's pick-and-place
  block (shared with the decoupled rig).
- decoupled: `fsc_autopilot_ros2/config/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim_pick_place.yaml`
  (generated, `tools/make_geo_pick_place_yaml.py`): the 10-01 tuned
  geometric + L1 mirror yaml with kp_z / kv_z 40 / 18 and kp_xy / kv_xy 15 / 10.

Launch (scene + stack): Pegasus
`scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_pick_and_place_sitl.sh`
for the decoupled rig, `..._whole_body_L1_4D_pick_and_place_sitl.sh` for the
whole-body rig. Both simulation-only; neither file has been flown on hardware.

## 8. Arm singularity margin of the poses (`tools/arm_singularity_check.py`)

The task is claw position + claw roll about its own axis, and the claw lies ON
the wrist-roll axis, so J_3y is singular exactly when the position Jacobian of
(q1, q2, q3) is -- there is no wrist singularity in this task, and q1 / q4 do
not change the margin. Inside the joint box there are two singular sets:
(1) the claw directly UNDER the arm-yaw axis (q1 then cannot move it): a curve
from (q2, q3) = (-63, 50) to (-15, -27) deg, roughly q3 = -12 - 1.65 (q2 + 24);
(2) the elbow straight in the q2/q3 plane: q3 = -31 ... -38 deg, every q2.

| pose [deg] | sigma_nd (kinematic) | sigma_nd (law) | claw reach from q1 axis | nearest singular point | weakest direction |
|---|---|---|---|---|---|
| home [0, 40, 40, 0] | 0.334 | 0.887 | 253 mm | 70 deg | q2/q3 plane |
| OLD pick [0, -30, 30, 0] | 0.268 | 0.680 | 88 mm | **16 deg** (set 1) | **q1 vs q4: the claw's lateral motion** |
| pick / place [0, -20, 30, 0] | 0.298 | 0.682 | 130 mm | 25 deg | q2/q3 plane |
| hold [0, -30, 40, 0] | 0.329 | 0.682 | 108 mm | 21 deg | q1 vs q4 |
| all-zero [0, 0, 0, 0] | 0.169 | -- | 155 mm | ~31 deg (set 1), 35 deg (set 2) | q2/q3 plane |

Keep-out (planner, teleop, arm sweep) is sigma_nd >= 0.10. Along all four arm
moves of the mission (home -> pick -> hold -> place -> home) the smallest margin
is at the pick / place pose itself (0.298): no transition passes closer. The
old pose's weakest direction is the lateral claw motion through q1 at an
88 mm lever -- the same mechanism that made its world-anchored hover diverge
(section 1). The law's own margin (it can also move the base) is flat at 0.68
for all three poses.

## 9. The decoupled rig unstable at the grasp -- debugged (2026-10-03, user report)

The user, flying the decoupled block by hand: "when I grasp the payload, it is not
stable". Their run (ROS logs, 20:49-20:51): descent complete at 78.2 s, Exit To Pick
pressed at 82.2 s (~4 s clamped), safety guard at 16.9 deg 0.8 s later, node watchdog
at 20.3 deg. Reproduced (`--clamp-hold 4`, geo_13): guard at 24 deg 1.8 s into the
hold -- BEFORE any climb. Four things were wrong; each is fixed where it lives, and the
fixes are the same for both controllers (and the hardware):

1. **Closing off-centre across the handle.** The open fingertips clear the 20 mm handle
   by only 8.75 mm a side. The three runs that diverged while clamped (geo_13/14/16) all
   closed 7.6-7.8 mm off along the jaws' closing axis -- one finger was already on the
   handle and shoved the vehicle sideways; runs closing <= 6 mm off did not (the
   whole-body rig closes within 0.7 mm). FIX (arm GS): the Pick closes only once the
   claw is CENTRED across the handle, `pick_place_grip_axis_tol` 3 mm (the captured
   object's y axis, from the planner's info[80]), as well as within the 2 cm
   `pick_place_grip_tol`. The amber lamp shows the closing-axis error while it waits.
2. **The descent ended off-centre.** The descent trim (~5 cm for the decoupled law)
   was flown INSIDE the descent -- a diagonal path -- and the soft lateral loop
   overshot ~10 mm when it stopped, at the bottom next to the handle (geo_22: a finger
   on the handle, the claw then swung +-0.1 m around it and knocked the payload off its
   pillar). FIX (planner): `pick_place_descent_trim_settle_s` 2.0 -- a trim > 5 mm is
   flown SIDEWAYS at the hover height, held 2 s, then the claw descends STRAIGHT down.
3. **Staying clamped while the payload still rests on the pillar.** Even centred, the
   clamped chain grows a roll / sideways divergence on the decoupled rig from the jaw
   stall (doubling ~0.35 s) until the box leaves the cap. With an operator pressing
   Exit (~4 s) that is always fatal. FIX (arm GS): when the pick's GripperCommand goal
   returns the grasp (ABORTED + stalled), the tab starts **Exit To Pick by itself**
   (`pick_place_auto_exit_pick`, default true); closed-on-nothing is reported instead.
4. **The lift was too slow to unload the box**, and the sim never reported the grasp.
   (a) A 3.2 s min-snap exit climbs 8 mm in its first 0.7 s: the box stayed on the cap
   ~0.9 s into the exit. FIX (planner): `pick_place_exit_time_s` 1.6 -- the reference
   rises at > 0.1 m/s by 0.4 s, where the decoupled law's kv_z alone lifts the 200 g.
   (b) The sim gripper action server judged the stall on the joint's instantaneous
   velocity, which jitters while clamped -- the grasp was reported 3+ s late or never
   (geo_14), so the GS would never have lifted. FIX (fsc_open_manipulator isaac bridge):
   stall = the hub moved < 1 deg over the last 0.3 s.

| runs (decoupled, 200 g) | fixes in | result |
|---|---|---|
| geo_13 | none, Exit pressed 4 s after the grasp | guard 24 deg while clamped |
| geo_14 | auto-lift on the action result (old sim stall test) | grasp never reported, guard 20 deg |
| geo_15 / 16 | + position-window stall test | completed / guard 16.7 deg (closed 7.7 mm off) |
| geo_17 / 18 / 19 | + centred close (3 mm) | completed / completed / guard 18.9 deg (lift too slow) |
| geo_20 / 21 / 22 | + 1.6 s exit | completed / completed / payload knocked off at the bottom (trim overshoot) |
| **geo_23 / 24 / 25** | **+ trim first (all four)** | **3 / 3 completed**, lift peak 7.5 / 5.6 / 5.3 deg, placed 11-25 mm off axis |

What is left on the decoupled rig is the LIFT-OFF transient: the 200 g lands on a claw
this law has no model of, the base sags ~4 cm and the 0.6 Hz sway rings to 5-8 deg,
decaying, while its L1 (1 rad/s) absorbs the weight over ~2 s. The whole-body rig with
the same changes (wb_4): completed, peak tilt 3.3 deg (lift 2.6 deg), placed 10 mm off axis -- its trims are 1-3 mm, so its descent stays one segment.

Tools: `pnp_mission_v2.py` `--clamp-hold`, `--grip-via-action` (the GS's grasp path:
GripperCommand, exit on its result), `--grip-axis-tol`; `run_pnp.sh` passes
`PNP_DRIVER_ARGS`.

## 10. Whole-body stability tuning (2026-10-03, user request) -- no change adopted

Offline (`tools/wb_payload_bench.py`: the mirror plant hovering at the pick pose with
the 200 g box added rigidly at +10 s and removed at +25 s, the law blind to it;
`tools/wb_payload_sweep.py` at 16 / 28 / 32 / 36 ms of transport delay): k_w is the
lever there (more attitude damping cuts the load kick) but beyond ~1.5 it eats the delay
margin; M_r_d up hurts badly; omega_c_q, K_y / D_y, omega_x barely matter. The bench's
best, k_R / k_w 1.4 / 1.5 (shipped 1.6 / 1.2), cut its load kick 22 % and flew 32 ms where
1.6 / 1.2 aborts.

**Isaac did not confirm it** (9 full missions; `tools/wb_mission_stats.py`, ripple =
attitude minus its moving average):

| gains / grip | runs | ripple > ~1 Hz roll / pitch rms | ripple > ~0.5 Hz roll rms | EE rms |
|---|---|---|---|---|
| 1.6 / 1.2, grip 0.3 N.m (shipped) | wb_1-4, wb_8 | 0.06-0.14 / 0.07-0.14 deg | 0.26-0.44 deg | 11.3-12.8 mm |
| 1.4 / 1.5, grip 0.3 | wb_5, wb_6 | 0.07-0.11 / **0.13-0.22 deg** | 0.21-0.22 deg | 12.6-14.4 mm |
| 1.6 / 1.5, grip 0.3 | wb_7 | -- / pitch at the lift 6.7 deg p-p | 0.30 deg | 13.0 mm |
| 1.4 / 1.5, **grip 1.0 N.m** | wb_9 | 0.07-0.10 / 0.09-0.17 deg | 0.29 deg | 16.8 mm |

More rate damping raised the fast PITCH ripple (1.4-1.8 Hz, nearer the rotor-lag band)
and the EE error; only the slow roll improved. **Reverted to 1.6 / 1.2.** What the data
says about the whole-body rig: its fast ripple is ~0.1 deg rms already; the 2-5 deg
per-step excursions are slow transients (leg accelerations, the payload's load and
release), and per-event peak-to-peak values scatter 1-2 deg between identical runs.

**The payload is held rigidly** -- measured, correcting a first reading. Decomposed
into body axes, the box's rotation in the jaws is a 0.75-0.95 deg rms fore-aft
oscillation and a slow 9-18 deg slip about the vertical during the 180 deg carry turn;
the 22-33 mm "swing" of the box under the claw is mostly the box moving rigidly with
the vehicle (0.22 m below the claw). A 3.3x firmer grip (PEGASUS_PNP_GRIP_TORQUE 1.0,
wb_9) changed none of it, so the scene keeps 0.3 N.m.

## 11. Benchmark comparison: Suarez et al., RA-L 2020, grasping benchmark (2026-10-04, user request)

Metric (Eq. 9): eps_UAV = || r_UAV^ref - r_UAV || (aerial platform, Earth frame), rho_UAV =
eps_UAV / L with **L = L1 + L2 = 0.370 m** (OM-X maximum reach: shoulder -> elbow 0.130 m,
elbow -> claw 0.132 + 0.108 m). r_UAV = the airframe (body origin) from odometry; r_UAV^ref =
the planned airframe position x_cd - R_z(psi_d) r_0c(q_d) from the planner's stream on its own
model, identical for both rigs (checked in hover: the CoM error and the body error agree to
0.4 mm). Protocol (Sec. IV-B): N = 5 per controller, success ratio, per-phase maximum
deviation (the GRAB row is the paper's eps_UAV^grab), the way-point condition eps < 0.25 L,
t_10% (until eps <= 0.1 L, held 1 s). Isaac HEADLESS at RTF 0.995-1.000 (every run, the
pacer's 10 s windows), interleaved wb / geo, the arm GS's grasp sequence (centred close +
automatic Exit). Runs `runs/bench_{wb,geo}_1..5`, numbers `runs/bench_metrics.txt`, figure
`runs/bench_rho_uav.png`, tools `tools/bench_campaign.sh`, `tools/bench_metrics.py`.

| | whole-body 4-D L1 | decoupled geometric + L1 |
|---|---|---|
| success (payload set upright on the place cap) | **5 / 5** | **5 / 5** |
| approach: max eps (rho) / rms | 12.6 mm (0.034) / 5.0 mm | 54.7 mm (0.148) / 49.5 mm |
| **grab: max eps (rho)** / rms | **46.9 +- 2.1 mm (0.127)** / 14.3 mm | **189.9 +- 54.5 mm (0.513)** / 59.1 mm |
| carry: max eps (rho) / rms | 48.3 mm (0.130) / 12.6 mm | 185.3 mm (0.500) / 44.2 mm |
| place: max eps (rho) / rms | 59.9 mm (0.162) / 22.6 mm | 133.8 mm (0.361) / 46.7 mm |
| return: max eps (rho) / rms | 31.6 mm (0.085) / 12.2 mm | 61.8 mm (0.167) / 50.8 mm |
| t_10% after the lift-off | 1.9 s (5/5) | 7.5 s (5/5) |
| t_10% after the release | 1.1 s (5/5) | 3.0 s (3/5; 2 never inside 0.1 L) |
| way-points with eps >= 0.25 L | 0 / 40 (worst 34 mm, rho 0.093) | 5 / 40 (all at Exit To Pick; worst 283 mm, rho 0.76) |
| grab / place phase time | 8.8 s / 10.7 s | 15.1 s / 14.8 s |
| (task accuracy) claw from target when the jaws close / open | 1.0-10.1 / 6.9-15.6 mm | 2.2-8.4 / 5.2-12.6 mm |
| (task accuracy) payload set down, off the pillar's axis | 9.6-24.2 mm | 4.4-15.7 mm |

Both peaks are HORIZONTAL and come right after the payload's weight changes: at the lift-off
the decoupled airframe swings 150-295 mm sideways (the whole-body one 40 mm, with 23 mm of
sag), and after the release 120-160 mm (whole-body ~65 mm). At rest the decoupled airframe sits
~50 mm off its reference (rho ~0.13 -- its unmatched lateral force over Kp 15, no integral),
which is above 0.1 L, so its t_10% is reached only where the payload's weight shifts that
offset. The decoupled rig's longer grab / place phases are the trim-first sideways move and the
centring wait. The TASK accuracy is comparable (last two rows): both rigs close only once the
claw is centred and land the box on the cap, the decoupled one by the planner's descent trim
and centring wait; the difference this benchmark measures is how much the AIRFRAME moves to
get there -- 4x more for the decoupled law at the grab, 2x at the place. Read the whole-body
grab row knowing the pick is WORLD-anchored by design (the claw held in the world, the
airframe absorbing its wander). One decoupled attempt aborted 3 s into DIRECT on a
driver race (a takeoff setpoint reaching the planner after the switch, captured as a drone
target -- fixed in `pnp_mission_v2.py`, the run re-flown); it is kept as
`runs/bench_geo_5_aborted_bringup.*` and not counted.

Report page (2026-10-04, user request: performance comparison + gains and parameters):
`report.html`, published as the artifact "Pick-and-Place Controller Comparison"
(https://claude.ai/artifact/7UGSXoeBKA4Ngfgc46m6of). Rebuild with
`tools/report_data.py` then `tools/build_report.py` (every number is computed from the
runs and the two controller yamls); republish the same file path to keep the URL.
