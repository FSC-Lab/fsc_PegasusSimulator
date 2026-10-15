# 2026-10-14 pick-and-drop FREE flights (T650-AM, whole-body 4-D L1, fused) — the bin and the loose arm

Bags: `docs/experimental_data_ros2_bag/1014 - T650-AM whole-body Pick&Drop [Free-Flight]-*/` (no payload in either).

| run | bag | setup | what happened |
|---|---|---|---|
| PD-1 17:28 | `flight_wb_l1_4d_pick_and_drop_20261014_172845` | full plastic bin under the drop point | all legs to the drop hold, oscillated over the bin, ABORT (climb 0.30 m), transit to (0, 0, 1.68), land |
| PD-2 18:39 | `flight_wb_l1_4d_pick_and_drop_20261014_183914` | bin cut to a holed ~5 cm tray | DIRECT 15.1-36.2 s; reverted by the operator 9 s into the pick approach while turning; landed in SAFETY |

The `*_analysis` folder next to PD-1's bag is an earlier quick look (other session); this folder supersedes it.

## Q1 — did the box's aerodynamics cause the attitude fluctuation? Yes, for the full bin.

- The 0.3-2 Hz pitch oscillation is 0.1-0.4 deg rms everywhere in PD-1 until the front rotors come over the bin
  (86 s), 1.8-2.7 deg over it at 1.35 m, still 1.1-1.8 deg 0.30 m higher over it (after the abort), and back to
  0.16-0.25 deg at (0, 0) at the SAME 1.68 m. Height alone is not it.
- **Control case**: 10-09 PP-1, PP-2 and 10-10 PP-4 held the SAME drop point, height (1.31-1.33 m), heading (-180)
  and arm pose, over the plain hat: 0.05-0.4 deg (median ~0.2). With the bin: ~10x.
- **It is an external load, not a model artefact** (`tools/moment_residual.py`, `ROBUST=1`): the moment the airframe
  felt, I*domega/dt - tau_rotors(lagged), with the rotor gain, lag and a +-60 ms shift FITTED per window to minimise it:
  0.071 N m rms over the bin vs 0.010-0.013 at the same point over the hat (0.025 with the basket) and 0.011 in the
  other hovers; 0.030 at 1.65 m over the bin. Vertical aerodynamic force (accelerometer m*a_z - rotor thrust, also
  fitted): 0.26 N vs 0.05-0.13 N; unfitted 3-8 Hz band 0.22 vs 0.06-0.10 N (broadband turbulence, not just the mode).
- **No steady ground effect**: mean u1 38.78 vs 38.79 N and the observer's vertical estimate -2.02 vs -2.02 N at the
  same point on 10-09. Only the fluctuation grew.
- **Not the mocap** (IR reflections off the bin): raw VRPN jitter >8 Hz 0.83 mm over the bin vs 0.75-1.04 mm
  elsewhere in the flight.
- The oscillation frequency, 0.59 Hz, is the law's own position-loop mode (k_x 50.03, k_v 12.58, m 3.746 kg:
  0.58 Hz, zeta 0.46 nominal, less with the attitude lag): the bin drives a mode the vehicle already has.
- **The cut-down tray is untested**: PD-2 never reached the drop point. The 10-09/10-10 flat hat at the same spot
  did nothing, so a flat holed tray is expected to be benign, but hover over it at the drop altitude for 10-15 s
  before the next full mission and check that pitch stays near its ~0.3 deg.

## Q2 — the loose front-left arm: how it shows up and since when

- **Signature = a standing clockwise yaw torque** the rotors hold off (`tools/yaw_history.py`, one fixed rotor
  model for every flight: ESC map, kf 4.260431e-05, km 0.018164; equals the law's commanded tau_flu z to 1e-3).
  PD-2: CW pair (FL, RR) at 0.75-0.77 command vs CCW pair (FR, RL) at 0.40 — 0.33 N m in the allocator's units,
  ~55 % of the ~0.65 N m yaw budget at hover. Mid-September: 0.65 vs 0.56, 0.06-0.10 N m.
- **History** (lever = tau_z / T, mm; 1 mm = 0.037 N m):
  09-18 2.6 · 09-21 1.6, 1.7 · 09-24 2.2 (F1) then 3.6, 2.7, 3.1 after F1's fly-away crash · 09-28..10-09 3.2-4.6 ·
  **10-10 5.2** (first flight after 10-09 PP-3 tipped 57 deg) · **10-14 PD-1 6.5** (after 10-10's 3.8 m/s impact) ·
  **PD-2 8.9**. Steady within each flight (+-0.5 mm; PD-1 crept 6.2 -> 7.1).
  So: first clearly outside the earlier band on 10-10, worse each flight since, the biggest step between today's two.
  A smaller step on 09-24 (2.2 -> 3.6) may be the first loosening; the band then held for two weeks.
- Size: the extra ~0.20 N m (model units; real yaw effectiveness 0.43-0.78 of the model per the 09-26 fit) is the
  FL rotor's thrust tilted ~1-2.5 deg sideways (tau = f_FL * 0.325 m * sin delta). A CW torque needs the FL motor to
  lean toward the nose/right (the direction a CW rotation moves that corner).
- **Vibration does not show it** (`tools/vibration.py`): 15-95 Hz accelerometer 1.9-2.9 m/s^2, gyro 2.2-3.5 dps on
  every flight, PD-2 2.20 / 2.49. The horizontal force estimate did shift on 10-10 (~1.8 N in body axes) but mostly
  along the FL arm axis, not the direction a twist gives, so it is not used as evidence.
- **PD-2's "undesired yaw" was mostly commanded**: the pick point's yaw was 179.6 deg (from a Get before the bag;
  PD-1 had 3.4), so `pick_place_align_yaw` turned the vehicle a half turn; it tracked to < 0.6 deg (18 deg/s, 110 deg
  when reverted). The DIRECTION is a planner rule: `legHeading` keeps a turn under 210 deg as given, so +179.6 turns
  counter-clockwise and the same basket read as -180.4 turns clockwise. The uncommanded yaw, all clockwise = the arm
  bias: -6 deg in the SAFETY takeoff, -8.4 deg at DIRECT entry (observer learning it), -13 deg at touchdown.
- **Where the 179.6 came from (checked 2026-10-15):** `pick_place/info` [80] = 179.6483766 deg, full precision, so
  a Get of `/obj_0/mocap`, not a typed value (PD-1: x typed 1.01, yaw Get 3.3713; every Get on 10-09/10-10 read
  0.6-1.3 deg). The pick-leg goal heading = that yaw (align_yaw); the reference left 0 at 27 s and reached 110 deg
  at the revert, measured within 0.4 deg. The Get was made before the recording and the basket lay on the floor
  in both bags, so the bags cannot say whether the basket was turned round or Motive read it backwards. Two
  facts bear on it: the capture also moved 0.21 m from PD-1's ((-0.15, -0.14), +7 mm) while the five markers'
  centroid is ~6 mm from the basket centre, so the basket was ALSO somewhere else (or the pivot / rigid body
  changed); and four of the five balls map onto each other under a 180 deg turn (only the holder ball, 32 mm
  forward and 78 mm up, tells the ends apart), so a hidden holder ball can give a 180 deg reading.

## Side finding

The raw mocap stream changed between 10-10 and 10-14: `/vrpn_mocap/uav_0/pose` ~245 Hz with a third of the frames
repeated and 4-6x more >8 Hz jitter (0.75-1.04 mm vs 0.15-0.25 mm at 124 Hz on 10-09/10-10). Same everywhere in
today's flights, so not the bin; the law flies EKF2-fused feedback, which filters it.

## Reproduce

```bash
# from the repo root
A=docs/docs_aerial_manipulator/archive/wb_pick_drop_flight_20261014
B="docs/experimental_data_ros2_bag/1014 - T650-AM whole-body Pick&Drop [Free-Flight]-20261015T003404Z-1-001/1014 - T650-AM whole-body Pick&Drop [Free-Flight]"
T09=docs/docs_aerial_manipulator/archive/wb_pick_place_flight_20261009/tools
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
/usr/bin/python3 $T09/extract_bag.py "$B/flight_wb_l1_4d_pick_and_drop_20261014_172845" $A/npz/d1.npz && /usr/bin/python3 $T09/np1_compat.py $A/npz/d1.npz
#   (same for 183914 -> d2; history bags: $A/tools/extract_subset.py <bag dir> $A/npz/hist/<bag>.npz + np1_compat.py)
cd $A/tools
PYTHONNOUSERSITE=1 /usr/bin/python3 yaw_history.py            # -> analysis/yaw_history.json
PYTHONNOUSERSITE=1 /usr/bin/python3 moment_residual.py        # -> analysis/moment_residual.json
ROBUST=1 PYTHONNOUSERSITE=1 /usr/bin/python3 moment_residual.py  # -> analysis/moment_residual_robust.json
PYTHONNOUSERSITE=1 /usr/bin/python3 vibration.py              # -> analysis/vibration.json
PYTHONNOUSERSITE=1 /usr/bin/python3 yaw_events.py             # -> analysis/yaw_events.json (spool-up/touchdown drift: too noisy to use)
PYTHONNOUSERSITE=1 /usr/bin/python3 fig_summary.py            # -> figures/pick_drop_1014_summary.png
```
Timelines: `analysis/timeline_d{1,2}.txt` (10-09 `timeline.py` with `AM_NPZ=../npz`).

## Q3 — the pilot's STAB takeover "killed the props" and he could not re-arm: OUR NODE, FIXED

**Cause.** In whole-body DIRECT the flight node (`single_aerial_manipulator_whole_body_direct_actuation_client.cpp`)
called `publishMotorsStopped()` -- an all-NaN `/fmu/in/actuator_motors` -- every tick in which PX4 was not armed in
OFFBOARD (`!inFlight()`). A pilot switch to STAB left the node in DIRECT, so it kept streaming that at 250 Hz until the
node was stopped. PX4 v1.16 bridges `/fmu/in/actuator_motors` into the SAME uORB topic its control allocator
publishes in STAB; `FunctionMotors::update()` takes the latest message and `MixingOutput::output_limit_calc_single()`
turns a NaN into the disarmed value -- a motor STOP. The pilot's throttle was interleaved with stop commands.

**Hardware evidence** (`tools/takeover.py`, `touchdown.py`, `takeover_table.py` -> `analysis/takeover_table.json`):
every disarm since 09-28 was `RC switch` (the pilot); all 6 takeovers from whole-body DIRECT streamed 782-10 142
NaN messages after the switch, 3 of the 6 tipped over (10-09 PP-1 46 deg, PP-3 57 deg, 10-10 PP-4 36 deg); 10
takeovers from SAFETY had no stream, 1 tipped (today's PD-2: climbed after the switch, dropped at 1.6 m/s, touched
down at 14 deg). The decoupled (geometric+L1) node has the same branch; its hardware landings were all taken over
from SAFETY.

**Fix (fsc_autopilot_ros2, whole-body + AM geometric+L1 clients):** (1) `publishMotorsStopped()` reaches PX4 only
while PX4 is in OFFBOARD; (2) new `px4TookOver()`: in DIRECT, any non-OFFBOARD PX4 mode (pilot or PX4 failsafe)
switches the node to SAFETY on the same tick -- DIRECT is only ever entered armed in OFFBOARD, so this cannot break
a workflow; (3) after a takeover the next OFFBOARD entry re-seeds the SAFETY reference to the current pose
(`reseed_on_offboard_`), so a hand-back holds where the vehicle is (run 1's pilot had drifted 11 m) while the normal
takeoff keeps the GS target sent before OFFBOARD. Not touched: the sim-only modular and coordinated-admittance
clients and the bare-drone clients, which have the same pattern.

**Simulator check** (`tools/run_takeover.sh wb|geo <tag>` + `takeover_test.py`: a scripted pilot over
`/fmu/in/manual_control_input` -- DIRECT hover, PX4 STABILIZED, hold height and position with the sticks, land,
disarm, re-arm, climb, hand back to OFFBOARD; `takeover_fig_data.py` -> `analysis/takeover_sim.json`):

| | old build (wb_old_1) | fixed (wb_fixed_4) |
|---|---|---|
| node mode after PX4's switch | still DIRECT | SAFETY, same tick |
| NaN stop commands to PX4 after the switch | 10 923 | 0 |
| STAB height hold, 8 s, from 0.98 m | fell to the ground (0.13 m, tipped) | lowest 0.90 m, std 20 mm |
| re-arm in STAB | refused ("attitude failure", tipped) | armed, climbed to 0.85 m |
| hand back to OFFBOARD | node still DIRECT | SAFETY, 4 mm drift in 8 s |

Runs 1-3 were scripting iterations (tick-counted ramps, no hover trim, no position hold; run 3 tripped the
known RTF-1 DIRECT hover instability before the takeover). Two pilot notes from them: in STAB this vehicle needs the
throttle ~0.2 above centre to hover (PX4's hover throttle is low for the arm's weight); after a tip-over PX4 refuses to
arm until upright. The hand-back dips ~0.3 m before recovering: the SAFETY re-entry dip every DIRECT->SAFETY revert
shows (19-23 cm on hardware), moved by the fix from the takeover to the hand-back.
