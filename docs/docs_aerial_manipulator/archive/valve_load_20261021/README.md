# Valve load screen: a hanging weight vs a rotary damper (2026-10-21)

User question: put a drum/pulley on the valve shaft that lifts a hanging payload as the valve
turns (tune the torque with the payload mass). Is it feasible and safe, does it make the turn
more stable, and how much torque should the valve resist? Or another design?

## Setup

The shipped valve config (valve_turning_20261015 README 15.10: omega_c_t 2.0, pose [0, 5, 25, 0],
tilt guard 10 deg, open-then-exit), headless, raw mocap, the 0.048 N.m bearing, plus a load from
three NEW scene knobs in `07_px4_t650_aerial_manipulator_valve_turning.py` (all default off,
forwarded by the valve wrapper):
- `PEGASUS_VALVE_LOAD_TORQUE` = m g r_drum [N.m], resisting the turn's direction
  (`PEGASUS_VALVE_LOAD_DIR`, default +1). String just taut at the start (payload on the floor),
  full torque after `PEGASUS_VALVE_LOAD_TAKEUP_DEG` (3), slack when unwound past the start.
  A CONSTANT torque: it back-drives the valve the moment nothing holds it. Payload inertia
  (~15 % of the rotor's at 0.2 N.m on a 20 mm drum) neglected.
- `PEGASUS_VALVE_RATCHET=1` = a one-way pawl (stops backward rotation, carries the weight).
  MODEL LIMIT: it leaks after an abort (slow back-drift ~14 deg/s in ld_w020r) -- it zeroes the
  backward momentum each step but competes with the bearing's own momentum clamp. Turn-phase
  results are unaffected; do not read post-abort back rotation of a ratchet run as real.
- the rotary damper is the existing `PEGASUS_VALVE_DAMPING` [N.m.s/rad] (joint drive, implicit).

Chain: `tools/chain.sh`, scorer `tools/load_score.py` (scores in `scores.txt`). Runs in
`../valve_turning_20261015/runs/ld_*`.

## A harness trap found on the way (fixed)

`pl_mission.py`'s default exit became RELEASE-FIRST on 2026-10-19 (the box push). The valve
harness `try_valve.sh` passed nothing for `RELEASE_AT_OPEN=0`, so since 10-19 it silently flew the
release-first exit, which is the exit known to fail on the free valve handle (8.9 there). The
first baseline and the first ratchet run failed exactly there (`*_relfirst`, kept). `try_valve.sh`
now passes `--open-first-exit` by name; the corrected baseline completed as on 10-18.

## Results (one run each)

| run | design | outcome | tilt max | arm on a stop | q3 min | swing std | base lead | after the release |
|---|---|---|---|---|---|---|---|---|
| l6_ship_1/2 (10-18) | none | complete x2 | 5.5-5.9 | 13 % | 11-14 | 10.2-10.7 | 54-58 mm | beam back 4-5 deg |
| ld_base1 | none | complete | 5.2 | 8.7 % | 9.7 | 10.6 | 38 mm | 4 deg |
| **ld_d050** | **damper 0.5 N.m.s/rad** (0.22 N.m at the 25 deg/s peak) | **complete** | **5.1** | **4.1 %** | **14.2** | **7.8** | **22 mm** | **1 deg** |
| ld_w010r | weight 0.10 + ratchet | abort 355 deg | 10.9 | 10.7 % | 10.3 | 10.6 | 58 | -- |
| ld_w020r | weight 0.20 + ratchet | abort 269 deg | 11.5 | 6.5 % | 6.4 | 10.9 | 49 | -- |
| ld_w020r_relfirst | weight 0.20 + ratchet | turn complete, exit failed (harness) | 9.9 | 1.6 % | -3.0 | 9.6 | 46 | -- |
| ld_w020 | weight 0.20, NO ratchet | abort 145 deg | 10.5 | 6.6 % | 14.7 | 9.0 | 45 | **beam spun back 552 deg at up to 1060 deg/s, arm to q3 -69** |
| ld_w040r | weight 0.40 + ratchet | abort 335 deg, FLIPPED | 10.7 | 13.9 % | 1.0 | 12.2 | 58 | q3 -90 |

- **A constant load makes the turn worse at every level tried, 0.10-0.40 N.m: 5 of 5 loaded turns
  peaked at 9.9-11.5 deg of tilt, 0 of 4 unloaded/damped turns above 5.9.** Four of five tripped the
  10 deg guard mid-turn. The first 15 s track like the baseline (tilt max 3.7-3.9 vs 3.9-4.1); the
  tilt grows later. A ~1 Hz q1/q4 antiphase swing grows before each trip (ld_w020, 12-18 s: q1
  -12..+22, q4 -25..+12 vs unloaded -8..+3 / -5..+9; ld_w010r in the deceleration). The load does
  NOT shift the mid-turn posture (q3 mean 12-18 s: 39-47 loaded vs 44-46 unloaded, 33.5 in
  ld_w020); it swings deeper: two of five weighted turns took q3 below 5 deg (1.0 at 0.40, -3.0 in
  ld_w020r_relfirst), no unloaded turn below 9.7. At 0.40 the arm buckled through q3 = 0 within
  0.5 s of the abort and the vehicle flipped.
- **Without a ratchet the weight is unsafe**: when the jaws open (abort or exit) the weight spins
  the beam back into the claw (ld_w020: 552 deg at up to 1060 deg/s; 0.20 N.m on the 2.6e-3
  kg m^2 rotor is ~70 rad/s^2: the fin moves ~25 mm back within 0.07 s and >200 mm within 0.2 s).
- **The rotary damper is the only load that improved the turn** (one run): base lead 22 mm vs
  38-58, the arm on a stop 4 % vs 9-13 %, swing 7.8 deg vs 10.2-11.6, q3 margin 14 deg, tilt
  unchanged, nothing to release. The 10-18 stress case at 2.0 N.m.s/rad aborted on the force
  guard (5.6 N), so the useful window is below ~1.0; 0.5 is the one point flown.

## Recommendation (to the user)

Do not use a hanging weight as the valve's resistance (with or without a ratchet). Use a
speed-proportional, two-way rotary damper of about 0.5 N.m.s/rad at the valve shaft (0.1-0.2 N.m
at the turn's 12-25 deg/s). With an n:1 step-up gear the damper sees c/n^2 (3:1 -> 0.056
N.m.s/rad, i.e. ~0.12 N.m = 12 N.cm at 20 rpm on the damper). Check it on the bench with a
weight on a string at the fin (r 0.160 m): 0.13 kg (0.20 N.m) should pull the fin round at
~25 deg/s. NOT confirmed: one damper flight only; repeat it and bracket 0.25 / 1.0 before
building. A string-and-weight payload lift for a demo needs a ratchet AND stays untested below
0.10 N.m.
