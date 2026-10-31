# The roll-fin valve handle in Isaac: the 90 deg turn, free vs locked vs the fixed fin (2026-10-31)

User request: fly the same 90 deg valve-turning task with the new handle (`rotorcraft/assets/Valve_Assembly_v3.stl`,
the roll fin of `archive/valve_handle_joint_20261030`), and judge from the simulation and the analysis whether the
design helps complete the task and improves the performance. The design answers the 10-29 analysis' item 8
("a handle the closed jaws can turn on in roll ... so the grip stops passing a roll couple into the arm and the
base"; `archive/wb_valve_turning_flight_20261029/README.md`).

## Verdict

- **The task flies with the roll fin.** Nominal: 4 of 4 complete (incl. the smoke flight), beam 89-90 deg,
  valve-minus-plan 3.4 deg rms (v2 3.4), peak tilt 4.1 deg (v2 5.3). The fin stayed exactly upright until the grasp
  (0.0 deg) in every run, only the two finger links ever touched it, and the joint held to < 0.1 mm.
- **It removes what it was designed to remove.** The couple the grip passes about the roll axis falls from 122-193
  mN.m rms (the fixed v2 fin, or the same handle locked) to **1-2 mN.m** (the O-ring). The couples about the jaw
  axis and the third axis are unchanged, as designed.
- **Isaac cannot show whether that fixes the hardware failure**: no handle ever produced the hardware's growing
  0.45-0.6 Hz sway here. Under the hardware-like drift, completion is not distinguishable: per drift seed 2-8,
  **v2 5/7, free 4/7, locked 2/7** (Fisher: free vs locked p = 0.59, v2 vs free p = 1.0). Every failure, with any
  handle, is the same mode: q3 folding toward its elbow-singular side mid-turn (4-19 s into the push step), the q3
  guard, and then DIRECT lost in 8 of 9 aborts. The roll fin neither causes nor removes that mode.
- **The price, seen in Isaac:** the base wanders more sideways in contact (16.8 / 19.0 mm rms nominal / drift vs v2
  10.4 / 16.6) and the wrist moves about twice as much (q4 6.1 / 7.1 deg std vs 2.5 / 4.1): the arm now takes up the
  sideways motion the stiff fin used to resist. The fin stays wherever the last exit left it (once flicked to 34 deg
  by the open fingers).
- **On hardware (offline models, section 4):** removing the roll couple returns the sideways mode's damping to free
  flight in the 10-28 reduced model (-1.92 vs -0.32..-0.63 1/s with the fixed fin). With the wrist swung 20/26 deg,
  half the forced grip rotation comes back about the jaw axis, which no roll pin frees; even then it is -0.8..-1.2 1/s.
  So the design should stop the growing sway that limited the 10-28/29 flights to 5-8 s of clamped contact. Two
  caveats: that model never reproduced the flown growth, it only ranks options; and the mid-turn arm fold is
  untouched. Confirm with the design's own A/B on one battery: the screw tight vs free, first a 10 s clamped hold (D1),
  then the turn.

## 1. Results

Run lists, per-run lines: `compare.txt`; per seed: `outcomes.txt`. Means over the turn (or up to the abort):

| handle | condition | completed | couple at G about X / N / C (mN.m rms) | sideways CoM rms (mm) | q4 std (deg) | valve - plan rms (deg) | roll fin in the turn |
|---|---|---|---|---|---|---|---|
| v2 fixed fin | nominal | 4/4 | 79 / 156 / 62 | 10.4 | 2.5 | 3.4 | -- |
| v2 fixed fin | drift | 7/9 (5/7 seeds) | 65 / 191 / 58 | 16.6 | 4.1 | 5.8 | -- |
| v3 LOCKED | nominal | 3/3 | 63 / 122 / 63 | 15.0 | 5.1 | 4.0 | +-0.2 deg |
| v3 LOCKED | drift | 2/7 | 105 / 193 / 88 | 16.7 | 5.1 | 7.7 | +-0.2 deg |
| v3 FREE (O-ring 2 mN.m) | nominal | 3/3 (+ smoke 1/1) | 53 / **1** / 60 | 16.8 | 6.1 | 3.4 | -4.5..+9.4 deg |
| v3 FREE | drift | 4/7 | 69 / **2** / 73 | 19.0 | 7.1 | 7.1 | -5.3..+11.8 deg |

(X = the radial jaw-closing axis, N = the roll axis, C = the claw axis in the fin's plane. Free-flight hover
reference before Go To Start: sideways 2.5-2.8 mm rms nominal, 10.4-11.8 with drift, the same for all handles.)

Per drift seed (`outcomes.txt`; v2 has the 10-31 control runs of seeds 2-4 too):

| seed | v2 | free | locked |
|---|---|---|---|
| 2 | ok, ok | ok | q3 guard 18.8 s, DIRECT lost |
| 3 | ok, ok | q3 guard 8.5 s, DIRECT lost | q3 guard 5.9 s, DIRECT lost |
| 4 | q3 guard 14.3 s, DIRECT kept | ok | q3 guard 5.0 s, DIRECT lost |
| 5 | q3 guard 3.9 s, DIRECT lost | q3 guard 8.2 s, DIRECT lost | ok |
| 6 | ok | ok | q3 guard 11.6 s, DIRECT lost |
| 7 | ok | q3 guard 8.5 s, DIRECT lost | tilt guard 4.7 s, DIRECT lost |
| 8 | ok | ok | ok |

## 2. What was checked and NOT found (do not re-derive)

- **Every failure, on every handle, has the base off its reference while the clamped arm runs out of range.**
  Example, free seed 3: 30-40 mm behind along the nose (+40 mrad pitch error x T / k_x), the valve barely
  turning, q3 unfolding 27 -> 4.5 deg, then the fold at 96 deg/s. v2's seed-5 failure looks the same.
- **v3 changes the standing pitch error in the turn, free and locked alike.** v2's is always negative
  (-4..-58 mrad, the base leading the claw). v3's swings in sign (locked -45..+49, free -20..+29). Not explained:
  - the vertical grip mismatch: corr 0.47 with the pitch error, the vertical force 0.04;
  - the valve turned by a couple instead of a push: true in one locked run, not in the group means;
  - contact chatter on the 23 g roll-fin body: v3's 2-30 Hz contact force 0.16-0.30 N vs v2 0.26-0.33;
  - the sim fingertips missing the shorter plate: the finger colliders put the grip at 0..+16 mm along the claw
    axis, inside both fins. A shim-raise diagnostic was queued and dropped for that reason.
- The along-nose offset does not predict the lowest q3 (corr 0.07): the folds are fast events (33-119 deg/s), not
  slow drifts.

## 3. What was built

- **`tools/stl_to_usda_valve_v3.py` -> `rotorcraft/assets/Valve_Assembly_v3.usda`.** The v1/v2 converter
  (`valve_turning_20261015/tools/stl_to_usda_valve.py`) is unchanged and cannot read v3: two marker mounts merged
  into the rotor shell on that export (its volume is up by exactly their 6.6 cm^3) and the fin top is gone. The new
  tool imports its helpers and classifies the 20 bodies against the design's own geometry (`roll_grip_design.py`:
  G = (160, 0.6, 179) mm, the roll axis N = (0, cos 30, sin 30), the fin's (s, c) outline), asserting every count.
  - `/rotor` (dynamic): the cut rotor shell + the printed handle base + the M4 + the M3 bolts / nuts + the O-ring;
    colliders beam, root (v2's boxes), the handle base (convexDecomposition), the M4 (convexHull), the five marker
    balls (spheres).
  - `/roll_fin` (dynamic): the roll fin, convexHull collider with the claw's grip material.
  - `/roll_joint`: a RevoluteJoint rotor -> roll fin about N through G (joint Y turned onto N by Rx(30 deg));
    collisions between the pair off; an angular DriveAPI the scene sets.
  - Masses from the design's densities (printed PLA 1040 kg/m^3, steel 7850): rotor body 226.2 g, roll fin 23.2 g
    (CoM 4.2 mm below the roll axis, 5.9e-6 kg m^2 about it). About the valve axis 3.57e-3 kg m^2 vs v2's 2.6e-3
    (+37 %, the design's +35 %). The markers' centroid is v2's (93.5, 1.4, 89.9) mm, so the planner's
    `push_pull_valve_axis_offset` and `push_pull_ee_offset` are unchanged: no yaml changed.
- **The scene** (`07_px4_t650_aerial_manipulator_valve_turning.py`), everything gated on the asset's `fsc:roll_fin`,
  so v1 / v2 runs are unchanged:
  - the 20 mm grip shim (the sim claw cannot close below ~19 mm) sits on the ROLL FIN, over its whole outline, and
    rolls with it;
  - `PEGASUS_VALVE_ROLL_FRICTION` (default 0.002 N.m, the design's 2-4 mN.m O-ring hold) as stick-slip Coulomb
    friction in the joint drive (the bearing's `drive` model); `PEGASUS_VALVE_ROLL_LOCK=1` = the screw tightened (a
    1e4 N.m/rad drive): the design's own locked / free A/B;
  - the rotor's mass defaults to the asset's (226 g; v1/v2 keep 0.204 kg);
  - the contact truth sums the drone's contacts on both valve bodies and carries the roll angle, its rate and the
    joint's error ([20..22]); a `ROLL FIN at ...` line every 2 deg, a `FIRST CONTACT <link> -> valve <body>` line
    per new pair;
  - the valve wrapper forwards the two new knobs.
  - Bug found and fixed on the smoke flight: rclpy makes `Float64MultiArray.data` an `array('d')` on assignment, so
    `+=` a list raises -- inside the scene's try/except, silently: that flight has no contact record.
- **Harness**: `tools/chain.sh` / `chain2.sh` = the 10-31 v2 runs' (`valve_safety_fixes_20261030/sim/chain.sh`):
  the v2 bearing (0.072 N.m, drive model), the shipped valve sim yaml, raw mocap, headless, the arm GS flying
  Ready / Turn / Exit, each flight stopped 4 s after the exit; drift = `PEGASUS_PLANT_FORCE_NOISE 0.32,0.12,<seed>`
  (the 10-22 hardware-like model). `tools/pane_keeper.sh` saves the Isaac pane per run (`runs/<tag>.isaac.log`).
- **Scorers**: `tools/grip_score.py` (per run: the 10-22 campaign row + the couple the grip passes at G on the fin's
  axes X radial / N roll axis / C, the sway, e_R, q1/q4, the roll fin, how the valve was turned, the exit's force),
  `tools/compare.py` (grouped, writes `compare.txt`), `tools/predict_hw.py` (the hardware prediction, writes
  `predict_hw.txt`).

## 4. The hardware prediction (Isaac cannot show it)

Isaac never reproduced the hardware's growing 0.45-0.6 Hz contact sway (10-28 section 5, 10-29 section 4): in
every sim run, with any handle, the sideways wander stays a slow 0.1-0.16 Hz drift. So the part of the design aimed
at that mode can only be judged with the 10-29 offline models (`tools/predict_hw.py`, `predict_hw.txt`):

- **Which grip rotation the vehicle forces, and what the 30 deg pin leaves** (the 10-29 mobility model, the pin
  FIXED at 30 deg in the valve frame as printed):

  | arm pose | sideways 10 mm: forced -> left | roll 1 deg: forced -> left |
  |---|---|---|
  | flown grasp [0, 5, 25, 0] | 1.34 -> 0.01 deg (1 %) | 0.27 -> 0.01 deg (2 %) |
  | fold [0, 10, 20, 0] | 1.28 -> 0.01 deg (1 %) | 0.30 -> 0.01 deg (2 %) |
  | fold [0, -10, 45, 0] | 1.79 -> 0.16 deg (9 %) | 0.05 -> 0.01 deg (18 %) |
  | wrist swung [+-10, 5, 25, -+13] | 1.33 -> 0.36-0.37 deg (27-28 %) | 0.26 -> 0.01-0.02 deg (4-9 %) |
  | wrist swung [20, 5, 25, -26] | 1.30 -> 0.69 deg (53 %) | 0.26 -> 0.03 deg (11 %) |

  What is left is a rotation about the radial (jaw-closing) axis, which no roll pin frees -- on the real gripper
  that is the pads spinning in their own plane, resisted only by pad friction.
- **The sideways mode** (the 10-28 reduced model; it ranks options and never reproduced the flown growth): decay
  rate, more negative = better damped:

  | grip roll stiffness k_s (calibrated to the flown mode: 3-7 N.m/rad) | fixed fin | roll fin, flown pose | roll fin, wrist swung 20/26 deg |
  |---|---|---|---|
  | 3 N.m/rad | -0.63 1/s | -1.92 | -1.17 |
  | 5 N.m/rad | -0.43 | -1.92 | -0.96 |
  | 7 N.m/rad | -0.32 | -1.92 | -0.81 |

  Free flight is -1.92 1/s. The O-ring caps the couple about the pin at 2-4 mN.m (the fixed fin passed ~70 mN.m rms
  on 10-29), gravity adds 0.8 mN.m/rad.

## 5. For the hardware test

- Fly the A/B the design was built for, on one battery, sway guard on: the screw tight (locked) vs free. First the
  D1 10 s clamped hold, no turn, then the turn. Prediction: free shows no growing 0.45-0.6 Hz sideways sway.
- Set the fin upright before each grasp. The O-ring holds it wherever the last exit left it (in sim, up to 34 deg;
  the jaws take ~30 deg).
- Isaac has no downwash. Check, in a hover over the handle, that the front rotors' wash does not roll the fin
  (estimate 0.6-1.5 mN.m against the 2-4 mN.m hold).
- Keep the q3 static + rate guards and the sway guard. The mid-turn fold is unchanged and is the failure every handle
  shares in Isaac.
- No planner, law or yaml change is needed: G, push_pull_ee_offset and push_pull_valve_axis_offset are the v2 values.
