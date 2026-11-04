# Does the sidewall add wind mid-field? Matched free flights, 10-28 (no wall) vs 11-02 (wall up)

User question (2026-11-03): the 11-02 free-flight bags were flown mid-field with the sidewall up, the setup of the
11-03 valve test. Does the wall cause more wind disturbance? Follows `../wb_hover_wander_history_20261103/`.

## The bags

`docs/experimental_data_ros2_bag/1102 - T650-AM Final Movies/`: five flights.
- The Drive download split three of them across its two zip parts: the whole-body figure-8 (183342) and both decoupled
  flights (184129, 184743) have their `.db3` in the other part from their `metadata.yaml`.
- They are paired by symlinks under `bags/`; the user's folder is unchanged.

**UPDATE (same day, all four trajectory pairs): see "All four pairs" below -- it supersedes the reading of the circle
alone.**

Matched pairs, same plan, same gains, same start point (about (-0.2, -0.2) m), battery 24.1-24.3 V on both days:
- The circle: the planner printed the identical plan (lap 24.17 s, peaks |v| 0.17, tau_j 0.84). 10-28 flew it twice in
  each of two bags (144602, 150821: four one-lap runs); 11-02 flew one two-lap run, split at its middle here.
- The free-flight hardware yaml is unchanged since 10-19.
- The circles ran on the fused stack on both days, so the odometry is EKF2 both days. The mocap was equally clean:
  jitter above 8 Hz 0.18-0.39 mm (11-02) vs 0.13-0.31 mm (10-28).

## Results

Tracking, scored with the paper's own scorer (`results/utils/experiment_tracking.py` -> the 09-28 metrics;
`tools/track_score.py`, cache in `cache/`, nothing written under results/):

| lap | 10-28, no wall | 11-02, wall up |
|---|---|---|
| first | 26.9, 31.1 mm | 41.0 mm |
| second | 45.4, 39.6 mm | 49.1 mm |
| mean | 35.8 mm | 45.1 mm (+26 %) |
| roll error rms | 0.55-0.65 deg | 0.74-0.77 deg |

The disturbance the vehicle actually feels, per lap (`tools/ab_laps.py`; every signal filtered over the whole flight,
then windowed):

| per lap | 10-28 (4 laps) | 11-02 (2 laps) |
|---|---|---|
| the law's slow moment estimate, 0.05-1 Hz | 21-52 mN.m | 27-37 mN.m |
| the law's slow force estimate, 0.1-1 Hz | 0.04-0.06 N | 0.06-0.07 N |
| the IMU's horizontal force (m x accelerometer x/y), 0.1-1 Hz | 0.35-0.68 N | 0.34-0.58 N |

- **Holds** (`tools/ab_holds.py`): mixed, one hold each.
  - The start hover was CALMER on 11-02: wander 14 vs 27/29 mm.
  - The rest after the circle was worse on 11-02: 29 vs 16/20 mm.
- **Pick-and-drop**: the hold above the pick pillar was about the same on both days (moment estimate 122 vs 117-153
  mN.m; wander 24 vs 9-27 mm). The pillar hold sat ~0.3 m nearer the table side on 11-02.

## Reading

**Mid-field, the data do not show more wind with the wall.**
- The forces and moments the vehicle feels (the law's estimates and the IMU) are inside the 10-28 range.
- The mocap is as clean as on 10-28.
- The circle's tracking error is 26 % higher, and the roll error is higher, but one flight against two is inside the
  run-to-run spread: 10-28's own laps span 27-45 mm.
- This does not prove there is no effect: two 11-02 laps cannot resolve a 20-30 % change. The figure-8 pair (10-28
  152202 vs 11-02 183342) would double the sample once its `.db3` is downloaded.

**Contrast with the table site** (`../wb_hover_wander_history_20261103/`): there the wander after 10-29 is 2-4x the
box days', and the law's moment estimate above the handle 4x. If the wall matters, its effect is local to the table, not
field-wide.

## Side finding

11-03's drone mocap was 2-3x noisier than every other day: jitter 0.58-0.79 mm, both mid-field and at the table (10-28
to 11-02: 0.13-0.39). That raised the raw-stack velocity noise to 48-57 mm/s. 11-02, with the wall up, was clean, so
this is not the wall. Possible causes: the cameras, the calibration, or the drone's markers that day.

## Tools

`tools/run_extract.sh` (11-02 bags -> npz/), `track_score.py` (extract | score), `ab.py` (flight table), `ab_holds.py`,
`ab_laps.py`, `mocap_noise.py`. Run with `PYTHONNOUSERSITE=1 /usr/bin/python3` (`track_score.py extract` with the
user-site numpy).

## All four pairs (whole-body / decoupled x circle / figure-8; 20 laps)

Same plans, same gains (whole-body yaml unchanged since 10-19, geometric+L1 since 10-05). 10-28 flew two one-lap runs per
bag; 11-02 one two-lap run, split at its middle. Scored by the paper's scorer (`tools/track_score.py`); body-axis split
in `tools/body_axes.py`.

| pair | roll error 10-28 | roll error 11-02 | EE 10-28 mean | EE 11-02 mean | gyro 0.5-5 Hz roll, 10-28 / 11-02 |
|---|---|---|---|---|---|
| WB circle | 0.55-0.65 deg | 0.74-0.77 | 35.8 mm | 45.1 (+26 %) | 1.27-1.70 / 2.05-2.37 deg/s |
| WB figure-8 | 0.62-0.63 | 0.93-0.94 | 36.2 | 52.7 (+46 %) | 1.52-1.55 / 2.41-2.44 |
| DEC circle | 0.60-0.66 | 0.96-1.22 | 90.1 | 94.1 (+4 %) | 1.26-1.33 / 2.84-4.27 |
| DEC figure-8 | 0.79 | 1.20-1.80 | 85.2 | 97.0 (+14 %) | 1.39-1.70 / 3.48-4.76 |

- **Every 11-02 lap tracked worse than every 10-28 lap** on roll error (8 against 12, no overlap). So something did
  change.
- **The extra shaking is not turbulence-like** (`tools/shake_spectrum.py`):
  - it sits in the 0.3-1 Hz band, at the vehicles' own slow modes: ~0.55 Hz, the decoupled tune's known zeta ~0.1 sway
    and the whole-body CoM-loop mode;
  - that band is 2-3.5x higher, while 3-10 Hz is unchanged;
  - the IMU's horizontal force is unchanged (0.26-0.76 N both days).
- **The vehicle's standing trim stepped, body-fixed** (`tools/trim_history.py`, every flight 10-20 .. 11-03, home-pose
  holds). Roll differential (left minus right motors):

  | flights | wall | roll trim |
  |---|---|---|
  | 10-20 .. 10-29 | no | -0.016 .. +0.009 |
  | 10-31 (wall up that day, before the valve flights) | yes | -0.010 .. +0.009 |
  | 11-01 | YES | -0.013 .. +0.005, unchanged |
  | 11-02, 11-03 | yes | +0.019 .. +0.057 |

  - It changed between 11-01 and 11-02, i.e. right after the 11-01 valve crash (175932, inverted 4.4 m away). It did
    NOT change when the wall went up (10-31, final date from the user).
  - It is mostly the rear pair: rear-left about +0.011, rear-right about -0.008.
  - On a circle the body turns a full 360 deg per lap, so a field-fixed wind would average out in body axes. A constant
    body-fixed trim is the vehicle.
- **The clean wall test, same vehicle** (`tools/gts_shake.py`): every valve flight flies the same Go To Start transit,
  start mark -> above the handle. Its 0.3-1 Hz shaking:

  | flights | 0.3-1 Hz shaking roll / pitch |
  |---|---|
  | before the wall (10-28/29) | 0.75-1.40 / 1.22-2.80 deg/s |
  | wall up, before the crash (10-31, 11-01) | 0.75-2.15 / 0.82-2.25 |
  | after the crash (11-03) | 1.03-1.32 / 1.09-1.28 |

  No step with the wall.

**Reading.**
- **Mid-field / transit**: the wall does not show. Go To Start shook the same before and after it, and the start-mark
  hovers are unchanged.
- **The 11-02 degradation coincides with the vehicle change after the 11-01 crash** (a new body-fixed trim, persisting on
  11-03), not with the wall, which was already up for the 10-31 and 11-01 flights (final date 10-31).
- Not separable: 11-02 was also a filming session, with cameras moving around the vehicle. The only post-crash
  trajectory laps are 11-02's.
- **At the table**, the earlier finding stands and predates the crash: the wall and the table's new spot are inseparable there
  (see `../wb_hover_wander_history_20261103/README.md`, CORRECTION).
- Recommended: inspect the vehicle after the 11-01 crash -- the rear arms, motors, props and mounts; thrust per command
  on the rear pair. Then the no-contact hover A/B at the valve site, with and without the wall.
- Also on every flight since 10-20: the CW pair (FL, RR) runs +0.37 .. +0.52 above the CCW pair, the 10-14 loose-arm yaw
  lever.
