# 2026-11-05 hardware box pushing on the `box-pushing-20261027` branch: not normal

Bags: `docs/experimental_data_ros2_bag/1105 - T650-AM Mail-Box, Robust Tracking, Box-Pushing-.../Box-Pushing [Trial -
Box-Pushing-1027-Branch]/`:

- `..._211154`: has the `.db3`, but no `metadata.yaml`. The tools read the `.db3` directly.
- `..._212207`: complete.
- `..._214032`: `metadata.yaml` only (105.7 s, 21:40); the `.db3` is not in the download.

The comparison is the two 10-27 box flights (`../wb_box_pushing_flight_20261027/`) and, for mid-field holds, the WB
circle flights of 10-27 (175015, 180659) and 11-05 (181041, 181833, 50 g).

## 1. The code and config are the 10-27 ones

- The debug widths are 10-27's:
  - `wb_control_debug` 118 (later code: 150);
  - `arrival_error` 30 (32);
  - `push_pull/info` 89;
  - `law_debug` 32;
  - `mocap_guard` 16.
- Every log line exists in the `box-pushing-20261027` source. No later-only message appears (`TWO SETS`, `GRIP:`, the arm
  RELEASE, "FAST hand-back", the emergency abort).
- The SAFETY thrust map fits exactly 0.037947 / 0.252449 (residual 2e-8), the `a316625` box yaml.

## 2. What happened

| | 211154 (21:11) | 212207 (21:22) |
|---|---|---|
| Go To Start done | claw 53 mm from the point (10-27: 18 / 35) | 49 mm |
| Ready | refused 72 s: claw 56-73 mm off or moving 38-78 mm/s; automatic Ready gave up, ~13 presses by hand | after 4 s |
| descent | claw **53-58 mm across the fin** for 1.5 s, CoM 70-86 mm off its reference | 24-39 mm across, CoM 17-53 mm |
| end of Ready | claw 37 mm off, **112 mm/s** (10-27: 10-21 mm, 14-23 mm/s) | 29 mm, 100 mm/s |
| ended by | an open finger swept the fin; box slid 150 mm along / 83 mm across in 1.0 s and fell off the table's far edge; box-tilt guard at 79 deg (the box falling) | **false box-tilt trip** 2.5 deg: the box moved <= 2 mm; its tilt reading jumped 0.5 <-> 2.1 deg with the arm above it; jaws never closed (claw held 12-14 mm across, the gate needs 8 mm) |
| after | abort climbed cleanly, landed normally | abort, then a second ABORT by hand (to 1.78 m), flown home on the drone GS, landed |

## 3. Why: the hover at the box site, not the code

Holds with a still reference (the 11-03 `site_control.py` definitions; this implementation reproduces its 10-27 numbers):

| holds | base speed median | CoM wander median | law's moment estimate |
|---|---|---|---|
| 10-27, above the handle / after an abort | 16 mm/s | 8.7 mm | 23 mN.m |
| **11-05, same holds** | **39 mm/s** | **39 mm** | **55 mN.m** |
| 10-27 mid-field (box flights) / tracking | 23 / -- | 22 / 16-20 | 26 |
| 11-05 mid-field (box flights) / tracking | 34 / -- | 24 (long holds 39, 60) / 30 | 29 |

Per phase:
- **Go To Start:** CoM rms 35 / 29 mm (10-27: 18 / 21).
- **Ready descent:** 67 / 35 mm (26 / 16).
- **Hand-over:** 70 / 21 mm (13 / 19).

What the comparison shows:

- **Not the vehicle's trim.** The standing motor trim is unchanged from 10-27: L-R ~0, F-B +0.02, CW-CCW +0.18..0.26 (the
  known loose-arm yaw bias). The 11-03 study's L-R step after the 11-01 crash is not present.
- **Same signature as the 11-03 study.** It is the "new spot + wall" group there (30-32 mm, 34-45 mm/s): today's box
  site is the new spot, table x 0.42-0.58 (10-27: 0.70-0.85).
- **The drone's mocap jitter is site-dependent today.** Above about 10 Hz it is 1.3-2.0 mm within 0.6 m of the box,
  against 0.6-0.7 mm far from it. On 10-27 it was 0.5-0.75 mm at the box. Today's tracking flights read 0.3 mm. There
  were no VRPN stalls today.
- **Not the drone body.** The drone body's offset to PX4's attitude varies between flights (-1.9 / -1.0 / +1.7 deg
  pitch). Its ground height 0.24-0.25 m matches the 10-20 measurement.

## 4. The box's mocap body

- **Tilt noise with the arm above it.** In the hold, 9.2 % of the 212207 samples read above the 2 deg guard (10-27:
  0 / 2.3 %; 10-27 B had already come within 0.09 deg in the slide).
- **Flips.** The planner's gate dropped 178 deg flips at 95-101 s, with the vehicle 2 m away.
- **Height.** At rest the box reads 0.859-0.862 m, against 0.846 m on 10-27. Either the table top differs at the new
  spot, or the pivot moved. Check that the pivot is the box centre (the 10-24 lesson).

## 5. Suggested before the next push

1. Find what changed at the table site, as measured from the ground.
   - What was near the table today: the wall, the fan from the wind tests, other objects.
   - Hover-test above the handle without Ready, and aim for about 10-15 mm of wander and at most 25 mm/s. Or move the
     table back to the 10-27 spot (x ~0.75).
2. The box body: re-check the markers (flips, tilt noise near the arm) and the pivot.
3. The third bag's `.db3` (214032).
4. Not built, noted: a box-moved check before Ready / the close (proposed 10-24). Flight 1's box had moved 20 mm 1 s before
   the tilt guard fired.

## Tools

All run with `PYTHONNOUSERSITE=1 /usr/bin/python3`; ROS sourced for the bag readers.

- `tools/bx_extract.py`: bag -> npz, sqlite, no metadata needed.
- Code-version checks: `tools/fingerprint.py` (the events_*.txt here), `tools/safety_map.py`.
- Box: `tools/box_event.py`, `tools/box_noise.py`.
- Vehicle: `tools/phases.py`, `tools/holds.py`, `tools/trim.py`, `tools/frames.py`.
