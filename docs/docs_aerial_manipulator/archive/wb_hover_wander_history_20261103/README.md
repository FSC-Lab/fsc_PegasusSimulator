# Hover wander across every whole-body flight, 10-20 .. 11-03, split at the 10-29 evening sidewall

User question (2026-11-03, after `../wb_valve_turning_flight_20261103/`): the box pushes wandered ~15 mm/s above the
handle, the valve 38-53 mm/s. Check all box and valve history. The user reports:
- a sidewall was added beside the table on 10-29 evening, and none existed for any box push;
- the 11-02 tracking flights showed larger wind disturbances, even mid-field.

Is the wall a problem? Does the valve fly closer to the table surface?

Data: 46 whole-body flights. Box pushes 10-21 .. 10-27 (13 bags), tracking 10-27 .. 10-29 (7), pick-and-drop 10-20/10-28
(5), valve 10-28 .. 11-03 (16), mail 10-31 (4). Before the wall: everything up to 10-29 15:30. After: mail 10-31 and
valve 10-31 .. 11-03. **The 11-02 tracking bags are not in the repo.**

## Method

- **Holds**: every still-reference hold of at least 3 s (first 1.5 s dropped, z > 0.6 m); 107 holds in `holds.csv`.
  Contact holds and the pick-and-drop pillars are left out of the groups.
- **CoM wander**: the law's CoM error about its mean.
- **Speed**: the EKF2 horizontal speed.
- **External force** (`ext_force.py`): from the raw mocap alone, F = m a - T (R e3) -- wind, or anything else that
  pushes the vehicle. Only holds of 9 s or more are reliable.
- **PX4-vs-mocap tilt error** (`att_err.py`).
- **The law's slow moment estimate** (`rot.py`, wb_control_debug [34..35]).
- **Geometry against the planner's own table** (pl_info [77..86]).

## Results

| site (free holds) | wall | holds | CoM wander rms | EKF2 v p50 | true tilt rms | moment estimate rms |
|---|---|---|---|---|---|---|
| mid-field (start mark, tracking, Land Start) | before | 42 | 19.5 mm | 22 mm/s | 0.38 deg | 11 mN.m |
| mid-field | after | 18 | 19.3 mm | 25 mm/s | 0.43 deg | 7 mN.m |
| at the table, above the handle | before (box pose) | 3 | 13.1 mm | 21 mm/s | 0.50 deg | 6 mN.m |
| at the table, above the handle | after (valve pose) | 6 | 32.8 mm | 41 mm/s | 0.85 deg | 27 mN.m |
| at the table, after an abort (home pose, ~1.5 m) | before (box) | 6 | 6.5 mm | 14 mm/s | 0.31 deg | 6 mN.m |
| at the table, after an abort (home pose, ~1.5 m) | after (valve) | 6 | 24.6 mm | 44 mm/s | 0.62 deg | 9 mN.m |

Values are group medians; per-hold rows are in `summary.txt`.

- **The base speed in the same 1 s right after Go To Start arrives** (`history.py`):

  | | flights | median | range |
  |---|---|---|---|
  | box | 13 | 25 mm/s | 10-49 |
  | valve before the wall | 3 | 15 mm/s | 9-43 |
  | valve after the wall | 12 | 35 mm/s | 21-70 |

  The "box 14-16 mm/s" of the 11-03 analysis was 10-27 only.
- **Same day, same flights**: on 10-31 the mail flights' mid-field holds wandered 9-15 mm, but 25 mm at the mailbox
  approach next to the table.
- **The extra motion is the vehicle's own tilt, not a sideways push.** In every long hold, before and after, at the
  table or mid-field, the slow external force is ~0.1 N (165100's 42 s hold above the handle: 0.09 N). The true tilt
  at the table, however, doubled (0.6-0.85 deg vs 0.3-0.5).
- **Something tilts the vehicle, and the law sees it.** Above the handle its slow moment estimate is 4x the box's
  (27 vs 6 mN.m).
- **Not the attitude estimate**: the PX4-vs-mocap tilt error is unchanged (0.1-0.4 deg in both).
- **Not flying lower.** The base, rotor plane and gear sit at the same height above the table top:
  - rotor plane 0.69-0.70 m (box) vs 0.72 m (valve), and 0.66 m on 11-03 (Vertical Margin 0.10);
  - that is 3.5-3.8 rotor radii, where ground effect is under 1 %.
- **The valve hovers beside the table's long side**: 1-12 cm in front of the near edge, 11-15 % of the rotor area over
  the 0.15 m-deep table, the far rotor tips 0.15-0.30 m beyond its back edge. The box hovered past the table's END:
  2-3 % over it, tips 0.32-0.34 m past the back edge. This hold geometry was the same for the valve before and after
  the wall.
- **The work site moved.** The valve's hold above the handle sat at world x 0.67-0.85 on 10-28/29 and at x 0.34-0.43
  from 10-31 on.

## Reading

- The extra wander appears only at the table site, after 10-29, and has the signature of a disturbing MOMENT:
  - it tilts the vehicle;
  - the tilt points thrust sideways (0.5 deg turns 0.3 N sideways);
  - the base wanders.
- That fits disturbed airflow through the rotors near a vertical surface, i.e. downwash recirculating off the
  floor/table and the new wall.
- It does not fit a horizontal wind gust (no sideways force) or ground effect (same height, under 1 %).
- What the data cannot separate is the wall itself from the work site's ~0.45 m move. The wall's position is not in the
  data: the user has to give it.
- Mid-field shows no change in these bags. But the after-wall mid-field holds are short (52 s, mostly right after DIRECT
  entry), and the 11-02 tracking runs were not available.

## Suggested checks

- **The 11-02 tracking bags**: copy them in. The 10-27..10-29 tracking flights of the same tasks are the before-wall
  baseline.
- **A no-contact hover A/B in the valve pose at the valve hold, 30 s each**:
  - as now;
  - with the wall removed or the table at least ~0.5 m from it.

## Tools

`tools/run_extract.sh` (the bags not extracted elsewhere -> npz/), `history.py`, `holds.py` (-> holds.csv),
`ext_force.py`, `att_err.py`, `rot.py`, `summary.py`. Run with `PYTHONNOUSERSITE=1 /usr/bin/python3` from `tools/`.

## Follow-up: is the translational observer bandwidth the lever? No (2026-11-03)

The flown values: box 1.0, valve 2.0 (unchanged since 10-19, so on 10-28/29 and 11-03 alike), free flight /
pick-and-drop / mail 2.927.

**Mid-field hovers (home pose, start area), grouped by bandwidth:**

| `wb_l1_omega_c_t` | holds | CoM wander median | EKF2 v50 | true tilt |
|---|---|---|---|---|
| 1.0 (box) | 17 | 21.0 mm | 25 mm/s | 0.42 deg |
| 2.0 (valve) | 17 | 19.0 mm | 25 mm/s | 0.42 deg |
| 2.927 (free flight) | 17 | 18.8 mm | 20 mm/s | 0.41 deg |

No dependence.

**At the table**, the valve flew 2.0 both on 10-28/29 (first second above the handle 15 mm/s, as calm as the box) and on
10-31 .. 11-03 (35 mm/s). Same gain, different outcome.

**The table-site signature is a moment (tilt), which the translational observer does not target.** The rotational
bandwidth is 0.7428 in every config, and Isaac aborted at 2.0 and diverged 1 in 2 at 1.25.

**History in Isaac**: 1.0 was what failed the clamped valve turn on 10-17 (base wandering 40-64 mm, the arm on both
stops); 2.0 fixed it, and the U push was validated at 2.0. 1.0 has never been flown on the U push.

## CORRECTION: the timeline (user, 2026-11-03, final) and the table-site control

**Final timeline (user):**
- 10-31: the mail-box test, then the front wall that same day, before the 10-31 valve flights (17:49 onward).
- The four corner box supports (2-3 cm tall, 54 mm sides) went on after the mail test, on 10-31 or 11-01 (the user's
  messages allow both).
- The table had already moved to its new spot (the hold at world x ~0.35, ~0.4 m from the 10-28/29 spot) before the
  mail test.

So:
- **before**: everything up to the 10-31 mail flights;
- **after**: the 10-31 valve flights onward;
- the ONLY data at the new spot without the wall are the mail flights.

`tools/site_control.py`: free planner HOLDs of at least 1.5 s near the console table, on-handle holds left out.

| group | wall | base speed (holds) | CoM wander | first second above the handle |
|---|---|---|---|---|
| box, old spot (10-21..10-27) | no | 18 mm/s | 8.7 mm | 25 mm/s (13 flights) |
| valve, old spot (10-28/29) | no | 14 | 4.9 | 15 (3) |
| mail, NEW spot (10-31) | no | 22 (11, 22, 51) | 18.7 | -- |
| valve, new spot (10-31) | YES (supports maybe not yet) | 34 | 29.9 | 31 (4) |
| valve, new spot (11-01, 11-03) | YES + supports | 45 | 31.9 | 38 (8) |

**Reading.**
- The wall and the new spot cannot be separated: every valve flight at the new spot had the wall.
- The mail flights at the new spot without the wall are short and mixed: three holds, 9 s, one as unsteady as the
  valve flights.
- The supports show nothing measurable: 10-31 vs 11-01/11-03 are within scatter, whichever day they went on. Physics
  agrees: 0.55-0.7 m below the rotor plane, ~3 % of one rotor disc each.
- The hover geometry relative to the table is the same at both spots, so the candidates are the wall and the room
  location.
- The earlier "before / after 10-29" tables above use the wrong date; read this section instead.

Decisive test: a no-contact hover in the valve pose at the valve hold point, about 30 s each:
1. as now;
2. the wall removed (table in place) -- the deciding one;
3. if it still wanders, the table and valve removed: the room location alone.
