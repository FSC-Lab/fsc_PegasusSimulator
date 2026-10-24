# Claw hold precision on hardware — input for sizing the mail slot (2026-10-23)

How still the T650-AM's claw (grasp point) holds at a commanded target, from the recorded hardware flights of the
whole-body 4-D L1 controller. Deliverable for the mail-delivery study: `stats.json` (every number, machine-readable),
`tables_C.md` / `tables_M.md` (all tables, both estimates), `windows.csv` (every hold window), `events.csv` (the
error at each gripper command), `hover_stats.log` (per-window printout and cross-checks).

## Data

| set | flight | bag (stamp) | payload | note |
|---|---|---|---|---|
| main | 1014 PD-1 / PD-2 | `_172845`, `_183914` | none | PD-1 drop hold over the full bin kept apart (aero); PD-2 only a DIRECT-entry hover |
| main | 1015 E-1 / D-1 | `_empty_172318`, `_175059` | none / 190 g basket | D-1 has the pre-fix loaded wrist swing |
| main | 1016 E-1 / D-1 / D-2 | `_empty_134438`, `_135100`, `_140020` | none / basket / basket | |
| main | 1020 | `_182234` | basket | extracted here (`tools/run_extract.sh` -> `npz/s1.npz`) |
| main | 1021 FF / BP | `_164338`, `_170123` | none | raw-mocap stack; BP's clamped push (close -> abort open + 1 s) excluded |
| supp | 1009 PP-1/2/3, 1010 PP-4 | `_172557`, `_180228`, `_181817`, `_190719` | none / basket / none / basket | ADDED for their long holds (PP-1 waited 38 s at the pick); set-down pick-and-place yaml, k_R/k_w 1.6/1.2 as on 10-14/15 (10-16+ pick-and-drop: 2.134/1.567; 10-21: box-pushing tune); reported apart and in "all" |

Every flight except 1020 reuses the npz of its own analysis folder (`../../wb_pick_drop_flight_2026101{4,5,6}`,
`../../wb_box_pushing_flight_20261021`, `../../wb_pick_place_flight_2026100{9}`, `..._20261010`).

## Method (`tools/hover_common.py`, `tools/hover_stats.py`, `tools/summary.py`)

- **Hold window**: the whole-body planner in `HOLD` (its leg-end edge = "the reference becomes static"; a min-snap
  leg's tail stays below 5 mm/s for up to 1.3 s before it, so speed alone was not used) AND the streamed
  `WholeBodyReference` static (r_ed, x_cd < 5 mm/s, headings < 0.5 deg/s), inside DIRECT. Ended at the next
  FLYING / ABORTING status edge, and cut at every gripper motion (window ends 0.1 s before the jaws move, resumes 1 s after).
- **Settle levels**: `settled` = from 3 s after the leg end (the spec); `s1` = from 1 s; `full` = from the leg end.
  The gripper holds of these missions last 0.2-3 s (operators pressed Pick / Drop 0.4-1.5 s after arrival), so
  they exist almost only as `full`.
- **Types**: `approach_wait` (claw behind / above the pick, or above the place in set-down mode, before the press),
  `at_pick` (jaws open around the fin, until the close), `at_drop` (until the open), `above_handle` (box pushing,
  CoM-anchored), `at_handle` (box pushing, claw WORLD-held at the fin, until the close) = the **gripper holds**;
  `other` = any other static hover >= 1.5 s. Kept apart: `direct_entry` (the hover from the SAFETY->DIRECT switch,
  observer restarting), `post_lift` (right after the load step), `post_abort`, `bin`, `near_ground` (< 0.5 m), contact.
- **Load**: loaded = from the gripper close at the pick to the open at the drop / place, on the basket flights.
- **Error** = claw - r_ed (the streamed reference), rotated into the nose-heading frame: **along** (+ = claw ahead
  of the target), **lateral** (+ = left), **vertical** (+ = above). Claw = the planner's FK (`transition_planner.
  make_params_t650`, hardware base_com, grasp point 0.108 m from the wrist) on the encoder angles
  (joint_states order [q2, q3, q1, q4], verified in every bag).
- **Two pose sources**: **C** = odometry position + PX4 `vehicle_attitude` — exactly what the planner's
  `pick_place/arrival_error` (and the law) use; **M** = the `/uav_0/mocap` pose (frozen repeats, flips and
  > 50 mm jumps vs odometry dropped). **C is used for the tables** (it is the one the planner agrees with; see
  cross-checks); M is in `tables_M.md`.
- 50 Hz grid, receive time on the recorder (the fused odometry's header is on PX4's clock, not used).

## Results (estimate C) — mean / std / p95|.| / p99|.| / max|.|, mm; heading and tilt in deg

| pool | windows / time | along-nose | lateral | vertical | EE heading mean / std / p95 | body tilt mean / std / p95 / max |
|---|---|---|---|---|---|---|
| **unloaded, settled >= 3 s (spec), all** | 8 / 73 s | -4 / 12 / 25 / 32 / 34 | -2 / 13 / 29 / 43 / 49 | 1 / 3 / 6 / 7 / 13 | 0.35 / 0.27 / 0.8 | 1.8 / 0.5 / 2.8 / 3.5 |
| unloaded, settled, main only | 2 / 16 s | -4 / 13 / 22 / 30 / 34 | -2 / 9 / 16 / 29 / 30 | 1 / 3 / 5 / 12 / 13 | 0.59 / 0.15 / 0.8 | 2.6 / 0.3 / 3.2 / 3.5 |
| **unloaded, gripper holds from the leg end, all** | 25 / 87 s | -1 / 17 / 33 / 48 / 53 | 0 / 13 / 26 / 41 / 49 | 1 / 3 / 7 / 8 / 16 | 0.33 / 0.67 / 0.9 | 2.0 / 0.6 / 3.0 / 4.2 |
| unloaded, gripper holds from the leg end, main | 17 / 32 s | -3 / 17 / 43 / 48 / 53 | 1 / 8 / 17 / 24 / 24 | 1 / 4 / 7 / 13 / 16 | 0.45 / 1.08 / 1.1 | 2.6 / 0.4 / 3.3 / 4.2 |
| unloaded, every hold from the leg end, all | 38 / 138 s | -2 / 16 / 33 / 48 / 57 | -1 / 12 / 25 / 39 / 49 | 1 / 4 / 7 / 14 / 22 | 0.36 / 0.60 / 1.2 | 1.9 / 0.6 / 3.0 / 4.2 |
| **loaded (basket), gripper holds from the leg end, all** | 6 / 9 s | 37 / 26 / 75 / 76 / 76 | 13 / 16 / 46 / 48 / 48 | 9 / 5 / 16 / 18 / 18 | 0.90 / 0.87 / 3.0 | 2.5 / 0.5 / 3.2 / 3.3 |
| loaded, every hold incl. Drop Start hovers, all | 9 / 19 s | 33 / 51 / 143 / 167 / 178 | 14 / 26 / 59 / 71 / 74 | 7 / 7 / 16 / 18 / 22 | 1.38 / 1.06 / 3.1 | 2.7 / 1.5 / 6.5 / 8.4 |

M (mocap pose) gives the same horizontal spread (unloaded settled along -6/13/28/35/39, lateral 2/14/32/44/49) but a
wider vertical (-4 / 7 / 14 / 17 / 20): its vertical carries a per-flight offset of up to 15 mm from the mocap
rigid body's attitude definition (see cross-checks).

**Offset vs wander** (unloaded, all; rms of per-window means / pooled within-window std, mm): from the leg end
along 17 / 13, lateral 8 / 11, vertical 3 / 3; settled along 9 / 11, lateral 6 / 13, vertical 1 / 3. Per-hold mean
along-nose offsets span -36..+46 mm when the hold starts at the leg end: arrival overshoot along the travel
direction, not a fixed bias.

**Arrival (unloaded, 3-D error, all flights, 35 legs)** — |e| median / p90 at the leg end 20 / 35 mm; at +1 s
19 / 38; +2 s 16 / 23; +3 s 12 / 20; +5 s 10 / 18 (legs still holding: 35, 24, 14, 7, 5). Share inside 20 mm:
46 % at the leg end, 71 % at +2 s, 86 % at +3 s, 5 / 5 at +5 s. With a 1 s dwell: inside 30 mm at once in 20 / 35
legs, inside 20 mm after a median 0.1 s in 14 / 35, inside 10 mm in only 5 / 35 (median 3.8 s) — most holds ended
(median 1.6 s) before. Loaded (n <= 4): 50-71 mm at the leg end, 24-39 mm at +3 s.

**At the gripper command** (mean over 0.25-0.05 s before the jaws move, `events.csv`): unloaded, 12 events (pick
closes, empty-run drop opens, the box-push close): 3-D median 10 mm, max 32 mm; along max 32, lateral max 5,
vertical max 8. Loaded drop opens (4): 3-D median 33 mm, max 63 mm (along +46 / +59 in two of them — the drop
was clicked 0.7-0.9 s after arrival).

**Wander frequency** (the three holds >= 13 s: 1009 PP-1, PP-3, 1021 FF): horizontal peaks at ~0.1 Hz (10 s period)
with a second component at 0.3-0.37 Hz; ~70 % of the horizontal variance is below 0.2 Hz. Vertical 0.4-0.9 Hz,
std only 2-3 mm. The windows are 13-39 s, so the < 0.1 Hz content is poorly resolved.

## Cross-checks

- **Planner `arrival_error` vs this FK (C)**: identical — per flight rms 0.1-0.75 mm, max 1.9 mm (10 Hz sampling
  vs the 50 Hz grid), over 25-425 samples where its target equals the streamed reference. (Elsewhere it reports the
  LAST claw leg's goal, not the current reference, so it must be filtered that way before use.)
- **Law task error `wb_control_debug[24..26]`** = claw error minus CoM error to 0.1 mm on every CoM-anchored hold:
  in pick-and-drop the claw's world error IS the airframe's drift; the arm task error is a few mm.
- **M - C** in holds: per-flight mean -6..+6 / 0..+8 / -15..+7 mm along / lateral / vertical, std 1-9 mm (14 mm on
  1014 PD-1, whose mocap stream ran at 391 Hz with 71 % repeated frames). Mocap tilt vs PX4 tilt differs by up to
  2.7 deg (1015 E-1: 5.5 vs 2.8 deg) — the rigid body was defined differently on different days.

## Caveats

- **The spec's 3-s-settled sample is small**: 8 windows / 73 s unloaded, of which 57 s come from the 10-09/10
  flights and 13.5 s from the 10-21 hover; the 10-14..10-20 pick-and-drop flights add 2.8 s. Loaded
  settled: 2 windows / 3.7 s. Treat the "from the leg end" rows as the operational numbers.
- **Loaded rows are a loose upper bound**: a 190 g basket hanging on a hook (pendulum, load step), arrival
  transients only, and the largest numbers come from 1015 D-1 (pre-fix wrist swing) and 1009 PP-2 / 1010 PP-4 (before the 10-16 gain and wrist fixes).
- **Absolute offsets carry ~+-15 mm of calibration uncertainty** (PX4 vs mocap attitude, FK model vs the real jaws
  — never measured on hardware). Calibrate the claw-to-slot offset on the ground (marker on the claw) or add it.
- Excluded and relevant to a wall-mounted slot: the 10-14 hold over the full bin (near-structure rotor wash)
  doubled the wander (along std 31-37 mm). DIRECT-entry hovers (along std 26-30 mm) are excluded too.
- One world-held window only (1021 at the fin, 2.4 s): horizontal std 5-6 mm, vertical 6 mm; that mode started
  the arm swing that ended in the 10-21 q3-guard abort.

## Re-run

```bash
cd tools
bash run_extract.sh                                             # only the 1020 bag (ROS 2 sourced inside)
PYTHONNOUSERSITE=1 nice -n 19 taskset -c 16-31 /usr/bin/python3 hover_stats.py > ../hover_stats.log
PYTHONNOUSERSITE=1 /usr/bin/python3 summary.py C > ../tables_C.md
PYTHONNOUSERSITE=1 /usr/bin/python3 summary.py M > ../tables_M.md
```

`stats.json` keys: `pooled[set][estimate][settle][type][load]` (set main / supp / all; estimate C / M; settle full /
s1 / settled; type = a window type or `gripper_holds` / `all_free`; per component: n, mean, std, rms, signed
p01/p05/p95/p99, p95_abs, p99_abs, max_abs, window_mean_rms, within_window_std, plus norm3d, horizontal,
heading_deg, tilt_deg, frac_within_10/20/30mm, frequency), `per_flight`, `arrival_summary` (incl.
`error_vs_time_after_leg_end`), `arrivals`, `events`, `windows`, `cross_checks`.
