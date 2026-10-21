# Box-pushing mean speed, 240 g box, 0.10 m push (2026-10-21)

User request (2026-10-21): for the first hardware push, the operator types only the
**Push Distance** on the arm GS; the push **time** follows from ONE planner setting, the
push **mean speed** (`push_pull_push_mean_speed`, no value in the code), and the GS shows
the time, contact time and mean speed under it, then the box's travel and speed live.
This campaign picks that speed in Isaac for the test of the day: a **240 g** box
(the real box emptied), pushed **0.10 m**.

## Setup

- Scene `07_px4_t650_aerial_manipulator_box_pushing.py`, the 2026-10-21 layout (table
  corner (0.60, 0.50), box centre (0.675, 0.28)), mu 0.25 / 0.25, T650 gear, raw mocap,
  headless, RTF 1.
- Push yaml = the generated `_sim_box_pushing.yaml` + the hardware guard values
  (`push_pull_guard_tilt_deg` 10, `push_pull_abort_release_wait_s` 1.5).
- `MASS=0.240 DIST=0.10`, mean speeds 8 / 13 / 21 / 30 mm/s (push 12.5 / 7.7 / 4.8 /
  3.3 s), 3 repeats, interleaved. 21 mm/s is the speed every validated 0.50 m push flew
  (0.50 m / 24 s).
- `tools/chain.sh` (pause flag `PAUSE` before a flight's clean slate), `tools/fly.sh` (a
  copy of the 10-16 campaign's: gives a planner that knows `push_pull_push_mean_speed`
  the SPEED, an older one the time d / v).

## Runs

| tag | note |
|---|---|
| v21_1 (first attempt) | **NOT FLOWN**: it started after the generator already wrote the speed key while the installed planner did not know it yet; the scene's planner check (correctly) refused the stack, Isaac never started, the driver hung. Stopped, files kept as `x_v21_1_invalid.*`; re-flown by `tools/chain2.sh` on the new planner. |


**12 / 12 flights completed** (`tools/chain.sh` flew v08_1, v13_1 on the OLD planner, the time
d / v; `tools/chain2.sh` the other ten on the NEW planner, the speed key). Per-run numbers:
`speed_metrics.txt` (`tools/speed_metrics.py`) and `table.txt` (`tools/table.py`, the 10-16 scorer).

## Results, mean [min .. max] over 3 runs

| mean speed [mm/s] | 8 | 13 | **21** | 30 |
|---|---|---|---|---|
| push time / clamped [s] | 12.5 / 14.0 | 7.7 / 9.2 | **4.8 / 6.3** | 3.4 / 4.9 |
| box stuck in the push [%] | 29 [23..42] | 13 [6..22] | **4 [0..11]** | 2 [0..6] |
| slide impedance residual [N rms] | 0.71 | 0.72 | **0.54** | 0.65 |
| box travel [mm of 100] | 99 | 100 | **99 [98..101]** | 96 [94..97] |
| box peak speed [mm/s] (plan's) | 40 (18) | 50 (29) | **68 (47)** | 81 (69) |
| box tilt [deg] (guard 2) | 0.2 | 0.1 | **0.2 [0.1..0.4]** | 0.5 [0.3..0.7] |
| box lag [mm] (guard 40) | 10 | 11 | **13** | 12 |
| q3 lowest, moving / release [deg] (guard 10) | 20 / 24 | 20 / **20 [9.9..28]** | **20 / 24 [23..26]** | 24 / 22 |
| vehicle tilt [deg] (guard 10) | 3.2 | 3.1 | **2.9** | 2.6 |

**21 mm/s** (= the speed every validated 0.50 m push flew, 0.50 m in 24 s): the box slides
almost without stopping (8 mm/s sticks a third of the push), the lowest impedance residual, the
box stays flat, the full 0.10 m, and the jaws hold the box 6.3 s. 30 mm/s also works but tips the
box more (0.3-0.7 deg), lurches it to ~95 mm/s and falls ~4 mm short. Shipped: the generator's
`push_pull_push_mean_speed: 0.021`, `push_pull_push_min_time_s: 3.0` (sim, hardware and CA box
yamls). The q3 minimum over the whole push step is AT THE GRASP (0.1-0.3 s into the push step,
before anything moves: 13-20 deg) -- speed-independent.

**The release window (jaws opening) is still the weak spot, at any speed:** v13_2 jumped q2 27 ->
44.7 deg (its 45 stop) and q3 31 -> 9.9 deg within 0.25 s of the release start, recovered in 1 s,
no guard trip (< 10 deg for under 0.2 s). The other 11 runs stayed at 18.6-27.6 deg. Same
mechanism as the 10-17 release record (the claw's room hold handed to the CoM anchor while the
arm absorbs the base's wander); one run in 12, not tied to the speed.

## The arm GS (built and checked the same day)

- `tools/gs_push_check.sh` (real planner, domain 77, the panel offscreen): PASS -- preview in
  amber, sent, live, summary, and every Monitor label whole at an 840 px window
  (`gs_push_1..6_*.png`).
- Live in Isaac on a 30 mm/s push (v30_1, the station of the sweep's own stack):
  `gs_live_v30_1_mid.png` (Push time 1.8 / 3.3 s, box 53 / 59 mm, 51 / 67 mm/s) and
  `gs_live_v30_1_after.png` (the summary: 3.3 / 3.3 s, 29.7 / 30.0 mm/s, box 99 of 100 mm).

## Green / red (same day, user request)

The plan column is **green** inside the tested range and **red** outside it, and the note
names what to retype. The ranges are planner yaml keys next to the speed (one source, display
only): `push_pull_push_distance_ok: [0.10, 0.50]` (this sweep's 0.10 m, the standard validated
0.50 m) and `push_pull_push_speed_ok: [0.008, 0.030]` (this sweep's speeds). Push time and contact
time are green only when both are; a negative distance (a pull, never tested) is red; the live and
held rows are not coloured. `tools/gs_push_check.sh`: 0.05 / -0.10 / 0.60 m red
(`gs_push_1b_short.png`, `1c_pull`, `1d_long`), 0.50 / 0.10 m green -- PASS.
