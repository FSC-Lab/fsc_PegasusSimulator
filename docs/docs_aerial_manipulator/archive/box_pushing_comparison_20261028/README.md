# 2026-10-28 — box pushing, three controllers on the latest whole-body settings (280 g, 0.40 m)

User request: compare the proposed whole-body controller with the two physical-interaction comparison
rigs (coordinated admittance, Liu et al. CEP 2024; arbitrary-convergence-time control, Chen et al. IEEE
TIE 2025) on the LATEST whole-body box-pushing settings, every reference and task setting the same, a
40 cm push; repeat each several times and report the best performance; summarise the results in
`results/simulation_results/` and export them to MATLAB.

## The common task (every attempt, every rig)

| | value | where it comes from |
|---|---|---|
| box | CAD box + 5 mm fin, **280 g** (the real box, 2026-10-25), μ 0.25 / 0.25 | `run_box_pushing_campaign.sh` MASS, the runners |
| box start | centre (0.675, 0.28): back face 0.10 m from the table's +y end | the runners' default |
| push | **0.40 m** toward −y at **21 mm/s** (19.05 s after a 1.5 s settle) | DIST / SPEED → `push_pull_push_distance` / `_mean_speed` |
| push pose | [0, 28, 32, 0] deg (2026-10-25) | the whole-body box yaml, copied by both comparison generators |
| layout | nominal (the 2026-10-21 layout); table placed from the box; Go To Start and the way home round the table | the runners, the planner block |
| planner block | identical (the comparison rigs' generators copy the whole-body one) except the coordinated rig's transit bounds (0.20 / 0.08 / 0.20, its documented requirement) and the arbitrary-time rig's own guard values (2026-10-25 user decision) | generators |
| plant | the mirror plant, headless Isaac at RTF 1, raw mocap | the stacks |
| timetable | `pl_mission.py --timetable`: ready 35 s after Go To Start, descent 2.5 s, close 8.0 s, release 22.05 s after Push (= 0.40/0.021 + 3), exit on the OPEN report, Go To Land Start 8.5 s after Exit, Land 40 s after that | campaign script |
| exit | release-first (`push_pull/release`, open, exit on the OPEN report) | the driver's default |

Kept = the driver reports `completed`, the box moved ≥ 90 % of the commanded distance (360 mm), no
step LATE on its slot. 5 kept runs per rig wanted; at most 2 attempts per run (the coordinated rig 3).
The BEST run of a rig = its kept run with the lowest slide impedance residual (the physical-interaction
metric; the figures plot it).

Flight order: whole-body, arbitrary-time, coordinated (`tools/chain.sh`, `METHODS=...`).

## Changes made for this campaign

- **The coordinated rig's OWN box-lag guard, 60 mm** (user decision, 2026-10-28, after its two attempts
  at the shared 40 mm both aborted ~13 s into the push with the box 41–42 mm behind its plan at peak push
  speed — its decoupled base lag, the box not jammed, landed upright; `pretest/guard40/`):
  `make_coordinated_yaml.py` `PLANNER_TAIL_OVERRIDES["box_pushing"]["push_pull_guard_box_lag"] = "0.060"`;
  every other guard stays the whole-body rig's. The valve profile is unchanged.

- **The comparison yamls were STALE** vs their generators (push pose [0, 32, 28, 0], abort back-off
  0.03 m): regenerated (`make_coordinated_yaml.py --profile box_pushing`, `make_arbitrary_time_yaml.py`),
  both `--check` UP TO DATE.
- `run_pl_cadm.sh` / `run_pl_atc.sh`: the flight flies a per-flight COPY of the generated yaml
  (`runs/<tag>.yaml`, passed as `WB_SIM_YAML`), with DIST / SPEED applied — the whole-body `fly.sh`'s
  pattern; the shipped yamls keep 0.50 m.
- `results/utils/run_box_pushing_campaign.sh`: one MASS / DIST / SPEED and one set of slots for all
  three rigs (the whole-body rig used to fly 500 g / 0.50 m and slot-ready 11 s); kept-criterion 90 % of
  DIST.
- `pl_mission.py`: the close gate now WAITS while the planner is still settling the claw before the room
  hold (`push_pull/info` [87]), as the arm GS does since 2026-10-24 (`--no-close-after-settle` = the old
  behaviour).
- `results/utils/build_box_pushing.py`: a best-run table (`box_pushing_sim_best.csv/.tex`,
  `metrics_best` in `box_pushing_metrics.mat`, a README table), the 3-D figure's limits from the data,
  the phase descriptions of the 2026-10-22 planner, push time printed to 0.1 s.
- The previous results (500 g / 0.50 m, 2026-10-17 and 10-25) moved, untouched, to
  `results/simulation_results/Isaac_simulation_data/box_pushing/previous_20261017_25_500g_0p50m/`
  (with their MATLAB folder).

## Pre-test (not part of the results): `pretest/`

The first coordinated-admittance attempt flew the old slots (ready 30 s, close 7.0 s). Its Go To Start
round the table is 27.3 s at that rig's bounds, so its base was still swaying at Ready (the planner
refused Ready until still), its 4.3 s descent ended 0.97 s past the close slot (LATE), and the driver
closed the jaws while the planner still reported the claw settling at 41–65 mm/s. Then the shared
40 mm box-lag guard aborted the push 10 s in (the box 41 mm behind its plan — that rig's known base
lag; landed upright). Hence the 35 s / 8 s slots and the settle wait above, for every rig.

## Flights

- Whole-body: 2026-10-28 20:37-21:03, 5 kept of 5 attempts. The chain was then paused on the user's request
  (the PAUSE file); the desktop rebooted overnight, and the chain was relaunched with
  `METHODS="arbitrary_time coordinated_admittance"` on 2026-10-29 (the campaign skips runs on disk).
- Arbitrary-time: 2026-10-29 12:39-13:19, 5 kept of 8 (aborts: box 114 mm behind its plan, dragged 50 mm
  sideways, 106 mm ahead -- its own guards, the stick-slip on its soft base; each landed upright).
- Coordinated admittance: 13:19-13:50, 5 kept of 6 (one abort at its own 60 mm box-lag guard).
- Between the two days another session rebuilt the planner (10-29 11:05: `push_pull_exit_world_held_dz`,
  default 0, absent from every box-pushing yaml, and the pick-and-place locator). Box pushing flies the same
  logic: the comparison rigs planned exactly the whole-body runs' step durations (19.3 / 3.2 / 20.5 / 6.0 /
  23.1 / 7.4 s; the coordinated rig 27.3 / 4.3 / 20.5 / 6.0 / 33.8 / 11.1 at its bounds). All three box
  yamls were `--check` UP TO DATE before the relaunch.

## Results (mean ± std over 5 kept runs; full tables in `results/simulation_results/README.md`)

| | whole-body | coordinated [Liu 2024] | arbitrary-time [Chen 2025] |
|---|---|---|---|
| e_imp slide rms (N) | 0.99 ± 0.12 | 2.58 ± 0.47 | 11.47 ± 2.06 |
| as displacement (mm) | 5.7 ± 0.8 | 27.2 ± 2.4 | 24.4 ± 4.9 |
| ‖ε_UAV‖ max / rms (mm) | 26.9 / 9.5 | 95.7 / 38.2 | 57.8 / 13.6 |
| EE RMSE (mm) | 9.8 | 34.5 | 13.9 |
| box pushed (mm, 400 commanded) | 401 ± 5 | 410 ± 9 | 406 ± 7 |
| box moved in the exit (mm) | 0.2 | 24.6 | 18.7 |
| best run (lowest slide e_imp) | run3: 0.78 N / 4.3 mm | run1: 1.93 N / 24.0 mm | run3: 8.07 N / 16.3 mm |

The simulation was closed after the campaign (no Isaac, PX4, stack nodes or tmux left).
