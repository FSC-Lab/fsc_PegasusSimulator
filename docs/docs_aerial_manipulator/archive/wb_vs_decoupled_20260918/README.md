# Whole-body (4-D L1) vs decoupled (geometric + L1) on the same end-effector task — 2026-09-18

Flight-test preparation campaign. Every run: `application/robotic_arm/utils/am_compare_cycle.sh`
(clean slate → controller stack → Isaac/PX4/arm → `am_ee_compare_driver.py`), launched by
`run_compare.sh` with tags `wb_c<lap>`, `dec_c<lap>`, `wb_f<lap>[_y<heading>]`,
`dec_f<lap>[_y<heading>]`. Scores: `am_ee_compare_score.py`; report page: `make_report.py`
→ `report.html` (published as the artifact "AM-T650 Flight-Test Prep"). Narrative and tables:
`Command.md` §7.18.3–7.18.4.

Files: `wb_*.npz` / `decoupled_*.npz` (driver recordings: odom, joints, the planner stream +
converted base reference, EE ref/measured, control debug, arm reference), `logs/` (stack,
Pegasus and driver logs per run; `*_FAILED_*` = the two aborted first attempts of the decoupled
rig: SAFETY UDE bound ±2 N could not lift the +10 % plant; bridge output re-entered the planner
as a target), `cmp_*`/`sweep*` (scorer JSON + plots), `runs.txt` (every launch with args).

Figure-8 trap: the planner anchors the lemniscate by its START TANGENT along the nose, 45° off
the long axis — heading 45/135 puts the long axis on world x/y, heading 0/90 on the diagonals.

## Why the arm GS drew three rings (2026-09-18)

Measured on `wb_wb_c24.npz` (the shipped circle: r 0.75 m, 2 laps of 23.6 s, arm q2
sinusoid 30 ± 10 deg with a 48 s period): the END-EFFECTOR path is an exact circle
(least-squares fit r = 0.7500 m, radius span over the run 0.0 mm), while the DRONE path —
the same reference minus the arm offset — drifts between laps because the q2 sweep takes
the whole run: lap 2 is 11.5 mm smaller in radius and 29 mm higher than lap 1. So the
screen carries EE (0.750), drone lap 1 (0.799) and drone lap 2 (0.7875). Both were
coloured by their own speed map, and at constant speed both sit at the top of their ramp
(two near-identical yellows), so the grouping was invisible; the drone path is now dashed.
Setting `ee_traj_q2_period_s` to one lap would make every lap identical — unflown, and it
doubles the arm's peak joint rate.

## The tuning decision and the safety guard (2026-09-18, second pass)

`run_gain_check.sh [wb|decoupled] <tag> <key=value ...> -- <driver args>` flies one demo
trajectory with a temporary yaml edit (restored on exit); `wb_yaml_set.py` is its scalar
setter for keys outside the `wb_*` blocks; `am_guard_report.py` scores what a guard trip
left behind (`am_ee_compare_driver.py --abort-mid-run` commands one).

- `wb_c48_wct1` — the 7.18.1 step-tuning candidate (`omega_c_t` 1.0) on the DEMO circle:
  worse than the shipped 2.0 on every column (EE 130 vs 121 mm, tilt 5.2 vs 1.5 deg).
  **Shipped config unchanged.**
- `wb_c48_guardtrip` — drift limit 0.12 m: tripped 1.6 s into DIRECT, held, arm home.
- `wb_c48_midrun_abort` — SAFETY commanded 30 s INTO the tracked circle: 291 mm of
  travel, settled 4 mm from the trip point at the trip altitude, arm folded from
  mid-sweep to home, planner stopped.
- `wb_c48_diverge` — `wb_k_r` 3.0 / `wb_k_w` 1.8: tripped on tilt at 20.1 deg while
  rotating at 96 deg/s, and flipped anyway. The guard is a fallback, not a rescue.
- `decoupled_dec_c48_guardtrip` — the NEW drift watchdog in the geometric+L1 fork,
  tripping at 3.3 deg tilt and 14 deg/s: the level walk-away the tilt and rate limits
  cannot see. Held, 9 mm from the trip point.
