# q2 redundancy sinusoid, 25° ± 15° at a 12 s period — simulation check (2026-09-24)

Question: before flying it, does the EE circle (r 0.5 m, 24 s lap = 0.131 m/s,
two laps) work with q2 = 25° ± 15° and a 12 s q2 period instead of the flown
30° ± 10° / 48 s?

**Answer: yes at a 55° fold.** At the flown 60° fold the planner refuses it
(q3 = 60° − q2 reaches exactly its +50° limit). At 55° it flew both laps with no
saturation, clamp or abort; base and EE errors equal the old design's; joint
error ~2x (lag 0.44–0.50 s); q3 peaked at 47.0°, 3° from the +50° guard.
Full write-up: `../wb_l1_4d_flight_20260924/report.html` (section 4, fig. 7),
Command.md §7.17.10.

## Runs (rig l1_4d_fused, config-A plant, shiqi-desktop, RTF 0.49)

| tag | design | requested s | plant sees |
|---|---|---|---|
| `q2new_f55_c25_a15_p12` | fold 55, 25 ± 15, 12 s | 1.0 | ~s 2 (stress) |
| `q2new_plant1` | fold 55, 25 ± 15, 12 s | 0.49 | s 1 |
| `q2old_plant1` | fold 60, 30 ± 10, 48 s | 0.49 | s 1 |

**The simulator runs at half real time while the law and planner run on the wall
clock**, so a requested s reaches the plant as s / RTF. Request s = RTF to test
the planned pace; verify with PX4's `sensor_combined.timestamp` span over the
wall span.

## Reproduce

    ./run_q2.sh <tag> <fold> <q2_center> <q2_amp> <q2_period> [laps] [s] [stack] [cfg]
    # e.g. ./run_q2.sh q2new_plant1 55.0 25.0 15.0 12.0 2 0.49

It backs up the 4-D `_sim` yaml, applies the four `ee_traj_*` keys, flies
`wb_l1_tune_cycle.sh` with `WB_L1_MISSION=ee_circle`, records `bags/<tag>` with
the hardware flights' topic set plus Isaac ground truth, and restores the yaml
on exit. Then:

    source ~/ros2_ws/install/setup.bash
    PYTHONNOUSERSITE=1 /usr/bin/python3 tools/extract_bag.py bags/<tag> <scratch>/<name>.npz
    PYTHONNOUSERSITE=1 /usr/bin/python3 tools/sim_q2.py <name>       # one run
    PYTHONNOUSERSITE=1 /usr/bin/python3 tools/compare_q2.py          # all runs, physics time
    tools/probe_q2.sh                                                # planner feasibility only

`common.py:load()` reads npz files from a fixed scratch path; edit it. Bags
(106–142 MB each) are not meant for git.
