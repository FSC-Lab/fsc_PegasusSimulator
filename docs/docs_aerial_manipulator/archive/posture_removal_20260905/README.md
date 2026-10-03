# Joint-posture PID removal campaign — 2026-09-05

Seven DIRECT soaks on the §7.14.1 sequence testing whether the whole-body law
holds without the joint-posture PID added to `u3` (that term is not part of the
published law). It does not. The term was restored; run H validates the tree as
it now stands.

| run | mismatch | posture PID | K_y / D_y | result |
|---|---|---|---|---|
| A | +15% | removed | 2 / 4 | diverged, abort 32 s |
| B | +15% | present | 2 / 4 | stable, 75 s |
| C | matched | removed | 2 / 4 | stable, 75 s |
| D | +7.5% | removed | 2 / 4 | diverged, abort 15 s |
| E | +15% | removed | 20 / 9 | diverged, abort 3.8 s |
| F | matched | removed | 2 / 4 | diverged, abort 17 s (repeat of C) |
| G | matched | removed | 2 / 4 | diverged, abort 3.7 s (repeat of C) |
| H | +15% | present | 2 / 4 | **stable, 90 s — the shipped tree** |

C/F/G are one configuration flown three times: **1 pass in 3**. A single
completed run on this stack proves nothing (wall-clock PX4 + DDS, not
deterministic).

Full analysis, tables and mechanism: `../Command.md` §7.14.6.

## Files

- `wb_direct_soak.py` — the driver. SAFETY takeoff to 1.2 m → settle → DIRECT →
  soak → revert → land, with an abort envelope on tilt / position / altitude.
  It publishes **only** `PositionControllerReference`; the whole-body planner owns
  `WholeBodyReference` and a second publisher on that topic invalidates the test.
- `metrics.json` — per-run metrics and event log for all eight runs.
- `posture_removal.patch` — the exact C++ removal, if it is ever wanted again.
  Apply with `git apply`, revert with `git apply -R`. Removing the term also
  means removing the five `wb_posture_*` keys from the three whole-body yamls.

## Running one

```bash
# full step-0 clean between EVERY run — PX4 stays armed otherwise and the
# next takeoff produces no lift
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y
# then §7.14.1 steps 2 and 3, then:
/usr/bin/python3 wb_direct_soak.py --soak 90 --out run.json
```
