# A lower drop: Drop Margin 0.05 m, and the Drop Start height it needs (2026-10-26)

User request: lower the release in the pick-and-drop, in the simulation first, then choose the
Drop Start height.

> **NOT KEPT (same day, user's call):** the three-controller comparison flies ONE margin for all, and
> MAC's ~125 mm standing height error at the release (claw 125 mm low in both of its 0.20 m drops,
> basket tilted 15-16 deg) would put its basket ~8 cm below the rim at 0.05 -- into the tray wall. The
> floor for MAC is ~0.20 (est. clearance 0.15 ~0, 0.20 ~+45 mm on a level approach). The comparison
> flies the mirror's **0.20 / Drop Start 1.0** (the user keeps the climb into the drop to show the
> altitude change), and both simulation overrides below were removed again; the generated yaml is
> byte-identical to before.

## What sets the release height

- The Drop point (`pick_place_place_point`, z 0.8455) is the payload's centre resting on the
  tray cover, i.e. on the rim (0.813) + 32.5 mm.
- The planner holds the payload centre one Drop Margin (`pick_place_drop_height`) above it and
  opens the gripper there. The drop leg has no approach and no descent. So at the release the
  payload's bottom sits one Drop Margin above the rim.
- The Vertical Margin (`pick_place_approach_dz`) plays no part in the drop. In this task it is
  only how far Exit To Pick lifts the payload.
- Ready To Drop flies one straight line from Drop Start to that hold. With Drop Start at 1.0 m
  the hanging payload starts at ~0.74 m, below the rim, and climbs into the tray. A lower
  release therefore crosses the tray's near wall lower.

## Change (simulation only) -- REVERTED, see the note above

In `pick_place_tune_20261001/tools/make_pick_place_yaml.py`, profile `pick_and_drop`, the
generated `..._sim_pick_and_drop.yaml` regenerated:

| key | mirror / hardware | simulation now |
|---|---|---|
| `pick_place_drop_height` | 0.20 | **0.05** |
| `pick_place_place_start` | [0.8, -1.0, 1.0, -180] | **[0.8, -1.0, 1.15, -180]** |

The hardware yamls are unchanged.

## Flights (whole-body, headless, 200 g, raw mocap, the corrected arm-bridge drain)

`../pick_and_drop_comparison_20261026/tools/run_pnd.sh wb <tag>` with `PND_RUNS_DIR=runs`.
Scored with `../pick_place_drop_20261014/tools/drop_score.py` and `tools/tray_clearance.py`.

| | reference (margin 0.20, DS 1.0) | A: margin 0.05, DS 1.0 | B: margin 0.05, DS 1.15 |
|---|---|---|---|
| run | `results/.../partial_drain_20261026/whole_body_l1_pnd_run1` | `m05_ds100` | `m05_ds115_b` |
| completed | yes | yes | yes |
| payload centre at the open | 1.04 m | 0.89 m | 0.89 m |
| payload bottom when it starts to fall | +184 mm over the rim | +35 mm | +37 mm |
| **payload's lowest bottom over the tray outline** | +155 mm | **+27 mm** | **+44 mm** |
| skids vs the tray (simulated AM_xfwd gear) | 175 mm | 25 mm | 24 mm |
| body z at the open | 1.305 | 1.156 | 1.156 |
| claw error before the open | 7.8 mm | 8.4 mm | 5.6 mm |
| payload at rest | flat, 41 mm off centre | flat, 7 mm off | flat, 3 mm off |
| Ready To Drop: body z | 1.00 -> 1.31 | 1.00 -> 1.16 | 1.15 -> 1.16 (level) |

- **Drop Start z 1.15 m** = the release hold's body height (1.156 flown). With it the approach is
  level, and the basket crosses the wall at about the release height. The clearance goes from
  27 to 44 mm: the full 50 mm Drop Margin less the few mm the payload rode lower on the way in
  (payload centre 0.887 m at Drop Start, 0.896 m at the hold).
- The skids' clearance is set by the hold height, not by Drop Start. 24-25 mm is with the
  SIMULATED gear (skids 0.3125 m below the body). The real T650 gear is ~65 mm shorter
  (0.248 m), so the same hold leaves ~90 mm on hardware. A Drop Margin below 0.05 would bring
  the simulated skids onto the tray at this hold.
- The first try of B (`m05_ds115`) aborted before the pick: the known simulation-only ~1.5 Hz
  roll mode grew from ~2 to 16 deg in the hover at Start (roll 1.44 Hz, before the plan was
  even made), the node's 15 deg watchdog handed over to SAFETY and the vehicle flipped.
  Unrelated to the change; the re-fly completed.

One flight per setting. Not flown on hardware. The MAC rig's generated pick-and-drop yaml still
carries 0.20 / 1.0 (regenerate it with `modular_adaptive_20260930/tools/make_modular_yaml.py
--profile pick_and_drop` before a comparison campaign); the Geo-L1 rig reads the whole-body
yaml's planner block and follows automatically.
