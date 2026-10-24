# 2026-10-21 T650-AM whole-body box pushing (240 g box, 0.10 m push at 21 mm/s)

| folder | bag | what happened |
|---|---|---|
| `box_pushing/` | `_170123` | The formal push. Every step flown; box pushed 107 mm (97 mm at rest). The exit was a q3-guard ABORT 0.78 s after the push ended; then Go To Land Start + Land. |
| `free_flight/` | `_164338` | Go To Start over a typed box point, 16 s hover above it, Go To Land Start, Land. No Ready / Push. |
| `stale/` | `_161745` | DIRECT, Plan refused (box point off the configured table), landed. |
| `stale/` | `_162604` | The same refused Plan, then 11 min on the ground. **No metadata.yaml** (the recorder was not closed); read the .db3 directly. |
| `stale/` | `_165311` | Plan refused (box at x 0.576, off the configured table), flown home from the drone ground station. |

Analysis: `docs/docs_aerial_manipulator/archive/wb_box_pushing_flight_20261021/README.md`.
