# Whole-body vs geometric + L1 adaptive - campaign summary

Same Isaac plant, same allocator constants, same commanded motion; the control law and its gains are the only difference. Method and commands: `docs/docs_aerial_manipulator/Comparison Command.md`.

| comparison | headline metric | whole-body | geometric+L1 | ratio |
|---|---|---|---|---|
| 1. hover + arm fold sweep | base position RMS [mm] | 5.26 | 10.98 | 2.09x |
| 2. base circle + yaw, EE pinned in the world | EE position RMS [mm] | 40.84 | 60.99 | 1.49x |
| 3. figure-8 base + EE up/down | EE position RMS [mm] | 87.71 | 128.57 | 1.47x |

Full per-task tables, per-axis breakdowns and figures are in each `results/<task>/` directory.

- `results/hover_arm_swing/` - 1. hover + arm fold sweep (scored window: task)
- `results/circle_ee_hold/` - 2. base circle + yaw, EE pinned in the world (scored window: pinned)
- `results/figure8_ee_updown/` - 3. figure-8 base + EE up/down (scored window: task)
