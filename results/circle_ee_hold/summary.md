# 2. base circle + yaw, EE pinned in the world

Scored window: **pinned** (4001 / 4001 samples).

Both runs were commanded from the same trajectory table (`utils_comparison/comparison_tasks.py`) and flew the same Isaac plant with the same allocator constants; only the control law and its gains differ.

| metric | whole-body | geometric+L1 | smaller |
|---|---|---|---|
| base_err_mm (mm, rms) | 33.273 | 29.732 | geometric+L1 |
| base_err_mm (mm, p95) | 70.580 | 62.949 | geometric+L1 |
| ee_err_mm (mm, rms) | 40.839 | 60.993 | whole-body |
| ee_err_mm (mm, p95) | 86.792 | 138.223 | whole-body |
| base_yaw_err_deg (deg, rms) | 4.037 | 8.706 | whole-body |
| ee_heading_err_deg (deg, rms) | 2.795 | 7.789 | whole-body |
| joint_err_deg (deg, rms) | 1.697 | 1.136 | geometric+L1 |
| tilt_deg (deg, rms) | 0.270 | 0.355 | whole-body |

Ball figure shows the **ee** error cloud (sphere radius = RMS norm).

Per-axis RMS [mm] - the cloud is not isotropic, so this is the shape behind the single radius:

| axis | whole-body base | L1 base | whole-body EE | L1 EE |
|---|---|---|---|---|
| x | 17.82 | 17.60 | 18.71 | 20.15 |
| y | 28.10 | 23.96 | 36.28 | 57.56 |
| z | 0.22 | 0.15 | 1.28 | 0.87 |
