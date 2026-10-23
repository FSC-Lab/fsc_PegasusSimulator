# 1. hover + arm fold sweep

Scored window: **the whole task, in DIRECT** (3999 / 3999 samples).

Both runs were commanded from the same trajectory table (`utils_comparison/comparison_tasks.py`) and flew the same Isaac plant with the same allocator constants; only the control law and its gains differ.

| metric | whole-body | geometric+L1 | smaller |
|---|---|---|---|
| base_err_mm (mm, rms) | 5.255 | 10.982 | whole-body |
| base_err_mm (mm, p95) | 9.730 | 18.197 | whole-body |
| ee_err_mm (mm, rms) | 8.769 | 12.594 | whole-body |
| ee_err_mm (mm, p95) | 18.706 | 21.255 | whole-body |
| base_yaw_err_deg (deg, rms) | 0.050 | 0.052 | whole-body |
| ee_heading_err_deg (deg, rms) | 0.063 | 0.064 | whole-body |
| joint_err_deg (deg, rms) | 0.914 | 0.789 | geometric+L1 |
| tilt_deg (deg, rms) | 0.104 | 0.146 | whole-body |

Ball figure shows the **base** error cloud (sphere radius = RMS norm).

Per-axis RMS [mm] - the cloud is not isotropic, so this is the shape behind the single radius:

| axis | whole-body base | L1 base | whole-body EE | L1 EE |
|---|---|---|---|---|
| x | 4.49 | 10.01 | 4.78 | 10.13 |
| y | 2.65 | 4.48 | 2.65 | 4.49 |
| z | 0.68 | 0.51 | 6.86 | 5.98 |
