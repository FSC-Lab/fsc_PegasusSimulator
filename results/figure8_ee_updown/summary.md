# 3. figure-8 base + EE up/down

Scored window: **the whole task, in DIRECT** (4799 / 4799 samples).

Both runs were commanded from the same trajectory table (`utils_comparison/comparison_tasks.py`) and flew the same Isaac plant with the same allocator constants; only the control law and its gains differ.

| metric | whole-body | geometric+L1 | smaller |
|---|---|---|---|
| base_err_mm (mm, rms) | 85.598 | 126.598 | whole-body |
| base_err_mm (mm, p95) | 199.511 | 314.822 | whole-body |
| ee_err_mm (mm, rms) | 87.712 | 128.571 | whole-body |
| ee_err_mm (mm, p95) | 206.345 | 318.411 | whole-body |
| base_yaw_err_deg (deg, rms) | 0.126 | 0.110 | geometric+L1 |
| ee_heading_err_deg (deg, rms) | 0.567 | 0.876 | whole-body |
| joint_err_deg (deg, rms) | 1.181 | 0.768 | geometric+L1 |
| tilt_deg (deg, rms) | 0.854 | 1.101 | whole-body |

Ball figure shows the **ee** error cloud (sphere radius = RMS norm).

Per-axis RMS [mm] - the cloud is not isotropic, so this is the shape behind the single radius:

| axis | whole-body base | L1 base | whole-body EE | L1 EE |
|---|---|---|---|---|
| x | 74.01 | 107.92 | 75.46 | 108.61 |
| y | 43.00 | 66.18 | 44.30 | 68.51 |
| z | 1.05 | 0.97 | 6.08 | 6.45 |

Where the base is MOVING, most of the error is the loop running behind the reference rather than being off the path. Splitting it:

| | whole-body | geometric+L1 |
|---|---|---|
| fitted lag [s] | 1.04 | 1.57 |
| along-path rms [mm] | 115.0 | 163.1 |
| cross-track rms [mm] | 49.9 | 86.6 |
| rms with the lag removed [mm] | 26.2 | 35.7 |
